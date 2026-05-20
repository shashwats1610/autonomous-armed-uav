"""Main orchestrator for the Raspberry Pi companion computer."""

from __future__ import annotations

import json
import logging
import os
import signal
import sys
import threading
import time
from logging.handlers import RotatingFileHandler
from pathlib import Path
from typing import Any, Optional

from config_loader import CompanionConfig, load_config, resolve_log_dir
from gimbal_controller import GimbalController
from mavlink_interface import MavlinkInterface
from telemetry_logger import TelemetryLogger
from trigger_handler import TriggerHandler
from video_stream import write_osd_text_file

logger = logging.getLogger(__name__)

_BATTERY_UNKNOWN = -1


def _battery_valid(percent: int) -> bool:
    return 0 <= percent <= 100


class CompanionOrchestrator:
    """Coordinates MAVLink, gimbal, trigger, logging, and OSD IPC."""

    def __init__(self, config: CompanionConfig) -> None:
        self._config = config
        self._stop = threading.Event()
        self._telemetry: Optional[TelemetryLogger] = None
        self._gimbal: Optional[GimbalController] = None
        self._mavlink: Optional[MavlinkInterface] = None
        self._trigger: Optional[TriggerHandler] = None
        self._runtime = Path(config.system.runtime_dir)
        self._osd_path = Path(config.video.osd_state_file)
        self._osd_text_path = Path(config.video.osd_text_file)
        self._thermal_path = Path(config.video.thermal_alert_file)
        self._watchdog_path = Path(config.system.watchdog.main_heartbeat_file)
        self._gimbal_log_interval = 1.0 / max(config.gimbal.gimbal_log_hz, 0.1)
        self._last_gimbal_log_mono = 0.0
        self._last_thermal_warn_mono = 0.0
        self._logging_configured = False

    def setup_logging(self) -> None:
        """Configure rotating file and console logging (once)."""
        if self._logging_configured:
            return
        log_dir = resolve_log_dir(self._config)
        log_dir.mkdir(parents=True, exist_ok=True)
        log_file = log_dir / "companion_main.log"
        fmt = logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
        )
        root = logging.getLogger()
        root.setLevel(logging.INFO)
        if not root.handlers:
            handler = RotatingFileHandler(
                log_file, maxBytes=5_000_000, backupCount=3
            )
            handler.setFormatter(fmt)
            root.addHandler(handler)
            console = logging.StreamHandler()
            console.setFormatter(fmt)
            root.addHandler(console)
        self._logging_configured = True

    def run(self) -> int:
        """Start all subsystems and block until shutdown."""
        self.setup_logging()
        self._runtime.mkdir(parents=True, exist_ok=True)
        logger.info("Companion computer starting (root=%s)", self._config.root)

        self._telemetry = TelemetryLogger(self._config)
        self._gimbal = GimbalController(self._config.gimbal)

        def on_command(msg: Any) -> None:
            if self._trigger:
                self._trigger.handle_command_long(msg)

        def on_message(msg: Any) -> None:
            if self._telemetry:
                self._telemetry.log_mavlink_message(msg)

        self._mavlink = MavlinkInterface(
            self._config.mavlink,
            on_command_long=on_command,
            on_message=on_message,
        )
        self._trigger = TriggerHandler(
            self._config.system.trigger,
            self._mavlink,
            on_trigger_event=self._telemetry.log_trigger_event,
        )

        self._mavlink.start()
        if not self._mavlink.wait_for_connection(
            self._config.mavlink.connection_timeout_s
        ):
            logger.warning(
                "No MAVLink heartbeat within timeout; continuing with retry"
            )

        self._trigger.ensure_safe_on_startup()
        home = self._gimbal.home()
        for channel, pwm in home.items():
            self._mavlink.send_servo(channel, pwm)
        self._mavlink.flush_outbound(timeout_s=1.0)

        threads = [
            threading.Thread(
                target=self._gimbal_loop, name="gimbal-loop", daemon=True
            ),
            threading.Thread(
                target=self._osd_loop, name="osd-writer", daemon=True
            ),
            threading.Thread(
                target=self._health_loop, name="health-logger", daemon=True
            ),
            threading.Thread(
                target=self._thermal_loop, name="thermal-monitor", daemon=True
            ),
            threading.Thread(
                target=self._watchdog_loop, name="watchdog", daemon=True
            ),
        ]
        for thread in threads:
            thread.start()

        logger.info("Companion computer running")
        while not self._stop.is_set():
            time.sleep(0.5)

        self.shutdown()
        return 0

    def shutdown(self) -> None:
        """Graceful shutdown: safe servos, close logs."""
        logger.info("Shutting down companion computer")
        if self._gimbal and self._mavlink:
            for channel, pwm in self._gimbal.emergency_disable().items():
                self._mavlink.send_servo(channel, pwm)
        if self._trigger:
            self._trigger.ensure_safe_on_startup()
        if self._mavlink:
            self._mavlink.flush_outbound(timeout_s=0.5)
            time.sleep(0.15)
            self._mavlink.stop()
        if self._telemetry:
            self._telemetry.flush_and_close()
        try:
            os.sync()
        except AttributeError:
            pass
        logger.info("Shutdown complete")

    def request_stop(self) -> None:
        """Signal main loop to exit."""
        self._stop.set()

    def _gimbal_loop(self) -> None:
        interval = 1.0 / self._config.gimbal.update_hz
        prev = time.monotonic()
        channels = self._config.gimbal.servo_channels
        while not self._stop.is_set():
            now = time.monotonic()
            dt = min(now - prev, 0.5)
            prev = now
            if not self._mavlink or not self._gimbal:
                time.sleep(interval)
                continue
            attitude = self._mavlink.get_attitude()
            if attitude.timestamp <= 0:
                pwm = self._gimbal.hold_last_or_home()
            else:
                pwm = self._gimbal.compute(attitude, dt, now_mono=now)
            error = self._gimbal.get_stabilization_error()
            for channel, value in pwm.items():
                self._mavlink.send_servo(channel, value)
            if self._telemetry and now - self._last_gimbal_log_mono >= self._gimbal_log_interval:
                self._telemetry.log_gimbal(pwm, error, channels)
                self._last_gimbal_log_mono = now
            self._touch_watchdog()
            time.sleep(max(0.0, interval - (time.monotonic() - now)))

    def _osd_loop(self) -> None:
        interval = 1.0 / self._config.system.osd_update_hz
        while not self._stop.is_set():
            if self._mavlink:
                att = self._mavlink.get_attitude()
                pos = self._mavlink.get_position()
                sys_st = self._mavlink.get_sys_status()
                hb = self._mavlink.get_heartbeat()
                osd = {
                    "lat": pos.lat,
                    "lon": pos.lon,
                    "alt_m": pos.relative_alt_m,
                    "battery_pct": sys_st.battery_remaining,
                    "mode": self._mavlink.flight_mode(),
                    "armed": hb.armed,
                    "roll": att.roll,
                    "pitch": att.pitch,
                    "timestamp": time.time(),
                }
                self._write_json(self._osd_path, osd)
                if self._config.video.osd_enabled:
                    write_osd_text_file(
                        str(self._osd_path), str(self._osd_text_path)
                    )
            time.sleep(interval)

    def _health_loop(self) -> None:
        interval = self._config.system.health_interval_s
        thermal_path = self._config.system.thermal_sysfs
        start_mono = time.monotonic()
        while not self._stop.is_set():
            try:
                with open(thermal_path, "r", encoding="utf-8") as handle:
                    temp_c = int(handle.read().strip()) / 1000.0
            except (OSError, ValueError):
                temp_c = 0.0
            load_1m = os.getloadavg()[0] if hasattr(os, "getloadavg") else 0.0
            uptime_s = time.monotonic() - start_mono
            battery = _BATTERY_UNKNOWN
            if self._mavlink:
                battery = self._mavlink.get_sys_status().battery_remaining
                if (
                    self._config.system.shutdown_on_low_battery
                    and _battery_valid(battery)
                    and battery <= self._config.system.low_battery_percent
                ):
                    logger.critical(
                        "Low battery %d%%, initiating shutdown", battery
                    )
                    self.request_stop()
            if self._telemetry:
                self._telemetry.log_health(temp_c, load_1m, uptime_s, battery)
            time.sleep(interval)

    def _thermal_loop(self) -> None:
        interval = self._config.system.thermal_statustext_interval_s
        while not self._stop.is_set():
            if self._thermal_path.is_file() and self._mavlink:
                try:
                    data = json.loads(
                        self._thermal_path.read_text(encoding="utf-8")
                    )
                    if data.get("warning") and self._mavlink.is_connected():
                        now = time.monotonic()
                        if now - self._last_thermal_warn_mono >= interval:
                            self._mavlink.send_statustext(
                                f"Pi temp {data.get('temp_c', 0):.0f}C HIGH",
                                severity=4,
                            )
                            self._last_thermal_warn_mono = now
                except (OSError, json.JSONDecodeError):
                    pass
            time.sleep(5.0)

    def _watchdog_loop(self) -> None:
        while not self._stop.is_set():
            self._touch_watchdog()
            time.sleep(2.0)

    def _touch_watchdog(self) -> None:
        try:
            self._watchdog_path.parent.mkdir(parents=True, exist_ok=True)
            self._watchdog_path.touch()
        except OSError as exc:
            logger.debug("Watchdog touch failed: %s", exc)

    def _write_json(self, path: Path, data: dict[str, Any]) -> None:
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("w", encoding="utf-8") as handle:
                json.dump(data, handle)
                handle.flush()
        except OSError as exc:
            logger.warning("Failed to write %s: %s", path, exc)


def main() -> int:
    try:
        config = load_config()
    except Exception as exc:
        print(f"Config error: {exc}", file=sys.stderr)
        return 1

    orchestrator = CompanionOrchestrator(config)

    def handle_signal(signum: int, _frame: object) -> None:
        logger.info("Signal %s received", signum)
        orchestrator.request_stop()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    return orchestrator.run()


if __name__ == "__main__":
    sys.exit(main())
