"""System health monitoring, thermal alerts, and process watchdog."""

from __future__ import annotations

import json
import logging
import os
import subprocess
import sys
import time
from pathlib import Path

from config_loader import CompanionConfig, load_config, resolve_log_dir

logger = logging.getLogger(__name__)


def read_cpu_temp_c(sysfs_path: str) -> float:
    """Read CPU temperature in Celsius from thermal sysfs."""
    try:
        with open(sysfs_path, "r", encoding="utf-8") as handle:
            millideg = int(handle.read().strip())
        return millideg / 1000.0
    except (OSError, ValueError) as exc:
        logger.warning("Failed to read CPU temperature: %s", exc)
        return 0.0


def read_load_1m() -> float:
    """Return 1-minute load average."""
    try:
        return os.getloadavg()[0]
    except OSError:
        return 0.0


class SystemMonitor:
    """Monitors thermal state, writes alert files, and watchdogs services."""

    def __init__(self, config: CompanionConfig) -> None:
        self._config = config
        self._system = config.system
        self._video = config.video
        self._watchdog = config.system.watchdog
        self._runtime = Path(self._system.runtime_dir)
        self._runtime.mkdir(parents=True, exist_ok=True)
        log_dir = resolve_log_dir(config)
        log_dir.mkdir(parents=True, exist_ok=True)
        self._events_log = log_dir / "system_events.log"
        self._stop = False
        self._last_main_restart = 0.0
        self._last_video_restart = 0.0

    def run(self) -> None:
        """Main monitoring loop."""
        self._log_event("system_monitor started")
        interval = self._system.health_interval_s
        while not self._stop:
            temp_c = read_cpu_temp_c(self._system.thermal_sysfs)
            self._update_thermal_alert(temp_c)
            self._check_watchdogs()
            time.sleep(interval)

    def stop(self) -> None:
        """Stop monitor loop."""
        self._stop = True
        self._log_event("system_monitor stopped")

    def _update_thermal_alert(self, temp_c: float) -> None:
        """Write thermal alert JSON for video bitrate and main STATUSTEXT."""
        level = 0
        if temp_c >= self._system.thermal_critical_c:
            level = min(3, len(self._video.thermal_bitrate_steps) - 1)
            self._log_event(f"thermal CRITICAL {temp_c:.1f}C")
        elif temp_c >= self._system.thermal_warning_c:
            level = min(2, len(self._video.thermal_bitrate_steps) - 1)
            self._log_event(f"thermal WARNING {temp_c:.1f}C")

        alert = {
            "temp_c": temp_c,
            "bitrate_level": level,
            "warning": temp_c >= self._system.thermal_warning_c,
            "timestamp": time.time(),
        }
        alert_path = Path(self._video.thermal_alert_file)
        try:
            with alert_path.open("w", encoding="utf-8") as handle:
                json.dump(alert, handle)
                handle.flush()
                os.fsync(handle.fileno())
        except OSError as exc:
            logger.warning("Failed to write thermal alert: %s", exc)

    def _check_watchdogs(self) -> None:
        """Restart services if heartbeat files are stale."""
        now = time.time()
        cooldown = self._watchdog.restart_cooldown_s

        main_hb = Path(self._watchdog.main_heartbeat_file)
        if main_hb.is_file():
            try:
                stale_s = now - main_hb.stat().st_mtime
            except OSError:
                stale_s = 0.0
            if stale_s > self._watchdog.main_stale_s:
                if now - self._last_main_restart >= cooldown:
                    self._log_event(
                        f"main heartbeat stale {stale_s:.0f}s, restarting companion-main"
                    )
                    self._restart_service("companion-main.service")
                    self._last_main_restart = now

        video_hb = Path(self._watchdog.video_heartbeat_file)
        if video_hb.is_file():
            try:
                stale_s = now - video_hb.stat().st_mtime
            except OSError:
                stale_s = 0.0
            if stale_s > self._watchdog.video_stale_s:
                if now - self._last_video_restart >= cooldown:
                    self._log_event(
                        f"video heartbeat stale {stale_s:.0f}s, restarting video-stream"
                    )
                    self._restart_service("video-stream.service")
                    self._last_video_restart = now

    def _restart_service(self, unit: str) -> None:
        try:
            subprocess.run(
                ["systemctl", "restart", unit],
                check=False,
                timeout=30,
            )
        except (subprocess.SubprocessError, OSError) as exc:
            logger.error("Failed to restart %s: %s", unit, exc)

    def _log_event(self, message: str) -> None:
        line = f"{time.time():.3f} {message}\n"
        logger.info(message)
        try:
            with self._events_log.open("a", encoding="utf-8") as handle:
                handle.write(line)
                handle.flush()
        except OSError as exc:
            logger.warning("Failed to write system event: %s", exc)


def setup_logging() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main() -> int:
    setup_logging()
    try:
        config = load_config()
    except Exception as exc:
        logger.error("Failed to load config: %s", exc)
        return 1

    monitor = SystemMonitor(config)

    def handle_signal(signum: int, _frame: object) -> None:
        logger.info("Received signal %s", signum)
        monitor.stop()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        monitor.run()
    except KeyboardInterrupt:
        monitor.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
