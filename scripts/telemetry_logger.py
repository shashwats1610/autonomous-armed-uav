"""Telemetry and flight data logging (.tlog, CSV, rotation)."""

from __future__ import annotations

import csv
import gzip
import logging
import os
import shutil
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

from config_loader import CompanionConfig, resolve_log_dir

logger = logging.getLogger(__name__)


class TelemetryLogger:
    """Thread-safe logger for MAVLink, gimbal, trigger, and health data."""

    def __init__(self, config: CompanionConfig) -> None:
        self._config = config
        self._log_dir = resolve_log_dir(config)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self._lock = threading.Lock()
        self._tlog_stamp = stamp
        self._tlog_path = self._log_dir / f"flight_{stamp}.tlog"
        self._gimbal_csv = self._log_dir / f"gimbal_{stamp}.csv"
        self._trigger_csv = self._log_dir / f"trigger_{stamp}.csv"
        self._health_csv = self._log_dir / f"health_{stamp}.csv"
        self._tlog_file: Optional[Any] = None
        self._tlog_msg_count = 0
        self._init_csv_files()
        self._open_tlog()

    def _init_csv_files(self) -> None:
        """Create CSV files with headers."""
        with self._gimbal_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "timestamp",
                    "roll_deg",
                    "pitch_deg",
                    "roll_pwm",
                    "pitch_pwm",
                    "yaw_pwm",
                    "roll_error_deg",
                    "pitch_error_deg",
                ]
            )
        with self._trigger_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["timestamp", "lat", "lon", "alt_m", "mode"])
        with self._health_csv.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                ["timestamp", "cpu_temp_c", "load_1m", "uptime_s", "battery_pct"]
            )

    def _open_tlog(self) -> None:
        """Open binary MAVLink log file."""
        self._tlog_file = open(self._tlog_path, "wb")  # noqa: SIM115
        self._tlog_msg_count = 0

    def _rotate_tlog_if_needed(self) -> None:
        max_bytes = self._config.system.log_rotation_max_bytes
        if not self._tlog_file:
            return
        try:
            if self._tlog_path.stat().st_size < max_bytes:
                return
        except OSError:
            return
        self._tlog_file.flush()
        self._tlog_file.close()
        archive = self._tlog_path.with_suffix(self._tlog_path.suffix + ".gz")
        with self._tlog_path.open("rb") as src:
            with gzip.open(archive, "wb") as dst:
                shutil.copyfileobj(src, dst)
        self._tlog_path.unlink(missing_ok=True)
        stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
        self._tlog_stamp = stamp
        self._tlog_path = self._log_dir / f"flight_{stamp}.tlog"
        self._open_tlog()
        logger.info("Rotated tlog to %s", self._tlog_path)

    def log_mavlink_message(self, msg: Any) -> None:
        """Append raw MAVLink message bytes to .tlog."""
        if not self._tlog_file:
            return
        try:
            buf = msg.get_msgbuf()
        except AttributeError:
            return
        interval = self._config.system.tlog_flush_interval
        with self._lock:
            self._tlog_file.write(buf)
            self._tlog_msg_count += 1
            if self._tlog_msg_count % interval == 0:
                self._tlog_file.flush()
            self._rotate_tlog_if_needed()

    def log_gimbal(
        self,
        pwm: dict[int, int],
        error: dict[str, float],
        channels: dict[str, int],
    ) -> None:
        """Log gimbal servo positions and stabilization error."""
        row = [
            time.time(),
            error.get("roll_deg", 0.0),
            error.get("pitch_deg", 0.0),
            pwm.get(channels["roll"], 0),
            pwm.get(channels["pitch"], 0),
            pwm.get(channels["yaw"], 0),
            error.get("roll_deg", 0.0),
            error.get("pitch_deg", 0.0),
        ]
        self._append_csv(self._gimbal_csv, row)

    def log_trigger_event(self, event: dict[str, Any]) -> None:
        """Log trigger actuation with GPS position."""
        row = [
            event.get("timestamp", time.time()),
            event.get("lat", 0.0),
            event.get("lon", 0.0),
            event.get("alt_m", 0.0),
            event.get("mode", ""),
        ]
        self._append_csv(self._trigger_csv, row, critical=True)

    def log_health(
        self, cpu_temp_c: float, load_1m: float, uptime_s: float, battery_pct: int
    ) -> None:
        """Log system health metrics."""
        row = [time.time(), cpu_temp_c, load_1m, uptime_s, battery_pct]
        self._append_csv(self._health_csv, row)

    def _append_csv(
        self, path: Path, row: list[Any], critical: bool = False
    ) -> None:
        with self._lock:
            with path.open("a", newline="", encoding="utf-8") as handle:
                csv.writer(handle).writerow(row)
                handle.flush()
                if critical:
                    os.fsync(handle.fileno())
            self._maybe_rotate_csv(path)

    def _maybe_rotate_csv(self, path: Path) -> None:
        max_bytes = self._config.system.log_rotation_max_bytes
        try:
            if path.stat().st_size < max_bytes:
                return
        except OSError:
            return
        archive = path.with_suffix(path.suffix + ".gz")
        with path.open("rb") as src:
            with gzip.open(archive, "wb") as dst:
                shutil.copyfileobj(src, dst)
        path.unlink()
        logger.info("Rotated log %s -> %s", path, archive)

    def flush_and_close(self) -> None:
        """Flush all logs and close files."""
        with self._lock:
            if self._tlog_file:
                self._tlog_file.flush()
                os.fsync(self._tlog_file.fileno())
                self._tlog_file.close()
                self._tlog_file = None
        try:
            os.sync()
        except AttributeError:
            pass
        logger.info("Telemetry logger closed")
