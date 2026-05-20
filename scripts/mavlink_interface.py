"""MAVLink serial interface for Pixhawk communication."""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional

from config_loader import MavlinkConfig
from pymavlink import mavutil

logger = logging.getLogger(__name__)

MAV_CMD_DO_SET_SERVO = 183


@dataclass
class AttitudeState:
    """Latest attitude from ATTITUDE message."""

    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    timestamp: float = 0.0


@dataclass
class PositionState:
    """Latest position from GLOBAL_POSITION_INT."""

    lat: float = 0.0
    lon: float = 0.0
    alt_m: float = 0.0
    relative_alt_m: float = 0.0
    timestamp: float = 0.0


@dataclass
class SysStatusState:
    """Latest SYS_STATUS snapshot."""

    battery_remaining: int = -1
    voltage_battery: float = 0.0
    timestamp: float = 0.0


@dataclass
class HeartbeatState:
    """Latest HEARTBEAT from target autopilot."""

    armed: bool = False
    custom_mode: int = 0
    base_mode: int = 0
    mavlink_version: int = 0
    timestamp: float = 0.0


class MavlinkInterface:
    """Thread-safe MAVLink connection with auto-reconnect and command queue."""

    def __init__(
        self,
        config: MavlinkConfig,
        on_command_long: Optional[Callable[[Any], None]] = None,
        on_message: Optional[Callable[[Any], None]] = None,
    ) -> None:
        self._config = config
        self._on_command_long = on_command_long
        self._on_message = on_message
        self._connection: Optional[Any] = None
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._writer_thread: Optional[threading.Thread] = None
        self._outbound: queue.Queue[tuple[str, tuple[Any, ...]]] = queue.Queue(
            maxsize=config.outbound_queue_depth
        )
        self._attitude = AttitudeState()
        self._position = PositionState()
        self._sys_status = SysStatusState()
        self._heartbeat = HeartbeatState()
        self._last_heartbeat_mono = 0.0
        self._connected = False
        self._servo_min_interval = 1.0 / max(config.servo_max_rate_hz, 1.0)
        self._last_servo_send = 0.0
        self._critical_ids = set(config.critical_message_ids)
        self._pending_servos: Dict[int, int] = {}

    def start(self) -> None:
        """Start reader and writer threads."""
        self._stop.clear()
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="mavlink-reader", daemon=True
        )
        self._writer_thread = threading.Thread(
            target=self._writer_loop, name="mavlink-writer", daemon=True
        )
        self._reader_thread.start()
        self._writer_thread.start()
        logger.info("MAVLink interface threads started")

    def stop(self) -> None:
        """Stop threads and close connection."""
        self._stop.set()
        if self._reader_thread:
            self._reader_thread.join(timeout=5.0)
        if self._writer_thread:
            self._writer_thread.join(timeout=5.0)
        with self._lock:
            if self._connection:
                try:
                    self._connection.close()
                except OSError as exc:
                    logger.warning("Error closing MAVLink connection: %s", exc)
                self._connection = None
        self._connected = False
        logger.info("MAVLink interface stopped")

    def flush_outbound(self, timeout_s: float = 0.5) -> None:
        """Drain outbound queue and send coalesced servo commands."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                cmd, args = self._outbound.get_nowait()
            except queue.Empty:
                break
            if cmd == "servo":
                channel, pwm_us = args
                self._pending_servos[channel] = pwm_us
            elif cmd == "statustext" and self.is_connected():
                self._send_statustext_now(*args)
        self._flush_pending_servos()

    def wait_for_connection(self, timeout_s: float) -> bool:
        """Block until connected or timeout."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self.is_connected():
                return True
            time.sleep(0.25)
        return self.is_connected()

    def is_connected(self) -> bool:
        """Return True if heartbeat received within timeout."""
        with self._lock:
            if not self._connected:
                return False
            if self._last_heartbeat_mono <= 0:
                return False
            return (
                time.monotonic() - self._last_heartbeat_mono
                < self._config.heartbeat_timeout_s
            )

    def get_attitude(self) -> AttitudeState:
        """Return copy of latest attitude."""
        with self._lock:
            return AttitudeState(
                roll=self._attitude.roll,
                pitch=self._attitude.pitch,
                yaw=self._attitude.yaw,
                timestamp=self._attitude.timestamp,
            )

    def get_position(self) -> PositionState:
        """Return copy of latest position."""
        with self._lock:
            return PositionState(
                lat=self._position.lat,
                lon=self._position.lon,
                alt_m=self._position.alt_m,
                relative_alt_m=self._position.relative_alt_m,
                timestamp=self._position.timestamp,
            )

    def get_sys_status(self) -> SysStatusState:
        """Return copy of latest system status."""
        with self._lock:
            return SysStatusState(
                battery_remaining=self._sys_status.battery_remaining,
                voltage_battery=self._sys_status.voltage_battery,
                timestamp=self._sys_status.timestamp,
            )

    def get_heartbeat(self) -> HeartbeatState:
        """Return copy of latest target heartbeat."""
        with self._lock:
            return HeartbeatState(
                armed=self._heartbeat.armed,
                custom_mode=self._heartbeat.custom_mode,
                base_mode=self._heartbeat.base_mode,
                mavlink_version=self._heartbeat.mavlink_version,
                timestamp=self._heartbeat.timestamp,
            )

    def is_armed(self) -> bool:
        """Return armed state from last heartbeat."""
        return self.get_heartbeat().armed

    def flight_mode(self) -> str:
        """Return ArduPilot flight mode name from custom_mode."""
        hb = self.get_heartbeat()
        return self._config.ardupilot_copter_modes.get(
            hb.custom_mode, f"MODE_{hb.custom_mode}"
        )

    def send_servo(self, channel: int, pwm_us: int) -> bool:
        """Queue servo command (coalesced in writer thread)."""
        try:
            self._outbound.put_nowait(("servo", (channel, pwm_us)))
            return True
        except queue.Full:
            logger.warning("MAVLink outbound queue full, dropping servo command")
            return False

    def send_statustext(self, text: str, severity: int = 4) -> bool:
        """Queue STATUSTEXT message to ground station."""
        try:
            self._outbound.put_nowait(("statustext", (text, severity)))
            return True
        except queue.Full:
            logger.warning("MAVLink outbound queue full, dropping statustext")
            return False

    def _reader_loop(self) -> None:
        backoff = self._config.reconnect_initial_s
        while not self._stop.is_set():
            if not self._ensure_connection():
                time.sleep(min(backoff, self._config.reconnect_max_s))
                backoff = min(backoff * 2, self._config.reconnect_max_s)
                continue
            backoff = self._config.reconnect_initial_s
            try:
                self._read_messages()
            except (OSError, ConnectionError, AttributeError) as exc:
                logger.error("MAVLink read error: %s", exc)
                self._mark_disconnected()
                time.sleep(backoff)
                backoff = min(backoff * 2, self._config.reconnect_max_s)

    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            batch_deadline = time.monotonic() + self._servo_min_interval
            while time.monotonic() < batch_deadline and not self._stop.is_set():
                try:
                    cmd, args = self._outbound.get(timeout=0.05)
                except queue.Empty:
                    continue
                if cmd == "servo":
                    channel, pwm_us = args
                    self._pending_servos[channel] = pwm_us
                elif cmd == "statustext" and self.is_connected():
                    try:
                        self._send_statustext_now(*args)
                    except (OSError, AttributeError, ValueError) as exc:
                        logger.error("MAVLink statustext error: %s", exc)
            if self.is_connected() and self._pending_servos:
                try:
                    self._flush_pending_servos()
                except (OSError, AttributeError, ValueError) as exc:
                    logger.error("MAVLink servo flush error: %s", exc)

    def _flush_pending_servos(self) -> None:
        if not self._pending_servos:
            return
        now = time.monotonic()
        if now - self._last_servo_send < self._servo_min_interval:
            return
        pending = dict(self._pending_servos)
        self._pending_servos.clear()
        for channel, pwm_us in pending.items():
            self._send_servo_now(channel, pwm_us)
        self._last_servo_send = now

    def _ensure_connection(self) -> bool:
        if self.is_connected():
            return True
        with self._lock:
            if self._connection:
                try:
                    self._connection.close()
                except OSError:
                    pass
                self._connection = None
        for port in self._config.serial_ports:
            if self._stop.is_set():
                return False
            conn_str = f"{port},{self._config.baud_rate}"
            logger.info("Attempting MAVLink connection on %s", conn_str)
            try:
                master = mavutil.mavlink_connection(
                    conn_str,
                    source_system=self._config.source_system,
                    source_component=self._config.source_component,
                    autoreconnect=False,
                )
                if not self._wait_target_heartbeat(master, timeout=5.0):
                    logger.warning("No heartbeat from target system on %s", port)
                    master.close()
                    continue
                with self._lock:
                    self._connection = master
                    self._connected = True
                    self._last_heartbeat_mono = time.monotonic()
                logger.info("MAVLink connected on %s", port)
                return True
            except (OSError, ConnectionError, TimeoutError) as exc:
                logger.warning("Failed to connect on %s: %s", port, exc)
        self._mark_disconnected()
        return False

    def _wait_target_heartbeat(self, master: Any, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not self._stop.is_set():
            msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if msg is None:
                continue
            if msg.get_srcSystem() == self._config.target_system:
                return True
        return False

    def _mark_disconnected(self) -> None:
        with self._lock:
            self._connected = False
            if self._connection:
                try:
                    self._connection.close()
                except OSError:
                    pass
                self._connection = None

    def _from_target(self, msg: Any) -> bool:
        return msg.get_srcSystem() == self._config.target_system

    def _read_messages(self) -> None:
        master = self._connection
        if not master:
            return
        while not self._stop.is_set() and self._connection:
            if (
                self._last_heartbeat_mono > 0
                and time.monotonic() - self._last_heartbeat_mono
                > self._config.heartbeat_timeout_s
            ):
                logger.warning("Heartbeat timeout, reconnecting")
                self._mark_disconnected()
                return
            msg = master.recv_match(blocking=True, timeout=1.0)
            if msg is None:
                continue
            msg_id = msg.get_msgId()
            if self._on_message:
                self._on_message(msg)
            if msg_id not in self._critical_ids:
                continue
            self._handle_message(msg)

    def _handle_message(self, msg: Any) -> None:
        msg_type = msg.get_type()
        now = time.monotonic()
        if msg_type == "HEARTBEAT":
            if not self._from_target(msg):
                return
            armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            with self._lock:
                self._heartbeat.armed = armed
                self._heartbeat.custom_mode = msg.custom_mode
                self._heartbeat.base_mode = msg.base_mode
                self._heartbeat.mavlink_version = getattr(msg, "mavlink_version", 0)
                self._heartbeat.timestamp = now
                self._last_heartbeat_mono = now
                self._connected = True
        elif msg_type == "ATTITUDE":
            if not self._from_target(msg):
                return
            with self._lock:
                self._attitude.roll = msg.roll
                self._attitude.pitch = msg.pitch
                self._attitude.yaw = msg.yaw
                self._attitude.timestamp = now
        elif msg_type == "GLOBAL_POSITION_INT":
            if not self._from_target(msg):
                return
            with self._lock:
                self._position.lat = msg.lat / 1e7
                self._position.lon = msg.lon / 1e7
                self._position.alt_m = msg.alt / 1000.0
                self._position.relative_alt_m = msg.relative_alt / 1000.0
                self._position.timestamp = now
        elif msg_type == "SYS_STATUS":
            if not self._from_target(msg):
                return
            with self._lock:
                self._sys_status.battery_remaining = msg.battery_remaining
                self._sys_status.voltage_battery = msg.voltage_battery / 1000.0
                self._sys_status.timestamp = now
        elif msg_type == "COMMAND_LONG":
            if self._on_command_long and self._should_handle_command(msg):
                self._on_command_long(msg)

    def _should_handle_command(self, msg: Any) -> bool:
        """Return True if COMMAND_LONG targets autopilot or broadcast."""
        target = int(getattr(msg, "target_system", 0))
        return target in (0, self._config.target_system)

    def _send_servo_now(self, channel: int, pwm_us: int) -> None:
        master = self._connection
        if not master:
            return
        master.mav.command_long_send(
            self._config.target_system,
            self._config.target_component,
            MAV_CMD_DO_SET_SERVO,
            0,
            float(channel),
            float(pwm_us),
            0,
            0,
            0,
            0,
            0,
        )

    def _send_statustext_now(self, text: str, severity: int) -> None:
        master = self._connection
        if not master:
            return
        payload = text.encode("utf-8")[:50]
        master.mav.statustext_send(severity, payload)
