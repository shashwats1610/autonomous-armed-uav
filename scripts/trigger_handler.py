"""Manual trigger control with ArduPilot safety interlocks."""

from __future__ import annotations

import logging
import time
from typing import Any, Callable, Optional

from config_loader import TriggerConfig
from mavlink_interface import MavlinkInterface

logger = logging.getLogger(__name__)

_GPS_ZERO_EPSILON = 1e-4


class TriggerHandler:
    """Handles payload trigger commands with safety checks and cooldown."""

    def __init__(
        self,
        config: TriggerConfig,
        mavlink: MavlinkInterface,
        on_trigger_event: Optional[Callable[[dict[str, Any]], None]] = None,
    ) -> None:
        self._config = config
        self._mavlink = mavlink
        self._on_trigger_event = on_trigger_event
        self._last_fire_mono = 0.0
        self._actuated = False

    def handle_command_long(self, msg: Any) -> None:
        """Process incoming COMMAND_LONG for trigger actuation."""
        cmd_id = int(msg.command)
        if cmd_id != self._config.command_id:
            return

        channel = int(msg.param1)
        if channel != self._config.servo_channel:
            return

        pwm_requested = int(msg.param2)
        fire = pwm_requested >= self._config.pwm_actuated_us

        if fire:
            self._attempt_fire()
        else:
            self._safe()

    def _position_valid(self, pos: Any, now_mono: float) -> bool:
        """Return True if GPS position is fresh and plausible."""
        if pos.timestamp <= 0:
            return False
        if now_mono - pos.timestamp > self._config.position_timeout_s:
            return False
        if abs(pos.lat) < _GPS_ZERO_EPSILON and abs(pos.lon) < _GPS_ZERO_EPSILON:
            return False
        return True

    def _attempt_fire(self) -> None:
        """Validate interlocks and actuate trigger servo."""
        now = time.monotonic()
        if now - self._last_fire_mono < self._config.cooldown_s:
            self._reject("Trigger cooldown active")
            return

        if not self._mavlink.is_armed():
            self._reject("Trigger rejected: vehicle disarmed")
            return

        mode = self._mavlink.flight_mode()
        if mode in self._config.blocked_modes:
            self._reject(f"Trigger rejected: mode {mode} blocked")
            return

        if mode not in self._config.allowed_modes:
            self._reject(f"Trigger rejected: mode {mode} not allowed")
            return

        pos = self._mavlink.get_position()
        if not self._position_valid(pos, now):
            self._reject("Trigger rejected: GPS position invalid or stale")
            return

        if pos.relative_alt_m < self._config.min_altitude_m:
            self._reject(
                f"Trigger rejected: altitude {pos.relative_alt_m:.1f}m "
                f"below minimum {self._config.min_altitude_m}m"
            )
            return

        self._mavlink.send_servo(
            self._config.servo_channel, self._config.pwm_actuated_us
        )
        self._last_fire_mono = now
        self._actuated = True
        self._mavlink.send_statustext("Trigger ACTUATED", severity=4)
        logger.info(
            "Trigger actuated at lat=%.6f lon=%.6f alt=%.1fm",
            pos.lat,
            pos.lon,
            pos.relative_alt_m,
        )
        if self._on_trigger_event:
            self._on_trigger_event(
                {
                    "lat": pos.lat,
                    "lon": pos.lon,
                    "alt_m": pos.relative_alt_m,
                    "timestamp": time.time(),
                    "mode": mode,
                }
            )

    def _safe(self) -> None:
        """Return trigger to safe position."""
        self._mavlink.send_servo(
            self._config.servo_channel, self._config.pwm_safe_us
        )
        self._actuated = False
        logger.info("Trigger returned to safe position")

    def _reject(self, message: str) -> None:
        """Log and notify ground station of rejected trigger."""
        logger.warning(message)
        self._mavlink.send_statustext(message[:50], severity=6)

    def ensure_safe_on_startup(self) -> None:
        """Set trigger servo to safe PWM at startup."""
        self._mavlink.send_servo(
            self._config.servo_channel, self._config.pwm_safe_us
        )
