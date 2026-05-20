"""Three-axis gimbal stabilization with PID and attitude compensation."""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional

from config_loader import GimbalConfig, PidGains
from mavlink_interface import AttitudeState

logger = logging.getLogger(__name__)


@dataclass
class LowPassFilter:
    """First-order low-pass filter for attitude smoothing."""

    cutoff_hz: float
    value: float = 0.0
    initialized: bool = False

    def update(self, raw: float, dt: float) -> float:
        """Update filter with new sample."""
        if dt <= 0:
            return self.value
        rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
        alpha = dt / (dt + rc)
        if not self.initialized:
            self.value = raw
            self.initialized = True
        else:
            self.value += alpha * (raw - self.value)
        return self.value


@dataclass
class PidController:
    """PID controller with anti-windup."""

    gains: PidGains
    output_limit: float
    integral: float = 0.0
    prev_error: float = 0.0

    def reset(self) -> None:
        """Reset integrator and derivative state."""
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error: float, dt: float) -> float:
        """Compute PID output for given error."""
        if dt <= 0:
            return 0.0
        p_term = self.gains.kp * error
        self.integral += error * dt
        i_term = self.gains.ki * self.integral
        d_term = self.gains.kd * (error - self.prev_error) / dt
        self.prev_error = error
        output = p_term + i_term + d_term
        if abs(output) > self.output_limit:
            output = math.copysign(self.output_limit, output)
            self.integral -= error * dt
        return output


class GimbalController:
    """Computes gimbal servo PWM from drone attitude and PID stabilization."""

    def __init__(self, config: GimbalConfig) -> None:
        self._config = config
        self._roll_lpf = LowPassFilter(config.attitude_lpf_cutoff_hz)
        self._pitch_lpf = LowPassFilter(config.attitude_lpf_cutoff_hz)
        self._roll_pid = PidController(
            config.pid["roll"], config.pid_output_limit_deg
        )
        self._pitch_pid = PidController(
            config.pid["pitch"], config.pid_output_limit_deg
        )
        self._enabled = True
        self._last_attitude_mono = 0.0
        self._last_roll_deg = 0.0
        self._last_pitch_deg = 0.0
        self._manual_yaw_deg = config.manual_yaw_deg
        self._last_valid_pwm: Optional[Dict[int, int]] = None

    def set_manual_yaw(self, yaw_deg: float) -> None:
        """Set operator pan angle in degrees."""
        self._manual_yaw_deg = yaw_deg

    def home(self) -> Dict[int, int]:
        """Return home PWM for all gimbal axes."""
        ch = self._config.servo_channels
        home = self._config.home_pwm
        return {
            ch["roll"]: home["roll"],
            ch["pitch"]: home["pitch"],
            ch["yaw"]: home["yaw"],
        }

    def emergency_disable(self) -> Dict[int, int]:
        """Disable stabilization and return safe home positions."""
        self._enabled = False
        self._roll_pid.reset()
        self._pitch_pid.reset()
        logger.warning("Gimbal stabilization disabled (emergency)")
        return self.home()

    def enable(self) -> None:
        """Re-enable stabilization."""
        self._enabled = True
        self._roll_pid.reset()
        self._pitch_pid.reset()

    def compute(
        self, attitude: AttitudeState, dt: float, now_mono: Optional[float] = None
    ) -> Dict[int, int]:
        """
        Compute servo PWM commands from attitude.

        Roll/pitch are compensated; yaw uses manual setpoint only.
        """
        now = now_mono if now_mono is not None else time.monotonic()
        home = self.home()

        if attitude.timestamp <= 0:
            return home

        if attitude.timestamp > self._last_attitude_mono:
            self._last_attitude_mono = attitude.timestamp

        stale = (
            self._last_attitude_mono > 0
            and now - self._last_attitude_mono > self._config.attitude_timeout_s
        )
        if stale:
            logger.warning("Attitude data stale, returning home")
            self._enabled = False
            return home

        if not self._enabled:
            return home

        roll_rad = self._roll_lpf.update(attitude.roll, dt)
        pitch_rad = self._pitch_lpf.update(attitude.pitch, dt)
        roll_deg = math.degrees(roll_rad)
        pitch_deg = math.degrees(pitch_rad)

        roll_error = -roll_deg
        pitch_error = -pitch_deg
        roll_cmd = self._roll_pid.update(roll_error, dt)
        pitch_cmd = self._pitch_pid.update(pitch_error, dt)

        roll_cmd = self._clamp_axis(
            roll_cmd,
            self._config.limits_deg["roll_min"],
            self._config.limits_deg["roll_max"],
        )
        pitch_cmd = self._clamp_axis(
            pitch_cmd,
            self._config.limits_deg["pitch_min"],
            self._config.limits_deg["pitch_max"],
        )

        self._last_roll_deg = roll_cmd
        self._last_pitch_deg = pitch_cmd

        ch = self._config.servo_channels
        pwm = {
            ch["roll"]: self._angle_to_pwm(roll_cmd, self._config.home_pwm["roll"]),
            ch["pitch"]: self._angle_to_pwm(pitch_cmd, self._config.home_pwm["pitch"]),
            ch["yaw"]: self._angle_to_pwm(
                self._manual_yaw_deg, self._config.home_pwm["yaw"]
            ),
        }
        self._last_valid_pwm = dict(pwm)
        return pwm

    def hold_last_or_home(self) -> Dict[int, int]:
        """Return last valid PWM during brief dropout, else home."""
        if self._last_valid_pwm:
            return dict(self._last_valid_pwm)
        return self.home()

    def get_stabilization_error(self) -> Dict[str, float]:
        """Return last stabilization command angles for logging."""
        return {"roll_deg": self._last_roll_deg, "pitch_deg": self._last_pitch_deg}

    def _clamp_axis(self, value: float, min_deg: float, max_deg: float) -> float:
        return max(min_deg, min(max_deg, value))

    def _angle_to_pwm(self, angle_deg: float, neutral_pwm: int) -> int:
        """Map angle in degrees to PWM microseconds."""
        delta_us = angle_deg / self._config.deg_per_us
        pwm = int(neutral_pwm + delta_us)
        return int(
            max(self._config.pwm_min_us, min(self._config.pwm_max_us, pwm))
        )
