"""Unit tests for gimbal stabilization calculations."""

from __future__ import annotations

import time
from pathlib import Path

import pytest

from config_loader import GimbalConfig, PidGains, load_gimbal_config
from gimbal_controller import GimbalController, LowPassFilter, PidController
from mavlink_interface import AttitudeState


@pytest.fixture
def gimbal_config(tmp_path: Path) -> GimbalConfig:
    """Load gimbal config from project or minimal fixture."""
    root = tmp_path
    cfg_dir = root / "config"
    cfg_dir.mkdir()
    source = (
        Path(__file__).resolve().parent.parent.parent
        / "config"
        / "gimbal_config.yaml"
    )
    if source.is_file():
        (cfg_dir / "gimbal_config.yaml").write_text(
            source.read_text(encoding="utf-8"), encoding="utf-8"
        )
    else:
        pytest.skip("gimbal_config.yaml not found")
    return load_gimbal_config(root)


def test_angle_to_pwm_within_limits(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    pwm = controller._angle_to_pwm(10.0, 1500)
    assert gimbal_config.pwm_min_us <= pwm <= gimbal_config.pwm_max_us


def test_roll_clamping(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    assert controller._clamp_axis(100.0, -45.0, 45.0) == 45.0
    assert controller._clamp_axis(-100.0, -45.0, 45.0) == -45.0


def test_lowpass_filter_smoothing() -> None:
    lpf = LowPassFilter(cutoff_hz=0.1)
    values = []
    for i in range(50):
        sample = 1.0 if i >= 25 else 0.0
        values.append(lpf.update(sample, dt=0.05))
    assert values[-1] > values[24]
    assert values[-1] < 1.0


def test_pid_anti_windup() -> None:
    pid = PidController(PidGains(kp=10.0, ki=5.0, kd=0.0), output_limit=5.0)
    for _ in range(100):
        out = pid.update(10.0, dt=0.05)
    assert abs(out) <= 5.0


def test_compute_returns_three_channels(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    attitude = AttitudeState(roll=0.1, pitch=-0.05, yaw=0.0, timestamp=time.monotonic())
    pwm = controller.compute(attitude, dt=0.05)
    assert len(pwm) == 3
    for value in pwm.values():
        assert gimbal_config.pwm_min_us <= value <= gimbal_config.pwm_max_us


def test_no_attitude_timestamp_returns_home(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    att = AttitudeState(roll=0.5, pitch=0.5, yaw=0.0, timestamp=0.0)
    pwm = controller.compute(att, dt=0.05)
    assert pwm == controller.home()


def test_stale_attitude_returns_home(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    old = AttitudeState(roll=0.5, pitch=0.5, yaw=0.0, timestamp=time.monotonic() - 10.0)
    pwm = controller.compute(old, dt=0.05, now_mono=time.monotonic())
    home = controller.home()
    assert pwm == home


def test_hold_last_or_home(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    assert controller.hold_last_or_home() == controller.home()
    att = AttitudeState(roll=0.2, pitch=0.0, yaw=0.0, timestamp=time.monotonic())
    pwm = controller.compute(att, dt=0.05)
    assert controller.hold_last_or_home() == pwm


def test_emergency_disable(gimbal_config: GimbalConfig) -> None:
    controller = GimbalController(gimbal_config)
    pwm = controller.emergency_disable()
    assert pwm == controller.home()
