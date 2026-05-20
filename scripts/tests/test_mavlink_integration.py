"""Integration tests for MAVLink-driven gimbal and trigger behavior."""

from __future__ import annotations

import time
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

from config_loader import TriggerConfig
from gimbal_controller import GimbalController
from mavlink_interface import AttitudeState, MavlinkInterface, PositionState
from trigger_handler import TriggerHandler


@pytest.fixture
def trigger_config() -> TriggerConfig:
    return TriggerConfig(
        servo_channel=12,
        pwm_safe_us=1000,
        pwm_actuated_us=2000,
        cooldown_s=2.0,
        command_id=183,
        min_altitude_m=10.0,
        position_timeout_s=2.0,
        allowed_modes=["LOITER", "GUIDED"],
        blocked_modes=["LAND", "RTL", "TAKEOFF"],
    )


def _mock_mavlink(
    armed: bool = True,
    mode: str = "LOITER",
    alt_m: float = 50.0,
    pos_timestamp: float | None = None,
    lat: float = 37.0,
    lon: float = -122.0,
) -> MagicMock:
    mav = MagicMock()
    mav.is_armed.return_value = armed
    mav.flight_mode.return_value = mode
    ts = pos_timestamp if pos_timestamp is not None else time.monotonic()
    mav.get_position.return_value = PositionState(
        lat=lat,
        lon=lon,
        alt_m=100.0,
        relative_alt_m=alt_m,
        timestamp=ts,
    )
    mav.send_servo.return_value = True
    mav.send_statustext.return_value = True
    return mav


def test_trigger_rejected_when_disarmed(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(armed=False)
    events: list = []
    handler = TriggerHandler(trigger_config, mav, on_trigger_event=events.append)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    mav.send_servo.assert_not_called()
    assert len(events) == 0


def test_trigger_fires_when_armed_and_valid(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(armed=True, mode="LOITER", alt_m=50.0)
    events: list = []
    handler = TriggerHandler(trigger_config, mav, on_trigger_event=events.append)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    mav.send_servo.assert_called_with(12, 2000)
    assert len(events) == 1


def test_trigger_rejected_stale_gps(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(
        armed=True,
        mode="LOITER",
        alt_m=50.0,
        pos_timestamp=time.monotonic() - 10.0,
    )
    handler = TriggerHandler(trigger_config, mav)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    mav.send_servo.assert_not_called()


def test_trigger_rejected_zero_gps(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(armed=True, mode="LOITER", alt_m=50.0, lat=0.0, lon=0.0)
    handler = TriggerHandler(trigger_config, mav)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    mav.send_servo.assert_not_called()


def test_trigger_rejected_takeoff_mode(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(armed=True, mode="TAKEOFF", alt_m=50.0)
    handler = TriggerHandler(trigger_config, mav)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    mav.send_servo.assert_not_called()


def test_trigger_cooldown(trigger_config: TriggerConfig) -> None:
    mav = _mock_mavlink(armed=True, mode="LOITER", alt_m=50.0)
    handler = TriggerHandler(trigger_config, mav)
    msg = SimpleNamespace(command=183, param1=12, param2=2000)
    handler.handle_command_long(msg)
    handler.handle_command_long(msg)
    assert mav.send_servo.call_count == 1


def test_gimbal_compensates_roll() -> None:
    from config_loader import load_config
    from pathlib import Path

    root = Path(__file__).resolve().parent.parent.parent
    if not (root / "config" / "gimbal_config.yaml").is_file():
        pytest.skip("Project config not found")
    config = load_config(root)
    controller = GimbalController(config.gimbal)
    att = AttitudeState(roll=0.2, pitch=0.0, yaw=0.0, timestamp=time.monotonic())
    pwm = controller.compute(att, dt=0.05)
    roll_ch = config.gimbal.servo_channels["roll"]
    assert pwm[roll_ch] != config.gimbal.home_pwm["roll"]


def test_mavlink_command_targets_autopilot() -> None:
    from config_loader import MavlinkConfig
    from mavlink_interface import MavlinkInterface

    cfg = MavlinkConfig(
        serial_ports=["/dev/ttyACM0"],
        baud_rate=57600,
        target_system=1,
        target_component=1,
        source_system=255,
        source_component=190,
        heartbeat_timeout_s=5.0,
        connection_timeout_s=120.0,
        reconnect_initial_s=1.0,
        reconnect_max_s=30.0,
        outbound_queue_depth=64,
        servo_max_rate_hz=20.0,
        critical_message_ids=[76],
        ardupilot_copter_modes={},
    )
    mav = MavlinkInterface(cfg)
    assert mav._should_handle_command(SimpleNamespace(target_system=1))
    assert mav._should_handle_command(SimpleNamespace(target_system=0))
    assert not mav._should_handle_command(SimpleNamespace(target_system=255))
