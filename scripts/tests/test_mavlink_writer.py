"""Unit tests for MAVLink servo command coalescing."""

from __future__ import annotations

from unittest.mock import MagicMock, patch

from config_loader import MavlinkConfig
from mavlink_interface import MavlinkInterface


def _mavlink_config() -> MavlinkConfig:
    return MavlinkConfig(
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
        critical_message_ids=[0, 30, 33, 1, 76],
        ardupilot_copter_modes={5: "LOITER"},
    )


def test_pending_servos_coalesce_latest_per_channel() -> None:
    mav = MavlinkInterface(_mavlink_config())
    mav._pending_servos[9] = 1500
    mav._pending_servos[9] = 1600
    mav._pending_servos[10] = 1400
    assert mav._pending_servos[9] == 1600
    assert mav._pending_servos[10] == 1400


def test_flush_pending_servos_calls_send() -> None:
    mav = MavlinkInterface(_mavlink_config())
    mav._connection = MagicMock()
    mav._pending_servos = {9: 1500, 10: 1600}
    mav._last_servo_send = 0.0
    with patch.object(mav, "_send_servo_now") as mock_send:
        mav._flush_pending_servos()
        assert mock_send.call_count == 2
