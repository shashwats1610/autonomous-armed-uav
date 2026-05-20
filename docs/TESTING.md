# Testing Guide

## Pre-Flight Checklist

### Hardware

- [ ] Dedicated 5V 3A BEC powers Raspberry Pi only
- [ ] Pixhawk USB cable secured (strain relief)
- [ ] CSI camera ribbon seated correctly
- [ ] Active cooling fan operational on Pi case
- [ ] SD card or SSD health verified (`sudo smartctl` if applicable)

### Software

- [ ] `./setup/test_hardware.sh` passes
- [ ] `config/video_config.yaml` ground station IP correct
- [ ] `journalctl -u companion-main` shows MAVLink heartbeat
- [ ] `journalctl -u video-stream` shows camera + FFmpeg running
- [ ] Trigger in **safe** PWM (1000 µs) at boot
- [ ] Gimbal moves smoothly through full range on bench (disarmed)

### Safety

- [ ] Payload trigger tested only in designated area
- [ ] Trigger rejected when disarmed (integration test)
- [ ] Trigger rejected below minimum altitude
- [ ] Emergency disarm procedure briefed

## Automated Tests

From project root (off-board or on Pi):

```bash
pip install -r requirements.txt
pytest
```

### Unit Tests

`scripts/tests/test_gimbal_controller.py`

- PWM mapping within limits
- Angle clamping
- Low-pass filter smoothing
- PID anti-windup
- Stale attitude → home position

### Integration Tests

`scripts/tests/test_mavlink_integration.py`

- Trigger rejected when disarmed
- Trigger fires when armed + valid mode/altitude
- Trigger rejected for stale GPS, zero GPS, TAKEOFF mode, cooldown
- Gimbal roll compensation produces non-neutral PWM
- COMMAND_LONG targeting autopilot system ID is accepted

### Video Helpers

`scripts/tests/test_video_stream.py` — camera command builder (no `--listen`), OSD sanitization, FFmpeg UDP args

### MAVLink Writer

`scripts/tests/test_mavlink_writer.py` — per-channel servo coalescing

## Bench Validation

1. Power Pi + Pixhawk without props.
2. Connect QGroundControl to Pixhawk radio.
3. Verify attitude in companion logs matches QGC.
4. Stream video to GCS — measure glass-to-glass latency (target &lt; 200 ms).
5. Move airframe by hand — gimbal should counter roll/pitch.
6. Send trigger command — verify rejection when disarmed.

## Log Review Post-Flight

```bash
ls -lh logs/
# flight_*.tlog  - MAVLink binary
# gimbal_*.csv   - servo positions
# trigger_*.csv  - actuation events
# health_*.csv   - CPU temp, load
```

## Periodic Maintenance

```bash
./setup/cleanup_logs.sh
```

Run weekly or via cron.
