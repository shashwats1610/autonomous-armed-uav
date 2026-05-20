# Configuration Reference

All tunable parameters live in `config/*.yaml`. The companion fails fast on startup if required keys are missing or invalid.

## mavlink_config.yaml

| Parameter | Description |
|-----------|-------------|
| `serial_ports` | Device paths tried in order (`/dev/ttyACM0`, `/dev/ttyUSB0`) |
| `baud_rate` | Serial baud (57600 standard) |
| `target_system` / `target_component` | Pixhawk MAVLink IDs |
| `source_system` / `source_component` | Companion computer IDs |
| `heartbeat_timeout_s` | Reconnect if no heartbeat (default 5 s) |
| `connection_timeout_s` | Boot wait for first connection |
| `servo_max_rate_hz` | Outbound servo command cap (20 Hz) |
| `outbound_queue_depth` | Command queue size |
| `critical_message_ids` | Messages always processed under load |
| `ardupilot_copter_modes` | custom_mode → name map for logging/interlocks |

## video_config.yaml

| Parameter | Description |
|-----------|-------------|
| `ground_station_ip` | UDP destination for MPEG-TS stream |
| `ground_station_port` | UDP port (default 5600) |
| `width` / `height` / `framerate` | Stream resolution (1280x720 @ 30) |
| `bitrate_bps` | Target H.264 bitrate |
| `thermal_bitrate_steps` | Bitrate ladder when Pi overheats |
| `camera_cli` | `auto`, `rpicam`, or `libcamera` |
| `osd_enabled` | FFmpeg `drawtext` overlay from OSD JSON |
| `osd_state_file` | IPC JSON written by `main.py` |
| `osd_text_file` | Text file for FFmpeg `drawtext` with `reload=1` |
| `thermal_alert_file` | IPC file written by `system_monitor.py` |
| `thermal_restart_debounce_s` | Min interval between thermal pipeline restarts |
| `use_inline_h264` | Enable camera inline H.264 and FFmpeg copy when OSD off |

## gimbal_config.yaml

| Parameter | Description |
|-----------|-------------|
| `servo_channels` | Pixhawk servo numbers (roll=9, pitch=10, yaw=11) |
| `pwm_min_us` / `pwm_max_us` | 1000–2000 µs limits |
| `home_pwm` | Neutral positions per axis |
| `update_hz` | Control loop rate (20) |
| `gimbal_log_hz` | CSV log rate (default 5); control stays at `update_hz` |
| `attitude_lpf_cutoff_hz` | Attitude low-pass (0.1 Hz) |
| `limits_deg` | Mechanical roll/pitch limits |
| `pid.roll` / `pid.pitch` | Kp, Ki, Kd per axis |
| `attitude_timeout_s` | Stale data → hold home (2 s) |
| `manual_yaw_deg` | Operator pan setpoint |
| `deg_per_us` | Angle-to-PWM linear scale |

## system_config.yaml

| Parameter | Description |
|-----------|-------------|
| `runtime_dir` | IPC directory (`/run/companion`) |
| `log_dir` | Flight logs (relative to project root) |
| `log_rotation_max_bytes` | Per-file rotation size (1 GB) |
| `thermal_warning_c` | MAVLink warning threshold (75°C) |
| `thermal_critical_c` | Aggressive bitrate reduction (80°C) |
| `low_battery_percent` | Graceful shutdown threshold |
| `trigger.*` | Servo channel, PWM, cooldown, allowed modes, `position_timeout_s` |
| `tlog_flush_interval` | Flush `.tlog` every N MAVLink messages |
| `thermal_statustext_interval_s` | Debounce Pi temperature warnings to GCS |
| `watchdog.main_heartbeat_file` | Written only by `main.py` (monitor reads) |
| `watchdog.video_heartbeat_file` | Written only by `video_stream.py` |
| `watchdog.restart_cooldown_s` | Min seconds between systemd restarts |

## ArduPilot Trigger Modes

Default allowed modes: `LOITER`, `GUIDED`, `AUTO`. Blocked: `TAKEOFF`, `LAND`, `RTL`.

Adjust `trigger.allowed_modes` for your operational rules. PX4 uses different mode names — this build targets **ArduPilot Copter**.

## MAVLink Wiring (Trigger Commands)

The companion opens the **Pixhawk USB serial** exclusively. Trigger commands use `MAV_CMD_DO_SET_SERVO` (183) with `param1=12` (servo channel) and must appear on that bus:

- Ground station → Pixhawk → USB → companion (passive listener), **or**
- MAVProxy / mavlink-router forwarding `COMMAND_LONG` to `target_system=1`

Commands addressed only to other systems are ignored. Servo outputs are executed on the autopilot via `target_system=1` in outbound `COMMAND_LONG`.

## Network Tuning

- Use wired Ethernet or dedicated 5 GHz WiFi for video when possible.
- Reduce `bitrate_bps` if packet loss is observed at the GCS.
- Match GCS decoder to MPEG-TS over UDP (e.g. VLC, QGroundControl custom pipeline).

## Environment Variables

| Variable | Effect |
|----------|--------|
| `RPI_COMPANION_ROOT` | Override project root path for configs |
