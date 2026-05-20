# Autonomous Armed UAV — Raspberry Pi Companion Computer

This repository contains **only the Raspberry Pi 4 companion computer software** for a larger autonomous tactical UAV program. It does **not** include Pixhawk firmware, airframe design, radio link hardware, ground-station application code, or payload mechanical design—those live in separate efforts and connect to this repo through defined interfaces (USB serial, PWM servos, UDP video, MAVLink commands).

If you are evaluating or deploying the full aircraft, treat this repo as one subsystem: **the onboard computer that handles FPV video, gimbal stabilization, payload trigger logic, and flight data logging**, while the autopilot flies the vehicle.

---

## Where this repo fits in the whole project

The full program splits responsibilities across hardware and software that are developed and maintained independently.

```text
┌─────────────────────────────────────────────────────────────────────────────┐
│                     AUTONOMOUS ARMED UAV (full program)                      │
├─────────────────────────────────────────────────────────────────────────────┤
│  Airframe & propulsion          │  Separate (structure, ESCs, batteries)   │
│  Pixhawk + ArduPilot            │  Separate (autopilot firmware, tuning)    │
│  Radio telemetry / GCS          │  Separate (handheld GS, mission planner)  │
│  Payload mechanism (mechanical) │  Separate (trigger hardware, wiring)      │
│  Gimbal hardware (servos)     │  Separate (3-axis mount, PWM to Pixhawk)  │
├─────────────────────────────────────────────────────────────────────────────┤
│  ★ THIS REPO ★                  │  Raspberry Pi 4 companion computer       │
│  autonomous-armed-uav           │  Python services, configs, setup, docs     │
└─────────────────────────────────────────────────────────────────────────────┘
```

### What the companion computer does (this repo)

| Responsibility | Handled here? | Handled elsewhere? |
|----------------|---------------|-------------------|
| Stabilize & navigate the aircraft | No | Pixhawk / ArduPilot |
| Arm/disarm, flight modes, RTL | No | Pixhawk / ArduPilot |
| Pilot FPV video to ground station | **Yes** | Pi CSI camera + FFmpeg UDP |
| Stabilize camera gimbal (roll/pitch) | **Yes** | Pi reads attitude, commands servos via Pixhawk |
| Manual payload trigger with safety checks | **Yes** | Pi sends servo PWM via MAVLink |
| Log MAVLink, gimbal, health, trigger events | **Yes** | Pi SD card under `logs/` |
| Monitor Pi temperature & restart services | **Yes** | `system_monitor.py` + systemd |

### Physical interfaces (how pieces connect)

```text
                    ┌──────────────┐
   CSI Camera ─────►│ Raspberry Pi │──── USB serial ────► Pixhawk (ArduPilot)
                    │  (this repo) │         MAVLink
                    └──────┬───────┘
                           │ UDP H.264 (port 5600)
                           ▼
                    ┌──────────────┐
                    │ Ground       │  Mission planner / video viewer
                    │ Station      │  (separate software)
                    └──────────────┘

   Pixhawk PWM outputs ──► Gimbal servos (ch 9–11) + Trigger servo (ch 12)
```

The Pi never replaces the autopilot. It **listens** to telemetry and **sends** high-level commands (servo positions, status text). The Pixhawk remains the authority on flight safety, arming, and motor control.

---

## Repository layout

```text
autonomous-armed-uav/
├── scripts/              # Python application code
├── config/               # YAML tunables (no hardcoded mission params in code)
├── systemd/              # Auto-start units for Pi OS
├── setup/                # Install, configure, test shell scripts
├── docs/                 # Setup, configuration, testing, engineering notes
├── logs/                 # Runtime flight logs (gitignored except .gitkeep)
├── requirements.txt      # Python dependencies
└── pytest.ini            # Off-board unit/integration tests
```

---

## Python modules (`scripts/`)

Each file is a focused module. **`main.py` is the only entry point for the primary flight stack**; video and system monitor run as separate processes under systemd.

### `main.py` — Orchestrator

**Role:** Ties every in-flight subsystem together on a single serial link to the Pixhawk.

**Contains:**

- `CompanionOrchestrator` — loads config, starts MAVLink, gimbal, trigger, and telemetry logger
- **Threads:**
  - Gimbal loop (20 Hz) — reads attitude, computes PWM, queues servo commands
  - OSD writer (10 Hz) — writes `osd_state.json` and `osd_overlay.txt` for video overlay
  - Health logger — CPU temp, load, battery; initiates shutdown on valid low-battery %
  - Thermal notifier — debounced `STATUSTEXT` to GCS when Pi overheats
  - Watchdog touch — updates `watchdog_main.heartbeat` for `system_monitor.py`
- Graceful shutdown on `SIGTERM` — safe trigger PWM, gimbal home, flush MAVLink queue, sync logs
- Signal handlers for systemd stop/restart

**Does not contain:** Camera capture or FFmpeg (delegated to `video_stream.py`).

---

### `mavlink_interface.py` — Pixhawk link

**Role:** Sole owner of the USB serial MAVLink connection (`/dev/ttyACM0` or `/dev/ttyUSB0`).

**Contains:**

- `MavlinkInterface` — connect, reconnect, heartbeat watchdog (5 s timeout)
- Thread-safe state: `ATTITUDE`, `GLOBAL_POSITION_INT`, `SYS_STATUS`, `HEARTBEAT`
- Inbound filtering by autopilot `target_system`; logs all messages via callback
- Outbound queue with **per-channel servo coalescing** (latest PWM per channel at 20 Hz max)
- `send_servo()` — `MAV_CMD_DO_SET_SERVO` to Pixhawk
- `send_statustext()` — warnings to ground station
- `flight_mode()` / `is_armed()` — ArduPilot mode map + heartbeat flags
- `COMMAND_LONG` dispatch for trigger handler (autopilot or broadcast target)
- `flush_outbound()` — drain queue on shutdown

**Edge cases handled:** USB disconnect/reconnect, serial buffer pressure, wrong-system heartbeats ignored, boot without Pixhawk (retry until connected).

---

### `gimbal_controller.py` — 3-axis gimbal logic

**Role:** Convert drone roll/pitch into compensating gimbal angles; output PWM map for servos 9–11.

**Contains:**

- `LowPassFilter` — 0.1 Hz smoothing on attitude (reduces IMU jitter)
- `PidController` — roll/pitch PID with anti-windup and output limits
- `GimbalController`:
  - `compute()` — stabilization angles → PWM (1000–2000 µs)
  - `home()` / `emergency_disable()` — safe neutral positions
  - `hold_last_or_home()` — brief dropout behavior
  - Invalid or stale attitude (`timestamp` / 2 s timeout) → home, no wild servo motion

**Yaw:** Manual pan only (not auto-stabilized); setpoint from config `manual_yaw_deg`.

**Does not contain:** MAVLink I/O (main loop calls `mavlink.send_servo`).

---

### `trigger_handler.py` — Payload trigger safety

**Role:** Actuate trigger servo (channel 12) only when ArduPilot interlocks pass.

**Contains:**

- `TriggerHandler.handle_command_long()` — reacts to `MAV_CMD_DO_SET_SERVO` (183) on trigger channel
- **Interlocks before fire:**
  - Vehicle **armed**
  - Flight mode in allow-list (`LOITER`, `GUIDED`, `AUTO`)
  - Mode not in block-list (`TAKEOFF`, `LAND`, `RTL`)
  - GPS position fresh (< 2 s) and not 0,0
  - Relative altitude ≥ configured minimum (default 10 m AGL)
  - 2 s cooldown between actuations
- Safe PWM (1000 µs) vs actuated (2000 µs)
- Rejection messages via MAVLink `STATUSTEXT`
- Callback to telemetry logger with lat/lon/alt/mode

**Important:** Trigger commands must appear on the **same serial bus** the Pi shares with the Pixhawk (see [docs/CONFIGURATION.md](docs/CONFIGURATION.md)).

---

### `video_stream.py` — FPV pipeline

**Role:** Standalone process for CSI camera → H.264 → UDP to ground station.

**Contains:**

- `detect_camera_cli()` — `rpicam-vid` or `libcamera-vid` auto-detect
- `build_camera_cmd()` — stdout pipe (**no `--listen`**; that mode is TCP-only)
- `build_ffmpeg_cmd()` — copy or re-encode path, UDP MPEG-TS with low-latency mux flags
- Live OSD via `drawtext=textfile=...:reload=1` reading `/run/companion/osd_overlay.txt`
- `VideoStreamer` — subprocess lifecycle, crash restart, thermal bitrate step restart (debounced)
- `watchdog_video.heartbeat` — touched every second while streaming
- Helpers: `sanitize_osd_text()`, `write_osd_text_file()`, `read_thermal_bitrate()`

**Runs under:** `video-stream.service` (not inside `main.py`) so encoding load does not block the 20 Hz gimbal loop.

---

### `telemetry_logger.py` — Flight recording

**Role:** Thread-safe persistence for post-flight analysis.

**Contains:**

- Timestamped **`.tlog`** — raw MAVLink bytes (rotate safely: close → gzip → new file)
- **`gimbal_*.csv`** — angles, PWM, stabilization error (rate limited by `gimbal_log_hz` in main)
- **`trigger_*.csv`** — actuation events with GPS
- **`health_*.csv`** — CPU temp, load, uptime, battery %
- `fsync` on critical events (trigger, shutdown); periodic tlog flush every N messages

---

### `system_monitor.py` — Pi health and watchdog

**Role:** Separate process; monitors the Pi itself, not the aircraft autopilot.

**Contains:**

- Thermal sysfs read → `thermal_alert.json` (drives video bitrate steps + main warnings)
- **Watchdog (read-only on heartbeats):**
  - Stale `watchdog_main.heartbeat` → restart `companion-main.service`
  - Stale `watchdog_video.heartbeat` → restart `video-stream.service`
  - Restart cooldown (60 s) to avoid storms
- `system_events.log` — thermal and restart events

**Does not write** the main heartbeat file (fixed bug where monitor masked a dead main process).

---

### `config_loader.py` — Configuration

**Role:** Load and validate all YAML under `config/`; fail fast on boot with clear errors.

**Contains:**

- Dataclasses: `MavlinkConfig`, `VideoConfig`, `GimbalConfig`, `SystemConfig`, `TriggerConfig`, `WatchdogConfig`, `CompanionConfig`
- Validators: IP format, PWM range, servo channel keys, positive baud/PID gains
- `RPI_COMPANION_ROOT` env override for non-default install paths
- `resolve_log_dir()` — absolute path to `logs/`

---

### `scripts/tests/` — Automated tests (25 cases)

| File | What it verifies |
|------|------------------|
| `test_gimbal_controller.py` | PWM limits, LPF, PID windup, stale/no attitude → home, hold-last |
| `test_mavlink_integration.py` | Trigger interlocks, GPS stale, cooldown, command targeting |
| `test_mavlink_writer.py` | Servo command coalescing per channel |
| `test_video_stream.py` | Camera cmd (no listen), FFmpeg UDP args, OSD sanitization |

Run off-board: `pip install -r requirements.txt && pytest`

---

## Configuration (`config/`)

| File | Purpose |
|------|---------|
| `mavlink_config.yaml` | Serial ports, baud 57600, system IDs, heartbeat timeout, message filters, ArduPilot mode map |
| `video_config.yaml` | Resolution, FPS, UDP IP/port, bitrate, OSD paths, thermal bitrate steps |
| `gimbal_config.yaml` | Servo channels 9–11, PID gains, mechanical limits, LPF cutoff, log rate |
| `system_config.yaml` | Thermal thresholds, watchdog paths, trigger rules, log rotation, battery shutdown |

Full key reference: [docs/CONFIGURATION.md](docs/CONFIGURATION.md)

---

## Setup scripts (`setup/`)

| Script | Purpose |
|--------|---------|
| `install_dependencies.sh` | `apt` packages (FFmpeg, libcamera/rpicam, Python libs) + `pip install -r requirements.txt` |
| `configure_system.sh` | Enable camera, USB autosuspend off, serial buffer, performance governor, `/run/companion` |
| `install_services.sh` | Copy systemd units, enable and start all three services |
| `test_hardware.sh` | Pre-flight: camera, serial, MAVLink heartbeat, thermal, FFmpeg smoke test |
| `cleanup_logs.sh` | Delete logs older than 30 days |

Detailed steps: [docs/SETUP.md](docs/SETUP.md)

---

## Systemd services (`systemd/`)

| Unit | Runs | Restart policy |
|------|------|----------------|
| `companion-main.service` | `scripts/main.py` | Always — MAVLink, gimbal, trigger, logging |
| `video-stream.service` | `scripts/video_stream.py` | Always — independent video pipeline |
| `system-monitor.service` | `scripts/system_monitor.py` | Always — thermal + watchdog |

Default install path on Pi: `/home/pi/autonomous-armed-uav` (edit units if you clone elsewhere).

---

## Runtime data flow (in flight)

```text
Pixhawk ──ATTITUDE/GPS/BATTERY──► mavlink_interface ──► gimbal_controller ──► servo commands ──► Pixhawk
                                      │
                                      ├──► trigger_handler ◄── COMMAND_LONG (from GCS on same bus)
                                      │
                                      └──► telemetry_logger ──► logs/

main.py ──osd_state.json / osd_overlay.txt──► video_stream.py ──UDP──► Ground Station

system_monitor.py ──thermal_alert.json──► video_stream.py (bitrate / restart)
                 └── reads watchdog_* (never fakes "main alive")
```

---

## Quick start (Raspberry Pi only)

```bash
git clone https://github.com/shashwats1610/autonomous-armed-uav.git
cd autonomous-armed-uav
chmod +x setup/*.sh
./setup/install_dependencies.sh
./setup/configure_system.sh
sudo reboot
# After reboot — set ground_station_ip in config/video_config.yaml first:
./setup/install_services.sh
./setup/test_hardware.sh
```

Verify UDP video at the ground station **before arming**. Pre-flight checklist: [docs/TESTING.md](docs/TESTING.md)

---

## Documentation index

| Document | Contents |
|----------|----------|
| [docs/SETUP.md](docs/SETUP.md) | Pi OS install, wiring assumptions, service enablement |
| [docs/CONFIGURATION.md](docs/CONFIGURATION.md) | Every YAML parameter; MAVLink routing for trigger |
| [docs/TESTING.md](docs/TESTING.md) | Bench and field validation checklist |
| [docs/CHALLENGES.md](docs/CHALLENGES.md) | Real-world issues (latency, brownouts, watchdog, camera pipe) and fixes |

---

## Hardware assumed by this repo

- Raspberry Pi 4 Model B (8 GB), Raspberry Pi OS Bookworm (2024), Python 3.11
- Raspberry Pi CSI camera (v2 or HQ)
- Pixhawk-class autopilot running **ArduPilot Copter**, USB connection to Pi
- Gimbal servos on Pixhawk outputs **9 (roll), 10 (pitch), 11 (yaw)**
- Trigger servo on output **12**
- Dedicated **5 V 3 A BEC** for the Pi (not shared with servo rail)
- Ground station on same network for UDP video (default port **5600**)

---

## License and use

Internal / educational project use. Operators are responsible for compliance with local laws regarding unmanned aircraft and payload systems. This software implements safety interlocks but does not remove the need for trained operators and appropriate airspace authorization.
