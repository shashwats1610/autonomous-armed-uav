# Raspberry Pi Companion Computer

Production-ready companion computer software for an autonomous tactical drone. Runs on **Raspberry Pi 4 (8GB)** with **Raspberry Pi OS (2024)** and integrates with a **Pixhawk / ArduPilot** autopilot over MAVLink.

## Features

- **MAVLink telemetry** — attitude, GPS, battery, heartbeat with auto-reconnect
- **FPV video** — CSI camera H.264 UDP stream via `rpicam-vid` / `libcamera-vid` + FFmpeg
- **3-axis gimbal** — roll/pitch stabilization with PID + low-pass filtering (20 Hz)
- **Trigger control** — manual payload actuation with ArduPilot safety interlocks
- **Telemetry logging** — `.tlog`, CSV gimbal/trigger/health logs with rotation
- **Watchdog** — thermal monitoring, process heartbeat, systemd auto-restart

## Architecture

```
Pixhawk (USB) <--MAVLink--> main.py (gimbal, trigger, logging)
CSI Camera --> video_stream.py --> UDP --> Ground Station
system_monitor.py --> thermal alerts, watchdog
```

Three systemd services run independently:

| Service | Script | Role |
|---------|--------|------|
| `companion-main.service` | `main.py` | MAVLink, gimbal, trigger, OSD writer |
| `video-stream.service` | `video_stream.py` | Camera + FFmpeg streaming |
| `system-monitor.service` | `system_monitor.py` | Thermal, watchdog |

## Quick Start

```bash
cd ~/autonomous-armed-uav
chmod +x setup/*.sh
./setup/install_dependencies.sh
./setup/configure_system.sh
sudo reboot
# After reboot:
./setup/install_services.sh
./setup/test_hardware.sh
```

Edit `config/video_config.yaml` to set your ground station IP before streaming.

## Documentation

- [SETUP.md](docs/SETUP.md) — Installation guide
- [CONFIGURATION.md](docs/CONFIGURATION.md) — Config reference
- [TESTING.md](docs/TESTING.md) — Pre-flight checklist
- [CHALLENGES.md](docs/CHALLENGES.md) — Engineering post-mortem (edge cases)

## Development (off-board)

```bash
pip install -r requirements.txt
pytest
```

## License

Internal / project use. Ensure compliance with local regulations for autonomous systems and payload actuation.
