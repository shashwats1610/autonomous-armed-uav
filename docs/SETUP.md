# Setup Guide

Step-by-step installation of the Raspberry Pi companion computer on **Pi 4 (8GB)** with **Raspberry Pi OS Bookworm (2024)**.

## Prerequisites

- Raspberry Pi 4 Model B (8GB)
- Raspberry Pi OS (64-bit recommended)
- CSI Camera Module v2 or HQ
- Pixhawk connected via USB (`/dev/ttyACM0`)
- Dedicated **5V 3A BEC** for the Pi (do not share with servo rail)
- Ground station PC on same network for UDP video

## 1. Flash and Boot Raspberry Pi OS

1. Flash Pi OS using Raspberry Pi Imager.
2. Enable SSH and set hostname (e.g. `uav-companion`).
3. Boot and update:

```bash
sudo apt update && sudo apt full-upgrade -y
```

## 2. Copy Project to Pi

```bash
cd ~
git clone <your-repo-url> autonomous-armed-uav
# or scp -r . pi@<pi-ip>:~/autonomous-armed-uav
```

## 3. Install Dependencies

```bash
cd ~/autonomous-armed-uav
chmod +x setup/*.sh
./setup/install_dependencies.sh
```

## 4. Configure System

```bash
./setup/configure_system.sh
sudo reboot
```

This enables the camera, disables USB autosuspend, increases serial buffer size, and sets CPU governor to performance.

## 5. Edit Configuration

Before first flight, update:

- `config/video_config.yaml` — `ground_station_ip`, `ground_station_port`
- `config/mavlink_config.yaml` — verify `serial_ports` and `target_system`
- `config/system_config.yaml` — trigger allowed modes and altitude

## 6. Install Systemd Services

```bash
./setup/install_services.sh
```

If the project is not at `/home/pi/autonomous-armed-uav`, edit paths in `systemd/*.service` before running.

## 7. Verify Hardware

```bash
./setup/test_hardware.sh
journalctl -u companion-main.service -f
journalctl -u video-stream.service -f
```

Confirm video at the ground station (UDP port 5600) **before arming**. Watchdog uses separate heartbeat files:

- `/run/companion/watchdog_main.heartbeat` — updated by `main.py` only
- `/run/companion/watchdog_video.heartbeat` — updated by `video_stream.py` only

`system_monitor.py` reads these files and does **not** refresh the main heartbeat (fixes false-negative watchdog).

## 8. Optional: Log Cleanup Cron

```bash
crontab -e
# Add: 0 3 * * 0 /home/pi/autonomous-armed-uav/setup/cleanup_logs.sh
```

## 9. Recommended Filesystem Hardening

For SD card reliability (see [CHALLENGES.md](CHALLENGES.md)):

- Use a high-endurance microSD or USB SSD boot
- Mount `logs/` on a separate partition if possible
- Avoid pulling power without `sudo shutdown`

## Troubleshooting

| Symptom | Action |
|---------|--------|
| No `/dev/ttyACM0` | Check USB cable, `lsusb`, user in `dialout` group |
| Camera not found | `rpicam-hello --list-cameras`, re-run `configure_system.sh` |
| No video at GCS | Verify IP/firewall, `tcpdump` UDP port 5600 |
| MAVLink disconnects | Check udev USB power rule, cable retention |
