#!/usr/bin/env bash
# install_services.sh - Install and enable systemd services for companion computer
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=== Copying systemd unit files ==="
sudo cp "${PROJECT_ROOT}/systemd/companion-main.service" /etc/systemd/system/
sudo cp "${PROJECT_ROOT}/systemd/video-stream.service" /etc/systemd/system/
sudo cp "${PROJECT_ROOT}/systemd/system-monitor.service" /etc/systemd/system/

echo "=== Reloading systemd ==="
sudo systemctl daemon-reload

echo "=== Enabling services ==="
sudo systemctl enable companion-main.service
sudo systemctl enable video-stream.service
sudo systemctl enable system-monitor.service

echo "=== Starting services ==="
sudo systemctl start companion-main.service
sudo systemctl start video-stream.service
sudo systemctl start system-monitor.service

echo "=== Service status ==="
sudo systemctl status companion-main.service --no-pager || true
sudo systemctl status video-stream.service --no-pager || true
sudo systemctl status system-monitor.service --no-pager || true

echo "=== Services installed ==="
