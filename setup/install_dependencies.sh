#!/usr/bin/env bash
# install_dependencies.sh - Install system and Python packages for companion computer
set -euo pipefail

echo "=== Installing system packages ==="
sudo apt-get update
sudo apt-get install -y \
  python3-pip \
  python3-venv \
  ffmpeg \
  libcamera-apps \
  rpicam-apps \
  python3-opencv \
  v4l-utils

echo "=== Installing Python packages ==="
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
pip3 install --user -r "${PROJECT_ROOT}/requirements.txt"

echo "=== Verifying installations ==="
python3 -c "import pymavlink; print('PyMAVLink OK')"
python3 -c "import yaml; print('PyYAML OK')"
python3 -c "import numpy; print('NumPy OK')"
ffmpeg -version | head -n1

if command -v rpicam-hello >/dev/null 2>&1; then
  rpicam-hello --list-cameras || true
elif command -v libcamera-hello >/dev/null 2>&1; then
  libcamera-hello --list-cameras || true
else
  echo "WARNING: No camera hello utility found"
fi

echo "=== Dependency installation complete ==="
