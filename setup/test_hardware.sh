#!/usr/bin/env bash
# test_hardware.sh - Pre-flight hardware validation for companion computer
set -euo pipefail

PASS=0
FAIL=0

check() {
  local name="$1"
  shift
  if "$@"; then
    echo "[PASS] ${name}"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] ${name}"
    FAIL=$((FAIL + 1))
  fi
}

echo "=== Raspberry Pi Companion Hardware Validation ==="

check "Python 3 available" command -v python3
check "PyMAVLink import" python3 -c "import pymavlink"
check "FFmpeg available" command -v ffmpeg

CAM_BIN=""
if command -v rpicam-vid >/dev/null 2>&1; then
  CAM_BIN="rpicam-vid"
elif command -v libcamera-vid >/dev/null 2>&1; then
  CAM_BIN="libcamera-vid"
fi

if [ -n "${CAM_BIN}" ]; then
  if ${CAM_BIN} --list-cameras >/dev/null 2>&1; then
    echo "[PASS] CSI camera detected (${CAM_BIN})"
    PASS=$((PASS + 1))
  else
    echo "[FAIL] CSI camera not detected (${CAM_BIN})"
    FAIL=$((FAIL + 1))
  fi
else
  echo "[FAIL] No camera CLI (rpicam-vid / libcamera-vid)"
  FAIL=$((FAIL + 1))
fi

if [ -e /dev/ttyACM0 ] || [ -e /dev/ttyUSB0 ]; then
  check "Pixhawk serial device present" test -e /dev/ttyACM0 -o -e /dev/ttyUSB0
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
  export RPI_COMPANION_ROOT="${PROJECT_ROOT}"
  if python3 -c "
from pymavlink import mavutil
import sys
port = '/dev/ttyACM0' if __import__('os').path.exists('/dev/ttyACM0') else '/dev/ttyUSB0'
try:
    m = mavutil.mavlink_connection(f'{port},57600', timeout=5)
    m.wait_heartbeat(timeout=5)
    sys.exit(0)
except Exception:
    sys.exit(1)
" 2>/dev/null; then
    echo "[PASS] MAVLink heartbeat received"
    PASS=$((PASS + 1))
  else
    echo "[WARN] Serial present but no heartbeat (connect power to Pixhawk)"
  fi
else
  echo "[WARN] No /dev/ttyACM0 or /dev/ttyUSB0 (connect Pixhawk for MAVLink test)"
fi

if [ -f /sys/class/thermal/thermal_zone0/temp ]; then
  TEMP=$(cat /sys/class/thermal/thermal_zone0/temp)
  TEMP_C=$((TEMP / 1000))
  echo "CPU temperature: ${TEMP_C}C"
  check "CPU temperature readable" test "${TEMP_C}" -gt 0
else
  echo "[FAIL] Thermal sensor not found"
  FAIL=$((FAIL + 1))
fi

if ffmpeg -hide_banner -f lavfi -i color=c=black:s=64x64:d=0.1 -frames:v 1 -f null - >/dev/null 2>&1; then
  echo "[PASS] FFmpeg encode smoke test"
  PASS=$((PASS + 1))
else
  echo "[FAIL] FFmpeg encode smoke test"
  FAIL=$((FAIL + 1))
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
if [ -f "${PROJECT_ROOT}/config/mavlink_config.yaml" ]; then
  check "Config files present" test -f "${PROJECT_ROOT}/config/mavlink_config.yaml"
fi

echo ""
echo "Results: ${PASS} passed, ${FAIL} failed"
if [ "${FAIL}" -gt 0 ]; then
  exit 1
fi
exit 0
