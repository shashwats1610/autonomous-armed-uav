#!/usr/bin/env bash
# configure_system.sh - Configure Raspberry Pi OS for companion computer operation
set -euo pipefail

echo "=== Enabling camera interface ==="
sudo raspi-config nonint do_camera 0

echo "=== Disabling serial console (free UART if needed) ==="
sudo raspi-config nonint do_serial 2

echo "=== Setting GPU memory split (128MB minimum for camera) ==="
sudo raspi-config nonint do_memory_split 128

echo "=== Disabling USB autosuspend ==="
echo 'SUBSYSTEM=="usb", ATTR{power/autosuspend}=-1' | sudo tee /etc/udev/rules.d/50-usb-power.rules

echo "=== Increasing USB serial buffer size for Pixhawk ==="
echo 'options usbserial vendor=0x26ac product=0x0011 buffer_size=4096' | sudo tee /etc/modprobe.d/usbserial.conf

echo "=== Setting CPU governor to performance ==="
if [ -f /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor ]; then
  echo 'performance' | sudo tee /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
fi

echo "=== Creating runtime directory for IPC ==="
sudo mkdir -p /run/companion
sudo chown pi:pi /run/companion

echo "=== Adding pi user to dialout group for serial access ==="
sudo usermod -aG dialout pi

echo "Configuration complete. Reboot required."
echo "Run: sudo reboot"
