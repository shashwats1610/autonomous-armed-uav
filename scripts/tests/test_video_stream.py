"""Unit tests for video streaming helpers."""

from __future__ import annotations

import json
from pathlib import Path

from video_stream import (
    build_camera_cmd,
    build_ffmpeg_cmd,
    detect_camera_cli,
    read_thermal_bitrate,
    sanitize_osd_text,
    write_osd_text_file,
)


def test_detect_camera_cli_auto_returns_string_or_none() -> None:
    result = detect_camera_cli("auto")
    assert result is None or result in ("rpicam-vid", "libcamera-vid")


def test_build_camera_cmd_no_listen() -> None:
    cmd = build_camera_cmd("rpicam-vid", 1280, 720, 30, True)
    assert "--listen" not in cmd
    assert "-o" in cmd
    assert cmd[-1] == "-"
    assert "--codec" in cmd


def test_build_ffmpeg_cmd_copy_mode() -> None:
    cmd = build_ffmpeg_cmd("192.168.1.1", 5600, 2_000_000, True, False, "")
    assert "-c:v" in cmd
    assert "copy" in cmd
    assert "pkt_size=1316" in "".join(cmd)


def test_sanitize_osd_text_strips_unsafe() -> None:
    raw = "Mode LOITER: alt 10m\\n"
    cleaned = sanitize_osd_text(raw)
    assert ":" not in cleaned
    assert "\\" not in cleaned


def test_read_thermal_bitrate_default(tmp_path: Path) -> None:
    bps, level = read_thermal_bitrate(
        str(tmp_path / "missing.json"), [2_000_000, 1_000_000], 2_000_000
    )
    assert bps == 2_000_000
    assert level == 0


def test_write_osd_text_file(tmp_path: Path) -> None:
    osd_json = tmp_path / "osd.json"
    text_file = tmp_path / "overlay.txt"
    osd_json.write_text(
        json.dumps({"lat": 1.0, "lon": 2.0, "alt_m": 10, "battery_pct": 80, "mode": "LOITER"}),
        encoding="utf-8",
    )
    write_osd_text_file(str(osd_json), str(text_file))
    content = text_file.read_text(encoding="utf-8")
    assert "LOITER" in content
