"""FFmpeg-based CSI camera video streaming with OSD overlay."""

from __future__ import annotations

import json
import logging
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import List, Optional, Tuple

from config_loader import CompanionConfig, load_config

logger = logging.getLogger(__name__)

# Characters that break FFmpeg drawtext / filter syntax
_OSD_UNSAFE_RE = re.compile(r"[':\\\n\r]")


def detect_camera_cli(preference: str) -> Optional[str]:
    """
    Detect available camera CLI binary.

    preference: auto | rpicam | libcamera
    """
    candidates: List[str] = []
    if preference == "rpicam":
        candidates = ["rpicam-vid"]
    elif preference == "libcamera":
        candidates = ["libcamera-vid"]
    else:
        candidates = ["rpicam-vid", "libcamera-vid"]

    for binary in candidates:
        if shutil.which(binary):
            return binary
    return None


def probe_camera(binary: str) -> bool:
    """Return True if at least one camera is detected."""
    try:
        result = subprocess.run(
            [binary, "--list-cameras"],
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        output = result.stdout + result.stderr
        return "No cameras" not in output and result.returncode == 0
    except (subprocess.SubprocessError, OSError) as exc:
        logger.warning("Camera probe failed: %s", exc)
        return False


def sanitize_osd_text(text: str) -> str:
    """Remove characters that break FFmpeg drawtext filters."""
    cleaned = _OSD_UNSAFE_RE.sub(" ", text)
    return cleaned[:120]


def build_osd_line(osd_path: str) -> str:
    """Build drawtext-safe OSD string from OSD state JSON."""
    try:
        with open(osd_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return "OSD unavailable"
    raw = (
        f"Lat {data.get('lat', 0):.5f} Lon {data.get('lon', 0):.5f} "
        f"Alt {data.get('alt_m', 0):.1f}m Bat {data.get('battery_pct', -1)}% "
        f"Mode {data.get('mode', 'UNK')}"
    )
    return sanitize_osd_text(raw)


def write_osd_text_file(osd_json_path: str, text_file_path: str) -> None:
    """Refresh OSD overlay text file for FFmpeg drawtext reload."""
    line = build_osd_line(osd_json_path)
    path = Path(text_file_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        handle.write(line + "\n")
        handle.flush()


def read_thermal_bitrate(
    thermal_path: str, steps: List[int], default_bps: int
) -> Tuple[int, int]:
    """Return (bitrate_bps, bitrate_level) from thermal alert file."""
    try:
        with open(thermal_path, "r", encoding="utf-8") as handle:
            data = json.load(handle)
        level = int(data.get("bitrate_level", 0))
        if 0 <= level < len(steps):
            return steps[level], level
    except (OSError, json.JSONDecodeError, ValueError):
        pass
    return default_bps, 0


def build_camera_cmd(
    binary: str, width: int, height: int, framerate: int, use_inline: bool
) -> List[str]:
    """Build camera capture command for stdout H.264 pipe (no --listen)."""
    cmd: List[str] = [
        binary,
        "-t",
        "0",
        "--width",
        str(width),
        "--height",
        str(height),
        "--framerate",
        str(framerate),
        "-n",
    ]
    if binary == "rpicam-vid":
        cmd.extend(["--codec", "h264"])
    if use_inline:
        cmd.append("--inline")
    cmd.extend(["-o", "-"])
    return cmd


def build_ffmpeg_cmd(
    udp_ip: str,
    udp_port: int,
    bitrate_bps: int,
    use_copy: bool,
    osd_enabled: bool,
    osd_text_file: str,
) -> List[str]:
    """Build FFmpeg command for UDP MPEG-TS output."""
    udp_target = f"udp://{udp_ip}:{udp_port}?pkt_size=1316"
    base = [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-fflags",
        "nobuffer",
        "-f",
        "h264",
        "-i",
        "-",
    ]
    if osd_enabled and osd_text_file:
        vf = (
            f"drawtext=textfile={osd_text_file}:reload=1:"
            f"x=10:y=10:fontsize=18:fontcolor=white"
        )
        return base + [
            "-vf",
            vf,
            "-c:v",
            "libx264",
            "-b:v",
            str(bitrate_bps),
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
            "-f",
            "mpegts",
            udp_target,
        ]
    if use_copy:
        return base + [
            "-c:v",
            "copy",
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
            "-f",
            "mpegts",
            udp_target,
        ]
    return base + [
        "-c:v",
        "libx264",
        "-b:v",
        str(bitrate_bps),
        "-muxdelay",
        "0",
        "-muxpreload",
        "0",
        "-f",
        "mpegts",
        udp_target,
    ]


class VideoStreamer:
    """Manages libcamera/rpicam -> FFmpeg UDP pipeline with auto-restart."""

    def __init__(self, config: CompanionConfig) -> None:
        self._config = config
        self._video = config.video
        self._system = config.system
        self._stop = False
        self._camera_proc: Optional[subprocess.Popen[bytes]] = None
        self._ffmpeg_proc: Optional[subprocess.Popen[bytes]] = None
        self._camera_binary: Optional[str] = None
        self._last_bitrate_level = -1
        self._last_thermal_restart = 0.0
        self._video_hb_path = Path(self._system.watchdog.video_heartbeat_file)
        self._osd_text_path = Path(self._video.osd_text_file)

    def run(self) -> None:
        """Main loop: start pipeline, monitor, restart on failure."""
        backoff = self._video.camera_retry_initial_s
        retries = 0
        while not self._stop:
            try:
                if self._start_pipeline():
                    self._monitor_pipeline()
            except (OSError, subprocess.SubprocessError) as exc:
                logger.error("Video pipeline error: %s", exc)
            self._kill_processes()
            if self._video.camera_max_retries > 0:
                retries += 1
                if retries >= self._video.camera_max_retries:
                    logger.error("Max camera retries exceeded, exiting")
                    break
            time.sleep(min(backoff, self._video.camera_retry_max_s))
            backoff = min(backoff * 2, self._video.camera_retry_max_s)

    def stop(self) -> None:
        """Stop streaming."""
        self._stop = True
        self._kill_processes()

    def _touch_video_heartbeat(self) -> None:
        try:
            self._video_hb_path.parent.mkdir(parents=True, exist_ok=True)
            self._video_hb_path.touch()
        except OSError as exc:
            logger.debug("Video heartbeat touch failed: %s", exc)

    def _update_osd_file(self) -> None:
        if self._video.osd_enabled:
            write_osd_text_file(self._video.osd_state_file, str(self._osd_text_path))

    def _should_restart_for_thermal(self) -> bool:
        """Return True if thermal bitrate level changed and debounce elapsed."""
        _, level = read_thermal_bitrate(
            self._video.thermal_alert_file,
            self._video.thermal_bitrate_steps,
            self._video.bitrate_bps,
        )
        if level == self._last_bitrate_level:
            return False
        now = time.monotonic()
        if now - self._last_thermal_restart < self._video.thermal_restart_debounce_s:
            return False
        self._last_bitrate_level = level
        self._last_thermal_restart = now
        logger.info("Thermal bitrate level changed to %d, restarting pipeline", level)
        return True

    def _start_pipeline(self) -> bool:
        self._camera_binary = detect_camera_cli(self._video.camera_cli)
        if not self._camera_binary:
            logger.error("No camera CLI found (rpicam-vid / libcamera-vid)")
            return False
        if not probe_camera(self._camera_binary):
            logger.error("CSI camera not detected")
            return False

        bitrate, level = read_thermal_bitrate(
            self._video.thermal_alert_file,
            self._video.thermal_bitrate_steps,
            self._video.bitrate_bps,
        )
        self._last_bitrate_level = level

        self._update_osd_file()
        use_osd = self._video.osd_enabled and self._osd_text_path.is_file()
        use_copy = self._video.use_inline_h264 and not use_osd

        cam_cmd = build_camera_cmd(
            self._camera_binary,
            self._video.width,
            self._video.height,
            self._video.framerate,
            self._video.use_inline_h264,
        )
        ffmpeg_cmd = build_ffmpeg_cmd(
            self._video.ground_station_ip,
            self._video.ground_station_port,
            bitrate,
            use_copy,
            use_osd,
            str(self._osd_text_path) if use_osd else "",
        )

        logger.info("Starting camera: %s", " ".join(cam_cmd))
        logger.info("Starting FFmpeg (bitrate=%d, copy=%s)", bitrate, use_copy)

        self._camera_proc = subprocess.Popen(
            cam_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        self._ffmpeg_proc = subprocess.Popen(
            ffmpeg_cmd,
            stdin=self._camera_proc.stdout,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        if self._camera_proc.stdout:
            self._camera_proc.stdout.close()
        self._touch_video_heartbeat()
        return True

    def _monitor_pipeline(self) -> None:
        """Wait until a child process exits or thermal restart requested."""
        last_osd_update = 0.0
        while not self._stop:
            if self._should_restart_for_thermal():
                break
            now = time.monotonic()
            if self._video.osd_enabled and now - last_osd_update >= 1.0:
                self._update_osd_file()
                last_osd_update = now
            cam_rc = self._camera_proc.poll() if self._camera_proc else 0
            ff_rc = self._ffmpeg_proc.poll() if self._ffmpeg_proc else 0
            if cam_rc is not None:
                logger.warning("Camera process exited with code %s", cam_rc)
                break
            if ff_rc is not None:
                logger.warning("FFmpeg process exited with code %s", ff_rc)
                break
            self._touch_video_heartbeat()
            time.sleep(1.0)

    def _kill_processes(self) -> None:
        """Terminate camera and FFmpeg process groups."""
        for proc in (self._ffmpeg_proc, self._camera_proc):
            if proc and proc.poll() is None:
                try:
                    if hasattr(os, "killpg") and proc.pid:
                        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    else:
                        proc.terminate()
                    proc.wait(timeout=5)
                except (ProcessLookupError, subprocess.TimeoutExpired, OSError):
                    try:
                        proc.kill()
                    except OSError:
                        pass
        self._camera_proc = None
        self._ffmpeg_proc = None


def setup_logging() -> None:
    """Configure module logging."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )


def main() -> int:
    """Entry point for video-stream systemd service."""
    setup_logging()
    try:
        config = load_config()
    except Exception as exc:
        logger.error("Failed to load config: %s", exc)
        return 1

    runtime = Path(config.system.runtime_dir)
    runtime.mkdir(parents=True, exist_ok=True)

    streamer = VideoStreamer(config)

    def handle_signal(signum: int, _frame: object) -> None:
        logger.info("Received signal %s, stopping video stream", signum)
        streamer.stop()

    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    try:
        streamer.run()
    except KeyboardInterrupt:
        streamer.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
