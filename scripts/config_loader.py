"""YAML configuration loader with validation for the companion computer."""

from __future__ import annotations

import ipaddress
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


def _project_root() -> Path:
    """Resolve project root from env or script location."""
    env = os.environ.get("RPI_COMPANION_ROOT")
    if env:
        return Path(env).resolve()
    return Path(__file__).resolve().parent.parent


@dataclass
class MavlinkConfig:
    """MAVLink connection configuration."""

    serial_ports: list[str]
    baud_rate: int
    target_system: int
    target_component: int
    source_system: int
    source_component: int
    heartbeat_timeout_s: float
    connection_timeout_s: float
    reconnect_initial_s: float
    reconnect_max_s: float
    outbound_queue_depth: int
    servo_max_rate_hz: float
    critical_message_ids: list[int]
    ardupilot_copter_modes: dict[int, str]


@dataclass
class VideoConfig:
    """Video streaming configuration."""

    ground_station_ip: str
    ground_station_port: int
    width: int
    height: int
    framerate: int
    bitrate_bps: int
    thermal_bitrate_steps: list[int]
    camera_cli: str
    osd_enabled: bool
    osd_state_file: str
    osd_text_file: str
    thermal_alert_file: str
    thermal_restart_debounce_s: float
    camera_retry_initial_s: float
    camera_retry_max_s: float
    camera_max_retries: int
    ffmpeg_retry_initial_s: float
    ffmpeg_retry_max_s: float
    use_inline_h264: bool


@dataclass
class PidGains:
    """PID controller gains."""

    kp: float
    ki: float
    kd: float


@dataclass
class GimbalConfig:
    """Gimbal stabilization configuration."""

    servo_channels: dict[str, int]
    pwm_min_us: int
    pwm_max_us: int
    home_pwm: dict[str, int]
    update_hz: float
    gimbal_log_hz: float
    attitude_lpf_cutoff_hz: float
    limits_deg: dict[str, float]
    pid: dict[str, PidGains]
    pid_output_limit_deg: float
    attitude_timeout_s: float
    manual_yaw_deg: float
    deg_per_us: float


@dataclass
class TriggerConfig:
    """Trigger safety configuration."""

    servo_channel: int
    pwm_safe_us: int
    pwm_actuated_us: int
    cooldown_s: float
    command_id: int
    min_altitude_m: float
    position_timeout_s: float
    allowed_modes: list[str]
    blocked_modes: list[str]


@dataclass
class WatchdogConfig:
    """Process watchdog configuration."""

    main_stale_s: float
    video_stale_s: float
    main_heartbeat_file: str
    video_heartbeat_file: str
    restart_cooldown_s: float


@dataclass
class SystemConfig:
    """System-wide configuration."""

    runtime_dir: str
    log_dir: str
    log_rotation_max_bytes: int
    tlog_flush_interval: int
    health_interval_s: float
    thermal_warning_c: float
    thermal_critical_c: float
    thermal_sysfs: str
    power_supply_sysfs: str
    low_battery_percent: int
    shutdown_on_low_battery: bool
    thermal_statustext_interval_s: float
    trigger: TriggerConfig
    watchdog: WatchdogConfig
    osd_update_hz: float
    project_root: str


@dataclass
class CompanionConfig:
    """Aggregate configuration for all modules."""

    root: Path
    mavlink: MavlinkConfig
    video: VideoConfig
    gimbal: GimbalConfig
    system: SystemConfig


class ConfigError(Exception):
    """Raised when configuration validation fails."""


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise ConfigError(f"Config file not found: {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ConfigError(f"Invalid YAML structure in {path}")
    return data


def _require(data: dict[str, Any], key: str, ctx: str) -> Any:
    if key not in data:
        raise ConfigError(f"Missing required key '{key}' in {ctx}")
    return data[key]


def _validate_ip(ip: str) -> None:
    try:
        ipaddress.ip_address(ip)
    except ValueError as exc:
        raise ConfigError(f"Invalid ground_station_ip: {ip}") from exc


def _validate_gimbal(cfg: GimbalConfig) -> None:
    required = {"roll", "pitch", "yaw"}
    if not required.issubset(cfg.servo_channels.keys()):
        raise ConfigError(f"servo_channels must include {required}")
    if cfg.pwm_min_us >= cfg.pwm_max_us:
        raise ConfigError("pwm_min_us must be less than pwm_max_us")
    if cfg.update_hz <= 0 or cfg.gimbal_log_hz <= 0:
        raise ConfigError("update_hz and gimbal_log_hz must be positive")
    for axis, gains in cfg.pid.items():
        if gains.kp < 0 or gains.ki < 0 or gains.kd < 0:
            raise ConfigError(f"PID gains for {axis} must be non-negative")


def load_mavlink_config(root: Path) -> MavlinkConfig:
    """Load and validate MAVLink configuration."""
    data = _load_yaml(root / "config" / "mavlink_config.yaml")
    modes_raw = _require(data, "ardupilot_copter_modes", "mavlink_config.yaml")
    modes = {int(k): str(v) for k, v in modes_raw.items()}
    baud = int(_require(data, "baud_rate", "mavlink"))
    if baud <= 0:
        raise ConfigError("baud_rate must be positive")
    return MavlinkConfig(
        serial_ports=[str(p) for p in _require(data, "serial_ports", "mavlink")],
        baud_rate=baud,
        target_system=int(_require(data, "target_system", "mavlink")),
        target_component=int(_require(data, "target_component", "mavlink")),
        source_system=int(_require(data, "source_system", "mavlink")),
        source_component=int(_require(data, "source_component", "mavlink")),
        heartbeat_timeout_s=float(_require(data, "heartbeat_timeout_s", "mavlink")),
        connection_timeout_s=float(_require(data, "connection_timeout_s", "mavlink")),
        reconnect_initial_s=float(_require(data, "reconnect_initial_s", "mavlink")),
        reconnect_max_s=float(_require(data, "reconnect_max_s", "mavlink")),
        outbound_queue_depth=int(_require(data, "outbound_queue_depth", "mavlink")),
        servo_max_rate_hz=float(_require(data, "servo_max_rate_hz", "mavlink")),
        critical_message_ids=[int(x) for x in _require(data, "critical_message_ids", "mavlink")],
        ardupilot_copter_modes=modes,
    )


def load_video_config(root: Path) -> VideoConfig:
    """Load and validate video configuration."""
    data = _load_yaml(root / "config" / "video_config.yaml")
    cli = str(_require(data, "camera_cli", "video")).lower()
    if cli not in ("auto", "rpicam", "libcamera"):
        raise ConfigError(f"Invalid camera_cli: {cli}")
    ip = str(_require(data, "ground_station_ip", "video"))
    _validate_ip(ip)
    port = int(_require(data, "ground_station_port", "video"))
    if not (1 <= port <= 65535):
        raise ConfigError(f"Invalid ground_station_port: {port}")
    return VideoConfig(
        ground_station_ip=ip,
        ground_station_port=port,
        width=int(_require(data, "width", "video")),
        height=int(_require(data, "height", "video")),
        framerate=int(_require(data, "framerate", "video")),
        bitrate_bps=int(_require(data, "bitrate_bps", "video")),
        thermal_bitrate_steps=[int(x) for x in _require(data, "thermal_bitrate_steps", "video")],
        camera_cli=cli,
        osd_enabled=bool(_require(data, "osd_enabled", "video")),
        osd_state_file=str(_require(data, "osd_state_file", "video")),
        osd_text_file=str(_require(data, "osd_text_file", "video")),
        thermal_alert_file=str(_require(data, "thermal_alert_file", "video")),
        thermal_restart_debounce_s=float(
            _require(data, "thermal_restart_debounce_s", "video")
        ),
        camera_retry_initial_s=float(_require(data, "camera_retry_initial_s", "video")),
        camera_retry_max_s=float(_require(data, "camera_retry_max_s", "video")),
        camera_max_retries=int(_require(data, "camera_max_retries", "video")),
        ffmpeg_retry_initial_s=float(_require(data, "ffmpeg_retry_initial_s", "video")),
        ffmpeg_retry_max_s=float(_require(data, "ffmpeg_retry_max_s", "video")),
        use_inline_h264=bool(_require(data, "use_inline_h264", "video")),
    )


def load_gimbal_config(root: Path) -> GimbalConfig:
    """Load and validate gimbal configuration."""
    data = _load_yaml(root / "config" / "gimbal_config.yaml")
    pid_raw = _require(data, "pid", "gimbal")
    pid: dict[str, PidGains] = {}
    for axis, gains in pid_raw.items():
        pid[axis] = PidGains(
            kp=float(gains["kp"]),
            ki=float(gains["ki"]),
            kd=float(gains["kd"]),
        )
    cfg = GimbalConfig(
        servo_channels={k: int(v) for k, v in _require(data, "servo_channels", "gimbal").items()},
        pwm_min_us=int(_require(data, "pwm_min_us", "gimbal")),
        pwm_max_us=int(_require(data, "pwm_max_us", "gimbal")),
        home_pwm={k: int(v) for k, v in _require(data, "home_pwm", "gimbal").items()},
        update_hz=float(_require(data, "update_hz", "gimbal")),
        gimbal_log_hz=float(data.get("gimbal_log_hz", 5.0)),
        attitude_lpf_cutoff_hz=float(_require(data, "attitude_lpf_cutoff_hz", "gimbal")),
        limits_deg={k: float(v) for k, v in _require(data, "limits_deg", "gimbal").items()},
        pid=pid,
        pid_output_limit_deg=float(_require(data, "pid_output_limit_deg", "gimbal")),
        attitude_timeout_s=float(_require(data, "attitude_timeout_s", "gimbal")),
        manual_yaw_deg=float(_require(data, "manual_yaw_deg", "gimbal")),
        deg_per_us=float(_require(data, "deg_per_us", "gimbal")),
    )
    _validate_gimbal(cfg)
    return cfg


def load_system_config(root: Path) -> SystemConfig:
    """Load and validate system configuration."""
    data = _load_yaml(root / "config" / "system_config.yaml")
    trig = _require(data, "trigger", "system")
    wd = _require(data, "watchdog", "system")
    proj = str(data.get("project_root", "") or "")
    return SystemConfig(
        runtime_dir=str(_require(data, "runtime_dir", "system")),
        log_dir=str(_require(data, "log_dir", "system")),
        log_rotation_max_bytes=int(_require(data, "log_rotation_max_bytes", "system")),
        tlog_flush_interval=int(data.get("tlog_flush_interval", 100)),
        health_interval_s=float(_require(data, "health_interval_s", "system")),
        thermal_warning_c=float(_require(data, "thermal_warning_c", "system")),
        thermal_critical_c=float(_require(data, "thermal_critical_c", "system")),
        thermal_sysfs=str(_require(data, "thermal_sysfs", "system")),
        power_supply_sysfs=str(_require(data, "power_supply_sysfs", "system")),
        low_battery_percent=int(_require(data, "low_battery_percent", "system")),
        shutdown_on_low_battery=bool(_require(data, "shutdown_on_low_battery", "system")),
        thermal_statustext_interval_s=float(
            data.get("thermal_statustext_interval_s", 60.0)
        ),
        trigger=TriggerConfig(
            servo_channel=int(trig["servo_channel"]),
            pwm_safe_us=int(trig["pwm_safe_us"]),
            pwm_actuated_us=int(trig["pwm_actuated_us"]),
            cooldown_s=float(trig["cooldown_s"]),
            command_id=int(trig["command_id"]),
            min_altitude_m=float(trig["min_altitude_m"]),
            position_timeout_s=float(trig.get("position_timeout_s", 2.0)),
            allowed_modes=[str(m) for m in trig["allowed_modes"]],
            blocked_modes=[str(m) for m in trig["blocked_modes"]],
        ),
        watchdog=WatchdogConfig(
            main_stale_s=float(wd["main_stale_s"]),
            video_stale_s=float(wd["video_stale_s"]),
            main_heartbeat_file=str(wd["main_heartbeat_file"]),
            video_heartbeat_file=str(wd["video_heartbeat_file"]),
            restart_cooldown_s=float(wd.get("restart_cooldown_s", 60.0)),
        ),
        osd_update_hz=float(_require(data, "osd_update_hz", "system")),
        project_root=proj,
    )


def load_config(root: Path | None = None) -> CompanionConfig:
    """Load all configuration files and return validated aggregate."""
    base = root or _project_root()
    if not (base / "config").is_dir():
        raise ConfigError(f"Config directory not found under {base}")
    return CompanionConfig(
        root=base,
        mavlink=load_mavlink_config(base),
        video=load_video_config(base),
        gimbal=load_gimbal_config(base),
        system=load_system_config(base),
    )


def resolve_log_dir(config: CompanionConfig) -> Path:
    """Resolve absolute log directory path."""
    log_path = Path(config.system.log_dir)
    if log_path.is_absolute():
        return log_path
    return config.root / log_path
