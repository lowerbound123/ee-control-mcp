from __future__ import annotations

from dataclasses import asdict, dataclass, field
from pathlib import Path

import yaml

from .commands import MOTION_AXES
from .types import SPEEDS, finite_number, finite_vector


def default_motions(position: float = 0.05, rotation: float = 0.1) -> dict:
    result = {}
    for name, (axis, sign) in MOTION_AXES.items():
        delta = [0.0] * 6
        delta[axis] = sign * (position if axis < 3 else rotation)
        result[name] = tuple(delta)
    return result


@dataclass(frozen=True)
class Granularity:
    translation_m: float
    angle_rad: float


@dataclass(frozen=True)
class HttpConfig:
    host: str = "127.0.0.1"
    port: int = 3000
    token_env: str = "EE_CONTROL_MCP_TOKEN"
    allowed_hosts: tuple[str, ...] = ("localhost:*", "127.0.0.1:*", "[::1]:*")
    allowed_origins: tuple[str, ...] = ("http://localhost:*", "http://127.0.0.1:*")


@dataclass(frozen=True)
class RosConfig:
    node_name: str = "ee_control_mcp"
    # Must match the robot's runtime, not merely the locally installed ROS.
    expected_distro: str = "humble"
    command_topics: dict = field(
        default_factory=lambda: {
            "left": "/arm_master_l_cmd",
            "right": "/arm_master_r_cmd",
        }
    )
    feedback_topics: dict = field(
        default_factory=lambda: {
            "left": "/arm_slave_l_status",
            "right": "/arm_slave_r_status",
        }
    )
    camera_topics: dict = field(
        default_factory=lambda: {
            "head": "/camera/camera_h/color/image_rect_raw/compressed",
            "left_wrist": "/camera/camera_l/color/image_rect_raw/compressed",
            "right_wrist": "/camera/camera_r/color/image_rect_raw/compressed",
        }
    )


@dataclass(frozen=True)
class ControlConfig:
    publish_rate_hz: float = 30.0
    default_speed: str = "slow"
    speeds: dict[str, Granularity] = field(
        default_factory=lambda: {
            "slow": Granularity(0.001, 0.005),
            "medium": Granularity(0.002, 0.010),
            "fast": Granularity(0.005, 0.025),
        }
    )
    delta_pos: float = 0.05
    delta_rot: float = 0.1
    motions: dict = field(default_factory=default_motions)
    gripper_open: float = 1.0
    gripper_close: float = 0.0
    # Feedback stays raw in state. These calibrate it into RobotCmd units only
    # for preserving the gripper and checking arrival; identity is the default.
    gripper_feedback_scale: float = 1.0
    gripper_feedback_offset: float = 0.0
    position_tolerance_m: float = 0.005
    rotation_tolerance_rad: float = 0.05
    joint_tolerance_rad: float = 0.02
    gripper_tolerance: float = 0.05
    settle_observations: int = 2
    step_timeout_s: float = 60.0
    max_pending_steps: int = 256
    completed_history_size: int = 128


@dataclass(frozen=True)
class ObservationConfig:
    feedback_max_age_s: float = 0.5
    image_max_age_s: float = 1.0
    image_quality: int = 85
    image_max_width: int = 1024
    image_max_height: int = 768


@dataclass(frozen=True)
class Config:
    http: HttpConfig = field(default_factory=HttpConfig)
    ros: RosConfig = field(default_factory=RosConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    observation: ObservationConfig = field(default_factory=ObservationConfig)

    def as_dict(self) -> dict:
        return asdict(self)


def _merge(defaults: dict, override: dict, prefix: str = "") -> dict:
    if not isinstance(override, dict):
        raise ValueError(f"{prefix or 'config'} must be an object")
    result = dict(defaults)
    for key, value in override.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if key not in defaults:
            raise ValueError(f"unknown setting {name}")
        result[key] = (
            _merge(defaults[key], value, name) if isinstance(defaults[key], dict) else value
        )
    return result


def _positive_fields(mapping: dict, names: tuple[str, ...]) -> None:
    for name in names:
        mapping[name] = finite_number(mapping[name], name)
        if mapping[name] <= 0:
            raise ValueError(f"{name} must be positive")


def _integer(mapping: dict, name: str, minimum: int, maximum: int) -> None:
    value = mapping[name]
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= maximum:
        raise ValueError(f"{name} must be an integer in [{minimum}, {maximum}]")


def load_config(path: str | Path | None = None) -> Config:
    try:
        raw = {} if path is None else yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:
        raise ValueError(f"invalid YAML in {path}: {exc}") from exc
    raw = {} if raw is None else raw
    data = _merge(Config().as_dict(), raw)
    http, ros, control, observation = (
        data[name]
        for name in (
            "http",
            "ros",
            "control",
            "observation",
        )
    )
    _integer(http, "port", 1, 65535)
    for name in ("host", "token_env"):
        if not isinstance(http[name], str) or not http[name].strip():
            raise ValueError(f"http.{name} must be a nonempty string")
    for name in ("allowed_hosts", "allowed_origins"):
        if not isinstance(http[name], (list, tuple)) or not all(
            isinstance(item, str) and item for item in http[name]
        ):
            raise ValueError(f"http.{name} must be a list of strings")
        http[name] = tuple(http[name])
    if not isinstance(ros["node_name"], str) or not ros["node_name"]:
        raise ValueError("ros.node_name must be a nonempty string")
    if not isinstance(ros["expected_distro"], str) or not ros["expected_distro"].isidentifier():
        raise ValueError("ros.expected_distro must be a ROS distribution name")
    for name in ("command_topics", "feedback_topics", "camera_topics"):
        if not all(
            isinstance(topic, str) and topic.startswith("/") for topic in ros[name].values()
        ):
            raise ValueError(f"ros.{name} must contain absolute ROS topic names")
    _positive_fields(
        control,
        (
            "publish_rate_hz",
            "delta_pos",
            "delta_rot",
            "position_tolerance_m",
            "rotation_tolerance_rad",
            "joint_tolerance_rad",
            "gripper_tolerance",
            "step_timeout_s",
        ),
    )
    for name in (
        "gripper_open",
        "gripper_close",
        "gripper_feedback_scale",
        "gripper_feedback_offset",
    ):
        control[name] = finite_number(control[name], name)
    if control["gripper_feedback_scale"] == 0:
        raise ValueError("gripper_feedback_scale cannot be zero")
    if control["default_speed"] not in SPEEDS:
        raise ValueError(f"default_speed must be one of {SPEEDS}")
    for name in ("settle_observations", "max_pending_steps", "completed_history_size"):
        _integer(control, name, 1, 100000)
    for speed, values in control["speeds"].items():
        _positive_fields(values, ("translation_m", "angle_rad"))
        control["speeds"][speed] = Granularity(**values)
    # A scalar default change affects every motion not explicitly overridden.
    deltas = default_motions(control["delta_pos"], control["delta_rot"])
    for name, value in raw.get("control", {}).get("motions", {}).items():
        delta = finite_vector(value, 6, f"motion {name}")
        if not any(delta):
            raise ValueError(f"motion {name} cannot be all zero")
        deltas[name] = delta
    control["motions"] = deltas
    _positive_fields(observation, ("feedback_max_age_s", "image_max_age_s"))
    _integer(observation, "image_quality", 1, 100)
    for name in ("image_max_width", "image_max_height"):
        _integer(observation, name, 1, 8192)
    return Config(
        HttpConfig(**http),
        RosConfig(**ros),
        ControlConfig(**control),
        ObservationConfig(**observation),
    )
