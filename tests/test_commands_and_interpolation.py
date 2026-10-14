import math

import numpy as np
import pytest
from scipy.spatial.transform import Rotation


def test_parallel_steps_and_speed_are_preserved():
    from ee_control_mcp.commands import parse_commands

    steps = parse_commands(
        [
            ["left:up:0.05@slow", "right:down:0.03@fast"],
            "both:home@medium",
        ]
    )
    assert len(steps) == 2
    assert steps[0].clauses[0].arms == ("left",)
    assert steps[0].clauses[0].value == 0.05
    assert steps[0].clauses[1].speed == "fast"
    assert steps[1].clauses[0].arms == ("left", "right")
    assert steps[1].clauses[0].motions == ("home",)


@pytest.mark.parametrize(
    "commands",
    [
        [],
        "",
        "middle:up",
        "left:fly",
        "left:up:-1",
        "left:up:nan",
        "left:up:inf",
        "left:up@turbo",
        "left:home&up",
        "left:hold&up",
        "left:hold | right:hold",
        "left:up | both:down",
        "left:open&close",
        "left:open:nan",
        "left:close:inf",
        "left:open:-inf",
        "left:up&open:-1.7",
        ["left:up", 42],
        [["left:up | right:up"]],
        "left:home:0.1",
        "left:open@slow",
        True,
    ],
)
def test_invalid_batch_is_rejected(commands):
    from ee_control_mcp.commands import parse_commands

    with pytest.raises(ValueError):
        parse_commands(commands)


def test_composed_motion_can_include_gripper():
    from ee_control_mcp.commands import parse_commands

    clause = parse_commands("left:close&up")[0].clauses[0]
    assert clause.motions == ("close", "up")
    assert clause.value is None


@pytest.mark.parametrize("speed, expected_steps", [("slow", 11), ("medium", 6), ("fast", 3)])
def test_uniform_translation_has_equal_increments_and_exact_endpoint(speed, expected_steps):
    from ee_control_mcp.config import Config
    from ee_control_mcp.interpolation import UniformTrajectory
    from ee_control_mcp.types import ArmTarget

    start = (0.0,) * 6
    target = ArmTarget(4, (0.011, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, speed)
    path = UniformTrajectory(start, target, Config().control.speeds[speed])
    points = np.asarray([path.sample(tick).values for tick in range(path.steps + 1)])
    assert path.steps == expected_steps
    assert np.diff(points[:, 0]) == pytest.approx(np.full(expected_steps, 0.011 / expected_steps))
    assert max(np.linalg.norm(delta) for delta in np.diff(points[:, :3], axis=0)) <= (
        Config().control.speeds[speed].translation_m + 1e-12
    )
    assert path.sample(path.steps).values == target.values


def test_rotation_uses_shortest_uniform_arc_across_rpy_wrap():
    from ee_control_mcp.config import Config
    from ee_control_mcp.interpolation import UniformTrajectory
    from ee_control_mcp.types import ArmTarget

    start = (0, 0, 0, 0, 0, math.pi - 0.01)
    target = ArmTarget(4, (0, 0, 0, 0, 0, -math.pi + 0.01), 0, "slow")
    path = UniformTrajectory(start, target, Config().control.speeds["slow"])
    rotations = [
        Rotation.from_euler("xyz", path.sample(i).values[3:]) for i in range(path.steps + 1)
    ]
    gaps = [(a.inv() * b).magnitude() for a, b in zip(rotations, rotations[1:], strict=False)]
    assert path.steps == 4
    assert gaps == pytest.approx([0.005] * 4)
    assert path.sample(path.steps).values == target.values


def test_joints_interpolate_six_axes_without_touching_gripper_dimension():
    from ee_control_mcp.config import Config
    from ee_control_mcp.interpolation import UniformTrajectory
    from ee_control_mcp.types import ArmTarget

    target = ArmTarget(5, (0.02, -0.01, 0, 0, 0, 0), 0.4, "medium")
    path = UniformTrajectory((0,) * 6, target, Config().control.speeds["medium"])
    assert path.steps == 2
    assert path.sample(1).values == pytest.approx((0.01, -0.005, 0, 0, 0, 0))
    assert path.sample(1).gripper == 0.4


def test_config_overrides_topics_granularity_and_direction(tmp_path):
    from ee_control_mcp.config import load_config

    path = tmp_path / "robot.yaml"
    path.write_text("""ros:
  command_topics:
    left: /test/left_cmd
control:
  speeds:
    slow:
      translation_m: 0.0005
  motions:
    left: [0, -0.05, 0, 0, 0, 0]
""")
    config = load_config(path)
    assert config.ros.command_topics == {"left": "/test/left_cmd", "right": "/arm_master_r_cmd"}
    assert config.control.speeds["slow"].translation_m == 0.0005
    assert config.control.speeds["slow"].angle_rad == 0.005
    assert config.control.motions["left"][1] == -0.05


@pytest.mark.parametrize(
    "yaml",
    [
        "control: {publish_rate_hz: 0}",
        "control: {speeds: {slow: {translation_m: .nan}}}",
        "control: {default_speed: turbo}",
        "ros: {feedback_topics: {middle: /status}}",
        "control: {motions: {home: [0, 0, 0, 0, 0, 0]}}",
        "http: {port: 70000}",
        "control: {publish_raet_hz: 30}",
    ],
)
def test_config_fails_early_on_invalid_settings(tmp_path, yaml):
    from ee_control_mcp.config import load_config

    path = tmp_path / "bad.yaml"
    path.write_text(yaml)
    with pytest.raises(ValueError):
        load_config(path)
