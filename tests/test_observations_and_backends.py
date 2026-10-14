import io
import sys
import threading
import time
from types import SimpleNamespace

import pytest
from PIL import Image


def put_feedback(cache, arm, *, gripper=2.5, received_at=None):
    cache.update_arm(
        arm,
        (0.1, 0.2, 0.3, 0, 0, 0),
        (1, 2, 3, 4, 5, 6, gripper),
        (7, 8, 9, 10, 11, 12, 13),
        (14, 15, 16, 17, 18, 19, 20),
        stamp=123.25,
        received_at=received_at,
    )


def test_observations_keep_deploydev_layout_and_raw_gripper():
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    put_feedback(cache, "left")
    put_feedback(cache, "right", gripper=3.5)
    state = cache.state(0.5)
    assert state["state_ready"]
    assert state["qpos"] == [1, 2, 3, 4, 5, 6, 2.5, 1, 2, 3, 4, 5, 6, 3.5]
    assert state["eef"] == [0.1, 0.2, 0.3, 0, 0, 0, 2.5, 0.1, 0.2, 0.3, 0, 0, 0, 3.5]
    assert state["qvel"] == [7, 8, 9, 10, 11, 12, 13] * 2
    assert state["effort"] == [14, 15, 16, 17, 18, 19, 20] * 2
    assert state["arms"]["left"]["timestamp"] == 123.25
    assert cache.state(0.5)["seq"] == state["seq"]  # Reading never consumes feedback.


def test_stale_or_missing_feedback_cannot_be_used_for_control():
    from ee_control_mcp.observations import ObservationCache, StateUnavailable

    cache = ObservationCache()
    put_feedback(cache, "left", received_at=time.monotonic() - 2)
    assert not cache.state(0.5)["state_ready"]
    with pytest.raises(StateUnavailable, match="stale"):
        cache.fresh_arm("left", 0.5)
    with pytest.raises(StateUnavailable, match="missing"):
        cache.fresh_arm("right", 0.5)


def test_new_state_wait_wakes_on_feedback_without_consuming_it():
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    result = []
    thread = threading.Thread(target=lambda: result.append(cache.wait_for_new(0, 1)))
    thread.start()
    put_feedback(cache, "left")
    thread.join(1)
    assert result == [True]
    assert cache.state(0.5)["arms"]["left"]["end_pos"] == [0.1, 0.2, 0.3, 0, 0, 0]
    assert cache.wait_for_new(cache.seq, 0) is False


def test_arrival_history_is_non_consuming_and_keeps_intervening_feedback():
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    put_feedback(cache, "left")
    since = cache.seq
    put_feedback(cache, "left", gripper=0.2)
    put_feedback(cache, "left", gripper=0)
    samples = cache.feedback_since("left", since)
    assert [sample.joint_pos[-1] for sample in samples] == [0.2, 0]
    assert cache.feedback_since("left", since) == samples
    assert cache.feedback_since("left", samples[-1].seq) == []


def test_arrival_history_overflow_is_reported_per_arm():
    from ee_control_mcp.observations import ObservationCache, StateUnavailable

    cache = ObservationCache()
    put_feedback(cache, "left")
    watermark = cache.seq
    for _ in range(300):
        put_feedback(cache, "right")
    assert cache.feedback_since("left", watermark) == []
    for _ in range(300):
        put_feedback(cache, "left")
    with pytest.raises(StateUnavailable, match="history.*overflow"):
        cache.feedback_since("left", watermark)
    assert cache.state(0.5)["state_ready"]


def test_jpeg_color_and_camera_metadata_survive_reencoding():
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    buffer = io.BytesIO()
    Image.new("RGB", (16, 12), (255, 0, 0)).save(buffer, "PNG")
    cache.update_camera("head", buffer.getvalue(), stamp=77.5)
    metadata, frames = cache.images(["head", "right_wrist"], max_age_s=1)
    assert metadata["cameras_missing"] == ["right_wrist"]
    assert metadata["images"][0]["timestamp"] == 77.5
    assert len(frames) == 1
    image = Image.open(io.BytesIO(frames[0].data))
    red, green, blue = image.getpixel((0, 0))
    assert red > 240 and green < 10 and blue < 10
    assert image.format == "JPEG"
    assert len(cache.images(["head"], max_age_s=1)[1]) == 1


def test_stale_and_corrupt_cameras_are_reported_individually():
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    cache.update_camera("head", b"bad JPEG", stamp=1)
    cache.update_camera("left_wrist", b"old JPEG", received_at=time.monotonic() - 2)
    metadata, frames = cache.images(["head", "left_wrist"], max_age_s=1)
    assert frames == []
    assert metadata["cameras_stale"] == ["left_wrist"]
    assert "head" in metadata["camera_errors"]
    with pytest.raises(ValueError, match="camera"):
        cache.images(["unknown"], max_age_s=1)


@pytest.mark.parametrize(
    "field,value",
    [
        ("end_pos", [0] * 7),
        ("joint_pos", [0] * 6),
        ("joint_cur", [float("nan")] * 7),
    ],
)
def test_malformed_ros_feedback_does_not_replace_valid_state(field, value):
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    put_feedback(cache, "left")
    initial_seq = cache.seq
    payload = dict(end_pos=[0] * 6, joint_pos=[0] * 7, joint_vel=[0] * 7, joint_cur=[0] * 7)
    payload[field] = value
    with pytest.raises(ValueError):
        cache.update_arm("left", **payload)
    assert cache.seq == initial_seq
    assert cache.state(0.5)["arms"]["left"]["gripper"] == 2.5


def fake_ros(monkeypatch):
    monkeypatch.setenv("ROS_DISTRO", "humble")
    nodes = []

    class Context:
        running = False

        def ok(self):
            return self.running

        def try_shutdown(self):
            self.running = False

    class RobotCmd:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None)
            self.end_pos, self.joint_pos = [0.0] * 6, [0.0] * 6
            self.gripper, self.mode = 0.0, 0

    class Publisher:
        def __init__(self, topic):
            self.topic, self.messages = topic, []

        def publish(self, message):
            self.messages.append(message)

        def get_subscription_count(self):
            return 1

    class Node:
        def __init__(self, name, **kwargs):
            self.publishers, self.subscriptions = [], []
            nodes.append(self)

        def create_publisher(self, kind, topic, qos):
            publisher = Publisher(topic)
            self.publishers.append(publisher)
            return publisher

        def create_subscription(self, kind, topic, callback, qos):
            self.subscriptions.append((kind, topic, callback))
            return callback

        def get_clock(self):
            return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: "new stamp"))

        def destroy_node(self):
            pass

    class Executor:
        def __init__(self, **kwargs):
            pass

        def add_node(self, node):
            pass

        def spin_once(self, timeout_sec):
            time.sleep(0.001)

        def shutdown(self, timeout_sec=None):
            pass

    def init(*, args=None, context=None):
        context.running = True

    modules = {
        "rclpy": SimpleNamespace(init=init),
        "rclpy.context": SimpleNamespace(Context=Context),
        "rclpy.node": SimpleNamespace(Node=Node),
        "rclpy.executors": SimpleNamespace(SingleThreadedExecutor=Executor),
        "rclpy.qos": SimpleNamespace(qos_profile_sensor_data="sensor qos"),
        "arx5_arm_msg.msg": SimpleNamespace(
            RobotCmd=RobotCmd, RobotStatus=type("RobotStatus", (), {})
        ),
        "sensor_msgs.msg": SimpleNamespace(CompressedImage=type("CompressedImage", (), {})),
    }
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    return nodes


def test_ros_publication_uses_independent_robotcmd_messages(monkeypatch):
    from ee_control_mcp.backends import RosBackend
    from ee_control_mcp.config import Config
    from ee_control_mcp.observations import ObservationCache
    from ee_control_mcp.types import ArmTarget

    nodes = fake_ros(monkeypatch)
    backend = RosBackend(Config(), ObservationCache())
    backend.start()
    try:
        backend.publish("left", ArmTarget(4, (0.1, 0.03, 0.05, 0, 0, 0), 1))
        backend.publish("right", ArmTarget(4, (0.05, 0, 0.1, 0, 0, 0), 1))
        backend.publish("left", ArmTarget(5, (0,) * 6, 0))
        left, right = nodes[0].publishers
        assert left.topic == "/arm_master_l_cmd" and right.topic == "/arm_master_r_cmd"
        assert left.messages[0].end_pos == [0.1, 0.03, 0.05, 0, 0, 0]
        assert right.messages[0].end_pos == [0.05, 0, 0.1, 0, 0, 0]
        assert left.messages[1].mode == 5 and left.messages[1].joint_pos == [0] * 6
        assert left.messages[0] is not right.messages[0] is not left.messages[1]
    finally:
        backend.close()


def test_readonly_ros_backend_creates_no_command_publishers(monkeypatch):
    from ee_control_mcp.backends import RosBackend
    from ee_control_mcp.config import Config
    from ee_control_mcp.observations import ObservationCache
    from ee_control_mcp.types import ArmTarget

    nodes = fake_ros(monkeypatch)
    backend = RosBackend(Config(), ObservationCache(), actuation_enabled=False)
    backend.start()
    try:
        assert nodes[0].publishers == []
        assert len(nodes[0].subscriptions) == 5
        with pytest.raises(RuntimeError, match="disabled"):
            backend.publish("left", ArmTarget(5, (0,) * 6, 0))
    finally:
        backend.close()


@pytest.mark.parametrize("distro", ["jazzy", ""])
def test_ros_distro_mismatch_is_rejected_before_creating_any_ros_node(monkeypatch, distro):
    from ee_control_mcp.backends import RosBackend
    from ee_control_mcp.config import Config
    from ee_control_mcp.observations import ObservationCache

    nodes = fake_ros(monkeypatch)
    monkeypatch.setenv("ROS_DISTRO", distro)
    backend = RosBackend(Config(), ObservationCache(), actuation_enabled=False)
    try:
        with pytest.raises(RuntimeError, match="humble"):
            backend.start()
        assert nodes == []
        assert backend._context is None
    finally:
        backend.close()


def test_ros_expected_distro_can_be_configured_for_a_different_robot(tmp_path, monkeypatch):
    from ee_control_mcp.backends import RosBackend
    from ee_control_mcp.config import load_config
    from ee_control_mcp.observations import ObservationCache

    config_file = tmp_path / "jazzy.yaml"
    config_file.write_text("ros: {expected_distro: jazzy}\n")
    config = load_config(config_file)
    nodes = fake_ros(monkeypatch)
    monkeypatch.setenv("ROS_DISTRO", "jazzy")
    backend = RosBackend(config, ObservationCache(), actuation_enabled=False)
    backend.start()
    try:
        assert len(nodes) == 1 and nodes[0].publishers == []
    finally:
        backend.close()


def test_robotics52_master_status_messages_produce_ready_dual_arm_state(monkeypatch):
    from pathlib import Path

    from ee_control_mcp.backends import RosBackend
    from ee_control_mcp.config import load_config
    from ee_control_mcp.observations import ObservationCache

    nodes = fake_ros(monkeypatch)
    root = Path(__file__).resolve().parents[1]
    cache = ObservationCache()
    backend = RosBackend(
        load_config(root / "config/robotics52.yaml"), cache, actuation_enabled=False
    )
    backend.start()
    try:
        # Real driver graph: only master status topics have publishers.
        messages = {
            "/arm_master_l_status": SimpleNamespace(
                end_pos=[0.1, 0.03, 0.05, 0, 0, 0],
                joint_pos=[0.01] * 6 + [-0.04],
                joint_vel=[0.0] * 7,
                joint_cur=[0.0] * 7,
            ),
            "/arm_master_r_status": SimpleNamespace(
                end_pos=[0.05, 0, 0.1, 0, 0, 0],
                joint_pos=[0.02] * 6 + [-0.03],
                joint_vel=[0.0] * 7,
                joint_cur=[0.0] * 7,
            ),
        }
        for _, topic, callback in nodes[0].subscriptions:
            if topic in messages:
                callback(messages[topic])
        state = cache.state(0.5)
        assert state["state_ready"]
        assert state["qpos"] == [0.01] * 6 + [-0.04] + [0.02] * 6 + [-0.03]
        assert state["eef"] == [0.1, 0.03, 0.05, 0, 0, 0, -0.04, 0.05, 0, 0.1, 0, 0, 0, -0.03]
    finally:
        backend.close()


def test_simulation_is_usable_without_ros_and_provides_camera_frames():
    from ee_control_mcp.backends import SimBackend
    from ee_control_mcp.config import Config
    from ee_control_mcp.observations import ObservationCache

    cache = ObservationCache()
    backend = SimBackend(Config(), cache)
    backend.start()
    try:
        assert cache.state(0.5)["state_ready"]
        metadata, frames = cache.images(["head", "left_wrist", "right_wrist"], max_age_s=1)
        assert len(frames) == 3
        assert not metadata["cameras_missing"]
    finally:
        backend.close()
