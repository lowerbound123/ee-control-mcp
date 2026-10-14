from __future__ import annotations

import io
import logging
import os
import threading
import time
from typing import Protocol

from PIL import Image, ImageDraw

from .config import Config
from .observations import ObservationCache
from .types import ARMS, END_CONTROL, ArmTarget

logger = logging.getLogger(__name__)


class RobotBackend(Protocol):
    name: str
    actuation_enabled: bool

    @property
    def connected(self) -> bool: ...

    def start(self) -> None: ...
    def has_subscriber(self, arm: str) -> bool: ...
    def publish(self, arm: str, target: ArmTarget) -> None: ...
    def close(self) -> None: ...


def _stamp(message) -> float | None:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    return None if stamp is None else float(stamp.sec) + float(stamp.nanosec) * 1e-9


class RosBackend:
    name = "ros"

    def __init__(self, config: Config, cache: ObservationCache, *, actuation_enabled=True):
        self.config, self.cache = config, cache
        self.actuation_enabled = actuation_enabled
        self._context = self._node = self._executor = self._thread = None
        self._message_type = None
        self._publishers: dict = {}
        self._subscriptions: list = []
        self._stop = threading.Event()
        self._error = None

    @property
    def connected(self) -> bool:
        return bool(
            self._context and self._context.ok() and not self._stop.is_set() and self._error is None
        )

    def start(self) -> None:
        expected = self.config.ros.expected_distro
        actual = os.environ.get("ROS_DISTRO") or "<unset>"
        if actual != expected:
            raise RuntimeError(
                f"ROS distro mismatch: robot requires {expected}, current environment is {actual}. "
                f"No ROS participant was created. Use ROS, Python and a message workspace "
                f"built for {expected}."
            )
        try:
            import rclpy
            from arx5_arm_msg.msg import RobotStatus
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CompressedImage
        except ImportError as exc:
            raise RuntimeError(
                "ROS dependencies unavailable. Source ROS 2 and the arx5_arm_msg workspace "
                "using their matching Python interpreter, or use --backend sim."
            ) from exc
        try:
            self._context = Context()
            rclpy.init(args=None, context=self._context)
            self._node = Node(
                self.config.ros.node_name, context=self._context, start_parameter_services=False
            )
            if self.actuation_enabled:
                from arx5_arm_msg.msg import RobotCmd

                self._message_type = RobotCmd
                self._publishers = {
                    arm: self._node.create_publisher(RobotCmd, topic, 10)
                    for arm, topic in self.config.ros.command_topics.items()
                }
            self._subscriptions = [
                self._node.create_subscription(
                    RobotStatus,
                    topic,
                    lambda message, arm=arm: self._feedback(arm, message),
                    qos_profile_sensor_data,
                )
                for arm, topic in self.config.ros.feedback_topics.items()
            ] + [
                self._node.create_subscription(
                    CompressedImage,
                    topic,
                    lambda message, camera=camera: self._camera(camera, message),
                    qos_profile_sensor_data,
                )
                for camera, topic in self.config.ros.camera_topics.items()
            ]
            self._executor = SingleThreadedExecutor(context=self._context)
            self._executor.add_node(self._node)
            self._thread = threading.Thread(target=self._spin, name="ee-control-ros", daemon=True)
            self._thread.start()
        except Exception:
            self.close()
            raise

    def _spin(self) -> None:
        try:
            while not self._stop.is_set() and self._context.ok():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            if not self._stop.is_set():
                self._error = str(exc)
                self.cache.record_error("ros", exc)
                logger.exception("ROS executor stopped")

    def _feedback(self, arm, message) -> None:
        try:
            self.cache.update_arm(
                arm,
                message.end_pos,
                message.joint_pos,
                message.joint_vel,
                message.joint_cur,
                stamp=_stamp(message),
            )
        except (AttributeError, TypeError, ValueError) as exc:
            self.cache.record_error(arm, exc)

    def _camera(self, name, message) -> None:
        try:
            self.cache.update_camera(name, message.data, stamp=_stamp(message))
        except (AttributeError, TypeError, ValueError) as exc:
            self.cache.record_error(name, exc)

    def has_subscriber(self, arm: str) -> bool:
        publisher = self._publishers.get(arm)
        return bool(publisher and publisher.get_subscription_count() > 0)

    def publish(self, arm: str, target: ArmTarget) -> None:
        if not self.actuation_enabled:
            raise RuntimeError("actuation disabled (--no-actuation)")
        if not self.connected:
            raise RuntimeError("ROS backend disconnected")
        message = self._message_type()
        message.header.stamp = self._node.get_clock().now().to_msg()
        message.mode = target.mode
        message.gripper = target.gripper
        if target.mode == END_CONTROL:
            message.end_pos = list(target.values)
        else:
            message.joint_pos = list(target.values)
        self._publishers[arm].publish(message)

    def close(self) -> None:
        self._stop.set()
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=2)
        if self._context is not None:
            self._context.try_shutdown()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._node is not None:
            self._node.destroy_node()
        self._publishers.clear()
        self._subscriptions.clear()
        self._node = self._context = self._executor = self._thread = None


class SimBackend:
    """Immediate target feedback and labeled images; no physics or FK/IK model."""

    name = "sim"

    def __init__(self, config: Config, cache: ObservationCache, *, actuation_enabled=True):
        self.config, self.cache = config, cache
        self.actuation_enabled = actuation_enabled
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._states = {
            "left": {"end_pos": (0.2, 0.1, 0.2, 0, 0, 0), "joint_pos": (0,) * 6, "gripper": 0.0},
            "right": {"end_pos": (0.2, -0.1, 0.2, 0, 0, 0), "joint_pos": (0,) * 6, "gripper": 0.0},
        }
        self._images = {}

    @property
    def connected(self) -> bool:
        return self._thread is not None and self._thread.is_alive() and not self._stop.is_set()

    def start(self) -> None:
        for index, name in enumerate(self.config.ros.camera_topics):
            image = Image.new(
                "RGB", (480, 320), ((30, 60, 100), (100, 50, 30), (30, 100, 50))[index % 3]
            )
            draw = ImageDraw.Draw(image)
            draw.text((20, 20), f"SIMULATION / {name}", fill="white")
            buffer = io.BytesIO()
            image.save(buffer, "JPEG", quality=85)
            self._images[name] = buffer.getvalue()
        self._emit()
        self._thread = threading.Thread(target=self._loop, name="ee-control-sim", daemon=True)
        self._thread.start()

    def _emit(self) -> None:
        with self._lock:
            for arm in ARMS:
                state = self._states[arm]
                control = self.config.control
                raw_gripper = (
                    state["gripper"] - control.gripper_feedback_offset
                ) / control.gripper_feedback_scale
                self.cache.update_arm(
                    arm,
                    state["end_pos"],
                    (*state["joint_pos"], raw_gripper),
                    (0,) * 7,
                    (0,) * 7,
                    stamp=time.time(),
                )
            for name, data in self._images.items():
                self.cache.update_camera(name, data, stamp=time.time())

    def _loop(self) -> None:
        period = 1 / self.config.control.publish_rate_hz
        while not self._stop.wait(period):
            self._emit()

    def has_subscriber(self, arm: str) -> bool:
        return self.connected

    def publish(self, arm: str, target: ArmTarget) -> None:
        if not self.actuation_enabled:
            raise RuntimeError("actuation disabled (--no-actuation)")
        if not self.connected:
            raise RuntimeError("simulation backend disconnected")
        with self._lock:
            self._states[arm]["end_pos" if target.mode == END_CONTROL else "joint_pos"] = (
                target.values
            )
            self._states[arm]["gripper"] = target.gripper

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        self._thread = None
