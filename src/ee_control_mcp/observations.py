from __future__ import annotations

import io
import threading
import time
from collections import deque
from dataclasses import dataclass

from PIL import Image

from .types import ARMS, finite_number, finite_vector


class StateUnavailable(RuntimeError):
    """The arm has no usable recent feedback."""


@dataclass(frozen=True)
class ArmFeedback:
    end_pos: tuple[float, ...]
    joint_pos: tuple[float, ...]
    joint_vel: tuple[float, ...]
    joint_cur: tuple[float, ...]
    received_at: float
    stamp: float | None
    seq: int


@dataclass(frozen=True)
class CameraFrame:
    data: bytes
    received_at: float
    stamp: float | None
    seq: int


@dataclass(frozen=True)
class EncodedImage:
    name: str
    data: bytes


class ObservationCache:
    """Thread-safe, non-consuming latest-message cache; age uses a monotonic clock."""

    def __init__(self, cameras=("head", "left_wrist", "right_wrist")):
        self._condition = threading.Condition()
        self._arms: dict[str, ArmFeedback] = {}
        # Keep intermediate receipt samples for consecutive arrival checking.
        # Public observations still use the latest sample and never consume it.
        self._history: dict[str, deque[ArmFeedback]] = {arm: deque(maxlen=256) for arm in ARMS}
        self._history_floor = dict.fromkeys(ARMS, 0)
        self._frames: dict[str, CameraFrame] = {}
        self._cameras = tuple(cameras)
        self._seq = 0
        self._image_seq = 0
        self._errors: dict[str, str] = {}
        self._closed = False

    @property
    def seq(self) -> int:
        with self._condition:
            return self._seq

    def update_arm(
        self, arm, end_pos, joint_pos, joint_vel, joint_cur, *, stamp=None, received_at=None
    ) -> None:
        if arm not in ARMS:
            raise ValueError(f"unknown arm {arm}")
        values = (
            finite_vector(end_pos, 6, "end_pos"),
            finite_vector(joint_pos, 7, "joint_pos"),
            finite_vector(joint_vel, 7, "joint_vel"),
            finite_vector(joint_cur, 7, "joint_cur"),
        )
        stamp = None if stamp is None else finite_number(stamp, "timestamp")
        received_at = time.monotonic() if received_at is None else finite_number(received_at, "age")
        with self._condition:
            self._seq += 1
            feedback = ArmFeedback(*values, received_at, stamp, self._seq)
            self._arms[arm] = feedback
            history = self._history[arm]
            if len(history) == history.maxlen:
                self._history_floor[arm] = history[0].seq
            history.append(feedback)
            self._errors.pop(arm, None)
            self._condition.notify_all()

    def update_camera(self, camera, data, *, stamp=None, received_at=None) -> None:
        if camera not in self._cameras:
            raise ValueError(f"unknown camera {camera}")
        stamp = None if stamp is None else finite_number(stamp, "timestamp")
        received_at = time.monotonic() if received_at is None else finite_number(received_at, "age")
        with self._condition:
            self._image_seq += 1
            self._frames[camera] = CameraFrame(bytes(data), received_at, stamp, self._image_seq)
            self._errors.pop(camera, None)
            self._condition.notify_all()

    def record_error(self, source: str, error: object) -> None:
        with self._condition:
            self._errors[source] = str(error)

    def fresh_arm(self, arm: str, max_age_s: float) -> ArmFeedback:
        with self._condition:
            feedback = self._arms.get(arm)
        if feedback is None:
            raise StateUnavailable(f"{arm} feedback missing")
        age = time.monotonic() - feedback.received_at
        if age > max_age_s:
            raise StateUnavailable(f"{arm} feedback stale ({age:.3f}s > {max_age_s:.3f}s)")
        return feedback

    def feedback_since(self, arm: str, seq: int) -> list[ArmFeedback]:
        """Read all later receipt samples; losing intermediate history is a fault."""
        if arm not in ARMS:
            raise ValueError(f"unknown arm {arm}")
        with self._condition:
            if seq < self._history_floor[arm]:
                raise StateUnavailable(f"{arm} feedback history overflow; cannot verify arrival")
            return [sample for sample in self._history[arm] if sample.seq > seq]

    def state(self, max_age_s: float) -> dict:
        with self._condition:
            arms, seq, errors = dict(self._arms), self._seq, dict(self._errors)
        now = time.monotonic()
        result = {"seq": seq, "timestamp": time.time(), "arms": {}, "observation_errors": errors}
        for arm in ARMS:
            feedback = arms.get(arm)
            if feedback is None:
                result["arms"][arm] = {
                    "ready": False,
                    "end_pos": None,
                    "joint_pos": None,
                    "gripper": None,
                    "age_s": None,
                    "timestamp": None,
                }
            else:
                age = max(0, now - feedback.received_at)
                result["arms"][arm] = {
                    "ready": age <= max_age_s,
                    "seq": feedback.seq,
                    "end_pos": list(feedback.end_pos),
                    "joint_pos": list(feedback.joint_pos),
                    "joint_vel": list(feedback.joint_vel),
                    "joint_cur": list(feedback.joint_cur),
                    "gripper": feedback.joint_pos[-1],
                    "age_s": age,
                    "timestamp": feedback.stamp,
                }
        result["state_ready"] = all(result["arms"][arm]["ready"] for arm in ARMS)
        for key, attribute in (
            ("qpos", "joint_pos"),
            ("qvel", "joint_vel"),
            ("effort", "joint_cur"),
        ):
            result[key] = (
                [value for arm in ARMS for value in getattr(arms[arm], attribute)]
                if (all(arm in arms for arm in ARMS))
                else None
            )
        result["eef"] = (
            [
                value
                for arm in ARMS
                for value in (
                    *arms[arm].end_pos,
                    arms[arm].joint_pos[-1],
                )
            ]
            if all(arm in arms for arm in ARMS)
            else None
        )
        return result

    def wait_for_new(self, since_seq: int, timeout_s: float) -> bool:
        with self._condition:
            self._condition.wait_for(lambda: self._seq > since_seq or self._closed, timeout_s)
            return self._seq > since_seq

    def images(
        self,
        cameras=None,
        *,
        max_age_s: float = 1,
        quality: int = 85,
        max_width: int = 1024,
        max_height: int = 768,
    ) -> tuple[dict, list[EncodedImage]]:
        cameras = list(self._cameras) if cameras is None else cameras
        if (
            not cameras
            or len(cameras) != len(set(cameras))
            or any(camera not in self._cameras for camera in cameras)
        ):
            raise ValueError(f"cameras must be unique names from {self._cameras}")
        if not 1 <= quality <= 100:
            raise ValueError("image_quality must be in [1, 100]")
        with self._condition:
            frames = dict(self._frames)
        metadata = {"images": [], "cameras_missing": [], "cameras_stale": [], "camera_errors": {}}
        encoded = []
        for name in cameras:
            frame = frames.get(name)
            if frame is None:
                metadata["cameras_missing"].append(name)
                continue
            age = max(0, time.monotonic() - frame.received_at)
            if age > max_age_s:
                metadata["cameras_stale"].append(name)
                continue
            try:
                with Image.open(io.BytesIO(frame.data)) as source:
                    rgb = source.convert("RGB")
                    rgb.thumbnail((max_width, max_height), Image.Resampling.LANCZOS)
                    output = io.BytesIO()
                    rgb.save(output, format="JPEG", quality=quality)
                    width, height = rgb.size
            except (OSError, ValueError, Image.DecompressionBombError) as exc:
                metadata["camera_errors"][name] = str(exc)
                continue
            encoded.append(EncodedImage(name, output.getvalue()))
            metadata["images"].append(
                {
                    "camera": name,
                    "seq": frame.seq,
                    "timestamp": frame.stamp,
                    "age_s": age,
                    "width": width,
                    "height": height,
                    "mime_type": "image/jpeg",
                }
            )
        return metadata, encoded

    def close(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
