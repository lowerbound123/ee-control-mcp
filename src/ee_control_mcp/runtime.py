from __future__ import annotations

import numpy as np

from .backends import RosBackend, SimBackend
from .config import Config
from .controller import Controller
from .interpolation import rotation_gap
from .observations import ObservationCache


class Runtime:
    def __init__(self, config: Config, *, backend: str = "ros", actuation_enabled: bool = True):
        self.config = config
        self.cache = ObservationCache(config.ros.camera_topics)
        if backend not in ("ros", "sim"):
            raise ValueError("backend must be ros or sim")
        self.backend = (RosBackend if backend == "ros" else SimBackend)(
            config,
            self.cache,
            actuation_enabled=actuation_enabled,
        )
        self.controller = Controller(config, self.cache, self.backend)

    def start(self) -> None:
        self.backend.start()
        try:
            self.controller.start()
        except Exception:
            self.backend.close()
            raise

    def snapshot(self) -> dict:
        state = self.cache.state(self.config.observation.feedback_max_age_s)
        progress = self.controller.progress()
        for arm, observation in state["arms"].items():
            target = progress["commanded_targets"].get(arm)
            observation["commanded_target"] = target
            observation["target_error"] = None
            if target and observation["end_pos"] is not None:
                if target["mode"] == 4:
                    difference = np.asarray(target["end_pos"][:3]) - observation["end_pos"][:3]
                    error = {
                        "position_delta_m": difference.tolist(),
                        "position_m": float(np.linalg.norm(difference)),
                        "orientation_rad": rotation_gap(
                            observation["end_pos"][3:], target["end_pos"][3:]
                        ),
                    }
                else:
                    difference = np.asarray(target["joint_pos"]) - observation["joint_pos"][:6]
                    error = {
                        "joint_delta_rad": difference.tolist(),
                        "joint_max_rad": float(np.max(np.abs(difference))),
                    }
                command_gripper = (
                    observation["gripper"] * self.config.control.gripper_feedback_scale
                    + self.config.control.gripper_feedback_offset
                )
                observation["target_error"] = {
                    **error,
                    "gripper_error": target["gripper"] - command_gripper,
                }
        return {
            **state,
            **progress,
            "backend": self.backend.name,
            "connected": self.backend.connected,
            "actuation_enabled": self.backend.actuation_enabled,
        }

    def close(self) -> None:
        # Do not destroy ROS resources if an in-flight publisher cannot be joined.
        self.controller.close()
        self.backend.close()
        self.cache.close()
