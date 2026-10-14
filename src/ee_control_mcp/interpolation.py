from __future__ import annotations

import math
import warnings

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from .config import Granularity
from .types import END_CONTROL, ArmTarget, finite_vector


def rotation_gap(start, goal) -> float:
    return float(
        (Rotation.from_euler("xyz", start).inv() * Rotation.from_euler("xyz", goal)).magnitude()
    )


def as_rpy(rotation: Rotation) -> tuple[float, ...]:
    # At gimbal lock scipy chooses an equivalent representation and warns; the
    # represented pose is valid, and the controller compares rotations, not RPY.
    with warnings.catch_warnings():
        warnings.filterwarnings("ignore", message="Gimbal lock detected.*", category=UserWarning)
        return tuple(float(value) for value in rotation.as_euler("xyz"))


class UniformTrajectory:
    """Sample a straight translation and constant-angle shortest rotation arc."""

    def __init__(self, start, target: ArmTarget, granularity: Granularity):
        self.start = np.asarray(finite_vector(start, 6, "start"))
        self.target = target
        self.goal = np.asarray(target.values)
        self._slerp = None
        if target.mode == END_CONTROL:
            distance = float(np.linalg.norm(self.goal[:3] - self.start[:3]))
            angle = rotation_gap(self.start[3:], self.goal[3:])
            ratios = (distance / granularity.translation_m, angle / granularity.angle_rad)
            self._slerp = Slerp([0, 1], Rotation.from_euler("xyz", [self.start[3:], self.goal[3:]]))
        else:
            ratios = (float(np.max(np.abs(self.goal - self.start))) / granularity.angle_rad,)
        self.steps = max(1, math.ceil(max(ratios) - 1e-12))

    def sample(self, tick: int) -> ArmTarget:
        if tick >= self.steps:
            return self.target
        fraction = max(0, tick) / self.steps
        values = self.start + fraction * (self.goal - self.start)
        if self._slerp is not None:
            values[3:] = as_rpy(self._slerp(fraction))
        return ArmTarget(self.target.mode, tuple(values), self.target.gripper, self.target.speed)
