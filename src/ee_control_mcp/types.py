from __future__ import annotations

import math
from dataclasses import dataclass
from numbers import Real
from typing import Literal

Arm = Literal["left", "right"]
Speed = Literal["slow", "medium", "fast"]
ARMS = ("left", "right")
SPEEDS = ("slow", "medium", "fast")
END_CONTROL = 4
POSITION_CONTROL = 5


def finite_number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be a finite number")
    return float(value)


def finite_vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if isinstance(value, (str, bytes, dict)):
        raise ValueError(f"{name} must contain exactly {length} numbers")
    try:
        items = list(value)  # type: ignore[arg-type]
    except TypeError as exc:
        raise ValueError(f"{name} must contain exactly {length} numbers") from exc
    if len(items) != length:
        raise ValueError(f"{name} must contain exactly {length} numbers")
    return tuple(finite_number(item, name) for item in items)


@dataclass(frozen=True)
class ArmTarget:
    mode: int
    values: tuple[float, ...]
    gripper: float
    speed: str = "slow"

    def __post_init__(self) -> None:
        if self.mode not in (END_CONTROL, POSITION_CONTROL):
            raise ValueError("only END_CONTROL (4) and POSITION_CONTROL (5) are supported")
        object.__setattr__(self, "values", finite_vector(self.values, 6, "target"))
        object.__setattr__(self, "gripper", finite_number(self.gripper, "gripper"))
        if self.speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")

    def as_dict(self) -> dict:
        key = "end_pos" if self.mode == END_CONTROL else "joint_pos"
        return {
            "mode": self.mode,
            key: list(self.values),
            "gripper": self.gripper,
            "speed": self.speed,
        }
