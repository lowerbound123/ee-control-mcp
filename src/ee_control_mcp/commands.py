from __future__ import annotations

from dataclasses import dataclass

from .types import ARMS, SPEEDS, finite_number, finite_vector

MOTION_AXES = {
    "forward": (0, 1),
    "backward": (0, -1),
    "left": (1, 1),
    "right": (1, -1),
    "up": (2, 1),
    "down": (2, -1),
    "twist-clockwise": (3, 1),
    "twist-counterclockwise": (3, -1),
    "tilt-down-toward-table": (4, 1),
    "tilt-up-away-from-table": (4, -1),
    "turn-left-across-table": (5, 1),
    "turn-right-across-table": (5, -1),
}
MOTIONS = tuple(MOTION_AXES) + ("open", "close", "home", "hold")


@dataclass(frozen=True)
class Clause:
    arms: tuple[str, ...]
    motions: tuple[str, ...]
    value: float | None
    speed: str


@dataclass(frozen=True)
class RequestedTarget:
    arm: str
    mode: int
    values: tuple[float, ...]
    gripper: float | None
    speed: str


@dataclass(frozen=True)
class Step:
    text: str
    clauses: tuple[Clause, ...] = ()
    targets: tuple[RequestedTarget, ...] = ()

    @property
    def arms(self) -> tuple[str, ...]:
        return tuple(
            arm
            for arm in ARMS
            if any(arm in clause.arms for clause in self.clauses)
            or any(target.arm == arm for target in self.targets)
        )


def _parse_clause(text: str, default_speed: str) -> Clause:
    if not isinstance(text, str) or not text.strip():
        raise ValueError("each arm clause must be a nonempty string")
    parts = text.strip().split("@")
    if len(parts) > 2:
        raise ValueError("use at most one @speed suffix per arm clause")
    speed = parts[1].strip() if len(parts) == 2 else default_speed
    if speed not in SPEEDS:
        raise ValueError(f"unknown speed {speed!r}; choose {SPEEDS}")
    fields = [item.strip() for item in parts[0].split(":")]
    if len(fields) not in (2, 3):
        raise ValueError("grammar: <left|right|both>:<motion>[&motion][:value][@speed]")
    arm, motion_text = fields[:2]
    if arm not in (*ARMS, "both"):
        raise ValueError("arm must be left, right or both")
    motions = tuple(item.strip() for item in motion_text.split("&"))
    if any(item not in MOTIONS for item in motions):
        raise ValueError(f"unknown motion; available motions: {MOTIONS}")
    if len(set(motions)) != len(motions):
        raise ValueError("a motion may only appear once in an arm clause")
    if any(item in motions for item in ("home", "hold")) and len(motions) != 1:
        raise ValueError("home and hold must be used alone")
    if "open" in motions and "close" in motions:
        raise ValueError("open and close cannot appear in the same arm clause")
    value = None
    if len(fields) == 3:
        try:
            value = finite_number(float(fields[2]), "value")
        except ValueError as exc:
            raise ValueError("value must be a finite number") from exc
        if motions[0] in ("home", "hold"):
            raise ValueError("home and hold do not take a value")
        if any(item in MOTION_AXES for item in motions) and value <= 0:
            raise ValueError("motion distance/angle must be positive; the word sets direction")
    if len(parts) == 2 and not any(item in MOTION_AXES or item == "home" for item in motions):
        raise ValueError("@speed requires a pose motion or home")
    return Clause(ARMS if arm == "both" else (arm,), motions, value, speed)


def parse_commands(raw: object, default_speed: str = "slow") -> list[Step]:
    if default_speed not in SPEEDS:
        raise ValueError(f"default_speed must be one of {SPEEDS}")
    entries = [raw] if isinstance(raw, str) else raw
    if not isinstance(entries, list) or not entries:
        raise ValueError("commands must be a string or a nonempty list of steps")
    steps = []
    for entry in entries:
        if isinstance(entry, str):
            clauses_text = entry.split("|")
        elif isinstance(entry, list) and entry:
            if any(not isinstance(part, str) or "|" in part for part in entry):
                raise ValueError("a nested step contains one arm clause per string")
            clauses_text = entry
        else:
            raise ValueError("each step must be a string or a nonempty list of arm clauses")
        clauses = tuple(_parse_clause(part, default_speed) for part in clauses_text)
        used = [arm for clause in clauses for arm in clause.arms]
        if len(used) != len(set(used)):
            raise ValueError("an arm may only appear once per parallel step")
        if all(clause.motions == ("hold",) for clause in clauses):
            raise ValueError("a step cannot consist only of hold")
        steps.append(Step(" | ".join(part.strip() for part in clauses_text), clauses))
    return steps


def structured_step(mode: int, targets: dict, default_speed: str = "slow") -> Step:
    if mode not in (4, 5) or not isinstance(targets, dict) or not targets:
        raise ValueError("targets must contain left and/or right arm targets")
    key = "end_pos" if mode == 4 else "joint_pos"
    requested = []
    for arm, payload in targets.items():
        if arm not in ARMS or not isinstance(payload, dict):
            raise ValueError("targets must contain left and/or right arm objects")
        if set(payload) - {key, "gripper", "speed"}:
            raise ValueError(f"target fields must be {key}, gripper and speed")
        speed = payload.get("speed") or default_speed
        if speed not in SPEEDS:
            raise ValueError(f"speed must be one of {SPEEDS}")
        gripper = payload.get("gripper")
        if gripper is not None:
            gripper = finite_number(gripper, "gripper")
        requested.append(
            RequestedTarget(
                arm,
                mode,
                finite_vector(payload.get(key), 6, key),
                gripper,
                speed,
            )
        )
    return Step(
        f"move_{'eef' if mode == 4 else 'joints'}: {', '.join(targets)}", targets=tuple(requested)
    )
