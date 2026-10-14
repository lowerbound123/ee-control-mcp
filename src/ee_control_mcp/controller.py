from __future__ import annotations

import copy
import logging
import threading
import time
import uuid
from collections import OrderedDict, deque
from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from .backends import RobotBackend
from .commands import MOTION_AXES, Step
from .config import Config
from .interpolation import UniformTrajectory, as_rpy, rotation_gap
from .observations import ArmFeedback, ObservationCache
from .types import END_CONTROL, POSITION_CONTROL, ArmTarget

logger = logging.getLogger(__name__)
TERMINAL = {"completed", "failed", "cancelled"}


class ExecutionCancelled(RuntimeError):
    pass


@dataclass
class Batch:
    batch_id: str
    commands: tuple[str, ...]
    status: str = "queued"
    completed_steps: int = 0
    failed_steps: int = 0
    cancelled_steps: int = 0
    error: str | None = None

    def result(self) -> dict:
        return {
            "accepted": True,
            "batch_id": self.batch_id,
            "commands": list(self.commands),
            "status": self.status,
            "total_steps": len(self.commands),
            "completed_steps": self.completed_steps,
            "failed_steps": self.failed_steps,
            "cancelled_steps": self.cancelled_steps,
            "error": self.error,
        }


@dataclass(frozen=True)
class WorkItem:
    batch: Batch
    index: int
    step: Step


class Controller:
    """One process-wide queue; only its worker is allowed to publish robot targets."""

    def __init__(self, config: Config, cache: ObservationCache, backend: RobotBackend):
        self.config, self.cache, self.backend = config, cache, backend
        self._condition = threading.Condition()
        self._queue: deque[WorkItem] = deque()
        self._batches: OrderedDict[str, Batch] = OrderedDict()
        self._active: WorkItem | None = None
        self._details: dict = {}
        self._targets: dict[str, ArmTarget] = {}
        self._last_error = None
        self._stop = threading.Event()
        self._thread = None

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("controller already started")
        self._thread = threading.Thread(target=self._run, name="ee-control-executor", daemon=True)
        self._thread.start()

    def _require_robot(self, arms) -> dict[str, ArmFeedback]:
        if not self.backend.actuation_enabled:
            raise RuntimeError("actuation disabled (--no-actuation)")
        if not self.backend.connected:
            raise RuntimeError("robot backend disconnected")
        feedback = {}
        for arm in arms:
            if not self.backend.has_subscriber(arm):
                raise RuntimeError(f"no subscriber on {self.config.ros.command_topics[arm]}")
            feedback[arm] = self.cache.fresh_arm(arm, self.config.observation.feedback_max_age_s)
        return feedback

    def submit(self, steps: list[Step], *, clear_queue: bool = False) -> dict:
        if not steps or any(not isinstance(step, Step) or not step.arms for step in steps):
            raise ValueError("a batch must contain nonempty, validated steps")
        arms = {arm for step in steps for arm in step.arms}
        self._require_robot(arms)
        with self._condition:
            if self._thread is None or self._stop.is_set():
                raise RuntimeError("controller is not running")
            pending = 0 if clear_queue else len(self._queue)
            if pending + len(steps) > self.config.control.max_pending_steps:
                raise RuntimeError("command queue capacity exceeded; no steps were queued")
            if clear_queue:
                self._cancel_pending("queue cleared by a new command")
            batch = Batch(uuid.uuid4().hex, tuple(step.text for step in steps))
            self._batches[batch.batch_id] = batch
            self._queue.extend(WorkItem(batch, index, step) for index, step in enumerate(steps))
            self._prune()
            self._condition.notify_all()
            return batch.result()

    def wait(self, batch_id: str, timeout_s: float) -> dict:
        with self._condition:
            if batch_id not in self._batches:
                raise ValueError("unknown or expired batch_id")
            batch = self._batches[batch_id]
            self._condition.wait_for(lambda: batch.status in TERMINAL, timeout_s)
            return {**batch.result(), "timed_out": batch.status not in TERMINAL}

    def progress(self) -> dict:
        with self._condition:
            return {
                "busy": self._active is not None,
                "pending": len(self._queue),
                "drained": self._active is None and not self._queue,
                "active_batch_id": self._active.batch.batch_id if self._active else None,
                "active_command": self._active.step.text if self._active else None,
                "active_step_index": self._active.index if self._active else None,
                "execution": copy.deepcopy(self._details),
                "commanded_targets": {
                    arm: target.as_dict() for arm, target in self._targets.items()
                },
                "last_command_error": self._last_error,
                "recent_batches": [batch.result() for batch in list(self._batches.values())[-8:]],
            }

    @staticmethod
    def _finalize(batch: Batch) -> None:
        done = batch.completed_steps + batch.failed_steps + batch.cancelled_steps
        if done == len(batch.commands):
            batch.status = (
                "failed"
                if batch.failed_steps
                else "cancelled"
                if batch.cancelled_steps
                else "completed"
            )

    def _cancel_pending(self, reason: str) -> None:
        while self._queue:
            batch = self._queue.popleft().batch
            batch.cancelled_steps += 1
            if batch.error is None:
                batch.error = reason
            self._finalize(batch)
        self._condition.notify_all()

    def _prune(self) -> None:
        finished = [key for key, batch in self._batches.items() if batch.status in TERMINAL]
        for key in finished[: -self.config.control.completed_history_size]:
            self._batches.pop(key)

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._queue or self._stop.is_set())
                if self._stop.is_set():
                    return
                item = self._queue.popleft()
                self._active, self._details = item, {}
                item.batch.status = "running"
            error, cancelled = None, False
            try:
                self._execute(item.step)
            except ExecutionCancelled as exc:
                error, cancelled = str(exc), True
            except Exception as exc:
                error = f"{type(exc).__name__}: {exc}"
                logger.warning("Command failed: %s", error)
            with self._condition:
                if cancelled:
                    item.batch.cancelled_steps += 1
                    item.batch.error = error
                elif error:
                    item.batch.failed_steps += 1
                    item.batch.error = error
                    self._last_error = error
                    self._targets.clear()
                    self._cancel_pending(f"cancelled after execution failure: {error}")
                else:
                    item.batch.completed_steps += 1
                    self._last_error = None
                self._finalize(item.batch)
                self._active, self._details = None, {}
                self._prune()
                self._condition.notify_all()

    def _command_gripper(self, feedback: ArmFeedback) -> float:
        cfg = self.config.control
        return feedback.joint_pos[-1] * cfg.gripper_feedback_scale + cfg.gripper_feedback_offset

    def _resolve(self, step: Step, feedback: dict) -> tuple[dict, dict]:
        with self._condition:
            previous = dict(self._targets)
        goals, starts = {}, {}
        cfg = self.config.control

        def baseline(arm, mode):
            prior = previous.get(arm)
            measured = feedback[arm].end_pos if mode == END_CONTROL else feedback[arm].joint_pos[:6]
            start = prior.values if prior and prior.mode == mode else measured
            gripper = prior.gripper if prior else self._command_gripper(feedback[arm])
            return start, gripper

        for request in step.targets:
            start, gripper = baseline(request.arm, request.mode)
            goals[request.arm] = ArmTarget(
                request.mode,
                request.values,
                gripper if request.gripper is None else request.gripper,
                request.speed,
            )
            starts[request.arm] = start
        for clause in step.clauses:
            for arm in clause.arms:
                prior = previous.get(arm)
                if clause.motions == ("home",):
                    mode = POSITION_CONTROL
                elif any(name in MOTION_AXES for name in clause.motions):
                    mode = END_CONTROL
                else:
                    mode = prior.mode if prior else POSITION_CONTROL
                start, gripper = baseline(arm, mode)
                pose = np.asarray(start, dtype=float).copy()
                delta = np.zeros(6)
                for name in clause.motions:
                    if name in MOTION_AXES:
                        motion_delta = np.asarray(cfg.motions[name], dtype=float)
                        if clause.value is not None:
                            # Keep calibrated direction when overriding magnitude.
                            axis = MOTION_AXES[name][0]
                            component = motion_delta[:3] if axis < 3 else motion_delta[3:]
                            magnitude = float(np.linalg.norm(component))
                            if magnitude == 0:
                                component_name = "translation" if axis < 3 else "rotation"
                                raise ValueError(f"{name} has no configured {component_name}")
                            motion_delta = motion_delta * clause.value / magnitude
                        delta += motion_delta
                    elif name in ("open", "close"):
                        gripper = (
                            clause.value
                            if clause.value is not None
                            else cfg.gripper_open
                            if name == "open"
                            else cfg.gripper_close
                        )
                    elif name == "home":
                        pose, gripper = np.zeros(6), 0.0
                if mode == END_CONTROL:
                    pose[:3] += delta[:3]
                    if np.any(delta[3:]):
                        pose[3:] = as_rpy(
                            Rotation.from_euler("xyz", pose[3:]) * Rotation.from_rotvec(delta[3:])
                        )
                goals[arm] = ArmTarget(mode, tuple(pose), gripper, clause.speed)
                starts[arm] = start
        return starts, goals

    def _errors(self, goal: ArmTarget, feedback: ArmFeedback) -> dict:
        cfg = self.config.control
        gripper_error = abs(self._command_gripper(feedback) - goal.gripper)
        if goal.mode == END_CONTROL:
            position = float(np.linalg.norm(np.asarray(goal.values[:3]) - feedback.end_pos[:3]))
            rotation = rotation_gap(feedback.end_pos[3:], goal.values[3:])
            result = {"position_m": position, "orientation_rad": rotation}
            arrived = (
                position <= cfg.position_tolerance_m and rotation <= cfg.rotation_tolerance_rad
            )
        else:
            joint = float(np.max(np.abs(np.asarray(goal.values) - feedback.joint_pos[:6])))
            result = {"joint_max_rad": joint}
            arrived = joint <= cfg.joint_tolerance_rad
        return {
            **result,
            "gripper_error": gripper_error,
            "within_tolerance": arrived and gripper_error <= cfg.gripper_tolerance,
        }

    def _execute(self, step: Step) -> None:
        feedback = self._require_robot(step.arms)
        starts, goals = self._resolve(step, feedback)
        trajectories = {
            arm: UniformTrajectory(starts[arm], goal, self.config.control.speeds[goal.speed])
            for arm, goal in goals.items()
        }
        period = 1 / self.config.control.publish_rate_hz
        ticks = max(trajectory.steps for trajectory in trajectories.values())
        if ticks * period > self.config.control.step_timeout_s:
            raise RuntimeError(
                "trajectory exceeds step timeout; use a shorter move or raise step_timeout_s"
            )
        with self._condition:
            self._targets.update(goals)
        deadline = time.monotonic() + self.config.control.step_timeout_s
        next_tick = time.monotonic() + period
        last_sequences = {arm: state.seq for arm, state in feedback.items()}
        settled = {arm: 0 for arm in goals}
        tick = 0
        endpoint_sent: set[str] = set()
        while True:
            if self._stop.wait(max(0, next_tick - time.monotonic())):
                raise ExecutionCancelled("server shutting down")
            if time.monotonic() > deadline:
                raise RuntimeError("step timeout while waiting for fresh arrival feedback")
            feedback = self._require_robot(goals)
            tick = min(tick + 1, ticks)
            for arm, trajectory in trajectories.items():
                self.backend.publish(arm, trajectory.sample(tick))
                if tick >= trajectory.steps and arm not in endpoint_sent:
                    # Establish each arm's boundary after its own publication.
                    # Conservatively skip callbacks concurrent with that publish.
                    last_sequences[arm] = self.cache.fresh_arm(
                        arm, self.config.observation.feedback_max_age_s
                    ).seq
                    settled[arm] = 0
                    endpoint_sent.add(arm)
            feedback = self._require_robot(goals)
            arrival = {}
            for arm, goal in goals.items():
                errors = self._errors(goal, feedback[arm])
                if arm in endpoint_sent:
                    for sample in self.cache.feedback_since(arm, last_sequences[arm]):
                        if self._errors(goal, sample)["within_tolerance"]:
                            settled[arm] += 1
                        else:
                            settled[arm] = 0
                        last_sequences[arm] = sample.seq
                else:
                    last_sequences[arm] = feedback[arm].seq
                if not errors["within_tolerance"]:
                    settled[arm] = 0
                arrival[arm] = {**errors, "settled_observations": settled[arm]}
            with self._condition:
                self._details = {
                    "phase": "interpolating" if tick < ticks else "waiting_for_arrival",
                    "arrival": arrival,
                    "interpolation": {
                        arm: {
                            "steps_total": path.steps,
                            "steps_sent": min(tick, path.steps),
                            "speed": path.target.speed,
                        }
                        for arm, path in trajectories.items()
                    },
                }
            if tick >= ticks and all(
                count >= self.config.control.settle_observations for count in settled.values()
            ):
                return
            # Never catch up by bursting several interpolation targets at once.
            next_tick += period
            if next_tick <= time.monotonic():
                next_tick = time.monotonic() + period

    def close(self) -> None:
        with self._condition:
            self._stop.set()
            self._cancel_pending("server shutting down")
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join(timeout=5)
            if self._thread.is_alive():
                raise RuntimeError("control publisher did not stop; backend must remain alive")
