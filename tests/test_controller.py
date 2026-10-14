import threading
import time
from contextlib import contextmanager
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from ee_control_mcp.backends import SimBackend
from ee_control_mcp.commands import parse_commands, structured_step
from ee_control_mcp.config import Config, load_config
from ee_control_mcp.observations import ObservationCache


class RecordingSim(SimBackend):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.calls = []

    def publish(self, arm, target):
        super().publish(arm, target)
        self.calls.append((arm, target))


def fast_config(**control):
    config = Config()
    return replace(
        config, control=replace(config.control, publish_rate_hz=1000, step_timeout_s=2, **control)
    )


@contextmanager
def running(config=None, backend_cls=RecordingSim, *, actuation_enabled=True):
    from ee_control_mcp.controller import Controller

    config = config or fast_config()
    cache = ObservationCache()
    backend = backend_cls(config, cache, actuation_enabled=actuation_enabled)
    backend.start()
    controller = Controller(config, cache, backend)
    controller.start()
    try:
        yield controller, backend, cache
    finally:
        release = getattr(backend, "release", None)
        if release:
            release.set()
        controller.close()
        backend.close()
        cache.close()


def execute(controller, steps, timeout=3):
    batch = controller.submit(steps)
    result = controller.wait(batch["batch_id"], timeout)
    assert result["status"] == "completed", result
    return result


def test_original_dual_eef_example_and_joint_home_endpoints():
    with running() as (controller, backend, cache):
        execute(
            controller,
            [
                structured_step(
                    4,
                    {
                        "left": {"end_pos": [0.1, 0.03, 0.05, 0, 0, 0], "gripper": 1},
                        "right": {"end_pos": [0.05, 0, 0.1, 0, 0, 0], "gripper": 1},
                    },
                )
            ],
        )
        state = cache.state(0.5)
        assert state["arms"]["left"]["end_pos"] == [0.1, 0.03, 0.05, 0, 0, 0]
        assert state["arms"]["right"]["end_pos"] == [0.05, 0, 0.1, 0, 0, 0]
        assert state["qpos"][6] == state["qpos"][13] == 1
        execute(controller, parse_commands("both:home"))
        assert cache.state(0.5)["qpos"] == [0] * 14
        for arm in ("left", "right"):
            target = [target for side, target in backend.calls if side == arm][-1]
            assert target.mode == 5 and target.values == (0,) * 6 and target.gripper == 0


def test_relative_targets_accumulate_positive_left_y_and_preserve_gripper():
    with running() as (controller, backend, cache):
        execute(controller, parse_commands(["left:open", "left:left:0.003", "left:up:0.004"]))
        assert cache.state(0.5)["arms"]["left"]["end_pos"][:3] == pytest.approx([0.2, 0.103, 0.204])
        assert cache.state(0.5)["arms"]["left"]["gripper"] == 1
        assert all(arm == "left" for arm, _ in backend.calls)


def test_parallel_arms_use_independent_granularity_and_hold_early_endpoint():
    with running() as (controller, backend, cache):
        execute(controller, parse_commands("left:up:0.006@slow | right:down:0.006@fast"))
        left = [target.values[2] for arm, target in backend.calls if arm == "left"]
        right = [target.values[2] for arm, target in backend.calls if arm == "right"]
        assert left[:6] == pytest.approx([0.201, 0.202, 0.203, 0.204, 0.205, 0.206])
        assert right[:6] == pytest.approx([0.197, 0.194, 0.194, 0.194, 0.194, 0.194])


def test_joint_to_eef_transition_reanchors_on_observed_pose():
    with running() as (controller, backend, cache):
        execute(controller, [structured_step(4, {"left": {"end_pos": [0.25, 0.1, 0.2, 0, 0, 0]}})])
        execute(controller, parse_commands("left:home"))
        # The simulator has no FK: emulate a different post-joint-mode Cartesian feedback.
        from ee_control_mcp.types import ArmTarget

        backend.publish("left", ArmTarget(4, (0.3, 0.15, 0.25, 0, 0, 0), 0))
        deadline = time.monotonic() + 1
        while cache.fresh_arm("left", 0.5).end_pos[0] != 0.3:
            assert time.monotonic() < deadline
            cache.wait_for_new(cache.seq, 0.01)
        execute(controller, parse_commands("left:up:0.001"))
        assert cache.state(0.5)["arms"]["left"]["end_pos"][:3] == pytest.approx([0.3, 0.15, 0.251])


def test_gripper_calibration_does_not_change_reported_raw_state():
    with running(fast_config(gripper_feedback_scale=0.5)) as (controller, backend, cache):
        execute(controller, parse_commands(["left:open", "left:up:0.001"]))
        assert cache.state(0.5)["qpos"][6] == 2
        assert backend.calls[-1][1].gripper == 1


@pytest.mark.parametrize(
    "command,raw",
    [("left:open:-1.7", -1.7), ("left:close:0", 0.0), ("left:open:2", 2.0)],
)
def test_explicit_gripper_commands_preserve_native_motor_units(command, raw):
    with running() as (controller, backend, cache):
        execute(controller, parse_commands(command))
        assert cache.state(0.5)["arms"]["left"]["gripper"] == raw
        assert backend.calls[-1][1].gripper == raw


class NativeGripperEndStopSim(RecordingSim):
    """Measured v2 motor feedback stops short of both commanded end points."""

    def publish(self, arm, target):
        opened = {"left": -3.317, "right": -3.296}[arm]
        closed = {"left": -0.068, "right": -0.058}[arm]
        super().publish(arm, replace(target, gripper=max(opened, min(target.gripper, closed))))


@pytest.mark.parametrize("word,raw", [("open", [-3.317, -3.296]), ("close", [-0.068, -0.058])])
def test_robotics52_native_gripper_words_arrive_with_measured_end_stops(word, raw):
    config = load_config(Path(__file__).resolve().parents[1] / "config/robotics52.yaml")
    config = replace(
        config,
        control=replace(config.control, publish_rate_hz=1000, step_timeout_s=0.3),
    )
    with running(config, backend_cls=NativeGripperEndStopSim) as (controller, backend, cache):
        execute(controller, parse_commands(f"both:{word}"))
        state = cache.state(0.5)
        assert [state["arms"][arm]["gripper"] for arm in ("left", "right")] == raw


def test_wait_timeout_does_not_cancel_motion_and_wait_is_batch_specific():
    config = fast_config()
    config = replace(config, control=replace(config.control, publish_rate_hz=100))
    with running(config) as (controller, backend, cache):
        first = controller.submit(parse_commands("left:up:0.002"))
        second = controller.submit(parse_commands("left:up:0.1"))
        immediate = controller.wait(first["batch_id"], 0)
        assert immediate["status"] in ("queued", "running")
        assert immediate["timed_out"]
        assert controller.wait(first["batch_id"], 1)["status"] == "completed"
        assert controller.wait(second["batch_id"], 0)["status"] in ("queued", "running")


class GatedSim(RecordingSim):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.entered, self.release = threading.Event(), threading.Event()

    def publish(self, arm, target):
        if not self.entered.is_set():
            self.entered.set()
            assert self.release.wait(2)
        super().publish(arm, target)


def test_clear_queue_cancels_pending_steps_without_interrupting_active_step():
    with running(backend_cls=GatedSim) as (controller, backend, cache):
        first = controller.submit(parse_commands(["left:up:0.002", "left:up:0.1"]))
        assert backend.entered.wait(1)
        replacement = controller.submit(parse_commands("left:forward:0.001"), clear_queue=True)
        backend.release.set()
        prior = controller.wait(first["batch_id"], 1)
        assert prior["status"] == "cancelled"
        assert prior["completed_steps"] == 1 and prior["cancelled_steps"] == 1
        assert controller.wait(replacement["batch_id"], 1)["status"] == "completed"
        assert cache.state(0.5)["arms"]["left"]["end_pos"][:3] == pytest.approx([0.201, 0.1, 0.202])


class FailingSim(RecordingSim):
    def publish(self, arm, target):
        raise RuntimeError("publisher failed")


def test_publication_failure_cancels_rest_of_batch_and_resets_targets():
    with running(backend_cls=FailingSim) as (controller, backend, cache):
        batch = controller.submit(parse_commands(["left:up:0.001", "right:open"]))
        result = controller.wait(batch["batch_id"], 1)
        assert result["status"] == "failed"
        assert result["failed_steps"] == 1 and result["cancelled_steps"] == 1
        assert "publisher failed" in result["error"]
        assert controller.progress()["commanded_targets"] == {}
        assert controller.progress()["pending"] == 0


def test_readonly_controller_refuses_control_before_queueing():
    with running(actuation_enabled=False) as (controller, backend, cache):
        with pytest.raises(RuntimeError, match="disabled"):
            controller.submit(parse_commands("both:home"))
        assert backend.calls == []
        assert controller.progress()["pending"] == 0


def test_oversized_batch_is_rejected_without_partial_enqueue():
    with running(fast_config(max_pending_steps=2)) as (controller, backend, cache):
        with pytest.raises(RuntimeError, match="queue"):
            controller.submit(parse_commands(["left:up"] * 3))
        assert backend.calls == []
        assert controller.progress()["pending"] == 0


class FrozenBackend:
    name = "test"
    actuation_enabled = True
    connected = True

    def __init__(self, cache, *, emit_once=False):
        self.cache, self.calls, self.emit_once = cache, [], emit_once

    def has_subscriber(self, arm):
        return True

    def publish(self, arm, target):
        if self.emit_once and not self.calls:
            self.cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
        self.calls.append((arm, target))


@pytest.mark.parametrize("emit_once", [False, True])
def test_arrival_requires_two_new_feedback_observations(emit_once):
    from ee_control_mcp.controller import Controller

    config = fast_config()
    config = replace(config, control=replace(config.control, step_timeout_s=0.04))
    cache = ObservationCache()
    for arm in ("left", "right"):
        cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
    backend = FrozenBackend(cache, emit_once=emit_once)
    controller = Controller(config, cache, backend)
    controller.start()
    try:
        batch = controller.submit(parse_commands("left:home"))
        result = controller.wait(batch["batch_id"], 1)
        assert result["status"] == "failed"
        assert "timeout" in result["error"]
    finally:
        controller.close()


def test_feedback_expiry_during_motion_fails_and_cancels_pending_steps():
    from ee_control_mcp.controller import Controller

    config = fast_config()
    config = replace(config, observation=replace(config.observation, feedback_max_age_s=0.03))
    cache = ObservationCache()
    for arm in ("left", "right"):
        cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
    backend = FrozenBackend(cache)
    controller = Controller(config, cache, backend)
    controller.start()
    try:
        batch = controller.submit(parse_commands(["left:up:0.1", "right:open"]))
        result = controller.wait(batch["batch_id"], 1)
        assert result["status"] == "failed" and "stale" in result["error"]
        assert result["cancelled_steps"] == 1
    finally:
        controller.close()


def test_shutdown_cancels_queue_and_does_not_publish_again():
    with running() as (controller, backend, cache):
        batch = controller.submit(parse_commands(["left:up:0.2", "both:home"]))
        controller.close()
        count = len(backend.calls)
        time.sleep(0.01)
        assert len(backend.calls) == count
        assert controller.wait(batch["batch_id"], 0)["status"] == "cancelled"


class VirtualClock:
    def __init__(self, on_wait=lambda tick: None):
        self.now = 0.0
        self.ticks = 0
        self.on_wait = on_wait

    def monotonic(self):
        return self.now

    def wait(self, duration):
        self.now += duration
        self.ticks += 1
        self.on_wait(self.ticks)
        return False


def test_fixed_period_does_not_accumulate_publication_processing_time(monkeypatch):
    from ee_control_mcp import controller as module

    clock = VirtualClock()
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock.monotonic))
    cache = ObservationCache()
    cache.update_arm("left", (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
    backend = FrozenBackend(cache)
    sent_at = []

    def publish(arm, target):
        sent_at.append(clock.now)
        clock.now += 0.004  # A publisher consumes 4 ms of each 33.3 ms period.
        for _ in range(2):
            cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)

    backend.publish = publish
    controller = module.Controller(Config(), cache, backend)
    controller._stop = clock
    controller._execute(parse_commands("left:home")[0])
    assert sent_at == pytest.approx([1 / 30, 2 / 30])


def test_feedback_before_final_target_does_not_count_as_arrival(monkeypatch):
    from ee_control_mcp import controller as module

    cache = ObservationCache()
    cache.update_arm("left", (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)

    def on_wait(tick):
        if tick == 2:
            cache.update_arm("left", (0.001, 0, 0, 0, 0, 0), (0,) * 7, (0,) * 7, (0,) * 7)

    clock = VirtualClock(on_wait)
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock.monotonic))
    backend = FrozenBackend(cache)

    def publish(arm, target):
        backend.calls.append((arm, target))
        if len(backend.calls) == 3:
            # Only one genuinely new observation after the exact endpoint.
            cache.update_arm(arm, target.values, (0,) * 7, (0,) * 7, (0,) * 7)

    backend.publish = publish
    config = Config()
    config = replace(config, control=replace(config.control, step_timeout_s=0.2))
    controller = module.Controller(config, cache, backend)
    controller._stop = clock
    with pytest.raises(RuntimeError, match="timeout"):
        controller._execute(parse_commands("left:forward:0.002")[0])


def test_intervening_out_of_tolerance_feedback_breaks_consecutive_arrival(monkeypatch):
    from ee_control_mcp import controller as module

    clock = VirtualClock()
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock.monotonic))
    cache = ObservationCache()
    cache.update_arm("left", (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
    backend = FrozenBackend(cache)

    def publish(arm, target):
        # Feedback arrives faster than publications. No adjacent pair settles.
        cache.update_arm(arm, (0,) * 6, (0.2, 0, 0, 0, 0, 0, 0), (0,) * 7, (0,) * 7)
        cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)

    backend.publish = publish
    config = Config()
    config = replace(config, control=replace(config.control, step_timeout_s=0.2))
    controller = module.Controller(config, cache, backend)
    controller._stop = clock
    with pytest.raises(RuntimeError, match="timeout"):
        controller._execute(parse_commands("left:home")[0])


def test_parallel_arm_feedback_before_its_endpoint_publication_is_excluded(monkeypatch):
    from ee_control_mcp import controller as module

    clock = VirtualClock()
    monkeypatch.setattr(module, "time", SimpleNamespace(monotonic=clock.monotonic))
    cache = ObservationCache()
    for arm in ("left", "right"):
        cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
    backend = FrozenBackend(cache)

    def publish(arm, target):
        backend.calls.append((arm, target))
        if arm == "left":
            cache.update_arm(arm, (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
        if len(backend.calls) == 1:
            # Right feedback is received while publishing left, before right's endpoint.
            cache.update_arm("right", (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)
        if len(backend.calls) == 4:
            # Right produces exactly one observation after its endpoint.
            cache.update_arm("right", (0,) * 6, (0,) * 7, (0,) * 7, (0,) * 7)

    backend.publish = publish
    config = Config()
    config = replace(config, control=replace(config.control, step_timeout_s=0.2))
    controller = module.Controller(config, cache, backend)
    controller._stop = clock
    with pytest.raises(RuntimeError, match="timeout"):
        controller._execute(parse_commands("both:home")[0])
