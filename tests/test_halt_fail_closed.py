"""A halt ends ``completed`` only on a robot observed still, and never fails open.

Review round 5. What is pinned here:

* Stillness needs BOTH the odometry twist (every axis present and near zero;
  a missing axis is unknown, never 0) AND the pose, which must not move
  across the samples, for at least ``STILL_MIN_SECONDS`` of observed odometry
  (not only a count of messages). A robot whose pose advances while its twist
  reads zero, moves sideways in ``linear.y``, or sends empty twist vectors is
  never reported still.
* A reconnect carries every goal still running into the hold, with its cancel
  token, so a later halt must cancel it and confirm it ended before it can
  complete.
* Every halt that cannot run (no time, ``motion.halt`` not on the graph, the
  link down, a presence connection, an operator refresh in progress) ends
  ``failed`` / ``effect_unknown``, arms the hold, starts a new halt epoch and
  keeps publishing zero where the connection can.
* The kept hold file that cannot be read (any ``OSError``) holds.
* A reconnect while held restarts the continuous zero; an rclpy backend that
  starts held does the same at once.
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest
from test_halt_hold import cancels, rosbridge
from test_motion_outcome import odometry_streaming
from test_observation_on_state import (  # noqa: F401 - pytest fixtures
    ODOM_PERIOD,
    FeedingCondition,
    clock,
    fake_rclpy,
    odometry,
    rclpy_backend,
)

from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RclpyROS2Backend

# The real constants, read before any fixture patches them.
REAL_STILL_MIN_SECONDS = getattr(adapter_module, "STILL_MIN_SECONDS", 0.0)
REAL_HALT_STILL_WAIT_S = adapter_module.HALT_STILL_WAIT_S

FULL_STILL_TWIST = {
    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
}


@pytest.fixture(autouse=True)
def short_waits(monkeypatch):
    monkeypatch.setattr(adapter_module, "HALT_STILL_WAIT_S", 0.6)
    monkeypatch.setattr(adapter_module, "HELD_GOAL_END_WAIT_S", 0.3)
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.2, raising=False)
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")


def _odom_message(x, y, twist):
    return {
        "op": "publish",
        "topic": "/odom",
        "msg": {
            "pose": {
                "pose": {
                    "position": {"x": x, "y": y},
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                }
            },
            "twist": {"twist": twist},
        },
    }


class _Drive:
    """Odometry at 50 Hz whose pose advances 0.5 m/s along ``axis``."""

    def __init__(self, backend, twist, axis="x", speed=0.5):
        self.stop = threading.Event()

        def run():
            travelled = 0.0
            while not self.stop.is_set():
                travelled += speed * 0.02
                x, y = (travelled, 0.0) if axis == "x" else (0.0, travelled)
                backend._handle_message(_odom_message(x, y, twist))
                self.stop.wait(0.02)

        self.thread = threading.Thread(target=run, daemon=True)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *_exc):
        self.stop.set()
        self.thread.join()


# --- 1. stillness is the full twist AND the pose -----------------------------


@pytest.mark.parametrize(
    ("twist", "axis"),
    [
        # A driver that publishes a zero twist while the pose advances.
        (FULL_STILL_TWIST, "x"),
        # Sideways motion: only linear.x and angular.z were read.
        (
            {"linear": {"x": 0.0, "y": 0.5, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
            "y",
        ),
        # Empty vectors: a missing field is unknown, never 0.
        ({"linear": {}, "angular": {}}, "x"),
    ],
    ids=["zero_twist_pose_advancing", "lateral_linear_y", "empty_twist"],
)
def test_a_halt_on_a_robot_whose_pose_advances_is_never_completed(twist, axis):
    backend, _sockets = rosbridge(answers=True)
    try:
        with _Drive(backend, twist, axis):
            result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == "effect_unknown"
    stillness = result.evidence["halt_confirmation"]["stillness"]
    assert stillness["stationary"] is False
    if twist["linear"]:
        # Every axis was there: the pose (or linear.y) refused, not a gap.
        assert stillness["unknown_twist_messages"] == 0
    else:
        assert stillness["unknown_twist_messages"] == stillness["odometry_messages"]
    assert adapter_module.HALT_HOLD.snapshot() is not None


@pytest.mark.parametrize(
    "twist",
    [
        {"linear": {"x": 0.0, "y": 0.2, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.3}},
        {"linear": {"x": 0.0, "y": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0}},
        {"linear": {"x": 0.0, "y": "fast", "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.2}, "angular": {"x": 0.0, "y": 0.0, "z": 0.0}},
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.3, "y": 0.0, "z": 0.0}},
        {"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.3, "z": 0.0}},
        # A twist with neither vector: nothing is known, not "all zero".
        {},
    ],
    ids=[
        "linear_y",
        "angular_z",
        "missing_linear_z",
        "missing_angular_z",
        "not_a_number",
        "linear_z",
        "angular_x",
        "angular_y",
        "no_vectors",
    ],
)
def test_a_twist_with_any_axis_moving_or_unknown_is_not_still(twist):
    # The pose stays put: the twist alone must refuse.
    backend, _sockets = rosbridge(answers=True)
    try:
        with _Drive(backend, twist, speed=0.0):
            result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["halt_confirmation"]["stillness"]["stationary"] is False


def test_a_robot_with_a_full_still_twist_and_a_still_pose_completes():
    backend, _sockets = rosbridge(answers=True)
    try:
        with _Drive(backend, FULL_STILL_TWIST, speed=0.0):
            result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_COMPLETED
    stillness = result.evidence["halt_confirmation"]["stillness"]
    assert stillness["stationary"] is True
    assert stillness["still_seconds"] >= adapter_module.STILL_MIN_SECONDS


def test_odometry_with_no_twist_at_all_is_never_still(clock):  # noqa: F811
    # The pose alone cannot show every axis is still.
    backend = adapter_module.RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _u: _Quiet()
    )
    condition = FeedingCondition(clock)
    backend._condition = condition

    def pose_only():
        return {"pose": {"pose": {"position": {"x": 0.1, "y": 0.0}, "orientation": {"w": 1.0}}}}

    condition.feed = [
        (lambda: backend._update_odometry(pose_only(), clock.now)) for _ in range(60)
    ]
    try:
        settled = backend.wait_until_stationary(1.0)
    finally:
        backend.disconnect()
    assert settled["stationary"] is False
    assert settled["odometry_messages"] > 3


class _Quiet:
    def __init__(self):
        import queue

        self.inbox: queue.Queue = queue.Queue()

    def send(self, _payload):
        return None

    def recv(self, timeout=None):
        import queue

        try:
            return self.inbox.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError from None

    def close(self):
        self.inbox.put(None)

    def pong(self, _data=b""):
        return None


# --- 2. stillness needs a minimum observed duration ---------------------------


def test_three_still_messages_in_a_moment_are_not_still(fake_rclpy, clock, monkeypatch):  # noqa: F811
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.5, raising=False)
    backend, condition = rclpy_backend(clock)
    # Still for about 0.2 s (a Nav2 pause, the start of a rotate), then moving.
    still = [lambda: backend._on_odometry(odometry(0.1)) for _ in range(6)]
    moving = [
        (lambda step=step: backend._on_odometry(odometry(0.1 + 0.01 * step, linear=0.3)))
        for step in range(1, 40)
    ]
    condition.feed = still + moving
    settled = backend.wait_until_stationary(2.0)
    assert settled["stationary"] is False
    backend.disconnect()


def test_stillness_is_reported_only_after_the_minimum_duration(fake_rclpy, clock, monkeypatch):  # noqa: F811
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.5, raising=False)
    backend, condition = rclpy_backend(clock)
    condition.feed = [lambda: backend._on_odometry(odometry(0.1)) for _ in range(60)]
    started = clock.now
    settled = backend.wait_until_stationary(2.0)
    assert settled["stationary"] is True
    # Not after STILL_SAMPLES messages (0.1 s): after 0.5 s of them.
    assert clock.now - started >= 0.5
    assert settled["still_seconds"] >= 0.5
    assert condition.delivered < 0.5 / ODOM_PERIOD + 4
    backend.disconnect()


def test_a_slow_creep_across_the_window_is_not_still(fake_rclpy, clock, monkeypatch):  # noqa: F811
    # 1 mm per message: under the per-message threshold, but the window drifts.
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.5, raising=False)
    backend, condition = rclpy_backend(clock)
    condition.feed = [
        (lambda step=step: backend._on_odometry(odometry(0.1 + 0.001 * step)))
        for step in range(60)
    ]
    settled = backend.wait_until_stationary(1.5)
    assert settled["stationary"] is False
    backend.disconnect()


def test_still_messages_cut_off_by_the_cap_before_the_minimum_duration_are_not_still(
    fake_rclpy, clock, monkeypatch  # noqa: F811
):
    # Enough still messages, but the cap ends the wait before they span
    # STILL_MIN_SECONDS: the count alone never makes the robot still.
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.5, raising=False)
    backend, condition = rclpy_backend(clock)
    condition.feed = [lambda: backend._on_odometry(odometry(0.1)) for _ in range(60)]
    settled = backend.wait_until_stationary(0.3)
    assert settled["still_samples"] >= adapter_module.STILL_SAMPLES
    assert settled["still_seconds"] < 0.5
    assert settled["stationary"] is False and settled["drifting"] is True
    backend.disconnect()


def test_a_twist_with_neither_vector_is_unknown_not_zero():
    assert adapter_module.odometry_twist_axes({}) is None
    assert adapter_module.odometry_twist_axes(SimpleNamespace()) is None
    assert adapter_module.odometry_twist_axes({"linear": {}, "angular": {}}) == (None,) * 6


def test_the_halt_waits_for_the_minimum_duration_with_the_real_constants():
    assert REAL_STILL_MIN_SECONDS >= 0.5
    assert REAL_HALT_STILL_WAIT_S > REAL_STILL_MIN_SECONDS


# --- 3. a reconnect carries running goals into the hold ---------------------------


def test_a_goal_running_at_a_rosbridge_reconnect_is_held_and_must_end_before_a_halt_completes():
    backend, sockets = rosbridge(answers=True)
    backend._active_actions["nav-1"] = "/navigate_to_pose"
    backend._goal_ids["nav-1"] = "nav-1"
    try:
        backend.reconnect()
        hold = adapter_module.HALT_HOLD.snapshot()
        assert hold is not None and hold["goals"] == ["nav-1"]
        assert adapter_module.HALT_HOLD.tokens()["nav-1"] == ("nav-1", "/navigate_to_pose")
        fresh = sockets[-1]
        # The goal's cancel is re-sent on the new connection at once.
        assert cancels(fresh) == ["nav-1"]
        # The action server still runs a goal: the halt stays effect_unknown.
        fresh.still_canceling = [{"goal_id": {"uuid": [1] * 16}}]
        with odometry_streaming(backend):
            held = backend.safe_stop("halt-1")
        assert held.outcome == OUTCOME_FAILED
        assert "nav-1" in held.evidence["halt_confirmation"]["held_goals_unconfirmed"]
        # Once its server confirms nothing runs, a halt seen still completes.
        fresh.still_canceling = []
        with odometry_streaming(backend):
            released = backend.safe_stop("halt-2")
        assert released.outcome == OUTCOME_COMPLETED
        assert released.evidence["halt_confirmation"]["held_goals_ended"] == ["nav-1"]
        cancel_all = [
            m
            for m in fresh.sent
            if m.get("op") == "call_service"
            and m["service"] == "/navigate_to_pose/_action/cancel_goal"
        ]
        assert cancel_all
    finally:
        backend.disconnect()


def test_a_goal_that_ended_before_the_reconnect_is_not_held():
    backend, _sockets = rosbridge(answers=True)
    backend._active_actions["nav-1"] = "/navigate_to_pose"
    backend._goal_ended["nav-1"] = 4
    try:
        backend.reconnect()
        assert adapter_module.HALT_HOLD.snapshot() is None
    finally:
        backend.disconnect()


class _Handle:
    def __init__(self, code=3):
        self.code = code
        self.cancels = 0

    def cancel_goal_async(self):
        self.cancels += 1
        return SimpleNamespace(
            done=lambda: True,
            result=lambda: SimpleNamespace(return_code=self.code, goals_canceling=()),
        )


def test_a_goal_running_at_an_rclpy_reconnect_is_held(fake_rclpy, clock):  # noqa: F811
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    running = _Handle()
    backend._goal_handles["nav-1"] = running
    backend._result_futures["nav-1"] = SimpleNamespace(done=lambda: False)
    pending = SimpleNamespace(done=lambda: False)
    backend._pending_goals["nav-2"] = pending
    ended = _Handle()
    backend._goal_handles["nav-3"] = ended
    backend._result_futures["nav-3"] = SimpleNamespace(done=lambda: True)
    try:
        backend.reconnect()
        assert adapter_module.HALT_HOLD.snapshot()["goals"] == ["nav-1", "nav-2"]
        tokens = adapter_module.HALT_HOLD.tokens()
        assert tokens["nav-1"] is running and tokens["nav-2"] is pending
        # The running goal's cancel is re-sent at once.
        assert running.cancels >= 1
    finally:
        backend.disconnect()


# --- 4. every halt that cannot run holds and ends effect_unknown -------------------


def _assert_held_halt(result):
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == "effect_unknown"
    assert result.evidence["effect_unknown"] is True
    assert result.detail.startswith("effect_unknown")
    assert adapter_module.HALT_HOLD.snapshot() is not None


def _adapter(declared=frozenset({"motion.advance", "motion.halt"})):
    backend, sockets = rosbridge(answers=True)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = set(declared)
    return adapter, backend, sockets


def test_a_halt_with_no_time_holds_and_still_publishes_zero():
    adapter, backend, sockets = _adapter()
    adapter_module.HALT_HOLD.admit("adv-1")
    epoch = adapter_module.HALT_HOLD._epoch
    try:
        result = adapter.invoke(CallRequest("h", "motion.halt", {}, deadline_seconds=0))
        _assert_held_halt(result)
        assert adapter_module.HALT_HOLD._epoch > epoch
        # Zero went out, and keeps going out while held.
        zeros = sockets[-1].zeros()
        assert zeros >= 1
        time.sleep(0.35)
        assert sockets[-1].zeros() >= zeros + 2
    finally:
        adapter.disconnect()


def test_a_halt_missing_from_the_graph_holds():
    adapter, backend, _sockets = _adapter({"motion.advance"})
    adapter.describe = lambda: ()
    epoch = adapter_module.HALT_HOLD._epoch
    try:
        _assert_held_halt(adapter.invoke(CallRequest("h", "motion.halt", {})))
        assert adapter_module.HALT_HOLD._epoch > epoch
    finally:
        adapter.disconnect()


def test_a_halt_with_arguments_it_does_not_take_still_halts():
    # A stop is never refused over its arguments: it runs and is judged.
    adapter, backend, _sockets = _adapter()
    try:
        with odometry_streaming(backend):
            result = adapter.invoke(CallRequest("h", "motion.halt", {"speed": "fast"}))
        assert result.outcome == OUTCOME_COMPLETED
    finally:
        adapter.disconnect()


def _down_transport():
    return SimpleNamespace(
        connected=False,
        status=lambda: {"transport": "ssh", "state": "failed"},
        config=None,
    )


def test_a_halt_through_a_forward_that_is_down_holds():
    adapter, _backend, _sockets = _adapter()
    adapter.transport = _down_transport()
    epoch = adapter_module.HALT_HOLD._epoch
    try:
        result = adapter.invoke(CallRequest("h", "motion.halt", {}))
        _assert_held_halt(result)
        assert result.evidence["transport_unavailable"] is True
        assert adapter_module.HALT_HOLD._epoch > epoch
    finally:
        adapter.transport = None
        adapter.disconnect()


def test_a_safe_stop_through_a_forward_that_is_down_holds():
    adapter, _backend, _sockets = _adapter()
    adapter.transport = _down_transport()
    epoch = adapter_module.HALT_HOLD._epoch
    try:
        _assert_held_halt(adapter.safe_stop())
        assert adapter_module.HALT_HOLD._epoch > epoch
    finally:
        adapter.transport = None
        adapter.disconnect()


def test_a_halt_on_a_presence_connection_holds_and_publishes_nothing():
    backend, sockets = rosbridge(answers=True)
    backend._presence_only = True
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    try:
        _assert_held_halt(adapter.invoke(CallRequest("h", "motion.halt", {})))
        time.sleep(0.25)
        assert sockets[-1].zeros() == 0
    finally:
        adapter.disconnect()


def test_a_halt_during_an_operator_refresh_holds():
    adapter, _backend, _sockets = _adapter()
    adapter._gate.pause()
    try:
        _assert_held_halt(adapter.invoke(CallRequest("h", "motion.halt", {})))
    finally:
        adapter._gate.resume()
        adapter.disconnect()


def test_a_motion_admitted_before_a_halt_that_could_not_run_is_never_sent():
    adapter, backend, sockets = _adapter()
    adapter_module.HALT_HOLD.admit("adv-1")
    try:
        adapter.invoke(CallRequest("h", "motion.halt", {}, deadline_seconds=0))
        # Even after a later halt released the hold, adv-1 is from an old epoch.
        assert adapter_module.HALT_HOLD.release() is not None
        assert adapter_module.HALT_HOLD.motion_refusal("adv-1") is not None
    finally:
        adapter.disconnect()


# --- 5. the kept hold file: any read error holds ------------------------------------


def test_a_kept_hold_path_that_is_a_directory_holds(tmp_path, monkeypatch):
    folder = tmp_path / "hold-is-a-directory"
    folder.mkdir()
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(folder))
    hold = adapter_module._HaltHold()
    snapshot = hold.snapshot()
    assert snapshot is not None and "cannot be read" in snapshot["reason"]


def test_a_kept_hold_file_with_an_io_error_holds(tmp_path, monkeypatch):
    path = tmp_path / "hold.json"
    path.write_text("{}", encoding="utf-8")
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(path))
    real = adapter_module.Path.read_text

    def broken(self, *args, **kwargs):
        if self == path:
            raise PermissionError(13, "Permission denied")
        return real(self, *args, **kwargs)

    monkeypatch.setattr(adapter_module.Path, "read_text", broken)
    assert adapter_module._HaltHold().snapshot() is not None


# --- 6. the continuous zero restarts with every held connection ------------------------


def test_a_reconnect_while_held_restarts_the_continuous_zero():
    backend, sockets = rosbridge(answers=True)
    try:
        adapter_module.HALT_HOLD.arm("halt not confirmed", {})
        backend.reconnect()
        fresh = sockets[-1]
        first = fresh.zeros()
        time.sleep(0.45)
        # More than the one-off zeros of the resume: the hold keeps sending.
        assert fresh.zeros() >= first + 3
        assert backend._zero_holder is not None and backend._zero_holder.is_alive()
    finally:
        backend.disconnect()


def test_an_rclpy_backend_that_starts_held_sends_zero_at_once(fake_rclpy, monkeypatch):  # noqa: F811
    zeros: list[float] = []
    monkeypatch.setattr(
        RclpyROS2Backend, "_publish_zero", lambda self: zeros.append(time.monotonic()) or True
    )
    adapter_module.HALT_HOLD.arm("kept from before a restart", {})
    backend = RclpyROS2Backend()
    try:
        assert len(zeros) >= 1
        time.sleep(0.35)
        assert len(zeros) >= 3
        assert backend._zero_holder is not None and backend._zero_holder.is_alive()
    finally:
        backend.disconnect()
