"""A halt is completed only on a robot seen still, and an unconfirmed one holds.

``motion.halt`` must never report ``completed`` for a robot nobody observed
stop. "No goal was tracked" is not "still": the adapter may have lost the goal
on a reconnect, or something else may be driving the base. So every halt waits
for fresh odometry that shows the robot still, and anything less is ``failed``
with ``reason_code: effect_unknown``.

An unconfirmed halt arms a hold that refuses motion. The hold is the robot's,
not one connection's: a reconnect (or a new adapter on this computer) keeps
it, re-sends the cancel of the goals it holds and zero velocity, and only a
later halt that sees the robot still releases it.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import pytest
from test_motion_outcome import CancelRosbridge, odometry_streaming
from test_observation_on_state import (  # noqa: F401 - pytest fixtures
    clock,
    fake_rclpy,
    odometry,
    rclpy_backend,
)

from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend


@pytest.fixture(autouse=True)
def short_still_wait(monkeypatch):
    monkeypatch.setattr(adapter_module, "HALT_STILL_WAIT_S", 0.3)
    monkeypatch.setattr(adapter_module, "STILL_MIN_SECONDS", 0.1)
    monkeypatch.setattr(adapter_module, "HELD_GOAL_END_WAIT_S", 0.3)
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")


def rosbridge(answers: bool):
    """A backend whose every (re)connection is a new CancelRosbridge socket."""
    sockets: list[CancelRosbridge] = []

    def connect(_url):
        socket = CancelRosbridge()
        socket.answers = answers
        sockets.append(socket)
        return socket

    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=connect)
    backend._topic_types = {"/cmd_vel": "geometry_msgs/msg/Twist"}
    return backend, sockets


def cancels(socket) -> list[str]:
    return [m["id"] for m in list(socket.sent) if m.get("op") == "cancel_action_goal"]


def advance(adapter, call_id="adv-1"):
    return adapter.invoke(CallRequest(call_id, "motion.advance", {"distance_m": 0.3}))


# --- 1. no goal tracked is not "still" ---------------------------------------


def test_a_halt_with_no_goal_and_no_odometry_is_effect_unknown():
    backend, _sockets = rosbridge(answers=True)
    try:
        result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    # Was: completed after one zero publish, with nothing observed.
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == "effect_unknown"
    assert result.evidence["effect_unknown"] is True
    stillness = result.evidence["halt_confirmation"]["stillness"]
    assert stillness["stationary"] is False and stillness["odometry_messages"] == 0
    assert "no fresh odometry" in result.detail


def test_a_halt_with_no_goal_on_a_moving_robot_is_effect_unknown():
    backend, _sockets = rosbridge(answers=True)
    try:
        with odometry_streaming(backend, linear=0.2):
            result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["reason_code"] == "effect_unknown"
    assert result.evidence["halt_confirmation"]["stillness"]["odometry_messages"] > 0


def test_a_halt_with_no_goal_completes_on_a_robot_seen_still():
    backend, _sockets = rosbridge(answers=True)
    try:
        with odometry_streaming(backend):
            result = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["halt_confirmation"]["stillness"]["stationary"] is True


def test_a_halt_not_seen_still_arms_the_hold_and_refuses_motion():
    backend, _sockets = rosbridge(answers=True)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    try:
        assert backend.safe_stop("halt-1").outcome == OUTCOME_FAILED
        assert backend.halt_hold() is not None
        refused = advance(adapter)
        assert refused.outcome == OUTCOME_REFUSED
        assert refused.detail.startswith("effect_unknown")
    finally:
        adapter.disconnect()


# --- 2. the hold survives a reconnect ------------------------------------------


def test_an_unconfirmed_halt_survives_a_rosbridge_reconnect():
    backend, sockets = rosbridge(answers=False)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    backend._active_actions["nav-1"] = "/navigate_to_pose"
    try:
        held = backend.safe_stop("halt-1")
        assert held.outcome == OUTCOME_FAILED

        backend.reconnect()

        # Goal ids belong to the old socket, but the hold is the robot's.
        assert backend._active_actions == {}
        hold = backend.halt_hold()
        assert hold is not None and "nav-1" in hold["goals"]
        # The new connection re-sends the held goal's cancel and zero velocity
        # at once, and keeps sending zero.
        fresh = sockets[-1]
        assert cancels(fresh) == ["nav-1"]
        zeros = fresh.zeros()
        assert zeros >= 1
        time.sleep(0.35)
        assert fresh.zeros() >= zeros + 2
        # And motion is still refused.
        refused = advance(adapter)
        assert refused.outcome == OUTCOME_REFUSED
        assert refused.detail.startswith("effect_unknown") and "nav-1" in refused.detail
        assert not [m for m in fresh.sent if m.get("op") == "send_action_goal"]

        # A halt that does not see the robot (no odometry) keeps the hold as it was.
        again = backend.safe_stop("halt-unseen")
        assert again.outcome == OUTCOME_FAILED
        assert again.evidence["halt_confirmation"]["reissued_cancels"] == ["nav-1"]
        assert backend.halt_hold()["goals"] == ["nav-1"]

        # A robot seen still is not enough while the held goal from the old
        # connection has no confirmed end: Nav2 could resume it after a release.
        with odometry_streaming(backend):
            orphaned = backend.safe_stop("halt-orphan")
        assert orphaned.outcome == OUTCOME_FAILED
        assert orphaned.evidence["reason_code"] == "effect_unknown"
        confirmation = orphaned.evidence["halt_confirmation"]
        assert confirmation["stillness"]["stationary"] is True
        assert "nav-1" in confirmation["held_goals_unconfirmed"]
        assert backend.halt_hold()["goals"] == ["nav-1"]

        # The action server answers its cancel-all but a goal is still canceling.
        fresh.answers = True
        fresh.still_canceling = [{"goal_id": {"uuid": [1] * 16}}]
        with odometry_streaming(backend):
            canceling = backend.safe_stop("halt-canceling")
        assert canceling.outcome == OUTCOME_FAILED
        assert "nav-1" in canceling.evidence["halt_confirmation"]["held_goals_unconfirmed"]
        assert backend.halt_hold() is not None

        # Only a halt that sees the robot still, with every held goal's end
        # confirmed by its action server, releases it.
        fresh.still_canceling = []
        with odometry_streaming(backend):
            released = backend.safe_stop("halt-2")
        assert released.outcome == OUTCOME_COMPLETED
        assert backend.halt_hold() is None
        assert released.evidence["halt_confirmation"]["released_hold"]["goals"] == ["nav-1"]
        assert released.evidence["halt_confirmation"]["held_goals_ended"] == ["nav-1"]
        cancel_all = [
            m for m in fresh.sent
            if m.get("op") == "call_service"
            and m["service"] == "/navigate_to_pose/_action/cancel_goal"
        ]
        assert cancel_all and cancel_all[-1]["type"] == "action_msgs/srv/CancelGoal"
        assert cancel_all[-1]["args"]["goal_info"]["goal_id"]["uuid"] == [0] * 16
        time.sleep(0.25)
        settled = fresh.zeros()
        time.sleep(0.3)
        assert fresh.zeros() == settled
    finally:
        adapter.disconnect()


def test_the_hold_refuses_motion_on_another_connection_to_the_robot():
    first, _sockets = rosbridge(answers=True)
    second, other_sockets = rosbridge(answers=True)
    adapter = GenericROS2Adapter(backend=second, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    try:
        assert first.safe_stop("halt-1").outcome == OUTCOME_FAILED
        first.disconnect()
        refused = advance(adapter)
        assert refused.outcome == OUTCOME_REFUSED
        assert refused.detail.startswith("effect_unknown")
        second.reconnect()
        assert other_sockets[-1].zeros() >= 1
    finally:
        adapter.disconnect()


def test_a_reconnect_without_a_hold_sends_nothing():
    backend, sockets = rosbridge(answers=True)
    try:
        backend.reconnect()
        assert sockets[-1].zeros() == 0
        assert cancels(sockets[-1]) == []
    finally:
        backend.disconnect()


class _Handle:
    def __init__(self, calls: list[str], *, raises: BaseException | None = None) -> None:
        self.calls = calls
        self.raises = raises

    def cancel_goal_async(self):
        self.calls.append("cancel")
        if self.raises is not None:
            raise self.raises
        return SimpleNamespace(
            done=lambda: True, result=lambda: SimpleNamespace(goals_canceling=())
        )


def test_an_unconfirmed_rclpy_halt_survives_a_reconnect(fake_rclpy, clock):  # noqa: F811
    backend, _condition = rclpy_backend(clock)
    calls: list[str] = []
    backend._publish_zero = lambda: calls.append("zero") or True
    handle = _Handle(calls)
    backend._goal_handles["old-call"] = handle
    backend._result_futures["old-call"] = SimpleNamespace(done=lambda: False)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    try:
        assert backend.safe_stop("stop-1").outcome == OUTCOME_FAILED
        calls.clear()

        backend.reconnect()

        assert backend._goal_handles == {}
        assert backend.halt_hold() is not None
        # The kept handle is asked to cancel again, and zero goes out.
        assert "cancel" in calls and "zero" in calls
        refused = advance(adapter)
        assert refused.outcome == OUTCOME_REFUSED
        assert refused.detail.startswith("effect_unknown")
    finally:
        adapter.disconnect()


# --- 3. an rclpy cancel that raises is guarded -----------------------------


class _InvalidHandle(Exception):
    """What rclpy raises for a goal handle whose client or node is gone."""


def test_an_rclpy_cancel_that_raises_is_effect_unknown_and_held(fake_rclpy, clock):  # noqa: F811
    backend, condition = rclpy_backend(clock)
    calls: list[str] = []
    backend._publish_zero = lambda: calls.append("zero") or True
    handle = _Handle(calls, raises=_InvalidHandle("goal handle is invalid"))
    backend._goal_handles["old-call"] = handle
    backend._result_futures["old-call"] = SimpleNamespace(done=lambda: False)
    callback = backend._node.callbacks["/odom"] if backend._node else None
    if callback is not None:
        condition.feed = [lambda: callback(odometry())] * 5
    try:
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_FAILED
        assert result.evidence["reason_code"] == "effect_unknown"
        unconfirmed = result.evidence["halt_confirmation"]["unconfirmed"]
        assert "goal handle is invalid" in unconfirmed["old-call"]
        assert backend._goal_handles == {"old-call": handle}
        assert backend.halt_hold() is not None
    finally:
        backend.disconnect()


# --- 4. a halt never fails open on a transport error -------------------------
#
# The socket breaking is the moment a halt most needs to hold. Whatever a send
# raises (RuntimeError, OSError, websockets' ConnectionClosed), the halt ends
# ``failed`` / ``effect_unknown`` with the hold armed; it never raises out of
# ``safe_stop`` and leaves motion allowed after a reconnect.


class _BrokenOn(CancelRosbridge):
    """A socket whose send raises ``error`` for messages ``matches`` picks."""

    def __init__(self, matches, error):
        super().__init__()
        self.matches = matches
        self.error = error

    def send(self, raw):
        import json

        if self.matches(json.loads(raw)):
            raise self.error
        super().send(raw)


def _broken_backend(matches, error):
    socket = _BrokenOn(matches, error)
    socket.answers = True
    backend = RosbridgeROS2Backend(url="ws://127.0.0.1:1", connection_factory=lambda _u: socket)
    backend._topic_types = {"/cmd_vel": "geometry_msgs/msg/Twist"}
    return backend, socket


def _is_cancel(message):
    return message.get("op") == "cancel_action_goal"


def _is_zero(message):
    return message.get("op") == "publish" and message.get("topic") == "/cmd_vel"


@pytest.mark.parametrize(
    "error", [OSError("socket broken"), RuntimeError("rosbridge adapter disconnected")]
)
def test_a_rosbridge_cancel_that_raises_is_an_unconfirmed_cancel_and_held(error):
    backend, _socket = _broken_backend(_is_cancel, error)
    backend._active_actions["nav-1"] = "/navigate_to_pose"
    try:
        with odometry_streaming(backend):
            result = backend.safe_stop("halt-1")
        assert result.outcome == OUTCOME_FAILED
        assert result.evidence["reason_code"] == "effect_unknown"
        # The halt went on past the failed cancel: it is that goal's reason.
        unconfirmed = result.evidence["halt_confirmation"]["unconfirmed"]
        assert unconfirmed["nav-1"].startswith("cancel not sent")
        assert str(error) in unconfirmed["nav-1"]
        assert result.evidence["zero_velocity_published"] is True
        hold = backend.halt_hold()
        assert hold is not None and hold["goals"] == ["nav-1"]
    finally:
        backend.disconnect()


def test_a_halt_whose_zero_velocity_cannot_be_published_is_held_even_if_still():
    backend, _socket = _broken_backend(_is_zero, OSError("socket broken"))
    try:
        with odometry_streaming(backend):
            result = backend.safe_stop("halt-1")
        assert result.outcome == OUTCOME_FAILED
        assert result.evidence["reason_code"] == "effect_unknown"
        assert result.evidence["zero_velocity_published"] is False
        assert "standard cmd_vel stop unavailable" in result.detail
        assert backend.halt_hold() is not None
    finally:
        backend.disconnect()


def test_a_halt_that_raises_anywhere_is_effect_unknown_and_held():
    backend, _socket = rosbridge(answers=True)
    backend._active_actions["nav-1"] = "/navigate_to_pose"

    def broken(_seconds):
        raise ConnectionResetError("socket broken while waiting")

    backend.wait_until_stationary = broken
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    try:
        result = backend.safe_stop("halt-1")
        assert result.outcome == OUTCOME_FAILED
        assert result.evidence["reason_code"] == "effect_unknown"
        assert result.evidence["effect_unknown"] is True
        assert "ConnectionResetError" in result.detail
        assert backend.halt_hold() is not None
        refused = advance(adapter)
        assert refused.outcome == OUTCOME_REFUSED
        assert refused.detail.startswith("effect_unknown")
    finally:
        adapter.disconnect()


def test_the_zero_hold_keeps_running_through_a_failed_publish():
    backend, sockets = rosbridge(answers=True)
    socket = sockets[-1]
    fails = {"left": 3}
    real_send = socket.send

    def flaky(raw):
        import json

        if _is_zero(json.loads(raw)) and fails["left"] > 0:
            fails["left"] -= 1
            raise OSError("socket hiccup")
        real_send(raw)

    try:
        assert backend.safe_stop("halt-1").outcome == OUTCOME_FAILED  # no odometry
        socket.send = flaky
        fails["left"] = 3
        time.sleep(0.5)
        assert fails["left"] == 0
        zeros = socket.zeros()
        time.sleep(0.35)
        # The holder thread outlived the errors and keeps sending zero.
        assert socket.zeros() >= zeros + 2
        assert backend._zero_holder is not None and backend._zero_holder.is_alive()
    finally:
        backend.disconnect()


# --- 5. an rclpy goal held from an earlier connection needs a confirmed end -----


class _EndedHandle(_Handle):
    """A kept goal handle whose action server answers its cancel with ``code``."""

    def __init__(self, calls, code):
        super().__init__(calls)
        self.code = code

    def cancel_goal_async(self):
        self.calls.append("cancel")
        return SimpleNamespace(
            done=lambda: True,
            result=lambda: SimpleNamespace(return_code=self.code, goals_canceling=()),
        )


@pytest.mark.parametrize(("code", "released"), [(3, True), (2, True), (0, False), (1, False)])
def test_an_rclpy_held_goal_releases_only_on_a_confirmed_end(
    fake_rclpy, clock, code, released  # noqa: F811
):
    backend, condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    calls: list[str] = []
    adapter_module.HALT_HOLD.arm("halt not confirmed", {"old-call": _EndedHandle(calls, code)})
    backend.wait_until_stationary = lambda _s: {
        "stationary": True, "drifting": False, "still_samples": 3,
        "odometry_messages": 5, "waited_seconds": 0.1,
    }
    try:
        result = backend.safe_stop("stop-1")
        confirmation = result.evidence["halt_confirmation"]
        if released:
            assert result.outcome == OUTCOME_COMPLETED
            assert confirmation["held_goals_ended"] == ["old-call"]
            assert backend.halt_hold() is None
        else:
            assert result.outcome == OUTCOME_FAILED
            assert result.evidence["reason_code"] == "effect_unknown"
            assert "old-call" in confirmation["held_goals_unconfirmed"]
            assert backend.halt_hold()["goals"] == ["old-call"]
    finally:
        backend.disconnect()


def test_a_held_goal_with_no_kept_handle_keeps_the_hold(fake_rclpy, clock):  # noqa: F811
    # A restarted rclpy host keeps no goal handle: nothing can confirm the end.
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    adapter_module.HALT_HOLD.arm("halt not confirmed", {"old-call": None})
    backend.wait_until_stationary = lambda _s: {
        "stationary": True, "drifting": False, "still_samples": 3,
        "odometry_messages": 5, "waited_seconds": 0.1,
    }
    try:
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_FAILED
        assert "old-call" in result.evidence["halt_confirmation"]["held_goals_unconfirmed"]
        assert backend.halt_hold() is not None
    finally:
        backend.disconnect()
