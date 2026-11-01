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

        # Only a halt that sees the robot still releases it.
        with odometry_streaming(backend):
            released = backend.safe_stop("halt-2")
        assert released.outcome == OUTCOME_COMPLETED
        assert backend.halt_hold() is None
        assert released.evidence["halt_confirmation"]["released_hold"]["goals"] == ["nav-1"]
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
