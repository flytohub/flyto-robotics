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

import json
import time
from types import SimpleNamespace

import pytest
from test_motion_outcome import CancelRosbridge, odometry_streaming
from test_observation_on_state import (  # noqa: F401 - pytest fixtures
    action_servers,
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
    monkeypatch.setattr(adapter_module, "CANCEL_CONFIRM_WAIT_S", 0.3)
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


def cancel_alls(socket) -> list[str]:
    """The action servers asked to cancel every goal (the zero goal id), in order."""
    return [
        m["service"]
        for m in list(socket.sent)
        if m.get("op") == "call_service" and m["service"].endswith("/_action/cancel_goal")
    ]


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
        # The new connection re-sends the held goal's cancel (to its action
        # server: the old goal id was the old socket's) and zero velocity at
        # once, and keeps sending zero.
        fresh = sockets[-1]
        assert cancel_alls(fresh) == ["/navigate_to_pose/_action/cancel_goal"]
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

        # The action server answers, but its goal is CANCELING: a cancel-all
        # then answers "no error, nothing canceling" while the server still
        # drives it. Only the goal status tells, and it says not ended.
        fresh.answers = True
        fresh.server_goals = {"nav-1": 3}
        fresh.finishes_cancel = False
        with odometry_streaming(backend):
            canceling = backend.safe_stop("halt-canceling")
        assert canceling.outcome == OUTCOME_FAILED
        assert "status [3]" in canceling.evidence["halt_confirmation"][
            "held_goals_unconfirmed"
        ]["nav-1"]
        assert backend.halt_hold() is not None

        # Only a halt that sees the robot still, with every held goal's end
        # confirmed by its action server's goal status, releases it.
        fresh.finishes_cancel = True
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


def _running(backend, call_id, handle, action="/navigate_to_pose"):
    """A goal this rclpy connection sent and its server still runs."""
    backend._goal_handles[call_id] = handle
    backend._result_futures[call_id] = SimpleNamespace(
        done=lambda: False, add_done_callback=lambda _callback: None
    )
    backend._goal_actions[call_id] = action


def test_an_unconfirmed_rclpy_halt_survives_a_reconnect(
    fake_rclpy, clock, action_servers  # noqa: F811
):
    backend, _condition = rclpy_backend(clock)
    calls: list[str] = []
    backend._publish_zero = lambda: calls.append("zero") or True
    _running(backend, "old-call", _Handle(calls))
    action_servers.goals = {"/navigate_to_pose": {"old-call": 2}}
    action_servers.finishes_cancel = False
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    try:
        assert backend.safe_stop("stop-1").outcome == OUTCOME_FAILED
        calls.clear()
        action_servers.cancel_alls.clear()

        backend.reconnect()

        assert backend._goal_handles == {}
        assert backend.halt_hold() is not None
        # The goal's action server is asked to cancel again, and zero goes out.
        assert action_servers.cancel_alls == ["/navigate_to_pose/_action/cancel_goal"]
        assert "zero" in calls
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


STILL = {
    "stationary": True, "drifting": False, "still_samples": 3,
    "odometry_messages": 5, "waited_seconds": 0.1,
}


@pytest.mark.parametrize(
    ("status", "finishes_cancel", "released"),
    [
        (2, True, True),  # cancelled, and the server ended it
        (5, False, True),  # already cancelled
        (4, False, True),  # already succeeded
        (2, False, False),  # cancelled, but still CANCELING
        (3, False, False),  # CANCELING all along: not cancelable, still driven
        (1, False, False),
    ],
)
def test_an_rclpy_held_goal_releases_only_once_its_status_shows_it_ended(
    fake_rclpy, clock, action_servers, status, finishes_cancel, released  # noqa: F811
):
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    action_servers.goals = {"/navigate_to_pose": {"old-call": status}}
    action_servers.finishes_cancel = finishes_cancel
    adapter_module.HALT_HOLD.arm(
        "halt not confirmed", {"old-call": ("old-call", "/navigate_to_pose")}
    )
    backend.wait_until_stationary = lambda _s: dict(STILL)
    try:
        result = backend.safe_stop("stop-1")
        confirmation = result.evidence["halt_confirmation"]
        # The held goal's cancel went to its action server either way.
        assert action_servers.cancel_alls == ["/navigate_to_pose/_action/cancel_goal"]
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


def test_an_rclpy_held_goal_with_no_status_published_keeps_the_hold(
    fake_rclpy, clock, action_servers  # noqa: F811
):
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    action_servers.publishes = False
    adapter_module.HALT_HOLD.arm("halt not confirmed", {"old-call": ("old-call", "/spin")})
    backend.wait_until_stationary = lambda _s: dict(STILL)
    try:
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_FAILED
        assert "no goal status" in result.evidence["halt_confirmation"][
            "held_goals_unconfirmed"
        ]["old-call"]
        assert backend.halt_hold() is not None
    finally:
        backend.disconnect()


def test_a_restarted_rclpy_host_confirms_a_kept_goal_by_its_action_name(
    fake_rclpy, clock, action_servers, tmp_path, monkeypatch  # noqa: F811
):
    # Was: a restarted rclpy host kept no goal handle, so nothing could ever
    # confirm the end and every halt failed. The kept token is the action name.
    kept = tmp_path / "hold.json"
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(kept))
    adapter_module.HALT_HOLD.arm(
        "halt not confirmed", {"old-call": ("old-call", "/navigate_to_pose")}
    )
    # The host process restarts: a new hold, read from the kept file.
    monkeypatch.setattr(adapter_module, "HALT_HOLD", adapter_module._HaltHold())
    action_servers.goals = {"/navigate_to_pose": {"old-call": 2}}
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    backend.wait_until_stationary = lambda _s: dict(STILL)
    try:
        assert adapter_module.HALT_HOLD.tokens() == {
            "old-call": ("old-call", "/navigate_to_pose")
        }
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_COMPLETED
        assert backend.halt_hold() is None
        assert not kept.exists()
    finally:
        backend.disconnect()


def test_an_rclpy_halt_releases_the_hold_after_a_disconnect_and_a_reconnect(
    fake_rclpy, clock, action_servers  # noqa: F811
):
    # Was: a disconnect destroyed the node and its action clients but kept
    # the goal handles, which the reconnect then held: no halt could release
    # them. The held goal is confirmed by action name on the new node.
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    _running(backend, "nav-1", _Handle([]))
    action_servers.goals = {"/navigate_to_pose": {"nav-1": 2}}
    try:
        backend.disconnect()
        backend.reconnect()
        assert adapter_module.HALT_HOLD.tokens() == {"nav-1": ("nav-1", "/navigate_to_pose")}
        backend.wait_until_stationary = lambda _s: dict(STILL)
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_COMPLETED
        assert result.evidence["halt_confirmation"]["held_goals_ended"] == ["nav-1"]
        assert backend.halt_hold() is None
        assert action_servers.goals == {"/navigate_to_pose": {"nav-1": 5}}
    finally:
        backend.disconnect()


def test_a_held_goal_with_no_action_name_keeps_the_hold(fake_rclpy, clock):  # noqa: F811
    # Nothing names its action server, so nothing can confirm its end: only
    # an operator's recorded release (section 7) lets the robot move again.
    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    adapter_module.HALT_HOLD.arm("halt not confirmed", {"old-call": None})
    backend.wait_until_stationary = lambda _s: dict(STILL)
    try:
        result = backend.safe_stop("stop-1")
        assert result.outcome == OUTCOME_FAILED
        assert "old-call" in result.evidence["halt_confirmation"]["held_goals_unconfirmed"]
        assert backend.halt_hold() is not None
    finally:
        backend.disconnect()


# --- 7. an operator's recorded release -------------------------------------------


def test_an_operator_release_needs_the_robot_seen_still_and_is_recorded():
    # A held goal no halt can confirm ended (no action name is kept for it):
    # every halt fails, and an operator who checked the robot releases it.
    backend, sockets = rosbridge(answers=True)
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance", "motion.halt"}
    adapter_module.HALT_HOLD.arm("a reconnect dropped goals still running: old", {"old": None})
    kept = adapter_module.halt_hold_path()
    audit = kept.with_name(f"{kept.stem}.releases.jsonl")
    try:
        with odometry_streaming(backend):
            assert backend.safe_stop("halt-1").outcome == OUTCOME_FAILED

        # Who and why are required, also on a robot seen still.
        with odometry_streaming(backend):
            for operator, reason in (("", "checked"), ("ops", " "), (None, None)):
                refused = adapter.release_hold(operator=operator, reason=reason)
                assert refused == {
                    "released": False,
                    "detail": "an operator and a reason are required",
                }
        assert adapter_module.HALT_HOLD.snapshot() is not None
        assert not audit.exists()
        # A robot not seen still is never released.
        with odometry_streaming(backend, linear=0.2):
            moving = adapter.release_hold(operator="ops", reason="checked the base")
        assert moving["released"] is False
        assert adapter_module.HALT_HOLD.snapshot() is not None
        assert not audit.exists()

        zeros = sockets[-1].zeros()
        with odometry_streaming(backend):
            released = adapter.release_hold(operator="ops", reason="checked the base")
        assert released["released"] is True
        assert sockets[-1].zeros() > zeros
        record = released["record"]
        assert record["operator"] == "ops" and record["reason"] == "checked the base"
        assert record["held"]["goals"] == ["old"]
        assert record["evidence"]["stillness"]["stationary"] is True
        # No action name is kept for "old", so no cancel could be sent for it.
        assert record["evidence"]["reissued_cancels"] == []
        assert adapter_module.HALT_HOLD.snapshot() is None
        assert not kept.exists()
        lines = audit.read_text().splitlines()
        assert len(lines) == 1 and json.loads(lines[0])["operator"] == "ops"
        assert adapter_module.HALT_HOLD.motion_refusal("anything") is not None  # not admitted
        adapter_module.HALT_HOLD.admit("adv-1")
        assert adapter_module.HALT_HOLD.motion_refusal("adv-1") is None
    finally:
        adapter.disconnect()


def test_a_release_that_cannot_be_recorded_keeps_the_hold():
    backend, _sockets = rosbridge(answers=True)
    adapter_module.HALT_HOLD.arm("held", {"old": None})
    kept = adapter_module.halt_hold_path()
    kept.with_name(f"{kept.stem}.releases.jsonl").mkdir(parents=True)
    try:
        with odometry_streaming(backend):
            result = backend.release_hold(operator="ops", reason="checked")
        assert result["released"] is False
        assert adapter_module.HALT_HOLD.snapshot() is not None
        assert kept.exists()
    finally:
        backend.disconnect()


def test_a_release_whose_zero_cannot_be_published_keeps_the_hold():
    # The operator's word and a still robot are not enough: the release
    # itself must have put the standard stop on cmd_vel.
    backend, _sockets = rosbridge(answers=True)
    adapter_module.HALT_HOLD.arm("held", {"old": None})
    kept = adapter_module.halt_hold_path()
    audit = kept.with_name(f"{kept.stem}.releases.jsonl")
    backend._publish_zero = lambda: False
    try:
        with odometry_streaming(backend):
            result = backend.release_hold(operator="ops", reason="checked")
        assert result == {"released": False, "detail": "standard cmd_vel stop unavailable"}
        assert adapter_module.HALT_HOLD.snapshot() is not None
        assert kept.exists() and not audit.exists()
    finally:
        backend.disconnect()


def test_a_held_goal_with_no_action_name_is_never_reported_recancelled():
    # Its cancel cannot be sent (nothing names its action server), so it must
    # not appear among the cancels that went out; the named one does.
    backend, sockets = rosbridge(answers=True)
    adapter_module.HALT_HOLD.arm(
        "held", {"old": None, "nav-1": ("nav-1", "/navigate_to_pose")}
    )
    try:
        assert backend._recancel_held() == ["nav-1"]
        assert cancel_alls(sockets[-1]) == ["/navigate_to_pose/_action/cancel_goal"]
        with pytest.raises(LookupError):
            backend._recancel("old", None)
    finally:
        backend.disconnect()


def test_a_presence_connection_never_releases_the_hold():
    backend, _sockets = rosbridge(answers=True)
    backend._presence_only = True
    adapter_module.HALT_HOLD.arm("held", {})
    try:
        with odometry_streaming(backend):
            assert backend.release_hold(operator="ops", reason="checked")["released"] is False
        assert adapter_module.HALT_HOLD.snapshot() is not None
    finally:
        backend.disconnect()


def test_deleting_the_kept_file_while_the_host_runs_releases_nothing():
    # The documented way out is the operator release, or stop the host,
    # delete the file, start the host: the running process keeps its hold.
    adapter_module.HALT_HOLD.arm("held", {"nav-1": ("nav-1", "/navigate_to_pose")})
    adapter_module.halt_hold_path().unlink()
    assert adapter_module.HALT_HOLD.snapshot()["goals"] == ["nav-1"]
    adapter_module.HALT_HOLD.admit("adv-1")
    assert adapter_module.HALT_HOLD.motion_refusal("adv-1") is not None
