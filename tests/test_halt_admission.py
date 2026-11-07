"""No goal is sent while a halt is running, held, or ran after the motion was admitted.

A motion is admitted when the adapter accepts the call; a halt starts a new
halt epoch. Every goal send (the first goal, a slow-down preempt, each leg of
an inflation escape) is checked against the hold at the moment it is sent,
under the same lock a halt takes before it reads the running goals. So:

* while the hold is armed, nothing sends a goal, not even the braking guard's
  slower preempt of a goal whose cancel was never confirmed;
* a halt that starts while a motion is still in its preflight, or between two
  legs of an escape, ends that motion: it is never sent afterwards, also when
  the halt itself completed;
* the hold survives a restart of the host process: it is kept in a file and a
  new process starts held (an unreadable file holds too).
"""

from __future__ import annotations

import threading
import time
from types import SimpleNamespace

import pytest
from test_generic_ros2_adapter import adapter as fake_adapter
from test_halt_hold import rosbridge
from test_inflation_escape import OBSERVED, EscapeBackend, navigate
from test_inflation_escape import make as escape_adapter
from test_motion_speed_contract import site_profile
from test_observation_on_state import clock, fake_rclpy  # noqa: F401 - pytest fixtures

from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter


@pytest.fixture(autouse=True)
def short_waits(monkeypatch):
    monkeypatch.setattr(adapter_module, "HALT_STILL_WAIT_S", 0.3)
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.25")


def _guard(**overrides):
    values = dict(
        resends=True,
        slow_to=0.06,
        distance_m=2.0,
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        direction_rad=0.0,
        speed_changes=[],
        minimum_room_m=None,
        last_slowdown_at=None,
        speed_mps=0.12,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


def _goals(socket):
    return [m for m in list(socket.sent) if m.get("op") == "send_action_goal"]


def _driving(backend, call_id="adv-1"):
    adapter_module.HALT_HOLD.admit(call_id)
    backend._active_actions[call_id] = "/drive_on_heading"
    backend._guards[call_id] = _guard()
    backend._pose = {"x": 0.1, "y": 0.0, "yaw": 0.0}


# --- 1. the braking guard's preempt never sends while held --------------------


def test_a_slow_down_preempt_sends_no_goal_while_the_hold_is_armed():
    backend, sockets = rosbridge(answers=False)
    try:
        _driving(backend)
        held = backend.safe_stop("halt-1")
        assert held.outcome == OUTCOME_FAILED
        assert adapter_module.HALT_HOLD.snapshot() is not None
        before = len(_goals(sockets[-1]))

        backend._slow_down("adv-1", "motion.advance")

        # Was: {'op': 'send_action_goal', 'id': 'adv-1#slow0', ...} while held.
        assert len(_goals(sockets[-1])) == before
        assert backend._goal_ids.get("adv-1", "adv-1") == "adv-1"
    finally:
        backend.disconnect()


def test_a_slow_down_racing_a_halt_sends_nothing_and_the_hold_keeps_the_driving_goal():
    backend, sockets = rosbridge(answers=False)
    try:
        _driving(backend)
        unanswered = backend.cancel

        def cancel_while_the_guard_fires(call_id):
            # The guard decides on a scan while the halt waits for its cancel.
            backend._slow_down(call_id, "motion.advance")
            return unanswered(call_id)

        backend.cancel = cancel_while_the_guard_fires
        held = backend.safe_stop("halt-1")

        assert held.outcome == OUTCOME_FAILED
        assert _goals(sockets[-1]) == []
        # The hold's cancel token is the goal that is driving, not a renamed one.
        assert adapter_module.HALT_HOLD.tokens()["adv-1"] == ("adv-1", "/drive_on_heading")
    finally:
        backend.disconnect()


def test_a_slow_down_after_a_completed_halt_sends_nothing():
    backend, sockets = rosbridge(answers=True)
    try:
        _driving(backend)
        with adapter_module.HALT_HOLD.halting():
            pass  # a halt ran (and completed) after the drive was admitted
        backend._slow_down("adv-1", "motion.advance")
        assert _goals(sockets[-1]) == []
    finally:
        backend.disconnect()


def test_an_admitted_slow_down_still_preempts():
    backend, sockets = rosbridge(answers=True)
    try:
        _driving(backend)
        backend._slow_down("adv-1", "motion.advance")
        assert [g["id"] for g in _goals(sockets[-1])] == ["adv-1#slow0"]
    finally:
        backend.disconnect()


# --- 2. a halt during the preflight ends the admitted motion ------------------


def _slow_preflight(device, during):
    real = device.backend.observation

    def slow(*args, **kwargs):
        during()
        return real(*args, **kwargs)

    device.backend.observation = slow


def _advance(device, call_id="adv"):
    return device.invoke(CallRequest(call_id, "motion.advance", {"distance_m": 0.25}))


def test_a_hold_armed_during_the_preflight_stops_the_motion_before_it_is_sent():
    device = fake_adapter({"motion.advance", "motion.halt"})
    device.describe()
    _slow_preflight(device, lambda: adapter_module.HALT_HOLD.arm("halt not confirmed", {}))
    result = _advance(device)
    assert device.backend.calls == []
    assert result.outcome == OUTCOME_REFUSED
    assert result.detail.startswith("effect_unknown")


def test_a_halt_that_completed_during_the_preflight_still_stops_the_motion():
    device = fake_adapter({"motion.advance", "motion.halt"})
    device.describe()
    halted = []
    _slow_preflight(
        device,
        lambda: halted
        or halted.append(device.invoke(CallRequest("halt-1", "motion.halt", {}))),
    )
    result = _advance(device)
    assert halted and halted[0].outcome == OUTCOME_COMPLETED
    # The robot must not move after a halt that reported completed.
    assert [c for c in device.backend.calls if c[1] != "motion.halt"] == []
    assert result.outcome == OUTCOME_REFUSED
    assert "halt" in result.detail


def test_a_concurrent_halt_in_progress_refuses_new_motion():
    device = fake_adapter({"motion.advance", "motion.halt"})
    device.describe()
    entered, leave = threading.Event(), threading.Event()

    def slow_stop(call_id):
        entered.set()
        leave.wait(2.0)
        return SimpleNamespace(call_id=call_id, outcome=OUTCOME_COMPLETED)

    device.backend.safe_stop = slow_stop
    halt = threading.Thread(target=lambda: device.invoke(CallRequest("h", "motion.halt", {})))
    halt.start()
    try:
        assert entered.wait(2.0)
        result = _advance(device)
    finally:
        leave.set()
        halt.join()
    assert result.outcome == OUTCOME_REFUSED
    assert "halt is in progress" in result.detail
    assert [c for c in device.backend.calls if c[1] == "motion.advance"] == []


# --- 3. between the legs of an inflation escape -------------------------------


def test_a_halt_between_escape_legs_stops_the_next_leg():
    device_holder: list[GenericROS2Adapter] = []

    def backoff_then_halt(call_id):
        # The halt lands while the backoff leg runs; it sees that leg only.
        device_holder[0].invoke(CallRequest("halt-1", "motion.halt", {}))
        return OUTCOME_COMPLETED

    backend = EscapeBackend(OBSERVED, outcomes={"motion.retreat": backoff_then_halt})
    device = escape_adapter(backend)
    device_holder.append(device)
    result = navigate(device)
    assert [call for call, _cap, _args in backend.calls] == ["nav-1:escape-backoff"]
    assert result.outcome != OUTCOME_COMPLETED
    assert "halt" in result.detail


def test_the_escape_backoff_obeys_the_site_speed_ceiling(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 0.03})
    backend = EscapeBackend(OBSERVED)
    navigate(escape_adapter(backend))
    backoff = [args for call, _cap, args in backend.calls if call.endswith(":escape-backoff")]
    assert backoff and backoff[0]["speed_mps"] <= 0.03


# --- 4. the hold survives a host restart --------------------------------------


def test_an_armed_hold_is_kept_across_a_process_restart(tmp_path, monkeypatch):
    path = tmp_path / "hold.json"
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(path))
    first = adapter_module._HaltHold()
    first.arm("cancel not confirmed for nav-1", {"nav-1": ("nav-1", "/navigate_to_pose")})

    restarted = adapter_module._HaltHold()  # a new host process
    held = restarted.snapshot()
    assert held is not None and held["goals"] == ["nav-1"]
    assert "nav-1" in held["reason"]
    assert restarted.tokens() == {"nav-1": ("nav-1", "/navigate_to_pose")}

    assert restarted.release() is not None
    assert adapter_module._HaltHold().snapshot() is None


def test_an_unreadable_hold_file_holds(tmp_path, monkeypatch):
    path = tmp_path / "hold.json"
    path.write_text("{not json", encoding="utf-8")
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(path))
    held = adapter_module._HaltHold().snapshot()
    assert held is not None and "cannot be read" in held["reason"]


def test_a_restarted_host_refuses_motion_until_a_halt_sees_the_robot_still(
    tmp_path, monkeypatch
):
    path = tmp_path / "hold.json"
    monkeypatch.setenv(adapter_module.HALT_HOLD_FILE_ENV, str(path))
    adapter_module.HALT_HOLD.arm("odometry did not show the robot still", {})
    monkeypatch.setattr(adapter_module, "HALT_HOLD", adapter_module._HaltHold())
    device = fake_adapter({"motion.advance", "motion.halt"})
    device.describe()
    refused = _advance(device)
    assert refused.outcome == OUTCOME_REFUSED
    assert refused.detail.startswith("effect_unknown")
    assert device.backend.calls == []


# --- 5. a presence-only connection never commands ---------------------------


def test_a_presence_only_reconnect_while_held_publishes_nothing():
    sockets = []

    def connect(_url):
        from test_motion_outcome import CancelRosbridge

        socket = CancelRosbridge()
        sockets.append(socket)
        return socket

    adapter_module.HALT_HOLD.arm("halt not confirmed", {"nav-1": ("nav-1", "/navigate_to_pose")})
    backend = adapter_module.RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=connect, presence_only=True
    )
    try:
        backend._topic_types = {"/cmd_vel": "geometry_msgs/msg/Twist"}
        backend.reconnect()
        time.sleep(0.3)
        for socket in sockets:
            commanded = [
                m
                for m in list(socket.sent)
                if m.get("op") in ("advertise", "publish", "cancel_action_goal")
            ]
            assert commanded == []
    finally:
        backend.disconnect()


# --- 6. the rclpy transport: the same gate, and a goal not yet accepted -------


class _Future:
    def __init__(self, value=None, done=False):
        self.value, self._done, self._callbacks = value, done, []

    def done(self):
        return self._done

    def result(self):
        return self.value

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def finish(self, value):
        self.value, self._done = value, True
        for callback in self._callbacks:
            callback(self)


class _AcceptedHandle:
    """An accepted goal whose server ends it (CANCELED) once its cancel arrives."""

    accepted = True

    def __init__(self):
        self.cancels = 0
        self.result = _Future()

    def cancel_goal_async(self):
        self.cancels += 1
        self.result.finish(SimpleNamespace(status=5))
        return _Future(SimpleNamespace(goals_canceling=("adv",)), done=True)

    def get_result_async(self):
        return self.result


@pytest.fixture
def rclpy_actions(fake_rclpy, monkeypatch):  # noqa: F811
    import sys
    import types

    for name, attributes in {
        "action_msgs": {},
        "action_msgs.msg": {"GoalStatus": SimpleNamespace(STATUS_SUCCEEDED=4, STATUS_CANCELED=5)},
        "rclpy.action": {"ActionClient": object},
    }.items():
        module = types.ModuleType(name)
        for key, value in attributes.items():
            setattr(module, key, value)
        monkeypatch.setitem(sys.modules, name, module)


def _rclpy_with_client(clock, monkeypatch, future):  # noqa: F811
    from test_observation_on_state import rclpy_backend

    backend, _condition = rclpy_backend(clock)
    backend._publish_zero = lambda: True
    sent = []

    def send_goal_async(goal, feedback_callback=None):
        sent.append(goal)
        return future

    backend._action_clients["motion.advance"] = SimpleNamespace(
        server_is_ready=lambda: True, send_goal_async=send_goal_async
    )
    monkeypatch.setattr(backend, "_goal", lambda _capability, _arguments: object())
    return backend, sent


def _rclpy_advance(backend):
    return backend.invoke(
        call_id="adv",
        capability_id="motion.advance",
        arguments={"distance_m": 0.3},
        deadline_seconds=0.2,
    )


def test_rclpy_sends_no_goal_while_the_hold_is_armed(rclpy_actions, clock, monkeypatch):  # noqa: F811
    backend, sent = _rclpy_with_client(clock, monkeypatch, _Future())
    adapter_module.HALT_HOLD.arm("halt not confirmed", {})
    try:
        result = _rclpy_advance(backend)
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_REFUSED
    assert result.detail.startswith("effect_unknown")
    assert sent == []


def test_an_rclpy_goal_not_yet_accepted_is_cancelled_by_a_halt(
    rclpy_actions, clock, monkeypatch  # noqa: F811
):
    pending = _Future()
    backend, sent = _rclpy_with_client(clock, monkeypatch, pending)
    try:
        timed_out = _rclpy_advance(backend)
        assert len(sent) == 1 and timed_out.outcome == "timeout"
        handle = _AcceptedHandle()
        pending.finish(handle)  # the server accepts it after the call gave up waiting
        halted = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    confirmation = halted.evidence["halt_confirmation"]
    assert confirmation["active_goals"] == ["adv"]
    assert confirmation["cancelled"] == ["adv"]
    assert handle.cancels == 1


def test_an_unanswered_rclpy_goal_holds_with_its_pending_send(
    rclpy_actions, clock, monkeypatch  # noqa: F811
):
    pending = _Future()
    backend, _sent = _rclpy_with_client(clock, monkeypatch, pending)
    try:
        _rclpy_advance(backend)
        halted = backend.safe_stop("halt-1")
    finally:
        backend.disconnect()
    assert halted.outcome == OUTCOME_FAILED
    assert "adv" in halted.evidence["halt_confirmation"]["unconfirmed"]
    assert adapter_module.HALT_HOLD.tokens() == {"adv": ("adv", "/drive_on_heading")}


# --- 7. every entry point of a halt, and every goal send -----------------------


def test_a_host_safe_stop_during_the_preflight_stops_the_motion():
    device = fake_adapter({"motion.advance", "motion.halt"})
    device.describe()
    stops = []
    _slow_preflight(device, lambda: stops or stops.append(device.safe_stop()))
    result = _advance(device)
    assert stops
    assert [c for c in device.backend.calls if c[1] == "motion.advance"] == []
    assert result.outcome == OUTCOME_REFUSED


def test_a_leg_of_a_call_nobody_admitted_is_never_sent():
    hold = adapter_module.HALT_HOLD
    hold.admit("leg", like="never-admitted")
    assert hold.motion_refusal("leg") is not None
    hold.admit("call")
    hold.admit("call:leg", like="call")
    assert hold.motion_refusal("call:leg") is None


def _rosbridge_advance(backend, call_id="adv-2"):
    return backend.invoke(
        call_id=call_id,
        capability_id="motion.advance",
        arguments={"distance_m": 0.3},
        deadline_seconds=0.2,
    )


def test_rosbridge_sends_no_first_goal_while_the_hold_is_armed():
    backend, sockets = rosbridge(answers=True)
    adapter_module.HALT_HOLD.arm("halt not confirmed", {})
    try:
        result = _rosbridge_advance(backend)
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_REFUSED
    assert result.detail.startswith("effect_unknown")
    assert _goals(sockets[-1]) == []


def test_rosbridge_sends_no_goal_admitted_before_a_halt():
    backend, sockets = rosbridge(answers=True)
    adapter_module.HALT_HOLD.admit("adv-2")
    with adapter_module.HALT_HOLD.halting():
        pass
    try:
        result = _rosbridge_advance(backend)
    finally:
        backend.disconnect()
    assert result.outcome == OUTCOME_REFUSED
    assert _goals(sockets[-1]) == []
