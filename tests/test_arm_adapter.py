"""M0 for a serial-bus arm: state, joint limits, a measured stop, fail closed.

Every test runs the adapter against :class:`SimulatedArmBus`; none touches a
port. The guarantees are the ones an operator relies on the day the arm is
plugged in: a target outside a limit never reaches the bus, a stop is reported
done only when the read-back proves it, and a move whose outcome is unknown is
never sent again.
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import pytest

from flyto_robotics.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
)
from flyto_robotics.arm_adapter import (
    CAPABILITY_HALT,
    CAPABILITY_MOVE,
    CAPABILITY_STATE,
    OBSERVATION,
    ProfileInvalid,
    SerialArmAdapter,
    load_profile,
    parse_profile,
)
from flyto_robotics.arm_bus import ArmBus, BusError, SimulatedArmBus
from flyto_robotics.lerobot_arm_bus import LeRobotArmBus

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE_PROFILE = ROOT / "examples" / "arm-profiles" / "am-arm200-follower.json"
JOINTS = ("shoulder_pan", "elbow_flex", "gripper")


def raw_profile(**overrides):
    raw = {
        "schema": "flyto.arm-profile.v1",
        "resource_id": "arm-test",
        "limits_accepted": True,
        "position_unit": "rad",
        "joints": [
            {"name": "shoulder_pan", "min_rad": -1.0, "max_rad": 1.0},
            {"name": "elbow_flex", "min_rad": -1.0, "max_rad": 1.0},
            {"name": "gripper", "min_rad": 0.0, "max_rad": 0.5},
        ],
        "position_tolerance_rad": 0.02,
        "max_joint_speed_rad_s": 1.0,
        "control_period_s": 0.05,
        "move_deadline_s": 5.0,
        "stall": {"window_s": 0.5, "min_progress_rad": 0.005},
        "settle": {"samples": 3, "max_drift_rad": 0.01},
        "stop": {"mode": "hold", "confirm_samples": 4, "confirm_period_s": 0.05,
                 "max_drift_rad": 0.02},
    }
    for key, value in overrides.items():
        if key == "stop_mode":
            raw["stop"]["mode"] = value
        else:
            raw[key] = value
    return raw


class FakeClock:
    """Time that only passes when the adapter sleeps."""

    def __init__(self):
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


def arm(bus=None, **overrides):
    profile = parse_profile(raw_profile(**overrides))
    bus = bus or SimulatedArmBus(JOINTS)
    clock = FakeClock()
    return SerialArmAdapter(bus, profile, clock=clock, sleep=clock.sleep), bus


def move(adapter, call_id="move-1", **targets):
    arguments = {"shoulder_pan_rad": 0.3, "elbow_flex_rad": -0.2, "gripper_rad": 0.1}
    arguments.update({f"{k}_rad": v for k, v in targets.items()})
    return adapter.invoke(CallRequest(call_id, CAPABILITY_MOVE, arguments))


# -- the bus seam --------------------------------------------------------------


def test_the_simulated_bus_satisfies_the_bus_interface():
    assert isinstance(SimulatedArmBus(JOINTS), ArmBus)


# -- state ---------------------------------------------------------------------


def test_state_reads_every_joint_and_torque_without_writing():
    adapter, bus = arm(SimulatedArmBus(JOINTS, positions={"elbow_flex": 0.25}))

    result = adapter.invoke(CallRequest("s1", CAPABILITY_STATE))

    assert result.outcome == OUTCOME_COMPLETED
    state = result.evidence["arm_state"]
    assert state[OBSERVATION] == {"frame": "joints", "shoulder_pan": 0.0,
                                  "elbow_flex": 0.25, "gripper": 0.0}
    assert state["torque_enabled"] == {name: False for name in JOINTS}
    assert state["outside_limits"] == []
    assert bus.writes == [] and bus.torque_commands == []


def test_state_reports_a_joint_outside_its_limits():
    adapter, _ = arm(SimulatedArmBus(JOINTS, positions={"gripper": 0.9}))

    assert adapter.read_state().evidence["arm_state"]["outside_limits"] == ["gripper"]


def test_state_with_a_missing_joint_is_a_failure_not_a_partial_state():
    bus = SimulatedArmBus(JOINTS)
    bus.missing_joints = {"gripper"}
    adapter, _ = arm(bus)

    result = adapter.read_state()

    assert result.outcome == OUTCOME_FAILED
    assert "gripper" in result.detail


def test_state_on_a_dropped_connection_fails_closed():
    bus = SimulatedArmBus(JOINTS)
    bus.connected = False
    adapter, _ = arm(bus)

    result = adapter.read_state()

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["reason"] == "connection_lost"
    assert result.evidence["arm_outcome"]["state_known"] is False


# -- limits --------------------------------------------------------------------


@pytest.mark.parametrize(
    "targets",
    [
        {"shoulder_pan": 1.2},  # above
        {"elbow_flex": -1.0001},  # below
        {"gripper": -0.1},  # below a 0..0.5 joint
        {"gripper": float("nan")},
        {"gripper": float("inf")},
    ],
)
def test_an_out_of_limit_target_is_refused_before_any_write(targets):
    adapter, bus = arm()

    result = move(adapter, **targets)

    assert result.outcome == OUTCOME_REFUSED
    assert bus.writes == [] and bus.torque_commands == []
    # Never clamped: the arm is exactly where it was.
    assert bus.positions == {name: 0.0 for name in JOINTS}


def test_a_target_on_the_limit_is_accepted():
    adapter, _ = arm()

    assert move(adapter, shoulder_pan=1.0, gripper=0.5).outcome == OUTCOME_COMPLETED


@pytest.mark.parametrize(
    "arguments",
    [
        {"shoulder_pan_rad": 0.1, "elbow_flex_rad": 0.1},  # a joint left out
        {"shoulder_pan_rad": 0.1, "elbow_flex_rad": 0.1, "gripper_rad": 0.1, "speed": 9},
        {"shoulder_pan_rad": True, "elbow_flex_rad": 0.1, "gripper_rad": 0.1},
        {"shoulder_pan_rad": "0.1", "elbow_flex_rad": 0.1, "gripper_rad": 0.1},
    ],
)
def test_malformed_move_arguments_are_refused_before_any_write(arguments):
    adapter, bus = arm()

    result = adapter.invoke(CallRequest("m", CAPABILITY_MOVE, arguments))

    assert result.outcome == OUTCOME_REFUSED
    assert bus.writes == []


def test_a_move_is_refused_while_the_limits_are_not_accepted_but_state_and_stop_work():
    adapter, bus = arm(limits_accepted=False)

    assert move(adapter).outcome == OUTCOME_REFUSED
    assert bus.writes == [] and bus.torque_commands == []
    assert adapter.read_state().outcome == OUTCOME_COMPLETED
    assert adapter.safe_stop().outcome == OUTCOME_COMPLETED


def test_a_move_from_a_pose_outside_the_limits_is_refused():
    adapter, bus = arm(SimulatedArmBus(JOINTS, positions={"gripper": 0.8}))

    result = move(adapter)

    assert result.outcome == OUTCOME_REFUSED
    assert bus.writes == []


# -- a normal move -------------------------------------------------------------


def test_a_move_arrives_settles_and_reports_phases_for_the_host_to_judge():
    adapter, bus = arm()

    result = move(adapter)

    assert result.outcome == OUTCOME_COMPLETED, result.detail
    phases = result.evidence["observations"][OBSERVATION]
    assert set(phases) == {"before", "after", "settled"}
    assert phases["before"]["shoulder_pan"] == 0.0
    assert abs(phases["settled"]["shoulder_pan"] - 0.3) <= 0.02
    assert all(phase["frame"] == "joints" for phase in phases.values())
    assert result.evidence["arm_outcome"]["reason"] == "completed"
    # Torque was enabled with the goal at the measured pose, then stepped.
    assert bus.writes[0] == {name: 0.0 for name in JOINTS}
    assert bus.torque_commands == [True]


def test_a_move_is_stepped_at_the_declared_speed():
    adapter, bus = arm(max_joint_speed_rad_s=0.2)  # 0.01 rad per 0.05 s tick

    assert move(adapter, shoulder_pan=0.1, elbow_flex=0.0, gripper=0.0).outcome == (
        OUTCOME_COMPLETED
    )
    steps = [b["shoulder_pan"] - a["shoulder_pan"] for a, b in zip(bus.writes, bus.writes[1:])]
    assert max(abs(step) for step in steps) <= 0.01 + 1e-9


def test_the_same_call_id_never_moves_twice():
    adapter, bus = arm()
    first = move(adapter, call_id="once")
    writes = len(bus.writes)

    again = move(adapter, call_id="once", shoulder_pan=-0.3)

    assert again is first
    assert len(bus.writes) == writes


# -- a stalled joint -----------------------------------------------------------


def test_a_stalled_joint_ends_the_move_as_failed_and_holds_where_it_is():
    bus = SimulatedArmBus(JOINTS)
    bus.stalled = {"elbow_flex"}
    adapter, _ = arm(bus)

    result = move(adapter)

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["reason"] == "stalled"
    assert result.evidence["arm_outcome"]["stalled"] == ["elbow_flex"]
    # The stop afterwards held the measured pose and confirmed it.
    assert result.evidence["stop"]["confirmed"] is True
    assert bus.writes[-1] == {k: v for k, v in bus.positions.items()}


def test_a_move_that_runs_out_of_time_is_a_timeout_followed_by_a_stop():
    adapter, _ = arm(move_deadline_s=0.2, max_joint_speed_rad_s=0.1)

    result = move(adapter)

    assert result.outcome == OUTCOME_TIMEOUT
    assert result.evidence["stop"]["confirmed"] is True


# -- a dropped connection mid-move ---------------------------------------------


def test_a_connection_lost_mid_move_fails_closed_and_is_never_retried():
    bus = SimulatedArmBus(JOINTS)
    bus.drop_after_writes = 4  # the pre-torque hold and three steps
    adapter, _ = arm(bus)

    result = move(adapter, call_id="lost")

    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["arm_outcome"]
    assert outcome == {"reason": "connection_lost", "state_known": False, "retry": "never"}
    writes = len(bus.writes)
    # The same call again returns the unknown outcome; nothing is re-sent.
    bus.connected = True
    bus.drop_after_writes = None
    assert move(adapter, call_id="lost") is result
    assert len(bus.writes) == writes
    # A new move is refused until the state has been read again.
    refused = move(adapter, call_id="next")
    assert refused.outcome == OUTCOME_REFUSED
    assert "read state first" in refused.detail
    assert len(bus.writes) == writes
    assert adapter.read_state().outcome == OUTCOME_COMPLETED
    assert move(adapter, call_id="after-read").outcome == OUTCOME_COMPLETED


# -- the stop ------------------------------------------------------------------


def test_a_hold_stop_writes_the_measured_pose_and_is_confirmed_by_read_back():
    bus = SimulatedArmBus(JOINTS, positions={"shoulder_pan": 0.4}, torque_enabled=True)
    adapter, _ = arm(bus)

    result = adapter.invoke(CallRequest("halt-1", CAPABILITY_HALT))

    assert result.outcome == OUTCOME_COMPLETED
    confirmation = result.evidence["stop_confirmation"]
    assert confirmation["mode"] == "hold" and confirmation["confirmed"] is True
    assert confirmation["torque_enabled"] == {name: True for name in JOINTS}
    assert confirmation["max_drift_rad"] == 0.0
    assert bus.writes == [{"shoulder_pan": 0.4, "elbow_flex": 0.0, "gripper": 0.0}]
    phases = result.evidence["observations"][OBSERVATION]
    assert set(phases) == {"before", "after"}


def test_a_hold_that_does_not_hold_is_a_failed_stop():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    bus.sag_rad = 0.01  # 0.04 rad over four samples, over the 0.02 tolerance
    adapter, _ = arm(bus)

    result = adapter.safe_stop()

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["reason"] == "stop_unconfirmed"
    assert "drifted" in result.detail


def test_a_torque_off_stop_is_confirmed_only_by_the_torque_read_back():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    adapter, _ = arm(bus, stop_mode="torque_off")

    result = adapter.safe_stop()

    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["stop_confirmation"]["torque_enabled"] == {n: False for n in JOINTS}
    assert bus.torque_commands == [False]


def test_a_torque_off_whose_read_back_still_shows_torque_is_a_failed_stop():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    bus.torque_off_ignored = True
    adapter, _ = arm(bus, stop_mode="torque_off")

    result = adapter.safe_stop()

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["stop_confirmation"]["confirmed"] is False
    assert "torque still reads enabled" in result.detail


def test_a_stop_on_a_dead_connection_is_a_failed_stop():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    bus.connected = False
    adapter, _ = arm(bus)

    result = adapter.safe_stop()

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["state_known"] is False


def test_an_unconfirmed_stop_blocks_moves_until_a_stop_is_confirmed():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    bus.torque_off_ignored = True
    adapter, _ = arm(bus, stop_mode="torque_off")
    assert adapter.safe_stop().outcome == OUTCOME_FAILED

    blocked = move(adapter)
    assert blocked.outcome == OUTCOME_REFUSED and "stop again" in blocked.detail
    # A state read alone does not clear an unconfirmed stop.
    adapter.read_state()
    assert move(adapter, call_id="m2").outcome == OUTCOME_REFUSED

    bus.torque_off_ignored = False
    assert adapter.safe_stop().outcome == OUTCOME_COMPLETED
    assert adapter.read_state().outcome == OUTCOME_COMPLETED
    assert move(adapter, call_id="m3").outcome == OUTCOME_COMPLETED


def test_a_stop_is_repeatable_and_never_answered_from_a_cache():
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    adapter, _ = arm(bus)
    adapter.invoke(CallRequest("halt", CAPABILITY_HALT))

    bus.torque_off_ignored = True
    bus.sag_rad = 0.05
    second = adapter.invoke(CallRequest("halt", CAPABILITY_HALT))

    assert second.outcome == OUTCOME_FAILED


def test_a_stop_during_a_move_cancels_the_move_and_holds():
    """Real threads and real time: the stop must not wait for the move to end."""
    profile = parse_profile(raw_profile(max_joint_speed_rad_s=0.05, move_deadline_s=10.0))
    bus = SimulatedArmBus(JOINTS)
    adapter = SerialArmAdapter(bus, profile)
    outcome = {}

    def run():
        outcome["move"] = move(adapter)

    worker = threading.Thread(target=run)
    worker.start()
    time.sleep(0.3)
    stop = adapter.safe_stop()
    worker.join(timeout=3.0)

    assert not worker.is_alive()
    assert stop.outcome == OUTCOME_COMPLETED
    assert outcome["move"].outcome == OUTCOME_CANCELLED
    writes = len(bus.writes)
    time.sleep(0.15)
    assert len(bus.writes) == writes, "nothing is written after the stop"


# -- the profile ---------------------------------------------------------------


def test_the_example_profile_loads_and_is_not_accepted_for_motion():
    profile = load_profile(EXAMPLE_PROFILE)

    assert profile.limits_accepted is False
    assert "unverified" in json.loads(EXAMPLE_PROFILE.read_text())["note"].lower()


@pytest.mark.parametrize(
    "overrides",
    [
        {"limits_accepted": "yes"},
        {"position_unit": "deg"},
        {"stop_mode": "brake"},
        {"joints": [{"name": "a", "min_rad": 1.0, "max_rad": 1.0}]},
        {"joints": [{"name": "Bad", "min_rad": 0.0, "max_rad": 1.0}]},
        {"joints": [{"name": "a", "min_rad": 0.0, "max_rad": float("nan")}]},
        {"joints": [{"name": "a", "min_rad": 0.0, "max_rad": 1.0, "maxspeed": 3}]},
        {"limit_accepted": True},  # misspelt key
        {"position_tolerance_rad": 0},
    ],
)
def test_a_profile_that_is_not_exact_is_refused(overrides):
    with pytest.raises(ProfileInvalid):
        parse_profile(raw_profile(**overrides))


# -- the LeRobot-style bus wrapper (unverified against real hardware) ----------


class StubMotorsBus:
    def __init__(self, fail=False):
        self.fail = fail
        self.written = []
        self.torque = 1

    def sync_read(self, register):
        if self.fail:
            raise OSError("port closed")
        return {"shoulder_pan": 90.0, "gripper": "?"}

    def sync_write(self, register, values):
        self.written.append((register, values))

    def read(self, register, name):
        return self.torque

    def enable_torque(self):
        self.torque = 1

    def disable_torque(self):
        self.torque = 0


def test_the_lerobot_bus_converts_degrees_and_turns_failures_into_bus_errors():
    stub = StubMotorsBus()
    bus = LeRobotArmBus(stub, joints=("shoulder_pan", "gripper"),
                        degree_joints=("shoulder_pan", "gripper"))

    assert bus.read_positions() == pytest.approx({"shoulder_pan": 1.5707963})  # gripper dropped
    bus.write_goal_positions({"shoulder_pan": 0.5})
    assert stub.written[0][1]["shoulder_pan"] == pytest.approx(28.6478897)
    bus.set_torque(False)
    assert bus.read_torque_enabled() == {"shoulder_pan": False, "gripper": False}
    stub.fail = True
    with pytest.raises(BusError):
        bus.read_positions()


def test_the_lerobot_bus_refuses_a_joint_without_a_declared_scale():
    with pytest.raises(ValueError):
        LeRobotArmBus(StubMotorsBus(), joints=("gripper",), degree_joints=())
