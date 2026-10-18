"""M1 for a manipulator: grasp and release, confirmed by read-back, fail closed.

Every test runs the adapter against :class:`SimulatedArmBus` with simulated
objects between the gripper's jaws; none touches a port. The guarantees are
the ones a pick and place relies on: a grasp is reported done only when the
jaws stopped short of closed on something and held it, a release only when
the jaws read back open, and a grip whose outcome is unknown is never sent
again.
"""

from __future__ import annotations

import copy
import threading
import time

import pytest

from flyto_robotics.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    CallRequest,
)
from flyto_robotics.arm_adapter import (
    CAPABILITY_GRASP,
    CAPABILITY_RELEASE,
    ProfileInvalid,
    SerialArmAdapter,
    parse_profile,
)
from flyto_robotics.arm_bus import SimulatedArmBus
from flyto_robotics.lerobot_arm_bus import LeRobotArmBus
from tests.test_arm_adapter import JOINTS, FakeClock, StubMotorsBus, move, raw_profile

# The test arm's gripper joint travels 0..0.5 rad.
GRIPPER = {
    "joint": "gripper",
    "closes_toward": "min",
    "release_rad": 0.3,
    "max_speed_rad_s": 0.5,
    "position_tolerance_rad": 0.02,
    "object_min_gap_rad": 0.05,
    "hold": {"samples": 4, "max_drift_rad": 0.01},
}
# Open and closed ends, and where an object sits, per closing direction.
DIRECTIONS = {
    "min": {"open": 0.5, "closed": 0.0, "object": 0.2, "closing": -1, "release": 0.45},
    "max": {"open": 0.0, "closed": 0.5, "object": 0.3, "closing": 1, "release": 0.05},
}


def gripper_profile(toward="min", **gripper):
    section = copy.deepcopy(GRIPPER)
    section["closes_toward"] = toward
    if toward == "max":
        section["release_rad"] = 0.2
    section.update(gripper)
    raw = raw_profile()
    raw["gripper"] = section
    return raw


def gripper_arm(toward="min", *, bus=None, real_time=False, **overrides):
    raw = gripper_profile(toward)
    for key, value in overrides.items():
        raw[key] = value
    profile = parse_profile(raw)
    side = DIRECTIONS[toward]
    bus = bus or SimulatedArmBus(JOINTS, positions={"gripper": side["open"]}, torque_enabled=True)
    if real_time:
        return SerialArmAdapter(bus, profile), bus
    clock = FakeClock()
    return SerialArmAdapter(bus, profile, clock=clock, sleep=clock.sleep), bus


def grasp(adapter, call_id="grasp-1", **arguments):
    return adapter.invoke(CallRequest(call_id, CAPABILITY_GRASP, arguments))


def release(adapter, call_id="release-1", open_to=0.45):
    return adapter.invoke(CallRequest(call_id, CAPABILITY_RELEASE, {"open_to_rad": open_to}))


# -- an object present ---------------------------------------------------------


@pytest.mark.parametrize("toward", sorted(DIRECTIONS))
def test_a_grasp_on_an_object_completes_with_the_object_held(toward):
    side = DIRECTIONS[toward]
    adapter, bus = gripper_arm(toward)
    bus.place_object("gripper", side["object"], closing=side["closing"])

    result = grasp(adapter)

    assert result.outcome == OUTCOME_COMPLETED, result.detail
    outcome = result.evidence["arm_outcome"]
    assert outcome["reason"] == "object_held" and outcome["object_held"] is True
    jaws = result.evidence["observations"]["gripper_position"]
    assert jaws["before"]["gripper"] == side["open"]
    assert jaws["after"]["gripper"] == pytest.approx(side["object"])
    assert all(phase["frame"] == "joints" for phase in jaws.values())
    held = result.evidence["observations"]["object_held"]
    assert (held["before"]["held"], held["after"]["held"]) == (0, 1)
    # Only the gripper was driven; no other joint was written.
    assert bus.writes and all(set(write) == {"gripper"} for write in bus.writes)


def test_a_grasp_is_stepped_at_the_grippers_declared_speed():
    adapter, bus = gripper_arm()  # 0.5 rad/s, 0.05 s ticks: 0.025 rad per write
    bus.place_object("gripper", 0.2)

    assert grasp(adapter).outcome == OUTCOME_COMPLETED
    goals = [0.5] + [write["gripper"] for write in bus.writes]
    assert max(abs(b - a) for a, b in zip(goals, goals[1:])) <= 0.025 + 1e-9


# -- no object -----------------------------------------------------------------


@pytest.mark.parametrize("toward", sorted(DIRECTIONS))
def test_a_grasp_that_closes_fully_on_nothing_fails_with_nothing_held(toward):
    adapter, bus = gripper_arm(toward)

    result = grasp(adapter)

    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["arm_outcome"]
    assert outcome["reason"] == "no_object" and outcome["object_held"] is False
    assert bus.positions["gripper"] == pytest.approx(DIRECTIONS[toward]["closed"], abs=0.02)


def test_jaws_stopped_just_short_of_closed_are_not_an_object():
    adapter, bus = gripper_arm()
    bus.place_object("gripper", 0.03)  # under the 0.05 rad an object needs

    result = grasp(adapter)

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["object_held"] is False
    assert result.evidence["arm_outcome"]["reason"] == "no_object"


# -- an object slipping --------------------------------------------------------


def test_an_object_slipping_during_the_hold_is_a_failed_grasp_and_a_stop():
    adapter, bus = gripper_arm()
    # It holds while the jaws settle on it, then gives way during the hold window.
    bus.place_object("gripper", 0.2, slip_rad_per_read=0.01, slip_after_reads=11)

    result = grasp(adapter)

    assert result.outcome == OUTCOME_FAILED, result.detail
    outcome = result.evidence["arm_outcome"]
    assert outcome["reason"] == "slipping" and outcome["object_held"] is False
    assert outcome["hold_drift_rad"] > 0.01
    assert result.evidence["observations"]["object_held"]["after"]["held"] == 0
    assert result.evidence["stop"]["confirmed"] is True


# -- a stall -------------------------------------------------------------------


def test_a_gripper_that_never_moves_is_a_stall_not_a_grasp():
    adapter, bus = gripper_arm()
    bus.stalled = {"gripper"}

    result = grasp(adapter)

    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["arm_outcome"]
    assert outcome["reason"] == "stalled" and outcome["object_held"] is False
    assert outcome["stalled"] == ["gripper"]
    assert result.evidence["stop"]["confirmed"] is True


def test_a_gripper_that_cannot_open_is_a_failed_release_and_a_stop():
    adapter, bus = gripper_arm()
    bus.place_object("gripper", 0.2)
    assert grasp(adapter).outcome == OUTCOME_COMPLETED
    bus.stalled = {"gripper"}

    result = release(adapter)

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"]["reason"] == "stalled"
    assert result.evidence["stop"]["confirmed"] is True
    assert bus.objects, "the object is still between the jaws"


# -- release -------------------------------------------------------------------


def test_a_release_completes_only_when_the_jaws_read_back_open():
    adapter, bus = gripper_arm()
    bus.place_object("gripper", 0.2)
    assert grasp(adapter).outcome == OUTCOME_COMPLETED

    result = release(adapter, open_to=0.45)

    assert result.outcome == OUTCOME_COMPLETED, result.detail
    outcome = result.evidence["arm_outcome"]
    assert outcome["reason"] == "released" and outcome["object_held"] is False
    jaws = result.evidence["observations"]["gripper_position"]
    assert abs(jaws["settled"]["gripper"] - 0.45) <= 0.02
    assert bus.objects == [] and len(bus.released) == 1
    # Nothing is left between the jaws: the next grasp closes on nothing.
    assert grasp(adapter, call_id="grasp-2").evidence["arm_outcome"]["object_held"] is False


@pytest.mark.parametrize(
    "arguments",
    [
        {"open_to_rad": 0.2},  # closer than the 0.3 release position
        {"open_to_rad": 0.51},  # past the joint's 0.5 limit
        {"open_to_rad": -0.1},
        {"open_to_rad": float("nan")},
        {"open_to_rad": float("inf")},
        {"open_to_rad": True},
        {"open_to_rad": "0.4"},
        {},
        {"open_to_rad": 0.4, "force": 1},
    ],
)
def test_a_release_target_out_of_range_is_refused_before_any_write(arguments):
    adapter, bus = gripper_arm(bus=SimulatedArmBus(JOINTS, positions={"gripper": 0.2}))

    result = adapter.invoke(CallRequest("r", CAPABILITY_RELEASE, arguments))

    assert result.outcome == OUTCOME_REFUSED
    assert bus.writes == [] and bus.torque_commands == []
    # Never clamped: the jaws are exactly where they were.
    assert bus.positions["gripper"] == 0.2


def test_a_release_target_on_the_ends_of_its_range_is_accepted():
    adapter, _ = gripper_arm(bus=SimulatedArmBus(JOINTS, positions={"gripper": 0.2}))

    assert release(adapter, "r1", open_to=0.3).outcome == OUTCOME_COMPLETED
    assert release(adapter, "r2", open_to=0.5).outcome == OUTCOME_COMPLETED


def test_a_grasp_takes_no_arguments():
    adapter, bus = gripper_arm()

    result = grasp(adapter, gripper_rad=0.1)

    assert result.outcome == OUTCOME_REFUSED
    assert bus.writes == []


def test_a_grasp_from_jaws_that_are_not_open_is_refused():
    adapter, bus = gripper_arm(bus=SimulatedArmBus(JOINTS, positions={"gripper": 0.1}))

    result = grasp(adapter)

    assert result.outcome == OUTCOME_REFUSED and "not open" in result.detail
    assert bus.writes == [] and bus.torque_commands == []


def test_grasp_and_release_are_refused_while_the_limits_are_not_accepted():
    adapter, bus = gripper_arm(limits_accepted=False)

    assert grasp(adapter).outcome == OUTCOME_REFUSED
    assert release(adapter).outcome == OUTCOME_REFUSED
    assert bus.writes == [] and bus.torque_commands == []
    assert adapter.safe_stop().outcome == OUTCOME_COMPLETED


def test_an_arm_without_a_gripper_refuses_grasp_and_release():
    profile = parse_profile(raw_profile())
    bus = SimulatedArmBus(JOINTS)
    adapter = SerialArmAdapter(bus, profile, clock=FakeClock(), sleep=lambda _s: None)

    assert grasp(adapter).outcome == OUTCOME_REFUSED
    assert release(adapter).outcome == OUTCOME_REFUSED
    assert bus.writes == []


def test_a_grasp_from_torque_off_holds_the_measured_pose_before_enabling():
    bus = SimulatedArmBus(JOINTS, positions={"gripper": 0.5, "shoulder_pan": 0.2})
    adapter, _ = gripper_arm(bus=bus)
    bus.place_object("gripper", 0.2)

    assert grasp(adapter).outcome == OUTCOME_COMPLETED
    assert bus.writes[0] == {"shoulder_pan": 0.2, "elbow_flex": 0.0, "gripper": 0.5}
    assert bus.torque_commands == [True]


# -- a dropped connection ------------------------------------------------------


@pytest.mark.parametrize("capability", [CAPABILITY_GRASP, CAPABILITY_RELEASE])
def test_a_connection_lost_mid_grip_fails_closed_and_is_never_retried(capability):
    start = 0.5 if capability == CAPABILITY_GRASP else 0.2
    bus = SimulatedArmBus(JOINTS, positions={"gripper": start}, torque_enabled=True)
    adapter, _ = gripper_arm(bus=bus)
    bus.place_object("gripper", 0.2)
    bus.drop_after_writes = 3
    arguments = {} if capability == CAPABILITY_GRASP else {"open_to_rad": 0.45}

    result = adapter.invoke(CallRequest("lost", capability, arguments))

    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["arm_outcome"] == {
        "reason": "connection_lost", "state_known": False, "retry": "never",
        "object_held": None,
    }
    writes = len(bus.writes)
    bus.connected = True
    bus.drop_after_writes = None
    # The same call again returns the unknown outcome; nothing is re-sent.
    assert adapter.invoke(CallRequest("lost", capability, arguments)) is result
    assert len(bus.writes) == writes
    # Every motion is refused until the state has been read again.
    for refused in (
        adapter.invoke(CallRequest("next", capability, arguments)),
        move(adapter, call_id="next-move"),
    ):
        assert refused.outcome == OUTCOME_REFUSED and "read state first" in refused.detail
    assert len(bus.writes) == writes
    assert adapter.read_state().outcome == OUTCOME_COMPLETED


# -- the stop during a grip ----------------------------------------------------


@pytest.mark.parametrize("capability", [CAPABILITY_GRASP, CAPABILITY_RELEASE])
def test_a_stop_during_a_grip_cancels_it_and_holds(capability):
    """Real threads and real time: the stop must not wait for the grip to end."""
    start = 0.5 if capability == CAPABILITY_GRASP else 0.3
    bus = SimulatedArmBus(JOINTS, positions={"gripper": start}, torque_enabled=True)
    raw = gripper_profile(max_speed_rad_s=0.05)
    raw["move_deadline_s"] = 10.0
    adapter = SerialArmAdapter(bus, parse_profile(raw))
    arguments = {} if capability == CAPABILITY_GRASP else {"open_to_rad": 0.5}
    outcome = {}

    def run():
        outcome["grip"] = adapter.invoke(CallRequest("g", capability, arguments))

    worker = threading.Thread(target=run)
    worker.start()
    time.sleep(0.3)
    stop = adapter.safe_stop()
    worker.join(timeout=3.0)

    assert not worker.is_alive()
    assert stop.outcome == OUTCOME_COMPLETED
    assert outcome["grip"].outcome == OUTCOME_CANCELLED
    assert outcome["grip"].evidence["arm_outcome"]["object_held"] is None
    writes = len(bus.writes)
    time.sleep(0.15)
    assert len(bus.writes) == writes, "nothing is written after the stop"


# -- the profile ---------------------------------------------------------------


@pytest.mark.parametrize(
    "gripper",
    [
        {"joint": "wrist"},  # not a declared joint
        {"closes_toward": "inward"},
        {"release_rad": 0.6},  # off the joint: never clamped
        {"release_rad": 0.05},  # too close to closed to read as released
        {"object_min_gap_rad": 0.02},  # not above the tolerance
        {"position_tolerance_rad": 0},
        {"hold": {"samples": 0, "max_drift_rad": 0.01}},
        {"hold": {"samples": 2}},
        {"squeeze": 1.0},  # misspelt or unknown key
    ],
)
def test_a_gripper_section_that_is_not_exact_is_refused(gripper):
    raw = gripper_profile()
    raw["gripper"].update(gripper)

    with pytest.raises(ProfileInvalid):
        parse_profile(raw)


def test_a_gripper_section_with_a_key_left_out_is_refused():
    raw = gripper_profile()
    del raw["gripper"]["object_min_gap_rad"]

    with pytest.raises(ProfileInvalid):
        parse_profile(raw)


def test_the_grippers_travel_is_its_joints_declared_limits():
    gripper = parse_profile(gripper_profile("max")).gripper

    assert (gripper.closed_rad, gripper.open_rad) == (0.5, 0.0)
    assert gripper.release_range == (0.0, 0.2)


# -- the LeRobot-style bus wrapper (unverified against real hardware) ----------


class StubGripperMotorsBus(StubMotorsBus):
    def __init__(self, motors):
        super().__init__()
        self.motors = {name: object() for name in motors}


def test_the_lerobot_bus_maps_a_gripper_only_when_the_motors_bus_exposes_it():
    joints = ("shoulder_pan", "gripper")
    exposed = StubGripperMotorsBus(joints)

    bus = LeRobotArmBus(exposed, joints=joints, degree_joints=joints, gripper="gripper")
    assert bus.gripper == "gripper"

    with pytest.raises(ValueError, match="does not expose"):
        LeRobotArmBus(
            StubGripperMotorsBus(("shoulder_pan",)), joints=joints, degree_joints=joints,
            gripper="gripper",
        )
    with pytest.raises(ValueError, match="does not expose"):
        # A motors bus that lists no motors cannot show that it has a gripper.
        LeRobotArmBus(StubMotorsBus(), joints=joints, degree_joints=joints, gripper="gripper")
    with pytest.raises(ValueError, match="not one of the joints"):
        LeRobotArmBus(exposed, joints=("shoulder_pan",), degree_joints=joints, gripper="gripper")
