"""A motion cut short by what stands ahead never reports ``completed``.

Live twin, task t-41169022a09d95b6 (0.6.5): ``motion.advance`` was asked for
1.2 m with a box ahead. Speed governance measured 0.0901 m of room to the floor
at send and shortened the goal to 0.0211 m (``plan.shortened``); Nav2
DriveOnHeading succeeded on that, the braking guard had also tripped on
clearance, and ``motion_outcome`` said ``completed`` / ``succeeded`` with 0.0345
of 1.2 m travelled. Cloud read "completed", planned no way round, and the task
failed as not proven. A run that stood a little further back drove, tripped the
guard mid-drive with a ``canceled`` status, and was routed round correctly: the
verdict depended on where the robot stood.

The reason now comes from one table over the motion's facts (requested,
commanded and travelled distance, the plan, the guard's trip, the nearest
return ahead), not from the action status. This file is that table's contract.
"""

from __future__ import annotations

import pytest

from flyto_robotics import braking_envelope as braking
from flyto_robotics import motion_outcome as mo
from flyto_robotics.adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED
from flyto_robotics.generic_ros2_adapter import SLOWDOWN_FRACTION
from tests.test_braking_envelope import (
    FLOOR,
    ApproachRosbridge,
    connected,
    odom,
    profile,
    scan,
    sweep_with,
)

DRIVE_ON_HEADING = "nav2_msgs/action/DriveOnHeading"

SUCCEEDED = mo.STATUS_SUCCEEDED
CANCELED = mo.STATUS_CANCELED
# The codes flyto-cloud's stop-reason table reads, spelled as it reads them.
OBSTACLE_BLOCKED = "obstacle_blocked"
PATH_BLOCKED = "path_blocked"
SENSOR_STALE = "sensor_stale"
COMPLETED = "completed"


def _plan(requested: float, commanded: float, room: float) -> dict:
    return {
        "mode": "planned_before_send",
        "requested_speed_mps": 0.2,
        "requested_distance_m": requested,
        "commanded_speed_mps": 0.05,
        "commanded_distance_m": commanded,
        "room_to_floor_at_send_m": room,
        "shortened": commanded < requested,
    }


def _summary(
    *,
    requested: float,
    commanded: float,
    travelled: float,
    status: int,
    tripped: str | None = None,
    planned: bool = True,
    ahead_m: float = 3.0,
    capability_id: str = "motion.advance",
) -> dict:
    braking_record: dict = {"tripped": tripped}
    if planned:
        braking_record["plan"] = _plan(requested, commanded, room=ahead_m - FLOOR)
    track = mo.MotionTrack(
        capability_id=capability_id,
        # The adapter sends the commanded distance; the ask is in the plan.
        arguments={"distance_m": commanded},
        start_pose={"x": 0.0, "y": 0.0, "yaw": 0.0},
        started_at=0.0,
        braking=braking_record,
    )
    # The robot drives along +x for an advance and -x for a retreat; the
    # sweep is in the robot frame, so "ahead" is the way it was going.
    sign = -1.0 if capability_id == "motion.retreat" else 1.0
    bearing = 180.0 if capability_id == "motion.retreat" else 0.0
    return mo.summarize(
        track,
        status=status,
        result_values={"error_code": 0},
        end_pose={"x": sign * travelled, "y": 0.0, "yaw": 0.0},
        range_observation={"minimum_range_m": ahead_m, "sweep": sweep_with({bearing: ahead_m})},
        clearance_floor_m=FLOOR,
        ended_at=1.0,
    )


# The contract. Each row: the motion's facts and the code they must produce.
CONTRACT = [
    pytest.param(
        # t-41169022a09d95b6, numbers as logged: planned 0.0211 of 1.2 m from
        # 0.0901 m of room, Nav2 succeeded, 0.0345 m travelled.
        dict(requested=1.2, commanded=0.0211, travelled=0.0345, status=SUCCEEDED,
             tripped="clearance", ahead_m=FLOOR + 0.0901 - 0.0345),
        OBSTACLE_BLOCKED,
        id="shortened-before-send-and-succeeded-is-obstacle_blocked",
    ),
    pytest.param(
        # The same planned stop without a guard trip.
        dict(requested=1.2, commanded=0.0211, travelled=0.0211, status=SUCCEEDED,
             ahead_m=FLOOR + 0.0901 - 0.0211),
        OBSTACLE_BLOCKED,
        id="shortened-before-send-without-trip-is-obstacle_blocked",
    ),
    pytest.param(
        # The same for a retreat (BackUp): the rule does not name an action.
        dict(requested=0.5, commanded=0.05, travelled=0.05, status=SUCCEEDED,
             ahead_m=FLOOR + 0.12, capability_id="motion.retreat"),
        OBSTACLE_BLOCKED,
        id="shortened-retreat-is-obstacle_blocked",
    ),
    pytest.param(
        # No room at all: nothing the odometry could see was commanded.
        dict(requested=1.2, commanded=0.0, travelled=0.0, status=SUCCEEDED,
             ahead_m=FLOOR + 0.001),
        PATH_BLOCKED,
        id="zero-room-is-path_blocked",
    ),
    pytest.param(
        # Mid-motion trip on clearance: the guard cancels, the code stands.
        dict(requested=1.2, commanded=1.2, travelled=0.4, status=CANCELED,
             tripped="clearance", planned=False, ahead_m=FLOOR + 0.05),
        OBSTACLE_BLOCKED,
        id="mid-motion-clearance-trip-keeps-obstacle_blocked",
    ),
    pytest.param(
        # Mid-motion trip because the LiDAR went blind: still sensor_stale.
        dict(requested=1.2, commanded=1.2, travelled=0.4, status=CANCELED,
             tripped="blind", planned=False),
        SENSOR_STALE,
        id="mid-motion-blind-trip-keeps-sensor_stale",
    ),
    pytest.param(
        dict(requested=1.2, commanded=1.2, travelled=1.2, status=SUCCEEDED, planned=False),
        COMPLETED,
        id="full-length-move-is-completed",
    ),
    pytest.param(
        # Shortened by the plan, but the request is reached within the
        # relative-move arrival tolerance (min(0.03, d/10) = 0.03 m at 1.2 m).
        dict(requested=1.2, commanded=1.18, travelled=1.175, status=SUCCEEDED,
             ahead_m=FLOOR + 0.2),
        COMPLETED,
        id="shortened-but-within-arrival-tolerance-is-completed",
    ),
]


@pytest.mark.parametrize(("facts", "expected"), CONTRACT)
def test_the_stop_code_follows_the_motion_facts(facts, expected):
    summary = _summary(**facts)
    assert summary["reason"] == expected


def test_the_defect_names_the_rule_that_decided_it():
    summary = _summary(
        requested=1.2, commanded=0.0211, travelled=0.0345, status=SUCCEEDED,
        tripped=mo.GUARD_TRIP_CLEARANCE, ahead_m=FLOOR + 0.0901 - 0.0345,
    )
    assert summary["action_status_name"] == "succeeded"  # what Nav2 said
    assert summary["reason"] == mo.REASON_OBSTACLE_BLOCKED  # what happened
    rule = summary["stop_rule"]
    assert rule["name"] == "stopped_short_by_clearance"
    assert rule["basis"]
    assert rule["shortfall_m"] == pytest.approx(1.2 - 0.0345, abs=1e-4)
    assert rule["arrival_tolerance_m"] == pytest.approx(0.03)


def test_every_rule_names_its_basis():
    names = [rule.name for rule in mo.STOP_RULES]
    assert len(names) == len(set(names))
    for rule in mo.STOP_RULES:
        assert rule.basis.strip(), rule.name
        assert rule.reason in mo.REASONS


def test_the_tolerance_is_the_relative_move_arrival_rule():
    from flyto_robotics.mission import relative_move_tolerance

    for requested in (0.005, 0.05, 0.2, 1.2, 5.0):
        facts = mo.StopFacts(
            requested_m=requested, commanded_m=requested, travelled_m=0.0,
            shortened=False, trip=None, ahead_inside_floor=False,
        )
        assert facts.arrival_tolerance_m == relative_move_tolerance(requested)


@pytest.mark.parametrize(
    ("reason", "outcome"),
    [
        (OBSTACLE_BLOCKED, OUTCOME_FAILED),
        (PATH_BLOCKED, OUTCOME_FAILED),
        (COMPLETED, OUTCOME_COMPLETED),
    ],
)
def test_a_succeeded_goal_takes_its_outcome_from_the_reason(reason, outcome):
    from flyto_robotics.generic_ros2_adapter import _succeeded_outcome

    assert _succeeded_outcome({"reason": reason}) == outcome


# -- end to end through the rosbridge transport ---------------------------------------


def _defect_goal(goal):
    """t-41169022a09d95b6: a 2 cm goal that Nav2 finishes while the guard trips."""
    if goal["id"] != "adv":
        return []
    return [
        odom(0.02, 0.05),
        scan(FLOOR + 0.005),  # room 5 mm: inside any stopping distance
        odom(0.0345, 0.0),
        {"op": "action_result", "id": "adv", "status": 4, "result": True,
         "values": {"error_code": 0}},
    ]


def test_the_live_defect_fails_with_obstacle_blocked_through_rosbridge(monkeypatch):
    socket = ApproachRosbridge(_defect_goal)
    backend = connected(socket, monkeypatch)
    governance = braking.decide_governance(DRIVE_ON_HEADING, transport_resends=True)
    guard = profile()
    plan = braking.plan_drive(
        guard, room_m=0.0901, requested_speed_mps=0.2, requested_distance_m=1.2,
        mode=governance.mode, speed_fraction=SLOWDOWN_FRACTION,
    )
    assert plan.shortened and plan.refusal is None
    backend.arm_braking_guard(
        "adv", "motion.advance", profile=guard, speed_mps=plan.speed_mps,
        requested_speed_mps=0.2, distance_m=plan.distance_m, start_room_m=0.0901,
        governance=governance, plan=plan,
    )
    result = backend.invoke(
        call_id="adv", capability_id="motion.advance",
        arguments={"distance_m": plan.distance_m, "speed_mps": plan.speed_mps},
        deadline_seconds=5.0,
    )
    backend.disconnect()

    outcome = result.evidence["motion_outcome"]
    assert outcome["requested_distance_m"] == 1.2
    assert outcome["reason"] == OBSTACLE_BLOCKED
    assert result.outcome == OUTCOME_FAILED
    assert result.detail.startswith(
        f"ROS 2 action status {outcome['action_status']} "
        f"({outcome['action_status_name']}): obstacle_blocked"
    )
