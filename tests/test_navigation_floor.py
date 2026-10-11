"""The 0.35 m floor on navigation legs: where a leg ends, and while it runs.

Real robot, 2026-10-09, run c6d8d79d: a person stood in front of the robot.
The advance stopped correctly 0.552 m from their shins, the inflation escape
backed off and moved 0.35 m left, and the goal leg then navigated to the
blocked move's original destination, about 0.1 m in front of the shins. The
minimum range during that leg was 0.135 m (``floor_held: false``); Nav2's
collision monitor only slowed the base. Two things were missing, and each
test here fails on 0.6.6:

* nothing checked that a leg's destination keeps the floor
  (``path_clearance.judge_destination``);
* nothing held the floor while a Nav2 leg ran (``braking.leg_floor_check``,
  the backend's floor guard).
"""

from __future__ import annotations

import math

import pytest

from flyto_robotics import braking_envelope as braking
from flyto_robotics import motion_outcome as mo
from flyto_robotics import path_clearance
from flyto_robotics.adapter_contract import OUTCOME_COMPLETED, OUTCOME_FAILED, OUTCOME_REFUSED
from tests.test_braking_envelope import ApproachRosbridge, connected, odom, profile, scan
from tests.test_inflation_escape import (
    FLOOR,
    OBSERVED,
    EscapeBackend,
    box,
    make,
    navigate,
    room,
)
from tests.test_inflation_escape import scan as ray_scan

START = {"frame": "map", "x": 0.0, "y": 0.0, "yaw": 0.0}
# The person's shins as the LiDAR saw them: 0.27 m wide, 0.552 m ahead.
SHINS = box(0.552, 0.10, 0.27)
# The blocked move's original destination: about 0.1 m in front of them.
INCIDENT_DESTINATION = (0.452, 0.0)


@pytest.fixture(autouse=True)
def _simulation_marker_off(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    monkeypatch.delenv("FLYTO_ROS2_INFLATION_ESCAPE", raising=False)


# -- the destination ----------------------------------------------------------


def test_the_incident_destination_in_front_of_the_shins_is_refused():
    verdict = path_clearance.judge_destination(
        ray_scan(SHINS + room()), START, INCIDENT_DESTINATION, floor_m=FLOOR, leg="goal"
    )
    assert not verdict.admitted
    assert verdict.reason == path_clearance.REASON_DESTINATION_BLOCKED == "destination_blocked"
    assert verdict.clearance_m == pytest.approx(0.10, abs=0.01)
    assert verdict.returns_inside_floor >= 5
    record = verdict.to_dict()
    assert record["limiting_side"] == "ahead"
    assert record["destination_robot_frame"] == {"x": 0.452, "y": 0.0}
    assert "inside the 0.350 m clearance floor" in verdict.describe()
    assert verdict.evidence()["reason_code"] == "destination_blocked"


def test_a_destination_the_floor_away_from_every_return_is_admitted():
    # 0.36 m short of the shins: just outside the floor.
    verdict = path_clearance.judge_destination(
        ray_scan(SHINS + room()), START, (0.552 - 0.36, 0.0), floor_m=FLOOR, leg="goal"
    )
    assert verdict.admitted and verdict.reason == path_clearance.REASON_CLEAR
    assert verdict.clearance_m == pytest.approx(0.36, abs=0.01)


def test_the_destination_is_placed_from_the_robot_map_pose():
    # The robot stands at (2, 1) facing +y: the shins are 0.552 m along +y.
    pose = {"frame": "map", "x": 2.0, "y": 1.0, "yaw": math.pi / 2}
    sweep = ray_scan(SHINS + room())
    blocked = path_clearance.judge_destination(
        sweep, pose, (2.0, 1.452), floor_m=FLOOR, leg="goal"
    )
    clear = path_clearance.judge_destination(
        sweep, pose, (2.452, 1.0), floor_m=FLOOR, leg="goal"
    )
    assert not blocked.admitted
    assert clear.admitted


@pytest.mark.parametrize(
    "sweep, pose",
    [
        (None, START),
        ({"angle_min_rad": 0.0, "angle_increment_rad": 0.1, "ranges_m": [None] * 63}, START),
        (ray_scan(room()), None),
    ],
)
def test_a_destination_that_cannot_be_judged_is_refused(sweep, pose):
    verdict = path_clearance.judge_destination(
        sweep, pose, (1.0, 0.0), floor_m=FLOOR, leg="goal"
    )
    assert not verdict.admitted
    assert verdict.reason == path_clearance.REASON_UNREADABLE


def test_the_floor_is_the_one_passed_in_and_never_lowered():
    sweep = ray_scan(SHINS + room())
    at = path_clearance.judge_destination(sweep, START, (0.552 - 0.30, 0.0), floor_m=0.35, leg="g")
    assert not at.admitted
    assert at.to_dict()["floor_m"] == 0.35


# -- the destination through the adapter ----------------------------------------


def test_navigate_to_the_incident_destination_is_refused_before_anything_moves():
    backend = EscapeBackend(SHINS + room())
    result = navigate(make(backend), x=INCIDENT_DESTINATION[0], y=INCIDENT_DESTINATION[1])
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == "destination_blocked"
    assert result.evidence["destination_clearance"]["leg"] == "goal"
    assert result.evidence["navigation_target"]["x"] == pytest.approx(0.452)
    assert backend.calls == []  # no back-off, no waypoint, no goal


class SteppingInBackend(EscapeBackend):
    """The escape's legs succeed; after the waypoint someone stands on the goal."""

    def __init__(self, segments, after_waypoint, **kwargs):
        super().__init__(segments, **kwargs)
        self.after_waypoint = after_waypoint

    def invoke(self, *, call_id, capability_id, arguments, deadline_seconds):
        result = super().invoke(
            call_id=call_id,
            capability_id=capability_id,
            arguments=arguments,
            deadline_seconds=deadline_seconds,
        )
        if call_id.endswith(":escape-waypoint"):
            # The robot now stands at the waypoint and sees the scene from there.
            x, y, yaw = arguments["x"], arguments["y"], arguments["yaw_radians"]

            def local(point):
                dx, dy = point[0] - x, point[1] - y
                return (
                    math.cos(yaw) * dx + math.sin(yaw) * dy,
                    -math.sin(yaw) * dx + math.cos(yaw) * dy,
                )

            seen = [(local(a), local(b)) for a, b in self.after_waypoint]
            self.payload = {
                **self.payload,
                "map_pose": {"frame": "map", "x": x, "y": y, "yaw": yaw},
                "range": {"minimum_range_m": 0.4, "sample_count": 360, "sweep": ray_scan(seen)},
            }
        return result


def test_the_goal_leg_is_refused_when_its_destination_is_no_longer_clear():
    # The goal 1.2 m ahead is clear at the start (the box face is 0.8 m from
    # it); after the waypoint a person stands 0.1 m beyond the goal.
    person = box(1.30, 0.10, 0.27)
    backend = SteppingInBackend(OBSERVED, OBSERVED + person)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_FAILED  # never "completed", never arrived
    assert result.evidence["reason_code"] == "destination_blocked"
    assert result.detail.startswith("inflation escape goal refused")
    assert [call for call, _, _ in backend.calls] == [
        "nav-1:escape-backoff",
        "nav-1:escape-waypoint",
    ]
    legs = result.evidence["navigation_escape"]["legs"]
    assert [leg["leg"] for leg in legs] == ["backoff", "waypoint", "goal"]
    assert legs[-1]["outcome"] == OUTCOME_REFUSED


class GuardRecordingBackend(EscapeBackend):
    def __init__(self, segments, **kwargs):
        super().__init__(segments, **kwargs)
        self.armed: list[tuple[str, str]] = []

    def braking_profile(self):
        return profile()

    def arm_floor_guard(self, call_id, capability_id, *, profile):
        assert profile.floor_m == FLOOR
        self.armed.append((call_id, capability_id))


def test_every_leg_of_an_escape_is_held_to_the_floor():
    backend = GuardRecordingBackend(OBSERVED)
    result = navigate(make(backend))
    assert result.outcome == OUTCOME_COMPLETED
    assert backend.armed == [
        ("nav-1:escape-backoff", "motion.retreat"),
        ("nav-1:escape-waypoint", "motion.navigate"),
        ("nav-1", "motion.navigate"),
    ]


def test_a_plain_navigation_is_held_to_the_floor():
    backend = GuardRecordingBackend(box(1.0, 0.3, 0.4) + room())
    result = navigate(make(backend), x=0.50)
    assert result.outcome == OUTCOME_COMPLETED
    assert backend.armed == [("nav-1", "motion.navigate")]


# -- the floor while a leg runs: pure ---------------------------------------------


def sweep_of(points: dict[float, float], bins: int = 180) -> dict:
    ranges = [3.0] * bins
    for bearing, value in points.items():
        ranges[round(bearing / (math.tau / bins)) % bins] = value
    return {"angle_min_rad": 0.0, "angle_increment_rad": math.tau / bins, "ranges_m": ranges}


def test_a_return_inside_the_floor_in_any_direction_breaches_it():
    check = braking.leg_floor_check(sweep_of({math.pi / 2: 0.30}), profile(), 0.0)
    assert check.breached and check.cause == braking.LEG_INSIDE_FLOOR
    assert check.to_dict()["nearest_direction"] == "left"


def test_the_way_the_base_moves_it_is_stopped_while_it_can_still_stop_at_the_floor():
    # 0.20 m/s, latency 0.5 s, 0.5 m/s^2: 0.14 m to stop; 0.45 m ahead leaves 0.10.
    check = braking.leg_floor_check(sweep_of({0.0: 0.45}), profile(0.5, 0.5), 0.20)
    assert check.breached and check.cause == braking.LEG_NO_ROOM_TO_STOP
    assert check.travel_room_m == pytest.approx(0.10, abs=1e-3)
    # The same return behind a base moving forward does not stop it ...
    behind = braking.leg_floor_check(sweep_of({math.pi: 0.45}), profile(0.5, 0.5), 0.20)
    assert not behind.breached
    # ... but does when it backs up.
    backing = braking.leg_floor_check(sweep_of({math.pi: 0.45}), profile(0.5, 0.5), -0.20)
    assert backing.breached and backing.cause == braking.LEG_NO_ROOM_TO_STOP


def test_a_wall_passed_alongside_outside_the_floor_does_not_stop_a_leg():
    check = braking.leg_floor_check(sweep_of({math.pi / 2: 0.40}), profile(0.5, 0.5), 0.22)
    assert not check.breached


def test_an_unreadable_sweep_is_not_room_for_a_leg():
    assert braking.leg_floor_check(None, profile(), 0.1) is None


# -- the floor while a leg runs: through the rosbridge transport --------------------


def _arm_leg(backend, call_id="nav-1"):
    backend.arm_floor_guard(call_id, "motion.navigate", profile=profile(0.5, 0.5))


def test_the_floor_guard_stops_a_navigation_leg_that_reaches_a_person(monkeypatch):
    # The c6d8d79d goal leg: the base drove on toward the shins while Nav2's
    # collision monitor only slowed it. Now the first scan inside the floor
    # stops it: zero, cancel, zero.
    def approach(goal):
        yield odom(0.2, 0.05)
        yield scan(0.60)
        yield odom(0.3, 0.05)
        yield scan(0.30)

    socket = ApproachRosbridge(lambda goal: list(approach(goal)))
    backend = connected(socket, monkeypatch)
    _arm_leg(backend)
    result = backend.invoke(
        call_id="nav-1", capability_id="motion.navigate",
        arguments={"x": 0.452, "y": 0.0}, deadline_seconds=5.0,
    )
    backend.disconnect()

    ops = [message["op"] for message in socket.sent]
    assert "cancel_action_goal" in ops
    cancel_at = ops.index("cancel_action_goal")
    assert "publish" in ops[cancel_at - 1:cancel_at] and "publish" in ops[cancel_at + 1:]
    assert result.outcome == OUTCOME_FAILED
    outcome = result.evidence["motion_outcome"]
    assert outcome["reason"] == mo.REASON_OBSTACLE_BLOCKED
    assert outcome["braking"]["kind"] == "navigation_leg"
    assert outcome["braking"]["tripped"] == mo.GUARD_TRIP_FLOOR == "floor_breach"
    assert outcome["braking"]["trip"]["cause"] == braking.LEG_INSIDE_FLOOR
    assert outcome["braking"]["trip"]["nearest_m"] == pytest.approx(0.30, abs=1e-3)
    assert "floor guard stopped the leg (inside_floor)" in result.detail


def test_a_leg_its_server_calls_succeeded_after_the_floor_broke_did_not_arrive(monkeypatch):
    def approach(goal):
        yield odom(0.3, 0.05)
        yield scan(0.30)

    def finish(goal):
        yield {"op": "action_result", "id": goal["id"], "status": 4, "result": True}

    socket = ApproachRosbridge(lambda goal: list(approach(goal)), finish=finish)
    backend = connected(socket, monkeypatch)
    _arm_leg(backend)
    result = backend.invoke(
        call_id="nav-1", capability_id="motion.navigate",
        arguments={"x": 0.452, "y": 0.0}, deadline_seconds=5.0,
    )
    backend.disconnect()
    assert result.outcome == OUTCOME_FAILED
    assert result.evidence["motion_outcome"]["reason"] == mo.REASON_OBSTACLE_BLOCKED


def test_a_leg_that_keeps_the_floor_is_not_stopped(monkeypatch):
    def approach(goal):
        yield odom(0.2, 0.05)
        yield scan(0.90)

    def finish(goal):
        yield {"op": "action_result", "id": goal["id"], "status": 4, "result": True}

    socket = ApproachRosbridge(lambda goal: list(approach(goal)), finish=finish)
    backend = connected(socket, monkeypatch)
    _arm_leg(backend)
    result = backend.invoke(
        call_id="nav-1", capability_id="motion.navigate",
        arguments={"x": 0.452, "y": 0.0}, deadline_seconds=5.0,
    )
    backend.disconnect()
    assert "cancel_action_goal" not in [message["op"] for message in socket.sent]
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["motion_outcome"]["braking"]["tripped"] is None
