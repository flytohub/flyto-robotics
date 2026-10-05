"""The floor applies along the path a motion sweeps, not all around.

Live case (2026-10-06, physical TurtleBot3): a return 0.334 m behind the robot
at about 168 degrees, 1.445 m ahead; a forward advance was refused with 0 m
travelled because the preflight read the nearest return in any direction.
"""

from __future__ import annotations

import math

import pytest
from test_braking_envelope import sweep_with
from test_generic_ros2_adapter import adapter

from flyto_robotics import adapter_contract as contract
from flyto_robotics import path_clearance as pc
from flyto_robotics.adapter_contract import OUTCOME_REFUSED, CallRequest
from flyto_robotics.generic_ros2_adapter import _motion_kind

FLOOR = 0.35
RADIUS = 0.10

# The live sweep: chair behind, room ahead, walls well to either side.
LIVE = {168: 0.334, 0: 1.445, 90: 0.80, -90: 0.90}


def judge(points, kind, **kwargs):
    return pc.judge(
        sweep_with(points), motion_kind=kind, floor_m=FLOOR, radius_m=RADIUS, **kwargs
    )


def test_the_live_rear_return_does_not_block_a_forward_advance():
    verdict = judge(LIVE, "advance")
    assert verdict.admitted
    assert verdict.clearance_m == pytest.approx(1.445)
    assert verdict.limiting_side == "ahead"
    assert verdict.sectors["behind"] == pytest.approx(0.334)
    assert verdict.sectors["ahead"] == pytest.approx(1.445)


def test_the_live_rear_return_blocks_a_retreat_and_says_which_side():
    verdict = judge(LIVE, "retreat")
    assert not verdict.admitted
    assert verdict.reason == "path_blocked"
    assert verdict.clearance_m == pytest.approx(0.334)
    assert verdict.limiting_side == "behind"
    assert verdict.threshold_m == FLOOR  # the floor is not lowered
    evidence = verdict.evidence()
    assert evidence["reason_code"] == "path_blocked"
    assert evidence["path_clearance"]["limiting_side"] == "behind"
    assert set(evidence["path_clearance"]["sectors"]) == {"ahead", "left", "behind", "right"}
    assert "0.334m behind" in verdict.describe()
    assert "below the 0.350m motion safety minimum" in verdict.describe()


@pytest.mark.parametrize(
    "side,bearing", [("ahead", 0), ("left", 90), ("behind", 180), ("right", -90)]
)
def test_a_rotation_needs_floor_plus_radius_on_every_side(side, bearing):
    points = {0: 1.0, 90: 1.0, 180: 1.0, -90: 1.0, bearing: FLOOR + RADIUS - 0.01}
    verdict = judge(points, "rotate")
    assert not verdict.admitted
    assert verdict.threshold_m == pytest.approx(FLOOR + RADIUS)
    assert verdict.limiting_side == side
    assert "rotation radius 0.100m" in verdict.describe()


def test_a_rotation_with_room_all_around_is_admitted():
    assert judge({0: 0.46, 90: 0.46, 180: 0.46, -90: 0.46}, "rotate").admitted


def test_a_return_beside_the_path_closer_than_the_floor_blocks_a_straight_drive():
    x, y = 0.05, 0.30  # just ahead, beside the robot: passing keeps it under the floor
    bearing = math.degrees(math.atan2(y, x))
    verdict = pc.judge(
        sweep_with({bearing: math.hypot(x, y)}, bins=3600),
        motion_kind="advance",
        floor_m=FLOOR,
        radius_m=RADIUS,
    )
    assert not verdict.admitted and verdict.limiting_side == "left"


def test_a_return_in_the_corridor_beyond_the_floor_is_reported_not_refused():
    x, y = 0.30, 0.40  # |y| < floor + radius: in the corridor, 0.5 m away
    verdict = pc.judge(
        sweep_with({math.degrees(math.atan2(y, x)): math.hypot(x, y)}, bins=3600),
        motion_kind="advance",
        floor_m=FLOOR,
        radius_m=RADIUS,
    )
    assert verdict.admitted
    assert verdict.clearance_m == pytest.approx(0.5, abs=1e-3)


def test_a_planned_navigation_is_left_to_nav2_and_the_in_motion_guard():
    verdict = judge({180: 0.2, 0: 0.2}, "planned")
    assert verdict.admitted and verdict.reason == "left_to_planner"
    assert verdict.sectors["behind"] == pytest.approx(0.2)


def test_without_a_sweep_the_nearest_return_is_judged_all_around():
    blocked = pc.judge(None, motion_kind="advance", floor_m=FLOOR, radius_m=RADIUS,
                       minimum_range_m=0.2)
    assert not blocked.admitted and blocked.shape == "footprint"
    clear = pc.judge(None, motion_kind="advance", floor_m=FLOOR, radius_m=RADIUS,
                     minimum_range_m=0.5)
    assert clear.admitted
    nothing = pc.judge(None, motion_kind="advance", floor_m=FLOOR, radius_m=RADIUS)
    assert not nothing.admitted and nothing.reason == "path_unreadable"


def test_the_motion_kind_comes_from_the_capability_contract():
    assert _motion_kind("motion.advance") == "advance"
    assert _motion_kind("motion.retreat") == "retreat"
    assert _motion_kind("motion.rotate") == "rotate"
    assert _motion_kind("motion.navigate") == "planned"
    assert _motion_kind("motion.halt") is None
    for capability_id in ("motion.advance", "motion.retreat", "motion.rotate", "motion.navigate"):
        kind = contract.capability_metadata(capability_id)["motion_kind"]
        assert kind in pc.SWEPT_PATHS


def _live_device(capability_id: str):
    device = adapter({capability_id})
    device.backend.observation_payload["range"] = {
        "minimum_range_m": 0.334,
        "sample_count": 400,
        "sweep": sweep_with(LIVE),
    }
    device.describe()
    return device


def test_the_adapter_admits_the_live_forward_advance_at_preflight():
    assert _live_device("motion.advance")._motion_preflight("motion.advance") is None


def test_the_adapter_refuses_the_live_retreat_with_structured_sectors():
    device = _live_device("motion.retreat")
    result = device.invoke(CallRequest("back", "motion.retreat", {"distance_m": 0.2}))
    assert result.outcome == OUTCOME_REFUSED
    assert result.evidence["reason_code"] == "path_blocked"
    clearance = result.evidence["path_clearance"]
    assert clearance["limiting_side"] == "behind"
    assert clearance["sectors"]["ahead"] == pytest.approx(1.445)
    assert clearance["sectors"]["behind"] == pytest.approx(0.334)
    assert device.backend.calls == []
