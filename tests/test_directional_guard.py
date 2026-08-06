"""Stopping for the direction of travel, with a floor under every relaxation."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import compile_workflow, load_plan
from flyto_robotics.contracts import load_job
from flyto_robotics.mission import (
    INTENT_FORWARD,
    INTENT_REVERSE,
    INTENT_ROTATE,
    MissionController,
    Pose2D,
    RangeField,
)

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples" / "jobs" / "tb3-lab-shortcut.json"
FORWARD_PLAN = ROOT / "examples" / "plans" / "shortcut-forward-40cm.json"
BACKWARD_PLAN = ROOT / "examples" / "plans" / "shortcut-backward-40cm.json"


def controller(plan=FORWARD_PLAN, **safety):
    job = load_job(JOB)
    if safety:
        job = job.__class__(**{**job.__dict__, "safety": job.safety.__class__(
            **{**job.safety.__dict__, **safety})})
    return job, MissionController(job, workflow=compile_workflow(load_plan(plan)))


def drive(c, field):
    c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=field, now=0.0)
    return c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=field, now=0.3)


def corridor(width_cm, *, ahead=2.0, rear=2.0):
    """A robot centred in an aisle: walls to each side, clear fore and aft."""
    wall = width_cm / 200
    return RangeField(forward=ahead, left=wall, right=wall, rear=rear,
                      closest=min(wall, ahead, rear), directional=True)


# -- what started this ---------------------------------------------------


def test_an_omnidirectional_gate_cannot_enter_a_narrow_aisle():
    """The measurement behind the change: a 28 cm aisle puts each wall 14 cm
    from the lidar at the robot's centre, and one 0.25 m threshold in every
    direction refuses every command forever."""
    _, c = controller()
    assert drive(c, 0.14).linear_x == 0.0


def test_the_same_aisle_is_passable_once_the_sectors_are_separate():
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, corridor(28)).linear_x > 0.0


# -- the direction of travel decides which sector matters ----------------


def test_reversing_is_judged_on_what_is_behind_not_in_front():
    """A robot backing up is not protected by a clear view ahead."""
    _, c = controller(BACKWARD_PLAN, lateral_stop_distance=0.10)
    blocked = drive(c, RangeField(forward=2.0, rear=0.10, left=2.0, right=2.0,
                                  closest=0.10, directional=True))
    assert blocked.linear_x == 0.0
    assert "behind" in c.events[-1].detail


def test_reversing_is_not_stopped_by_something_ahead_of_it():
    _, c = controller(BACKWARD_PLAN, lateral_stop_distance=0.10)
    assert drive(c, RangeField(forward=0.15, rear=2.0, left=2.0, right=2.0,
                               closest=0.15, directional=True)).linear_x != 0.0


def test_turning_in_place_watches_every_side():
    """A rotation sweeps the whole footprint, so a wall it was safely parallel
    to becomes the wall it turns into."""
    _, c = controller(lateral_stop_distance=0.10)
    c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=2.0, now=0.0)
    guarded = c._obstacle_guard(
        RangeField(forward=2.0, rear=0.12, left=2.0, right=2.0,
                   closest=0.12, directional=True),
        0.3,
        intent=INTENT_ROTATE,
    )
    assert guarded is not None and "in the turn" in c.events[-1].detail


def test_driving_forward_is_not_stopped_by_a_wall_it_is_parallel_to():
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, corridor(28)).linear_x > 0.0


def test_forward_protection_is_unchanged_by_relaxing_the_sides():
    """The reason for two thresholds rather than one smaller one."""
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, corridor(28, ahead=0.20)).linear_x == 0.0
    assert "ahead" in c.events[-1].detail


# -- the floor under every relaxation ------------------------------------


def test_nothing_may_be_nearer_than_the_emergency_distance_in_any_direction():
    """Loosening the sides for a narrow aisle must never become permission to
    graze one."""
    job, c = controller(lateral_stop_distance=0.02)
    assert job.safety.emergency_stop_distance == pytest.approx(0.08)
    stopped = drive(c, RangeField(forward=2.0, left=0.04, right=2.0, rear=2.0,
                                  closest=0.04, directional=True))
    assert stopped.linear_x == 0.0
    assert "emergency" in c.events[-1].detail


def test_the_emergency_floor_applies_while_reversing_too():
    _, c = controller(BACKWARD_PLAN, lateral_stop_distance=0.02)
    stopped = drive(c, RangeField(forward=0.05, rear=2.0, left=2.0, right=2.0,
                                  closest=0.05, directional=True))
    assert stopped.linear_x == 0.0
    assert "emergency" in c.events[-1].detail


# -- nothing older changes behaviour -------------------------------------


def test_a_scalar_reading_keeps_the_omnidirectional_rule():
    """Absent sectors means nothing was measured there, never that nothing is
    there — so a caller that passes one number gets the old, stricter gate."""
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, 0.20).linear_x == 0.0


def test_a_job_that_sets_no_lateral_limit_keeps_the_forward_distance():
    job, c = controller()
    assert job.safety.lateral_stop_distance is None
    assert drive(c, corridor(28)).linear_x == 0.0


@pytest.mark.parametrize("width", [25, 28, 30])
def test_the_arena_aisles_are_passable(width):
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, corridor(width)).linear_x > 0.0


# -- the wire the tests missed -------------------------------------------


def test_a_sweep_becomes_sectors_not_one_number():
    """The gap that let a fixed guard stay unreachable: every test exercised
    _obstacle_guard with a RangeField, and nothing asserted that anything ever
    built one. The nodes fed it a scalar, so the directional path never ran."""
    from flyto_robotics.mission import sector_field

    n = 360
    ranges = [2.0] * n
    for i in range(n):
        if abs(i - 90) <= 25 or abs(i - 270) <= 25:
            ranges[i] = 0.14          # corridor walls
        if abs(i - 45) <= 3 or abs(i - 315) <= 3:
            ranges[i] = 0.12          # the corners of an intersection

    field = sector_field(ranges, angle_min=0.0, angle_increment=math.radians(1.0))
    assert field.directional is True
    assert field.forward == pytest.approx(2.0)
    assert field.left == pytest.approx(0.14)
    assert field.right == pytest.approx(0.14)
    assert field.closest == pytest.approx(0.12), "the diagonal belongs to no named sector"


def test_a_sector_with_no_return_stays_unknown_rather_than_clear():
    """Infinity means nothing was seen there, which is not the same as nothing
    being there — but it is the only thing a sweep can say."""
    from flyto_robotics.mission import sector_field

    field = sector_field([math.inf] * 360, angle_min=0.0, angle_increment=math.radians(1.0))
    assert field.forward == math.inf and field.closest == math.inf


def test_out_of_band_returns_are_discarded():
    from flyto_robotics.mission import sector_field

    field = sector_field(
        [0.001, 50.0, 0.30] + [math.inf] * 357,
        angle_min=0.0, angle_increment=math.radians(1.0),
        range_min=0.05, range_max=12.0,
    )
    assert field.forward == pytest.approx(0.30)
