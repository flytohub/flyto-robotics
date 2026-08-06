"""Stopping for what is ahead, separately from what is beside."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import compile_workflow, load_plan
from flyto_robotics.contracts import load_job
from flyto_robotics.mission import MissionController, Pose2D

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples" / "jobs" / "tb3-lab-shortcut.json"
PLAN = ROOT / "examples" / "plans" / "shortcut-forward-40cm.json"


def controller(**safety):
    job = load_job(JOB)
    if safety:
        job = job.__class__(**{**job.__dict__, "safety": job.safety.__class__(
            **{**job.safety.__dict__, **safety})})
    return job, MissionController(job, workflow=compile_workflow(load_plan(PLAN)))


def drive(c, *, ahead, beside=math.inf):
    c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=ahead, now=0.0, lateral_range=beside)
    return c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=ahead, now=0.3, lateral_range=beside)


def test_a_corridor_narrower_than_twice_the_stop_distance_used_to_be_impassable():
    """The measurement that started this: a 25 cm aisle put the walls 12.5 cm
    away, inside a 0.25 m omnidirectional gate, so the robot never moved."""
    _, c = controller()
    assert drive(c, ahead=0.125, beside=0.125).linear_x == 0.0


def test_the_same_corridor_is_passable_once_the_walls_are_judged_separately():
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, ahead=2.0, beside=0.125).linear_x > 0.0


def test_lowering_the_lateral_limit_does_not_lower_the_protection_in_front():
    """The reason for two thresholds rather than one smaller one: a hand in the
    robot's path must still stop it early."""
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, ahead=0.20, beside=2.0).linear_x == 0.0


def test_something_too_close_beside_still_stops_it():
    _, c = controller(lateral_stop_distance=0.10)
    assert drive(c, ahead=2.0, beside=0.06).linear_x == 0.0


def test_a_job_that_sets_no_lateral_limit_behaves_exactly_as_before():
    """Every job written before the split keeps the omnidirectional rule."""
    job, c = controller()
    assert job.safety.lateral_stop_distance is None
    assert drive(c, ahead=2.0, beside=0.20).linear_x == 0.0


def test_omitting_the_lateral_reading_means_nothing_is_beside():
    """Callers written before the split pass one range and must keep working."""
    _, c = controller(lateral_stop_distance=0.10)
    c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=2.0, now=0.0)
    assert c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=2.0, now=0.3).linear_x > 0.0


def test_the_reason_says_which_side_it_stopped_for():
    """An operator reading the timeline needs to know whether to move the
    obstacle or widen the aisle."""
    _, c = controller(lateral_stop_distance=0.10)
    drive(c, ahead=0.10, beside=2.0)
    ahead_events = [e for e in c.events if e.kind == "obstacle_stop"]
    assert ahead_events and "ahead" in ahead_events[-1].detail

    _, c2 = controller(lateral_stop_distance=0.10)
    drive(c2, ahead=2.0, beside=0.05)
    beside_events = [e for e in c2.events if e.kind == "obstacle_stop"]
    assert beside_events and "alongside" in beside_events[-1].detail


@pytest.mark.parametrize("corridor_cm,passable", [(25, True), (30, True), (50, True)])
def test_the_arena_corridors_are_passable_with_a_lateral_limit(corridor_cm, passable):
    """The arena's 25-30 cm aisles, which is what this was built for."""
    _, c = controller(lateral_stop_distance=0.10)
    moved = drive(c, ahead=2.0, beside=corridor_cm / 200).linear_x > 0.0
    assert moved is passable
