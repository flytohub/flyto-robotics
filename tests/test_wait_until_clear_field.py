"""The wait gate reads a sectored sweep instead of crashing on it.

``tick`` has accepted ``float | RangeField`` since sectors arrived, and every
motion primitive was taught to read the field. ``wait_until_clear`` was not. It
kept the old signature and called :func:`math.isfinite` on whatever it was
handed, so a sectored sweep reaching the one primitive whose entire job is to
stand still and be careful raised ``TypeError`` and took the control loop down.

Fixing the crash is not the same as fixing the reading, so both are pinned here:
the gate must not collapse the field to ``closest`` (a side wall at an ordinary
corridor distance would hold it shut while the path ahead was open), it must
still refuse when the sector it depends on was never measured, and the emergency
floor must still watch every direction.
"""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from flyto_robotics.contracts import load_job
from flyto_robotics.mission import MissionController, Pose2D, RangeField, sector_field
from flyto_robotics.workflow import (
    MissionState,
    PrimitiveKind,
    WorkflowPlan,
    WorkflowStep,
)

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples" / "jobs" / "tb3-lab-shortcut.json"

# From the job: stop at 0.25 m, emergency floor at the contract default 0.08 m.
STOP_DISTANCE = 0.25
EMERGENCY_DISTANCE = 0.08


def controller(**safety: object) -> MissionController:
    """A mission whose first and only step is the clearance gate."""
    job = load_job(JOB)
    if safety:
        job = job.__class__(
            **{
                **job.__dict__,
                "safety": job.safety.__class__(**{**job.safety.__dict__, **safety}),
            }
        )
    assert job.safety.obstacle_stop_distance == STOP_DISTANCE
    assert job.safety.emergency_stop_distance == EMERGENCY_DISTANCE
    plan = WorkflowPlan(
        workflow_id="wait_until_clear.field.v1",
        steps=(
            WorkflowStep(
                step_id="delivery.wait_until_clear",
                kind=PrimitiveKind.WAIT_UNTIL_CLEAR,
                active_state=MissionState.WAITING_FOR_CLEAR,
                arguments=(("clear_seconds", 0.5),),
            ),
        ),
    )
    return MissionController(job, workflow=plan)


def gate(c: MissionController, field: float | RangeField, now: float = 0.0):
    return c.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=field, now=now)


def aisle(wall_m: float, *, ahead: float = 2.0) -> RangeField:
    """Walls to each side, open ahead -- the shape that used to jam the gate."""
    return RangeField(
        forward=ahead,
        left=wall_m,
        right=wall_m,
        rear=2.0,
        closest=min(wall_m, ahead),
        directional=True,
    )


# -- the crash -----------------------------------------------------------


def test_the_exact_crash_a_sectored_sweep_used_to_cause() -> None:
    """``math.isfinite(RangeField)`` is a ``TypeError``, and that is what the
    gate did with the field ``tick`` is documented to accept."""
    field = aisle(0.14)
    with pytest.raises(TypeError, match="must be real number, not RangeField"):
        math.isfinite(field)  # type: ignore[arg-type]


def test_the_wait_gate_accepts_the_field_tick_accepts() -> None:
    c = controller(lateral_stop_distance=0.10)
    command = gate(c, aisle(0.14))
    assert command.reason in {"verifying_clearance", "waiting_for_clearance"}


# -- reading the field, rather than collapsing it ------------------------


def test_a_side_wall_does_not_hold_the_gate_shut_on_a_clear_path_ahead() -> None:
    """A 28 cm aisle puts each wall 0.14 m from a centred lidar. That is nearer
    than the 0.25 m stop distance and it is not in the way: the gate releases
    forward travel, and forward is open. Collapsing to ``closest`` waited here
    until the mission timed out."""
    c = controller(lateral_stop_distance=0.10)
    field = aisle(0.14)
    assert field.closest < STOP_DISTANCE  # the value the old gate read
    assert field.forward > STOP_DISTANCE  # the value that decides

    assert gate(c, field).reason == "verifying_clearance"
    assert [event.kind for event in c.events].count("clearance_blocked") == 0
    assert "clearance_window_started" in [event.kind for event in c.events]


def test_an_obstacle_ahead_still_holds_the_gate_shut() -> None:
    c = controller(lateral_stop_distance=0.10)
    field = RangeField(
        forward=0.20, left=2.0, right=2.0, rear=2.0, closest=0.20, directional=True
    )
    assert gate(c, field).reason == "waiting_for_clearance"
    assert c.events[-1].kind == "clearance_blocked"
    assert "ahead" in c.events[-1].detail


def test_the_gate_completes_once_the_forward_path_stays_clear() -> None:
    c = controller(lateral_stop_distance=0.10)
    gate(c, aisle(0.14), now=0.0)
    gate(c, aisle(0.14), now=0.6)
    assert "primitive_completed" in [event.kind for event in c.events]


# -- what survives the relaxation ---------------------------------------


def test_the_emergency_floor_still_watches_every_direction() -> None:
    """Relaxing the sides is not permission to graze one. Nothing may be inside
    the emergency distance in any direction, whatever the path ahead says."""
    c = controller(lateral_stop_distance=0.02)
    field = aisle(EMERGENCY_DISTANCE - 0.01)
    assert field.forward > STOP_DISTANCE

    assert gate(c, field).reason == "waiting_for_clearance"
    assert c.events[-1].kind == "clearance_blocked"
    assert "emergency" in c.events[-1].detail


def test_a_forward_sector_that_could_not_be_measured_fails_closed() -> None:
    """Every beam NaN: the sweep covered the forward sector and read nothing
    from it. A gate that cannot see ahead has observed nothing clear, so it
    must not report one."""
    c = controller()
    field = sector_field(
        [math.nan] * 360, angle_min=0.0, angle_increment=math.radians(1.0)
    )
    assert "forward" in field.blind
    assert not math.isfinite(field.closest)  # nothing to compare a threshold to

    assert gate(c, field).reason == "waiting_for_clearance"
    assert c.events[-1].kind == "clearance_blocked"
    assert "cannot measure forward" in c.events[-1].detail


# -- the pre-sector callers are untouched --------------------------------


def test_a_scalar_below_the_stop_distance_still_blocks() -> None:
    c = controller()
    assert gate(c, 0.14).reason == "waiting_for_clearance"
    assert c.events[-1].kind == "clearance_blocked"
    assert c.events[-1].detail == (
        "wait gate observed range below configured stop distance"
    )


def test_a_scalar_above_the_stop_distance_still_starts_the_window() -> None:
    c = controller()
    assert gate(c, 2.0).reason == "verifying_clearance"


def test_an_infinite_scalar_is_still_treated_as_clear() -> None:
    """Unchanged behaviour, kept explicit: the old gate's ``math.isfinite``
    check let infinity through, and callers depend on that."""
    c = controller()
    assert gate(c, math.inf).reason == "verifying_clearance"
