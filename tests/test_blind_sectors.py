"""Refusing to drive on a sector the lidar could not measure.

Infinity in a RangeField used to mean two opposite things at once: "measured,
and nothing is out there" and "no usable beam came back". Every threshold in
the obstacle guard is written `math.isfinite(x) and x < limit`, so the second
meaning skipped every check and the robot drove.

Four sweeps that used to be indistinguishable from an open corridor:

    stalled rotor          every beam 0.0
    covered sensor         every beam below range_min
    wall inside range_min  every beam below range_min
    dead driver            every beam NaN

and two that legitimately are clear and must stay that way:

    open space             every beam past range_max
    no echo                every beam +inf, which is how most drivers spell it
"""

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
    sector_field,
)

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples" / "jobs" / "tb3-lab-shortcut.json"
FORWARD_PLAN = ROOT / "examples" / "plans" / "shortcut-forward-40cm.json"

# An LDS-01/03 as the lab robot reports it.
SWEEP = dict(angle_min=0.0, angle_increment=math.radians(1.0), range_min=0.12, range_max=3.5)
BEAMS = 360


def sweep(value: float) -> RangeField:
    return sector_field([value] * BEAMS, **SWEEP)


class TestBeamsThatAreNotAnAnswer:
    @pytest.mark.parametrize(
        ("name", "value"),
        [
            ("stalled rotor", 0.0),
            ("covered sensor", 0.05),
            ("wall inside range_min", 0.10),
            ("dead driver", math.nan),
            ("negative garbage", -1.0),
        ],
    )
    def test_a_sweep_with_no_usable_beam_is_blind_everywhere(self, name, value):
        field = sweep(value)
        assert field.blind == frozenset({"forward", "left", "right", "rear"}), name
        assert field.forward == math.inf, "the range is still infinity"

    def test_infinity_alone_no_longer_decides_anything(self):
        """The whole point: same forward range, opposite meaning."""
        blind, clear = sweep(0.0), sweep(10.0)
        assert blind.forward == clear.forward == math.inf
        assert blind.blind and not clear.blind


class TestBeamsThatAreAnAnswer:
    @pytest.mark.parametrize(
        ("name", "value"),
        [
            ("open space past range_max", 10.0),
            ("no echo, spelled +inf", math.inf),
            ("exactly range_max", 3.5),
            ("exactly range_min", 0.12),
        ],
    )
    def test_definite_readings_are_not_blind(self, name, value):
        assert sweep(value).blind == frozenset(), name

    def test_one_usable_beam_redeems_its_sector(self):
        """Partial degradation still carries information; do not over-refuse."""
        ranges = [0.0] * BEAMS
        ranges[0] = 1.5  # straight ahead
        field = sector_field(ranges, **SWEEP)
        assert "forward" not in field.blind
        assert field.forward == pytest.approx(1.5)
        assert {"left", "right", "rear"} <= field.blind

    def test_a_hand_built_field_is_never_blind(self):
        """Callers that supply readings directly keep their old behaviour."""
        assert RangeField.omnidirectional(0.4).blind == frozenset()
        assert RangeField(forward=1.0, directional=True).blind == frozenset()


class TestWhichSectorsEachMotionNeeds:
    """blind_for must consult exactly what blocking consults, or a motion is
    guarded by a range nobody checked."""

    @pytest.mark.parametrize(
        ("intent", "blind", "expected"),
        [
            (INTENT_FORWARD, {"forward"}, "forward"),
            (INTENT_FORWARD, {"rear"}, None),
            (INTENT_REVERSE, {"rear"}, "rear"),
            (INTENT_REVERSE, {"forward"}, None),
            (INTENT_ROTATE, {"left"}, "left"),
            (INTENT_ROTATE, {"rear"}, "rear"),
            (INTENT_ROTATE, set(), None),
        ],
    )
    def test_blind_for_mirrors_blocking(self, intent, blind, expected):
        field = RangeField(directional=True, blind=frozenset(blind))
        assert field.blind_for(intent) == expected

    def test_rotation_needs_every_side(self):
        """Turning sweeps the whole footprint, so any unread side blocks it."""
        for sector in ("forward", "left", "right", "rear"):
            field = RangeField(directional=True, blind=frozenset({sector}))
            assert field.blind_for(INTENT_ROTATE) == sector


class TestTheControllerRefuses:
    """End to end: the defect was that the robot moved, so assert it does not."""

    def controller(self):
        job = load_job(JOB)
        return MissionController(job, workflow=compile_workflow(load_plan(FORWARD_PLAN)))

    def drive(self, field):
        control = self.controller()
        control.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=field, now=0.0)
        return control, control.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=field, now=0.3)

    def test_a_blind_sweep_stops_the_robot(self):
        _, command = self.drive(sweep(0.0))
        assert command.linear_x == 0.0
        assert command.angular_z == 0.0

    def test_a_blind_sweep_is_recorded_as_a_safety_stop(self):
        """Not merely stopped — the evidence must say why.

        A blind run that reported succeeded with safety_stop_count 0 was
        byte-identical to a clean run down an empty corridor.
        """
        control, _ = self.drive(sweep(0.0))
        assert control.safety_stop_count >= 1

    def test_the_reason_names_the_unread_sector(self):
        control, _ = self.drive(sweep(0.0))
        reasons = " ".join(
            event.detail for event in control.events if "obstacle" in event.kind
        )
        assert "cannot measure" in reasons
        assert "forward" in reasons

    def test_a_genuinely_open_sweep_still_drives(self):
        """The refusal must not cost the robot an empty corridor."""
        _, command = self.drive(sweep(10.0))
        assert command.linear_x > 0.0

    def test_a_wall_inside_the_sensor_floor_no_longer_reads_as_open(self):
        """0.10 m is under an LDS-01's 0.12 m floor, so it was invisible.

        This is the case that scared me most: an obstacle close enough to be
        unmeasurable was safer to the guard than one at 0.30 m.
        """
        _, command = self.drive(sweep(0.10))
        assert command.linear_x == 0.0
