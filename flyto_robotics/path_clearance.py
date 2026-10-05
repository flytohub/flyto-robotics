"""Clearance before a motion starts, measured along the path it will sweep.

The 0.35 m floor is a distance between the LiDAR and the nearest thing the
robot would come closer to. Checked against the nearest return in every
direction, it refused a forward advance because of a chair 0.33 m behind the
robot (live, 2026-10-06: rear 0.334 m at about 168 degrees, front 1.445 m, 0 m
travelled). The floor is not lowered here; it is applied where the motion
goes, with the same frame-of-travel geometry as the in-motion braking guard
(``braking_envelope.room_to_floor``):

- a straight drive sweeps a corridor in its direction of travel: returns with
  ``x > 0`` and ``|y| < floor + robot radius`` in the frame of travel, and the
  nearest of them must be at least the floor away;
- a rotation in place sweeps the robot's footprint all around, so every
  return must be at least ``floor + rotation radius`` away (omnidirectional is
  correct there);
- a planned navigation is left to Nav2, the inflation escape and the
  in-motion guard; it is not judged on a static distance here.

Which of these applies is the capability contract's ``motion_kind``, never
the capability's name. Pure arithmetic: no ROS.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from .inflation_escape import Beam, sweep_beams

SHAPE_CORRIDOR = "corridor"
SHAPE_FOOTPRINT = "footprint"
SHAPE_PLANNED = "planned"

REASON_CLEAR = "clear"
REASON_BLOCKED = "path_blocked"
REASON_UNREADABLE = "path_unreadable"
REASON_NOT_JUDGED = "left_to_planner"


@dataclass(frozen=True)
class SweptPath:
    shape: str
    #: Direction of travel in the robot frame (radians, counter-clockwise).
    direction_rad: float = 0.0


#: Keyed by the contract's ``motion_kind`` (``adapter_contract``). A movement
#: without one is judged as before: the floor, all around.
SWEPT_PATHS: Mapping[str, SweptPath] = {
    "advance": SweptPath(SHAPE_CORRIDOR, 0.0),
    "retreat": SweptPath(SHAPE_CORRIDOR, math.pi),
    "rotate": SweptPath(SHAPE_FOOTPRINT),
    "planned": SweptPath(SHAPE_PLANNED),
}
UNKNOWN_KIND = SweptPath(SHAPE_FOOTPRINT)

#: Four 90 degree sectors in the robot frame, reported with every verdict.
SECTOR_CENTRES_RAD: Mapping[str, float] = {
    "ahead": 0.0,
    "left": math.pi / 2,
    "behind": math.pi,
    "right": -math.pi / 2,
}


def _round(value: float | None) -> float | None:
    return None if value is None else round(value, 4)


def side_of(bearing_rad: float) -> str:
    """The robot-frame sector a bearing falls in."""
    degrees = math.degrees(math.remainder(bearing_rad, math.tau))
    if abs(degrees) <= 45.0:
        return "ahead"
    if abs(degrees) >= 135.0:
        return "behind"
    return "left" if degrees > 0 else "right"


@dataclass(frozen=True)
class PathClearance:
    """The verdict on one motion's path, with what was measured."""

    motion_kind: str
    shape: str
    admitted: bool
    reason: str
    floor_m: float
    radius_m: float
    threshold_m: float
    #: Nearest return on the swept path; None when nothing is on it.
    clearance_m: float | None = None
    limiting_bearing_rad: float | None = None
    sectors: Mapping[str, float | None] = field(default_factory=dict)

    @property
    def limiting_side(self) -> str | None:
        if self.limiting_bearing_rad is None:
            return None
        return side_of(self.limiting_bearing_rad)

    def to_dict(self) -> dict[str, Any]:
        return {
            "motion_kind": self.motion_kind,
            "shape": self.shape,
            "admitted": self.admitted,
            "reason": self.reason,
            "floor_m": round(self.floor_m, 4),
            "radius_m": round(self.radius_m, 4),
            "threshold_m": round(self.threshold_m, 4),
            "clearance_m": _round(self.clearance_m),
            "limiting_side": self.limiting_side,
            "limiting_bearing_rad": _round(self.limiting_bearing_rad),
            # Nearest return per robot-frame sector; None: no return there.
            "sectors": {name: _round(value) for name, value in self.sectors.items()},
        }

    def evidence(self) -> dict[str, Any]:
        return {"reason_code": self.reason, "path_clearance": self.to_dict()}

    def describe(self) -> str:
        if self.reason == REASON_UNREADABLE:
            return (
                f"LiDAR cannot see {self.limiting_side or 'the path'} on the {self.motion_kind} "
                f"path; an unreadable sector is not room (motion safety minimum "
                f"{self.threshold_m:.3f}m)"
            )
        if self.admitted or self.clearance_m is None:
            return f"{self.motion_kind} path clear"
        basis = (
            f" (floor {self.floor_m:.3f}m + rotation radius {self.radius_m:.3f}m)"
            if self.shape == SHAPE_FOOTPRINT and self.radius_m
            else ""
        )
        return (
            f"LiDAR clearance {self.clearance_m:.3f}m {self.limiting_side} on the "
            f"{self.motion_kind} path is below the {self.threshold_m:.3f}m motion safety "
            f"minimum{basis}"
        )


def sector_clearances(beams: tuple[Beam, ...]) -> dict[str, float | None]:
    """Nearest return in each sector; None when the sector has no return at all."""
    nearest: dict[str, float | None] = dict.fromkeys(SECTOR_CENTRES_RAD)
    for beam in beams:
        if beam.range_m is None:
            continue
        side = side_of(beam.bearing)
        current = nearest[side]
        if current is None or beam.range_m < current:
            nearest[side] = beam.range_m
    return nearest


def _corridor(
    beams: tuple[Beam, ...], direction: float, half_width: float
) -> tuple[float, float] | None:
    """Nearest return in the corridor ahead of ``direction``: (range, bearing)."""
    best: tuple[float, float] | None = None
    for beam in beams:
        if beam.range_m is None:
            continue
        relative = math.remainder(beam.bearing - direction, math.tau)
        x = beam.range_m * math.cos(relative)
        y = beam.range_m * math.sin(relative)
        if x > 0.0 and abs(y) < half_width and (best is None or beam.range_m < best[0]):
            best = (beam.range_m, beam.bearing)
    return best


def _nearest(beams: tuple[Beam, ...]) -> tuple[float, float] | None:
    readable = [(beam.range_m, beam.bearing) for beam in beams if beam.range_m is not None]
    return min(readable) if readable else None  # type: ignore[type-var]


def judge(
    sweep: Mapping[str, Any] | None,
    *,
    motion_kind: str | None,
    floor_m: float,
    radius_m: float,
    minimum_range_m: float | None = None,
) -> PathClearance:
    """Whether the path ``motion_kind`` sweeps keeps the floor, and why.

    Without a readable sweep the nearest return in any direction
    (``minimum_range_m``) is all there is, and it is judged all around.
    """
    kind = motion_kind or "unspecified"
    path = SWEPT_PATHS.get(kind, UNKNOWN_KIND)
    radius = radius_m if path.shape == SHAPE_FOOTPRINT and kind in SWEPT_PATHS else 0.0
    threshold = floor_m + radius
    beams = sweep_beams(sweep)
    sectors = sector_clearances(beams) if beams else {}

    def verdict(
        reason: str,
        clearance: float | None = None,
        bearing: float | None = None,
        shape: str = path.shape,
    ) -> PathClearance:
        return PathClearance(
            motion_kind=kind,
            shape=shape,
            admitted=reason in (REASON_CLEAR, REASON_NOT_JUDGED),
            reason=reason,
            floor_m=floor_m,
            radius_m=radius,
            threshold_m=threshold,
            clearance_m=clearance,
            limiting_bearing_rad=bearing,
            sectors=sectors,
        )

    def measured(found: tuple[float, float] | None) -> PathClearance:
        if found is None:
            return verdict(REASON_CLEAR)
        reason = REASON_BLOCKED if found[0] < threshold else REASON_CLEAR
        return verdict(reason, found[0], found[1])

    if path.shape == SHAPE_PLANNED:
        return verdict(REASON_NOT_JUDGED)
    if not beams:
        if minimum_range_m is None:
            return verdict(REASON_UNREADABLE, shape=SHAPE_FOOTPRINT)
        minimum = float(minimum_range_m)
        reason = REASON_BLOCKED if minimum < threshold else REASON_CLEAR
        return verdict(reason, minimum, shape=SHAPE_FOOTPRINT)
    # A bin with no return is open space beyond the LiDAR's range as often as
    # it is anything else; like the in-motion guard (room_to_floor), the path
    # is judged on the returns it has. A sector without any reads None.
    if path.shape == SHAPE_CORRIDOR:
        return measured(_corridor(beams, path.direction_rad, floor_m + radius_m))
    return measured(_nearest(beams))
