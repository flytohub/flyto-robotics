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
- a planned navigation's path is left to Nav2, the inflation escape and the
  in-motion floor guard (``braking_envelope.leg_floor_check``); its
  destination is judged here (:func:`judge_destination`): the robot comes to
  rest there, so every LiDAR return must be at least the floor away from it.
  On 2026-10-09 (run c6d8d79d, real robot) a detour's goal leg was the blocked
  move's original destination, about 0.1 m in front of a person's shins, and
  nothing checked it.

Which of these applies is the capability contract's ``motion_kind``, never
the capability's name. Pure arithmetic: no ROS.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

from .inflation_escape import Beam, sweep_beams, to_robot

SHAPE_CORRIDOR = "corridor"
SHAPE_FOOTPRINT = "footprint"
SHAPE_PLANNED = "planned"

REASON_CLEAR = "clear"
REASON_BLOCKED = "path_blocked"
REASON_UNREADABLE = "path_unreadable"
REASON_NOT_JUDGED = "left_to_planner"
#: A navigation leg refused because a return lies inside the floor of the place
#: it would come to rest: arriving there would break the floor by itself.
REASON_DESTINATION_BLOCKED = "destination_blocked"


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


@dataclass(frozen=True)
class DestinationClearance:
    """Whether a navigation leg's destination keeps the floor, with what was measured."""

    leg: str
    admitted: bool
    reason: str
    floor_m: float
    #: The destination in the robot frame at the time of the sweep (x ahead,
    #: y left); None when the robot's map pose could not be read.
    destination_robot: tuple[float, float] | None
    #: Distance from the destination to the nearest LiDAR return; None: no return.
    clearance_m: float | None = None
    #: That return, as the LiDAR saw it.
    limiting_range_m: float | None = None
    limiting_bearing_rad: float | None = None
    #: How many returns lie inside the floor of the destination.
    returns_inside_floor: int = 0

    def to_dict(self) -> dict[str, Any]:
        where = self.destination_robot
        return {
            "leg": self.leg,
            "admitted": self.admitted,
            "reason": self.reason,
            "floor_m": round(self.floor_m, 4),
            "destination_robot_frame": (
                None if where is None else {"x": round(where[0], 4), "y": round(where[1], 4)}
            ),
            "clearance_m": _round(self.clearance_m),
            "limiting_range_m": _round(self.limiting_range_m),
            "limiting_bearing_rad": _round(self.limiting_bearing_rad),
            "limiting_side": (
                None if self.limiting_bearing_rad is None else side_of(self.limiting_bearing_rad)
            ),
            "returns_inside_floor": self.returns_inside_floor,
        }

    def evidence(self) -> dict[str, Any]:
        return {"reason_code": self.reason, "destination_clearance": self.to_dict()}

    def describe(self) -> str:
        if self.reason == REASON_UNREADABLE:
            return (
                f"the {self.leg} leg's destination cannot be judged: no readable LiDAR "
                "sweep and map pose; not knowing is not room"
            )
        if self.admitted:
            return f"{self.leg} leg destination clear"
        return (
            f"{self.leg} leg refused: {self.returns_inside_floor} LiDAR return(s) within "
            f"{self.clearance_m:.3f} m of its destination, inside the {self.floor_m:.3f} m "
            f"clearance floor (nearest seen at {self.limiting_range_m:.3f} m "
            f"{side_of(self.limiting_bearing_rad or 0.0)})"
        )


def judge_destination(
    sweep: Mapping[str, Any] | None,
    robot_map_pose: Mapping[str, Any] | None,
    destination_xy: tuple[float, float],
    *,
    floor_m: float,
    leg: str,
) -> DestinationClearance:
    """Whether a navigation leg may be sent to ``destination_xy`` (map frame).

    Each LiDAR return is placed in the robot frame and its distance to the
    destination measured; one closer than the floor refuses the leg. Only
    returns the LiDAR has are judged (a bin with no return says nothing about
    the destination); a sweep or pose that cannot be read refuses, because
    the destination cannot be judged without them. The floor is the caller's
    and is applied as it is: nothing here lowers it.
    """
    beams = sweep_beams(sweep)
    readable = [beam for beam in beams if beam.range_m is not None]
    try:
        destination = to_robot(robot_map_pose or {}, destination_xy)
    except (KeyError, TypeError, ValueError):
        destination = None
    if not readable or destination is None:
        return DestinationClearance(
            leg=leg,
            admitted=False,
            reason=REASON_UNREADABLE,
            floor_m=floor_m,
            destination_robot=destination,
        )
    dx, dy = destination
    measured = [
        (math.hypot(beam.range_m * math.cos(beam.bearing) - dx,
                    beam.range_m * math.sin(beam.bearing) - dy), beam)
        for beam in readable
        if beam.range_m is not None
    ]
    inside = sum(1 for distance, _ in measured if distance < floor_m)
    distance, beam = min(measured, key=lambda item: item[0])
    blocked = distance < floor_m
    return DestinationClearance(
        leg=leg,
        admitted=not blocked,
        reason=REASON_DESTINATION_BLOCKED if blocked else REASON_CLEAR,
        floor_m=floor_m,
        destination_robot=destination,
        clearance_m=distance,
        limiting_range_m=beam.range_m,
        limiting_bearing_rad=beam.bearing,
        returns_inside_floor=inside,
    )
