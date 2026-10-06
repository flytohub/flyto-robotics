"""Whether a navigation starts inside an obstacle's inflation, and the way out.

Nav2's costmap inflates every obstacle by ``inflation_radius``. A robot whose
centre starts closer to an obstacle than that radius plus its own radius sits
in the cost gradient around it, and the stock controller can fail to make any
progress from there: it reports "Failed to make progress", the behaviour
server cycles spin/wait/backup, and the goal is aborted minutes later.

This module decides that from facts and computes a way out, with no ROS
import so every step is unit-testable:

- :func:`read_costmap_geometry` reads the inflation radius and the robot's
  radius (or footprint) from the costmaps' own parameters, through a reader
  the transport supplies, and falls back to Nav2's documented defaults.
- :func:`plan_escape` reads one LiDAR sweep in the robot frame (x forward,
  y left, bearings counter-clockwise from straight ahead) and returns an
  :class:`EscapeDecision`: not pinned, pinned with an escape (a straight
  back-off and one lateral waypoint), or pinned with no safe escape.

Every straight segment the escape drives keeps at least the clearance floor
from every LiDAR return, and a sector the scan cannot see is never treated as
room. The floor is the caller's (0.35 m by default); nothing here lowers it.
The LiDAR is taken to sit at the robot's centre, facing forward.
"""

from __future__ import annotations

import ast
import math
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any

#: Nav2's documented costmap defaults, used when a parameter cannot be read.
DEFAULT_INFLATION_RADIUS_M = 0.55
DEFAULT_ROBOT_RADIUS_M = 0.1
#: Nav2's SimpleGoalChecker default ``xy_goal_tolerance``: a goal this close
#: to the robot counts as reached, used when the controller cannot be asked.
DEFAULT_ARRIVAL_TOLERANCE_M = 0.25
#: Nav2's SimpleGoalChecker default ``yaw_goal_tolerance``: a goal whose
#: heading is this close counts as reached, used when it cannot be read.
DEFAULT_HEADING_TOLERANCE_RAD = 0.25
#: A radius outside this band is a misread, not a robot.
MAX_PLAUSIBLE_RADIUS_M = 5.0

#: Sector half-width either side of its centre. Four sectors partition the sweep.
SECTOR_HALF_WIDTH_RAD = math.radians(45.0)
SECTOR_CENTRES_RAD: Mapping[str, float] = {
    "front": 0.0,
    "left": math.pi / 2,
    "rear": math.pi,
    "right": -math.pi / 2,
}
#: A sector with fewer readable bins than this fraction is unreadable: a
#: missing return may be open space or a covered sensor, and it is not room.
MIN_READABLE_FRACTION = 0.5
#: Neighbouring returns further apart than this belong to different objects.
CLUSTER_JUMP_M = 0.15

DECISION_CLEAR = "clear"
DECISION_ESCAPE = "escape"
REASON_NO_ESCAPE_ROOM = "no_escape_room"

ParameterReader = Callable[[str, str], "Mapping[str, Any] | None"]


# -- costmap parameters ------------------------------------------------------


@dataclass(frozen=True)
class CostmapGeometry:
    """The two radii that decide whether a start is inside inflation."""

    inflation_radius_m: float
    robot_radius_m: float
    source: str = "fallback"
    notes: tuple[str, ...] = ()

    @property
    def pinned_threshold_m(self) -> float:
        return self.inflation_radius_m + self.robot_radius_m

    def to_dict(self) -> dict[str, Any]:
        return {
            "inflation_radius_m": round(self.inflation_radius_m, 4),
            "robot_radius_m": round(self.robot_radius_m, 4),
            "source": self.source,
            **({"notes": list(self.notes)} if self.notes else {}),
        }


FALLBACK_GEOMETRY = CostmapGeometry(DEFAULT_INFLATION_RADIUS_M, DEFAULT_ROBOT_RADIUS_M)


def _plausible(value: float | None) -> bool:
    return value is not None and math.isfinite(value) and 0.0 < value <= MAX_PLAUSIBLE_RADIUS_M


def parameter_number(value: Mapping[str, Any] | None) -> float | None:
    """A ``rcl_interfaces/ParameterValue`` that holds an integer or a double."""
    if not isinstance(value, Mapping):
        return None
    kind = value.get("type")
    names = {2: "integer_value", 3: "double_value"}
    raw = value.get(names[kind]) if kind in names else None
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        return None
    return float(raw)


def parameter_text(value: Mapping[str, Any] | None) -> str | None:
    if not isinstance(value, Mapping) or value.get("type") != 4:
        return None
    raw = value.get("string_value")
    return raw if isinstance(raw, str) else None


def parameter_bool(value: Mapping[str, Any] | None) -> bool | None:
    if not isinstance(value, Mapping) or value.get("type") != 1:
        return None
    raw = value.get("bool_value")
    return raw if isinstance(raw, bool) else None


def parameter_strings(value: Mapping[str, Any] | None) -> tuple[str, ...]:
    if not isinstance(value, Mapping) or value.get("type") != 9:
        return ()
    raw = value.get("string_array_value")
    if not isinstance(raw, (list, tuple)):
        return ()
    return tuple(item for item in raw if isinstance(item, str) and item)


def parse_footprint(text: str | None) -> tuple[tuple[float, float], ...] | None:
    """A Nav2 ``footprint`` string (``"[[x, y], ...]"``) as points; None if empty or bad."""
    if not text or not text.strip():
        return None
    try:
        parsed = ast.literal_eval(text.strip())
    except (ValueError, SyntaxError, MemoryError, RecursionError):
        return None
    if not isinstance(parsed, (list, tuple)) or len(parsed) < 3:
        return None
    points: list[tuple[float, float]] = []
    for point in parsed:
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            return None
        x, y = point
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in (x, y)):
            return None
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
        points.append((float(x), float(y)))
    return tuple(points)


def footprint_radius(points: Sequence[tuple[float, float]]) -> float:
    """The circumscribed radius: the conservative radius for a polygon footprint."""
    return max(math.hypot(x, y) for x, y in points)


def read_costmap_geometry(
    reader: ParameterReader | None, nodes: Iterable[str]
) -> CostmapGeometry:
    """Inflation and robot radius from the live costmaps, else Nav2's defaults.

    ``reader(node, name)`` returns the parameter's ``ParameterValue`` as a
    mapping, None when the node does not have it, and raises when the node
    cannot be asked. Every costmap is read and the largest radius of each kind
    is kept: the robot is pinned if any costmap it plans on says so. A
    polygon footprint takes precedence over ``robot_radius``, as in Nav2.
    """
    if reader is None:
        return CostmapGeometry(
            DEFAULT_INFLATION_RADIUS_M,
            DEFAULT_ROBOT_RADIUS_M,
            notes=("no parameter reader on this transport",),
        )
    inflations: list[float] = []
    radii: list[float] = []
    notes: list[str] = []
    for node in nodes:
        try:
            for plugin in parameter_strings(reader(node, "plugins")):
                kind = parameter_text(reader(node, f"{plugin}.plugin")) or ""
                if not kind.endswith("InflationLayer"):
                    continue
                if parameter_bool(reader(node, f"{plugin}.enabled")) is False:
                    continue
                value = parameter_number(reader(node, f"{plugin}.inflation_radius"))
                if _plausible(value):
                    inflations.append(float(value))  # type: ignore[arg-type]
            footprint = parse_footprint(parameter_text(reader(node, "footprint")))
            if footprint is not None and _plausible(footprint_radius(footprint)):
                radii.append(footprint_radius(footprint))
            else:
                value = parameter_number(reader(node, "robot_radius"))
                if _plausible(value):
                    radii.append(float(value))  # type: ignore[arg-type]
        except Exception as error:  # noqa: BLE001 - one unreadable node falls back
            notes.append(f"{node}: {type(error).__name__}")
    if not inflations:
        notes.append(f"inflation_radius fallback {DEFAULT_INFLATION_RADIUS_M}")
    if not radii:
        notes.append(f"robot_radius fallback {DEFAULT_ROBOT_RADIUS_M}")
    source = (
        "parameters" if inflations and radii else "partial" if inflations or radii else "fallback"
    )
    return CostmapGeometry(
        max(inflations) if inflations else DEFAULT_INFLATION_RADIUS_M,
        max(radii) if radii else DEFAULT_ROBOT_RADIUS_M,
        source=source,
        notes=tuple(notes),
    )


def _largest_goal_checker_value(
    reader: ParameterReader | None,
    nodes: Iterable[str],
    key: str,
    plausible: Callable[[float | None], bool],
) -> float | None:
    """The largest ``<plugin>.<key>`` over every goal checker the controllers load."""
    if reader is None:
        return None
    found: list[float] = []
    for node in nodes:
        try:
            for plugin in parameter_strings(reader(node, "goal_checker_plugins")):
                value = parameter_number(reader(node, f"{plugin}.{key}"))
                if plausible(value):
                    found.append(float(value))  # type: ignore[arg-type]
        except Exception:  # noqa: BLE001 - one unreadable node falls back
            continue
    return max(found) if found else None


def read_arrival_tolerance(
    reader: ParameterReader | None, nodes: Iterable[str]
) -> tuple[float, str]:
    """How close Nav2 counts a goal as reached, from the controllers' goal checkers.

    Every goal checker the controller loads (``goal_checker_plugins``) is read
    and the largest ``xy_goal_tolerance`` is kept: the leg is judged by
    whichever is loosest. Falls back to Nav2's default when none can be read.
    Returns the tolerance and where it came from (``parameters``/``fallback``).
    """
    value = _largest_goal_checker_value(reader, nodes, "xy_goal_tolerance", _plausible)
    if value is None:
        return DEFAULT_ARRIVAL_TOLERANCE_M, "fallback"
    return value, "parameters"


def _plausible_heading(value: float | None) -> bool:
    return value is not None and math.isfinite(value) and 0.0 <= value < math.pi / 2


def read_heading_tolerance(
    reader: ParameterReader | None, nodes: Iterable[str]
) -> tuple[float, str]:
    """How far from a goal's heading Nav2 still counts it reached (radians).

    The largest ``yaw_goal_tolerance`` of the controllers' goal checkers: a leg
    can end facing anywhere within it of the heading it was sent, so the
    loosest checker is the one that bounds where the robot may be facing.
    A tolerance of a quarter turn or more is not a heading constraint and is
    not believed; Nav2's default is used when none can be read.
    """
    value = _largest_goal_checker_value(reader, nodes, "yaw_goal_tolerance", _plausible_heading)
    if value is None:
        return DEFAULT_HEADING_TOLERANCE_RAD, "fallback"
    return value, "parameters"


# -- scan geometry -----------------------------------------------------------


@dataclass(frozen=True)
class Beam:
    """One sweep bin: its bearing in the robot frame and its range, or None."""

    bearing: float
    range_m: float | None
    #: False for a bin the LiDAR could not read that is taken as part of an
    #: obstacle beside it (see :func:`obstacle_cluster`); its range is assumed.
    seen: bool = True

    def point(self) -> tuple[float, float] | None:
        if self.range_m is None:
            return None
        return (self.range_m * math.cos(self.bearing), self.range_m * math.sin(self.bearing))


def sweep_beams(sweep: Mapping[str, Any] | None) -> tuple[Beam, ...]:
    """The observation's reduced sweep as beams; empty when it has no geometry."""
    if not isinstance(sweep, Mapping):
        return ()
    try:
        start = float(sweep["angle_min_rad"])
        step = float(sweep["angle_increment_rad"])
        ranges = list(sweep["ranges_m"])
    except (KeyError, TypeError, ValueError):
        return ()
    if not (math.isfinite(start) and math.isfinite(step)) or step == 0.0:
        return ()
    beams: list[Beam] = []
    for index, raw in enumerate(ranges):
        bearing = math.remainder(start + step * index, math.tau)
        value: float | None = None
        if isinstance(raw, (int, float)) and not isinstance(raw, bool):
            value = float(raw) if math.isfinite(raw) and raw > 0.0 else None
        beams.append(Beam(bearing, value))
    return tuple(beams)


def _in_sector(beam: Beam, centre: float, half_width: float) -> bool:
    return abs(math.remainder(beam.bearing - centre, math.tau)) <= half_width + 1e-9


def sector_minimum(
    beams: Sequence[Beam],
    centre: float,
    half_width: float = SECTOR_HALF_WIDTH_RAD,
) -> float | None:
    """Nearest return in a sector, or None when the sector is unreadable."""
    inside = [beam for beam in beams if _in_sector(beam, centre, half_width)]
    readable = [beam.range_m for beam in inside if beam.range_m is not None]
    if not inside or len(readable) < MIN_READABLE_FRACTION * len(inside):
        return None
    return min(readable)  # type: ignore[type-var]


def sector_minima(beams: Sequence[Beam]) -> dict[str, float | None]:
    return {name: sector_minimum(beams, centre) for name, centre in SECTOR_CENTRES_RAD.items()}


def obstacle_cluster(beams: Sequence[Beam]) -> tuple[Beam, ...]:
    """The object nearest straight ahead: contiguous returns around it.

    Starts at the nearest return in the front sector and grows in both
    directions while neighbouring returns stay within ``CLUSTER_JUMP_M`` of
    each other and remain ahead of the robot (|bearing| < 90 degrees).

    A bin the LiDAR could not read is not room, and so not the object's edge:
    growth carries on past it, and every unread bin met on the way -- inside
    the object or just beyond its last return -- is kept as part of it, at the
    range of the last return before it (``seen=False``), as far as an arc of
    ``CLUSTER_JUMP_M`` from that return. The object can only come out wider
    for what the scan did not see, never narrower. (Twin 2026-10-06: one
    unread bin in a 0.40 m box's face cut its extent on one side to 0.02 m
    and 0.085 m, and the way round was placed 0.22 m and 0.29 m to the side
    instead of 0.40 m.)
    """
    count = len(beams)
    front = [
        index
        for index, beam in enumerate(beams)
        if beam.range_m is not None and _in_sector(beam, 0.0, SECTOR_HALF_WIDTH_RAD)
    ]
    if not front:
        return ()
    seed = min(front, key=lambda index: beams[index].range_m)  # type: ignore[arg-type,return-value]
    members: dict[int, Beam] = {seed: beams[seed]}
    for direction in (1, -1):
        previous = beams[seed]
        unread: list[int] = []
        index = seed
        for _ in range(count - 1):
            index = (index + direction) % count
            beam = beams[index]
            if index in members or abs(beam.bearing) >= math.pi / 2:
                break
            if beam.range_m is None:
                # Only within the continuity scale that joins two returns into
                # one object: a longer unread run is no more this object's
                # surface than a return that far away would be.
                arc = previous.range_m * abs(  # type: ignore[operator]
                    math.remainder(beam.bearing - previous.bearing, math.tau)
                )
                if arc > CLUSTER_JUMP_M:
                    break
                unread.append(index)
                continue
            if abs(beam.range_m - previous.range_m) > CLUSTER_JUMP_M:  # type: ignore[operator]
                break
            for gap in unread:
                members[gap] = Beam(beams[gap].bearing, previous.range_m, seen=False)
            unread = []
            members[index] = beam
            previous = beam
        for gap in unread:
            # Unread bins past the last return: the object may go on through them.
            members.setdefault(gap, Beam(beams[gap].bearing, previous.range_m, seen=False))
    return tuple(members[index] for index in sorted(members))


def segment_clearance(
    beams: Sequence[Beam], start: tuple[float, float], end: tuple[float, float]
) -> float | None:
    """Smallest distance from any LiDAR return to the segment start-end."""
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length_sq = dx * dx + dy * dy
    best: float | None = None
    for beam in beams:
        point = beam.point()
        if point is None:
            continue
        px, py = point
        t = 0.0
        if length_sq > 0.0:
            t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / length_sq))
        distance = math.hypot(px - (ax + t * dx), py - (ay + t * dy))
        best = distance if best is None else min(best, distance)
    return best


def backoff_distance(
    nearest_front_m: float, threshold_m: float, max_backoff_m: float
) -> float:
    """How far to back off so the front return is at the pinned threshold."""
    return max(0.0, min(max_backoff_m, threshold_m - nearest_front_m))


def allowed_backoff(
    wanted_m: float, rear_clearance_m: float | None, floor_m: float
) -> float:
    """The back-off the rear allows: rear clearance after it stays >= floor."""
    if rear_clearance_m is None or wanted_m <= 0.0:
        return 0.0
    return max(0.0, min(wanted_m, rear_clearance_m - floor_m))


def choose_sides(left_m: float | None, right_m: float | None) -> tuple[str, str]:
    """Both sides, the one with more free space first (an unreadable side has none)."""
    left = -1.0 if left_m is None else left_m
    right = -1.0 if right_m is None else right_m
    return ("left", "right") if left >= right else ("right", "left")


def _lateral_positions(cluster: Sequence[Beam]) -> list[float]:
    return [
        beam.range_m * math.sin(beam.bearing) for beam in cluster if beam.range_m is not None
    ]


def lateral_offset(
    cluster: Sequence[Beam], side: str, robot_radius_m: float, margin_m: float
) -> float:
    """Signed y (left positive) that clears the obstacle's extent on ``side``.

    For each obstacle return at (range, bearing) its lateral position is
    ``range * sin(bearing)``; the waypoint sits beyond the outermost one by
    the robot's radius plus the margin.
    """
    lateral = _lateral_positions(cluster)
    if not lateral:
        return 0.0
    if side == "left":
        return max(0.0, max(lateral)) + robot_radius_m + margin_m
    return min(0.0, min(lateral)) - robot_radius_m - margin_m


def clear_heading(
    cluster: Sequence[Beam],
    waypoint: tuple[float, float],
    side: str,
    clearance_m: float,
) -> float:
    """The heading, in the start's robot frame, that bounds a way past the obstacle.

    From the waypoint on ``side`` (left: y > 0), a heading on the open side of
    the returned one passes every obstacle return by at least ``clearance_m``
    (the tangent to a circle of that radius round each return), and never
    turns back across the start's line of travel: the LiDAR sees only the
    obstacle's near face, so its side may run on behind it, and a heading
    that crosses that line would meet it sooner or later. Left: the result is
    a lower bound (``>= 0``); right: an upper bound (``<= 0``).
    """
    wx, wy = waypoint
    bound = 0.0
    for beam in cluster:
        point = beam.point()
        if point is None:
            continue
        dx, dy = point[0] - wx, point[1] - wy
        distance = math.hypot(dx, dy)
        if distance <= 0.0:
            continue
        bearing = math.atan2(dy, dx)
        widen = math.asin(min(1.0, clearance_m / distance))
        bound = max(bound, bearing + widen) if side == "left" else min(bound, bearing - widen)
    return bound


# -- the decision ------------------------------------------------------------


@dataclass(frozen=True)
class EscapeDecision:
    """Whether the start is pinned, and if so how to leave or why it cannot."""

    pinned: bool
    feasible: bool
    reason: str
    threshold_m: float
    floor_m: float
    clearances: Mapping[str, float | None]
    geometry: CostmapGeometry
    backoff_m: float = 0.0
    backoff_limited_by: str | None = None
    side: str | None = None
    lateral_offset_m: float = 0.0
    arrival_tolerance_m: float = 0.0
    #: How far from a goal's heading Nav2 still counts it reached (radians).
    heading_tolerance_rad: float = 0.0
    #: Bound on the waypoint's heading in the start's robot frame: every
    #: heading on the open side of it points past the obstacle (see
    #: :func:`clear_heading`). Left escapes need ``>=``, right ``<=``.
    clear_heading_rad: float = 0.0
    obstacle: Mapping[str, Any] | None = None
    sides_tried: tuple[Mapping[str, Any], ...] = field(default_factory=tuple)

    @property
    def waypoint_robot(self) -> tuple[float, float]:
        """The lateral waypoint in the start's robot frame (x forward, y left)."""
        return (-self.backoff_m, self.lateral_offset_m)

    def to_dict(self) -> dict[str, Any]:
        def metres(value: float | None) -> float | None:
            return None if value is None else round(value, 3)

        data: dict[str, Any] = {
            "pinned": self.pinned,
            "decision": self.reason,
            "pinned_threshold_m": metres(self.threshold_m),
            "clearance_floor_m": metres(self.floor_m),
            "clearances_m": {key: metres(value) for key, value in self.clearances.items()},
            "costmap": self.geometry.to_dict(),
            "arrival_tolerance_m": metres(self.arrival_tolerance_m),
            "heading_tolerance_rad": round(self.heading_tolerance_rad, 4),
        }
        if self.pinned:
            data["backoff_m"] = metres(self.backoff_m)
            if self.backoff_limited_by:
                data["backoff_limited_by"] = self.backoff_limited_by
            if self.obstacle is not None:
                data["obstacle"] = dict(self.obstacle)
            if self.sides_tried:
                data["sides_tried"] = [dict(item) for item in self.sides_tried]
        if self.feasible and self.pinned:
            x, y = self.waypoint_robot
            data["side"] = self.side
            data["lateral_offset_m"] = metres(self.lateral_offset_m)
            data["waypoint_robot_frame"] = {"x": metres(x), "y": metres(y)}
            data["clear_heading_rad"] = round(self.clear_heading_rad, 4)
        return data

    def describe(self) -> str:
        """One line for an operator, led by the named reason."""
        c = self.clearances

        def show(name: str) -> str:
            value = c.get(name)
            return "unreadable" if value is None else f"{value:.2f} m"

        head = (
            f"front {show('front')} < {self.threshold_m:.2f} m "
            f"(inflation {self.geometry.inflation_radius_m:.2f} + "
            f"robot {self.geometry.robot_radius_m:.2f})"
        )
        rest = (
            f"rear {show('rear')}, left {show('left')}, right {show('right')}; "
            f"floor {self.floor_m:.2f} m"
        )
        return f"{self.reason}: {head}; {rest}"


def plan_escape(
    sweep: Mapping[str, Any] | None,
    geometry: CostmapGeometry,
    *,
    floor_m: float,
    max_backoff_m: float,
    margin_m: float,
    max_lateral_m: float,
    arrival_tolerance_m: float,
    heading_tolerance_rad: float,
) -> EscapeDecision | None:
    """Decide from one sweep; None when the sweep has no usable geometry.

    ``arrival_tolerance_m`` is how close Nav2 counts a goal as reached. The
    lateral waypoint is placed at least that far plus the margin from where
    the back-off ends, so Nav2 cannot report it reached before the robot has
    moved: a waypoint inside the tolerance is "reached" where the robot
    stands (twin 2026-10-06: a 0.22 m waypoint against a 0.25 m tolerance
    succeeded in 20 ms and the goal leg started from right behind the box).

    ``heading_tolerance_rad`` is how far from the waypoint's heading Nav2
    lets the leg end; the decision keeps it with the heading bound past the
    obstacle so :func:`waypoint_heading` can keep every accepted arrival
    heading clear of it.
    """
    beams = sweep_beams(sweep)
    if not beams:
        return None
    clearances = sector_minima(beams)
    threshold = geometry.pinned_threshold_m
    front = clearances["front"]
    base = {
        "threshold_m": threshold,
        "floor_m": floor_m,
        "clearances": clearances,
        "geometry": geometry,
        "arrival_tolerance_m": max(0.0, arrival_tolerance_m),
        "heading_tolerance_rad": max(0.0, heading_tolerance_rad),
    }
    if front is None or front >= threshold:
        # Nothing ahead inside inflation (an unreadable front is the
        # preflight's to refuse, not something an escape can judge).
        return EscapeDecision(pinned=False, feasible=True, reason=DECISION_CLEAR, **base)

    wanted = backoff_distance(front, threshold, max(0.0, max_backoff_m))
    backoff = allowed_backoff(wanted, clearances["rear"], floor_m)
    limited_by = None
    if backoff < wanted:
        limited_by = "rear_unreadable" if clearances["rear"] is None else "rear_clearance"
    if backoff > 0.0:
        swept = segment_clearance(beams, (0.0, 0.0), (-backoff, 0.0))
        if swept is None or swept < floor_m:
            backoff, limited_by = 0.0, "backoff_path"

    cluster = obstacle_cluster(beams)
    lateral = _lateral_positions(cluster)
    obstacle = {
        "returns": sum(1 for beam in cluster if beam.seen),
        "unread_bins": sum(1 for beam in cluster if not beam.seen),
        "bearing_min_rad": round(min(beam.bearing for beam in cluster), 4),
        "bearing_max_rad": round(max(beam.bearing for beam in cluster), 4),
        "lateral_min_m": round(min(lateral), 3),
        "lateral_max_m": round(max(lateral), 3),
    }
    tried: list[dict[str, Any]] = []
    start = (-backoff, 0.0)
    for side in choose_sides(clearances["left"], clearances["right"]):
        offset = lateral_offset(cluster, side, geometry.robot_radius_m, margin_m)
        leaves = max(0.0, arrival_tolerance_m) + margin_m
        if abs(offset) < leaves:
            offset = leaves if side == "left" else -leaves
        attempt: dict[str, Any] = {"side": side, "lateral_offset_m": round(offset, 3)}
        side_room = clearances[side]
        swept = segment_clearance(beams, start, (-backoff, offset))
        attempt["path_clearance_m"] = None if swept is None else round(swept, 3)
        if side_room is None:
            attempt["refused"] = "side_unreadable"
        elif abs(offset) > max_lateral_m:
            attempt["refused"] = "beyond_max_lateral"
        elif swept is None or swept < floor_m:
            attempt["refused"] = "path_below_floor"
        tried.append(attempt)
        if "refused" not in attempt:
            return EscapeDecision(
                pinned=True,
                feasible=True,
                reason=DECISION_ESCAPE,
                backoff_m=backoff,
                backoff_limited_by=limited_by,
                side=side,
                lateral_offset_m=offset,
                clear_heading_rad=clear_heading(
                    cluster, (-backoff, offset), side, geometry.robot_radius_m + margin_m
                ),
                obstacle=obstacle,
                sides_tried=tuple(tried),
                **base,
            )
    return EscapeDecision(
        pinned=True,
        feasible=False,
        reason=REASON_NO_ESCAPE_ROOM,
        backoff_m=backoff,
        backoff_limited_by=limited_by,
        obstacle=obstacle,
        sides_tried=tuple(tried),
        **base,
    )


def to_map(
    start_map_pose: Mapping[str, Any], robot_xy: tuple[float, float]
) -> tuple[float, float]:
    """A point in the start's robot frame, in the map frame."""
    x0 = float(start_map_pose["x"])
    y0 = float(start_map_pose["y"])
    yaw = float(start_map_pose["yaw"])
    rx, ry = robot_xy
    return (
        x0 + math.cos(yaw) * rx - math.sin(yaw) * ry,
        y0 + math.sin(yaw) * rx + math.cos(yaw) * ry,
    )


def to_robot(start_map_pose: Mapping[str, Any], map_xy: tuple[float, float]) -> tuple[float, float]:
    """A map-frame point in the start's robot frame (the inverse of :func:`to_map`)."""
    yaw = float(start_map_pose["yaw"])
    dx = map_xy[0] - float(start_map_pose["x"])
    dy = map_xy[1] - float(start_map_pose["y"])
    return (
        math.cos(yaw) * dx + math.sin(yaw) * dy,
        -math.sin(yaw) * dx + math.cos(yaw) * dy,
    )


def waypoint_heading(
    start_map_pose: Mapping[str, Any],
    decision: EscapeDecision,
    goal_xy: tuple[float, float],
) -> dict[str, Any]:
    """The heading to send with the lateral waypoint, and what decided it.

    Nav2 ends the waypoint leg anywhere within ``heading_tolerance_rad`` of
    the heading it was sent, and the goal leg starts from there. The heading
    toward the final goal runs past the obstacle's near corner, so the
    tolerance alone can leave the robot facing the obstacle: twin 2026-10-06,
    a waypoint sent at -0.278 rad ended at -0.523 (inside 0.25 rad), the goal
    leg then failed to make progress eight times and took 119 s, while every
    leg that started within 0.07 rad of the line of travel took 9-19 s.

    The heading is therefore the goal's bearing, turned toward the open side
    until the whole accepted band, heading +/- tolerance, lies on the open
    side of :attr:`EscapeDecision.clear_heading_rad`. Robot-frame values are
    radians from the start's line of travel, left positive.
    """
    gx, gy = to_robot(start_map_pose, goal_xy)
    wx, wy = decision.waypoint_robot
    toward_goal = math.atan2(gy - wy, gx - wx)
    tolerance = decision.heading_tolerance_rad
    if decision.side == "right":
        limit = decision.clear_heading_rad - tolerance
        heading = min(toward_goal, limit)
    else:
        limit = decision.clear_heading_rad + tolerance
        heading = max(toward_goal, limit)
    return {
        "heading_rad": round(heading, 4),
        "toward_goal_rad": round(toward_goal, 4),
        "clear_heading_rad": round(decision.clear_heading_rad, 4),
        "tolerance_rad": round(tolerance, 4),
        "limited_by": "goal" if heading == toward_goal else "obstacle",
        "frame": "start_robot",
    }


def waypoint_pose(
    start_map_pose: Mapping[str, Any],
    decision: EscapeDecision,
    goal_xy: tuple[float, float],
) -> dict[str, float]:
    """The lateral waypoint in the map frame, at :func:`waypoint_heading`."""
    x, y = to_map(start_map_pose, decision.waypoint_robot)
    heading = waypoint_heading(start_map_pose, decision, goal_xy)["heading_rad"]
    yaw = math.remainder(float(start_map_pose["yaw"]) + heading, math.tau)
    return {"x": round(x, 4), "y": round(y, 4), "yaw_radians": round(yaw, 4)}
