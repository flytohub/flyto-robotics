"""Why a motion ended, decided without importing ROS.

A Nav2 action that does not succeed ends in one status, ``6`` (aborted), for
very different reasons: something stood in the way, the time allowance ran out,
localization was lost, or the server had no path. A planner reading only
"ROS 2 action status 6" cannot tell "go around the box" from "the robot is
broken", so every motion result carries a :func:`summarize` record with a
machine-readable ``reason`` and what the robot measured when it stopped: where
it started and ended, how far it went against how far it was asked to, and
the nearest LiDAR return.

Reasons, most specific first:

``completed``
    the action server reported success and, for a motion with a requested
    distance, the robot got there (see :data:`STOP_RULES`).
``path_blocked``
    a straight drive had no room to go any distance before the clearance
    floor: nothing the odometry could see was commanded.
``cancelled``
    the goal was cancelled (an operator stop, a halt, a cancel).
``obstacle_blocked``
    the action reported a collision ahead, the collision monitor stopped the
    base for one of its polygons, this adapter's braking guard stopped a
    straight drive because the room left to the clearance floor was inside
    its stopping distance, the drive was planned before sending to end short
    of the request for the same reason, or the nearest return the way the
    robot was going is inside the clearance floor at the stop.

Whether a motion with a requested distance stopped short because of what
stands ahead is decided first, by one table (:data:`STOP_RULES`) over the
motion's facts, never by the action status: Nav2 reports "succeeded" for a
drive that went the 2 cm it was sent when 1.2 m was asked.
``sensor_stale``
    the collision monitor stopped the base because its sensor data was late
    or missing ("invalid source"), or this adapter's braking guard stopped a
    straight drive because the LiDAR went quiet or unreadable; the robot was
    not blocked, it was blind.
``timeout``
    the action's time allowance, the controller, the planner, or this
    adapter's own deadline ran out.
``localization_error``
    a transform the server needed was unavailable.
``no_path``
    the planner found no path, or the start or goal is occupied or off the map.
``no_progress``
    the controller stopped making progress or found no valid command.
``aborted_by_server``
    the server aborted and said nothing more specific.
``unknown``
    any other terminal status.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any

from . import braking_envelope as braking
from . import sim_time
from .mission import relative_move_tolerance

REASON_COMPLETED = "completed"
REASON_PATH_BLOCKED = "path_blocked"
REASON_CANCELLED = "cancelled"
REASON_OBSTACLE_BLOCKED = "obstacle_blocked"
REASON_SENSOR_STALE = "sensor_stale"
REASON_TIMEOUT = "timeout"
REASON_LOCALIZATION_ERROR = "localization_error"
REASON_NO_PATH = "no_path"
REASON_NO_PROGRESS = "no_progress"
REASON_ABORTED_BY_SERVER = "aborted_by_server"
REASON_UNKNOWN = "unknown"

REASONS = (
    REASON_COMPLETED,
    REASON_PATH_BLOCKED,
    REASON_CANCELLED,
    REASON_OBSTACLE_BLOCKED,
    REASON_SENSOR_STALE,
    REASON_TIMEOUT,
    REASON_LOCALIZATION_ERROR,
    REASON_NO_PATH,
    REASON_NO_PROGRESS,
    REASON_ABORTED_BY_SERVER,
    REASON_UNKNOWN,
)

# action_msgs/msg/GoalStatus
STATUS_NAMES = {
    0: "unknown",
    1: "accepted",
    2: "executing",
    3: "canceling",
    4: "succeeded",
    5: "canceled",
    6: "aborted",
}
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

# nav2_msgs error codes (Jazzy). Each action owns a block of a hundred or
# ten; NavigateToPose passes on the code of the child action that failed.
_COLLISION_CODES = frozenset({703, 714, 723})  # Spin, BackUp, DriveOnHeading
_TIMEOUT_CODES = frozenset({701, 711, 721, 107, 207})
_TF_CODES = frozenset({702, 712, 722, 102, 202})
_NO_PATH_CODES = frozenset({103, 203, 204, 205, 206, 208})
_NO_PROGRESS_CODES = frozenset({104, 105, 106})

# nav2_msgs/msg/CollisionMonitorState
COLLISION_STOP = 1
COLLISION_SLOWDOWN = 2
COLLISION_APPROACH = 3
COLLISION_LIMIT = 4
# The polygon name the collision monitor reports when it stops the base
# because a source is late or missing rather than because of a polygon.
INVALID_SOURCE = "invalid source"

# Why the adapter's braking guard stopped a straight drive.
GUARD_TRIP_CLEARANCE = "clearance"  # room to the floor inside the stopping distance
GUARD_TRIP_BLIND = "blind"  # the sweep could not be read
GUARD_TRIP_STALE = "stale"  # no scan arrived in time
# A straight drive the guard did not stop, which ended where it was planned
# to before sending: short of the request, because the clearance measured then
# left no more room to stop at the floor (braking_envelope.plan_drive).
GUARD_END_PLANNED_STOP = "planned_stop"

# Half-width of the wedge read the way the robot was travelling.
TRAVEL_HALF_WIDTH_RAD = math.radians(20.0)
# Where that wedge is centred, relative to the robot's heading.
#: The base counts as standing still below these odometry speeds.
STILL_LINEAR_MPS = 0.01
STILL_ANGULAR_RADPS = 0.05
#: A still spell this long while a goal runs is recorded as a stall. Shorter
#: than Nav2's default progress window (10 s), so a stall is on record before
#: the controller gives up on it.
STALL_MIN_S = 2.0
#: Bounds on what one motion keeps.
MAX_STALLS = 16
MAX_RECOVERY_TIMES = 32


@dataclass
class _Commanded:
    """Velocity commands seen on one topic during a still spell."""

    samples: int = 0
    max_linear_mps: float = 0.0
    max_angular_radps: float = 0.0

    def saw(self, linear: float, angular: float) -> None:
        self.samples += 1
        self.max_linear_mps = max(self.max_linear_mps, abs(linear))
        self.max_angular_radps = max(self.max_angular_radps, abs(angular))

    def to_dict(self) -> dict[str, Any]:
        return {
            "samples": self.samples,
            "max_linear_mps": round(self.max_linear_mps, 4),
            "max_angular_radps": round(self.max_angular_radps, 4),
        }


_TRAVEL_DIRECTION = {
    "motion.advance": 0.0,
    "motion.retreat": math.pi,
}


@dataclass
class MotionTrack:
    """What one motion goal saw between its send and its end."""

    capability_id: str
    arguments: Mapping[str, float]
    start_pose: Mapping[str, Any] | None
    started_at: float
    minimum_range_during_m: float | None = None
    feedback: dict[str, Any] = field(default_factory=dict)
    # Collision monitor actions (type, polygon) reported while it ran.
    collision_events: list[tuple[int, str]] = field(default_factory=list)
    # What the collision monitor is doing to the base right now, including
    # "nothing" (type 0) once a stop clears. The monitor publishes only on a
    # change, so a track is seeded with the state in force when it began.
    collision_now: tuple[int, str] | None = None
    # The start pose in the map frame, when localization was up (additive;
    # ``start_pose`` stays the odometry pose the motion is judged on).
    start_map_pose: Mapping[str, Any] | None = None
    # The braking guard's record for a straight drive: the profile it used,
    # the speed it commanded and why it stopped the robot, if it did (see
    # braking_envelope). Filled in by the adapter when the track ends.
    braking: dict[str, Any] | None = None
    # Spells the base stood still while the goal ran, and what was commanded
    # meanwhile on each velocity topic watched: a stall with commands near
    # zero is the controller choosing to stand; one with real commands is
    # something between the controller and the wheels.
    stalls: list[dict[str, Any]] = field(default_factory=list)
    recoveries_at_s: list[float] = field(default_factory=list)
    # The simulator's clock during the motion (simulation only); None with
    # ``sim_time_note`` saying why there is no record.
    clock: sim_time.ClockRecord | None = None
    sim_time_note: dict[str, Any] | None = None
    _still_since: float | None = None
    _still_pose: Mapping[str, Any] | None = None
    _still_commands: dict[str, _Commanded] = field(default_factory=dict)

    def __post_init__(self) -> None:
        seeded = self.collision_now
        self.collision_now = None
        if seeded is not None:
            self.saw_collision_state(*seeded)

    def saw_range(self, minimum_range_m: float | None) -> None:
        if minimum_range_m is None or not math.isfinite(minimum_range_m):
            return
        if self.minimum_range_during_m is None or minimum_range_m < self.minimum_range_during_m:
            self.minimum_range_during_m = float(minimum_range_m)

    def saw_base(
        self,
        velocity: tuple[float, float] | None,
        at: float,
        pose: Mapping[str, Any] | None = None,
    ) -> None:
        """Odometry speed (linear, angular) at monotonic time ``at``."""
        if velocity is None:
            return
        linear, angular = velocity
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return
        if abs(linear) < STILL_LINEAR_MPS and abs(angular) < STILL_ANGULAR_RADPS:
            if self._still_since is None:
                self._still_since = at
                self._still_pose = dict(pose) if isinstance(pose, Mapping) else None
                self._still_commands = {}
            return
        self.close_still(at)

    def saw_clock(self, sim_s: float, wall_s: float) -> None:
        if self.clock is not None:
            self.clock.add(sim_s, wall_s)

    def sim_time_summary(self) -> dict[str, Any]:
        if self.clock is not None:
            return self.clock.summary()
        return dict(self.sim_time_note or sim_time.not_applicable())

    def saw_command(self, topic: str, linear: float, angular: float) -> None:
        """A velocity command on ``topic``; kept only while the base stands still."""
        if self._still_since is None:
            return
        if not (math.isfinite(linear) and math.isfinite(angular)):
            return
        if topic not in self._still_commands and len(self._still_commands) >= 4:
            return
        self._still_commands.setdefault(topic, _Commanded()).saw(linear, angular)

    def close_still(self, at: float) -> None:
        """End the current still spell at ``at``, recording it if it was a stall."""
        since = self._still_since
        if since is None:
            return
        self._still_since = None
        duration = at - since
        if duration < STALL_MIN_S or len(self.stalls) >= MAX_STALLS:
            return
        stall: dict[str, Any] = {
            "at_s": round(max(0.0, since - self.started_at), 3),
            "duration_s": round(duration, 3),
            "commanded": {
                topic: seen.to_dict() for topic, seen in sorted(self._still_commands.items())
            },
        }
        pose = _pose(self._still_pose)
        if pose is not None:
            stall["pose"] = pose
        self.stalls.append(stall)

    def saw_feedback(self, values: Mapping[str, Any] | None, at: float | None = None) -> None:
        if not isinstance(values, Mapping):
            return
        recoveries = values.get("number_of_recoveries")
        if (
            at is not None
            and isinstance(recoveries, (int, float))
            and math.isfinite(float(recoveries))
            and float(recoveries) > self.feedback.get("number_of_recoveries", 0.0)
            and len(self.recoveries_at_s) < MAX_RECOVERY_TIMES
        ):
            self.recoveries_at_s.append(round(max(0.0, at - self.started_at), 3))
        for key in ("distance_traveled", "angular_distance_traveled", "distance_remaining",
                    "number_of_recoveries"):
            value = values.get(key)
            if isinstance(value, (int, float)) and math.isfinite(float(value)):
                self.feedback[key] = float(value)

    def saw_collision_state(self, action_type: Any, polygon_name: Any) -> None:
        try:
            kind = int(action_type)
        except (TypeError, ValueError):
            return
        name = str(polygon_name or "")[:64]
        self.collision_now = (kind, name)
        if kind == 0:
            return
        if len(self.collision_events) < 32:
            self.collision_events.append((kind, name))


def travel_range(capability_id: str, sweep: Mapping[str, Any] | None) -> float | None:
    """Nearest return in the wedge the robot was driving into, or None.

    Only straight drives have a direction; a turn or a planned path does not,
    so they are judged on the whole sweep instead.
    """
    centre = _TRAVEL_DIRECTION.get(capability_id)
    if centre is None or not isinstance(sweep, Mapping):
        return None
    try:
        start = float(sweep["angle_min_rad"])
        step = float(sweep["angle_increment_rad"])
        ranges = list(sweep["ranges_m"])
    except (KeyError, TypeError, ValueError):
        return None
    nearest: float | None = None
    for index, value in enumerate(ranges):
        if value is None:
            continue
        angle = start + step * index
        if abs(math.remainder(angle - centre, math.tau)) > TRAVEL_HALF_WIDTH_RAD:
            continue
        beam = float(value)
        if math.isfinite(beam) and (nearest is None or beam < nearest):
            nearest = beam
    return nearest


def stop_clearance(
    capability_id: str,
    sweep: Mapping[str, Any] | None,
    minimum_range_m: float | None,
    floor_m: float,
) -> dict[str, Any] | None:
    """Where the nearest return was when the robot stopped, and whether the floor held.

    The nearest return is reported with its bearing from the way the robot
    was going (from its heading for a turn or a planned path), so a wall
    passed alongside is never read as one driven into. ``floor_held`` is the
    honest answer over every direction; ``travel_floor_held`` only over the
    path a straight drive would have continued along.
    """
    if minimum_range_m is None and not isinstance(sweep, Mapping):
        return None
    direction = _TRAVEL_DIRECTION.get(capability_id, 0.0)
    nearest = braking.nearest_return(sweep, direction)
    record: dict[str, Any] = {"floor_m": round(floor_m, 3)}
    overall = minimum_range_m
    if nearest is not None:
        beam, bearing = nearest
        record["nearest_bearing_rad"] = round(bearing, 4)
        record["nearest_direction"] = braking.direction_label(bearing)
        overall = beam if overall is None else min(overall, beam)
    if overall is not None:
        record["nearest_range_m"] = round(overall, 3)
        record["floor_held"] = overall >= floor_m
    if capability_id in _TRAVEL_DIRECTION:
        room = braking.room_to_floor(sweep, direction, floor_m)
        if room is not None:
            record["travel_room_to_floor_m"] = (
                round(room.room_m, 3) if math.isfinite(room.room_m) else None
            )
            record["travel_floor_held"] = room.room_m >= 0.0
    return record


def _pose(pose: Mapping[str, Any] | None) -> dict[str, float] | None:
    if not isinstance(pose, Mapping):
        return None
    try:
        return {
            "x": round(float(pose["x"]), 4),
            "y": round(float(pose["y"]), 4),
            "yaw": round(float(pose["yaw"]), 4),
        }
    except (KeyError, TypeError, ValueError):
        return None


def _map_pose(pose: Mapping[str, Any] | None) -> dict[str, Any] | None:
    """A map-frame pose as reported, or None for any other frame."""
    if not isinstance(pose, Mapping) or pose.get("frame") != "map":
        return None
    planar = _pose(pose)
    return {"frame": "map", **planar} if planar is not None else None


def _error_code(result_values: Mapping[str, Any] | None) -> int | None:
    if not isinstance(result_values, Mapping):
        return None
    try:
        code = int(result_values.get("error_code"))
    except (TypeError, ValueError):
        return None
    return code or None



# -- the stop rule -----------------------------------------------------------


@dataclass(frozen=True)
class StopFacts:
    """What a motion with a requested distance measured, for :data:`STOP_RULES`.

    ``requested_m`` is what the caller asked for, ``commanded_m`` what was sent
    after speed governance, ``travelled_m`` the odometry displacement from the
    start pose to the stop. ``shortened`` is the before-send plan's verdict,
    ``trip`` the braking guard's cause if it stopped the drive, and
    ``ahead_inside_floor`` whether the nearest return the way the robot was
    going lies inside the clearance floor at the stop.
    """

    requested_m: float
    commanded_m: float
    travelled_m: float
    shortened: bool
    trip: str | None
    ahead_inside_floor: bool

    @property
    def arrival_tolerance_m(self) -> float:
        # The relative-move arrival rule this repository already applies to a
        # straight move (mission.relative_move_tolerance): min(0.03 m, d/10).
        return relative_move_tolerance(self.requested_m)

    @property
    def shortfall_m(self) -> float:
        return self.requested_m - self.travelled_m

    @property
    def reached(self) -> bool:
        return self.shortfall_m <= self.arrival_tolerance_m

    @property
    def governed(self) -> bool:
        """The adapter, not the server, decided where this drive stopped."""
        return self.shortened or self.trip is not None


@dataclass(frozen=True)
class StopRule:
    name: str
    applies: Callable[[StopFacts], bool]
    reason: str
    basis: str


#: First match wins. Every row names the physical fact it reads and the
#: threshold's basis. A motion no row matches is left to :func:`_reason`'s
#: account of the server's own end (status, error code, collision monitor).
STOP_RULES: tuple[StopRule, ...] = (
    StopRule(
        "no_room_to_drive",
        lambda f: f.commanded_m < braking.MIN_DISTANCE_M <= f.requested_m,
        REASON_PATH_BLOCKED,
        "commanded distance below braking_envelope.MIN_DISTANCE_M (0.01 m, the "
        "shortest drive odometry can tell from standing still): the room ahead "
        "was used up by the stopping distance before the drive could start",
    ),
    StopRule(
        "reached_request",
        lambda f: f.governed and f.reached,
        REASON_COMPLETED,
        "travelled within mission.relative_move_tolerance (min(0.03 m, d/10)) of "
        "the requested distance: a planned or guarded stop that still arrived",
    ),
    StopRule(
        "guard_blind",
        lambda f: f.trip in (GUARD_TRIP_BLIND, GUARD_TRIP_STALE),
        REASON_SENSOR_STALE,
        "the braking guard stopped the drive because the LiDAR was unreadable "
        "or late: the robot was blind, not blocked",
    ),
    StopRule(
        "stopped_short_by_clearance",
        lambda f: not f.reached and (f.shortened or f.trip == GUARD_TRIP_CLEARANCE),
        REASON_OBSTACLE_BLOCKED,
        "short of the request by more than mission.relative_move_tolerance, and "
        "the clearance ahead was within the stopping budget (floor + v t + "
        "v^2/2a, braking_envelope) before sending (plan.shortened) or while "
        "driving (guard trip 'clearance')",
    ),
    StopRule(
        "stopped_short_facing_obstacle",
        lambda f: not f.reached and f.ahead_inside_floor,
        REASON_OBSTACLE_BLOCKED,
        "short of the request by more than mission.relative_move_tolerance, "
        "with the nearest return the way the robot was going inside the "
        "clearance floor at the stop (stop_clearance.travel_floor_held false)",
    ),
)


def stop_rule(facts: StopFacts | None) -> StopRule | None:
    """The first row of :data:`STOP_RULES` that holds for ``facts``."""
    if facts is None:
        return None
    return next((rule for rule in STOP_RULES if rule.applies(facts)), None)


def _number_or_none(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def stop_facts(
    track: MotionTrack,
    *,
    travelled_m: float | None,
    clearance: Mapping[str, Any] | None,
) -> StopFacts | None:
    """The facts of a motion with a requested distance; None for any other.

    The ask is the before-send plan's when there is one (the adapter sends the
    governed distance), else the call's own ``distance_m``. Nothing is assumed
    when odometry gave no displacement.
    """
    record = track.braking if isinstance(track.braking, Mapping) else {}
    plan = record.get("plan") if isinstance(record.get("plan"), Mapping) else {}
    commanded = _number_or_none(track.arguments.get("distance_m"))
    requested = _number_or_none(plan.get("requested_distance_m"))
    planned = _number_or_none(plan.get("commanded_distance_m"))
    if planned is not None:
        commanded = planned
    if requested is None:
        requested = commanded
    if requested is None or commanded is None or travelled_m is None:
        return None
    trip = record.get("tripped")
    clearance = clearance if isinstance(clearance, Mapping) else {}
    return StopFacts(
        requested_m=abs(requested),
        commanded_m=abs(commanded),
        travelled_m=abs(travelled_m),
        shortened=bool(plan.get("shortened")),
        trip=trip if isinstance(trip, str) else None,
        ahead_inside_floor=clearance.get("travel_floor_held") is False,
    )


def _blocks(kind: int, name: str) -> bool:
    return kind in (COLLISION_STOP, COLLISION_APPROACH) and name != INVALID_SOURCE


def _blind(kind: int, name: str) -> bool:
    return kind != 0 and name == INVALID_SOURCE


def _reason(
    *,
    status: int | None,
    error_code: int | None,
    collision_events: list[tuple[int, str]],
    collision_now: tuple[int, str] | None,
    stop_range: float | None,
    clearance_floor_m: float,
    guard_trip: str | None = None,
    rule: StopRule | None = None,
) -> str:
    """Most specific first.

    A motion whose facts match a :data:`STOP_RULES` row takes that row's
    reason, whatever status its server ended with. Otherwise what holds at
    the stop (Nav2's collision code, the monitor's state now, the range
    ahead) beats a specific Nav2 error, which beats an event the monitor
    raised and then cleared during the run: a slowdown for a box passed
    earlier must not turn "no path to the goal" into "blocked".
    """
    if rule is not None:
        return rule.reason
    if status == STATUS_SUCCEEDED:
        return REASON_COMPLETED
    # The adapter's own braking guard cancels the goal it stops, so the
    # server reports "canceled"; the guard's cause is the reason, not the
    # cancel it used to act on it.
    if guard_trip == GUARD_TRIP_CLEARANCE:
        return REASON_OBSTACLE_BLOCKED
    if guard_trip in (GUARD_TRIP_BLIND, GUARD_TRIP_STALE):
        return REASON_SENSOR_STALE
    if status == STATUS_CANCELED:
        return REASON_CANCELLED
    if error_code in _COLLISION_CODES:
        return REASON_OBSTACLE_BLOCKED
    if collision_now is not None and _blocks(*collision_now):
        return REASON_OBSTACLE_BLOCKED
    if collision_now is not None and _blind(*collision_now):
        return REASON_SENSOR_STALE
    if stop_range is not None and stop_range < clearance_floor_m:
        return REASON_OBSTACLE_BLOCKED
    if error_code in _TF_CODES:
        return REASON_LOCALIZATION_ERROR
    if error_code in _NO_PATH_CODES:
        return REASON_NO_PATH
    # A base the monitor held still makes no progress and runs out of time,
    # so an earlier stop explains those better than the code does.
    if any(_blocks(kind, name) for kind, name in collision_events):
        return REASON_OBSTACLE_BLOCKED
    if any(_blind(kind, name) for kind, name in collision_events):
        return REASON_SENSOR_STALE
    if status is None or error_code in _TIMEOUT_CODES:
        # No terminal status: this adapter's own deadline ran out first.
        return REASON_TIMEOUT
    if error_code in _NO_PROGRESS_CODES:
        return REASON_NO_PROGRESS
    if status == STATUS_ABORTED:
        return REASON_ABORTED_BY_SERVER
    return REASON_UNKNOWN


def summarize(
    track: MotionTrack,
    *,
    status: int | None,
    result_values: Mapping[str, Any] | None,
    end_pose: Mapping[str, Any] | None,
    range_observation: Mapping[str, Any] | None,
    clearance_floor_m: float,
    ended_at: float,
    end_map_pose: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The record a motion result carries; ``status`` None means still running.

    ``start_pose`` and ``final_pose`` are odometry. When localization was up
    the same poses in the map frame are added as ``start_map_pose`` and
    ``final_map_pose``, so a reader can name the place in the frame a
    navigation goal is given in.
    """
    start = _pose(track.start_pose)
    end = _pose(end_pose)
    minimum_at_stop: float | None = None
    sweep = None
    if isinstance(range_observation, Mapping):
        try:
            minimum_at_stop = float(range_observation["minimum_range_m"])
        except (KeyError, TypeError, ValueError):
            minimum_at_stop = None
        sweep = range_observation.get("sweep")
    ahead = travel_range(track.capability_id, sweep)
    # A straight drive is blocked by what is the way it was going; a wall
    # behind it is not in the way. Anything else is judged on the whole sweep.
    stop_range = ahead if track.capability_id in _TRAVEL_DIRECTION else minimum_at_stop
    if track.capability_id in _TRAVEL_DIRECTION and ahead is None:
        stop_range = minimum_at_stop
    error_code = _error_code(result_values)
    error_msg = ""
    if isinstance(result_values, Mapping):
        error_msg = str(result_values.get("error_msg") or "")[:200]
    guard_trip = (track.braking or {}).get("tripped")
    clearance = stop_clearance(
        track.capability_id, sweep, minimum_at_stop, clearance_floor_m
    )
    travelled: float | None = None
    if start is not None and end is not None and track.capability_id != "motion.rotate":
        travelled = math.hypot(end["x"] - start["x"], end["y"] - start["y"])
    facts = (
        stop_facts(track, travelled_m=travelled, clearance=clearance)
        if status is not None
        else None
    )
    rule = stop_rule(facts)
    reason = _reason(
        status=status,
        error_code=error_code,
        collision_events=track.collision_events,
        collision_now=track.collision_now,
        stop_range=stop_range,
        clearance_floor_m=clearance_floor_m,
        guard_trip=guard_trip if isinstance(guard_trip, str) else None,
        rule=rule,
    )
    summary: dict[str, Any] = {
        "reason": reason,
        "capability_id": track.capability_id,
        "action_status": status,
        "action_status_name": (
            STATUS_NAMES.get(status, "unknown") if status is not None else "still_running"
        ),
        "start_pose": start,
        "final_pose": end,
        "elapsed_seconds": round(max(0.0, ended_at - track.started_at), 3),
        "minimum_range_at_stop_m": (
            round(minimum_at_stop, 3) if minimum_at_stop is not None else None
        ),
        "minimum_range_during_m": (
            round(track.minimum_range_during_m, 3)
            if track.minimum_range_during_m is not None
            else None
        ),
        "clearance_floor_m": round(clearance_floor_m, 3),
    }
    start_map = _map_pose(track.start_map_pose)
    end_map = _map_pose(end_map_pose)
    if start_map is not None:
        summary["start_map_pose"] = start_map
    if end_map is not None:
        summary["final_map_pose"] = end_map
    if ahead is not None:
        summary["travel_direction_range_m"] = round(ahead, 3)
    if clearance is not None:
        summary["stop_clearance"] = clearance
    if rule is not None and facts is not None:
        # Which row decided, on what numbers, and why its threshold is that.
        summary["stop_rule"] = {
            "name": rule.name,
            "reason": rule.reason,
            "basis": rule.basis,
            "requested_m": round(facts.requested_m, 4),
            "commanded_m": round(facts.commanded_m, 4),
            "travelled_m": round(facts.travelled_m, 4),
            "shortfall_m": round(facts.shortfall_m, 4),
            "arrival_tolerance_m": round(facts.arrival_tolerance_m, 4),
        }
    if track.braking:
        summary["braking"] = dict(track.braking)
    if error_code is not None:
        summary["error_code"] = error_code
    if error_msg:
        summary["error_msg"] = error_msg
    if track.collision_events:
        summary["collision_monitor"] = [
            {"action_type": kind, "polygon": name} for kind, name in track.collision_events
        ]
    if track.collision_now is not None:
        kind, name = track.collision_now
        summary["collision_monitor_at_stop"] = {"action_type": kind, "polygon": name}
    if track.feedback:
        summary["feedback"] = dict(track.feedback)
    if status is not None:
        track.close_still(ended_at)
    if track.stalls:
        summary["stalls"] = [dict(stall) for stall in track.stalls]
    if track.recoveries_at_s:
        summary["recoveries_at_s"] = list(track.recoveries_at_s)
    summary["sim_time"] = track.sim_time_summary()
    if start is not None and end is not None:
        if track.capability_id == "motion.rotate":
            summary["yaw_turned_rad"] = round(
                abs(math.remainder(end["yaw"] - start["yaw"], math.tau)), 4
            )
        else:
            summary["distance_travelled_m"] = round(
                math.hypot(end["x"] - start["x"], end["y"] - start["y"]), 4
            )
    if "distance_m" in track.arguments:
        summary["requested_distance_m"] = float(track.arguments["distance_m"])
    plan = (track.braking or {}).get("plan")
    if isinstance(plan, Mapping) and plan.get("shortened"):
        # The caller asked for more than was sent; report against the ask.
        summary["requested_distance_m"] = float(plan["requested_distance_m"])
        summary["commanded_distance_m"] = float(plan["commanded_distance_m"])
    if track.capability_id == "motion.rotate" and "yaw_radians" in track.arguments:
        summary["requested_yaw_rad"] = abs(float(track.arguments["yaw_radians"]))
    if track.capability_id == "motion.navigate":
        summary["requested_goal"] = {
            key: float(track.arguments[key])
            for key in ("x", "y", "yaw_radians")
            if key in track.arguments
        }
    return summary


def describe(summary: Mapping[str, Any]) -> str:
    """One line for an operator, ahead of whatever the host appends."""
    status = summary.get("action_status")
    if status is None:
        head = "ROS 2 action still running at the adapter deadline"
    else:
        head = f"ROS 2 action status {status} ({summary.get('action_status_name')})"
    parts = [f"{head}: {summary.get('reason')}"]
    if "error_code" in summary:
        message = summary.get("error_msg")
        parts.append(
            f"error {summary['error_code']}" + (f" {message!r}" if message else "")
        )
    travelled = summary.get("distance_travelled_m")
    requested = summary.get("requested_distance_m")
    if travelled is not None and requested is not None:
        parts.append(f"travelled {travelled:.3f} of {requested:.3f} m")
    elif travelled is not None:
        parts.append(f"travelled {travelled:.3f} m")
    turned = summary.get("yaw_turned_rad")
    if turned is not None:
        wanted = summary.get("requested_yaw_rad")
        parts.append(
            f"turned {turned:.3f}" + (f" of {wanted:.3f}" if wanted is not None else "") + " rad"
        )
    nearest = summary.get("minimum_range_at_stop_m")
    clearance = summary.get("stop_clearance")
    clearance = clearance if isinstance(clearance, Mapping) else {}
    if nearest is not None:
        where = ""
        bearing = clearance.get("nearest_bearing_rad")
        if isinstance(bearing, (int, float)):
            where = (
                f" {clearance.get('nearest_direction')} "
                f"({math.degrees(bearing):+.0f} deg from travel)"
            )
        held = clearance.get("floor_held")
        verdict = "" if held is None else (", floor held" if held else ", floor NOT held")
        parts.append(
            f"nearest LiDAR return at stop {nearest:.3f} m{where} "
            f"(floor {summary.get('clearance_floor_m', 0.0):.3f} m{verdict})"
        )
    guard = summary.get("braking")
    if isinstance(guard, Mapping) and guard.get("tripped"):
        trip = guard.get("trip") if isinstance(guard.get("trip"), Mapping) else {}
        if guard.get("tripped") == GUARD_TRIP_CLEARANCE and trip:
            parts.append(
                f"braking guard stopped at {trip.get('speed_mps', 0.0):.3f} m/s with "
                f"{trip.get('room_m', 0.0):.3f} m to the floor "
                f"(stopping distance {trip.get('stopping_distance_m', 0.0):.3f} m)"
            )
        else:
            parts.append(f"braking guard stopped: LiDAR {guard.get('tripped')}")
    if isinstance(guard, Mapping) and guard.get("ended") == GUARD_END_PLANNED_STOP:
        plan = guard.get("plan") if isinstance(guard.get("plan"), Mapping) else {}
        room = plan.get("room_to_floor_at_send_m") or 0.0
        parts.append(
            f"ended at the stop point planned from {room:.3f} m "
            f"of room to the floor at send: {plan.get('commanded_distance_m', 0.0):.3f} m "
            f"at {plan.get('commanded_speed_mps', 0.0):.3f} m/s"
        )
    governance = guard.get("governance") if isinstance(guard, Mapping) else None
    if isinstance(governance, Mapping) and (
        guard.get("tripped") or guard.get("ended")
    ):
        parts.append(
            f"speed governance {governance.get('mode')} ({governance.get('source')})"
        )
    stopped = summary.get("final_map_pose")
    if isinstance(stopped, Mapping):
        parts.append(
            f"stopped at map x={stopped['x']:.3f} y={stopped['y']:.3f} "
            f"yaw={stopped['yaw']:.3f}"
        )
    return "; ".join(parts)
