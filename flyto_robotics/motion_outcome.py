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
    the action server reported success.
``cancelled``
    the goal was cancelled (an operator stop, a halt, a cancel).
``obstacle_blocked``
    the action reported a collision ahead, the collision monitor stopped the
    base for one of its polygons, or the nearest return the way the robot was
    going is inside the clearance floor at the stop.
``sensor_stale``
    the collision monitor stopped the base because its sensor data was late
    or missing ("invalid source"); the robot was not blocked, it was blind.
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
from collections.abc import Mapping
from dataclasses import dataclass, field
from typing import Any

REASON_COMPLETED = "completed"
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

# Half-width of the wedge read the way the robot was travelling.
TRAVEL_HALF_WIDTH_RAD = math.radians(20.0)
# Where that wedge is centred, relative to the robot's heading.
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

    def saw_feedback(self, values: Mapping[str, Any] | None) -> None:
        if not isinstance(values, Mapping):
            return
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


def _error_code(result_values: Mapping[str, Any] | None) -> int | None:
    if not isinstance(result_values, Mapping):
        return None
    try:
        code = int(result_values.get("error_code"))
    except (TypeError, ValueError):
        return None
    return code or None


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
) -> str:
    """Most specific first.

    What holds at the stop (Nav2's collision code, the monitor's state now,
    the range ahead) beats a specific Nav2 error, which beats an event the
    monitor raised and then cleared during the run: a slowdown for a box
    passed earlier must not turn "no path to the goal" into "blocked".
    """
    if status == STATUS_SUCCEEDED:
        return REASON_COMPLETED
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
) -> dict[str, Any]:
    """The record a motion result carries; ``status`` None means still running."""
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
    reason = _reason(
        status=status,
        error_code=error_code,
        collision_events=track.collision_events,
        collision_now=track.collision_now,
        stop_range=stop_range,
        clearance_floor_m=clearance_floor_m,
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
    if ahead is not None:
        summary["travel_direction_range_m"] = round(ahead, 3)
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
    if nearest is not None:
        parts.append(
            f"nearest LiDAR return at stop {nearest:.3f} m "
            f"(floor {summary.get('clearance_floor_m', 0.0):.3f} m)"
        )
    return "; ".join(parts)
