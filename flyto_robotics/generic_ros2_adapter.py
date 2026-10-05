#!/usr/bin/env python3
"""Generic ROS 2 adapter for an external AI Space / Computer.

This adapter is intentionally NOT a TurtleBot3 runtime. It runs on a Mac,
laptop, Steam Deck, mini PC, or AI Space computer that can see a standard ROS 2
graph over DDS, Zenoh, or another ordinary ROS transport.

Southbound, it speaks only standard ROS 2 interfaces:
- nav2_msgs/action/NavigateToPose
- nav2_msgs/action/DriveOnHeading
- nav2_msgs/action/BackUp
- nav2_msgs/action/Spin
- geometry_msgs/msg/Twist or TwistStamped for a final zero-velocity stop
- nav_msgs/msg/Odometry and sensor_msgs/msg/LaserScan for evidence
- sensor_msgs/msg/Image + CameraInfo and tf2_msgs/msg/TFMessage for observation parity

No Flyto2 process, credential, scheduler, gateway, task database, or custom ROS
node is required on the robot. Reinstalling a TurtleBot3 from upstream ROS 2
and TurtleBot3 documentation is a resource replacement, not Flyto2
re-provisioning.

The declarations arrive as device_report / DISCOVERED. ROS graph discovery is
evidence that an interface exists, not permission to move a robot.
"""

from __future__ import annotations

import base64
import contextlib
import hashlib
import json
import logging
import math
import os
import queue
import threading
import time
from collections.abc import Iterable, Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Callable, Protocol

from . import adapter_contract as decl
from . import braking_envelope as braking
from . import inflation_escape, motion_outcome, places, provider_evidence
from . import map_frame as map_frame_module
from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)
from .ros2_observation_bundle import Ros2ObservationError, runtime_snapshot
from .ros2_observation_bundle import (
    build_ros2_observation_bundle as build_observation_bundle,
)
from .scan_clearance import sweep as scan_sweep

logger = logging.getLogger(__name__)

CMD_VEL_TYPES = frozenset(
    {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}
)
ODOM_TYPE = "nav_msgs/msg/Odometry"
CLOCK_TYPE = "rosgraph_msgs/msg/Clock"


def _observation_wait_seconds() -> float:
    """How long an observation may wait for its first odometry and LiDAR.

    An execution host builds the adapter for a job and connects just before it
    is asked to move. Odometry arrives within a few messages, but the first
    LiDAR revolution can take longer, so a sub-second wait refused every first
    motion as "fresh LiDAR is required". A ready graph answers well inside this.
    """
    try:
        seconds = float(os.getenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "3"))
    except ValueError:
        seconds = 3.0
    return min(10.0, max(0.25, seconds))


# What an observation may wait for. Each is set by a subscription callback, so
# a wait for it ends on that callback rather than at a fixed spin deadline.
REQUIRE_POSE = "pose"
REQUIRE_RANGE = "range"
REQUIRE_MAP_TF = "map_tf"

# A robot is stationary once this many consecutive odometry messages report
# speeds this close to zero. A TurtleBot3 publishes odometry at about 30 Hz,
# so a robot that has stopped is recognised in about 0.1 s.
STILL_SAMPLES = 3
STILL_LINEAR_MPS = 0.01
STILL_ANGULAR_RADPS = 0.02
# Per-call results and execution counts kept for idempotent retries of the
# same call_id. A warm adapter lives as long as its host, so this is bounded.
CALL_HISTORY_LIMIT = 256
# For a driver whose odometry carries no twist: movement between two messages.
STILL_POSE_DELTA_M = 0.002
STILL_YAW_DELTA_RAD = 0.01
# A reading after this long without one means the robot came back (it was
# powered off, or its drivers restarted). Matches the execution host's
# silence threshold for "unreachable".
SILENT_SECONDS = 10.0
LIFECYCLE_EVENT_TYPE = "lifecycle_msgs/msg/TransitionEvent"


def _lifecycle_event_topics() -> tuple[str, ...]:
    """Lifecycle topics of the servers that host the motion actions.

    Nav2 brings its servers up as lifecycle nodes, after the map loads, and
    each one publishes a TransitionEvent when it activates. That message is
    the moment its actions join the graph, so a host is told then rather than
    finding out on its next discovery pass. Empty disables it.
    """
    raw = os.getenv(
        "FLYTO_ROS2_LIFECYCLE_EVENT_TOPICS",
        "/bt_navigator/transition_event,/behavior_server/transition_event",
    )
    return tuple(item.strip() for item in raw.split(",") if item.strip())


def _default_requirements() -> tuple[str, ...]:
    return (REQUIRE_POSE, REQUIRE_RANGE) if _needs_lidar() else (REQUIRE_POSE,)


def _clearance_floor() -> float:
    """The LiDAR clearance a motion needs to start; never below 0.1 m."""
    return max(0.1, float(os.getenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.35")))


# -- the braking guard ---------------------------------------------------------------
#
# The floor above is checked once, before a motion starts. A straight drive is
# then watched on every scan: it is stopped while the room left before a return
# reaches the floor is still at least the distance the robot needs to stop from
# its speed (braking_envelope), and its commanded speed is lowered as that room
# shrinks. Turns and planned paths have no single direction and stay with Nav2.

#: Direction of travel of each straight drive, from the robot's heading.
STRAIGHT_MOTIONS: Mapping[str, float] = {
    "motion.advance": 0.0,
    "motion.retreat": math.pi,
}
#: Speed a straight drive is sent at when the caller names none, in m/s.
DEFAULT_STRAIGHT_SPEED_MPS: Mapping[str, float] = {
    "motion.advance": 0.12,
    "motion.retreat": 0.10,
}
#: A drive is commanded at this fraction of the fastest speed its room allows,
#: so the guard is not tripped by the next scan of an approach it is managing.
SLOWDOWN_FRACTION = 0.8
#: A slowdown is sent only when it lowers the speed below this fraction of the
#: current one, so a long approach is a few steps, not a goal per scan.
SLOWDOWN_STEP = 0.9
#: Shortest interval between two slowdowns of one drive, in seconds.
SLOWDOWN_MIN_INTERVAL_S = 0.5


def _env_number(name: str, default: float, *, low: float, high: float) -> float:
    try:
        value = float(os.getenv(name, str(default)))
    except ValueError:
        return default
    if not math.isfinite(value):
        return default
    return min(high, max(low, value))


def _stop_latency_s() -> float:
    """Configured end-to-end stop latency; a measured one can only raise it."""
    return _env_number(
        "FLYTO_ROS2_STOP_LATENCY_S", braking.DEFAULT_LATENCY_S,
        low=braking.LATENCY_BOUNDS_S[0], high=braking.LATENCY_BOUNDS_S[1],
    )


def _stop_decel_mps2() -> float:
    """Configured achievable deceleration; a declared one can only lower it."""
    return _env_number(
        "FLYTO_ROS2_STOP_DECEL_MPS2", braking.DEFAULT_DECEL_MPS2,
        low=braking.DECEL_BOUNDS_MPS2[0], high=braking.DECEL_BOUNDS_MPS2[1],
    )


def _stop_control_period_s() -> float:
    """One cycle of the robot-side loop that acts on a cancel (Nav2 behaviors: 10 Hz)."""
    return _env_number("FLYTO_ROS2_STOP_CONTROL_PERIOD_S", 0.1, low=0.0, high=1.0)


def _stop_actuation_s() -> float:
    """Velocity smoothing and motor response after the stop is commanded."""
    return _env_number("FLYTO_ROS2_STOP_ACTUATION_S", 0.1, low=0.0, high=1.0)


def _decel_parameter() -> tuple[str, str] | None:
    """``node:parameter`` holding the base's declared deceleration limit."""
    raw = os.getenv("FLYTO_ROS2_DECEL_PARAMETER", "/velocity_smoother:max_decel").strip()
    node, _, name = raw.partition(":")
    if not node or not name:
        return None
    return node, name


def _declared_decel(value: Mapping[str, Any] | None) -> float | None:
    """The linear deceleration in a ParameterValue: a number, or x of an array."""
    number = inflation_escape.parameter_number(value)
    if number is None and isinstance(value, Mapping) and value.get("type") == 8:
        array = value.get("double_array_value")
        if isinstance(array, (list, tuple)) and array:
            with contextlib.suppress(TypeError, ValueError):
                number = float(array[0])
    if number is None or not math.isfinite(number) or number == 0.0:
        return None
    return abs(number)


def _scan_timeout_s(period_s: float | None) -> float:
    """How long a guarded drive goes without a scan before it is stopped."""
    measured = 3.0 * period_s if period_s is not None else 0.0
    configured = _env_number("FLYTO_ROS2_STOP_SCAN_TIMEOUT_S", 0.5, low=0.1, high=5.0)
    return max(configured, measured)


@dataclass
class _BrakingGuard:
    """One straight drive's braking envelope and what it did."""

    capability_id: str
    profile: braking.BrakingProfile
    direction_rad: float
    speed_mps: float
    requested_speed_mps: float
    distance_m: float
    start_room_m: float | None
    start_pose: Mapping[str, Any] | None = None
    tripped: str | None = None
    trip: dict[str, Any] | None = None
    # A slower speed the room now calls for, waiting to be sent.
    slow_to: float | None = None
    last_slowdown_at: float = 0.0
    speed_changes: list[dict[str, Any]] = field(default_factory=list)
    minimum_room_m: float | None = None
    # Odometry at the trip and at rest, for the measured stopping distance.
    trip_pose: Mapping[str, Any] | None = None
    stop_command_sent: bool = False

    def record(self, rest_pose: Mapping[str, Any] | None) -> dict[str, Any]:
        out: dict[str, Any] = {
            "profile": self.profile.to_dict(),
            "requested_speed_mps": round(self.requested_speed_mps, 4),
            "commanded_speed_mps": round(self.speed_mps, 4),
            "start_room_to_floor_m": (
                round(self.start_room_m, 4)
                if self.start_room_m is not None and math.isfinite(self.start_room_m)
                else None
            ),
            "required_clearance_at_start_m": round(
                self.profile.required_clearance(self.speed_changes[0]["speed_mps"])
                if self.speed_changes
                else self.profile.required_clearance(self.speed_mps),
                4,
            ),
            "speed_changes": list(self.speed_changes),
            "tripped": self.tripped,
        }
        if self.minimum_room_m is not None:
            out["minimum_room_to_floor_m"] = round(self.minimum_room_m, 4)
        if self.trip is not None:
            trip = dict(self.trip)
            if self.trip_pose is not None and rest_pose is not None:
                # What the robot actually covered after the guard acted:
                # the number that calibrates latency and deceleration.
                trip["measured_stop_distance_m"] = round(
                    math.hypot(
                        float(rest_pose["x"]) - float(self.trip_pose["x"]),
                        float(rest_pose["y"]) - float(self.trip_pose["y"]),
                    ),
                    4,
                )
            out["trip"] = trip
        return out


def _collision_state_topic() -> str:
    """Nav2's collision monitor state, read to tell why a motion stopped.

    Empty disables it for a robot without a collision monitor.
    """
    return os.getenv("FLYTO_ROS2_COLLISION_STATE_TOPIC", "/collision_monitor_state").strip()


COLLISION_STATE_TYPE = "nav2_msgs/msg/CollisionMonitorState"


def _max_observation_age() -> float:
    return max(0.1, float(os.getenv("FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS", "5")))


def _simulation_marker_topic() -> str:
    """Topic whose presence means the ROS graph is a simulator.

    A Gazebo graph publishes /clock for use_sim_time; the physical TurtleBot3
    does not. An empty value disables the cross-check for a robot that
    legitimately publishes /clock.
    """
    return os.getenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "/clock").strip()
SCAN_TYPE = "sensor_msgs/msg/LaserScan"

# What makes a motion safe to start on this robot, set by whoever installed it.
# LiDAR clearance is the default and the only basis that lets the adapter
# judge the space itself. A robot without LiDAR declares that a person is
# present instead: Cloud already holds every actuating capability for an
# operator's Run (direct_capabilities: actuates -> CONFIRM), so the adapter
# drops the clearance check, keeps odometry for verification, and bounds each
# motion so that person can still stop it. Any other value refuses motion.
SAFETY_BASIS_LIDAR = "lidar_clearance"
SAFETY_BASIS_OPERATOR = "operator_present"
SAFETY_BASES = (SAFETY_BASIS_LIDAR, SAFETY_BASIS_OPERATOR)
OPERATOR_PRESENT_OBSERVATION = "operator:present"
SUPERVISED_MAX_SPEED_MPS = 0.05
SUPERVISED_MAX_DISTANCE_M = 0.3
SUPERVISED_MAX_YAW_RAD = math.pi / 2


# The longest the adapter waits for odometry to show the robot stopped before
# it reads the settled pose; the same cap Flyto2 Desktop has always used.
SETTLE_SECONDS = 1.0

# Read-only capabilities that return one sensor reading; they never move.
CAPTURE_CAPABILITIES = frozenset({"vision.observe", "sensing.map"})
CAPTURE_WAIT_SECONDS = 5.0
# Well inside rosbridge's websocket_ping_timeout (20 s on the TurtleBot3).
KEEPALIVE_SECONDS = 5.0
MAX_PHOTO_BYTES = 2_000_000
MAX_MAP_CELLS = 4_000_000


def _message_bytes(raw: Any) -> bytes:
    """A uint8[]/int8[] field as rosbridge sends it: base64 text or a list."""
    if isinstance(raw, str):
        return base64.b64decode(raw, validate=False)
    if isinstance(raw, list):
        return bytes(int(value) & 0xFF for value in raw)
    raise ValueError("message data is not bytes")


def photo_capture(message: Mapping[str, Any]) -> dict[str, Any]:
    """A CompressedImage as a JPEG for the host to keep and show."""
    data = _message_bytes(message.get("data"))
    if not data.startswith(b"\xff\xd8"):
        raise ValueError("camera frame is not a JPEG; the compressed topic must publish jpeg")
    if len(data) > MAX_PHOTO_BYTES:
        raise ValueError("camera frame is larger than 2 MB")
    return {
        "kind": "photo",
        "media_type": "image/jpeg",
        "data_base64": base64.b64encode(data).decode("ascii"),
    }


def map_capture(message: Mapping[str, Any]) -> dict[str, Any]:
    """An OccupancyGrid's cells as bytes: 0-100 occupied, 255 unknown."""
    info = message.get("info") if isinstance(message.get("info"), Mapping) else {}
    width, height = int(info.get("width", 0)), int(info.get("height", 0))
    resolution = float(info.get("resolution", 0.0))
    if width <= 0 or height <= 0 or resolution <= 0 or width * height > MAX_MAP_CELLS:
        raise ValueError("map has no usable size")
    cells = _message_bytes(message.get("data"))
    if len(cells) != width * height:
        raise ValueError("map data does not match its size")
    raw_origin = info.get("origin")
    origin = raw_origin.get("position", {}) if isinstance(raw_origin, Mapping) else {}
    return {
        "kind": "map",
        "width": width,
        "height": height,
        "resolution_m": resolution,
        "origin": {"x": float(origin.get("x", 0.0)), "y": float(origin.get("y", 0.0))},
        "cells_base64": base64.b64encode(cells).decode("ascii"),
    }


# Motions that need the map, because Nav2 plans them.
PLANNED_MOTIONS = frozenset({"motion.navigate"})


def _safety_basis() -> str:
    return os.getenv("FLYTO_ROS2_SAFETY_BASIS", SAFETY_BASIS_LIDAR).strip().lower()


def _needs_lidar() -> bool:
    """Whether observations wait for LiDAR; anything but operator_present does."""
    return _safety_basis() != SAFETY_BASIS_OPERATOR


def _supervised_arguments(
    capability_id: str, arguments: Mapping[str, float]
) -> tuple[dict[str, float], str | None]:
    """Arguments bounded for operator_present, or why the motion is refused."""
    bounded = dict(arguments)
    if capability_id in PLANNED_MOTIONS:
        return bounded, (
            "navigation needs LiDAR clearance; this robot's safety basis is "
            "operator_present"
        )
    if capability_id in {"motion.advance", "motion.retreat"}:
        if bounded.get("distance_m", 0.0) > SUPERVISED_MAX_DISTANCE_M:
            return bounded, (
                f"operator_present allows at most {SUPERVISED_MAX_DISTANCE_M:.2f}m "
                "per motion"
            )
        bounded["speed_mps"] = min(
            bounded.get("speed_mps", SUPERVISED_MAX_SPEED_MPS), SUPERVISED_MAX_SPEED_MPS
        )
    turn = abs(bounded.get("yaw_radians", 0.0))
    if capability_id == "motion.rotate" and turn > SUPERVISED_MAX_YAW_RAD:
        return bounded, (
            f"operator_present allows at most {SUPERVISED_MAX_YAW_RAD:.2f}rad per turn"
        )
    return bounded, None

DEFAULT_INTERFACES = {
    "motion.navigate": (
        "action",
        os.getenv("FLYTO_ROS2_NAVIGATE_ACTION", "/navigate_to_pose"),
        "nav2_msgs/action/NavigateToPose",
    ),
    "vision.observe": (
        "topic",
        os.getenv("FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC", "/camera/image_raw/compressed"),
        "sensor_msgs/msg/CompressedImage",
    ),
    "sensing.map": (
        "topic",
        os.getenv("FLYTO_ROS2_MAP_TOPIC", "/map"),
        "nav_msgs/msg/OccupancyGrid",
    ),
    "motion.advance": (
        "action",
        os.getenv("FLYTO_ROS2_ADVANCE_ACTION", "/drive_on_heading"),
        "nav2_msgs/action/DriveOnHeading",
    ),
    "motion.retreat": (
        "action",
        os.getenv("FLYTO_ROS2_BACKUP_ACTION", "/backup"),
        "nav2_msgs/action/BackUp",
    ),
    "motion.rotate": (
        "action",
        os.getenv("FLYTO_ROS2_SPIN_ACTION", "/spin"),
        "nav2_msgs/action/Spin",
    ),
    "motion.halt": (
        "topic",
        os.getenv("FLYTO_ROS2_CMD_VEL_TOPIC", "/cmd_vel"),
        "geometry_msgs/msg/Twist|geometry_msgs/msg/TwistStamped",
    ),
}

# Interfaces the adapter uses on its own behalf, never declared as capabilities:
# an escape from an obstacle's inflation sends its waypoint and the original
# goal as one NavigateThroughPoses goal when the graph has it.
NAVIGATE_THROUGH = "motion.navigate_through_poses"
ESCAPE_INTERFACES = {
    NAVIGATE_THROUGH: (
        "action",
        os.getenv("FLYTO_ROS2_NAVIGATE_THROUGH_ACTION", "/navigate_through_poses"),
        "nav2_msgs/action/NavigateThroughPoses",
    ),
}
GET_PARAMETERS_TYPE = "rcl_interfaces/srv/GetParameters"


def _interface(capability_id: str) -> tuple[str, str, str]:
    """A declared capability's interface, or one the adapter uses itself."""
    return DEFAULT_INTERFACES.get(capability_id) or ESCAPE_INTERFACES[capability_id]


def _env_metres(name: str, default: float, *, low: float, high: float) -> float:
    try:
        value = float(os.getenv(name, str(default)))
    except ValueError:
        return default
    if not math.isfinite(value):
        return default
    return min(high, max(low, value))


def _costmap_nodes() -> tuple[str, ...]:
    """Costmap nodes whose parameters say how far obstacles are inflated."""
    raw = os.getenv(
        "FLYTO_ROS2_COSTMAP_NODES",
        "/local_costmap/local_costmap,/global_costmap/global_costmap",
    )
    return tuple(item.strip() for item in raw.split(",") if item.strip())


def _escape_enabled() -> bool:
    return os.getenv("FLYTO_ROS2_INFLATION_ESCAPE", "on").strip().lower() not in {
        "0",
        "off",
        "false",
        "no",
    }


# How an escape backs off: slowly, since it starts close to an obstacle.
ESCAPE_BACKOFF_SPEED_MPS = 0.05


def _goal_pose(frame: str, x: float, y: float, yaw: float) -> dict[str, Any]:
    """A geometry_msgs/PoseStamped as rosbridge sends it."""
    return {
        "header": {"frame_id": frame},
        "pose": {
            "position": {"x": float(x), "y": float(y), "z": 0.0},
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": math.sin(yaw / 2.0),
                "w": math.cos(yaw / 2.0),
            },
        },
    }


def _route_poses(arguments: Mapping[str, Any]) -> list[tuple[float, float, float]]:
    """A NavigateThroughPoses call's poses: its waypoints, then the goal."""
    poses = [
        (float(item["x"]), float(item["y"]), float(item.get("yaw_radians", 0.0)))
        for item in arguments.get("waypoints", ())
    ]
    poses.append(
        (float(arguments["x"]), float(arguments["y"]), float(arguments.get("yaw_radians", 0.0)))
    )
    return poses


# Named places, kept on this host (``places.py``), not on the robot.
PLACES_LIST = "places.list"
PLACES_MARK = "places.mark"
PLACE_CAPABILITIES = frozenset({PLACES_LIST, PLACES_MARK})
# Declared beside navigation: a place is a navigation target on the same map.
HOST_INTERFACES = {
    PLACES_LIST: ("host", "places", places.PLACES_SCHEMA),
    PLACES_MARK: ("host", "places", places.PLACES_SCHEMA),
}
# What a places.list result returns as a contract artifact.
PLACES_ARTIFACT_KIND = "places"
PLACES_MEDIA_TYPE = "application/json"


def _place_argument(description: str, *, required: bool) -> decl.DeclaredArgument:
    return decl.DeclaredArgument(
        "place",
        type="string",
        required=required,
        description=description,
        max_length=places.MAX_NAME_LENGTH,
    )


ARGUMENTS: Mapping[str, tuple[decl.DeclaredArgument, ...]] = {
    # Either x and y, or a place saved on this host; never both. A place
    # carries its own heading, so yaw_radians goes only with x and y.
    "motion.navigate": (
        decl.DeclaredArgument("x", required=False, minimum=-1000.0, maximum=1000.0, unit="m"),
        decl.DeclaredArgument("y", required=False, minimum=-1000.0, maximum=1000.0, unit="m"),
        decl.DeclaredArgument(
            "yaw_radians", required=False, minimum=-math.pi, maximum=math.pi, unit="rad"
        ),
        _place_argument("A named place on this map, instead of x and y", required=False),
    ),
    PLACES_LIST: (),
    PLACES_MARK: (_place_argument("The name to save the current position under", required=True),),
    "vision.observe": (),
    "sensing.map": (),
    "motion.advance": (
        decl.DeclaredArgument(
            "distance_m", required=True, minimum=0.05, maximum=2.0, unit="m"
        ),
        decl.DeclaredArgument(
            "speed_mps", required=False, minimum=0.02, maximum=0.25, unit="m/s"
        ),
    ),
    "motion.retreat": (
        decl.DeclaredArgument(
            "distance_m", required=True, minimum=0.05, maximum=2.0, unit="m"
        ),
        decl.DeclaredArgument(
            "speed_mps", required=False, minimum=0.02, maximum=0.20, unit="m/s"
        ),
    ),
    "motion.rotate": (
        decl.DeclaredArgument(
            "yaw_radians", required=True, minimum=-math.pi, maximum=math.pi, unit="rad"
        ),
    ),
    "motion.halt": (),
}


@dataclass(frozen=True)
class StandardInterface:
    kind: str
    name: str
    type: str


class ROS2Backend(Protocol):
    def discover(self) -> Sequence[StandardInterface]: ...

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult: ...

    def cancel(self, call_id: str) -> CallResult: ...

    def safe_stop(self, call_id: str) -> CallResult: ...

    def execution_count(self, call_id: str) -> int: ...

    def observation(self, required: Iterable[str] | None = None) -> Mapping[str, Any]: ...


class _ObservationState:
    """Latest readings shared by both backends, and waits that end on them.

    Subscription callbacks store each reading under ``self._condition`` and
    notify it. A caller that needs a reading the robot has not sent yet waits
    on that condition, so it returns on the callback that delivers it, never
    after a fixed spin or sleep; the observation wait only caps how long.
    """

    _connected: bool
    _results: dict[str, CallResult]
    _counts: dict[str, int]

    def _init_observation_state(self) -> None:
        self._condition = threading.Condition()
        # Told when the transport drops or returns, so a host keeping this
        # connection warm reconnects on the drop rather than on a timer.
        self._connection_listeners: list[Callable[[bool], None]] = []
        self._pose: dict[str, float | str] | None = None
        self._pose_seen_at: float | None = None
        # (linear x, angular z) from the odometry twist, or None if absent.
        self._velocity: tuple[float, float] | None = None
        self._odom_sequence = 0
        self._minimum_range: float | None = None
        self._range_sample_count = 0
        self._range_sweep: dict[str, Any] | None = None
        self._range_seen_at: float | None = None
        self._camera: dict[str, Any] | None = None
        self._camera_seen_at: float | None = None
        self._camera_calibration_snapshot: str | None = None
        self._map_tf_seen_at: float | None = None
        # The latest map->odom transform as (x, y, yaw), when localization
        # published one with its values; composed with odometry into the
        # robot's map pose (see map_frame).
        self._map_from_odom: tuple[float, float, float] | None = None
        # When this connection started listening: a robot never heard from
        # since then has been silent for as long as it has been listening.
        self._listening_since = time.monotonic()
        # Told what changed on the robot's side of the graph (see
        # add_graph_listener). Delivered on their own thread, never under
        # ``self._condition``, so a listener may call back into the backend.
        self._graph_listeners: list[Callable[[str], None]] = []
        self._graph_events: queue.SimpleQueue | None = None
        # Reading kinds heard on this connection, and when the last one came.
        self._heard_kinds: set[str] = set()
        self._last_heard_at: float | None = None
        # What each motion goal saw while it ran, so its result can say why
        # it ended (see motion_outcome). Keyed by call_id, bounded like results.
        self._motion_tracks: dict[str, motion_outcome.MotionTrack] = {}
        # The collision monitor's latest (action type, polygon); it publishes
        # only on a change, so a new motion starts from this one.
        self._collision_state: tuple[int, str] | None = None
        # Scan arrival intervals and delivery delays, measured as they come:
        # the latency term of every straight drive's braking envelope.
        self._latency = braking.LatencyEstimator()
        # The base's declared deceleration limit, read once per connection.
        self._declared_decel_mps2: float | None = None
        self._declared_decel_read = False
        # Straight drives being watched on every scan, keyed by call_id.
        self._guards: dict[str, _BrakingGuard] = {}

    def _begin_track(
        self, call_id: str, capability_id: str, arguments: Mapping[str, Any]
    ) -> None:
        with self._condition:
            snapshot = self._snapshot()
            track = motion_outcome.MotionTrack(
                capability_id=capability_id,
                arguments={
                    key: float(value)
                    for key, value in arguments.items()
                    if isinstance(value, (int, float))
                },
                start_pose=snapshot.get("pose"),
                start_map_pose=snapshot.get("map_pose"),
                started_at=time.monotonic(),
                collision_now=self._collision_state,
            )
            reading = snapshot.get("range")
            if isinstance(reading, Mapping):
                track.saw_range(reading.get("minimum_range_m"))
            _remember(self._motion_tracks, call_id, track)

    # -- braking guard ---------------------------------------------------------

    def _note_scan_timing(self, observed: float, stamp_s: float | None) -> None:
        """Called under the condition with each scan's arrival and header stamp."""
        delay = time.time() - stamp_s if stamp_s else None
        self._latency.observe(observed, delay)

    def _declared_deceleration(self) -> float | None:
        """The base's declared deceleration limit, or None if it says none.

        Read once per connection, never under the condition: it is a service
        call to the robot.
        """
        if self._declared_decel_read:
            return self._declared_decel_mps2
        self._declared_decel_read = True
        target = _decel_parameter()
        reader = getattr(self, "get_parameter", None)
        if target is None or not callable(reader):
            return None
        try:
            value = reader(*target)
        except Exception as error:  # noqa: BLE001 - a missing node is "not declared"
            logger.info("declared deceleration %s:%s unreadable: %s", *target, error)
            return None
        self._declared_decel_mps2 = _declared_decel(value)
        return self._declared_decel_mps2

    def braking_profile(self) -> braking.BrakingProfile:
        """The envelope a straight drive is guarded with, right now."""
        declared = self._declared_deceleration()
        with self._condition:
            budget = self._latency.budget(
                control_period_s=_stop_control_period_s(),
                actuation_s=_stop_actuation_s(),
            )
        return braking.resolve_profile(
            floor_m=_clearance_floor(),
            configured_latency_s=_stop_latency_s(),
            configured_decel_mps2=_stop_decel_mps2(),
            measured_latency=budget,
            declared_decel_mps2=declared,
        )

    def arm_braking_guard(
        self,
        call_id: str,
        capability_id: str,
        *,
        profile: braking.BrakingProfile,
        speed_mps: float,
        requested_speed_mps: float,
        distance_m: float,
        start_room_m: float | None,
    ) -> None:
        """Watch the straight drive ``call_id`` against ``profile`` from now on."""
        with self._condition:
            if call_id in self._guards:
                return  # a resumed call keeps the guard it started with
            guard = _BrakingGuard(
                capability_id=capability_id,
                profile=profile,
                direction_rad=STRAIGHT_MOTIONS[capability_id],
                speed_mps=float(speed_mps),
                requested_speed_mps=float(requested_speed_mps),
                distance_m=float(distance_m),
                start_room_m=start_room_m,
                start_pose=dict(self._pose) if self._pose is not None else None,
            )
            guard.speed_changes.append(
                {"at_s": 0.0, "speed_mps": round(float(speed_mps), 4), "room_m": (
                    round(start_room_m, 4)
                    if start_room_m is not None and math.isfinite(start_room_m)
                    else None
                )}
            )
            guard.last_slowdown_at = time.monotonic()
            _remember(self._guards, call_id, guard)

    def _trip_guard(
        self, guard: _BrakingGuard, cause: str, detail: Mapping[str, Any]
    ) -> None:
        """Called under the condition: this drive must stop now."""
        if guard.tripped is not None:
            return
        guard.tripped = cause
        guard.trip = dict(detail)
        guard.trip_pose = dict(self._pose) if self._pose is not None else None
        guard.slow_to = None

    def _check_guards(self) -> None:
        """Called under the condition after each scan: judge every guarded drive."""
        if not self._guards:
            return
        measured = abs(self._velocity[0]) if self._velocity is not None else 0.0
        for guard in self._guards.values():
            if guard.tripped is not None:
                continue
            room = braking.room_to_floor(
                self._range_sweep, guard.direction_rad, guard.profile.floor_m
            )
            if room is None:
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_BLIND, {
                    "detail": "the LiDAR sweep could not be read",
                })
                continue
            if guard.minimum_room_m is None or room.room_m < guard.minimum_room_m:
                guard.minimum_room_m = room.room_m
            # The robot may still be faster than the last command asked for.
            speed = max(guard.speed_mps, measured)
            stopping = guard.profile.stopping_distance(speed)
            allowed = guard.profile.max_speed(room.room_m) * SLOWDOWN_FRACTION
            if room.room_m <= stopping or allowed < braking.MIN_SPEED_MPS:
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_CLEARANCE, {
                    "room_m": round(room.room_m, 4),
                    "speed_mps": round(speed, 4),
                    "stopping_distance_m": round(stopping, 4),
                    "required_clearance_m": round(guard.profile.required_clearance(speed), 4),
                    **room.to_dict(),
                })
                continue
            if (
                allowed < guard.speed_mps * SLOWDOWN_STEP
                and time.monotonic() - guard.last_slowdown_at >= SLOWDOWN_MIN_INTERVAL_S
            ):
                guard.slow_to = allowed
        self._condition.notify_all()

    def _check_scan_fresh(self, call_id: str) -> None:
        """Stop a guarded drive whose LiDAR has gone quiet."""
        with self._condition:
            guard = self._guards.get(call_id)
            if guard is None or guard.tripped is not None:
                return
            limit = _scan_timeout_s(self._latency.scan_period_s)
            seen = self._range_seen_at
            age = None if seen is None else time.monotonic() - seen
            if age is None or age > limit:
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_STALE, {
                    "scan_age_s": None if age is None else round(age, 3),
                    "limit_s": round(limit, 3),
                })

    def _guard(self, call_id: str) -> _BrakingGuard | None:
        with self._condition:
            return self._guards.get(call_id)

    def _track_range(self, minimum_range_m: float) -> None:
        """Called under the condition with each LiDAR minimum."""
        for track in self._motion_tracks.values():
            track.saw_range(minimum_range_m)

    def _track_collision_state(self, action_type: Any, polygon_name: Any) -> None:
        try:
            kind = int(action_type)
        except (TypeError, ValueError):
            return
        with self._condition:
            self._collision_state = (kind, str(polygon_name or "")[:64])
            for track in self._motion_tracks.values():
                track.saw_collision_state(action_type, polygon_name)

    def _track_feedback(self, call_id: str, values: Any) -> None:
        with self._condition:
            track = self._motion_tracks.get(call_id)
            if track is not None:
                track.saw_feedback(values)

    def _motion_summary(
        self,
        call_id: str,
        *,
        status: int | None,
        result_values: Mapping[str, Any] | None = None,
    ) -> dict[str, Any] | None:
        """Why the motion ``call_id`` ended; None if it was never tracked.

        A terminal ``status`` ends the track. None (the adapter's deadline
        came first) keeps it, as the goal is still running.
        """
        with self._condition:
            track = (
                self._motion_tracks.get(call_id)
                if status is None
                else self._motion_tracks.pop(call_id, None)
            )
            if track is None:
                return None
            snapshot = self._snapshot()
            guard = (
                self._guards.get(call_id)
                if status is None
                else self._guards.pop(call_id, None)
            )
            if guard is not None:
                track.braking = guard.record(snapshot.get("pose"))
        return motion_outcome.summarize(
            track,
            status=status,
            result_values=result_values,
            end_pose=snapshot.get("pose"),
            end_map_pose=snapshot.get("map_pose"),
            range_observation=snapshot.get("range"),
            clearance_floor_m=_clearance_floor(),
            ended_at=time.monotonic(),
        )

    def _forget_track(self, call_id: str) -> None:
        with self._condition:
            self._motion_tracks.pop(call_id, None)
            self._guards.pop(call_id, None)

    def add_graph_listener(self, listener: Callable[[str], None]) -> None:
        """Call ``listener(reason)`` when the robot's graph may have changed.

        Fired on state, never on a timer: the first odometry on a connection
        (the robot's drivers are up), odometry after a silence (the robot came
        back), the first map transform (localization is up), and a Nav2
        lifecycle transition (its servers, and their actions, came or went).
        A host re-reads the graph and republishes the resource then.
        """
        with self._condition:
            self._graph_listeners.append(listener)
            self._graph_event_queue()

    def _graph_event_queue(self) -> queue.SimpleQueue | None:
        """The delivery queue, started on demand while anyone is listening."""
        if self._graph_events is None and self._graph_listeners:
            self._graph_events = queue.SimpleQueue()
            threading.Thread(
                target=self._deliver_graph_events,
                args=(self._graph_events,),
                name="ros2-graph-events",
                daemon=True,
            ).start()
        return self._graph_events

    def _stop_graph_events(self) -> None:
        """End the delivery thread with the connection; the next event restarts it."""
        with self._condition:
            events, self._graph_events = self._graph_events, None
        if events is not None:
            events.put(None)

    def _deliver_graph_events(self, events: queue.SimpleQueue) -> None:
        while True:
            reason = events.get()
            if reason is None:
                return
            for listener in tuple(self._graph_listeners):
                try:
                    listener(reason)
                except Exception:  # noqa: BLE001 - a listener never breaks the transport
                    continue

    def _graph_changed(self, reason: str) -> None:
        """Record that the graph may have changed; safe under the condition."""
        self._forget_graph()
        events = self._graph_event_queue()
        if events is not None:
            events.put(reason)

    def _forget_graph(self) -> None:
        """Drop a cached view of the graph; a backend with one overrides this."""

    def _heard(self, kind: str, observed: float) -> None:
        """Note a reading; the first of its kind, or one after silence, is news."""
        previous = self._last_heard_at
        self._last_heard_at = observed
        if kind not in self._heard_kinds:
            self._heard_kinds.add(kind)
            self._graph_changed(f"{kind}_appeared")
        elif previous is not None and observed - previous >= SILENT_SECONDS:
            self._graph_changed(f"{kind}_returned")

    def _store_odometry(
        self,
        pose: dict[str, float | str],
        velocity: tuple[float, float] | None,
        observed: float,
    ) -> None:
        with self._condition:
            self._pose = pose
            self._velocity = velocity
            self._pose_seen_at = observed
            self._odom_sequence += 1
            self._heard("odometry", observed)
            self._condition.notify_all()

    def _reset_readings(self) -> None:
        """Forget readings from a connection that has ended.

        A reading from before a drop says nothing about the robot now, so the
        next observation waits for the new connection's first message instead
        of passing an old pose or scan off as fresh.
        """
        with self._condition:
            self._pose_seen_at = None
            self._velocity = None
            self._range_seen_at = None
            self._camera_seen_at = None
            self._map_tf_seen_at = None
            self._map_from_odom = None
            self._latency.reset()
            self._declared_decel_read = False
            self._declared_decel_mps2 = None
            self._listening_since = time.monotonic()
            self._heard_kinds.clear()
            self._last_heard_at = None
            self._condition.notify_all()

    def silent_seconds(self) -> float | None:
        """Seconds since the robot last sent any subscribed reading.

        A transport can stay up while the robot behind it has gone (an rclpy
        node keeps spinning when the robot powers off, and a rosbridge may
        outlive its drivers). A robot publishing odometry is never silent
        for long, so a host reads a long silence as the equipment being
        unreachable. None while disconnected.
        """
        if not self._connected:
            return None
        with self._condition:
            heard = [
                seen
                for seen in (
                    self._pose_seen_at,
                    self._range_seen_at,
                    self._camera_seen_at,
                    self._map_tf_seen_at,
                )
                if seen is not None
            ]
            last = max(heard, default=self._listening_since)
        return max(0.0, time.monotonic() - last)

    def _missing(self, required: Iterable[str]) -> list[str]:
        # Stale counts as missing: on a warm connection the next callback is
        # one message period away, so it is waited for rather than refused.
        now = time.monotonic()
        max_age = _max_observation_age()
        seen = {
            REQUIRE_POSE: self._pose_seen_at,
            REQUIRE_RANGE: self._range_seen_at,
            REQUIRE_MAP_TF: self._map_tf_seen_at,
        }
        return [
            key
            for key in required
            if seen.get(key) is None or now - seen[key] > max_age
        ]

    def is_connected(self) -> bool:
        return bool(self._connected)

    def add_connection_listener(self, listener: Callable[[bool], None]) -> None:
        self._connection_listeners.append(listener)

    def _notify_connection(self, connected: bool) -> None:
        for listener in tuple(self._connection_listeners):
            try:
                listener(connected)
            except Exception:  # noqa: BLE001 - a listener never breaks the transport
                continue

    def _keep_result(self, call_id: str, result: CallResult) -> CallResult:
        _remember(self._results, call_id, result)
        return result

    def _count_execution(self, call_id: str) -> None:
        _remember(self._counts, call_id, self._counts.get(call_id, 0) + 1)

    def observation(self, required: Iterable[str] | None = None) -> Mapping[str, Any]:
        """Current readings, waiting only for a required one not fresh yet.

        ``required`` defaults to odometry, plus LiDAR unless the robot's safety
        basis is operator_present. A warm connection already holds them, so
        this returns at once.
        """
        needed = tuple(_default_requirements() if required is None else required)
        deadline = time.monotonic() + _observation_wait_seconds()
        with self._condition:
            while self._connected and self._missing(needed):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._condition.wait(timeout=remaining)
            return self._snapshot()

    def held_observation(self) -> dict[str, Any]:
        """The readings held now, without waiting for any.

        For evidence read after a motion failed: the host's safe stop comes
        next and must not first wait out the observation timeout.
        """
        with self._condition:
            return self._snapshot()

    def _store_map_transform(
        self, transform: tuple[float, float, float] | None, observed: float
    ) -> None:
        """Record a map->odom transform; called under ``self._condition``."""
        self._map_tf_seen_at = observed
        self._map_from_odom = transform
        self._heard("map_transform", observed)
        self._condition.notify_all()

    def _snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        max_age = _max_observation_age()

        def fresh(seen_at: float | None) -> bool:
            return seen_at is not None and now - seen_at <= max_age

        pose = (
            dict(self._pose)
            if self._pose is not None and fresh(self._pose_seen_at)
            else None
        )
        # The same pose in the map frame, only while localization's transform
        # is fresh too: a stale correction would place the robot somewhere it
        # was. Odometry stays the pose motion is judged on.
        map_pose = (
            map_frame_module.compose(self._map_from_odom, pose)
            if pose is not None and fresh(self._map_tf_seen_at)
            else None
        )
        return {
            "pose": pose,
            "map_pose": map_pose,
            "range": (
                {
                    "minimum_range_m": self._minimum_range,
                    "sample_count": self._range_sample_count,
                    **({"sweep": self._range_sweep} if self._range_sweep else {}),
                }
                if self._minimum_range is not None and fresh(self._range_seen_at)
                else None
            ),
            "camera": (
                dict(self._camera)
                if self._camera is not None and fresh(self._camera_seen_at)
                else None
            ),
            "map_tf_available": fresh(self._map_tf_seen_at),
        }

    def _still(self, previous: Mapping[str, Any] | None) -> bool:
        if self._velocity is not None:
            linear, angular = self._velocity
            return abs(linear) <= STILL_LINEAR_MPS and abs(angular) <= STILL_ANGULAR_RADPS
        if previous is None or self._pose is None:
            return False
        moved = math.hypot(
            float(self._pose["x"]) - float(previous["x"]),
            float(self._pose["y"]) - float(previous["y"]),
        )
        turned = abs(
            math.remainder(float(self._pose["yaw"]) - float(previous["yaw"]), math.tau)
        )
        return moved <= STILL_POSE_DELTA_M and turned <= STILL_YAW_DELTA_RAD

    def wait_until_stationary(
        self, max_seconds: float = 1.0, *, samples: int = STILL_SAMPLES
    ) -> dict[str, Any]:
        """Return once ``samples`` consecutive odometry messages show no motion.

        Each odometry callback re-judges the robot, so a robot that has stopped
        is recognised on the messages that say so. ``max_seconds`` is only the
        cap: a robot still moving then is reported as drifting, not waited for.
        """
        needed = max(1, int(samples))
        started = time.monotonic()
        deadline = started + max(0.0, float(max_seconds))
        still = 0
        with self._condition:
            seen = self._odom_sequence
            previous = dict(self._pose) if self._pose is not None else None
            while still < needed and self._connected:
                if self._odom_sequence == seen:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(timeout=remaining)
                    continue
                seen = self._odom_sequence
                still = still + 1 if self._still(previous) else 0
                previous = dict(self._pose) if self._pose is not None else None
        stationary = still >= needed
        return {
            "stationary": stationary,
            "drifting": not stationary,
            "still_samples": still,
            "waited_seconds": round(time.monotonic() - started, 3),
        }


def _travelled_along(
    start: Mapping[str, Any] | None,
    now: Mapping[str, Any] | None,
    direction_rad: float,
) -> float:
    """Odometry distance covered along a straight drive's direction of travel."""
    if not isinstance(start, Mapping) or not isinstance(now, Mapping):
        return 0.0
    heading = float(start["yaw"]) + direction_rad
    return (float(now["x"]) - float(start["x"])) * math.cos(heading) + (
        float(now["y"]) - float(start["y"])
    ) * math.sin(heading)


def _header_stamp(message: Mapping[str, Any]) -> float | None:
    """A rosbridge message's header stamp in seconds, or None."""
    header = message.get("header")
    stamp = header.get("stamp") if isinstance(header, Mapping) else None
    if not isinstance(stamp, Mapping):
        return None
    try:
        value = float(stamp.get("sec", 0)) + float(stamp.get("nanosec", 0)) * 1e-9
    except (TypeError, ValueError):
        return None
    return value if value > 0.0 else None


def _remember(store: dict[str, Any], key: str, value: Any) -> None:
    """Store ``value`` as the newest entry, dropping the oldest past the limit."""
    store.pop(key, None)
    store[key] = value
    while len(store) > CALL_HISTORY_LIMIT:
        store.pop(next(iter(store)))


def _navigate_target(arguments: Mapping[str, Any]) -> None:
    """Exactly one target: a place, or x and y (with an optional heading)."""
    has_place = "place" in arguments
    if has_place and ("x" in arguments or "y" in arguments):
        raise ValueError("give either place or x and y, not both")
    if has_place and "yaw_radians" in arguments:
        raise ValueError("a place carries its own heading; yaw_radians goes only with x and y")
    if not has_place and not ("x" in arguments and "y" in arguments):
        raise ValueError("x and y are required unless a place is named")


def _numeric_arguments(capability_id: str, arguments: Mapping[str, Any]) -> dict[str, Any]:
    """The call's arguments validated against ``ARGUMENTS``; text is a place name."""
    schemas = {item.name: item for item in ARGUMENTS[capability_id]}
    unknown = set(arguments) - set(schemas)
    if unknown:
        raise ValueError(f"unsupported arguments: {sorted(unknown)}")
    result: dict[str, Any] = {}
    for name, schema in schemas.items():
        raw = arguments.get(name)
        if raw is None:
            if schema.required:
                raise ValueError(f"{name} is required")
            continue
        if schema.type == "string":
            result[name] = places.normalize_name(raw)
            continue
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{name} must be a number")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        if schema.minimum is not None and value < schema.minimum:
            raise ValueError(f"{name} is below the declared minimum")
        if schema.maximum is not None and value > schema.maximum:
            raise ValueError(f"{name} is above the declared maximum")
        result[name] = value
    if capability_id == "motion.navigate":
        _navigate_target(result)
    return result


def _target_evidence(target: Mapping[str, Any], place: places.Place | None) -> dict[str, Any]:
    """Where a navigation was sent, and, for a place, the coordinates it stood for.

    ``navigation_target`` is the goal handed to Nav2, in the map frame.
    ``resolved_arguments`` exists only for a call by place: the x, y and
    yaw_radians the place resolved to, which are the arguments the arrival
    evidence (``distance_to`` on ``map_pose``) is judged against.
    """
    evidence: dict[str, Any] = {
        "navigation_target": {
            "frame": places.MAP_FRAME,
            "x": target["x"],
            "y": target["y"],
            **({"yaw_radians": target["yaw_radians"]} if "yaw_radians" in target else {}),
            **({"place": place.name} if place is not None else {}),
        }
    }
    if place is not None:
        evidence["resolved_arguments"] = {
            "x": place.x,
            "y": place.y,
            "yaw_radians": place.yaw,
        }
    return evidence


def _places_artifact(listed: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    raw = json.dumps(list(listed), ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    return {
        "kind": PLACES_ARTIFACT_KIND,
        "media_type": PLACES_MEDIA_TYPE,
        "data_base64": base64.b64encode(raw).decode("ascii"),
    }


def _message_fields(message: Any) -> dict[str, Any] | None:
    """The fields motion_outcome reads from an rclpy feedback or result."""
    if message is None:
        return None
    fields: dict[str, Any] = {}
    for name in (
        "error_code",
        "error_msg",
        "distance_traveled",
        "angular_distance_traveled",
        "distance_remaining",
        "number_of_recoveries",
    ):
        value = getattr(message, name, None)
        if isinstance(value, (int, float, str)):
            fields[name] = value
    return fields


def _motion_result(
    call_id: str,
    outcome: str,
    evidence: Mapping[str, Any],
    summary: Mapping[str, Any] | None,
    *,
    fallback: str = "",
) -> CallResult:
    """A motion's result carrying why it ended (``evidence.motion_outcome``).

    A motion that did not complete leads its detail with the reason, so an
    operator reads "obstacle_blocked; travelled 0.12 of 0.30 m" rather than
    a bare action status. A completed or cancelled one keeps its usual detail.
    """
    merged = dict(evidence)
    if summary is not None:
        merged["motion_outcome"] = dict(summary)
    detail = fallback
    if summary is not None and outcome in {OUTCOME_FAILED, OUTCOME_TIMEOUT}:
        detail = motion_outcome.describe(summary)
    return CallResult(call_id, outcome, evidence=merged, detail=detail)


def _with_artifacts(result: CallResult) -> CallResult:
    """A capture's result with its picture also as a contract artifact.

    ``capture`` stays for hosts released before artifacts; ``artifacts`` is
    what ``flyto.capability-contract.v1`` hosts keep. A map that cannot be
    drawn keeps its cells and returns no artifact, which a contract host
    reports as a failed capture.
    """
    capture = (result.evidence or {}).get("capture")
    if result.outcome != OUTCOME_COMPLETED or not isinstance(capture, Mapping):
        return result
    try:
        artifacts = provider_evidence.capture_artifacts(capture)
    except (ValueError, KeyError, TypeError, OSError):
        logger.warning("could not turn a %s capture into an artifact", capture.get("kind"))
        return result
    return CallResult(
        result.call_id,
        result.outcome,
        evidence={**dict(result.evidence), "artifacts": artifacts},
        detail=result.detail,
    )


class GenericROS2Adapter:
    """Flyto2 capability adapter over a standard ROS 2 graph."""

    # One connection may serve every job for this resource: per-call state is
    # keyed by call_id, and a host keeping it warm skips the connect,
    # subscribe and first-sample wait a fresh adapter pays on every job.
    supports_shared_connection = True

    def __init__(
        self,
        *,
        backend: ROS2Backend | None = None,
        resource_id: str = "",
        places_store: places.PlacesStore | None = None,
    ):
        self.backend = backend or RclpyROS2Backend()
        self.resource_id = (
            resource_id
            or os.getenv("FLYTO_ROS2_RESOURCE_ID", "").strip()
            or "ros2-resource"
        )
        self._places_store = places_store
        self._declared: set[str] = set()
        # Per call id, bounded like the backend's results: the observation
        # before a motion, the result once evidence was added to it, and the
        # place a navigation resolved to (a resumed call keeps its target even
        # if the place was moved since).
        self._before: dict[str, Any] = {}
        self._provided: dict[str, CallResult] = {}
        self._targets: dict[str, places.Place] = {}
        # Per call id: the escape decided before a navigation first moved (a
        # resumed call keeps it rather than re-deciding half way out), and the
        # escape leg running now, so a cancel of the call reaches it.
        self._escapes: dict[str, inflation_escape.EscapeDecision] = {}
        self._escape_legs: dict[str, str] = {}
        self._escape_starts: dict[str, dict[str, Any]] = {}
        self._cancelled_escapes: dict[str, bool] = {}

    @property
    def places_store(self) -> places.PlacesStore:
        if self._places_store is None:
            self._places_store = places.PlacesStore.for_resource(self.resource_id)
        return self._places_store

    def _discovered_capabilities(self) -> set[str]:
        interfaces = {(item.kind, item.name, item.type) for item in self.backend.discover()}
        available: set[str] = set()
        for capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if capability_id == "motion.halt":
                if any(
                    item_kind == "topic"
                    and item_name == name
                    and item_type in CMD_VEL_TYPES
                    for item_kind, item_name, item_type in interfaces
                ):
                    available.add(capability_id)
                continue
            if (kind, name, interface_type) in interfaces:
                available.add(capability_id)
        if "motion.navigate" in available:
            available |= PLACE_CAPABILITIES
        return available

    def describe(self) -> Sequence[decl.CapabilityDeclaration]:
        available = self._discovered_capabilities()
        self._declared = available
        declarations: list[decl.CapabilityDeclaration] = []
        for capability_id in sorted(available):
            kind, name, interface_type = (
                HOST_INTERFACES.get(capability_id) or DEFAULT_INTERFACES[capability_id]
            )
            basis = _safety_basis()
            observations = (
                f"{os.getenv('FLYTO_ROS2_ODOM_TOPIC', '/odom')}:{ODOM_TYPE}",
                # Shown when the capability is approved: what this robot's
                # motion safety rests on.
                OPERATOR_PRESENT_OBSERVATION
                if basis == SAFETY_BASIS_OPERATOR
                else f"{os.getenv('FLYTO_ROS2_SCAN_TOPIC', '/scan')}:{SCAN_TYPE}",
            )
            if capability_id == PLACES_LIST:
                observations = ()
            elif capability_id == PLACES_MARK:
                observations = (
                    f"{os.getenv('FLYTO_ROS2_ODOM_TOPIC', '/odom')}:{ODOM_TYPE}",
                    f"{os.getenv('FLYTO_ROS2_TF_TOPIC', '/tf')}:tf2_msgs/msg/TFMessage",
                )
            declarations.append(
                decl.declare(
                    capability_id=capability_id,
                    resource_id=self.resource_id,
                    executor_kind=decl.EXECUTOR_EXTERNAL_API,
                    source=decl.SOURCE_DEVICE,
                    arguments=ARGUMENTS[capability_id],
                    required_observations=observations,
                    runtime_name=f"{kind}:{name}:{interface_type}",
                    version="1.0.0",
                )
            )
        return declarations

    def _motion_preflight(self, capability_id: str) -> str | None:
        if capability_id == "motion.halt":
            return None
        mismatch = self._deployment_mismatch()
        if mismatch is not None:
            return mismatch
        basis = _safety_basis()
        if basis not in SAFETY_BASES:
            return f"FLYTO_ROS2_SAFETY_BASIS {basis!r} is not one of {', '.join(SAFETY_BASES)}"
        if capability_id in PLANNED_MOTIONS and basis != SAFETY_BASIS_OPERATOR:
            # Wait for the map->odom transform as well, so a navigate on a
            # fresh connection is judged once it arrives instead of refused
            # because LiDAR happened to come first.
            observation = self.backend.observation(
                required=(REQUIRE_POSE, REQUIRE_RANGE, REQUIRE_MAP_TF)
            )
        else:
            observation = self.backend.observation()
        if observation.get("pose") is None or (
            basis != SAFETY_BASIS_OPERATOR and observation.get("range") is None
        ):
            # Readings did not arrive in time: the drivers may have restarted
            # behind the transport, so the next call reads the graph afresh.
            self._invalidate_discovery()
        if observation.get("pose") is None:
            return "fresh odometry is required before motion"
        if basis == SAFETY_BASIS_OPERATOR:
            return None
        range_observation = observation.get("range")
        if not isinstance(range_observation, Mapping):
            return "fresh LiDAR is required before motion"
        try:
            clearance = float(range_observation["minimum_range_m"])
        except (KeyError, TypeError, ValueError):
            return "valid LiDAR clearance is required before motion"
        required_clearance = _clearance_floor()
        if clearance < required_clearance:
            return (
                f"LiDAR clearance {clearance:.3f}m is below the "
                f"{required_clearance:.3f}m motion safety minimum"
            )
        if capability_id in PLANNED_MOTIONS and not observation.get(
            "map_tf_available", False
        ):
            return "fresh map-to-odom transform is required before navigation"
        return None

    def _deployment_mismatch(self) -> str | None:
        """Refuse motion when the graph is not the kind of robot configured.

        The physical robot and its simulator expose the same endpoint
        (ws://127.0.0.1:19090), so the configured deployment mode alone cannot
        tell them apart. Without this check a command meant for the simulator
        could drive the real robot.
        """
        marker = _simulation_marker_topic()
        if not marker:
            return None
        mismatch = self._deployment_mismatch_in(self.backend.discover(), marker)
        if mismatch is None:
            return None
        # The graph may be cached from before a simulator or driver restart;
        # refuse only on what the graph says now.
        self._invalidate_discovery()
        return self._deployment_mismatch_in(self.backend.discover(), marker)

    @staticmethod
    def _deployment_mismatch_in(
        interfaces: Iterable[StandardInterface], marker: str
    ) -> str | None:
        mode = os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware").strip().lower()
        simulated = any(item.kind == "topic" and item.name == marker for item in interfaces)
        if mode == "simulation" and not simulated:
            return (
                f"adapter is configured for simulation but the ROS graph has no {marker}; "
                "it may be physical hardware"
            )
        if mode != "simulation" and simulated:
            return (
                f"the ROS graph publishes {marker} (a simulator) but the adapter is "
                "configured for hardware"
            )
        return None

    def invoke(self, request: CallRequest) -> CallResult:
        if getattr(self.backend, "_presence_only", False):
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail="a presence connection never commands the robot",
            )
        if request.capability_id not in self._declared:
            # A capability the graph did not have before may have appeared
            # since (Nav2 starts once the map loads), so a miss re-reads it.
            self._invalidate_discovery()
            self.describe()
        if request.capability_id not in self._declared:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"{request.capability_id} is not present on the ROS 2 graph",
            )
        if request.deadline_seconds <= 0:
            return CallResult(request.call_id, OUTCOME_TIMEOUT, detail="no time to run")
        try:
            arguments = _numeric_arguments(request.capability_id, request.arguments)
        except ValueError as error:
            return CallResult(request.call_id, OUTCOME_REFUSED, detail=str(error))
        if request.capability_id == PLACES_LIST:
            return self._list_places(request.call_id)
        if request.capability_id == PLACES_MARK:
            return self._mark_place(request.call_id, arguments["place"])
        place: places.Place | None = None
        if request.capability_id == "motion.navigate" and "place" in arguments:
            # Resolved before anything else: an unknown place, or a places
            # file that cannot be read, refuses with no motion.
            resolved = self._resolve_target(request.call_id, arguments["place"])
            if isinstance(resolved, CallResult):
                return resolved
            place = resolved
            arguments = {"x": place.x, "y": place.y, "yaw_radians": place.yaw}
        if request.capability_id in CAPTURE_CAPABILITIES:
            capture = getattr(self.backend, "capture", None)
            if not callable(capture):
                return CallResult(
                    request.call_id,
                    OUTCOME_REFUSED,
                    detail=f"{request.capability_id} needs the rosbridge transport",
                )
            return _with_artifacts(
                capture(
                    call_id=request.call_id,
                    capability_id=request.capability_id,
                    deadline_seconds=float(request.deadline_seconds),
                )
            )
        supervised = _safety_basis() == SAFETY_BASIS_OPERATOR
        if supervised and request.capability_id != "motion.halt":
            arguments, bound_error = _supervised_arguments(request.capability_id, arguments)
            if bound_error is not None:
                return CallResult(request.call_id, OUTCOME_REFUSED, detail=bound_error)
        preflight_error = self._motion_preflight(request.capability_id)
        if preflight_error is not None:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=preflight_error,
            )
        if request.capability_id == "motion.halt":
            return self.backend.safe_stop(request.call_id)
        kept = self._provided.get(request.call_id)
        if kept is not None:
            # The same call again: its result, not a second motion's evidence.
            return kept
        escape: inflation_escape.EscapeDecision | None = None
        if request.capability_id == "motion.navigate" and not supervised:
            escape = self._escape_decision(request.call_id)
            if escape is not None and escape.pinned and not escape.feasible:
                # Nav2 would thrash in the inflation for minutes; say why now.
                return CallResult(
                    request.call_id,
                    OUTCOME_REFUSED,
                    evidence={
                        "navigation_escape": escape.to_dict(),
                        "reason_code": inflation_escape.REASON_NO_ESCAPE_ROOM,
                        **_target_evidence(arguments, place),
                    },
                    detail=escape.describe(),
                )
        if request.capability_id in STRAIGHT_MOTIONS and not supervised:
            braked = self._arm_braking(request.call_id, request.capability_id, arguments)
            if isinstance(braked, CallResult):
                return braked
            arguments = braked
        before = self._before.get(request.call_id)
        if before is None:
            # A call resumed after a timeout keeps the observation from before
            # it first moved, not one taken half way.
            before = self._observe_quietly("before", request.call_id)
            _remember(self._before, request.call_id, before)
        if escape is not None and escape.pinned:
            result = self._navigate_with_escape(
                request.call_id, arguments, escape, float(request.deadline_seconds)
            )
        else:
            result = self.backend.invoke(
                call_id=request.call_id,
                capability_id=request.capability_id,
                arguments=arguments,
                deadline_seconds=float(request.deadline_seconds),
            )
        if request.capability_id == "motion.navigate":
            extra = _target_evidence(arguments, place)
            if escape is not None:
                extra["navigation_escape"] = {
                    **escape.to_dict(),
                    **dict((result.evidence or {}).get("navigation_escape") or {}),
                }
            result = CallResult(
                result.call_id,
                result.outcome,
                evidence={**dict(result.evidence or {}), **extra},
                detail=result.detail,
            )
        return self._with_motion_evidence(request.capability_id, before, result)

    # -- braking envelope for straight drives ---------------------------------------

    def _arm_braking(
        self, call_id: str, capability_id: str, arguments: Mapping[str, Any]
    ) -> dict[str, Any] | CallResult:
        """Cap a straight drive's speed by its room and have the backend guard it.

        The speed is the requested one, or lower: the fastest that can still
        come to rest at the clearance floor from the room the drive starts
        with, less a margin. A drive that cannot be given even the minimum
        speed is refused before it moves.
        """
        arm = getattr(self.backend, "arm_braking_guard", None)
        profile_of = getattr(self.backend, "braking_profile", None)
        if not callable(arm) or not callable(profile_of):
            return dict(arguments)
        profile = profile_of()
        observation = self.backend.observation()
        reading = observation.get("range")
        sweep = reading.get("sweep") if isinstance(reading, Mapping) else None
        room = braking.room_to_floor(sweep, STRAIGHT_MOTIONS[capability_id], profile.floor_m)
        if room is None:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="the LiDAR sweep cannot be read, so the room to stop cannot be judged",
            )
        requested = float(
            arguments.get("speed_mps", DEFAULT_STRAIGHT_SPEED_MPS[capability_id])
        )
        allowed = profile.max_speed(room.room_m) * SLOWDOWN_FRACTION
        speed = min(requested, allowed)
        if speed < braking.MIN_SPEED_MPS:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                evidence={
                    "reason_code": motion_outcome.REASON_OBSTACLE_BLOCKED,
                    "braking": {"profile": profile.to_dict(), **room.to_dict()},
                },
                detail=(
                    f"only {room.room_m:.3f} m of room before the {profile.floor_m:.3f} m "
                    "clearance floor the way the robot would drive; it could not stop "
                    f"there from even {braking.MIN_SPEED_MPS:.2f} m/s"
                ),
            )
        arm(
            call_id,
            capability_id,
            profile=profile,
            speed_mps=speed,
            requested_speed_mps=requested,
            distance_m=float(arguments["distance_m"]),
            start_room_m=room.room_m,
        )
        return {**arguments, "speed_mps": speed}

    # -- leaving an obstacle's inflation before navigating -------------------------

    def _escape_decision(self, call_id: str) -> inflation_escape.EscapeDecision | None:
        """Whether this navigation starts inside inflation, decided once per call.

        None when the check is switched off or the observation carries no
        LiDAR sweep to judge it by; the navigation then goes to Nav2 as sent.
        """
        kept = self._escapes.get(call_id)
        if kept is not None:
            return kept
        if not _escape_enabled():
            return None
        observation = self.backend.observation(
            required=(REQUIRE_POSE, REQUIRE_RANGE, REQUIRE_MAP_TF)
        )
        reading = observation.get("range")
        sweep = reading.get("sweep") if isinstance(reading, Mapping) else None
        if not isinstance(sweep, Mapping) or not isinstance(observation.get("map_pose"), Mapping):
            return None
        reader = getattr(self.backend, "get_parameter", None)
        geometry = inflation_escape.read_costmap_geometry(
            reader if callable(reader) else None, _costmap_nodes()
        )
        can_back_off = "motion.retreat" in self._declared
        decision = inflation_escape.plan_escape(
            sweep,
            geometry,
            floor_m=_clearance_floor(),
            max_backoff_m=(
                _env_metres("FLYTO_ROS2_ESCAPE_MAX_BACKOFF_M", 0.30, low=0.0, high=1.0)
                if can_back_off
                else 0.0
            ),
            margin_m=_env_metres("FLYTO_ROS2_ESCAPE_MARGIN_M", 0.10, low=0.0, high=1.0),
            max_lateral_m=_env_metres("FLYTO_ROS2_ESCAPE_MAX_LATERAL_M", 1.0, low=0.1, high=3.0),
        )
        if decision is None:
            return None
        if decision.pinned:
            logger.info("navigation %s starts pinned: %s", call_id, decision.describe())
        _remember(self._escapes, call_id, decision)
        # Kept with the decision: the escape's waypoint is placed from this pose.
        _remember(self._escape_starts, call_id, dict(observation["map_pose"]))
        return decision

    def _escape_leg(
        self,
        call_id: str,
        leg_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline: float,
    ) -> CallResult:
        if call_id in self._cancelled_escapes:
            return CallResult(leg_id, OUTCOME_CANCELLED, detail="cancelled")
        self._escape_legs[call_id] = leg_id
        try:
            return self.backend.invoke(
                call_id=leg_id,
                capability_id=capability_id,
                arguments=arguments,
                deadline_seconds=max(0.0, deadline - time.monotonic()),
            )
        finally:
            self._escape_legs.pop(call_id, None)

    def _navigate_with_escape(
        self,
        call_id: str,
        target: Mapping[str, Any],
        decision: inflation_escape.EscapeDecision,
        deadline_seconds: float,
    ) -> CallResult:
        """Back off, pass the lateral waypoint, then go to the original goal.

        The original goal stays the final pose under this call's own id, so
        its result, and the arrival judged on ``map_pose`` against the target,
        are the real target's. An escape leg that does not complete ends the
        call with that leg's outcome and says which leg it was.
        """
        deadline = time.monotonic() + deadline_seconds
        start = self._escape_starts.get(call_id) or {}
        waypoint = inflation_escape.waypoint_pose(
            start, decision, (float(target["x"]), float(target["y"]))
        )
        legs: list[dict[str, Any]] = []

        def ended(leg: str, result: CallResult) -> CallResult:
            legs.append({"leg": leg, "outcome": result.outcome})
            outcome = result.outcome if result.outcome != OUTCOME_REFUSED else OUTCOME_FAILED
            return CallResult(
                call_id,
                outcome,
                evidence={
                    **dict(result.evidence or {}),
                    "navigation_escape": {"waypoint_map": waypoint, "legs": legs},
                },
                detail=f"inflation escape {leg} {result.outcome}: {result.detail}".strip(),
            )

        if decision.backoff_m > 0.0:
            result = self._escape_leg(
                call_id,
                f"{call_id}:escape-backoff",
                "motion.retreat",
                {"distance_m": decision.backoff_m, "speed_mps": ESCAPE_BACKOFF_SPEED_MPS},
                deadline,
            )
            if result.outcome != OUTCOME_COMPLETED:
                return ended("backoff", result)
            legs.append({"leg": "backoff", "outcome": result.outcome})
        # Two NavigateToPose legs, not one NavigateThroughPoses goal: a stock
        # Nav2 configuration can route through-poses goals to a behaviour tree
        # that only reads a single goal (TurtleBot3 Jazzy did: the planner got an
        # empty pose and the goal was refused in under a second), while
        # NavigateToPose is served by every Nav2.
        result = self._escape_leg(
            call_id, f"{call_id}:escape-waypoint", "motion.navigate", waypoint, deadline
        )
        if result.outcome != OUTCOME_COMPLETED:
            return ended("waypoint", result)
        legs.append({"leg": "waypoint", "outcome": result.outcome})
        if call_id in self._cancelled_escapes:
            return ended("goal", CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled"))
        result = self.backend.invoke(
            call_id=call_id,
            capability_id="motion.navigate",
            arguments=target,
            deadline_seconds=max(0.0, deadline - time.monotonic()),
        )
        legs.append({"leg": "goal", "outcome": result.outcome})
        return CallResult(
            result.call_id,
            result.outcome,
            evidence={
                **dict(result.evidence or {}),
                "navigation_escape": {"waypoint_map": waypoint, "legs": legs},
            },
            detail=result.detail,
        )

    # -- named places (host-side, never on the robot) ---------------------------

    def _resolve_target(self, call_id: str, name: str) -> places.Place | CallResult:
        kept = self._targets.get(call_id)
        if kept is not None:
            return kept
        try:
            place = self.places_store.resolve(name)
        except places.UnknownPlace as error:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                evidence={"known_places": list(error.known)},
                detail=str(error),
            )
        except places.PlacesStoreError as error:
            return CallResult(call_id, OUTCOME_REFUSED, detail=str(error))
        _remember(self._targets, call_id, place)
        return place

    def _list_places(self, call_id: str) -> CallResult:
        try:
            listed = [place.to_dict() for place in self.places_store.list()]
        except places.PlacesStoreError as error:
            return CallResult(call_id, OUTCOME_FAILED, detail=str(error))
        return CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "places": listed,
                "map_id": self.places_store.map_id,
                "artifacts": [_places_artifact(listed)],
            },
            detail=f"{len(listed)} places on map {self.places_store.map_id}",
        )

    def _mark_place(self, call_id: str, name: str) -> CallResult:
        """Save the robot's map-frame pose (the same ``map_pose`` arrival reads)."""
        kept = self._provided.get(call_id)
        if kept is not None:
            # The same call again: its result, not the pose it is at now.
            return kept
        observation = self.backend.observation(required=(REQUIRE_POSE, REQUIRE_MAP_TF))
        map_pose = observation.get("map_pose")
        if not isinstance(map_pose, Mapping):
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail=(
                    "a fresh map-frame pose is required to mark a place; "
                    "localization's map-to-odom transform is missing or stale"
                ),
            )
        try:
            place, replaced = self.places_store.mark(name, map_pose)
        except places.PlacesError as error:
            return CallResult(call_id, OUTCOME_REFUSED, detail=str(error))
        except places.PlacesStoreError as error:
            return CallResult(call_id, OUTCOME_FAILED, detail=str(error))
        except OSError as error:
            return CallResult(
                call_id, OUTCOME_FAILED, detail=f"could not save the place: {type(error).__name__}"
            )
        result = CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "place": place.to_dict(),
                "replaced": replaced.to_dict() if replaced is not None else None,
                "map_id": self.places_store.map_id,
            },
            detail=f"saved {place.name!r} on map {self.places_store.map_id}",
        )
        _remember(self._provided, call_id, result)
        return result

    def _observe_quietly(self, phase: str, call_id: str) -> dict[str, Any] | None:
        """An observation for evidence; a reading that fails costs the item, not the call."""
        try:
            return self.observe(phase=phase, execution_id=call_id)
        except Exception:  # noqa: BLE001 - evidence is best effort, motion is not
            logger.warning("could not observe %s for %s", phase, call_id, exc_info=True)
            return None

    def _with_motion_evidence(
        self,
        capability_id: str,
        before: Mapping[str, Any] | None,
        result: CallResult,
    ) -> CallResult:
        """The motion's result with the evidence this adapter can vouch for.

        Additive: every key the backend reported stays. ``evidence_items``
        carries the clearance the motion started with and, for a completed
        motion, the odometry before, after and once settled (only when the
        robot can say it has settled; otherwise the host observes it, as
        before). A motion that stopped short also carries
        ``recovery_context``. Nothing here waits on a failed motion: the
        host's safe stop comes next.
        """
        if result.outcome == OUTCOME_REFUSED:
            self._before.pop(result.call_id, None)
            return result
        evidence = dict(result.evidence or {})
        items: list[dict[str, Any]] = []
        clearance = provider_evidence.clearance_item(before, _clearance_floor())
        if clearance is not None:
            items.append(clearance)
        if result.outcome == OUTCOME_COMPLETED:
            arrival = self._arrival(capability_id, before, result.call_id)
            if arrival is not None:
                items.append(arrival)
        elif result.outcome in {OUTCOME_FAILED, OUTCOME_TIMEOUT}:
            held = getattr(self.backend, "held_observation", None)
            reading = held().get("range") if callable(held) else None
            context = provider_evidence.recovery_context(
                evidence.get("motion_outcome"),
                sweep=reading.get("sweep") if isinstance(reading, Mapping) else None,
            )
            if context is not None:
                evidence["recovery_context"] = context
        if items:
            evidence["evidence_items"] = items
        provided = CallResult(
            result.call_id, result.outcome, evidence=evidence, detail=result.detail
        )
        if result.outcome != OUTCOME_TIMEOUT:
            # A timed-out goal is still running and may be waited on again.
            self._before.pop(result.call_id, None)
            _remember(self._provided, result.call_id, provided)
        return provided

    def _arrival(
        self, capability_id: str, before: Mapping[str, Any] | None, call_id: str
    ) -> dict[str, Any] | None:
        after = self._observe_quietly("after", call_id)
        if capability_id not in provider_evidence.JUDGED_MOTIONS:
            return provider_evidence.arrival_item(
                capability_id, before=before, after=after, settled=None
            )
        if before is None or after is None:
            return None
        # Settled means the robot said so. One that cannot tell leaves the
        # settled observation to the host, which waits its own way.
        if not isinstance(self.wait_until_stationary(SETTLE_SECONDS), Mapping):
            return None
        settled = self._observe_quietly("post_stop", call_id)
        if settled is None:
            return None
        return provider_evidence.arrival_item(
            capability_id, before=before, after=after, settled=settled
        )

    def cancel(self, call_id: str) -> CallResult:
        leg = self._escape_legs.get(call_id)
        if call_id in self._escapes:
            # Between legs nothing is active; this stops the next one starting.
            _remember(self._cancelled_escapes, call_id, True)
        if leg is not None:
            withdrawn = self.backend.cancel(leg)
            if withdrawn.outcome == OUTCOME_CANCELLED:
                return CallResult(
                    call_id, OUTCOME_CANCELLED, detail="cancelled during inflation escape"
                )
        return self.backend.cancel(call_id)

    def safe_stop(self) -> CallResult:
        return self.backend.safe_stop(f"safe-stop-{time.monotonic_ns()}")

    def execution_count(self, call_id: str) -> int:
        return self.backend.execution_count(call_id)

    def observe(
        self,
        *,
        phase: str = "preflight",
        execution_id: str | None = None,
        deployment_mode: str | None = None,
        provider: str | None = None,
    ) -> dict[str, Any]:
        interfaces = tuple(self.backend.discover())
        observation = dict(self.backend.observation())
        mode = (
            deployment_mode
            or os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
        ).strip().lower()
        source = provider or ("gazebo" if mode == "simulation" else "ros2")

        def bundle(range_observation):
            return build_observation_bundle(
                resource_id=self.resource_id,
                runtime_snapshot=runtime_snapshot(interfaces),
                deployment_mode=mode,
                provider=source,
                phase=phase,
                execution_id=execution_id,
                pose=observation.get("pose"),
                map_pose=observation.get("map_pose"),
                range_observation=range_observation,
                camera=observation.get("camera"),
                map_tf_available=bool(observation.get("map_tf_available", False)),
            )

        reading = observation.get("range")
        try:
            return bundle(reading)
        except Ros2ObservationError:
            # The sweep is for a person to look at. One the contract refuses
            # must not fail the observation a motion check reads.
            if not isinstance(reading, Mapping) or "sweep" not in reading:
                raise
            return bundle({key: value for key, value in reading.items() if key != "sweep"})

    def wait_until_stationary(self, max_seconds: float = 1.0) -> dict[str, Any] | None:
        """Wait for odometry to show the robot stopped, at most ``max_seconds``.

        None when the backend cannot tell, so a host falls back to its own wait.
        """
        method = getattr(self.backend, "wait_until_stationary", None)
        if not callable(method):
            return None
        return method(max_seconds)

    def silent_seconds(self) -> float | None:
        """Seconds since the robot last sent a reading; None if unknown."""
        method = getattr(self.backend, "silent_seconds", None)
        if not callable(method):
            return None
        value = method()
        return None if value is None else float(value)

    @property
    def connected(self) -> bool:
        state = getattr(self.backend, "is_connected", None)
        if callable(state):
            return bool(state())
        return bool(getattr(self.backend, "_connected", True))

    def add_connection_listener(self, listener: Callable[[bool], None]) -> None:
        """Call ``listener(connected)`` whenever the transport drops or returns."""
        method = getattr(self.backend, "add_connection_listener", None)
        if callable(method):
            method(listener)

    def add_graph_listener(self, listener: Callable[[str], None]) -> None:
        """Call ``listener(reason)`` when the robot's graph may have changed."""
        method = getattr(self.backend, "add_graph_listener", None)
        if callable(method):
            method(listener)

    def _invalidate_discovery(self) -> None:
        method = getattr(self.backend, "invalidate_discovery", None)
        if callable(method):
            method()

    def disconnect(self) -> None:
        method = getattr(self.backend, "disconnect", None)
        if callable(method):
            method()

    def reconnect(self) -> None:
        method = getattr(self.backend, "reconnect", None)
        if callable(method):
            method()


class RclpyROS2Backend(_ObservationState):
    """rclpy client for standard Nav2 actions and observation topics.

    A background executor delivers callbacks for the adapter's whole life, so
    readings, goal acceptance and results arrive as they happen. No caller
    spins the node: rclpy forbids spinning one node from two threads, and a
    spin to a fixed deadline costs that deadline whether or not anything came.
    """

    def __init__(self, *, presence_only: bool = False) -> None:
        # See RosbridgeROS2Backend: odometry and lifecycle events only.
        self._presence_only = bool(presence_only)
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image, LaserScan
            from tf2_msgs.msg import TFMessage
        except ImportError as error:
            raise RuntimeError(
                "rclpy/nav2 messages are required on the external ROS 2 adapter host"
            ) from error

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node_type = Node
        self._message_types = (Odometry, LaserScan, Image, CameraInfo, TFMessage)
        self._sensor_qos = qos_profile_sensor_data
        self._executor_type = SingleThreadedExecutor
        self._executor: Any | None = None
        self._executor_thread: threading.Thread | None = None
        self._connected = True
        self._init_observation_state()
        # rmw fills a new node's graph cache as DDS discovery runs; it is known
        # to have caught up once the first subscribed message arrives.
        self._graph_ready = False
        self._goal_handles: dict[str, Any] = {}
        self._result_futures: dict[str, Any] = {}
        self._results: dict[str, CallResult] = {}
        self._counts: dict[str, int] = {}
        self._publishers: dict[str, Any] = {}
        # Kept per capability: a fresh ActionClient has not matched its server
        # yet, and rclpy's wait_for_server then polls in 0.25 s steps.
        self._action_clients: dict[str, Any] = {}
        self._node: Any | None = None
        self._create_node()

    def _create_node(self) -> None:
        """A node with the standard observation subscriptions."""
        odometry, scan, image, camera_info, tf_message = self._message_types
        node = self._node_type(
            "flyto_external_generic_ros2_presence"
            if self._presence_only
            else "flyto_external_generic_ros2_adapter"
        )
        node.create_subscription(
            odometry,
            os.getenv("FLYTO_ROS2_ODOM_TOPIC", "/odom"),
            self._guarded(self._on_odometry),
            self._sensor_qos,
        )
        self._subscribe_lifecycle(node)
        if self._presence_only:
            self._node = node
            self._graph_ready = False
            self._start_executor()
            return
        node.create_subscription(
            scan,
            os.getenv("FLYTO_ROS2_SCAN_TOPIC", "/scan"),
            self._guarded(self._on_scan),
            self._sensor_qos,
        )
        node.create_subscription(
            image,
            os.getenv("FLYTO_ROS2_CAMERA_TOPIC", "/camera/image_raw"),
            self._guarded(self._on_camera),
            self._sensor_qos,
        )
        node.create_subscription(
            camera_info,
            os.getenv("FLYTO_ROS2_CAMERA_INFO_TOPIC", "/camera/camera_info"),
            self._guarded(self._on_camera_info),
            self._sensor_qos,
        )
        node.create_subscription(
            tf_message,
            os.getenv("FLYTO_ROS2_TF_TOPIC", "/tf"),
            self._guarded(self._on_tf),
            10,
        )
        self._subscribe_collision_state(node)
        self._node = node
        self._graph_ready = False
        self._create_action_clients(node)
        self._start_executor()

    def _subscribe_lifecycle(self, node: Any) -> None:
        topics = _lifecycle_event_topics()
        if not topics:
            return
        try:
            from lifecycle_msgs.msg import TransitionEvent
        except ImportError:
            return
        for topic in topics:

            def changed(_message: Any, topic: str = topic) -> None:
                with self._condition:
                    self._graph_changed(f"lifecycle:{topic}")

            node.create_subscription(TransitionEvent, topic, changed, 10)

    def _subscribe_collision_state(self, node: Any) -> None:
        topic = _collision_state_topic()
        if not topic:
            return
        try:
            from nav2_msgs.msg import CollisionMonitorState
        except ImportError:
            return
        node.create_subscription(
            CollisionMonitorState,
            topic,
            self._guarded(
                lambda message: self._track_collision_state(
                    message.action_type, message.polygon_name
                )
            ),
            10,
        )

    def _create_action_clients(self, node: Any) -> None:
        """Create the motion action clients with the node.

        A client matches its server during DDS discovery, alongside the
        subscriptions, so by the first goal it is usually ready. Created at
        the first goal instead, it had not matched yet, and rclpy's
        wait_for_server then polls for it.
        """
        try:
            from rclpy.action import ActionClient
        except ImportError:
            return
        motions = ("motion.navigate", "motion.advance", "motion.retreat", "motion.rotate")
        for capability_id in motions:
            if capability_id in self._action_clients:
                continue
            try:
                self._action_clients[capability_id] = ActionClient(
                    node,
                    self._action_type(capability_id),
                    DEFAULT_INTERFACES[capability_id][1],
                )
            except Exception:  # noqa: BLE001 - created on first use instead
                logger.debug("ROS 2 action client for %s deferred", capability_id, exc_info=True)

    def _destroy_node(self) -> None:
        """Release the node with its subscriptions, publishers and clients.

        Each discovery pass on an execution host builds an adapter and
        disconnects it; a node left behind kept its subscriptions and DDS
        participant alive for the life of the process.
        """
        node, self._node = self._node, None
        self._publishers.clear()
        self._action_clients.clear()
        if node is not None:
            with contextlib.suppress(Exception):
                node.destroy_node()

    def _start_executor(self) -> None:
        if self._executor_thread is not None and self._executor_thread.is_alive():
            return
        executor = self._executor_type()
        executor.add_node(self._node)
        thread = threading.Thread(
            target=self._spin_executor,
            args=(executor,),
            name="ros2-adapter-executor",
            daemon=True,
        )
        self._executor = executor
        self._executor_thread = thread
        thread.start()

    def _spin_executor(self, executor: Any) -> None:
        try:
            executor.spin()
        except Exception as error:  # noqa: BLE001 - reported as a dropped connection
            if executor is self._executor:
                logger.warning("ROS 2 adapter executor stopped: %s", type(error).__name__)
        if executor is not self._executor:
            # Stopped on purpose by disconnect() or reconnect().
            return
        # Nothing delivers readings or goal results any more. Say so, so a
        # host keeping this adapter warm reconnects it now instead of handing
        # every job an adapter whose readings only go stale.
        with self._condition:
            self._connected = False
            self._condition.notify_all()
        self._notify_connection(False)

    def is_connected(self) -> bool:
        thread = self._executor_thread
        return bool(self._connected) and thread is not None and thread.is_alive()

    def _guarded(self, callback: Callable[[Any], None]) -> Callable[[Any], None]:
        """One malformed message is dropped; it must not end the executor."""

        def deliver(message: Any) -> None:
            try:
                callback(message)
            except Exception as error:  # noqa: BLE001 - drop the bad message only
                logger.debug(
                    "ROS 2 message dropped by %s: %s",
                    getattr(callback, "__name__", "callback"),
                    type(error).__name__,
                )

        return deliver

    def _stop_executor(self) -> None:
        executor, thread = self._executor, self._executor_thread
        self._executor = None
        self._executor_thread = None
        if executor is not None:
            with contextlib.suppress(Exception):
                executor.shutdown()
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)

    def _on_odometry(self, message: Any) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        twist = getattr(getattr(message, "twist", None), "twist", None)
        velocity = (
            (float(twist.linear.x), float(twist.angular.z)) if twist is not None else None
        )
        self._store_odometry(
            {
                "frame": "odom",
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": float(yaw),
            },
            velocity,
            time.monotonic(),
        )

    def _on_scan(self, message: Any) -> None:
        usable = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
            and float(value) >= float(message.range_min)
            and float(value) <= float(message.range_max)
        ]
        if usable:
            sweep = scan_sweep(
                list(message.ranges),
                angle_min=message.angle_min,
                angle_increment=message.angle_increment,
                range_min=message.range_min,
                range_max=message.range_max,
            )
            stamp = getattr(getattr(message, "header", None), "stamp", None)
            stamp_s = (
                float(stamp.sec) + float(stamp.nanosec) * 1e-9 if stamp is not None else None
            )
            with self._condition:
                self._minimum_range = min(usable)
                self._track_range(self._minimum_range)
                self._range_sample_count = len(usable)
                self._range_sweep = sweep
                self._range_seen_at = time.monotonic()
                self._note_scan_timing(self._range_seen_at, stamp_s)
                self._check_guards()
                self._condition.notify_all()

    def _on_camera(self, message: Any) -> None:
        payload = bytes(message.data)
        frame_seed = (
            str(message.encoding).encode("utf-8")
            + b"|"
            + str(int(message.width)).encode("ascii")
            + b"x"
            + str(int(message.height)).encode("ascii")
            + b"|"
            + payload
        )
        with self._condition:
            self._camera = {
                "encoding": str(message.encoding),
                "width": int(message.width),
                "height": int(message.height),
                "calibrated": self._camera_calibration_snapshot is not None,
                "calibration_snapshot": self._camera_calibration_snapshot,
                "frame_snapshot": hashlib.sha256(frame_seed).hexdigest(),
            }
            self._camera_seen_at = time.monotonic()
            self._condition.notify_all()

    def _on_camera_info(self, message: Any) -> None:
        matrix = tuple(float(value) for value in message.k)
        calibrated = len(matrix) == 9 and matrix[0] != 0.0
        if not calibrated:
            self._camera_calibration_snapshot = None
            if self._camera is not None:
                self._camera["calibrated"] = False
                self._camera["calibration_snapshot"] = None
            return
        seed = repr(
            (
                str(message.distortion_model),
                tuple(float(value) for value in message.d),
                matrix,
                tuple(float(value) for value in message.r),
                tuple(float(value) for value in message.p),
                int(message.width),
                int(message.height),
            )
        ).encode("utf-8")
        self._camera_calibration_snapshot = hashlib.sha256(seed).hexdigest()
        if self._camera is not None:
            self._camera["calibrated"] = True
            self._camera["calibration_snapshot"] = self._camera_calibration_snapshot

    def _on_tf(self, message: Any) -> None:
        map_frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
        odom_frame = os.getenv("FLYTO_ROS2_ODOM_FRAME", "odom")
        for item in message.transforms:
            if item.header.frame_id != map_frame or item.child_frame_id != odom_frame:
                continue
            transform = getattr(item, "transform", None)
            planar = (
                map_frame_module.planar_transform(
                    getattr(transform, "translation", None),
                    getattr(transform, "rotation", None),
                )
                if transform is not None
                else None
            )
            observed = time.monotonic()
            with self._condition:
                self._store_map_transform(planar, observed)
            return

    def _await_graph(self) -> None:
        """Wait, on a new node only, until DDS discovery has reached the robot.

        rmw's graph cache starts empty and fills as discovery runs, so reading
        it the moment the node exists reports no equipment at all. The first
        subscribed message proves a robot participant has been matched; the
        wait ends on that callback and is capped by the observation wait.
        """
        if self._graph_ready:
            return
        deadline = time.monotonic() + _observation_wait_seconds()
        with self._condition:
            while self._connected and not self._heard_from_robot():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return
                self._condition.wait(timeout=remaining)
            self._graph_ready = self._heard_from_robot()

    def _heard_from_robot(self) -> bool:
        return self._odom_sequence > 0 or any(
            seen is not None
            for seen in (self._range_seen_at, self._camera_seen_at, self._map_tf_seen_at)
        )

    def discover(self) -> Sequence[StandardInterface]:
        self._await_graph()
        return self._read_graph()

    def _read_graph(self) -> list[StandardInterface]:
        # The graph APIs read rmw's own graph cache, which DDS keeps current
        # without the node being spun, so this needs neither a spin nor a cache.
        node = self._node
        if node is None:
            # Disconnected: the node and its view of the graph are gone.
            return []
        topics = {name: tuple(types) for name, types in node.get_topic_names_and_types()}
        actions = {name: tuple(types) for name, types in node.get_action_names_and_types()}
        found: list[StandardInterface] = []
        for _capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if kind == "action":
                if interface_type in actions.get(name, ()):
                    found.append(StandardInterface(kind, name, interface_type))
                continue
            for topic_type in topics.get(name, ()):
                if topic_type in CMD_VEL_TYPES or topic_type == interface_type:
                    found.append(StandardInterface("topic", name, topic_type))
        marker = _simulation_marker_topic()
        if marker and CLOCK_TYPE in topics.get(marker, ()):
            found.append(StandardInterface("topic", marker, CLOCK_TYPE))
        return found

    def _await_guarded(self, call_id: str, future: Any, deadline: float) -> bool:
        """Wait for a goal's result, stopping it if its braking guard trips.

        This transport only stops a guarded drive; it does not re-send it
        slower (the rosbridge transport does). The drive's starting speed is
        still capped by the room it starts with.
        """
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        while not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return False
            guard = self._guard(call_id)
            if guard is None:
                done.wait(timeout=remaining)
                continue
            self._check_scan_fresh(call_id)
            if guard.tripped is not None and not guard.stop_command_sent:
                guard.stop_command_sent = True
                logger.warning("braking guard stopping %s: %s %s", call_id,
                               guard.tripped, guard.trip)
                with contextlib.suppress(Exception):
                    self._publish_zero()
                handle = self._goal_handles.get(call_id)
                if handle is not None:
                    with contextlib.suppress(Exception):
                        handle.cancel_goal_async()
                with contextlib.suppress(Exception):
                    self._publish_zero()
            done.wait(timeout=min(remaining, 0.05))
        return True

    def _wait_future(self, future: Any, deadline: float) -> bool:
        """Wait for an rclpy future that the background executor completes."""
        if future.done():
            return True
        done = threading.Event()
        future.add_done_callback(lambda _future: done.set())
        done.wait(timeout=max(0.0, deadline - time.monotonic()))
        return future.done()

    def _evidence(self, capability_id: str) -> dict[str, Any]:
        kind, name, interface_type = _interface(capability_id)
        evidence: dict[str, Any] = {
            "adapter": "generic_ros2",
            "standard_interface": {
                "kind": kind,
                "name": name,
                "type": interface_type,
            },
        }
        if self._pose is not None:
            evidence["odom"] = dict(self._pose)
        with self._condition:
            map_pose = self._snapshot().get("map_pose")
        if map_pose is not None:
            evidence["map_pose"] = map_pose
        if self._minimum_range is not None:
            evidence["minimum_range_m"] = self._minimum_range
        return evidence

    def _action_type(self, capability_id: str):
        from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin

        if capability_id == NAVIGATE_THROUGH:
            from nav2_msgs.action import NavigateThroughPoses

            return NavigateThroughPoses
        return {
            "motion.navigate": NavigateToPose,
            "motion.advance": DriveOnHeading,
            "motion.retreat": BackUp,
            "motion.rotate": Spin,
        }[capability_id]

    def action_available(self, capability_id: str) -> bool:
        """Whether the graph offers an action the adapter uses on its own behalf."""
        node = self._node
        if node is None:
            return False
        _, name, interface_type = _interface(capability_id)
        actions = {name: tuple(types) for name, types in node.get_action_names_and_types()}
        return interface_type in actions.get(name, ())

    def get_parameter(self, node_name: str, name: str) -> Mapping[str, Any] | None:
        """One parameter of another node as a ParameterValue mapping, or None."""
        from rcl_interfaces.srv import GetParameters

        node = self._node
        if node is None:
            raise RuntimeError("ROS 2 adapter disconnected")
        client = node.create_client(GetParameters, f"{node_name.rstrip('/')}/get_parameters")
        try:
            if not client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError(f"{node_name} parameters unavailable")
            request = GetParameters.Request()
            request.names = [name]
            future = client.call_async(request)
            if not self._wait_future(future, time.monotonic() + 1.0):
                raise RuntimeError(f"{node_name} parameters timed out")
            values = list(getattr(future.result(), "values", ()) or ())
        finally:
            with contextlib.suppress(Exception):
                node.destroy_client(client)
        if not values:
            return None
        value = values[0]
        return {
            "type": int(value.type),
            "bool_value": bool(value.bool_value),
            "integer_value": int(value.integer_value),
            "double_value": float(value.double_value),
            "string_value": str(value.string_value),
            "string_array_value": list(value.string_array_value),
            "double_array_value": [float(item) for item in value.double_array_value],
        }

    def _goal(self, capability_id: str, arguments: Mapping[str, float]):
        action_type = self._action_type(capability_id)
        goal = action_type.Goal()
        if capability_id == NAVIGATE_THROUGH:
            from geometry_msgs.msg import PoseStamped

            frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
            stamp = self._node.get_clock().now().to_msg()
            for x, y, yaw in _route_poses(arguments):
                pose = PoseStamped()
                pose.header.frame_id = frame
                pose.header.stamp = stamp
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                goal.poses.append(pose)
            return goal
        if capability_id not in PLANNED_MOTIONS:
            allowance = max(
                1.0, float(os.getenv("FLYTO_ROS2_ACTION_ALLOWANCE_SECONDS", "30"))
            )
            goal.time_allowance.sec = int(allowance)
            goal.time_allowance.nanosec = int(
                (allowance - int(allowance)) * 1_000_000_000
            )
        if capability_id in PLANNED_MOTIONS:
            yaw = float(arguments.get("yaw_radians", 0.0))
            goal.pose.header.frame_id = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(arguments["x"])
            goal.pose.pose.position.y = float(arguments["y"])
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        elif capability_id == "motion.rotate":
            goal.target_yaw = float(arguments["yaw_radians"])
        else:
            goal.target.x = float(arguments["distance_m"])
            goal.speed = float(
                arguments.get(
                    "speed_mps",
                    0.10 if capability_id == "motion.retreat" else 0.12,
                )
            )
        return goal

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult:
        if not self._connected:
            return CallResult(
                call_id, OUTCOME_REFUSED, detail="ROS 2 adapter disconnected"
            )
        if call_id in self._results:
            return self._results[call_id]

        from action_msgs.msg import GoalStatus
        from rclpy.action import ActionClient

        if call_id not in self._goal_handles:
            _, action_name, _ = _interface(capability_id)
            client = self._action_clients.get(capability_id)
            if client is None:
                client = ActionClient(
                    self._node, self._action_type(capability_id), action_name
                )
                self._action_clients[capability_id] = client
            # A warm client has matched its server already; rclpy's
            # wait_for_server polls, so it is only asked when that is not so.
            if not client.server_is_ready() and not client.wait_for_server(
                timeout_sec=min(1.0, deadline_seconds)
            ):
                return CallResult(
                    call_id, OUTCOME_REFUSED, detail="ROS 2 action server unavailable"
                )
            started = client.send_goal_async(
                self._goal(capability_id, arguments),
                feedback_callback=self._guarded(
                    lambda message, call_id=call_id: self._track_feedback(
                        call_id, _message_fields(getattr(message, "feedback", None))
                    )
                ),
            )
            accept_deadline = time.monotonic() + min(2.0, deadline_seconds)
            if not self._wait_future(started, accept_deadline):
                return CallResult(
                    call_id, OUTCOME_TIMEOUT, detail="goal acceptance timed out"
                )
            handle = started.result()
            if handle is None or not handle.accepted:
                return CallResult(
                    call_id, OUTCOME_REFUSED, detail="ROS 2 action goal refused"
                )
            self._goal_handles[call_id] = handle
            self._result_futures[call_id] = handle.get_result_async()
            self._count_execution(call_id)
            self._begin_track(call_id, capability_id, arguments)

        future = self._result_futures[call_id]
        deadline = time.monotonic() + deadline_seconds

        if not self._await_guarded(call_id, future, deadline):
            return _motion_result(
                call_id,
                OUTCOME_TIMEOUT,
                self._evidence(capability_id),
                self._motion_summary(call_id, status=None),
                fallback="ROS 2 action still running",
            )
        response = future.result()
        status = int(getattr(response, "status", 0))
        guard = self._guard(call_id)
        braked = guard is not None and guard.tripped is not None
        if braked:
            with contextlib.suppress(Exception):
                self._publish_zero()
            self.wait_until_stationary(1.5)
        summary = self._motion_summary(
            call_id,
            status=status,
            result_values=_message_fields(getattr(response, "result", None)),
        )
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = _motion_result(
                call_id, OUTCOME_COMPLETED, self._evidence(capability_id), summary
            )
        elif braked:
            result = _motion_result(
                call_id,
                OUTCOME_FAILED,
                self._evidence(capability_id),
                summary,
                fallback="stopped by the braking guard",
            )
        elif status == GoalStatus.STATUS_CANCELED:
            result = _motion_result(call_id, OUTCOME_CANCELLED, {}, summary, fallback="cancelled")
        else:
            result = _motion_result(
                call_id,
                OUTCOME_FAILED,
                self._evidence(capability_id),
                summary,
                fallback=f"ROS 2 action status {status}",
            )
        # The goal is over: nothing is left to cancel or wait for.
        self._forget_goal(call_id)
        return self._keep_result(call_id, result)

    def _forget_goal(self, call_id: str) -> None:
        self._goal_handles.pop(call_id, None)
        self._result_futures.pop(call_id, None)
        self._forget_track(call_id)

    def _publish_zero(self) -> bool:
        topic = DEFAULT_INTERFACES["motion.halt"][1]
        # A stop reads the graph as it is; it never waits for discovery.
        types = {
            item.type
            for item in self._read_graph()
            if item.kind == "topic" and item.name == topic
        }
        if len(types) != 1:
            return False
        topic_type = next(iter(types))
        if topic_type == "geometry_msgs/msg/TwistStamped":
            from geometry_msgs.msg import TwistStamped

            publisher = self._publishers.get(topic_type)
            if publisher is None:
                publisher = self._node.create_publisher(TwistStamped, topic, 10)
                self._publishers[topic_type] = publisher
            message = TwistStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = os.getenv("FLYTO_ROS2_BASE_FRAME", "base_link")
        elif topic_type == "geometry_msgs/msg/Twist":
            from geometry_msgs.msg import Twist

            publisher = self._publishers.get(topic_type)
            if publisher is None:
                publisher = self._node.create_publisher(Twist, topic, 10)
                self._publishers[topic_type] = publisher
            message = Twist()
        else:
            return False
        # rmw sends the message on publish; nothing has to spin to flush it.
        publisher.publish(message)
        return True

    def cancel(self, call_id: str) -> CallResult:
        handle = self._goal_handles.get(call_id)
        if handle is None:
            return CallResult(
                call_id, OUTCOME_REFUSED, detail="no active ROS 2 goal"
            )
        future = handle.cancel_goal_async()
        if not self._wait_future(future, time.monotonic() + 2.0):
            return CallResult(
                call_id, OUTCOME_FAILED, detail="ROS 2 cancel timed out"
            )
        response = future.result()
        accepted = bool(getattr(response, "goals_canceling", ()))
        stopped = self._publish_zero()
        if not accepted or not stopped:
            return CallResult(
                call_id, OUTCOME_FAILED, detail="ROS 2 cancel/stop was not confirmed"
            )
        self._forget_goal(call_id)
        return self._keep_result(
            call_id, CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        )

    def safe_stop(self, call_id: str) -> CallResult:
        # Zero velocity goes out first, so the stop never waits behind a cancel.
        stopped = self._publish_zero()
        active = tuple(self._goal_handles)
        for active_call in active:
            future = self._result_futures.get(active_call)
            if future is None or not future.done():
                self.cancel(active_call)
            # Tried once: a goal whose cancel went unanswered (Nav2 restarted)
            # must not cost every later emergency stop another wait.
            self._forget_goal(active_call)
        if active:
            # The action server kept writing cmd_vel until its cancel landed,
            # over the first zero. A server that stops without a zero of its
            # own would leave the base on its last command, so the last word
            # on cmd_vel is this stop's.
            stopped = self._publish_zero() or stopped
        if not stopped:
            return CallResult(
                call_id, OUTCOME_FAILED, detail="standard cmd_vel stop unavailable"
            )
        self._count_execution(call_id)
        result = CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "adapter": "generic_ros2",
                "standard_interface": {
                    "kind": "topic",
                    "name": DEFAULT_INTERFACES["motion.halt"][1],
                    "type": "geometry_msgs/msg/Twist|TwistStamped",
                },
                "zero_velocity_published": True,
            },
        )
        return self._keep_result(call_id, result)

    def execution_count(self, call_id: str) -> int:
        return self._counts.get(call_id, 0)

    def disconnect(self) -> None:
        with self._condition:
            self._connected = False
            self._condition.notify_all()
        self._stop_executor()
        self._destroy_node()
        self._reset_readings()
        self._stop_graph_events()

    def reconnect(self) -> None:
        self._stop_executor()
        self._reset_readings()
        # Goals sent before the drop are no longer followed by this executor.
        self._goal_handles.clear()
        self._result_futures.clear()
        with self._condition:
            self._connected = True
            self._condition.notify_all()
        if self._node is None:
            # Disconnected before: the old node was destroyed with it.
            self._create_node()
        else:
            self._start_executor()
        # An executor that died at once has announced the drop already.
        if self.is_connected():
            self._notify_connection(True)



class RosbridgeROS2Backend(_ObservationState):
    """WebSocket ROS 2 backend for an external AI Space host.

    The robot runs only upstream rosbridge/rosapi. The adapter stays on the
    external computer and speaks the same standard ROS 2 capability contract
    as :class:`RclpyROS2Backend`.
    """

    def __init__(
        self,
        *,
        url: str | None = None,
        connection_factory: Callable[[str], Any] | None = None,
        presence_only: bool = False,
    ) -> None:
        # A presence connection only watches whether the robot is there and
        # what its graph offers (odometry, Nav2 lifecycle events); it never
        # carries LiDAR, camera or transforms, and never moves the robot.
        self._presence_only = bool(presence_only)
        self._lifecycle_topics = frozenset(_lifecycle_event_topics())
        self._url = (
            url
            or os.getenv("FLYTO_ROSBRIDGE_URL", "").strip()
            or "ws://127.0.0.1:19090"
        )
        self._connection_factory = connection_factory or self._connect
        self._ws: Any | None = None
        self._connected = False
        self._init_observation_state()
        self._keepalive_stop = threading.Event()
        # The ROS graph as rosapi last reported it, re-read on reconnect and
        # whenever what it says may be out of date (a call fails, a capability
        # or reading is missing, a deployment check would refuse) rather than
        # on every observation. Nav2 or a driver can restart behind a rosbridge
        # that stays up, so a refusal is never decided on this copy alone.
        self._interfaces: tuple[StandardInterface, ...] | None = None
        self._action_names: frozenset[str] = frozenset()
        self._send_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._responses: dict[str, dict[str, Any]] = {}
        self._action_results: dict[str, dict[str, Any]] = {}
        # One-shot reads for vision.observe and sensing.map: topic -> waiter.
        self._capture_waits: dict[str, str] = {}
        self._captured: dict[str, dict[str, Any]] = {}
        self._active_actions: dict[str, str] = {}
        # Call ids whose invoke() is waiting for the goal's result right now.
        # A cancel or a safe stop on this shared connection reads the result
        # without taking it, so the call that sent the goal still learns how
        # it ended instead of waiting out its whole deadline.
        self._result_waiters: dict[str, int] = {}
        self._results: dict[str, CallResult] = {}
        # A straight drive slowed by its braking guard is re-sent as a new goal
        # that preempts the running one; its result still answers the call.
        # call_id -> the goal id now driving it, and goal id -> call_id.
        self._goal_ids: dict[str, str] = {}
        self._goal_owner: dict[str, str] = {}
        self._counts: dict[str, int] = {}
        self._advertised_topics: set[tuple[str, str]] = set()
        self._topic_types: dict[str, str] = {}
        self.reconnect()

    @staticmethod
    def _connect(url: str) -> Any:
        try:
            from websockets.sync.client import connect
        except ImportError as error:
            raise RuntimeError(
                "websockets>=12 is required for the rosbridge ROS 2 backend"
            ) from error
        return connect(
            url,
            open_timeout=3,
            close_timeout=1,
            max_size=None,
        )

    def _send(self, payload: Mapping[str, Any]) -> None:
        if not self._connected or self._ws is None:
            raise RuntimeError("rosbridge adapter disconnected")
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        with self._send_lock:
            self._ws.send(encoded)

    def _reader_loop(self, websocket: Any) -> None:
        while True:
            with self._condition:
                if not self._connected or websocket is not self._ws:
                    return
            try:
                raw = websocket.recv(timeout=1.0)
            except TimeoutError:
                continue
            except Exception:
                with self._condition:
                    dropped = websocket is self._ws
                    if dropped:
                        self._connected = False
                        self._interfaces = None
                        self._condition.notify_all()
                if dropped:
                    self._notify_connection(False)
                return
            try:
                message = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if not isinstance(message, Mapping):
                continue
            self._handle_message(dict(message))

    def _handle_message(self, message: dict[str, Any]) -> None:
        operation = message.get("op")
        if operation == "publish":
            self._handle_publish(
                str(message.get("topic", "")),
                message.get("msg"),
            )
            return
        identifier = message.get("id")
        if not isinstance(identifier, str):
            return
        with self._condition:
            owner = self._goal_owner.get(identifier, identifier)
            if operation in ("action_result", "action_feedback") and (
                self._goal_ids.get(owner, owner) != identifier
            ):
                # A goal the braking guard has since replaced with a slower
                # one: the server aborted it for the preemption, which says
                # nothing about how the call ends.
                return
            if operation == "service_response":
                self._responses[identifier] = message
                self._condition.notify_all()
            elif operation == "action_result":
                self._action_results[owner] = message
                self._condition.notify_all()
            elif operation == "action_feedback":
                self._track_feedback(owner, message.get("values"))

    @staticmethod
    def _camera_payload(data: Any) -> bytes:
        if isinstance(data, str):
            try:
                return base64.b64decode(data, validate=False)
            except (ValueError, TypeError):
                return data.encode("utf-8", errors="ignore")
        if isinstance(data, list):
            try:
                return bytes(int(value) & 0xFF for value in data)
            except (TypeError, ValueError):
                return b""
        return b""

    def _update_odometry(self, message: Mapping[str, Any], observed: float) -> None:
        pose = message.get("pose")
        nested = pose.get("pose") if isinstance(pose, Mapping) else None
        position = nested.get("position") if isinstance(nested, Mapping) else None
        orientation = nested.get("orientation") if isinstance(nested, Mapping) else None
        if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
            return
        try:
            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            ox = float(orientation.get("x", 0.0))
            oy = float(orientation.get("y", 0.0))
            oz = float(orientation.get("z", 0.0))
            ow = float(orientation.get("w", 1.0))
        except (TypeError, ValueError):
            return
        self._store_odometry(
            {
                "frame": "odom",
                "x": x,
                "y": y,
                "yaw": math.atan2(
                    2.0 * (ow * oz + ox * oy),
                    1.0 - 2.0 * (oy * oy + oz * oz),
                ),
            },
            self._twist(message),
            observed,
        )

    @staticmethod
    def _twist(message: Mapping[str, Any]) -> tuple[float, float] | None:
        outer = message.get("twist")
        twist = outer.get("twist") if isinstance(outer, Mapping) else None
        linear = twist.get("linear") if isinstance(twist, Mapping) else None
        angular = twist.get("angular") if isinstance(twist, Mapping) else None
        if not isinstance(linear, Mapping) or not isinstance(angular, Mapping):
            return None
        try:
            return float(linear.get("x", 0.0)), float(angular.get("z", 0.0))
        except (TypeError, ValueError):
            return None

    def _update_scan(self, message: Mapping[str, Any], observed: float) -> None:
        ranges = message.get("ranges")
        if not isinstance(ranges, list):
            return
        try:
            minimum = float(message.get("range_min", 0.0))
            maximum = float(message.get("range_max", math.inf))
        except (TypeError, ValueError):
            return
        usable: list[float] = []
        for raw in ranges:
            try:
                value = float(raw)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and minimum <= value <= maximum:
                usable.append(value)
        if not usable:
            return
        self._minimum_range = min(usable)
        self._track_range(self._minimum_range)
        self._range_sample_count = len(usable)
        self._range_sweep = scan_sweep(
            ranges,
            angle_min=message.get("angle_min", 0.0),
            angle_increment=message.get("angle_increment", 0.0),
            range_min=minimum,
            range_max=maximum,
        )
        self._range_seen_at = observed
        self._note_scan_timing(observed, _header_stamp(message))
        self._check_guards()
        self._condition.notify_all()

    def _update_camera(self, message: Mapping[str, Any], observed: float) -> None:
        try:
            encoding = str(message["encoding"])
            width = int(message["width"])
            height = int(message["height"])
        except (KeyError, TypeError, ValueError):
            return
        payload = self._camera_payload(message.get("data", ""))
        seed = (
            encoding.encode("utf-8")
            + b"|"
            + str(width).encode("ascii")
            + b"x"
            + str(height).encode("ascii")
            + b"|"
            + payload
        )
        self._camera = {
            "encoding": encoding,
            "width": width,
            "height": height,
            "calibrated": self._camera_calibration_snapshot is not None,
            "calibration_snapshot": self._camera_calibration_snapshot,
            "frame_snapshot": hashlib.sha256(seed).hexdigest(),
        }
        self._camera_seen_at = observed
        self._condition.notify_all()

    def _update_camera_info(self, message: Mapping[str, Any]) -> None:
        matrix = message.get("k")
        if not isinstance(matrix, list):
            return
        try:
            k = tuple(float(value) for value in matrix)
        except (TypeError, ValueError):
            return
        if len(k) != 9 or k[0] == 0.0:
            self._camera_calibration_snapshot = None
            if self._camera is not None:
                self._camera["calibrated"] = False
                self._camera["calibration_snapshot"] = None
            return
        try:
            seed_value = (
                str(message.get("distortion_model", "")),
                tuple(float(value) for value in message.get("d", [])),
                k,
                tuple(float(value) for value in message.get("r", [])),
                tuple(float(value) for value in message.get("p", [])),
                int(message.get("width", 0)),
                int(message.get("height", 0)),
            )
        except (TypeError, ValueError):
            return
        self._camera_calibration_snapshot = hashlib.sha256(
            repr(seed_value).encode("utf-8")
        ).hexdigest()
        if self._camera is not None:
            self._camera["calibrated"] = True
            self._camera["calibration_snapshot"] = self._camera_calibration_snapshot

    def _update_tf(self, message: Mapping[str, Any], observed: float) -> None:
        transforms = message.get("transforms")
        if not isinstance(transforms, list):
            return
        map_frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
        odom_frame = os.getenv("FLYTO_ROS2_ODOM_FRAME", "odom")
        for item in transforms:
            header = item.get("header") if isinstance(item, Mapping) else None
            if not isinstance(header, Mapping):
                continue
            if (
                header.get("frame_id") == map_frame
                and item.get("child_frame_id") == odom_frame
            ):
                transform = item.get("transform")
                planar = (
                    map_frame_module.planar_transform(
                        transform.get("translation"), transform.get("rotation")
                    )
                    if isinstance(transform, Mapping)
                    else None
                )
                self._store_map_transform(planar, observed)
                return

    def _forget_graph(self) -> None:
        self._interfaces = None

    def _handle_publish(self, topic: str, raw_message: Any) -> None:
        if not isinstance(raw_message, Mapping):
            return
        if topic in self._lifecycle_topics:
            with self._condition:
                self._graph_changed(f"lifecycle:{topic}")
            return
        if topic and topic == _collision_state_topic():
            self._track_collision_state(
                raw_message.get("action_type"), raw_message.get("polygon_name")
            )
            return
        message = dict(raw_message)
        observed = time.monotonic()
        waiter = self._capture_waits.get(topic)
        if waiter is not None:
            with self._condition:
                self._captured[waiter] = message
                self._condition.notify_all()
        handlers = {
            os.getenv(
                "FLYTO_ROS2_ODOM_TOPIC",
                "/odom",
            ): lambda: self._update_odometry(message, observed),
            os.getenv(
                "FLYTO_ROS2_SCAN_TOPIC",
                "/scan",
            ): lambda: self._update_scan(message, observed),
            os.getenv(
                "FLYTO_ROS2_CAMERA_TOPIC",
                "/camera/image_raw",
            ): lambda: self._update_camera(message, observed),
            os.getenv(
                "FLYTO_ROS2_CAMERA_INFO_TOPIC",
                "/camera/camera_info",
            ): lambda: self._update_camera_info(message),
            os.getenv(
                "FLYTO_ROS2_TF_TOPIC",
                "/tf",
            ): lambda: self._update_tf(message, observed),
        }
        handler = handlers.get(topic)
        if handler is None:
            return
        with self._condition:
            handler()

    def _wait_for(
        self,
        store: dict[str, dict[str, Any]],
        identifier: str,
        deadline: float,
        *,
        consume: bool = True,
    ) -> dict[str, Any] | None:
        with self._condition:
            while identifier not in store and self._connected:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)
            if not consume:
                return store.get(identifier)
            return store.pop(identifier, None)

    def _service(
        self,
        service: str,
        args: Mapping[str, Any],
        *,
        timeout: float = 5.0,
        service_type: str | None = None,
    ) -> dict[str, Any]:
        identifier = f"svc-{time.monotonic_ns()}"
        self._send(
            {
                "op": "call_service",
                "service": service,
                "args": dict(args),
                "id": identifier,
                **({"type": service_type} if service_type else {}),
            }
        )
        response = self._wait_for(
            self._responses,
            identifier,
            time.monotonic() + timeout,
        )
        if response is None:
            raise RuntimeError(f"rosbridge service timed out: {service}")
        if response.get("result") is not True:
            raise RuntimeError(
                f"rosbridge service failed: {service}: {response.get('values')}"
            )
        values = response.get("values")
        if not isinstance(values, Mapping):
            raise RuntimeError(f"rosbridge service response is invalid: {service}")
        return dict(values)

    def _subscribe_standard_observations(self) -> None:
        subscriptions = (
            (
                os.getenv("FLYTO_ROS2_ODOM_TOPIC", "/odom"),
                "nav_msgs/msg/Odometry",
                "reliable",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_SCAN_TOPIC", "/scan"),
                "sensor_msgs/msg/LaserScan",
                "best_effort",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_CAMERA_TOPIC", "/camera/image_raw"),
                "sensor_msgs/msg/Image",
                "best_effort",
                # Kept only as a digest, so once a second is plenty: a raw
                # 640x480 frame is ~1 MB of JSON, and ten a second starved the
                # socket a motion's cancel has to travel on.
                1000,
            ),
            (
                os.getenv(
                    "FLYTO_ROS2_CAMERA_INFO_TOPIC",
                    "/camera/camera_info",
                ),
                "sensor_msgs/msg/CameraInfo",
                "reliable",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_TF_TOPIC", "/tf"),
                "tf2_msgs/msg/TFMessage",
                "best_effort",
                0,
            ),
        )
        collision_topic = _collision_state_topic()
        if collision_topic:
            # Published when the monitor changes what it does to the base,
            # so it costs nothing while the robot drives freely.
            subscriptions = (
                *subscriptions,
                (collision_topic, COLLISION_STATE_TYPE, "reliable", 0),
            )
        if self._presence_only:
            # Odometry says the robot is there; a few a second is plenty.
            subscriptions = ((subscriptions[0][0], subscriptions[0][1], "reliable", 500),)
        subscriptions = (
            *subscriptions,
            *(
                (topic, LIFECYCLE_EVENT_TYPE, "reliable", 0)
                for topic in sorted(self._lifecycle_topics)
            ),
        )
        for index, (topic, message_type, reliability, throttle_rate) in enumerate(
            subscriptions
        ):
            self._send(
                {
                    "op": "subscribe",
                    "id": f"observation-{index}",
                    "topic": topic,
                    "type": message_type,
                    "qos": {
                        "history": "keep_last",
                        "depth": 5,
                        "reliability": reliability,
                        "durability": "volatile",
                    },
                    "throttle_rate": throttle_rate,
                }
            )

    def capture(
        self, *, call_id: str, capability_id: str, deadline_seconds: float
    ) -> CallResult:
        """Read one message from a capture topic and return it as evidence."""
        if not self._connected:
            return CallResult(call_id, OUTCOME_REFUSED, detail="rosbridge adapter disconnected")
        _, topic, message_type = DEFAULT_INTERFACES[capability_id]
        waiter = f"capture-{call_id}"
        with self._condition:
            self._capture_waits[topic] = waiter
        self._send(
            {
                "op": "subscribe",
                "id": waiter,
                "topic": topic,
                "type": message_type,
                "qos": {
                    "history": "keep_last",
                    "depth": 1,
                    "reliability": "reliable",
                    # The map is latched; a photo is the next frame.
                    "durability": (
                        "transient_local" if capability_id == "sensing.map" else "volatile"
                    ),
                },
            }
        )
        try:
            message = self._wait_for(
                self._captured,
                waiter,
                time.monotonic() + min(max(1.0, deadline_seconds), CAPTURE_WAIT_SECONDS),
            )
        finally:
            with self._condition:
                self._capture_waits.pop(topic, None)
            with contextlib.suppress(RuntimeError):
                self._send({"op": "unsubscribe", "id": waiter, "topic": topic})
        if message is None:
            return CallResult(call_id, OUTCOME_FAILED, detail=f"no {topic} message arrived")
        try:
            payload = (
                map_capture(message) if capability_id == "sensing.map" else photo_capture(message)
            )
        except ValueError as error:
            return CallResult(call_id, OUTCOME_FAILED, detail=str(error))
        return CallResult(call_id, OUTCOME_COMPLETED, evidence={"capture": payload})

    def invalidate_discovery(self) -> None:
        with self._condition:
            self._interfaces = None

    def discover(self) -> Sequence[StandardInterface]:
        if not self._connected:
            return ()
        cached = self._interfaces
        if cached is not None:
            return cached
        try:
            found = self._read_graph()
        except Exception:
            self.invalidate_discovery()
            raise
        with self._condition:
            if self._connected:
                self._interfaces = found
        return found

    def _read_graph(self) -> tuple[StandardInterface, ...]:
        topics = self._service("/rosapi/topics", {})
        topic_names = topics.get("topics")
        topic_types = topics.get("types")
        if not isinstance(topic_names, list) or not isinstance(topic_types, list):
            raise RuntimeError("rosapi topics response is invalid")
        self._topic_types = {
            str(name): str(message_type)
            for name, message_type in zip(topic_names, topic_types, strict=False)
        }
        actions = self._service("/rosapi/action_servers", {})
        raw_actions = actions.get("action_servers")
        if not isinstance(raw_actions, list):
            raise RuntimeError("rosapi action server response is invalid")
        action_names = {str(name) for name in raw_actions}
        self._action_names = frozenset(action_names)

        found: list[StandardInterface] = []
        for _capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if kind == "action":
                if name in action_names:
                    found.append(StandardInterface(kind, name, interface_type))
                continue
            topic_type = self._topic_types.get(name)
            if topic_type in CMD_VEL_TYPES or topic_type == interface_type:
                found.append(StandardInterface("topic", name, topic_type))
        marker = _simulation_marker_topic()
        if marker and self._topic_types.get(marker) == CLOCK_TYPE:
            found.append(StandardInterface("topic", marker, CLOCK_TYPE))
        return tuple(found)

    def action_available(self, capability_id: str) -> bool:
        """Whether rosapi lists an action the adapter uses on its own behalf."""
        if not self._connected:
            return False
        self.discover()
        return _interface(capability_id)[1] in self._action_names

    def get_parameter(self, node_name: str, name: str) -> Mapping[str, Any] | None:
        """One parameter of another node, through its standard GetParameters service."""
        response = self._service(
            f"{node_name.rstrip('/')}/get_parameters",
            {"names": [name]},
            timeout=1.0,
            service_type=GET_PARAMETERS_TYPE,
        )
        values = response.get("values")
        if not isinstance(values, list) or not values or not isinstance(values[0], Mapping):
            return None
        return dict(values[0])

    @staticmethod
    def _duration(seconds: float) -> dict[str, int]:
        bounded = max(0.0, seconds)
        whole = int(bounded)
        return {
            "sec": whole,
            "nanosec": int((bounded - whole) * 1_000_000_000),
        }

    def _action_goal(
        self,
        capability_id: str,
        arguments: Mapping[str, Any],
    ) -> dict[str, Any]:
        allowance = max(
            1.0,
            float(os.getenv("FLYTO_ROS2_ACTION_ALLOWANCE_SECONDS", "30")),
        )
        if capability_id == NAVIGATE_THROUGH:
            frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
            return {
                "poses": [_goal_pose(frame, x, y, yaw) for x, y, yaw in _route_poses(arguments)],
                "behavior_tree": "",
            }
        if capability_id in PLANNED_MOTIONS:
            yaw = float(arguments.get("yaw_radians", 0.0))
            frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
            return {
                "pose": {
                    "header": {
                        "frame_id": frame,
                    },
                    "pose": {
                        "position": {
                            "x": float(arguments["x"]),
                            "y": float(arguments["y"]),
                            "z": 0.0,
                        },
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(yaw / 2.0),
                            "w": math.cos(yaw / 2.0),
                        },
                    },
                },
                "behavior_tree": "",
            }
        if capability_id == "motion.rotate":
            return {
                "target_yaw": float(arguments["yaw_radians"]),
                "time_allowance": self._duration(allowance),
            }
        distance = float(arguments["distance_m"])
        speed = float(
            arguments.get(
                "speed_mps",
                0.10 if capability_id == "motion.retreat" else 0.12,
            )
        )
        return {
            "target": {"x": distance, "y": 0.0, "z": 0.0},
            "speed": speed,
            "time_allowance": self._duration(allowance),
        }

    def _evidence(self, capability_id: str, *, wait: bool = True) -> dict[str, Any]:
        """What the robot reported with a motion's result.

        ``wait=False`` reads what is held now: a motion that failed or ran out
        of time is followed by the host's safe stop, which must not first wait
        out the observation timeout for a LiDAR that went quiet.
        """
        kind, name, interface_type = _interface(capability_id)
        evidence: dict[str, Any] = {
            "adapter": "generic_ros2",
            "transport": "rosbridge",
            "standard_interface": {
                "kind": kind,
                "name": name,
                "type": interface_type,
            },
        }
        if wait:
            observation = self.observation()
        else:
            with self._condition:
                observation = self._snapshot()
        if observation.get("pose") is not None:
            evidence["odom"] = dict(observation["pose"])
        if observation.get("map_pose") is not None:
            evidence["map_pose"] = dict(observation["map_pose"])
        range_observation = observation.get("range")
        if isinstance(range_observation, Mapping):
            evidence["minimum_range_m"] = range_observation["minimum_range_m"]
        return evidence

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult:
        if not self._connected:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="rosbridge adapter disconnected",
            )
        if call_id in self._results:
            return self._results[call_id]
        if call_id not in self._active_actions:

            _, action_name, action_type = _interface(capability_id)
            try:
                self._send(
                    {
                        "op": "send_action_goal",
                        "id": call_id,
                        "action": action_name,
                        "action_type": action_type,
                        "args": self._action_goal(capability_id, arguments),
                        "feedback": True,
                    }
                )
            except RuntimeError as error:
                self.invalidate_discovery()
                return CallResult(call_id, OUTCOME_REFUSED, detail=str(error))
            self._active_actions[call_id] = action_name
            self._count_execution(call_id)
            self._begin_track(call_id, capability_id, arguments)

        with self._condition:
            self._result_waiters[call_id] = self._result_waiters.get(call_id, 0) + 1
        try:
            result_message = self._await_result(
                call_id, capability_id, time.monotonic() + deadline_seconds
            )
        finally:
            with self._condition:
                remaining = self._result_waiters.get(call_id, 0) - 1
                if remaining > 0:
                    self._result_waiters[call_id] = remaining
                else:
                    self._result_waiters.pop(call_id, None)
        if result_message is None:
            return _motion_result(
                call_id,
                OUTCOME_TIMEOUT,
                self._evidence(capability_id, wait=False),
                self._motion_summary(call_id, status=None),
                fallback="ROS 2 action still running",
            )

        self._active_actions.pop(call_id, None)
        self._forget_goal_ids(call_id)
        status = int(result_message.get("status", 0))
        values = result_message.get("values")
        guard = self._guard(call_id)
        braked = guard is not None and guard.tripped is not None
        if braked:
            # The last command on cmd_vel must be a stop, whatever the server
            # published while the cancel was on its way.
            with contextlib.suppress(RuntimeError):
                self._publish_zero()
            # Read where the robot came to rest, not where it was cancelled.
            self.wait_until_stationary(1.5)
        summary = self._motion_summary(
            call_id,
            status=status,
            result_values=values if isinstance(values, Mapping) else None,
        )
        if result_message.get("result") is True and status == 4:
            result = _motion_result(
                call_id, OUTCOME_COMPLETED, self._evidence(capability_id), summary
            )
        elif braked:
            result = _motion_result(
                call_id,
                OUTCOME_FAILED,
                self._evidence(capability_id, wait=False),
                summary,
                fallback="stopped by the braking guard",
            )
        elif status == 5:
            result = _motion_result(call_id, OUTCOME_CANCELLED, {}, summary, fallback="cancelled")
        else:
            # An aborted or unknown goal may mean the server went away; the
            # next discover() asks rosapi again rather than trusting the cache.
            self.invalidate_discovery()
            result = _motion_result(
                call_id,
                OUTCOME_FAILED,
                self._evidence(capability_id, wait=False),
                summary,
                fallback=f"ROS 2 action status {status}",
            )
        return self._keep_result(call_id, result)

    def _await_result(
        self, call_id: str, capability_id: str, deadline: float
    ) -> dict[str, Any] | None:
        """The goal's result, acting on its braking guard while waiting.

        Returns None when the deadline passes or the connection drops first.
        A guarded drive is re-judged on every scan (see ``_check_guards``);
        this loop carries out what that decides: a stop, or a slower goal.
        """
        while True:
            action = None
            with self._condition:
                while True:
                    if call_id in self._action_results:
                        return self._action_results.pop(call_id)
                    if not self._connected:
                        return None
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    guard = self._guards.get(call_id)
                    if guard is not None:
                        self._check_scan_fresh(call_id)
                        if guard.tripped is not None and not guard.stop_command_sent:
                            guard.stop_command_sent = True
                            action = "brake"
                            break
                        if guard.tripped is None and guard.slow_to is not None:
                            action = "slow"
                            break
                    # A guarded drive also wakes to notice a LiDAR gone quiet.
                    self._condition.wait(
                        timeout=min(remaining, 0.05) if guard is not None else remaining
                    )
            if action == "brake":
                self._brake(call_id)
            elif action == "slow":
                self._slow_down(call_id, capability_id)

    def _brake(self, call_id: str) -> None:
        """Stop a guarded drive: zero first, then cancel its goal, then zero again."""
        guard = self._guard(call_id)
        logger.warning(
            "braking guard stopping %s: %s %s",
            call_id,
            guard.tripped if guard else "?",
            guard.trip if guard else {},
        )
        with contextlib.suppress(RuntimeError):
            self._publish_zero()
        action_name = self._active_actions.get(call_id)
        if action_name is not None:
            with contextlib.suppress(RuntimeError):
                self._send(
                    {
                        "op": "cancel_action_goal",
                        "id": self._goal_ids.get(call_id, call_id),
                        "action": action_name,
                    }
                )
        with contextlib.suppress(RuntimeError):
            self._publish_zero()

    def _slow_down(self, call_id: str, capability_id: str) -> None:
        """Preempt a guarded drive with the same heading, the rest of its
        distance, and the slower speed its room now allows."""
        action_name = self._active_actions.get(call_id)
        with self._condition:
            guard = self._guards.get(call_id)
            if guard is None or guard.slow_to is None or action_name is None:
                return
            speed = guard.slow_to
            guard.slow_to = None
            remaining = guard.distance_m - _travelled_along(
                guard.start_pose, self._pose, guard.direction_rad
            )
            if remaining <= 0.02:
                return  # nearly there; a new goal would only add a stop
            previous = self._goal_ids.get(call_id, call_id)
            sequence = len(guard.speed_changes)
            goal_id = f"{call_id}#slow{sequence}"
            self._goal_ids[call_id] = goal_id
            self._goal_owner[goal_id] = call_id
            now = time.monotonic()
            guard.last_slowdown_at = now
            guard.speed_mps = speed
            started = self._motion_tracks.get(call_id)
            guard.speed_changes.append(
                {
                    "at_s": round(now - started.started_at, 3) if started else None,
                    "speed_mps": round(speed, 4),
                    "room_m": (
                        round(guard.minimum_room_m, 4)
                        if guard.minimum_room_m is not None
                        else None
                    ),
                    "remaining_m": round(remaining, 4),
                }
            )
        _, _, action_type = _interface(capability_id)
        try:
            self._send(
                {
                    "op": "send_action_goal",
                    "id": goal_id,
                    "action": action_name,
                    "action_type": action_type,
                    "args": self._action_goal(
                        capability_id, {"distance_m": remaining, "speed_mps": speed}
                    ),
                    "feedback": True,
                }
            )
        except RuntimeError:
            # The faster goal is still the one driving; the guard keeps
            # watching it and stops it if the room runs out.
            with self._condition:
                self._goal_ids[call_id] = previous
                self._goal_owner.pop(goal_id, None)

    def _forget_goal_ids(self, call_id: str) -> None:
        with self._condition:
            self._goal_ids.pop(call_id, None)
            for goal_id in [g for g, owner in self._goal_owner.items() if owner == call_id]:
                self._goal_owner.pop(goal_id, None)

    def _publish_zero(self) -> bool:
        topic = DEFAULT_INTERFACES["motion.halt"][1]
        topic_type = self._topic_types.get(topic)
        if topic_type not in CMD_VEL_TYPES:
            # A stop must not trust a cached graph that lacked cmd_vel.
            self.invalidate_discovery()
            try:
                self.discover()
            except RuntimeError:
                return False
            topic_type = self._topic_types.get(topic)
        if topic_type not in CMD_VEL_TYPES:
            return False
        key = (topic, topic_type)
        if key not in self._advertised_topics:
            self._send(
                {
                    "op": "advertise",
                    "topic": topic,
                    "type": topic_type,
                    "qos": {
                        "history": "keep_last",
                        "depth": 1,
                        "reliability": "reliable",
                        "durability": "volatile",
                    },
                }
            )
            self._advertised_topics.add(key)
        if topic_type == "geometry_msgs/msg/TwistStamped":
            message: dict[str, Any] = {
                "header": {
                    "frame_id": os.getenv("FLYTO_ROS2_BASE_FRAME", "base_link"),
                },
                "twist": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
            }
        else:
            message = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        self._send({"op": "publish", "topic": topic, "msg": message})
        return True

    def cancel(self, call_id: str) -> CallResult:
        action_name = self._active_actions.get(call_id)
        if action_name is None:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="no active ROS 2 goal",
            )
        self._send(
            {
                "op": "cancel_action_goal",
                "id": self._goal_ids.get(call_id, call_id),
                "action": action_name,
            }
        )
        stopped = self._publish_zero()
        # Read, not take: the goal may belong to another job on this shared
        # connection (a safe stop cancels every active goal), and its own
        # invoke() must still receive the result.
        result_message = self._wait_for(
            self._action_results,
            call_id,
            time.monotonic() + 3.0,
            consume=False,
        )
        confirmed = result_message is not None and int(result_message.get("status", 0)) == 5
        with self._condition:
            if confirmed and call_id not in self._result_waiters:
                # Nobody is waiting; the kept CANCELLED result answers later.
                self._action_results.pop(call_id, None)
        if (
            result_message is None
            or int(result_message.get("status", 0)) != 5
            or not stopped
        ):
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                detail="ROS 2 cancel/stop was not confirmed",
            )
        self._active_actions.pop(call_id, None)
        self._forget_track(call_id)
        return self._keep_result(
            call_id, CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        )

    def safe_stop(self, call_id: str) -> CallResult:
        # Zero velocity goes out first, so the stop never waits behind a cancel.
        try:
            stopped = self._publish_zero()
        except RuntimeError:
            stopped = False
        active = tuple(self._active_actions)
        for active_call in active:
            with contextlib.suppress(RuntimeError):
                self.cancel(active_call)
            # Tried once: an unanswered cancel must not cost every later
            # emergency stop another wait.
            self._active_actions.pop(active_call, None)
            self._forget_track(active_call)
        if active:
            # Until the cancel landed the action server could overwrite the
            # first zero; the last command on cmd_vel must be this stop's.
            with contextlib.suppress(RuntimeError):
                stopped = self._publish_zero() or stopped
        if not stopped:
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                detail="standard cmd_vel stop unavailable",
            )
        self._count_execution(call_id)
        result = CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "adapter": "generic_ros2",
                "transport": "rosbridge",
                "standard_interface": {
                    "kind": "topic",
                    "name": DEFAULT_INTERFACES["motion.halt"][1],
                    "type": "geometry_msgs/msg/Twist|TwistStamped",
                },
                "zero_velocity_published": True,
            },
        )
        return self._keep_result(call_id, result)

    def execution_count(self, call_id: str) -> int:
        return self._counts.get(call_id, 0)

    def disconnect(self) -> None:
        with self._condition:
            websocket = self._ws
            self._connected = False
            self._ws = None
            self._interfaces = None
            self._keepalive_stop.set()
            self._condition.notify_all()
        if websocket is not None:
            with contextlib.suppress(Exception):
                websocket.close()
        self._reset_readings()
        self._stop_graph_events()

    def reconnect(self) -> None:
        self.disconnect()
        # Goal ids belong to the socket that sent them.
        self._active_actions.clear()
        self._goal_ids.clear()
        self._goal_owner.clear()
        websocket = self._connection_factory(self._url)
        keepalive_stop = threading.Event()
        with self._condition:
            self._ws = websocket
            self._connected = True
            self._interfaces = None
            self._keepalive_stop = keepalive_stop
            self._responses.clear()
            self._action_results.clear()
            self._advertised_topics.clear()
        self._reader = threading.Thread(
            target=self._reader_loop,
            args=(websocket,),
            name="rosbridge-reader",
            daemon=True,
        )
        self._reader.start()
        threading.Thread(
            target=self._keepalive_loop,
            args=(websocket, keepalive_stop),
            name="rosbridge-keepalive",
            daemon=True,
        ).start()
        self._subscribe_standard_observations()
        self._notify_connection(True)

    def _keepalive_loop(self, websocket: Any, stop: threading.Event) -> None:
        """Send an unsolicited pong every few seconds while connected.

        The robot runs rosbridge with websocket_ping_interval == ping_timeout
        (20 s). Tornado measures the timeout from the last pong it received,
        so at its first ping it closes any client that has not sent one: every
        call longer than 20 s lost its socket mid-motion, and with it the
        cancel and the safe stop. A pong is a valid unsolicited heartbeat
        (RFC 6455 5.5.3) and keeps that clock fresh.
        """
        while True:
            with self._condition:
                if not self._connected or websocket is not self._ws:
                    return
            try:
                with self._send_lock:
                    websocket.pong(b"flyto")
            except Exception:  # noqa: BLE001 - the reader notices a dead socket
                return
            # Woken at once by disconnect() instead of finishing its interval.
            if stop.wait(KEEPALIVE_SECONDS):
                return


def build(resource_id: str = "", *, presence_only: bool = False) -> GenericROS2Adapter:
    """Build the host-side adapter for one commanded ROS 2 resource.

    ``presence_only`` builds a light connection that only watches whether the
    robot is there and what its graph offers; it refuses every command.
    """
    transport = os.getenv("FLYTO_ROS2_TRANSPORT", "rclpy").strip().lower()
    if transport == "rclpy":
        backend: ROS2Backend = RclpyROS2Backend(presence_only=presence_only)
    elif transport == "rosbridge":
        backend = RosbridgeROS2Backend(presence_only=presence_only)
    else:
        raise RuntimeError(f"unsupported ROS 2 transport: {transport}")
    return GenericROS2Adapter(backend=backend, resource_id=resource_id)

