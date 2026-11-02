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
import dataclasses
import hashlib
import json
import logging
import math
import os
import queue
import re
import socket
import threading
import time
from collections.abc import Iterable, Iterator, Mapping, Sequence
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Protocol

from . import adapter_contract as decl
from . import braking_envelope as braking
from . import (
    cargo_sensor,
    fsio,
    inflation_escape,
    live_telemetry,
    motion_outcome,
    path_clearance,
    places,
    provider_evidence,
    ros2_interfaces,
    safety_profile,
    sim_time,
    ssh_transport,
)
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

# A robot is stationary once odometry shows it still on BOTH counts, over at
# least STILL_SAMPLES consecutive messages spanning at least STILL_MIN_SECONDS
# of their receive times:
# * the twist: every axis (linear x/y/z, angular x/y/z) present and this
#   close to zero. A missing or non-numeric axis is unknown, never 0, so a
#   driver that fills no twist can never show the robot still;
# * the pose: within STILL_POSE_DELTA_M / STILL_YAW_DELTA_RAD of where the
#   still run began, so a driver whose twist reads zero while the pose moves
#   (or a slow creep) is not still.
# The duration keeps a momentary pause (a speed reversal, the start of a
# rotate) from passing as a stop: at 30-50 Hz three messages are under
# 0.1 s. It does not cover a long pause such as Nav2's Wait recovery (5 s).
# A halt cancels the goals this host process knows (the ones it runs, and the
# ones in the hold) and confirms from their action servers' status that held
# ones ended. It does NOT cover a goal this process never knew: one sent by
# another client, or by a host process that crashed before anything armed the
# hold. Such a goal paused for STILL_MIN_SECONDS or longer survives a
# ``completed`` halt.
STILL_SAMPLES = 3
STILL_MIN_SECONDS = 0.5
STILL_LINEAR_MPS = 0.01
STILL_ANGULAR_RADPS = 0.02
# motion.halt is completed only once odometry shows the robot still (as
# above), within this long after the cancel was confirmed.
HALT_STILL_WAIT_S = 2.0
# While a halted goal's cancel is unconfirmed, zero velocity is re-sent this
# often, so the action server's last command never stands.
ZERO_HOLD_PERIOD_S = 0.1
# A running goal's cancel is confirmed only by the goal ending (its result, a
# terminal status) within this long; a cancel's answer alone never confirms it.
CANCEL_CONFIRM_WAIT_S = 3.0
# A goal held from an earlier connection is released only once its action
# server's goal status shows no goal still live; that is awaited this long.
HELD_GOAL_END_WAIT_S = 2.0
# action_msgs/msg/GoalStatus values of a goal that has ended. Every other
# value (ACCEPTED 1, EXECUTING 2, CANCELING 3, UNKNOWN 0, or anything that is
# not a status) is live: a CANCELING goal's server is still driving it.
GOAL_ENDED_STATUSES = frozenset({4, 5, 6})
GOAL_STATUS_ARRAY_TYPE = "action_msgs/msg/GoalStatusArray"
CANCEL_GOAL_TYPE = "action_msgs/srv/CancelGoal"
# A rosbridge service call sent without waiting: its answer is dropped.
UNAWAITED_SERVICE_PREFIX = "unawaited-"


def action_status_topic(action_name: str) -> str:
    """The action server's goal status topic (``<action>/_action/status``)."""
    return f"{action_name.rstrip('/')}/_action/status"


def action_cancel_service(action_name: str) -> str:
    """The action server's own cancel service (``<action>/_action/cancel_goal``)."""
    return f"{action_name.rstrip('/')}/_action/cancel_goal"


def goal_statuses(message: Any) -> list[int]:
    """Every goal's status in a GoalStatusArray (rosbridge dict or rclpy message).

    A malformed array or entry reads as UNKNOWN (0), which counts as live.
    """
    entries = (
        message.get("status_list")
        if isinstance(message, Mapping)
        else getattr(message, "status_list", None)
    )
    if not isinstance(entries, (list, tuple)):
        return [0]
    statuses: list[int] = []
    for entry in entries:
        raw = entry.get("status") if isinstance(entry, Mapping) else getattr(entry, "status", None)
        statuses.append(int(raw) if isinstance(raw, int) and not isinstance(raw, bool) else 0)
    return statuses


def live_goal_statuses(statuses: Iterable[int]) -> list[int]:
    """The statuses of goals that have not ended (see ``GOAL_ENDED_STATUSES``)."""
    return [status for status in statuses if status not in GOAL_ENDED_STATUSES]


def token_action(token: Any) -> str | None:
    """The action name a hold's cancel token carries: ``(goal id, action)``."""
    if isinstance(token, tuple) and len(token) == 2 and isinstance(token[1], str):
        return token[1] or None
    return None
# The typed reason of a halt that could not confirm the robot stopped. It is
# Flyto2 Cloud's own code (services/space_tasks/uncertain_task_effect.py), so
# the task is held for inspection rather than taken as stopped.
REASON_EFFECT_UNKNOWN = "effect_unknown"
# Per-call results and execution counts kept for idempotent retries of the
# same call_id. A warm adapter lives as long as its host, so this is bounded.
CALL_HISTORY_LIMIT = 256
# How far the pose may move across a still run (see STILL_SAMPLES).
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
    """The LiDAR clearance a motion needs to start; never below 0.35 m."""
    return max(0.35, float(os.getenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.35")))


def _rotation_radius() -> float:
    """The footprint's radius about its centre of rotation (Nav2 ``robot_radius``)."""
    return _env_metres(
        "FLYTO_ROS2_ROBOT_RADIUS_M", inflation_escape.DEFAULT_ROBOT_RADIUS_M, low=0.0, high=1.0
    )


def _motion_kind(capability_id: str) -> str | None:
    """What a capability's motion sweeps, from its contract (not its name)."""
    try:
        kind = decl.capability_metadata(capability_id).get("motion_kind")
    except KeyError:
        return None
    return str(kind) if kind else None


# -- the braking guard ---------------------------------------------------------------
#
# The floor above is checked once, before a motion starts. A straight drive is
# then watched on every scan: it is stopped while the room left before a return
# reaches the floor is still at least the distance the robot needs to stop from
# its speed (braking_envelope). Its speed is governed one of two ways
# (braking.decide_governance): lowered by a preempting goal as the room shrinks,
# on a server that takes one, or else planned before sending together with a
# distance that ends the drive where it can still stop at the floor. Turns and
# planned paths have no single direction; a navigation leg (and an inflation
# escape's back-off) is watched by the same guard as a floor guard instead
# (GUARD_KIND_LEG): a return inside the floor in any direction, or the room the
# way the base is moving within its stopping distance, stops the leg
# (braking.leg_floor_check). Nav2's collision monitor only slowed the real robot
# to a person's shins on 2026-10-09 (run c6d8d79d: 0.135 m against 0.35 m).

#: How a guarded motion is judged on each scan.
GUARD_KIND_STRAIGHT = "straight_drive"
GUARD_KIND_LEG = "navigation_leg"

#: Direction of travel of each straight drive, from the robot's heading.
STRAIGHT_MOTIONS: Mapping[str, float] = {
    "motion.advance": 0.0,
    "motion.retreat": math.pi,
}
# A straight drive's speed is declared with its arguments (ARGUMENTS:
# ``speed_mps`` bounds and the default a call that names none is sent at),
# tightened by the resource's own top speed; see ``declared_arguments``.
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


def _scan_timeout_s() -> float:
    """The configured minimum a guarded motion may go without a scan."""
    return _env_number("FLYTO_ROS2_STOP_SCAN_TIMEOUT_S", 0.5, low=0.1, high=5.0)


#: A header stamp further than this from this computer's wall clock is not
#: wall time (a simulator's clock, or a robot clock never set) and is not aged.
MAX_STAMP_AGE_S = 60.0


def _stamp_age_s(stamp_s: float | None) -> float | None:
    """Seconds since a header stamp by the wall clock, or None if not wall time."""
    if stamp_s is None:
        return None
    age = time.time() - stamp_s
    return round(age, 3) if abs(age) <= MAX_STAMP_AGE_S else None


#: How often a guarded motion's waiter re-checks the LiDAR's age, in seconds.
#: A stale scan is noticed up to this late, so the floor bound allows for it.
GUARD_POLL_S = 0.05

#: How long each kind of guarded motion may go without a scan
#: (braking.scan_stale_limit). Straight drives: 15 nominal periods (1.5 s at
#: the robot's 10 Hz; the live stall of 2026-10-12 was 0.965 s), never below
#: 3 x the worst recent interval (the 0.8.1 rule), never above 2.0 s, and never
#: longer than a stop decided then could still hold the floor from the room
#: the last scan showed. A navigation leg's speed and path are Nav2's, so the
#: guard has no speed to bound its travel with: it keeps the 0.8.1 rule.
SCAN_STALENESS: Mapping[str, braking.ScanStalePolicy] = {
    GUARD_KIND_STRAIGHT: braking.ScanStalePolicy(
        missed_periods=15.0, worst_interval_multiple=3.0, ceiling_s=2.0, floor_bound=True,
    ),
    GUARD_KIND_LEG: braking.ScanStalePolicy(
        missed_periods=0.0, worst_interval_multiple=3.0, ceiling_s=None, floor_bound=False,
    ),
}


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
    # How the speed is governed while the drive runs (braking.decide_governance)
    # and the speed and distance planned before it was sent.
    governance: braking.SpeedGovernance | None = None
    plan: braking.DrivePlan | None = None
    # How the drive ended when the guard did not stop it: at the stop point
    # planned before sending (GUARD_END_PLANNED_STOP), short of the request.
    ended: str | None = None
    kind: str = GUARD_KIND_STRAIGHT
    # A navigation leg's nearest return over the whole leg, as the guard saw it.
    minimum_range_m: float | None = None
    # A straight drive's room to the floor on the latest scan (the start room
    # until one arrives): what a stale-scan limit is bounded by.
    last_room_m: float | None = None

    @property
    def resends(self) -> bool:
        """Whether a slower goal may be sent while this drive runs."""
        return self.governance is not None and self.governance.resends

    def record(self, rest_pose: Mapping[str, Any] | None) -> dict[str, Any]:
        if self.kind == GUARD_KIND_LEG:
            return self._leg_record(rest_pose)
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
        if self.governance is not None:
            out["governance"] = self.governance.to_dict()
        if self.plan is not None:
            out["plan"] = self.plan.to_dict()
        if self.ended is not None:
            out["ended"] = self.ended
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

    def _leg_record(self, rest_pose: Mapping[str, Any] | None) -> dict[str, Any]:
        out: dict[str, Any] = {
            "kind": self.kind,
            "profile": self.profile.to_dict(),
            "minimum_range_m": (
                None if self.minimum_range_m is None else round(self.minimum_range_m, 4)
            ),
            "tripped": self.tripped,
        }
        if self.trip is not None:
            trip = dict(self.trip)
            if self.trip_pose is not None and rest_pose is not None:
                trip["measured_stop_distance_m"] = round(
                    math.hypot(
                        float(rest_pose["x"]) - float(self.trip_pose["x"]),
                        float(rest_pose["y"]) - float(self.trip_pose["y"]),
                    ),
                    4,
                )
            out["trip"] = trip
        return out


def _command_watch_topics() -> tuple[str, ...]:
    """Velocity topics read (never written) to say why a goal stood still.

    Nav2's controller output (``/cmd_vel_nav``) and what reaches the base
    (``/cmd_vel``): a stall with near-zero controller commands is the
    controller choosing to stand; real commands that never reach the base
    point between them (smoother, collision monitor). Empty turns it off.
    """
    raw = os.getenv("FLYTO_ROS2_COMMAND_WATCH_TOPICS", "/cmd_vel_nav,/cmd_vel")
    return tuple(dict.fromkeys(item.strip() for item in raw.split(",") if item.strip()))


#: Watched velocity topics are sampled at most this often (ms) by rosbridge.
COMMAND_WATCH_THROTTLE_MS = 100


#: An odometry twist, axis by axis: linear x, y, z, then angular x, y, z.
#: An axis the message did not carry (or not as a finite number) is None.
TwistAxes = tuple[
    float | None, float | None, float | None, float | None, float | None, float | None
]


def _axis(vector: Any, name: str) -> float | None:
    raw = vector.get(name) if isinstance(vector, Mapping) else getattr(vector, name, None)
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        return None
    value = float(raw)
    return value if math.isfinite(value) else None


def odometry_twist_axes(twist: Any) -> TwistAxes | None:
    """Every axis of an odometry twist (a rosbridge dict or an rclpy message).

    None when there is no twist at all. A missing or non-numeric axis is None:
    unknown, never taken as 0.
    """
    if twist is None:
        return None
    if isinstance(twist, Mapping):
        linear, angular = twist.get("linear"), twist.get("angular")
    else:
        linear, angular = getattr(twist, "linear", None), getattr(twist, "angular", None)
    if linear is None and angular is None:
        return None
    return (
        _axis(linear, "x"),
        _axis(linear, "y"),
        _axis(linear, "z"),
        _axis(angular, "x"),
        _axis(angular, "y"),
        _axis(angular, "z"),
    )


def _twist_still(axes: TwistAxes | None) -> bool:
    """Every axis known and near zero; anything unknown is not still."""
    if axes is None or any(value is None for value in axes):
        return False
    return all(abs(value) <= STILL_LINEAR_MPS for value in axes[:3]) and all(  # type: ignore[arg-type]
        abs(value) <= STILL_ANGULAR_RADPS for value in axes[3:]  # type: ignore[arg-type]
    )


def _pose_within(anchor: Mapping[str, Any], pose: Mapping[str, Any]) -> bool:
    """The pose is still where ``anchor`` was (position and heading)."""
    moved = math.hypot(
        float(pose["x"]) - float(anchor["x"]), float(pose["y"]) - float(anchor["y"])
    )
    turned = abs(math.remainder(float(pose["yaw"]) - float(anchor["yaw"]), math.tau))
    return moved <= STILL_POSE_DELTA_M and turned <= STILL_YAW_DELTA_RAD


def _twist_command(message: Any) -> tuple[float, float] | None:
    """(linear x, angular z) of a Twist or TwistStamped message, or None."""
    if not isinstance(message, Mapping):
        return None
    twist = message.get("twist") if isinstance(message.get("twist"), Mapping) else message
    linear = twist.get("linear")
    angular = twist.get("angular")
    if not isinstance(linear, Mapping) or not isinstance(angular, Mapping):
        return None
    try:
        return float(linear.get("x", 0.0)), float(angular.get("z", 0.0))
    except (TypeError, ValueError):
        return None


def _collision_state_topic() -> str:
    """Nav2's collision monitor state, read to tell why a motion stopped.

    Empty disables it for a robot without a collision monitor.
    """
    return os.getenv("FLYTO_ROS2_COLLISION_STATE_TOPIC", "/collision_monitor_state").strip()


COLLISION_STATE_TYPE = "nav2_msgs/msg/CollisionMonitorState"


@dataclass(frozen=True)
class ContinuousSubscription:
    """One topic a rosbridge connection reads for as long as it is open."""

    topic_env: str
    default_topic: str
    message_type: str
    reliability: str
    #: rosbridge sends at most one message per this many ms (0: every one).
    throttle_ms: int
    #: rosbridge queues at most this many and drops the oldest (0: its default).
    queue_length: int
    #: What reads it, and why this rate is enough.
    reader: str


#: What every full rosbridge connection reads continuously. rosbridge on the
#: robot is Python and converts every message it sends to JSON; on 2026-10-12
#: it used 75 % of a core on burger-01 (load ~10 on 4 cores) and scans reached
#: the adapter up to 0.965 s late while the sensor kept 10 Hz. Measured
#: read-only through the robot's tunnel that day, one connection carried
#: /camera/image_raw 756 KiB/s (0.9 msg/s, 811 KiB each), /scan 98 KiB/s
#: (10/s), /odom 13 KiB/s (18/s), /tf 10 KiB/s (27/s), /camera/camera_info
#: 6 KiB/s (15/s). Each row is only what its reader needs:
#:
#: * no camera: only the observation bundle's ``camera`` digest read the raw
#:   frames and nothing consumes it; ``vision.observe`` subscribes to the
#:   compressed topic for the one frame it captures, and the HMI's live camera
#:   is its own channel;
#: * scan: every scan, newest first -- the braking guard's period is the
#:   sensor's, and an old scan is worth nothing once a newer one exists;
#: * odometry: 10 Hz, newest first -- the guard's speed and stop pose, the
#:   stillness wait and motion tracks; the robot publishes about 20 Hz;
#: * /tf: unthrottled, because rosbridge throttles per topic and the frequent
#:   odom->base transforms would crowd out map->odom.
ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS: tuple[ContinuousSubscription, ...] = (
    ContinuousSubscription(
        "FLYTO_ROS2_ODOM_TOPIC", "/odom", "nav_msgs/msg/Odometry", "reliable", 100, 1,
        "pose, velocity and stillness; braking guard speed and stop pose",
    ),
    ContinuousSubscription(
        "FLYTO_ROS2_SCAN_TOPIC", "/scan", "sensor_msgs/msg/LaserScan", "best_effort", 0, 1,
        "clearance, braking guard, floor guard, escape and destination checks",
    ),
    ContinuousSubscription(
        "FLYTO_ROS2_TF_TOPIC", "/tf", "tf2_msgs/msg/TFMessage", "best_effort", 0, 0,
        "map->odom for the map pose",
    ),
)


BATTERY_STATE_TYPE = "sensor_msgs/msg/BatteryState"
#: The robot publishes its battery about once a second; never sample faster.
BATTERY_THROTTLE_MS = 1000


def _battery_topic() -> str:
    return os.getenv("FLYTO_ROS2_BATTERY_TOPIC", "/battery_state")


def _max_observation_age() -> float:
    return max(0.1, float(os.getenv("FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS", "5")))


def _simulation_marker_topic() -> str:
    """Topic whose presence means the ROS graph is a simulator.

    A Gazebo graph publishes /clock for use_sim_time; the physical TurtleBot3
    does not. An empty value disables the cross-check for a robot that
    legitimately publishes /clock.
    """
    return os.getenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "/clock").strip()


#: The served deployment of a simulation configured with the marker check
#: disabled: nothing can confirm it, so it is never labelled real or simulation.
DEPLOYMENT_UNCONFIRMED = "unconfirmed"


def _served_deployment_mode() -> str:
    """``simulation``, ``real`` or ``unconfirmed``, as ``served_identity`` answers.

    A motion or a reading reaches the robot only after the graph has
    confirmed the configured mode (``_deployment_mismatch``), so during one
    this is the deployment the adapter serves. A simulation configured with
    the marker check disabled is ``unconfirmed``: it is refused, and simulated
    data is never labelled ``real``. Never derived from a resource name.
    """
    if _configured_deployment_mode() == "simulation":
        return "simulation" if _simulation_marker_topic() else DEPLOYMENT_UNCONFIRMED
    return "real"


def _unmarked_simulation_refusal() -> str | None:
    """Why a simulation configured with the marker check disabled is refused, or None.

    With no marker the graph cannot show it is not the physical robot, so
    neither motion, nor a reading, nor ``served_identity`` is served.
    """
    if _configured_deployment_mode() == "simulation" and not _simulation_marker_topic():
        return (
            "the adapter is configured for simulation but the simulation marker "
            "check is disabled (FLYTO_ROS2_SIM_MARKER_TOPIC is empty), so the "
            "graph cannot confirm it is not physical hardware"
        )
    return None


def _safe_fragment(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value or "").strip())
    return text.strip("-._")[:48] or "resource"


def configured_resource_id() -> str:
    """The one resource the adapter on this computer is configured to serve.

    ``FLYTO_ROS2_RESOURCE_ID``, else a name derived from this host and its
    ROS domain. Discovery publishes it, and it is the only id an adapter here
    may be built for or may say it serves: the id a host hands the factory is
    a request, not a fact about what the transport reaches.
    """
    domain = os.getenv("ROS_DOMAIN_ID", "0").strip() or "0"
    return (
        os.getenv("FLYTO_ROS2_RESOURCE_ID", "").strip()
        or f"ros2-{_safe_fragment(socket.gethostname())}-{_safe_fragment(domain)}"
    )[:128]


def _configured_deployment_mode() -> str:
    """``simulation`` or ``hardware``: anything but simulation is hardware."""
    mode = os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware").strip().lower()
    return "simulation" if mode == "simulation" else "hardware"


class SimulationOnlyError(RuntimeError):
    """A simulation-only action was asked of a deployment not confirmed simulated."""


def simulation_publish_refusal() -> str | None:
    """Why this configuration may not publish simulated state, or None.

    Only a deployment configured as ``simulation`` with the simulator marker
    check on may; the graph must then also carry the marker
    (``RosbridgeROS2Backend.publish_simulated_state``).
    """
    if _configured_deployment_mode() != "simulation":
        return (
            "simulated state is published only to a simulation; "
            "FLYTO_ROS2_DEPLOYMENT_MODE is not simulation"
        )
    if not _simulation_marker_topic():
        return "the simulator marker check (FLYTO_ROS2_SIM_MARKER_TOPIC) is disabled"
    return None


class ServedIdentityError(RuntimeError):
    """The adapter cannot show which resource it serves, or in which deployment."""
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

#: Capability id -> (kind, graph name, type), derived from the one interface
#: table that the ROS 2 pairing approval is derived from too.
DEFAULT_INTERFACES = ros2_interfaces.declared_interfaces()

# Interfaces the adapter uses on its own behalf, never declared as capabilities:
# an escape from an obstacle's inflation sends its waypoint and the original
# goal as one NavigateThroughPoses goal when the graph has it.
NAVIGATE_THROUGH = "motion.navigate_through_poses"
ESCAPE_INTERFACES = ros2_interfaces.own_interfaces()
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


def _controller_nodes() -> tuple[str, ...]:
    """Controller nodes whose goal checkers say how close a goal counts as reached."""
    raw = os.getenv("FLYTO_ROS2_CONTROLLER_NODES", "/controller_server")
    return tuple(item.strip() for item in raw.split(",") if item.strip())


def _escape_enabled() -> bool:
    return os.getenv("FLYTO_ROS2_INFLATION_ESCAPE", "on").strip().lower() not in {
        "0",
        "off",
        "false",
        "no",
    }


# How an escape backs off: slowly, since it starts close to an obstacle. Never
# above the site's ceiling either (see _escape_backoff_arguments).
ESCAPE_BACKOFF_SPEED_MPS = 0.05


def _escape_backoff_arguments(distance_m: float) -> dict[str, Any] | str:
    """The backoff leg's ``motion.retreat`` arguments, or why it cannot be sent.

    Its speed is ``ESCAPE_BACKOFF_SPEED_MPS`` or the declared ``speed_mps``
    maximum, which the site's ``max_linear_speed`` lowers
    (``declared_arguments``), whichever is slower. Its distance is the
    escape's own (a short backoff is below a caller's declared minimum).
    """
    speed = next(item for item in declared_arguments("motion.retreat") if item.name == "speed_mps")
    chosen = min(ESCAPE_BACKOFF_SPEED_MPS, speed.maximum or ESCAPE_BACKOFF_SPEED_MPS)
    if speed.minimum is not None and chosen < speed.minimum:
        return (
            f"inflation escape backoff cannot be sent: {chosen:g} m/s is below the "
            f"declared minimum {speed.minimum:g} m/s"
        )
    return {"distance_m": distance_m, "speed_mps": chosen}


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


#: Where each navigation capability is going, in the map frame: the goal of a
#: NavigateToPose, the last pose of a NavigateThroughPoses. A capability not
#: listed is not a navigation and has no goal an HMI follows.
NAVIGATION_GOALS: Mapping[str, Callable[[Mapping[str, Any]], tuple[float, float]]] = {
    "motion.navigate": lambda arguments: (float(arguments["x"]), float(arguments["y"])),
    NAVIGATE_THROUGH: lambda arguments: _route_poses(arguments)[-1][:2],
}


def _navigation_goal(
    capability_id: str, arguments: Mapping[str, Any]
) -> tuple[float, float] | None:
    goal = NAVIGATION_GOALS.get(capability_id)
    if goal is None:
        return None
    try:
        x, y = goal(arguments)
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    return (x, y) if math.isfinite(x) and math.isfinite(y) else None


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
            "speed_mps", required=False, minimum=0.02, maximum=0.25, unit="m/s",
            default=0.12,
        ),
    ),
    "motion.retreat": (
        decl.DeclaredArgument(
            "distance_m", required=True, minimum=0.05, maximum=2.0, unit="m"
        ),
        decl.DeclaredArgument(
            "speed_mps", required=False, minimum=0.02, maximum=0.20, unit="m/s",
            default=0.10,
        ),
    ),
    "motion.rotate": (
        decl.DeclaredArgument(
            "yaw_radians", required=True, minimum=-math.pi, maximum=math.pi, unit="rad"
        ),
    ),
    "motion.halt": (),
    cargo_sensor.CAPABILITY_ID: (),
}

#: The one speed ceiling above the contract: the site safety profile's
#: ``max_linear_speed`` (``safety_profile.CONSTRAINABLE``, ``at_most``), the
#: same limit every job's SafetyLimits is folded into.
SITE_SPEED_LIMIT = "max_linear_speed"


def _site_speed_ceiling() -> float | None:
    """The site's ``max_linear_speed`` in m/s, None when the site sets none.

    Raises ``SafetyProfileError`` when the profile exists but cannot be read:
    a robot whose site limit cannot be read does not move.
    """
    return safety_profile.configured_profile().get(SITE_SPEED_LIMIT)


def _speed_limit_problem() -> str | None:
    """Why motion is refused because of the site's speed ceiling, or None."""
    try:
        ceiling = _site_speed_ceiling()
    except safety_profile.SafetyProfileError as error:
        return f"site safety profile is unreadable, so motion is refused: {error}"
    if ceiling is None:
        return None
    slowest = min(
        (
            item.minimum
            for arguments in ARGUMENTS.values()
            for item in arguments
            if item.name == "speed_mps" and item.minimum is not None
        ),
        default=braking.MIN_SPEED_MPS,
    )
    if ceiling < slowest:
        return (
            f"site safety profile {SITE_SPEED_LIMIT} {ceiling:g} m/s is below the slowest "
            f"declared drive speed {slowest:g} m/s, so no drive can obey it"
        )
    return None


def declared_arguments(capability_id: str) -> tuple[decl.DeclaredArgument, ...]:
    """A capability's arguments as this installation declares them.

    The contract's ``speed_mps`` maximum and default are folded under the
    site's ``max_linear_speed`` (``safety_profile.apply_profile``): lowered
    when the site's ceiling is lower, never raised. An unreadable profile, or
    a ceiling below the contract's minimum, leaves the contract as it is and
    motion is refused by the preflight instead.
    """
    arguments = ARGUMENTS[capability_id]
    try:
        profile = safety_profile.configured_profile()
    except safety_profile.SafetyProfileError:
        return arguments
    if SITE_SPEED_LIMIT not in profile:
        return arguments
    site = {SITE_SPEED_LIMIT: profile[SITE_SPEED_LIMIT]}
    folded: list[decl.DeclaredArgument] = []
    for item in arguments:
        if item.name == "speed_mps" and item.maximum is not None:
            maximum = safety_profile.apply_profile(
                site, {SITE_SPEED_LIMIT: item.maximum}
            ).values[SITE_SPEED_LIMIT]
            maximum = max(maximum, item.minimum or 0.0)
            default = None if item.default is None else min(item.default, maximum)
            item = dataclasses.replace(item, maximum=maximum, default=default)
        folded.append(item)
    return tuple(folded)


def declared_default(capability_id: str, name: str) -> float:
    """The declared default of one argument; a contract without one is a bug."""
    for item in declared_arguments(capability_id):
        if item.name == name and item.default is not None:
            return float(item.default)
    raise KeyError(f"{capability_id} declares no default for {name}")


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
    #: Whether this transport reads the simulator clock for ``sim_time``.
    READS_SIM_CLOCK = False

    def _init_observation_state(self) -> None:
        self._condition = threading.Condition()
        # While the robot's halt hold (HALT_HOLD) is armed, this connection
        # re-sends zero velocity from this thread.
        self._hold_lock = threading.Lock()
        self._zero_holder: threading.Thread | None = None
        # Told when the transport drops or returns, so a host keeping this
        # connection warm reconnects on the drop rather than on a timer.
        self._connection_listeners: list[Callable[[bool], None]] = []
        self._pose: dict[str, float | str] | None = None
        self._pose_seen_at: float | None = None
        # The odometry twist, every axis (None for an axis not carried).
        self._twist_axes: TwistAxes | None = None
        # (linear x, angular z) from the odometry twist, or None unless both known.
        self._velocity: tuple[float, float] | None = None
        self._odom_sequence = 0
        self._minimum_range: float | None = None
        self._range_sample_count = 0
        self._range_sweep: dict[str, Any] | None = None
        self._range_seen_at: float | None = None
        # The latest scan's header stamp (robot clock, seconds), for evidence.
        self._range_stamp_s: float | None = None
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
        # The latest battery reading (live_telemetry.battery_payload) and the
        # latest navigation goal sent on this connection, for a host's HMI.
        self._battery: dict[str, float | None] | None = None
        self._battery_seen_at: float | None = None
        self._navigation: live_telemetry.NavigationRecord | None = None

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
            goal = _navigation_goal(capability_id, arguments)
            if goal is not None:
                self._navigation = live_telemetry.NavigationRecord(
                    call_id=call_id,
                    goal_x=goal[0],
                    goal_y=goal[1],
                    updated_at=time.monotonic(),
                )
            if _served_deployment_mode() == "simulation":
                if self.READS_SIM_CLOCK:
                    track.clock = sim_time.ClockRecord(_simulation_marker_topic())
                else:
                    track.sim_time_note = {
                        "applicable": True,
                        "measured": False,
                        "reason": "this transport does not read the simulator clock",
                    }
            _remember(self._motion_tracks, call_id, track)

    def _track_clock(self, message: Any, observed: float) -> None:
        """A simulator clock sample (wall time ``observed``) for every running motion."""
        seconds = sim_time.clock_seconds(message)
        if seconds is None:
            return
        with self._condition:
            for track in self._motion_tracks.values():
                track.saw_clock(seconds, observed)

    # -- braking guard ---------------------------------------------------------

    def _note_scan_timing(self, observed: float, stamp_s: float | None) -> None:
        """Called under the condition with each scan's arrival and header stamp."""
        delay = time.time() - stamp_s if stamp_s else None
        self._range_stamp_s = stamp_s or None
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
        governance: braking.SpeedGovernance | None = None,
        plan: braking.DrivePlan | None = None,
    ) -> None:
        """Watch the straight drive ``call_id`` against ``profile`` from now on.

        Without a ``governance`` the drive is never re-sent: the guard only
        stops it, which is safe on any server.
        """
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
                governance=governance,
                plan=plan,
                last_room_m=start_room_m,
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

    def arm_floor_guard(
        self, call_id: str, capability_id: str, *, profile: braking.BrakingProfile
    ) -> None:
        """Watch the navigation leg ``call_id`` against the floor from now on.

        Stopped, never slowed: the leg's goal is cancelled and the base sent
        zero once a scan shows a return inside the floor or the room the way
        it moves within its stopping distance (``braking.leg_floor_check``).
        """
        with self._condition:
            if call_id in self._guards:
                return  # a resumed call keeps the guard it started with
            guard = _BrakingGuard(
                capability_id=capability_id,
                profile=profile,
                direction_rad=0.0,
                speed_mps=0.0,
                requested_speed_mps=0.0,
                distance_m=0.0,
                start_room_m=None,
                start_pose=dict(self._pose) if self._pose is not None else None,
                kind=GUARD_KIND_LEG,
            )
            _remember(self._guards, call_id, guard)

    def _check_leg_guard(self, guard: _BrakingGuard) -> None:
        """Called under the condition after each scan: judge one navigation leg."""
        linear = self._velocity[0] if self._velocity is not None else 0.0
        check = braking.leg_floor_check(self._range_sweep, guard.profile, linear)
        if check is None:
            self._trip_guard(guard, motion_outcome.GUARD_TRIP_BLIND, {
                "detail": "the LiDAR sweep could not be read",
            })
            return
        if guard.minimum_range_m is None or check.nearest_m < guard.minimum_range_m:
            guard.minimum_range_m = check.nearest_m
        if check.breached:
            self._trip_guard(guard, motion_outcome.GUARD_TRIP_FLOOR, check.to_dict())

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
            if guard.kind == GUARD_KIND_LEG:
                self._check_leg_guard(guard)
                continue
            room = braking.room_to_floor(
                self._range_sweep, guard.direction_rad, guard.profile.floor_m
            )
            if room is None:
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_BLIND, {
                    "detail": "the LiDAR sweep could not be read",
                })
                continue
            guard.last_room_m = room.room_m
            if guard.minimum_room_m is None or room.room_m < guard.minimum_room_m:
                guard.minimum_room_m = room.room_m
            # The robot may still be faster than the last command asked for.
            speed = max(guard.speed_mps, measured)
            stopping = guard.profile.stopping_distance(speed)
            allowed = guard.profile.max_speed(room.room_m) * SLOWDOWN_FRACTION
            # A drive that can be slowed is stopped once even the slowest
            # speed no longer fits; one that cannot is stopped only when its
            # own speed no longer fits (its distance was planned to end first).
            cannot_slow = guard.resends and allowed < braking.MIN_SPEED_MPS
            if room.room_m <= stopping or cannot_slow:
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_CLEARANCE, {
                    "room_m": round(room.room_m, 4),
                    "speed_mps": round(speed, 4),
                    "stopping_distance_m": round(stopping, 4),
                    "required_clearance_m": round(guard.profile.required_clearance(speed), 4),
                    **room.to_dict(),
                })
                continue
            if (
                guard.resends
                and allowed < guard.speed_mps * SLOWDOWN_STEP
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
            seen = self._range_seen_at
            age = None if seen is None else time.monotonic() - seen
            if age is not None and age <= GUARD_POLL_S:
                return  # a scan just arrived; its own check has judged it
            measured = abs(self._velocity[0]) if self._velocity is not None else 0.0
            speed = max(guard.speed_mps, measured)
            nominal = self._latency.nominal_scan_period_s
            worst = self._latency.scan_period_s
            stale = braking.scan_stale_limit(
                SCAN_STALENESS[guard.kind],
                configured_s=_scan_timeout_s(),
                nominal_period_s=nominal,
                worst_period_s=worst,
                profile=guard.profile,
                room_m=guard.last_room_m,
                speed_mps=speed,
                poll_s=GUARD_POLL_S,
            )
            if age is None or age > stale.limit_s:
                stamp = self._range_stamp_s
                self._trip_guard(guard, motion_outcome.GUARD_TRIP_STALE, {
                    "scan_age_s": None if age is None else round(age, 3),
                    **stale.to_dict(),
                    "nominal_period_s": None if nominal is None else round(nominal, 4),
                    "worst_period_s": None if worst is None else round(worst, 4),
                    "room_m": (
                        round(guard.last_room_m, 4)
                        if guard.last_room_m is not None and math.isfinite(guard.last_room_m)
                        else None
                    ),
                    "speed_mps": round(speed, 4),
                    # By the robot's clock: tells a sensor that stopped from a
                    # transport that held its scans (clock skew included).
                    # None when the stamp is not wall time (a simulator clock).
                    "scan_stamp_age_s": _stamp_age_s(stamp),
                })

    def _guard(self, call_id: str) -> _BrakingGuard | None:
        with self._condition:
            return self._guards.get(call_id)

    def _track_range(self, minimum_range_m: float) -> None:
        """Called under the condition with each LiDAR minimum."""
        for track in self._motion_tracks.values():
            track.saw_range(minimum_range_m)

    def _track_command(self, topic: str, message: Mapping[str, Any]) -> None:
        """A velocity command seen on a watched topic (see ``_command_watch_topics``)."""
        command = _twist_command(message)
        if command is None:
            return
        with self._condition:
            for track in self._motion_tracks.values():
                track.saw_command(topic, *command)

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
                track.saw_feedback(values, time.monotonic())
            navigation = self._navigation
            if navigation is not None and navigation.call_id == call_id:
                navigation.saw_feedback(values, time.monotonic())

    def _end_navigation(self, call_id: str, reason: str, *, reached: bool = False) -> None:
        """Record how the navigation goal ``call_id`` ended, if it is the latest."""
        with self._condition:
            navigation = self._navigation
            if (
                navigation is not None
                and navigation.call_id == call_id
                and navigation.state == "navigating"
            ):
                navigation.end(reason, time.monotonic(), reached=reached)

    def _store_battery(self, percentage: Any, voltage: Any, observed: float) -> None:
        """A ``sensor_msgs/BatteryState`` reading; never a graph event."""
        with self._condition:
            self._battery = live_telemetry.battery_payload(percentage, voltage)
            self._battery_seen_at = observed

    def telemetry_readings(self) -> dict[str, Any]:
        """What this connection currently knows, for ``live_telemetry.samples``."""
        with self._condition:
            snapshot = self._snapshot()
            now = time.monotonic()
            battery_fresh = (
                self._battery is not None
                and self._battery_seen_at is not None
                and now - self._battery_seen_at <= live_telemetry.BATTERY_MAX_AGE_S
            )
            return {
                "battery": dict(self._battery) if battery_fresh else None,
                "battery_seen_at": self._battery_seen_at if battery_fresh else None,
                "pose": snapshot.get("pose"),
                "map_pose": snapshot.get("map_pose"),
                "pose_seen_at": self._pose_seen_at,
                "navigation": self._navigation.copy() if self._navigation is not None else None,
            }

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
                if (
                    status == motion_outcome.STATUS_SUCCEEDED
                    and guard.tripped is None
                    and guard.plan is not None
                    and guard.plan.shortened
                ):
                    # The drive ended where it was planned to, short of the
                    # request, because going on would have broken the floor.
                    guard.ended = motion_outcome.GUARD_END_PLANNED_STOP
                track.braking = guard.record(snapshot.get("pose"))
        summary = motion_outcome.summarize(
            track,
            status=status,
            result_values=result_values,
            end_pose=snapshot.get("pose"),
            end_map_pose=snapshot.get("map_pose"),
            range_observation=snapshot.get("range"),
            clearance_floor_m=_clearance_floor(),
            ended_at=time.monotonic(),
        )
        if status is not None:
            self._end_navigation(
                call_id,
                str(summary.get("reason") or ""),
                reached=status == motion_outcome.STATUS_SUCCEEDED or _rule_reached(summary),
            )
        return summary

    def _forget_track(self, call_id: str) -> None:
        with self._condition:
            self._motion_tracks.pop(call_id, None)
            self._guards.pop(call_id, None)
        # A goal dropped before its result came back was cancelled or stopped.
        self._end_navigation(call_id, motion_outcome.REASON_CANCELLED)

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
        twist: TwistAxes | None,
        observed: float,
    ) -> None:
        velocity = (
            (twist[0], twist[5])
            if twist is not None and twist[0] is not None and twist[5] is not None
            else None
        )
        with self._condition:
            self._pose = pose
            self._twist_axes = twist
            self._velocity = velocity  # type: ignore[assignment]
            self._pose_seen_at = observed
            self._odom_sequence += 1
            for track in self._motion_tracks.values():
                track.saw_base(velocity, observed, pose)
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
            self._twist_axes = None
            self._battery_seen_at = None
            self._battery = None
            self._range_seen_at = None
            self._range_stamp_s = None
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

    def latest_odometry(self) -> dict[str, Any] | None:
        """The newest odometry message on this connection, without waiting.

        ``{"pose", "sequence", "age_s"}``: the pose, the message's number on
        this backend (it only ever grows, so an unchanged number means no new
        message), and how long ago this host received it. None before the
        first message on this connection or after a drop.
        """
        with self._condition:
            if self._pose is None or self._pose_seen_at is None:
                return None
            return {
                "pose": dict(self._pose),
                "sequence": self._odom_sequence,
                "age_s": max(0.0, time.monotonic() - self._pose_seen_at),
            }

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

    def wait_until_stationary(
        self,
        max_seconds: float = 1.0,
        *,
        samples: int = STILL_SAMPLES,
        min_seconds: float | None = None,
    ) -> dict[str, Any]:
        """Return once odometry shows the robot still (see ``STILL_SAMPLES``).

        Only odometry received after the call counts. A run of still messages
        (full twist near zero, pose within the deltas of the run's first
        message) is stationary once it has ``samples`` messages spanning
        ``min_seconds`` (default ``STILL_MIN_SECONDS``) of receive time. A
        message that is not still ends the run. ``max_seconds`` is only the
        cap: a robot not shown still by then is reported as drifting.
        """
        needed = max(1, int(samples))
        span = STILL_MIN_SECONDS if min_seconds is None else max(0.0, float(min_seconds))
        started = time.monotonic()
        deadline = started + max(0.0, float(max_seconds))
        still = 0
        still_for = 0.0
        messages = 0
        unknown_twist = 0
        anchor: dict[str, Any] | None = None
        anchor_at = 0.0
        with self._condition:
            seen = self._odom_sequence
            while self._connected and not (still >= needed and still_for >= span):
                if self._odom_sequence == seen:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(timeout=remaining)
                    continue
                messages += self._odom_sequence - seen
                seen = self._odom_sequence
                pose, at = self._pose, self._pose_seen_at
                twist = self._twist_axes
                if twist is None or any(value is None for value in twist):
                    unknown_twist += 1
                if pose is None or at is None or not _twist_still(twist):
                    still, still_for, anchor = 0, 0.0, None
                    continue
                if anchor is None or not _pose_within(anchor, pose):
                    # A new still run starts here; one that moved is not still.
                    anchor, anchor_at, still, still_for = dict(pose), at, 1, 0.0
                    continue
                still += 1
                still_for = at - anchor_at
        stationary = still >= needed and still_for >= span
        return {
            "stationary": stationary,
            "drifting": not stationary,
            "still_samples": still,
            "still_seconds": round(still_for, 3),
            "min_still_seconds": span,
            # Odometry messages received during the wait: 0 means the robot
            # was not observed at all, which is never taken as still.
            "odometry_messages": messages,
            # Messages whose twist lacked an axis: never still, whatever the pose.
            "unknown_twist_messages": unknown_twist,
            "waited_seconds": round(time.monotonic() - started, 3),
        }

    # -- motion.halt: completed only when the robot is observed still ----------

    def _active_goal_ids(self) -> tuple[str, ...]:
        raise NotImplementedError

    def _goal_active(self, call_id: str) -> bool:
        raise NotImplementedError

    def _cancel_for_halt(self, call_id: str) -> str | None:
        """None once the goal is over (cancel confirmed, or it ended); else why not.

        Never raises: a cancel that could not be sent is a reason, not an error.
        """
        raise NotImplementedError

    def _release_goal(self, call_id: str) -> None:
        raise NotImplementedError

    def _cancel_token(self, call_id: str) -> Any:
        """``(goal id, action name)``: what cancels this goal, and confirms its end,
        from a later connection or a restarted host process."""
        raise NotImplementedError

    def _cancel_all_goals(self, action_name: str) -> None:
        """Ask ``action_name``'s own cancel service to cancel every goal it runs.

        The zero goal id with a zero stamp: every goal the server has accepted
        or is executing. Sent, not awaited (the answer proves nothing; see
        ``_held_goal_end``). May raise when nothing can be sent.
        """
        raise NotImplementedError

    def _goal_statuses_until(
        self,
        action_name: str,
        settled: Callable[[list[int]], bool],
        timeout: float,
    ) -> list[int] | None:
        """Read ``action_name``'s goal status array until ``settled`` holds or time runs out.

        The newest statuses seen (``goal_statuses``), or None when no status
        message arrived. The status topic is transient-local, so a server
        that has run a goal answers a late reader at once.
        """
        raise NotImplementedError

    def _recancel(self, call_id: str, token: Any) -> None:
        """Re-send a held or running goal's cancel without waiting (may raise).

        Through the action server's own cancel service by action name: a
        rosbridge goal id belongs to the socket that sent it and an rclpy
        handle to the node that sent it, and neither outlives a reconnect.
        """
        action_name = token_action(token)
        if action_name is None:
            raise LookupError(f"no action name is kept for {call_id}")
        self._cancel_all_goals(action_name)

    def _held_goal_end(self, call_id: str, token: Any) -> str | None:
        """None once the action server's status shows the held goal has ended; else why not.

        Asked of goals held from an earlier connection (or host process),
        whose cancel this connection cannot follow: a robot seen still does
        not show the goal is over, and its server could resume it. Only the
        goal status confirms it: no goal on that action server may be
        ACCEPTED, EXECUTING or CANCELING. A cancel response never does: a
        server answers a cancel of a CANCELING goal with no error and no
        goal canceling, while it still drives that goal.
        """
        action_name = token_action(token)
        if action_name is None:
            return "no action name is kept for it, so nothing can confirm its end"
        statuses = self._goal_statuses_until(
            action_name,
            lambda seen: not live_goal_statuses(seen),
            HELD_GOAL_END_WAIT_S,
        )
        if statuses is None:
            return f"no goal status from {action_name} within {HELD_GOAL_END_WAIT_S:g} s"
        live = live_goal_statuses(statuses)
        if live:
            return (
                f"{action_name} still has {len(live)} goal(s) not ended "
                f"(status {sorted(set(live))})"
            )
        return None

    def _publish_zero(self) -> bool:
        raise NotImplementedError

    def _zero_published(self) -> bool:
        """Whether zero velocity went out. Never raises: a transport error
        (RuntimeError, OSError, a closed websocket) is a stop not published."""
        try:
            return bool(self._publish_zero())
        except Exception:  # noqa: BLE001 - an unsent stop is unpublished, not an error
            return False

    def halt_hold(self) -> dict[str, Any] | None:
        """The robot's halt hold (see ``HALT_HOLD``), or None when none is armed."""
        return HALT_HOLD.snapshot()

    def _halt(self, call_id: str, evidence: Mapping[str, Any]) -> CallResult:
        """Stop the base, and say ``completed`` only when the robot is seen still.

        Completed means every running goal's cancel was confirmed (or the goal
        ended), zero velocity was published, fresh odometry then shows the
        robot still (full twist and pose, for ``STILL_MIN_SECONDS``), and
        every goal in the hold (from an earlier connection: a reconnect
        carries running goals into it) is confirmed ended by its action
        server's goal status. No goal being tracked is not enough: something
        else may drive the base. Anything less is ``failed`` with the typed
        reason ``effect_unknown`` and arms ``HALT_HOLD``: a goal whose cancel
        was refused or went unanswered keeps its handle, zero velocity is
        re-sent, and motion is refused until a halt sees the robot still.

        The whole halt runs in ``HALT_HOLD.halting()``: no goal is sent while
        it runs, and nothing admitted before it is sent after it.
        """
        with HALT_HOLD.halting():
            try:
                return self._halt_in_epoch(call_id, evidence)
            except Exception as error:  # noqa: BLE001 - a halt never fails open
                return self._halt_raised(call_id, evidence, error)

    def _halt_raised(
        self, call_id: str, evidence: Mapping[str, Any], error: BaseException
    ) -> CallResult:
        """A halt that raised: ``effect_unknown``, the hold armed, zero held."""
        why = f"the halt raised {type(error).__name__}: {error}"[:200]
        return hold_unrun_halt(self, call_id, why, evidence, confirmation={"error": why})

    def _running_goal_ids(self) -> list[str]:
        """Goals this connection sent that may still run (sent, accepted, not ended)."""
        return [goal for goal in self._active_goal_ids() if self._goal_active(goal)]

    def _hold_running_goals(self) -> None:
        """Before a reconnect forgets this connection's goals: hold every running one.

        Its goal ids (rosbridge) or handles (rclpy) are about to be dropped, so
        a later halt would not see the goal at all. The hold keeps each one
        with its cancel token: the reconnect re-sends the cancel, motion is
        refused, and only a halt that sees the robot still AND confirms the
        goal ended (``_held_goal_end``) releases it. The caller holds
        ``HALT_HOLD.forgetting()`` from before this until the goals are
        forgotten, so no goal is sent and recorded in between. Never raises.
        """
        tokens: dict[str, Any] = {}
        with contextlib.suppress(Exception):
            tokens = {goal: self._cancel_token(goal) for goal in self._running_goal_ids()}
        if tokens:
            HALT_HOLD.arm(
                "a reconnect dropped goals still running: " + ", ".join(sorted(tokens)),
                tokens,
            )

    def _halt_in_epoch(self, call_id: str, evidence: Mapping[str, Any]) -> CallResult:
        # Zero velocity goes out first, so the stop never waits behind a cancel.
        stopped = self._zero_published()
        active = self._active_goal_ids()
        tokens = {goal: self._cancel_token(goal) for goal in active}
        cancelled: list[str] = []
        unconfirmed: dict[str, str] = {}
        for goal in active:
            why = self._cancel_for_halt(goal)
            if why is None:
                self._release_goal(goal)
                cancelled.append(goal)
            else:
                unconfirmed[goal] = why[:200]
        # Goals held from an earlier connection: their cancel is sent again,
        # unanswerable here, so the robot's stillness decides.
        reissued = self._recancel_held(skip=active)
        if active or reissued:
            # Until its cancel landed the action server could overwrite the
            # first zero; the last command on cmd_vel must be this stop's.
            stopped = self._zero_published() or stopped
        confirmation: dict[str, Any] = {
            "active_goals": sorted(active),
            "cancelled": cancelled,
            "unconfirmed": unconfirmed,
        }
        if reissued:
            confirmation["reissued_cancels"] = reissued
        evidence = {
            "adapter": "generic_ros2",
            **evidence,
            "standard_interface": {
                "kind": "topic",
                "name": DEFAULT_INTERFACES["motion.halt"][1],
                "type": "geometry_msgs/msg/Twist|TwistStamped",
            },
            "zero_velocity_published": stopped,
        }

        def unconfirmed_halt(why: str) -> CallResult:
            HALT_HOLD.arm(why, {goal: tokens.get(goal) for goal in unconfirmed})
            self._hold_zero()
            confirmation["zero_velocity_held"] = True
            confirmation["hold"] = HALT_HOLD.snapshot()
            return self._halt_unconfirmed(call_id, evidence, confirmation, why)

        if unconfirmed:
            return unconfirmed_halt("cancel not confirmed for " + ", ".join(sorted(unconfirmed)))
        if not stopped:
            return unconfirmed_halt("standard cmd_vel stop unavailable")
        stillness = self.wait_until_stationary(HALT_STILL_WAIT_S)
        confirmation["stillness"] = stillness
        if not stillness["stationary"]:
            if not stillness.get("odometry_messages"):
                return unconfirmed_halt(f"no fresh odometry within {HALT_STILL_WAIT_S:g} s")
            return unconfirmed_halt(
                f"odometry did not show the robot still within {HALT_STILL_WAIT_S:g} s"
            )
        # Goals held from an earlier connection were never cancelled here:
        # the hold is released only once their action server's goal status
        # shows they ended, or Nav2 could resume one after the release.
        held = {goal: token for goal, token in HALT_HOLD.tokens().items() if goal not in cancelled}
        unended: dict[str, str] = {}
        # One status read per action server answers for every goal held on it.
        ends: dict[str, str | None] = {}
        for goal, token in held.items():
            action_name = token_action(token)
            if action_name is not None and action_name in ends:
                why = ends[action_name]
            else:
                try:
                    why = self._held_goal_end(goal, token)
                except Exception as error:  # noqa: BLE001 - an unanswered end is unconfirmed
                    why = f"end not confirmed: {type(error).__name__}: {error}"
                if action_name is not None:
                    ends[action_name] = why
            if why is not None:
                unended[goal] = why[:200]
        if held:
            confirmation["held_goals_ended"] = sorted(set(held) - set(unended))
        if unended:
            confirmation["held_goals_unconfirmed"] = unended
            return unconfirmed_halt(
                "no confirmed end for held goals " + ", ".join(sorted(unended))
            )
        released = HALT_HOLD.release()
        if released is not None:
            confirmation["released_hold"] = released
        self._count_execution(call_id)
        return self._keep_result(
            call_id,
            CallResult(
                call_id,
                OUTCOME_COMPLETED,
                evidence={**evidence, "halt_confirmation": confirmation},
            ),
        )

    @staticmethod
    def _halt_unconfirmed(
        call_id: str,
        evidence: Mapping[str, Any],
        confirmation: Mapping[str, Any],
        why: str,
    ) -> CallResult:
        return CallResult(
            call_id,
            OUTCOME_FAILED,
            evidence={
                **evidence,
                "reason_code": REASON_EFFECT_UNKNOWN,
                "effect_unknown": True,
                "halt_confirmation": dict(confirmation),
            },
            detail=f"{REASON_EFFECT_UNKNOWN}: halt not confirmed: {why}",
        )

    def _recancel_held(self, *, skip: Iterable[str] = ()) -> list[str]:
        """Re-send the cancel of every held goal not in ``skip``; the ids it went out for."""
        skipped = set(skip)
        sent: list[str] = []
        for goal, token in HALT_HOLD.tokens().items():
            if goal in skipped:
                continue
            try:
                self._recancel(goal, token)
            except Exception:  # noqa: BLE001 - an unsent cancel is reported by omission
                continue
            sent.append(goal)
        return sorted(sent)

    def release_hold(self, *, operator: str, reason: str) -> dict[str, Any]:
        """An operator's release of the hold, for a held goal no halt can confirm ended.

        The way out when ``_held_goal_end`` cannot answer (an action server
        that is gone, a hold with no action name kept, a status topic that
        never answers) while the operator has checked the robot. It still
        needs the robot seen still: zero velocity is published, the held
        goals' cancels re-sent, and fresh odometry must show the robot still
        (``wait_until_stationary``) or nothing is released. A release is
        recorded with who released it and why (``_HaltHold.operator_release``)
        before the hold is cleared; a record that cannot be written releases
        nothing. Never raises.
        """
        operator = str(operator or "").strip()[:200]
        reason = str(reason or "").strip()[:200]
        if not operator or not reason:
            return {"released": False, "detail": "an operator and a reason are required"}
        if getattr(self, "_presence_only", False):
            return {
                "released": False,
                "detail": "a presence connection never commands the robot",
            }
        with HALT_HOLD.halting():
            if HALT_HOLD.snapshot() is None:
                return {"released": False, "detail": "no halt hold is armed"}
            try:
                if not self._zero_published():
                    return {"released": False, "detail": "standard cmd_vel stop unavailable"}
                reissued = self._recancel_held()
                self._zero_published()
                stillness = self.wait_until_stationary(HALT_STILL_WAIT_S)
                if not stillness.get("stationary"):
                    return {
                        "released": False,
                        "detail": "odometry did not show the robot still; the hold stays",
                        "stillness": stillness,
                    }
                record = HALT_HOLD.operator_release(
                    operator=operator,
                    reason=reason,
                    evidence={"stillness": stillness, "reissued_cancels": reissued},
                )
            except Exception as error:  # noqa: BLE001 - a failed release keeps the hold
                return {
                    "released": False,
                    "detail": f"release raised {type(error).__name__}: {error}"[:200],
                }
        if record is None:
            return {"released": False, "detail": "no halt hold is armed"}
        return {"released": True, "record": record}

    def _resume_hold(self) -> None:
        """On a (re)connection while the robot is held: zero, the held cancels, the hold.

        A presence-only connection never commands, so it only refuses.
        """
        if getattr(self, "_presence_only", False) or HALT_HOLD.snapshot() is None:
            return
        if not self._connected:
            return
        self._zero_published()
        self._recancel_held()
        self._zero_published()
        self._hold_zero()

    def _hold_zero(self) -> None:
        """Keep re-sending zero velocity on this connection while the robot is held."""
        with self._hold_lock:
            if self._zero_holder is not None and self._zero_holder.is_alive():
                return
            holder = threading.Thread(
                target=self._zero_hold_loop, name="halt-zero-hold", daemon=True
            )
            self._zero_holder = holder
        holder.start()

    def _zero_hold_loop(self) -> None:
        pause = threading.Event()
        while True:
            # The halt has just sent its own zero; the hold follows it.
            pause.wait(ZERO_HOLD_PERIOD_S)
            with self._hold_lock:
                if HALT_HOLD.snapshot() is None or not self._connected:
                    self._zero_holder = None
                    return
            self._zero_published()


def hold_unrun_halt(
    backend: Any,
    call_id: str,
    why: str,
    evidence: Mapping[str, Any] | None = None,
    *,
    confirmation: Mapping[str, Any] | None = None,
) -> CallResult:
    """A halt that could not run (or raised): hold, cancel, keep zero going, ``effect_unknown``.

    The one implementation for every halt that does not run to the end: no
    time, the link down, ``motion.halt`` missing, an operator refresh, a
    presence connection, a halt that raised. The hold is armed with every
    running goal's cancel token, so a host restart keeps them. Where the
    connection commands, every running and held goal's cancel is re-sent at
    once: zero alone is not a stop while a goal's controller keeps publishing
    faster than the hold re-sends zero. Zero is published once and then held
    (a presence connection never commands, a dropped one resumes on
    reconnect). The caller runs it inside ``HALT_HOLD.halting()``, so a new
    halt epoch starts too. Never raises, on any backend.
    """
    goals: dict[str, Any] = {}
    with contextlib.suppress(Exception):
        goals = {goal: backend._cancel_token(goal) for goal in backend._active_goal_ids()}
    HALT_HOLD.arm(why, goals)
    published = False
    reissued: list[str] = []
    if not getattr(backend, "_presence_only", False):
        with contextlib.suppress(Exception):
            published = bool(backend._publish_zero())
        for goal, token in goals.items():
            try:
                backend._recancel(goal, token)
            except Exception:  # noqa: BLE001 - an unsent cancel is reported by omission
                continue
            reissued.append(goal)
        with contextlib.suppress(Exception):
            reissued.extend(backend._recancel_held(skip=goals))
        if reissued:
            # The last command on cmd_vel is this stop's, not a goal's.
            with contextlib.suppress(Exception):
                published = bool(backend._publish_zero()) or published
        with contextlib.suppress(Exception):
            backend._hold_zero()
    held = {
        **dict(confirmation or {}),
        "zero_velocity_held": published,
        "hold": HALT_HOLD.snapshot(),
    }
    if reissued:
        held["reissued_cancels"] = sorted(set(reissued))
    return _ObservationState._halt_unconfirmed(
        call_id,
        {
            "adapter": "generic_ros2",
            **dict(evidence or {}),
            "zero_velocity_published": published,
        },
        held,
        why,
    )


#: Names the file that keeps the halt hold across a restart of the host process.
HALT_HOLD_FILE_ENV = "FLYTO_ROS2_HALT_HOLD_FILE"
HALT_HOLD_SCHEMA = "flyto.ros2-halt-hold.v1"
HALT_HOLD_RELEASE_SCHEMA = "flyto.ros2-halt-hold-release.v1"
_MAX_ADMITTED = 512


def halt_hold_path() -> Path:
    """Where the hold is kept: ``$FLYTO_ROS2_HALT_HOLD_FILE``, else
    ``<data dir>/halt-hold/<resource id>.json`` (``places.data_dir``)."""
    explicit = os.getenv(HALT_HOLD_FILE_ENV, "").strip()
    if explicit:
        return Path(explicit).expanduser()
    return places.data_dir() / "halt-hold" / f"{_safe_fragment(configured_resource_id())}.json"


def _token_to_json(token: Any) -> Any:
    # A rosbridge cancel token (goal id, action) outlives the process; an rclpy
    # goal handle does not, so a restarted host only re-sends rosbridge cancels.
    if isinstance(token, tuple) and len(token) == 2 and all(
        isinstance(part, str) for part in token
    ):
        return list(token)
    return None


def _token_from_json(raw: Any) -> Any:
    if isinstance(raw, list) and len(raw) == 2 and all(isinstance(part, str) for part in raw):
        return (raw[0], raw[1])
    return None


class _HaltHold:
    """The robot's hold after a halt that did not see it stop, and the halt epoch.

    One per process: an adapter host serves one robot (``configured_resource_id``),
    and every connection to it (a reconnect, a per-job adapter, a presence
    watch) reads the same hold, so dropping a connection never re-allows
    motion. Armed by every ``motion.halt`` that ends ``effect_unknown``;
    released only by a halt that then sees the robot still and confirms every
    held goal ended, or by an operator's recorded release
    (``operator_release``) on a robot seen still. Deleting the file while
    the host runs releases nothing: the hold is read once and kept in memory,
    and the next arm writes it again. While armed,
    motion is refused and each commanding connection re-sends zero velocity
    and the held goals' cancels.

    The hold is also kept in a file (``halt_hold_path``), written when it is
    armed and removed when it is released, so a host process that restarts
    starts held. A file that cannot be read holds as well.

    Admission: a motion is admitted (``admit``) when the adapter accepts the
    call, in the current halt epoch. Every halt (``halting``) starts a new
    epoch, and every goal send runs under ``sending``, which refuses while
    the hold is armed, while a halt is in progress, or when a halt started
    after the motion was admitted. ``halting`` takes the same lock as
    ``sending`` before the halt reads the running goals, so a goal is either
    sent before the halt looks (and the halt cancels it) or not at all.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        # Serialises a goal send (send + record it as running) against a halt's start.
        self._gate = threading.RLock()
        self._reason: str | None = None
        self._goals: dict[str, Any] = {}
        self._epoch = 0
        self._halting = 0
        self._admitted: dict[str, int] = {}
        self._loaded_from: Path | None = None
        self._persist_error: str | None = None

    # -- persistence ------------------------------------------------------------

    def _load_locked(self) -> None:
        """Read the kept hold once per file; a hold in it is armed here too."""
        path = halt_hold_path()
        if path == self._loaded_from:
            return
        self._loaded_from = path
        try:
            raw = path.read_text(encoding="utf-8")
        except FileNotFoundError:
            return
        except OSError as error:
            self._arm_locked(f"the kept halt hold {path} cannot be read: {error}", {})
            return
        try:
            document = json.loads(raw)
            if not isinstance(document, dict) or document.get("schema") != HALT_HOLD_SCHEMA:
                raise ValueError("not a halt hold record")
            reason = document["reason"]
            goals = document.get("goals")
            if not isinstance(reason, str) or not isinstance(goals, dict):
                raise ValueError("malformed halt hold record")
        except (ValueError, KeyError) as error:
            self._arm_locked(f"the kept halt hold {path} cannot be read: {error}", {})
            return
        self._arm_locked(
            f"kept from before a restart: {reason}",
            {str(goal): _token_from_json(token) for goal, token in goals.items()},
        )

    def _arm_locked(self, reason: str, goals: Mapping[str, Any]) -> None:
        self._reason = str(reason)[:200]
        self._goals.update(goals)
        document = {
            "schema": HALT_HOLD_SCHEMA,
            "reason": self._reason,
            "goals": {goal: _token_to_json(token) for goal, token in self._goals.items()},
        }
        path = self._loaded_from or halt_hold_path()
        try:
            fsio.atomic_write(path, json.dumps(document, separators=(",", ":")))
            self._persist_error = None
        except OSError as error:
            # Still held in this process; the snapshot says it is not kept.
            self._persist_error = f"{type(error).__name__}: {error}"[:200]
            logger.error("halt hold could not be kept in %s: %s", path, error)

    # -- the hold ---------------------------------------------------------------

    def arm(self, reason: str, goals: Mapping[str, Any]) -> None:
        with self._lock:
            self._load_locked()
            self._arm_locked(reason, goals)

    def snapshot(self) -> dict[str, Any] | None:
        with self._lock:
            self._load_locked()
            if self._reason is None:
                return None
            held: dict[str, Any] = {"reason": self._reason, "goals": sorted(self._goals)}
            if self._persist_error is not None:
                held["kept_across_restart"] = False
                held["keep_error"] = self._persist_error
            return held

    def tokens(self) -> dict[str, Any]:
        with self._lock:
            self._load_locked()
            return dict(self._goals)

    def release(self) -> dict[str, Any] | None:
        """Clear the hold; what it held, or None. Only a halt seen still calls it."""
        with self._lock:
            self._load_locked()
            if self._reason is None:
                return None
            released = {"reason": self._reason, "goals": sorted(self._goals)}
            path = self._loaded_from or halt_hold_path()
            try:
                path.unlink(missing_ok=True)
            except OSError as error:
                # A restart would hold again; that is the safe side.
                logger.error("kept halt hold %s could not be removed: %s", path, error)
            self._reason = None
            self._goals = {}
            self._persist_error = None
            return released

    # -- admission and the halt epoch ---------------------------------------------

    def admit(self, call_id: str, *, like: str | None = None) -> None:
        """Admit a motion in the current halt epoch (or in ``like``'s, for its legs)."""
        with self._lock:
            # A leg of a call nobody admitted is never current (-1).
            epoch = self._epoch if like is None else self._admitted.get(like, -1)
            self._admitted.pop(call_id, None)
            self._admitted[call_id] = epoch
            while len(self._admitted) > _MAX_ADMITTED:
                self._admitted.pop(next(iter(self._admitted)))

    def admit_if_new(self, call_id: str) -> None:
        """Admit a call sent straight to a backend (no adapter admitted it)."""
        with self._lock:
            known = call_id in self._admitted
        if not known:
            self.admit(call_id)

    def motion_refusal(self, call_id: str) -> str | None:
        """Why this admitted motion may not send a goal now, or None."""
        with self._lock:
            self._load_locked()
            if self._reason is not None:
                # The base may still be moving, or driven by a goal nobody
                # confirmed stopped; a reconnect or a restart does not change that.
                goals = ", ".join(sorted(self._goals)) or "no tracked goal"
                return (
                    f"{REASON_EFFECT_UNKNOWN}: a halt did not see the robot stop "
                    f"({self._reason}; held: {goals}); motion is refused until a "
                    "halt sees it still"
                )
            if self._halting:
                return "a halt is in progress; motion is refused until it ends"
            if self._admitted.get(call_id, -1) != self._epoch:
                return "a halt ran after this motion was admitted; it is not sent"
            return None

    @contextlib.contextmanager
    def sending(self, call_id: str) -> Iterator[str | None]:
        """Hold the send gate; yields why the goal may not be sent, or None.

        The caller sends the goal and records it as running inside the block.
        """
        with self._gate:
            yield self.motion_refusal(call_id)

    @contextlib.contextmanager
    def forgetting(self) -> Iterator[None]:
        """A connection forgetting its goals: no goal is sent and recorded meanwhile.

        Holds the same gate as ``sending``. A reconnect holds the running
        goals and forgets them inside it, so a goal is either recorded before
        the hold reads them (and is held) or sent after (on the new
        connection, where a halt sees it).
        """
        with self._gate:
            yield

    def operator_release(
        self, *, operator: str, reason: str, evidence: Mapping[str, Any]
    ) -> dict[str, Any] | None:
        """Clear the hold on an operator's word; the release record, or None.

        Only ``_ObservationState.release_hold`` calls it, after the robot was
        seen still. The record (who, why, what was held, the evidence) is
        appended to ``<hold file stem>.releases.jsonl`` beside the hold file
        and logged before the hold is cleared; if it cannot be written, this
        raises and nothing is released.
        """
        with self._lock:
            self._load_locked()
            if self._reason is None:
                return None
            record = {
                "schema": HALT_HOLD_RELEASE_SCHEMA,
                "released_at": datetime.now(timezone.utc).isoformat(),
                "operator": operator,
                "reason": reason,
                "held": {"reason": self._reason, "goals": sorted(self._goals)},
                "evidence": dict(evidence),
            }
            path = self._loaded_from or halt_hold_path()
            audit = path.with_name(f"{path.stem}.releases.jsonl")
            audit.parent.mkdir(parents=True, exist_ok=True)
            with audit.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, separators=(",", ":"), default=str) + "\n")
            logger.warning(
                "halt hold released by operator %s: %s (held: %s)",
                operator,
                reason,
                record["held"],
            )
            try:
                path.unlink(missing_ok=True)
            except OSError as error:
                logger.error("kept halt hold %s could not be removed: %s", path, error)
            self._reason = None
            self._goals = {}
            self._persist_error = None
            return record

    @contextlib.contextmanager
    def halting(self) -> Iterator[None]:
        """A halt in progress: a new epoch, and no goal is sent until it ends."""
        with self._gate, self._lock:
            self._epoch += 1
            self._halting += 1
        try:
            yield
        finally:
            with self._lock:
                self._halting -= 1


#: The robot's halt hold; see ``_HaltHold``.
HALT_HOLD = _HaltHold()


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
    """The call's arguments validated against ``declared_arguments``; text is a place name."""
    schemas = {item.name: item for item in declared_arguments(capability_id)}
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


def _succeeded_outcome(summary: Mapping[str, Any] | None) -> str:
    """The outcome of a goal its server ended as succeeded, or one the stop rule settled.

    The server's "succeeded" is about the goal it was sent, which speed
    governance may have shortened to a few centimetres. Whether the call did
    what was asked is ``motion_outcome``'s reason, decided from the motion's
    facts (``motion_outcome.STOP_RULES``): anything but ``completed`` -- an
    ``obstacle_blocked`` drive planned or tripped short of the request, a
    ``path_blocked`` one with no room at all -- is a failed call, so a caller
    plans a way round instead of trusting an arrival that did not happen.
    """
    if not isinstance(summary, Mapping):
        return OUTCOME_COMPLETED
    reason = summary.get("reason")
    return OUTCOME_COMPLETED if reason == motion_outcome.REASON_COMPLETED else OUTCOME_FAILED


def _rule_reached(summary: Mapping[str, Any] | None) -> bool:
    """A planned or guarded stop the stop rule found still arrived."""
    rule = summary.get("stop_rule") if isinstance(summary, Mapping) else None
    return isinstance(rule, Mapping) and rule.get("reason") == motion_outcome.REASON_COMPLETED


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
        transport: ssh_transport.ManagedTunnel | None = None,
    ):
        self.backend = backend or RclpyROS2Backend()
        # The managed SSH forward this adapter holds (None: the link is direct,
        # rclpy or a rosbridge URL someone else keeps reachable). Calls are
        # admitted through its gate, shared by every adapter on that forward,
        # so an operator refresh can stop them all before tearing it down.
        self.transport = transport
        self._transport_held = transport is not None
        self._gate = transport.calls if transport is not None else ssh_transport.CallGate()
        self._refresh_lock = threading.Lock()
        self._last_link_error = ""
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
        # The behavior plugin class the server loaded per action, once read.
        self._server_plugins: dict[str, str | None] = {}
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
        if self._cargo_interface() in interfaces:
            available.add(cargo_sensor.CAPABILITY_ID)
        return available

    def _cargo_interface(self) -> tuple[str, str, str] | None:
        """The cargo topic, when this transport can read it; else None.

        The backend names it from configuration and its graph
        (``cargo_sensor.served_topic``); whether the robot offers it is the
        graph's answer. A backend with no fresh-read support (rclpy) offers none.
        """
        cargo_topic = getattr(self.backend, "cargo_topic", None)
        if not callable(cargo_topic) or not callable(
            getattr(self.backend, "cargo_reading", None)
        ):
            return None
        topic = cargo_topic()
        if not topic:
            return None
        return ("topic", topic, cargo_sensor.MESSAGE_TYPE)

    def describe(self) -> Sequence[decl.CapabilityDeclaration]:
        available = self._discovered_capabilities()
        self._declared = available
        declarations: list[decl.CapabilityDeclaration] = []
        for capability_id in sorted(available):
            interface = (
                HOST_INTERFACES.get(capability_id)
                or DEFAULT_INTERFACES.get(capability_id)
                or (
                    self._cargo_interface()
                    if capability_id == cargo_sensor.CAPABILITY_ID
                    else None
                )
            )
            if interface is None or not interface[1] or not interface[2]:
                # A declaration names the interface it is served by; never a
                # blank one a planner would take as served by nothing.
                raise RuntimeError(f"{capability_id} has no interface to declare")
            kind, name, interface_type = interface
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
            elif capability_id == cargo_sensor.CAPABILITY_ID:
                observations = (cargo_sensor.interface_observation(name),)
            declarations.append(
                decl.declare(
                    capability_id=capability_id,
                    resource_id=self.resource_id,
                    executor_kind=decl.EXECUTOR_EXTERNAL_API,
                    source=decl.SOURCE_DEVICE,
                    arguments=declared_arguments(capability_id),
                    required_observations=observations,
                    runtime_name=f"{kind}:{name}:{interface_type}",
                    version="1.0.0",
                )
            )
        return declarations

    def _motion_preflight(
        self, call_id: str, capability_id: str
    ) -> str | path_clearance.PathClearance | None:
        """None when the motion may start; else why not (a refused path with evidence)."""
        if capability_id == "motion.halt":
            return None
        held = HALT_HOLD.motion_refusal(call_id)
        if held is not None:
            return held
        mismatch = self._deployment_mismatch()
        if mismatch is not None:
            return mismatch
        basis = _safety_basis()
        if basis not in SAFETY_BASES:
            return f"FLYTO_ROS2_SAFETY_BASIS {basis!r} is not one of {', '.join(SAFETY_BASES)}"
        speed_limit = _speed_limit_problem()
        if speed_limit is not None:
            return speed_limit
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
        # The floor applies along the path this motion sweeps, not all around.
        verdict = path_clearance.judge(
            range_observation.get("sweep"),
            motion_kind=_motion_kind(capability_id),
            floor_m=_clearance_floor(),
            radius_m=_rotation_radius(),
            minimum_range_m=clearance,
        )
        if not verdict.admitted:
            return verdict
        if capability_id in PLANNED_MOTIONS and not observation.get(
            "map_tf_available", False
        ):
            return "fresh map-to-odom transform is required before navigation"
        return None

    def _read_cargo(self, request: CallRequest) -> CallResult:
        """A fresh cargo reading, behind the same graph check as motion.

        A graph that is not the configured deployment (a simulation claimed
        over physical hardware, or the reverse) refuses the read: a value from
        the wrong robot is no answer about this one.
        """
        mismatch = self._deployment_mismatch()
        if mismatch is not None:
            return cargo_sensor.refused_result(
                request.call_id,
                reason=cargo_sensor.REASON_DEPLOYMENT_UNCONFIRMED,
                detail=mismatch,
                deployment_mode=DEPLOYMENT_UNCONFIRMED,
            )
        return self.backend.cargo_reading(  # type: ignore[attr-defined]
            call_id=request.call_id, deadline_seconds=float(request.deadline_seconds)
        )

    def _deployment_mismatch(self) -> str | None:
        """Refuse motion when the graph is not the kind of robot configured.

        The physical robot and its simulator expose the same endpoint
        (ws://127.0.0.1:19090), so the configured deployment mode alone cannot
        tell them apart. Without this check a command meant for the simulator
        could drive the real robot.
        """
        marker = _simulation_marker_topic()
        if not marker:
            return _unmarked_simulation_refusal()
        mismatch = self._deployment_mismatch_in(self.backend.discover(), marker)
        if mismatch is None:
            return None
        # The graph may be cached from before a simulator or driver restart;
        # refuse only on what the graph says now.
        self._invalidate_discovery()
        return self._deployment_mismatch_in(self.backend.discover(), marker)

    def served_identity(self) -> dict[str, str]:
        """Which resource this adapter serves now, and in which deployment.

        The execution host asks this before any call reaches equipment and
        refuses a job unless both are what Cloud scored (flyto-cloud
        ``local/served_resource.py``). The answer never comes from the id the
        adapter was built with: the resource is the one configured on this
        computer, and the deployment is the configured mode confirmed against
        the ROS graph as it is now (a simulator iff it publishes the marker
        topic, /clock by default). Raises ``ServedIdentityError`` when the
        configured mode and the graph disagree, when the graph shows nothing,
        or when a simulation is claimed with the marker check disabled: real
        is never lowered to simulation on the configuration alone.

        Returns ``{"resource_id", "deployment_mode"}`` with the mode
        ``simulation`` or ``real``, as in this adapter's discovery manifest.
        """
        resource_id = configured_resource_id()
        marker = _simulation_marker_topic()
        if not marker:
            unmarked = _unmarked_simulation_refusal()
            if unmarked is not None:
                raise ServedIdentityError(unmarked)
            return {"resource_id": resource_id, "deployment_mode": _served_deployment_mode()}
        # Read the graph as it is now: the robot behind this endpoint may have
        # been swapped for its simulator (or back) since discovery cached it.
        self._invalidate_discovery()
        interfaces = tuple(self.backend.discover())
        if not interfaces:
            raise ServedIdentityError(
                f"the ROS graph behind the adapter for {resource_id!r} shows no "
                "interfaces, so it cannot show which robot it reaches"
            )
        mismatch = self._deployment_mismatch_in(interfaces, marker)
        if mismatch is not None:
            raise ServedIdentityError(mismatch)
        return {"resource_id": resource_id, "deployment_mode": _served_deployment_mode()}

    @staticmethod
    def _deployment_mismatch_in(
        interfaces: Iterable[StandardInterface], marker: str
    ) -> str | None:
        mode = _configured_deployment_mode()
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
        if request.capability_id == "motion.halt":
            return self._invoke_halt(request)
        if getattr(self.backend, "_presence_only", False):
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail="a presence connection never commands the robot",
            )
        unavailable = self._link_unavailable(request.call_id)
        if unavailable is not None:
            return unavailable
        token = (self, request.call_id)
        if not self._gate.begin(token, actuating=_actuates(request.capability_id)):
            return self._transport_unavailable(
                request.call_id, "the connection is being refreshed by an operator"
            )
        try:
            return self._invoke(request)
        finally:
            self._gate.end(token)

    # -- motion.halt: it runs, or it holds ----------------------------------------

    def _invoke_halt(self, request: CallRequest) -> CallResult:
        """``motion.halt``: run the halt, or hold the robot when it cannot run.

        A halt is never refused, timed out or turned away like a motion. When
        it cannot run (a presence connection, the link down, an operator
        refresh in progress, ``motion.halt`` not on the graph, no time left)
        it ends ``failed`` / ``effect_unknown`` with the hold armed, a new halt
        epoch started (``_halt_not_run``), and zero velocity published and
        held wherever this connection can. Its arguments are not read: a
        stop takes none.
        """
        call_id = request.call_id
        if getattr(self.backend, "_presence_only", False):
            return self._halt_not_run(call_id, "a presence connection never commands the robot")
        unavailable = self._link_unavailable(call_id)
        if unavailable is not None:
            return self._halt_not_run(
                call_id,
                f"the link to the robot is unavailable: {unavailable.detail}",
                {"transport_unavailable": True, **dict(unavailable.evidence or {})},
            )
        token = (self, call_id)
        if not self._gate.begin(token, actuating=False):
            return self._halt_not_run(
                call_id, "the connection is being refreshed by an operator"
            )
        try:
            if "motion.halt" not in self._declared:
                # cmd_vel may have appeared since the graph was last read.
                self._invalidate_discovery()
                with contextlib.suppress(Exception):
                    self.describe()
            if "motion.halt" not in self._declared:
                return self._halt_not_run(
                    call_id, "motion.halt (the standard cmd_vel) is not present on the ROS 2 graph"
                )
            if request.deadline_seconds <= 0:
                return self._halt_not_run(call_id, "no time to run the halt")
            with HALT_HOLD.halting():
                return self.backend.safe_stop(call_id)
        finally:
            self._gate.end(token)

    def _halt_not_run(
        self, call_id: str, why: str, evidence: Mapping[str, Any] | None = None
    ) -> CallResult:
        """A halt that could not run: a new epoch, then ``hold_unrun_halt``.

        Nothing admitted before it is ever sent; motion is refused until a
        halt sees the robot still. The hold carries every running goal's
        cancel token, and the cancels and zero go out where the connection
        can send them.
        """
        with HALT_HOLD.halting():
            return hold_unrun_halt(self.backend, call_id, why, evidence)

    def release_hold(self, *, operator: str, reason: str) -> dict[str, Any]:
        """An operator's audited release of the halt hold (``_ObservationState.release_hold``)."""
        release = getattr(self.backend, "release_hold", None)
        if not callable(release):
            return {"released": False, "detail": "this backend cannot release a halt hold"}
        return release(operator=operator, reason=reason)

    def _invoke(self, request: CallRequest) -> CallResult:
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
        # Admitted now: a halt from here on ends this call before it moves.
        # (motion.halt never comes here: ``_invoke_halt``.)
        HALT_HOLD.admit(request.call_id)
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
        if request.capability_id == cargo_sensor.CAPABILITY_ID:
            return self._read_cargo(request)
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
        if supervised:
            arguments, bound_error = _supervised_arguments(request.capability_id, arguments)
            if bound_error is not None:
                return CallResult(request.call_id, OUTCOME_REFUSED, detail=bound_error)
        preflight_error = self._motion_preflight(request.call_id, request.capability_id)
        if isinstance(preflight_error, path_clearance.PathClearance):
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                evidence=preflight_error.evidence(),
                detail=preflight_error.describe(),
            )
        if preflight_error is not None:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=preflight_error,
            )
        kept = self._provided.get(request.call_id)
        if kept is not None:
            # The same call again: its result, not a second motion's evidence.
            return kept
        escape: inflation_escape.EscapeDecision | None = None
        if request.capability_id == "motion.navigate":
            # The robot comes to rest at the destination: a return inside the
            # floor there refuses the call before anything moves.
            destination = self._destination_verdict("goal", arguments)
            if not destination.admitted:
                return CallResult(
                    request.call_id,
                    OUTCOME_REFUSED,
                    evidence={**destination.evidence(), **_target_evidence(arguments, place)},
                    detail=destination.describe(),
                )
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
            if request.capability_id == "motion.navigate":
                self._arm_leg_guard(request.call_id, request.capability_id)
            result = self._send_motion(
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
        """Plan a straight drive from its room and have the backend guard it.

        How the speed can be governed once the drive runs is decided first
        (``braking.decide_governance``, from the server running the drive).
        The speed is the requested one, or lower: a margin under the fastest
        that can still come to rest at the clearance floor from the room the
        drive starts with. A server that cannot take a slower goal mid-drive
        also gets a shorter distance, so the drive ends by itself where it can
        still stop at the floor. A drive that cannot be given the minimum
        speed or distance is refused before it moves.
        """
        arm = getattr(self.backend, "arm_braking_guard", None)
        profile_of = getattr(self.backend, "braking_profile", None)
        if not callable(arm) or not callable(profile_of):
            return dict(arguments)
        profile = profile_of()
        governance = self._speed_governance(capability_id)
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
        plan = braking.plan_drive(
            profile,
            room_m=room.room_m,
            requested_speed_mps=float(
                arguments.get("speed_mps", declared_default(capability_id, "speed_mps"))
            ),
            requested_distance_m=float(arguments["distance_m"]),
            mode=governance.mode,
            speed_fraction=SLOWDOWN_FRACTION,
        )
        if plan.refusal is not None:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                evidence={
                    # Refused before moving: there is no room to drive any
                    # distance and still stop at the floor, so the swept path
                    # is not clear (motion_outcome STOP_RULES "no_room_to_drive").
                    "reason_code": motion_outcome.REASON_PATH_BLOCKED,
                    "braking": {
                        "profile": profile.to_dict(),
                        "governance": governance.to_dict(),
                        "plan": plan.to_dict(),
                        **room.to_dict(),
                    },
                },
                detail=(
                    f"only {room.room_m:.3f} m of room before the {profile.floor_m:.3f} m "
                    f"clearance floor the way the robot would drive; {plan.refusal}"
                ),
            )
        arm(
            call_id,
            capability_id,
            profile=profile,
            speed_mps=plan.speed_mps,
            requested_speed_mps=plan.requested_speed_mps,
            distance_m=plan.distance_m,
            start_room_m=room.room_m,
            governance=governance,
            plan=plan,
        )
        return {**arguments, "speed_mps": plan.speed_mps, "distance_m": plan.distance_m}

    def _speed_governance(self, capability_id: str) -> braking.SpeedGovernance:
        """How this drive's speed can be governed on the server that runs it."""
        _, action_name, declared_type = _interface(capability_id)
        action_type = next(
            (
                item.type
                for item in self._interfaces_for_governance()
                if item.kind == "action" and item.name == action_name
            ),
            declared_type,
        )
        return braking.decide_governance(
            action_type,
            transport_resends=bool(getattr(self.backend, "RESENDS_GOALS", False)),
            server_plugin=self._server_plugin(action_name),
        )

    def _server_plugin(self, action_name: str) -> str | None:
        """The behavior plugin class the running server loaded for an action.

        Nav2's behavior server names each behavior after its action and
        declares ``<behavior>.plugin`` (``nav2_behaviors::DriveOnHeading``).
        Read once per action on this adapter; None when the server does not
        say or cannot be read, which is kept too: a robot without a behavior
        server must not wait out a parameter read before every drive, and
        "not said" only ever makes a drive planned, the mode safe everywhere.
        """
        if action_name in self._server_plugins:
            return self._server_plugins[action_name]
        reader = getattr(self.backend, "get_parameter", None)
        node = os.getenv("FLYTO_ROS2_BEHAVIOR_SERVER", "/behavior_server").strip()
        if not callable(reader) or not node:
            return None
        try:
            value = reader(node, f"{action_name.strip('/')}.plugin")
        except Exception as error:  # noqa: BLE001 - unreadable is "not said"
            logger.info("behavior plugin for %s unreadable: %s", action_name, error)
            value = None
        plugin = inflation_escape.parameter_text(value)
        self._server_plugins[action_name] = plugin
        return plugin

    def _interfaces_for_governance(self) -> Sequence[StandardInterface]:
        try:
            return tuple(self.backend.discover())
        except Exception:  # noqa: BLE001 - the declared type then stands
            return ()

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
        arrival_tolerance, _ = inflation_escape.read_arrival_tolerance(
            reader if callable(reader) else None, _controller_nodes()
        )
        heading_tolerance, _ = inflation_escape.read_heading_tolerance(
            reader if callable(reader) else None, _controller_nodes()
        )
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
            arrival_tolerance_m=arrival_tolerance,
            heading_tolerance_rad=heading_tolerance,
        )
        if decision is None:
            return None
        if decision.pinned:
            logger.info("navigation %s starts pinned: %s", call_id, decision.describe())
        _remember(self._escapes, call_id, decision)
        # Kept with the decision: the escape's waypoint is placed from this pose.
        _remember(self._escape_starts, call_id, dict(observation["map_pose"]))
        return decision

    def _arm_leg_guard(self, leg_id: str, capability_id: str) -> None:
        """Have the backend hold the floor on every scan while this leg runs."""
        arm = getattr(self.backend, "arm_floor_guard", None)
        profile_of = getattr(self.backend, "braking_profile", None)
        if callable(arm) and callable(profile_of):
            arm(leg_id, capability_id, profile=profile_of())

    def _destination_verdict(
        self, leg: str, destination: Mapping[str, Any]
    ) -> path_clearance.DestinationClearance:
        """Whether a navigation leg's destination keeps the floor, from a fresh sweep."""
        observation = self.backend.observation(
            required=(REQUIRE_POSE, REQUIRE_RANGE, REQUIRE_MAP_TF)
        )
        reading = observation.get("range")
        sweep = reading.get("sweep") if isinstance(reading, Mapping) else None
        return path_clearance.judge_destination(
            sweep,
            observation.get("map_pose"),
            (float(destination["x"]), float(destination["y"])),
            floor_m=_clearance_floor(),
            leg=leg,
        )

    def _send_motion(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult:
        """Hand an admitted motion to the backend, unless a halt has run since.

        Checked right before the goal goes out (each leg of an escape too): a
        halt that started after the call was admitted, or a hold armed since
        the preflight, ends the call here. The backends check again under the
        send gate, so nothing slips between this check and the send.
        """
        refusal = HALT_HOLD.motion_refusal(call_id)
        if refusal is not None:
            return CallResult(call_id, OUTCOME_REFUSED, detail=refusal)
        return self.backend.invoke(
            call_id=call_id,
            capability_id=capability_id,
            arguments=arguments,
            deadline_seconds=deadline_seconds,
        )

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
        self._arm_leg_guard(leg_id, capability_id)
        self._escape_legs[call_id] = leg_id
        HALT_HOLD.admit(leg_id, like=call_id)
        try:
            return self._send_motion(
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
        began = time.monotonic()
        deadline = began + deadline_seconds
        start = self._escape_starts.get(call_id) or {}
        goal_xy = (float(target["x"]), float(target["y"]))
        waypoint = inflation_escape.waypoint_pose(start, decision, goal_xy)
        heading = inflation_escape.waypoint_heading(start, decision, goal_xy)
        legs: list[dict[str, Any]] = []
        leg_started = began

        def record(leg: str, result: CallResult) -> None:
            # When each leg began and ended, from the call's start: the gap
            # between one leg's end and the next one's start is the adapter's.
            nonlocal leg_started
            now = time.monotonic()
            entry: dict[str, Any] = {
                "leg": leg,
                "outcome": result.outcome,
                "started_at_s": round(leg_started - began, 3),
                "elapsed_s": round(now - leg_started, 3),
            }
            # How fast a simulator ran during this leg, from its own record.
            outcome_record = (result.evidence or {}).get("motion_outcome")
            if isinstance(outcome_record, Mapping) and isinstance(
                outcome_record.get("sim_time"), Mapping
            ):
                entry["sim_time"] = dict(outcome_record["sim_time"])
            legs.append(entry)
            leg_started = now

        def escape_record() -> dict[str, Any]:
            timed = [leg["sim_time"] for leg in legs if "sim_time" in leg]
            summary: dict[str, Any] = {
                "waypoint_map": waypoint,
                "waypoint_heading": heading,
                "legs": legs,
            }
            if timed:
                # Over the whole call: the legs' clocks summed, the slowest window.
                summary["sim_time"] = sim_time.combine(timed)
            return summary

        def ended(leg: str, result: CallResult) -> CallResult:
            record(leg, result)
            outcome = result.outcome if result.outcome != OUTCOME_REFUSED else OUTCOME_FAILED
            return CallResult(
                call_id,
                outcome,
                evidence={
                    **dict(result.evidence or {}),
                    "navigation_escape": escape_record(),
                },
                detail=f"inflation escape {leg} {result.outcome}: {result.detail}".strip(),
            )

        if decision.backoff_m > 0.0:
            backoff = _escape_backoff_arguments(decision.backoff_m)
            if isinstance(backoff, str):
                return ended(
                    "backoff",
                    CallResult(f"{call_id}:escape-backoff", OUTCOME_REFUSED, detail=backoff),
                )
            result = self._escape_leg(
                call_id, f"{call_id}:escape-backoff", "motion.retreat", backoff, deadline
            )
            if result.outcome != OUTCOME_COMPLETED:
                return ended("backoff", result)
            record("backoff", result)
        # Two NavigateToPose legs, not one NavigateThroughPoses goal: a stock
        # Nav2 configuration can route through-poses goals to a behaviour tree
        # that only reads a single goal (TurtleBot3 Jazzy did: the planner got an
        # empty pose and the goal was refused in under a second), while
        # NavigateToPose is served by every Nav2.
        # Each leg's destination is judged from the sweep where the robot now
        # stands, before the leg is sent: the robot comes to rest there.
        verdict = self._destination_verdict("waypoint", waypoint)
        if not verdict.admitted:
            return ended("waypoint", self._destination_refusal(
                f"{call_id}:escape-waypoint", verdict
            ))
        result = self._escape_leg(
            call_id, f"{call_id}:escape-waypoint", "motion.navigate", waypoint, deadline
        )
        if result.outcome != OUTCOME_COMPLETED:
            return ended("waypoint", result)
        record("waypoint", result)
        if call_id in self._cancelled_escapes:
            return ended("goal", CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled"))
        verdict = self._destination_verdict("goal", target)
        if not verdict.admitted:
            return ended("goal", self._destination_refusal(call_id, verdict))
        self._arm_leg_guard(call_id, "motion.navigate")
        result = self._send_motion(
            call_id=call_id,
            capability_id="motion.navigate",
            arguments=target,
            deadline_seconds=max(0.0, deadline - time.monotonic()),
        )
        record("goal", result)
        return CallResult(
            result.call_id,
            result.outcome,
            evidence={
                **dict(result.evidence or {}),
                "navigation_escape": escape_record(),
            },
            detail=result.detail,
        )

    @staticmethod
    def _destination_refusal(
        leg_id: str, verdict: path_clearance.DestinationClearance
    ) -> CallResult:
        return CallResult(
            leg_id, OUTCOME_REFUSED, evidence=verdict.evidence(), detail=verdict.describe()
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
        down = self._transport_down(call_id)
        if down is not None:
            return down
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
        """The host's stop: a halt that never fails open (see ``_invoke_halt``)."""
        call_id = f"safe-stop-{time.monotonic_ns()}"
        down = self._transport_down(call_id)
        if down is not None:
            return self._halt_not_run(
                call_id,
                f"the link to the robot is unavailable: {down.detail}",
                {"transport_unavailable": True, **dict(down.evidence or {})},
            )
        with HALT_HOLD.halting():
            return self.backend.safe_stop(call_id)

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

    def telemetry_readings(self) -> dict[str, Any] | None:
        """The backend's live readings for a host's HMI; None while disconnected."""
        method = getattr(self.backend, "telemetry_readings", None)
        if not callable(method) or not self.connected:
            return None
        return method()

    def bridge_url(self) -> str | None:
        """The rosbridge this adapter reads the robot through; None over rclpy."""
        method = getattr(self.backend, "bridge_url", None)
        return method() if callable(method) else None

    def published_topics(self) -> dict[str, str]:
        """``{topic: type}`` as the graph was last read; empty while disconnected."""
        method = getattr(self.backend, "published_topics", None)
        if not callable(method) or not self.connected:
            return {}
        return dict(method())

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
        # The forward goes with the last adapter holding it (after its linger).
        if self.transport is not None and self._transport_held:
            self._transport_held = False
            self.transport.release()

    def reopen(self) -> None:
        """Reconnect the ROS link only; the forward is left as it is."""
        self._hold_transport()
        method = getattr(self.backend, "reconnect", None)
        if callable(method):
            method()

    def _hold_transport(self) -> None:
        if self.transport is None or self._transport_held:
            return
        held = ssh_transport.acquire(self.transport.config)
        if held is not None:
            self.transport = held
            self._gate = held.calls
            self._transport_held = True

    def _drop_link(self) -> None:
        method = getattr(self.backend, "disconnect", None)
        if callable(method):
            method()

    # -- the link: state, fail-fast admission, operator refresh ------------------------

    def transport_status(self) -> dict[str, Any]:
        """``{transport, state, since, attempts, last_error, ..., rosbridge_ok}``.

        ``state`` is ``connected``, ``reconnecting`` or ``failed`` (``stopped``
        once the forward is torn down). Without a managed forward the state is
        the ROS link's own.
        """
        connected = self.connected
        if self.transport is not None:
            status = self.transport.status()
        else:
            status = {
                "transport": "direct",
                "state": ssh_transport.STATE_CONNECTED if connected else ssh_transport.STATE_FAILED,
                "since": None,
                "attempts": 0,
                "last_error": "" if connected else self._last_link_error,
                "error_code": "",
                "accepting_calls": not self._gate.paused,
            }
        return {**status, "rosbridge_ok": connected}

    def _transport_unavailable(
        self, call_id: str, why: str = "", *, outcome: str = OUTCOME_REFUSED
    ) -> CallResult:
        status = self.transport_status()
        detail = (
            ssh_transport.unavailable_detail(status)
            if self.transport is not None
            else f"{ssh_transport.REASON_TRANSPORT_UNAVAILABLE}: nothing was sent"
        )
        return CallResult(
            call_id,
            outcome,
            evidence={
                "reason_code": ssh_transport.REASON_TRANSPORT_UNAVAILABLE,
                "transport": status,
            },
            detail=f"{detail}; {why}" if why else detail,
        )

    def _transport_down(self, call_id: str) -> CallResult | None:
        if self.transport is None or self.transport.connected:
            return None
        return self._transport_unavailable(call_id, outcome=OUTCOME_FAILED)

    def _link_unavailable(self, call_id: str) -> CallResult | None:
        """Fail fast while the forward is down; reopen a ROS link it dropped."""
        if self.transport is None:
            return None
        if not self.transport.connected:
            return self._transport_unavailable(call_id)
        if self.connected:
            return None
        try:
            self.reopen()
        except Exception as error:  # noqa: BLE001 - rosbridge is not answering yet
            self._last_link_error = f"{type(error).__name__}: {error}"[:300]
            return self._transport_unavailable(
                call_id, f"rosbridge did not answer through the forward ({self._last_link_error})"
            )
        if not self.connected:
            return self._transport_unavailable(call_id, "rosbridge did not answer")
        return None

    def reconnect(self) -> dict[str, Any]:
        """Operator refresh: stop what moves, rebuild the link, re-read the robot.

        Pauses new calls on this link; if an actuating call is in flight, safe
        stops it (zero velocity, then cancel) and refuses the refresh when the
        stop is not confirmed. Otherwise tears the SSH forward (if managed)
        and the rosbridge session down now, establishes both again (the robot
        host resolved anew), and re-reads the ROS graph and
        ``served_identity()``. Returns ``transport_status()`` plus
        ``refused``, ``rosbridge_ok``, ``topics_seen``, ``served_identity`` /
        ``served_identity_error``, ``rosbridge_error`` and ``safe_stop``
        (None when nothing was moving). Never raises; calling it again
        repeats the refresh.
        """
        with self._refresh_lock:
            stop = self._stop_actuating_calls()
            try:
                if stop is not None and not stop["confirmed"]:
                    return {
                        **self.transport_status(),
                        "refused": True,
                        "reason_code": "safe_stop_unconfirmed",
                        "detail": "an actuating call is in flight and its safe stop was not "
                        "confirmed; the link was left as it is",
                        "topics_seen": None,
                        "served_identity": None,
                        "safe_stop": stop,
                    }
                return {**self._refresh_link(), "refused": False, "safe_stop": stop}
            finally:
                self._gate.resume()

    def _stop_actuating_calls(self) -> dict[str, Any] | None:
        tokens = self._gate.pause()
        if not tokens:
            return None
        adapters = list({id(adapter): adapter for adapter, _call in tokens}.values())
        outcomes: list[dict[str, str]] = []
        for adapter in adapters:
            try:
                result = adapter.safe_stop()
                outcomes.append({"outcome": result.outcome, "detail": result.detail[:200]})
            except Exception as error:  # noqa: BLE001 - an unconfirmed stop is the answer
                outcomes.append({"outcome": OUTCOME_FAILED, "detail": str(error)[:200]})
        return {
            "calls": sorted(str(call) for _adapter, call in tokens),
            "outcomes": outcomes,
            "confirmed": all(item["outcome"] == OUTCOME_COMPLETED for item in outcomes),
        }

    def _refresh_link(self) -> dict[str, Any]:
        self._hold_transport()
        if self.transport is not None:
            self.transport.refresh()
            self.transport.wait_connected(self.transport.config.ready_timeout_s)
        rosbridge_error = ""
        if self.transport is None or self.transport.connected:
            try:
                self.reopen()
            except Exception as error:  # noqa: BLE001 - reported, never raised
                rosbridge_error = f"{type(error).__name__}: {error}"[:300]
        else:
            self._drop_link()
            rosbridge_error = "the SSH forward is not up"
        self._last_link_error = rosbridge_error
        topics_seen: int | None = None
        identity: dict[str, str] | None = None
        identity_error = ""
        if self.connected:
            try:
                self._invalidate_discovery()
                self.describe()
                topics_seen = sum(item.kind == "topic" for item in self.backend.discover())
                identity = self.served_identity()
            except Exception as error:  # noqa: BLE001 - reported, never raised
                identity_error = f"{type(error).__name__}: {error}"[:300]
        return {
            **self.transport_status(),
            "rosbridge_error": rosbridge_error,
            "topics_seen": topics_seen,
            "served_identity": identity,
            "served_identity_error": identity_error,
        }


def _actuates(capability_id: str) -> bool:
    if capability_id == "motion.halt":
        return False
    try:
        return decl.capability_metadata(capability_id).get("safety_class") == "movement"
    except KeyError:
        return False


def configured_transport() -> str:
    """``FLYTO_ROS2_TRANSPORT``; a configured SSH forward implies rosbridge."""
    explicit = os.getenv("FLYTO_ROS2_TRANSPORT", "").strip().lower()
    if explicit:
        return explicit
    return "rosbridge" if os.getenv(ssh_transport.ENV_HOST, "").strip() else "rclpy"


class RclpyROS2Backend(_ObservationState):
    """rclpy client for standard Nav2 actions and observation topics.

    A background executor delivers callbacks for the adapter's whole life, so
    readings, goal acceptance and results arrive as they happen. No caller
    spins the node: rclpy forbids spinning one node from two threads, and a
    spin to a fixed deadline costs that deadline whether or not anything came.
    """

    #: This transport only stops a guarded drive; it never re-sends one.
    RESENDS_GOALS = False

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
        # Goals sent and not yet accepted (their send_goal_async futures): a
        # halt sees and cancels them too.
        self._pending_goals: dict[str, Any] = {}
        # Each goal's action name: what cancels it and confirms its end from
        # another node or a restarted host (``_cancel_token``).
        self._goal_actions: dict[str, str] = {}
        # The action servers' own cancel services, per action name.
        self._cancel_clients: dict[str, Any] = {}
        self._results: dict[str, CallResult] = {}
        self._counts: dict[str, int] = {}
        self._publishers: dict[str, Any] = {}
        # Kept per capability: a fresh ActionClient has not matched its server
        # yet, and rclpy's wait_for_server then polls in 0.25 s steps.
        self._action_clients: dict[str, Any] = {}
        self._node: Any | None = None
        self._create_node()
        # Like a rosbridge connection: a host that starts held (a kept hold
        # file, or another connection's hold) sends zero and holds it now.
        if self.is_connected():
            self._resume_hold()

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
        self._subscribe_battery(node)
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

    def _subscribe_battery(self, node: Any) -> None:
        """Read only: the battery for a host's HMI (``live_telemetry``)."""
        try:
            from sensor_msgs.msg import BatteryState
        except ImportError:
            return

        def battery(message: Any) -> None:
            self._store_battery(
                getattr(message, "percentage", None),
                getattr(message, "voltage", None),
                time.monotonic(),
            )

        node.create_subscription(
            BatteryState, _battery_topic(), self._guarded(battery), self._sensor_qos
        )

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
        self._cancel_clients.clear()
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
        twist = odometry_twist_axes(getattr(getattr(message, "twist", None), "twist", None))
        self._store_odometry(
            {
                "frame": "odom",
                "x": float(message.pose.pose.position.x),
                "y": float(message.pose.pose.position.y),
                "yaw": float(yaw),
            },
            twist,
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
        slower, so its drives are planned before sending (speed and distance
        from the room they start with; see ``braking.plan_drive``).
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
            done.wait(timeout=min(remaining, GUARD_POLL_S))
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
        return ros2_interfaces.interface_class(_interface(capability_id)[2])

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
                arguments.get("speed_mps", declared_default(capability_id, "speed_mps"))
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
            HALT_HOLD.admit_if_new(call_id)
            with HALT_HOLD.sending(call_id) as refusal:
                # Sent and recorded as pending under the gate a halt takes
                # before it reads the running goals.
                if refusal is not None:
                    return CallResult(call_id, OUTCOME_REFUSED, detail=refusal)
                started = client.send_goal_async(
                    self._goal(capability_id, arguments),
                    feedback_callback=self._guarded(
                        lambda message, call_id=call_id: self._track_feedback(
                            call_id, _message_fields(getattr(message, "feedback", None))
                        )
                    ),
                )
                self._pending_goals[call_id] = started
                self._goal_actions[call_id] = action_name
            accept_deadline = time.monotonic() + min(2.0, deadline_seconds)
            if not self._wait_future(started, accept_deadline):
                # Still pending: a halt keeps seeing it until it is answered.
                return CallResult(
                    call_id, OUTCOME_TIMEOUT, detail="goal acceptance timed out"
                )
            handle = self._accepted_goal(call_id)
            if handle is None:
                answered = started.result()
                if answered is not None and getattr(answered, "accepted", False):
                    # Accepted, and a halt took it and ended it meanwhile.
                    return CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled by a halt")
                return CallResult(
                    call_id, OUTCOME_REFUSED, detail="ROS 2 action goal refused"
                )
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
        if status == GoalStatus.STATUS_SUCCEEDED or _rule_reached(summary):
            # A failed one leads its detail with motion_outcome.describe().
            result = _motion_result(
                call_id, _succeeded_outcome(summary), self._evidence(capability_id), summary
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

    def _accepted_goal(self, call_id: str) -> Any:
        """Move an answered pending goal to the running goals; its handle, or None.

        Called by the call that sent it and by a halt, whichever reads the
        answer first; both get the same handle.
        """
        with self._condition:
            handle = self._goal_handles.get(call_id)
            if handle is not None:
                return handle
            started = self._pending_goals.get(call_id)
            if started is None or not started.done():
                return None
            self._pending_goals.pop(call_id, None)
            handle = started.result()
            if handle is None or not handle.accepted:
                return None
            self._goal_handles[call_id] = handle
            self._result_futures[call_id] = handle.get_result_async()
            return handle

    def _forget_goal(self, call_id: str) -> None:
        self._goal_handles.pop(call_id, None)
        self._result_futures.pop(call_id, None)
        self._pending_goals.pop(call_id, None)
        self._goal_actions.pop(call_id, None)
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
        deadline = time.monotonic() + CANCEL_CONFIRM_WAIT_S
        future = handle.cancel_goal_async()
        if not self._wait_future(future, min(deadline, time.monotonic() + 2.0)):
            return CallResult(
                call_id, OUTCOME_FAILED, detail="ROS 2 cancel timed out"
            )
        # The cancel's answer is not the goal's end: a CANCELING goal is
        # still driven by its server. Only its result (a terminal status) is.
        result_future = self._result_futures.get(call_id)
        ended = result_future is not None and self._wait_future(result_future, deadline)
        stopped = self._publish_zero()
        if not ended or not stopped:
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                detail=(
                    "ROS 2 cancel/stop was not confirmed"
                    + ("" if ended else ": the goal has not ended")
                ),
            )
        self._forget_goal(call_id)
        return self._keep_result(
            call_id, CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        )

    def safe_stop(self, call_id: str) -> CallResult:
        return self._halt(call_id, {})

    def _active_goal_ids(self) -> tuple[str, ...]:
        return tuple(dict.fromkeys([*self._goal_handles, *self._pending_goals]))

    def _goal_active(self, call_id: str) -> bool:
        future = self._result_futures.get(call_id)
        return call_id in self._goal_handles and not (future is not None and future.done())

    def _running_goal_ids(self) -> list[str]:
        # A goal sent and not yet answered may be accepted after the reconnect.
        return [
            goal
            for goal in self._active_goal_ids()
            if goal in self._pending_goals or self._goal_active(goal)
        ]

    def _cancel_for_halt(self, call_id: str) -> str | None:
        started = self._pending_goals.get(call_id)
        if call_id not in self._goal_handles and started is not None:
            # Sent but not yet answered: wait for the answer, then cancel it.
            if not self._wait_future(started, time.monotonic() + 2.0):
                return "goal acceptance was not answered, so it could not be cancelled"
            if self._accepted_goal(call_id) is None:
                return None  # refused: it never ran
        if not self._goal_active(call_id):
            return None  # the goal already ended
        try:
            result = self.cancel(call_id)
        except Exception as error:  # noqa: BLE001 - an unsent cancel is unconfirmed
            result = CallResult(
                call_id,
                OUTCOME_FAILED,
                detail=f"cancel not sent: {type(error).__name__}: {error}",
            )
        if result.outcome == OUTCOME_CANCELLED or not self._goal_active(call_id):
            return None
        return result.detail or "ROS 2 cancel was not confirmed"

    def _release_goal(self, call_id: str) -> None:
        self._forget_goal(call_id)

    def _cancel_token(self, call_id: str) -> Any:
        # By action name, never the goal handle: a handle belongs to the node
        # (and process) that sent it, and a reconnect or restart outlives it.
        return (call_id, self._goal_actions.get(call_id))

    def _cancel_all_goals(self, action_name: str) -> None:
        node = self._node
        if node is None:
            raise RuntimeError("ROS 2 adapter node is gone")
        from action_msgs.srv import CancelGoal

        client = self._cancel_clients.get(action_name)
        if client is None:
            client = node.create_client(CancelGoal, action_cancel_service(action_name))
            self._cancel_clients[action_name] = client
        # A default request carries the zero goal id and stamp: cancel all.
        client.call_async(CancelGoal.Request())

    def _goal_statuses_until(
        self,
        action_name: str,
        settled: Callable[[list[int]], bool],
        timeout: float,
    ) -> list[int] | None:
        node = self._node
        if node is None:
            return None
        try:
            from action_msgs.msg import GoalStatusArray
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        except ImportError:
            return None
        seen: dict[str, list[int]] = {}

        def on_status(message: Any) -> None:
            with self._condition:
                seen["statuses"] = goal_statuses(message)
                self._condition.notify_all()

        # The status publisher is reliable and transient-local: a late reader
        # gets the newest array at once.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        subscription = node.create_subscription(
            GoalStatusArray, action_status_topic(action_name), self._guarded(on_status), qos
        )
        try:
            deadline = time.monotonic() + timeout
            with self._condition:
                while self._connected:
                    statuses = seen.get("statuses")
                    if statuses is not None and settled(statuses):
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(timeout=remaining)
                return seen.get("statuses")
        finally:
            with contextlib.suppress(Exception):
                node.destroy_subscription(subscription)

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
        # Goals sent before the drop are no longer followed by this executor:
        # every one still running goes into the hold before it is forgotten,
        # with no goal sent and recorded in between.
        with HALT_HOLD.forgetting():
            self._hold_running_goals()
            self._stop_executor()
            self._reset_readings()
            self._goal_handles.clear()
            self._result_futures.clear()
            self._pending_goals.clear()
            self._goal_actions.clear()
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
            # The goals were forgotten above; the robot's hold was not.
            self._resume_hold()



class RosbridgeROS2Backend(_ObservationState):
    """WebSocket ROS 2 backend for an external AI Space host.

    The robot runs only upstream rosbridge/rosapi. The adapter stays on the
    external computer and speaks the same standard ROS 2 capability contract
    as :class:`RclpyROS2Backend`.
    """

    #: This transport can send a second goal for a running straight drive, so
    #: a server that takes preempting goals can have a drive slowed mid-way.
    RESENDS_GOALS = True
    #: Samples /clock (read only, throttled) in a simulation deployment.
    READS_SIM_CLOCK = True

    def __init__(
        self,
        *,
        url: str | None = None,
        connection_factory: Callable[[str], Any] | None = None,
        presence_only: bool = False,
        url_source: Callable[[], str] | None = None,
        connect: bool = True,
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
        # A managed SSH forward may move its local port between attempts: the
        # URL is read again on every reconnect.
        self._url_source = url_source
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
        # The terminal status of every goal that ended, by owning call id. Never
        # taken by a waiter, so a cancel confirms against it even after the
        # goal's own invoke() took the result message.
        self._goal_ended: dict[str, int] = {}
        # The newest reading of the configured cargo topic (sensing.cargo_present).
        self._cargo = cargo_sensor.CargoSensor()
        # One-shot reads for vision.observe and sensing.map: topic -> waiter.
        self._capture_waits: dict[str, str] = {}
        self._captured: dict[str, dict[str, Any]] = {}
        # Goal status reads (``_goal_statuses_until``): topic -> waiter, and
        # waiter -> the newest statuses seen.
        self._status_waits: dict[str, str] = {}
        self._status_seen: dict[str, list[int]] = {}
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
        if connect:
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
                if identifier.startswith(UNAWAITED_SERVICE_PREFIX):
                    return  # sent without waiting; nobody reads the answer
                self._responses[identifier] = message
                self._condition.notify_all()
            elif operation == "action_result":
                self._action_results[owner] = message
                status = message.get("status")
                _remember(
                    self._goal_ended,
                    owner,
                    int(status) if isinstance(status, (int, float)) else 0,
                )
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
    def _twist(message: Mapping[str, Any]) -> TwistAxes | None:
        outer = message.get("twist")
        twist = outer.get("twist") if isinstance(outer, Mapping) else None
        return odometry_twist_axes(twist) if isinstance(twist, Mapping) else None

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
        if topic in _command_watch_topics():
            self._track_command(topic, raw_message)
            return
        if (
            topic
            and topic == _simulation_marker_topic()
            and _served_deployment_mode() == "simulation"
        ):
            self._track_clock(raw_message, time.monotonic())
            return
        status_waiter = self._status_waits.get(topic)
        if status_waiter is not None:
            with self._condition:
                self._status_seen[status_waiter] = goal_statuses(raw_message)
                self._condition.notify_all()
            return
        if topic and topic == self._cargo_subscription_topic():
            with self._condition:
                if self._cargo.record(raw_message, received=time.monotonic()):
                    self._condition.notify_all()
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
            _battery_topic(): lambda: self._store_battery(
                message.get("percentage"), message.get("voltage"), observed
            ),
        }
        handler = handlers.get(topic)
        if handler is None:
            return
        with self._condition:
            handler()

    def cargo_reading(self, *, call_id: str, deadline_seconds: float) -> CallResult:
        """The configured cargo topic's reading, only while it is fresh.

        A fresh reading answers at once. Otherwise the call waits for a new
        message, at most the freshness window (and never past its deadline),
        then fails with the typed reason and no value.
        """
        if not self._connected:
            return cargo_sensor.refused_result(
                call_id,
                reason=cargo_sensor.REASON_DISCONNECTED,
                detail="rosbridge adapter disconnected",
                deployment_mode=_served_deployment_mode(),
            )
        topic = self.cargo_topic()
        if topic is None:
            return cargo_sensor.no_topic_result(
                call_id, deployment_mode=_served_deployment_mode()
            )
        max_age = cargo_sensor.max_age_s()
        wait_until = time.monotonic() + min(max(0.0, deadline_seconds), max_age)
        with self._condition:
            while True:
                now = time.monotonic()
                verdict = self._cargo.verdict(now=now, max_age_s=max_age)
                if verdict.reason is None or now >= wait_until or not self._connected:
                    break
                self._condition.wait(wait_until - now)
        return cargo_sensor.call_result(
            call_id,
            verdict,
            topic=topic,
            max_age=max_age,
            deployment_mode=_served_deployment_mode(),
        )

    def _cargo_subscription_topic(self) -> str | None:
        """The topic this connection listens to for cargo: ``served_topic`` with
        the marker assumed present (the graph decides the read later)."""
        return cargo_sensor.served_topic(
            deployment_mode=_served_deployment_mode(),
            sim_marker_on_graph=True,
            marker_check=bool(_simulation_marker_topic()),
        )

    def cargo_topic(self) -> str | None:
        """The cargo topic this robot answers on, or None (``served_topic``)."""
        if self._interfaces is None and self._connected:
            with contextlib.suppress(RuntimeError):
                self.discover()
        return self._served_cargo_topic()

    def _served_cargo_topic(self) -> str | None:
        """``cargo_sensor.served_topic`` on the graph as last read."""
        marker = _simulation_marker_topic()
        return cargo_sensor.served_topic(
            deployment_mode=_served_deployment_mode(),
            sim_marker_on_graph=bool(marker) and self._topic_types.get(marker) == CLOCK_TYPE,
            marker_check=bool(marker),
        )

    def _wait_for(
        self,
        store: dict[str, Any],
        identifier: str,
        deadline: float,
        *,
        consume: bool = True,
    ) -> Any:
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
        subscriptions = tuple(
            (os.getenv(row.topic_env, row.default_topic), row.message_type,
             row.reliability, row.throttle_ms, row.queue_length)
            for row in ROSBRIDGE_CONTINUOUS_SUBSCRIPTIONS
        )
        collision_topic = _collision_state_topic()
        if collision_topic:
            # Published when the monitor changes what it does to the base,
            # so it costs nothing while the robot drives freely.
            subscriptions = (
                *subscriptions,
                (collision_topic, COLLISION_STATE_TYPE, "reliable", 0, 0),
            )
        clock_topic = _simulation_marker_topic()
        if not self._presence_only and clock_topic and _served_deployment_mode() == "simulation":
            # Read only, sampled: how fast simulated time runs against the wall.
            subscriptions = (
                *subscriptions,
                (clock_topic, CLOCK_TYPE, "best_effort", sim_time.CLOCK_THROTTLE_MS, 0),
            )
        cargo_topic = self._cargo_subscription_topic()
        if cargo_topic:
            # Read only, every message: the robot's own cargo sensor.
            subscriptions = (
                *subscriptions,
                (cargo_topic, cargo_sensor.MESSAGE_TYPE, "reliable", 0, 0),
            )
        if not self._presence_only:
            # Read only, sampled: what was commanded while a goal stood still.
            subscriptions = (
                *subscriptions,
                *(
                    (topic, None, "reliable", COMMAND_WATCH_THROTTLE_MS, 0)
                    for topic in _command_watch_topics()
                ),
            )
        if self._presence_only:
            # Odometry says the robot is there; a few a second is plenty.
            subscriptions = ((subscriptions[0][0], subscriptions[0][1], "reliable", 500, 0),)
        # Read only, sampled: the battery for a host's HMI, on every connection.
        subscriptions = (
            *subscriptions,
            (_battery_topic(), BATTERY_STATE_TYPE, "best_effort", BATTERY_THROTTLE_MS, 0),
        )
        subscriptions = (
            *subscriptions,
            *(
                (topic, LIFECYCLE_EVENT_TYPE, "reliable", 0, 0)
                for topic in sorted(self._lifecycle_topics)
            ),
        )
        for index, (topic, message_type, reliability, throttle_rate, queue_length) in enumerate(
            subscriptions
        ):
            request: dict[str, Any] = {
                "op": "subscribe",
                "id": f"observation-{index}",
                "topic": topic,
                "qos": {
                    "history": "keep_last",
                    "depth": 5,
                    "reliability": reliability,
                    "durability": "volatile",
                },
                "throttle_rate": throttle_rate,
            }
            if queue_length:
                # rosbridge keeps only the newest when it falls behind.
                request["queue_length"] = queue_length
            if message_type is not None:
                # Without a type rosbridge takes the one the graph advertises,
                # so a stamped and an unstamped cmd_vel both subscribe.
                request["type"] = message_type
            self._send(request)

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

    def bridge_url(self) -> str:
        """The bridge URL this connection uses (a managed forward's, when it moves)."""
        return self._url

    def published_topics(self) -> dict[str, str]:
        """``{topic: type}`` from the last rosapi read of the graph."""
        with self._condition:
            return dict(self._topic_types) if self._connected else {}

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
        cargo_topic = self._served_cargo_topic()
        if cargo_topic and self._topic_types.get(cargo_topic) == cargo_sensor.MESSAGE_TYPE:
            found.append(StandardInterface("topic", cargo_topic, cargo_sensor.MESSAGE_TYPE))
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
            arguments.get("speed_mps", declared_default(capability_id, "speed_mps"))
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
            HALT_HOLD.admit_if_new(call_id)
            with HALT_HOLD.sending(call_id) as refusal:
                # Sent and recorded as running under the gate a halt takes
                # before it reads the running goals.
                if refusal is not None:
                    return CallResult(call_id, OUTCOME_REFUSED, detail=refusal)
                with self._condition:
                    # A new goal: an earlier one's end says nothing about it.
                    self._goal_ended.pop(call_id, None)
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
        succeeded = result_message.get("result") is True and status == 4
        if succeeded or _rule_reached(summary):
            outcome = _succeeded_outcome(summary)
            result = _motion_result(
                call_id,
                outcome,
                # A failed motion does not wait for quiet sensors before the
                # host's safe stop; a completed one reads where it settled.
                self._evidence(capability_id, wait=outcome == OUTCOME_COMPLETED),
                summary,
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
                        timeout=min(remaining, GUARD_POLL_S) if guard is not None else remaining
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
        distance, and the slower speed its room now allows.

        A preempt is a new goal: it is sent only under ``HALT_HOLD.sending``,
        never while the hold is armed, while a halt runs, or after a halt that
        started once the drive was admitted. The renamed goal id is set under
        the same gate, so a halt's cancel token is always the driving goal's.
        """
        with HALT_HOLD.sending(call_id) as refusal:
            if refusal is not None:
                with self._condition:
                    guard = self._guards.get(call_id)
                    if guard is not None:
                        guard.slow_to = None  # nothing to carry out
                return
            self._send_slower_goal(call_id, capability_id)

    def _send_slower_goal(self, call_id: str, capability_id: str) -> None:
        action_name = self._active_actions.get(call_id)
        with self._condition:
            guard = self._guards.get(call_id)
            if (
                guard is None
                or not guard.resends
                or guard.slow_to is None
                or action_name is None
            ):
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

    def _advertise_once(self, topic: str, topic_type: str) -> None:
        """Advertise a topic this connection publishes, once per connection (volatile)."""
        key = (topic, topic_type)
        if key in self._advertised_topics:
            return
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

    def publish_simulated_state(self, topic: str, value: bool) -> None:
        """SIMULATION ONLY: publish the simulated tray's boolean state.

        Refused (``SimulationOnlyError``) unless the configured deployment is
        ``simulation`` and this connection's graph carries the simulator
        marker, so simulated state never reaches a physical robot. The graph
        is the one read on this connection (a reconnect forgets it and reads
        the new one); a graph without the marker is read again once before
        refusing, in case it predates a restart.
        Only a ``std_msgs/msg/Bool`` with a literal boolean, only on
        ``cargo_sensor.simulated_tray_topic()`` and never on a command topic.
        Volatile, not latched: the caller repeats it
        at ``cargo_sensor.MIN_PUBLISH_HZ`` or faster.
        """
        if type(value) is not bool:
            raise ValueError("simulated state is a literal boolean")
        if not isinstance(topic, str) or not topic.startswith("/") or len(topic) > 256:
            raise ValueError("a state topic is an absolute ROS name")
        if topic in {DEFAULT_INTERFACES["motion.halt"][1], *_command_watch_topics()}:
            raise ValueError(f"{topic} is a command topic")
        tray = cargo_sensor.simulated_tray_topic()
        if topic != tray:
            # Not a general publisher: only the simulated tray's own topic.
            raise ValueError(f"simulated state is published only on the tray topic {tray}")
        refusal = simulation_publish_refusal()
        if refusal is not None:
            raise SimulationOnlyError(refusal)
        marker = _simulation_marker_topic()
        if not self._marker_on_graph(marker):
            raise SimulationOnlyError(
                f"the graph does not carry the simulator marker {marker}; "
                "it may be a physical robot"
            )
        self._advertise_once(topic, cargo_sensor.MESSAGE_TYPE)
        self._send({"op": "publish", "topic": topic, "msg": {"data": value}})

    def _marker_on_graph(self, marker: str) -> bool:
        for attempt in range(2):
            if attempt:
                self.invalidate_discovery()  # a cached graph may predate a restart
            try:
                self.discover()
            except RuntimeError:
                continue
            if self._topic_types.get(marker) == CLOCK_TYPE:
                return True
        return False

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
        self._advertise_once(topic, topic_type)
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
        # The goal's terminal status, never its result message: the goal may
        # belong to another job on this shared connection (a safe stop cancels
        # every active goal), whose own invoke() takes the message when it
        # lands and must still receive it.
        status = self._wait_for(
            self._goal_ended,
            call_id,
            time.monotonic() + CANCEL_CONFIRM_WAIT_S,
            consume=False,
        )
        confirmed = status == 5
        with self._condition:
            if confirmed and call_id not in self._result_waiters:
                # Nobody is waiting; the kept CANCELLED result answers later.
                self._action_results.pop(call_id, None)
        if not confirmed or not stopped:
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
        return self._halt(call_id, {"transport": "rosbridge"})

    def _active_goal_ids(self) -> tuple[str, ...]:
        return tuple(self._active_actions)

    def _goal_active(self, call_id: str) -> bool:
        # A terminal status that came back means the server ended the goal.
        return call_id in self._active_actions and call_id not in self._goal_ended

    def _cancel_for_halt(self, call_id: str) -> str | None:
        try:
            result = self.cancel(call_id)
        except Exception as error:  # noqa: BLE001 - an unsent cancel is unconfirmed
            result = CallResult(
                call_id,
                OUTCOME_FAILED,
                detail=f"cancel not sent: {type(error).__name__}: {error}",
            )
        if result.outcome == OUTCOME_CANCELLED:
            return None
        with self._condition:
            ended = call_id in self._goal_ended
        if ended:
            return None
        return result.detail or "ROS 2 cancel was not confirmed"

    def _release_goal(self, call_id: str) -> None:
        self._active_actions.pop(call_id, None)
        self._forget_track(call_id)

    def _cancel_token(self, call_id: str) -> Any:
        return (self._goal_ids.get(call_id, call_id), self._active_actions.get(call_id))

    def _cancel_all_goals(self, action_name: str) -> None:
        # A goal id belongs to the rosbridge session that sent it, so the
        # action server's own cancel service is called, with the zero goal id.
        self._send(
            {
                "op": "call_service",
                "service": action_cancel_service(action_name),
                "type": CANCEL_GOAL_TYPE,
                "args": {
                    "goal_info": {
                        "goal_id": {"uuid": [0] * 16},
                        "stamp": {"sec": 0, "nanosec": 0},
                    }
                },
                "id": f"{UNAWAITED_SERVICE_PREFIX}cancel-all-{time.monotonic_ns()}",
            }
        )

    def _goal_statuses_until(
        self,
        action_name: str,
        settled: Callable[[list[int]], bool],
        timeout: float,
    ) -> list[int] | None:
        topic = action_status_topic(action_name)
        waiter = f"goal-status-{time.monotonic_ns()}"
        with self._condition:
            self._status_waits[topic] = waiter
        try:
            self._send(
                {
                    "op": "subscribe",
                    "id": waiter,
                    "topic": topic,
                    "type": GOAL_STATUS_ARRAY_TYPE,
                    # The server publishes it reliable and transient-local:
                    # a late reader gets the newest array at once.
                    "qos": {
                        "history": "keep_last",
                        "depth": 1,
                        "reliability": "reliable",
                        "durability": "transient_local",
                    },
                }
            )
            deadline = time.monotonic() + timeout
            with self._condition:
                while self._connected:
                    statuses = self._status_seen.get(waiter)
                    if statuses is not None and settled(statuses):
                        break
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        break
                    self._condition.wait(timeout=remaining)
                return self._status_seen.get(waiter)
        finally:
            with self._condition:
                if self._status_waits.get(topic) == waiter:
                    self._status_waits.pop(topic, None)
                self._status_seen.pop(waiter, None)
            with contextlib.suppress(Exception):
                self._send({"op": "unsubscribe", "id": waiter, "topic": topic})

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
        # Goal ids belong to the socket that sent them: every goal still
        # running goes into the hold (with its id and action) before they are
        # forgotten, so a later halt must cancel it and confirm it ended. No
        # goal is sent and recorded in between (``HALT_HOLD.forgetting``).
        with HALT_HOLD.forgetting():
            self._hold_running_goals()
            self.disconnect()
            self._active_actions.clear()
            self._goal_ids.clear()
            self._goal_owner.clear()
        if self._url_source is not None:
            self._url = self._url_source()
        websocket = self._connection_factory(self._url)
        keepalive_stop = threading.Event()
        with self._condition:
            self._ws = websocket
            self._connected = True
            self._interfaces = None
            self._keepalive_stop = keepalive_stop
            self._responses.clear()
            self._action_results.clear()
            self._goal_ended.clear()
            self._advertised_topics.clear()
            # The graph is the new connection's to read: a cached marker or
            # cargo topic may be another robot's.
            self._topic_types = {}
            self._action_names = frozenset()
            # A reading belongs to the connection it arrived on.
            self._cargo.clear()
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
        # Goal ids belonged to the old socket; the robot's hold did not.
        self._resume_hold()
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
    transport = configured_transport()
    if transport == "rclpy":
        if os.getenv(ssh_transport.ENV_HOST, "").strip():
            raise ssh_transport.TransportConfigError(
                f"{ssh_transport.ENV_HOST} carries rosbridge, not DDS; "
                "set FLYTO_ROS2_TRANSPORT=rosbridge (or unset it)"
            )
        backend: ROS2Backend = RclpyROS2Backend(presence_only=presence_only)
    elif transport == "rosbridge":
        tunnel = ssh_transport.acquire()
        if tunnel is not None:
            return _build_over_forward(tunnel, resource_id, presence_only=presence_only)
        backend = RosbridgeROS2Backend(presence_only=presence_only)
    else:
        raise RuntimeError(f"unsupported ROS 2 transport: {transport}")
    return GenericROS2Adapter(backend=backend, resource_id=resource_id)


def _build_over_forward(
    tunnel: ssh_transport.ManagedTunnel, resource_id: str, *, presence_only: bool
) -> GenericROS2Adapter:
    """An adapter on the managed forward, holding it until it is disconnected.

    The first build waits for the forward to come up once; a build while it is
    down after that returns at once, and the adapter's calls fail fast with
    ``transport_unavailable`` until it is back.
    """
    try:
        backend = RosbridgeROS2Backend(
            presence_only=presence_only,
            url_source=tunnel.url,
            connect=tunnel.wait_first(),
        )
    except Exception:
        tunnel.release()
        raise
    return GenericROS2Adapter(backend=backend, resource_id=resource_id, transport=tunnel)

