"""The robot's live readings, as the telemetry channels an execution host publishes.

A host calls the discoverer's ``telemetry_sampler`` about once a second and
publishes each sample on a channel the manifest declares
(``flyto.resource-telemetry.v1``); Cloud keeps the latest one per channel and
an HMI reads it through the pack's presentation panels. Nothing here talks to
the robot: the samples are built from readings the adapter's connections
already hold (battery state, odometry, the map transform, and the Nav2
feedback of a goal this computer sent).

Three channels, one row each in :data:`CHANNELS`:

* ``power.battery`` -- ``percent`` (0..100, from ``BatteryState.percentage``
  read on the scale it falls in, :data:`PERCENTAGE_SCALES`) and
  ``voltage_v``. A field the robot does not measure is null, never 0.
* ``pose.planar`` -- ``x``, ``y``, ``heading_deg`` (counter-clockwise from +x)
  and the ``frame`` they are in: ``map`` while localization's transform is
  fresh, else ``odom``. The goal (``goal_x``, ``goal_y``) is added only in the
  map frame it was given in, and a ``view_*`` window that contains both.
* ``nav.goal`` -- ``state`` (:data:`NAVIGATION_STATES`), ``distance_m`` left
  and ``progress_pct`` from Nav2's ``distance_remaining``, the goal, and the
  ``reason`` an ended goal gave (``motion_outcome``).

The channel and field names are a contract with the module pack's
presentation block (flyto-modules-robotics ``presentation.py``); both repos
pin the same table in their tests.
"""

from __future__ import annotations

import copy
import math
import time
from collections.abc import Iterable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from . import motion_outcome

CHANNEL_BATTERY = "power.battery"
CHANNEL_POSE = "pose.planar"
CHANNEL_NAVIGATION = "nav.goal"

NAVIGATION_STATES = ("idle", "navigating", "arrived", "blocked", "cancelled", "failed")

#: How a goal that ended reads on an HMI, by ``motion_outcome`` reason. Any
#: reason not listed is ``failed``; a goal judged reached is ``arrived``.
ENDED_STATES: Mapping[str, str] = {
    motion_outcome.REASON_COMPLETED: "arrived",
    motion_outcome.REASON_CANCELLED: "cancelled",
    motion_outcome.REASON_PATH_BLOCKED: "blocked",
    motion_outcome.REASON_OBSTACLE_BLOCKED: "blocked",
    motion_outcome.REASON_NO_PATH: "blocked",
    motion_outcome.REASON_NO_PROGRESS: "blocked",
}
ENDED_DEFAULT_STATE = "failed"

#: Seconds a battery reading stays current. The robot publishes it about once
#: a second and the connection samples it at that rate.
BATTERY_MAX_AGE_S = 10.0
#: The pose window: at least this far either side of its centre, plus margin.
VIEW_MIN_HALF_SPAN_M = 2.0
VIEW_MARGIN_M = 1.0

_NUMBER = {"type": ["number", "null"]}
_COORDINATE = {"type": "number"}


@dataclass(frozen=True)
class Channel:
    channel_id: str
    display_name: str
    properties: Mapping[str, Mapping[str, Any]]
    max_age_seconds: float


CHANNELS: tuple[Channel, ...] = (
    Channel(
        CHANNEL_BATTERY,
        "Battery",
        {
            "percent": {**_NUMBER, "minimum": 0, "maximum": 100},
            "voltage_v": _NUMBER,
        },
        BATTERY_MAX_AGE_S,
    ),
    Channel(
        CHANNEL_POSE,
        "Position",
        {
            "x": _COORDINATE,
            "y": _COORDINATE,
            "heading_deg": _COORDINATE,
            "frame": {"enum": ["map", "odom"]},
            "goal_x": _COORDINATE,
            "goal_y": _COORDINATE,
            "view_x_min": _COORDINATE,
            "view_x_max": _COORDINATE,
            "view_y_min": _COORDINATE,
            "view_y_max": _COORDINATE,
        },
        5.0,
    ),
    Channel(
        CHANNEL_NAVIGATION,
        "Navigation",
        {
            "state": {"enum": list(NAVIGATION_STATES)},
            "distance_m": _NUMBER,
            "progress_pct": {**_NUMBER, "minimum": 0, "maximum": 100},
            "goal_x": _COORDINATE,
            "goal_y": _COORDINATE,
            "reason": {"type": "string"},
        },
        5.0,
    ),
)


def channel_declarations() -> list[dict[str, Any]]:
    """The manifest's ``telemetry_channels`` entries, as fresh copies."""
    return [
        {
            "channel_id": channel.channel_id,
            "display_name": channel.display_name,
            "payload_schema": {
                "type": "object",
                "properties": copy.deepcopy(dict(channel.properties)),
            },
            "max_age_seconds": channel.max_age_seconds,
        }
        for channel in CHANNELS
    ]


def _finite(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


@dataclass(frozen=True)
class PercentageScale:
    """A range ``BatteryState.percentage`` can fall in, and what it means there."""

    name: str
    low: float
    low_inclusive: bool
    high: float
    #: Multiplier to a percentage 0..100.
    factor: float

    def holds(self, value: float) -> bool:
        above = value >= self.low if self.low_inclusive else value > self.low
        return above and value <= self.high


#: How ``BatteryState.percentage`` is read, first match wins. The message's
#: contract is a fraction 0..1, but the TurtleBot3 driver publishes 0..100
#: (burger-01, 2026-10-12: 93.88). A value in (1, 100] can only be a percent;
#: 0..1 is read as the contract says. Anything else is unknown: null, never
#: clamped. The one ambiguity is a percent-scale robot at or below 1 %, which
#: reads as a fraction; no scale can tell those apart from one message.
PERCENTAGE_SCALES: tuple[PercentageScale, ...] = (
    PercentageScale("fraction", 0.0, True, 1.0, 100.0),
    PercentageScale("percent", 1.0, False, 100.0, 1.0),
)


def battery_percent(percentage: Any) -> float | None:
    """``BatteryState.percentage`` as 0..100 by :data:`PERCENTAGE_SCALES`, else None."""
    value = _finite(percentage)
    if value is None:
        return None
    scale = next((row for row in PERCENTAGE_SCALES if row.holds(value)), None)
    return None if scale is None else round(value * scale.factor, 1)


def battery_payload(percentage: Any, voltage: Any) -> dict[str, float | None]:
    """``sensor_msgs/BatteryState`` fields as the battery channel's payload.

    ``percentage`` is NaN when unmeasured; its scale is read by
    :data:`PERCENTAGE_SCALES`, and a value on no scale is left out.
    """
    volts = _finite(voltage)
    return {
        "percent": battery_percent(percentage),
        "voltage_v": round(volts, 2) if volts is not None else None,
    }


@dataclass
class NavigationRecord:
    """The latest navigation goal this connection sent, as an HMI follows it."""

    call_id: str
    goal_x: float | None
    goal_y: float | None
    updated_at: float
    state: str = "navigating"
    distance_m: float | None = None
    baseline_m: float | None = None
    reason: str = ""

    def saw_feedback(self, values: Mapping[str, Any] | None, at: float) -> None:
        if not isinstance(values, Mapping) or self.state != "navigating":
            return
        remaining = _finite(values.get("distance_remaining"))
        if remaining is None or remaining < 0:
            return
        self.updated_at = at
        # Nav2 reports 0 until it has planned a path: not a reading yet.
        if remaining == 0.0 and self.baseline_m is None:
            return
        self.distance_m = remaining
        if self.baseline_m is None or remaining > self.baseline_m:
            self.baseline_m = remaining

    def end(self, reason: str, at: float, *, reached: bool = False) -> None:
        self.reason = str(reason or "")
        self.state = "arrived" if reached else ENDED_STATES.get(self.reason, ENDED_DEFAULT_STATE)
        self.updated_at = at
        if self.state == "arrived":
            self.distance_m = 0.0

    def progress_pct(self) -> float | None:
        if self.state == "arrived":
            return 100.0
        if self.distance_m is None or not self.baseline_m:
            return None
        done = 1.0 - self.distance_m / self.baseline_m
        return round(min(100.0, max(0.0, done * 100.0)), 1)

    def payload(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "state": self.state,
            "distance_m": None if self.distance_m is None else round(self.distance_m, 3),
            "progress_pct": self.progress_pct(),
        }
        if self.goal_x is not None and self.goal_y is not None:
            payload["goal_x"] = round(self.goal_x, 3)
            payload["goal_y"] = round(self.goal_y, 3)
        if self.reason:
            payload["reason"] = self.reason
        return payload

    def copy(self) -> NavigationRecord:
        return copy.copy(self)


def _window(points: list[tuple[float, float]]) -> dict[str, float]:
    xs = [x for x, _ in points]
    ys = [y for _, y in points]
    centre_x = (min(xs) + max(xs)) / 2.0
    centre_y = (min(ys) + max(ys)) / 2.0
    half = max(
        VIEW_MIN_HALF_SPAN_M,
        (max(xs) - min(xs)) / 2.0 + VIEW_MARGIN_M,
        (max(ys) - min(ys)) / 2.0 + VIEW_MARGIN_M,
    )
    return {
        "view_x_min": round(centre_x - half, 2),
        "view_x_max": round(centre_x + half, 2),
        "view_y_min": round(centre_y - half, 2),
        "view_y_max": round(centre_y + half, 2),
    }


def pose_payload(
    odom_pose: Mapping[str, Any] | None,
    map_pose: Mapping[str, Any] | None,
    navigation: NavigationRecord | None,
) -> dict[str, Any] | None:
    """The pose channel's payload: the map pose when there is one, else odometry."""
    pose = map_pose if isinstance(map_pose, Mapping) else odom_pose
    if not isinstance(pose, Mapping):
        return None
    x, y, yaw = (_finite(pose.get(key)) for key in ("x", "y", "yaw"))
    if x is None or y is None:
        return None
    frame = "map" if pose is map_pose else "odom"
    payload: dict[str, Any] = {"x": round(x, 3), "y": round(y, 3), "frame": frame}
    if yaw is not None:
        payload["heading_deg"] = round(math.degrees(math.remainder(yaw, math.tau)), 1)
    points = [(x, y)]
    # A goal is given in the map frame; drawn beside an odometry pose it
    # would sit in the wrong place, so it is shown only beside a map pose.
    if (
        frame == "map"
        and navigation is not None
        and navigation.goal_x is not None
        and navigation.goal_y is not None
    ):
        payload["goal_x"] = round(navigation.goal_x, 3)
        payload["goal_y"] = round(navigation.goal_y, 3)
        points.append((navigation.goal_x, navigation.goal_y))
    payload.update(_window(points))
    return payload


def _wall_time(monotonic_at: float | None, now_monotonic: float, now_wall: float) -> str:
    at = now_wall if monotonic_at is None else now_wall - max(0.0, now_monotonic - monotonic_at)
    return datetime.fromtimestamp(at, tz=timezone.utc).isoformat()


def _newest(items: Iterable[tuple[float | None, Any]]) -> tuple[float | None, Any]:
    found: tuple[float | None, Any] = (None, None)
    for seen_at, value in items:
        if value is None:
            continue
        if found[1] is None or (seen_at or 0.0) > (found[0] or 0.0):
            found = (seen_at, value)
    return found


def samples(
    resource_id: str,
    readings: Iterable[Mapping[str, Any] | None],
    *,
    now_monotonic: float | None = None,
    now_wall: float | None = None,
) -> list[dict[str, Any]]:
    """One sample per channel with a current reading, from every live connection.

    ``readings`` are ``telemetry_readings()`` of each connection to the robot
    (the presence watch, the adapters jobs run on). Battery and odometry come
    from the freshest connection; a map pose wins over odometry; the newest
    navigation record wins. With no connection there is nothing to say, so
    nothing is sampled: the host's ``connection`` channel says why.
    """
    current = [item for item in readings if isinstance(item, Mapping)]
    if not current:
        return []
    mono = time.monotonic() if now_monotonic is None else now_monotonic
    wall = time.time() if now_wall is None else now_wall

    def newest(value_key: str, seen_key: str) -> tuple[float | None, Any]:
        return _newest((item.get(seen_key), item.get(value_key)) for item in current)

    battery_at, battery = newest("battery", "battery_seen_at")
    map_at, map_pose = newest("map_pose", "pose_seen_at")
    odom_at, odom_pose = newest("pose", "pose_seen_at")
    navigation_records = [
        item["navigation"]
        for item in current
        if isinstance(item.get("navigation"), NavigationRecord)
    ]
    navigation = max(navigation_records, key=lambda record: record.updated_at, default=None)

    found: list[tuple[str, dict[str, Any] | None, float | None]] = [
        (CHANNEL_BATTERY, dict(battery) if isinstance(battery, Mapping) else None, battery_at),
        (
            CHANNEL_POSE,
            pose_payload(odom_pose, map_pose, navigation),
            map_at if map_pose is not None else odom_at,
        ),
        (
            CHANNEL_NAVIGATION,
            navigation.payload() if navigation is not None else {"state": "idle"},
            None,
        ),
    ]
    return [
        {
            "resource_id": resource_id,
            "channel_id": channel_id,
            "payload": payload,
            "quality": "good",
            "observed_at": _wall_time(seen_at, mono, wall),
        }
        for channel_id, payload, seen_at in found
        if payload is not None
    ]


__all__ = [
    "BATTERY_MAX_AGE_S",
    "CHANNELS",
    "CHANNEL_BATTERY",
    "CHANNEL_NAVIGATION",
    "CHANNEL_POSE",
    "ENDED_STATES",
    "NAVIGATION_STATES",
    "NavigationRecord",
    "PERCENTAGE_SCALES",
    "PercentageScale",
    "VIEW_MIN_HALF_SPAN_M",
    "battery_payload",
    "battery_percent",
    "channel_declarations",
    "pose_payload",
    "samples",
]
