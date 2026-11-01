"""SIMULATION symbolic cargo state of the mobile robot's tray.

This is not a sensor and not physics. It is the one place that decides
whether the simulated tray holds the block:

- the tray starts empty (it is the simulation's own state, so that is known);
- a place reported by the fixed arm loads the tray only while the robot is
  parked at the station: a fresh odometry pose within the station tolerance
  and the chassis still. A missing, stale or malformed pose is unknown, and
  an unknown pose refuses the place; it is never assumed to be at the station;
- once loaded, the cargo stays with the robot wherever it drives;
- only an explicit unload clears it.

Every decision is recorded as an event so a run can show why the tray holds
what it holds.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Callable

#: The chassis counts as still below these odometry speeds.
PARKED_LINEAR_MPS = 0.01
PARKED_ANGULAR_RPS = 0.02
MAX_EVENTS = 50


@dataclass(frozen=True)
class Station:
    """Where the robot parks for the handoff, in the odometry frame."""

    x: float
    y: float
    tolerance_m: float = 0.10

    def as_dict(self) -> dict[str, float]:
        return {"x": self.x, "y": self.y, "tolerance_m": self.tolerance_m}


@dataclass(frozen=True)
class ParkedPose:
    x: float
    y: float
    linear_mps: float
    angular_rps: float

    @property
    def still(self) -> bool:
        return self.linear_mps <= PARKED_LINEAR_MPS and self.angular_rps <= PARKED_ANGULAR_RPS


def _number(value: Any) -> float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    return number if math.isfinite(number) else None


def _path(message: Any, *keys: str) -> Any:
    for key in keys:
        if not isinstance(message, Mapping):
            return None
        message = message.get(key)
    return message


def parked_pose(message: Any) -> ParkedPose | None:
    """Position and speed from a nav_msgs/Odometry message, or None if any is unreadable."""
    x = _number(_path(message, "pose", "pose", "position", "x"))
    y = _number(_path(message, "pose", "pose", "position", "y"))
    vx = _number(_path(message, "twist", "twist", "linear", "x"))
    vy = _number(_path(message, "twist", "twist", "linear", "y"))
    wz = _number(_path(message, "twist", "twist", "angular", "z"))
    if None in (x, y, vx, vy, wz):
        return None
    return ParkedPose(x, y, math.hypot(vx, vy), abs(wz))  # type: ignore[arg-type]


class CargoTray:
    """Thread-safe symbolic cargo state of one simulated mobile tray."""

    def __init__(
        self,
        station: Station,
        *,
        clock: Callable[[], float] = time.monotonic,
        pose_max_age_s: float = 1.0,
    ) -> None:
        self.station = station
        self._clock = clock
        self._pose_max_age_s = pose_max_age_s
        self._lock = threading.Lock()
        self._loaded = False
        self._pose: ParkedPose | None = None
        self._pose_at: float | None = None
        self._events: list[dict[str, Any]] = []

    @property
    def cargo_present(self) -> bool:
        with self._lock:
            return self._loaded

    def observe_odom(self, message: Any) -> None:
        pose = parked_pose(message)
        with self._lock:
            # An unreadable message forgets the last pose instead of keeping
            # an old one alive: the robot's position is then unknown.
            self._pose = pose
            self._pose_at = self._clock() if pose is not None else None

    def _fresh_pose(self) -> ParkedPose | None:
        if self._pose is None or self._pose_at is None:
            return None
        if self._clock() - self._pose_at > self._pose_max_age_s:
            return None
        return self._pose

    def _record(self, event: dict[str, Any]) -> dict[str, Any]:
        event = {"simulation": True, "at": time.time(), **event}
        self._events.append(event)
        del self._events[:-MAX_EVENTS]
        return dict(event)

    def report_place(self, *, source: str) -> dict[str, Any]:
        """The fixed arm reports it placed the block into the tray."""
        with self._lock:
            pose = self._fresh_pose()
            base: dict[str, Any] = {"kind": "place", "source": source}
            if pose is None:
                return self._record({**base, "accepted": False, "reason": "robot_pose_unknown"})
            distance = math.hypot(pose.x - self.station.x, pose.y - self.station.y)
            base["robot_pose"] = {"x": round(pose.x, 4), "y": round(pose.y, 4)}
            base["station_distance_m"] = round(distance, 4)
            if distance > self.station.tolerance_m:
                return self._record({**base, "accepted": False, "reason": "robot_not_at_station"})
            if not pose.still:
                return self._record({**base, "accepted": False, "reason": "robot_not_parked"})
            self._loaded = True
            return self._record({**base, "accepted": True, "reason": "placed_while_parked"})

    def unload(self, *, source: str) -> dict[str, Any]:
        with self._lock:
            self._loaded = False
            return self._record(
                {"kind": "unload", "source": source, "accepted": True, "reason": "unloaded"}
            )

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            pose = self._fresh_pose()
            return {
                "simulation": True,
                "source": "symbolic_tray_state_simulation_not_a_sensor",
                "cargo_present": self._loaded,
                "station": self.station.as_dict(),
                "robot_pose": None if pose is None else {"x": pose.x, "y": pose.y},
                "events": [dict(event) for event in self._events],
            }
