"""The robot's pose in the map frame, from odometry and the map->odom transform.

Odometry says where the base is in the ``odom`` frame, which drifts and
restarts at zero with the driver. Localization (SLAM or AMCL) publishes the
``map``->``odom`` transform that corrects it. Composing the two gives the pose
a navigation goal is written in. Pure: no ROS, no clock.

Motion verification stays on odometry; the map pose is reported beside it so
a host can name a place in the frame a navigation goal is given in.
"""

from __future__ import annotations

import math
from collections.abc import Mapping
from typing import Any

MAP_FRAME = "map"


def yaw_of(x: float, y: float, z: float, w: float) -> float:
    """Heading (rotation about z) of a quaternion, in [-pi, pi]."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _finite(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def planar_transform(
    translation: Mapping[str, Any] | Any, rotation: Mapping[str, Any] | Any
) -> tuple[float, float, float] | None:
    """(x, y, yaw) of a transform given as message fields or a mapping, or None.

    Accepts rclpy messages (attributes) and rosbridge JSON (keys) alike; a
    field that is missing or not a finite number makes the transform unusable.
    """

    def field(source: Any, name: str, default: float | None) -> float | None:
        if isinstance(source, Mapping):
            raw = source.get(name, default)
        else:
            raw = getattr(source, name, default)
        return _finite(raw) if raw is not None else None

    x = field(translation, "x", None)
    y = field(translation, "y", None)
    qx = field(rotation, "x", 0.0)
    qy = field(rotation, "y", 0.0)
    qz = field(rotation, "z", 0.0)
    qw = field(rotation, "w", None)
    if None in (x, y, qx, qy, qz, qw):
        return None
    return float(x), float(y), yaw_of(qx, qy, qz, qw)


def compose(
    map_from_odom: tuple[float, float, float] | None, odom_pose: Mapping[str, Any] | None
) -> dict[str, float | str] | None:
    """The odometry pose expressed in the map frame, or None when either is missing."""
    if map_from_odom is None or not isinstance(odom_pose, Mapping):
        return None
    x = _finite(odom_pose.get("x"))
    y = _finite(odom_pose.get("y"))
    yaw = _finite(odom_pose.get("yaw"))
    if None in (x, y, yaw):
        return None
    tx, ty, tyaw = map_from_odom
    cos, sin = math.cos(tyaw), math.sin(tyaw)
    return {
        "frame": MAP_FRAME,
        "x": tx + cos * x - sin * y,
        "y": ty + sin * x + cos * y,
        "yaw": math.remainder(tyaw + yaw, math.tau),
    }


__all__ = ["MAP_FRAME", "compose", "planar_transform", "yaw_of"]
