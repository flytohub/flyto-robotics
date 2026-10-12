"""What the robot can be watched through while it runs (``flyto.live-channels.v1``).

The execution host's live relay (flyto-cloud ``local/live_routes.py``) reads
each declared channel's host-side ``source`` and forwards its frames to a
viewer holding a watch token. For a ``rosbridge://host:port/<topic>?type=<type>``
source it opens its own bridge connection, subscribes to that one topic with a
throttle of ``max_rate_hz`` and normalises each message by its type. It only
subscribes: a live channel is never a command path.

One row per channel in :data:`LIVE_CHANNELS`. A channel is declared only when
the graph the adapter just read publishes its topic with its type, and only
when the adapter reaches the robot over a rosbridge the relay can read too
(:data:`SOURCE_SCHEMES`). The robot and its twin publish the same topics, so
both declare the same block. Cloud owns the contract; the values here mirror
its schema and source adapters by value and are pinned in
``tests/test_live_channels.py``.
"""

from __future__ import annotations

import os
from collections.abc import Callable, Mapping
from dataclasses import dataclass, field
from typing import Any
from urllib.parse import urlsplit, urlunsplit

#: Adapter bridge scheme -> the relay's source scheme for it.
SOURCE_SCHEMES: Mapping[str, str] = {"ws": "rosbridge", "wss": "rosbridges"}


def _clearance_floor() -> float:
    # The floor the adapter enforces, read the way it reads it.
    from .generic_ros2_adapter import _clearance_floor as floor

    return floor()


@dataclass(frozen=True)
class LiveChannel:
    channel_id: str
    kind: str
    transport: str
    max_rate_hz: float
    #: The adapter's own setting for the topic, and its default.
    topic_env: str
    default_topic: str
    message_type: str
    label: Mapping[str, str]
    #: Extra fields computed when declared (a scan's floor).
    extras: Mapping[str, Callable[[], Any]] = field(default_factory=dict)

    def topic(self) -> str:
        return os.getenv(self.topic_env, self.default_topic).strip()


LIVE_CHANNELS: tuple[LiveChannel, ...] = (
    LiveChannel(
        "scan",
        "scan",
        "ws-json",
        5,
        "FLYTO_ROS2_SCAN_TOPIC",
        "/scan",
        "sensor_msgs/msg/LaserScan",
        {"zh-TW": "光達掃描", "en": "LiDAR scan"},
        {"floor_m": _clearance_floor},
    ),
    LiveChannel(
        "pose",
        "pose",
        "ws-json",
        5,
        "FLYTO_ROS2_ODOM_TOPIC",
        "/odom",
        "nav_msgs/msg/Odometry",
        {"zh-TW": "里程計位置", "en": "Odometry pose"},
    ),
    # A JPEG per frame on every viewer's socket: 5 Hz of the camera's 30.
    LiveChannel(
        "camera",
        "video",
        "ws-jpeg",
        5,
        "FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC",
        "/camera/image_raw/compressed",
        "sensor_msgs/msg/CompressedImage",
        {"zh-TW": "攝影機", "en": "Camera"},
    ),
)


def _same_type(published: Any, expected: str) -> bool:
    # rosapi spells a type with ``/msg/``; older bridges without it.
    return str(published or "").replace("/msg/", "/") == expected.replace("/msg/", "/")


def _bridge(url: str | None) -> tuple[str, str] | None:
    """``(source scheme, host:port)`` of a bridge the relay can read, else None."""
    parts = urlsplit(str(url or "").strip())
    scheme = SOURCE_SCHEMES.get(parts.scheme.lower())
    if scheme is None or not parts.hostname or parts.path not in ("", "/") or parts.query:
        return None
    return scheme, parts.netloc


def declarations(bridge_url: str | None, topics: Mapping[str, str]) -> list[dict[str, Any]]:
    """The manifest's ``live_channels`` for a robot reached at ``bridge_url``.

    ``topics`` is ``{topic: type}`` as the adapter last read the graph. Empty
    when there is no readable bridge or no channel's topic is published; the
    caller then leaves the field out (Cloud requires at least one channel).
    """
    bridge = _bridge(bridge_url)
    if bridge is None:
        return []
    scheme, netloc = bridge
    declared: list[dict[str, Any]] = []
    for row in LIVE_CHANNELS:
        topic = row.topic()
        if not topic.startswith("/") or not _same_type(topics.get(topic), row.message_type):
            continue
        source = urlunsplit((scheme, netloc, topic, f"type={row.message_type}", ""))
        declared.append(
            {
                "id": row.channel_id,
                "kind": row.kind,
                "transport": row.transport,
                "max_rate_hz": row.max_rate_hz,
                "label": dict(row.label),
                **{name: compute() for name, compute in row.extras.items()},
                "source": source,
            }
        )
    return declared


__all__ = ["LIVE_CHANNELS", "LiveChannel", "SOURCE_SCHEMES", "declarations"]
