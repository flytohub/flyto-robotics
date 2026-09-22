#!/usr/bin/env python3
"""Measure fresh standard ROS 2 observations without changing robot state."""

from __future__ import annotations

import json
import time

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, LaserScan
from tf2_msgs.msg import TFMessage


def main() -> int:
    rclpy.init()
    node = Node("native_ros2_soak_probe")
    keys = ("odom", "scan", "camera", "map", "map_tf")
    counts = {key: 0 for key in keys}
    last: dict[str, float] = {}

    def hit(key: str) -> None:
        counts[key] += 1
        last[key] = time.monotonic()

    node.create_subscription(Odometry, "/odom", lambda _m: hit("odom"), 10)
    node.create_subscription(
        LaserScan,
        "/scan",
        lambda _m: hit("scan"),
        qos_profile_sensor_data,
    )
    node.create_subscription(
        Image,
        "/camera/image_raw",
        lambda _m: hit("camera"),
        qos_profile_sensor_data,
    )
    map_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(
        OccupancyGrid,
        "/map",
        lambda _m: hit("map"),
        map_qos,
    )

    def tf_callback(message: TFMessage) -> None:
        if any(
            transform.header.frame_id == "map"
            and transform.child_frame_id == "odom"
            for transform in message.transforms
        ):
            hit("map_tf")

    node.create_subscription(TFMessage, "/tf", tf_callback, 10)

    started = time.monotonic()
    duration_seconds = 30.0
    while time.monotonic() - started < duration_seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    now = time.monotonic()
    result = {
        "duration_seconds": duration_seconds,
        "counts": counts,
        "age_seconds": {
            key: round(now - last[key], 3)
            for key in last
        },
        "passed": all(counts[key] > 0 for key in keys)
        and all(now - last[key] < 5.0 for key in ("odom", "scan", "camera", "map_tf")),
    }
    print(json.dumps(result, sort_keys=True))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
