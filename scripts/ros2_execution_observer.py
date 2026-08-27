#!/usr/bin/env python3
"""Observe the exact active-then-motion boundary for runtime fault injection."""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def main() -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    parser = argparse.ArgumentParser()
    parser.add_argument("--active-marker", type=Path, required=True)
    parser.add_argument("--motion-marker", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-seconds", type=float, default=120.0)
    parser.add_argument("--motion-threshold-m", type=float, default=0.05)
    args = parser.parse_args()
    if not math.isfinite(args.timeout_seconds) or not 1 <= args.timeout_seconds <= 300:
        raise SystemExit("timeout is outside the safe range")
    if not math.isfinite(args.motion_threshold_m) or not 0.01 <= args.motion_threshold_m <= 1:
        raise SystemExit("motion threshold is outside the safe range")

    args.active_marker.unlink(missing_ok=True)
    args.motion_marker.unlink(missing_ok=True)
    state: dict[str, object] = {
        "contract_version": "flyto.robotics.execution-observer.v1",
        "active_observed": False,
        "motion_observed": False,
        "active_at": None,
        "motion_at": None,
        "motion_x_m": None,
        "threshold_m": args.motion_threshold_m,
    }
    rclpy.init()
    node = Node("flyto_resilience_execution_observer")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )

    def on_active(message: Bool) -> None:
        if message.data and state["active_observed"] is False:
            state["active_observed"] = True
            state["active_at"] = _utc_now()
            args.active_marker.write_text(str(state["active_at"]) + "\n")

    def on_odometry(message: Odometry) -> None:
        if state["active_observed"] is not True or state["motion_observed"] is True:
            return
        position = float(message.pose.pose.position.x)
        if abs(position) >= args.motion_threshold_m:
            state["motion_observed"] = True
            state["motion_at"] = _utc_now()
            state["motion_x_m"] = position
            args.motion_marker.write_text(str(state["motion_at"]) + "\n")

    active_sub = node.create_subscription(
        Bool,
        "/flyto/navigation_execution_active",
        on_active,
        qos,
    )
    odometry_sub = node.create_subscription(
        Odometry,
        "/flyto/odom",
        on_odometry,
        20,
    )
    deadline = time.monotonic() + args.timeout_seconds
    while time.monotonic() < deadline and state["motion_observed"] is not True:
        rclpy.spin_once(node, timeout_sec=0.05)
    state["finished_at"] = _utc_now()
    state["passed"] = state["active_observed"] is True and state["motion_observed"] is True
    args.output.write_text(json.dumps(state, ensure_ascii=False, indent=2) + "\n")
    node.destroy_subscription(active_sub)
    node.destroy_subscription(odometry_sub)
    node.destroy_node()
    rclpy.shutdown()
    if state["passed"] is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
