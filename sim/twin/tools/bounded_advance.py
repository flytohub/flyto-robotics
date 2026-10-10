"""Bounded movement through the formal Generic ROS 2 Adapter over rosbridge.

Works unchanged against the robot (through `ssh -L 19090:127.0.0.1:9090
ubuntu@flyto-robot.local`) and against the twin (`docker compose up`).

    python3 sim/twin/tools/bounded_advance.py preflight
    python3 sim/twin/tools/bounded_advance.py advance [distance_m] [speed_mps]

preflight: discovery and observation only, no motion.
advance:   one motion.advance (default 0.05 m at 0.02 m/s), then a zero-velocity
           safe stop, with pose and clearance evidence before and after. The
           adapter refuses motion below its 0.35 m LiDAR clearance floor, and
           refuses when FLYTO_ROS2_DEPLOYMENT_MODE does not match the graph.

On the real robot, a person must be present and the area clear before advance.
Set FLYTO_ROS2_DEPLOYMENT_MODE=simulation for the twin.
"""

import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from flyto_robotics.adapter_contract import CallRequest  # noqa: E402
from flyto_robotics.generic_ros2_adapter import (  # noqa: E402
    GenericROS2Adapter,
    RosbridgeROS2Backend,
)


def snapshot(adapter):
    observation = adapter.backend.observation()
    pose = observation.get("pose") or {}
    scan = observation.get("range") or {}
    return {
        "pose": {key: pose.get(key) for key in ("x", "y", "yaw")},
        "min_range_m": scan.get("minimum_range_m"),
        "map_tf": observation.get("map_tf_available"),
    }


def advance(adapter, before):
    distance = float(sys.argv[2]) if len(sys.argv) > 2 else 0.05
    speed = float(sys.argv[3]) if len(sys.argv) > 3 else 0.02
    result = adapter.invoke(
        CallRequest(
            call_id=f"bounded-advance-{int(time.time())}",
            capability_id="motion.advance",
            arguments={"distance_m": distance, "speed_mps": speed},
            deadline_seconds=20.0,
        )
    )
    print("advance result:", result.outcome, "|", (result.detail or "")[:300])
    stop = adapter.safe_stop()
    print("safe_stop:", stop.outcome, "|", (stop.detail or "")[:200])
    time.sleep(2.0)
    after = snapshot(adapter)
    time.sleep(1.5)
    settled = snapshot(adapter)
    print("after:", json.dumps(after))
    print("settled:", json.dumps(settled))
    if before["pose"]["x"] is None or after["pose"]["x"] is None:
        return
    dx = after["pose"]["x"] - before["pose"]["x"]
    dy = after["pose"]["y"] - before["pose"]["y"]
    yaw = before["pose"]["yaw"] or 0.0
    forward = dx * math.cos(yaw) + dy * math.sin(yaw)
    still_moving = abs((settled["pose"]["x"] or 0) - (after["pose"]["x"] or 0)) > 0.003
    print(
        f"odom displacement along heading: {forward:+.3f} m "
        f"(total {math.hypot(dx, dy):.3f} m); still moving after stop: {still_moving}"
    )


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "preflight"
    adapter = GenericROS2Adapter(
        backend=RosbridgeROS2Backend(url="ws://127.0.0.1:19090"), resource_id="flyto-robot"
    )
    adapter.reconnect()
    try:
        deadline = time.time() + 20
        while time.time() < deadline:
            state = snapshot(adapter)
            if state["pose"].get("x") is not None and state["min_range_m"] is not None:
                break
            time.sleep(0.5)
        print("capabilities:", sorted(item.capability_id for item in adapter.describe()))
        before = snapshot(adapter)
        print("before:", json.dumps(before))
        if mode == "advance":
            advance(adapter, before)
    finally:
        adapter.disconnect()


if __name__ == "__main__":
    main()
