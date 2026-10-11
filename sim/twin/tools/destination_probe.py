"""Twin only: navigate to a destination `gap` metres in front of the nearest obstacle ahead.

The run c6d8d79d check (0.6.7) on the twin, never on the robot:

    python3 sim/twin/tools/destination_probe.py ws://127.0.0.1:19090 [gap]

With the default gap (0.15 m, inside the 0.35 m floor) the call must be
refused with `destination_blocked` and no navigate goal sent; 0.6.6 sent the
goal and drove at the obstacle. The tool forces
FLYTO_ROS2_DEPLOYMENT_MODE=simulation, so a graph without /clock (the
physical robot) refuses the motion.
"""

import json
import math
import os
import sys
import time
from pathlib import Path

# Simulation only, always: a graph without /clock (the robot) refuses motion.
os.environ["FLYTO_ROS2_DEPLOYMENT_MODE"] = "simulation"
sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from flyto_robotics.adapter_contract import CallRequest  # noqa: E402
from flyto_robotics.generic_ros2_adapter import (  # noqa: E402
    GenericROS2Adapter,
    RosbridgeROS2Backend,
)

url = sys.argv[1] if len(sys.argv) > 1 else "ws://127.0.0.1:19090"
gap = float(sys.argv[2]) if len(sys.argv) > 2 else 0.15
backend = RosbridgeROS2Backend(url=url)
sent: list[dict] = []
original_send = backend._send


def recording_send(message):
    if message.get("op") == "send_action_goal":
        sent.append({"id": message.get("id"), "action": message.get("action")})
    return original_send(message)


backend._send = recording_send
adapter = GenericROS2Adapter(backend=backend, resource_id="twin-probe")
adapter.reconnect()
try:
    deadline = time.time() + 60
    observation = {}
    while time.time() < deadline:
        observation = backend.observation(required=("pose", "range", "map_tf"))
        if observation.get("map_pose") and (observation.get("range") or {}).get("sweep"):
            break
        time.sleep(1.0)
    caps = sorted(item.capability_id for item in adapter.describe())
    print("capabilities:", caps)
    sweep = observation["range"]["sweep"]
    pose = observation["map_pose"]
    start = float(sweep["angle_min_rad"])
    step = float(sweep["angle_increment_rad"])
    best = None
    for index, value in enumerate(sweep["ranges_m"]):
        if value is None or not math.isfinite(value) or value <= 0:
            continue
        bearing = math.remainder(start + step * index, math.tau)
        if abs(bearing) <= math.radians(60) and (best is None or value < best[0]):
            best = (value, bearing)
    print("nearest ahead:", best, "map_pose:", pose)
    distance = best[0] - gap
    rx, ry = distance * math.cos(best[1]), distance * math.sin(best[1])
    yaw = float(pose["yaw"])
    x = float(pose["x"]) + math.cos(yaw) * rx - math.sin(yaw) * ry
    y = float(pose["y"]) + math.sin(yaw) * rx + math.cos(yaw) * ry
    result = adapter.invoke(
        CallRequest(
            call_id=f"probe-{int(time.time())}",
            capability_id="motion.navigate",
            arguments={"x": round(x, 3), "y": round(y, 3)},
            deadline_seconds=8.0,
        )
    )
    evidence = result.evidence or {}
    print(json.dumps({
        "outcome": result.outcome,
        "reason_code": evidence.get("reason_code"),
        "motion_reason": (evidence.get("motion_outcome") or {}).get("reason"),
        "destination_clearance": evidence.get("destination_clearance"),
        "goals_sent": sent,
        "detail": (result.detail or "")[:300],
    }, indent=1))
finally:
    try:
        adapter.safe_stop()
    except Exception as error:  # noqa: BLE001
        print("safe_stop:", error)
    adapter.disconnect()
