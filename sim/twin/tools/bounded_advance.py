"""Bounded movement through the formal Generic ROS 2 Adapter over rosbridge.

Works unchanged against the robot (through `ssh -L 19090:127.0.0.1:9090
ubuntu@flyto-robot.local`) and against the twin (`docker compose up`).

    python3 sim/twin/tools/bounded_advance.py preflight
    python3 sim/twin/tools/bounded_advance.py advance [distance_m] [speed_mps]

preflight: discovery and observation only, no motion.
advance:   one motion.advance (default 0.05 m at 0.02 m/s), then a zero-velocity
           safe stop, with pose and clearance evidence before and after. The
           adapter refuses motion below its 0.35 m LiDAR clearance floor.

On the real robot, a person must be present and the area clear before advance.
Set FLYTO_ROS2_DEPLOYMENT_MODE=simulation for the twin.
"""
import json, sys, time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from flyto_robotics.adapter_contract import CallRequest
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend

mode = sys.argv[1] if len(sys.argv) > 1 else "preflight"
adapter = GenericROS2Adapter(backend=RosbridgeROS2Backend(url="ws://127.0.0.1:19090"), resource_id="flyto-robot")

def snapshot():
    obs = adapter.backend.observation()
    pose = obs.get("pose") or {}
    rng = obs.get("range") or {}
    return {"pose": {k: pose.get(k) for k in ("x", "y", "yaw")}, "min_range_m": rng.get("minimum_range_m"), "map_tf": obs.get("map_tf_available")}

deadline = time.time() + 20
while time.time() < deadline:
    s = snapshot()
    if s["pose"].get("x") is not None and s["min_range_m"] is not None:
        break
    time.sleep(0.5)
caps = sorted(d.capability_id for d in adapter.describe())
print("capabilities:", caps)
before = snapshot()
print("before:", json.dumps(before))
if mode == "advance":
    result = adapter.invoke(CallRequest(call_id=f"bounded-advance-{int(time.time())}", capability_id="motion.advance",
                                        arguments={"distance_m": float(sys.argv[2]) if len(sys.argv) > 2 else 0.05, "speed_mps": float(sys.argv[3]) if len(sys.argv) > 3 else 0.02}, deadline_seconds=20.0))
    print("advance result:", result.outcome, "|", (result.detail or "")[:300])
    stop = adapter.safe_stop()
    print("safe_stop:", stop.outcome, "|", (stop.detail or "")[:200])
    time.sleep(2.0)
    after = snapshot()
    time.sleep(1.5)
    settled = snapshot()
    print("after:", json.dumps(after))
    print("settled:", json.dumps(settled))
    if before["pose"]["x"] is not None and after["pose"]["x"] is not None:
        import math
        dx, dy = after["pose"]["x"] - before["pose"]["x"], after["pose"]["y"] - before["pose"]["y"]
        yaw = before["pose"]["yaw"] or 0.0
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        print(f"odom displacement along heading: {forward:+.3f} m (total {math.hypot(dx, dy):.3f} m); still moving after stop: {abs((settled['pose']['x'] or 0) - (after['pose']['x'] or 0)) > 0.003}")
adapter.disconnect()
