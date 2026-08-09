#!/usr/bin/env bash
# Drive the robot one bounded step from an operator shell.
#
#   scripts/move-robot.sh forward|backward|left|right
#
# Exit codes: 0 moved, 2 bad usage, 3 refused (too close), 4 refused (blind),
# anything else is the mission's own exit code.
#
# The controller already stops for obstacles and already fails safe when a
# sensor is missing. This refuses *before* moving so a blocked run is a sentence
# rather than a near miss — and, unlike the ad-hoc version this replaces, it
# refuses when it cannot see at all. That one parsed the scan with a bare
# `except: print('99')`, so an unreadable lidar came back as 99 m of room and
# the check waved the robot through. It was 0.36 m from something at the time.

# ROS setup.bash reads unset variables, so -u cannot be on while sourcing it.
set -eo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

# shellcheck disable=SC1091
source "${ROS_SETUP:-/opt/ros/jazzy/setup.bash}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-30}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
# A 0.40 m step needs more than 0.40 m of room: the controller's own stop
# distance has to fit in front of where the step ends.
REQUIRED_CLEARANCE="${REQUIRED_CLEARANCE:-0.70}"

case "${1:-}" in
  forward)  PLAN=shortcut-forward-40cm  ; SECTOR=front ;;
  backward) PLAN=shortcut-backward-40cm ; SECTOR=rear  ;;
  left)     PLAN=shortcut-turn-left-90deg  ; SECTOR=none ;;
  right)    PLAN=shortcut-turn-right-90deg ; SECTOR=none ;;
  *) echo "usage: $(basename "$0") forward|backward|left|right" >&2; exit 2 ;;
esac

if [ "$SECTOR" != none ]; then
  set +e
  python3 - "$SECTOR" "$SCAN_TOPIC" "$REQUIRED_CLEARANCE" <<'PY'
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from flyto_robotics.scan_clearance import describe, is_clear, sector_clearance

sector, topic, required = sys.argv[1], sys.argv[2], float(sys.argv[3])
READ_TIMEOUT_SECONDS = 20.0

rclpy.init()
node = Node("clearance_probe")
received: list[LaserScan] = []
node.create_subscription(
    LaserScan, topic, received.append, qos_profile_sensor_data
)

deadline = time.monotonic() + READ_TIMEOUT_SECONDS
while not received and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.2)

# No message at all and a sweep with nothing usable in it are the same answer:
# the sector was not measured. Neither may be reported as room.
clearance = sector_clearance(received[-1].ranges, sector) if received else None
print(f"clearance {sector}: {describe(clearance)}")

node.destroy_node()
rclpy.shutdown()

if clearance is None:
    print(
        f"REFUSED: cannot see {sector}. No usable return on {topic} in "
        f"{READ_TIMEOUT_SECONDS:.0f}s, so there is no clearance to check.",
        file=sys.stderr,
    )
    sys.exit(4)
if not is_clear(clearance, required):
    print(
        f"REFUSED: needs {required:.2f} m to move 0.40 m safely.",
        file=sys.stderr,
    )
    sys.exit(3)
PY
  probe_status=$?
  set -e
  [ "$probe_status" -eq 0 ] || exit "$probe_status"

  # No settle pause here on purpose. It was suspected that the probe's DDS
  # teardown delayed the mission's discovery, so a pause was added. Twelve
  # alternating runs on the robot refuted it: median odometry discovery was
  # 2.11s without a preceding probe and 2.07s with one. The latency is
  # intrinsic and highly variable (7ms to 2.6s in that sample, 9.1s seen
  # earlier), and nothing this script does in front of it moves the number.
fi

python3 - "$PLAN" <<'PY'
import json
import sys
from pathlib import Path

from flyto_robotics.ros2_node import run

code = run(
    Path("examples/jobs/tb3-lab-shortcut.json"),
    Path("/tmp/move-result.json"),
    plan_path=Path(f"examples/plans/{sys.argv[1]}.json"),
    ros_args=[
        "--ros-args",
        "-p", "cmd_vel_topic:=/cmd_vel",
        "-p", "odom_topic:=/odom",
        "-p", "scan_topic:=/scan",
    ],
)
result = json.loads(Path("/tmp/move-result.json").read_text())
pose = result.get("final_pose") or {}
print(
    f"{result['status']}  x={pose.get('x')} y={pose.get('y')} "
    f"yaw={pose.get('yaw')}  safety_stops={result.get('safety_stop_count')}"
)
for event in result.get("events", []):
    if event["kind"] == "primitive_completed":
        print("  " + event["detail"])
sys.exit(code)
PY
