#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
instance_name="${FLYTO_GAZEBO_LIMA_INSTANCE:-flyto-robot-gazebo}"
run_id="${FLYTO_GAZEBO_VERIFY_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
result_directory="${repository_root}/results/virtual-robot/${run_id}"
mkdir -p "${result_directory}"

"${repository_root}/scripts/run-lima-gazebo.sh" --no-gateway

limactl shell "${instance_name}" bash -s <<'GUEST'
set -eo pipefail
workspace_root="${HOME}/.local/share/flyto-robot-gazebo/workspace"
runtime_root="${HOME}/.local/share/flyto-robot-gazebo/runtime"
source /opt/ros/jazzy/setup.bash
source "${workspace_root}/install/setup.bash"
export ROS_DOMAIN_ID=30

python3 - <<'PY'
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan


class FidelityProbe(Node):
    def __init__(self) -> None:
        super().__init__("flyto_turtlebot3_fidelity_probe")
        self.odom: list[Odometry] = []
        self.scan: LaserScan | None = None
        self.imu: Imu | None = None
        self.publisher = self.create_publisher(Twist, "/flyto/cmd_vel", 10)
        self.create_subscription(Odometry, "/flyto/odom", self.odom.append, 20)
        self.create_subscription(
            LaserScan, "/flyto/scan", self._scan, qos_profile_sensor_data
        )
        self.create_subscription(Imu, "/flyto/imu", self._imu, qos_profile_sensor_data)

    def _scan(self, message: LaserScan) -> None:
        self.scan = message

    def _imu(self, message: Imu) -> None:
        self.imu = message


def spin_until(node: Node, condition: object, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if condition():
            return
    raise RuntimeError("sensor readiness timeout")


rclpy.init()
probe = FidelityProbe()
try:
    spin_until(
        probe,
        lambda: len(probe.odom) >= 3 and probe.scan is not None and probe.imu is not None,
        30.0,
    )
    assert probe.scan is not None
    assert probe.imu is not None
    assert len(probe.scan.ranges) >= 350
    assert 0.115 <= probe.scan.range_min <= 0.125
    assert 3.4 <= probe.scan.range_max <= 3.6
    start = probe.odom[-1].pose.pose.position
    command = Twist()
    command.linear.x = 0.06
    motion_deadline = time.monotonic() + 1.25
    while time.monotonic() < motion_deadline:
        probe.publisher.publish(command)
        rclpy.spin_once(probe, timeout_sec=0.08)
    stop = Twist()
    odometry_count_at_stop = len(probe.odom)
    stop_deadline = time.monotonic() + 3.0
    while time.monotonic() < stop_deadline:
        probe.publisher.publish(stop)
        rclpy.spin_once(probe, timeout_sec=0.08)
        fresh_odometry = len(probe.odom) >= odometry_count_at_stop + 3
        if fresh_odometry and abs(probe.odom[-1].twist.twist.linear.x) <= 0.03:
            break
    finish = probe.odom[-1].pose.pose.position
    displacement = math.hypot(finish.x - start.x, finish.y - start.y)
    final_speed = abs(probe.odom[-1].twist.twist.linear.x)
    assert 0.025 <= displacement <= 0.16, displacement
    assert final_speed <= 0.03, final_speed
    report = {
        "passed": True,
        "model": "turtlebot3_burger",
        "displacement_m": displacement,
        "final_speed_mps": final_speed,
        "lidar_samples": len(probe.scan.ranges),
        "lidar_range_m": [probe.scan.range_min, probe.scan.range_max],
        "imu_angular_velocity_finite": math.isfinite(probe.imu.angular_velocity.z),
    }
    output = Path.home() / ".local/share/flyto-robot-gazebo/runtime/normal-probe.json"
    output.write_text(json.dumps(report, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
finally:
    probe.publisher.publish(Twist())
    probe.destroy_node()
    rclpy.shutdown()
PY
GUEST

"${repository_root}/scripts/run-lima-gazebo.sh" \
  --fault lidar_dropout \
  --no-gateway

limactl shell "${instance_name}" bash -s <<'GUEST'
set -eo pipefail
workspace_root="${HOME}/.local/share/flyto-robot-gazebo/workspace"
source /opt/ros/jazzy/setup.bash
source "${workspace_root}/install/setup.bash"
export ROS_DOMAIN_ID=30

python3 - <<'PY'
import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class FaultProbe(Node):
    def __init__(self) -> None:
        super().__init__("flyto_turtlebot3_fault_probe")
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.fault = ""
        self.stopped = False
        self.fault_active_at: float | None = None
        self.stopped_at: float | None = None
        self.actuator: list[Twist] = []
        self.command = self.create_publisher(Twist, "/flyto/cmd_vel", 10)
        self.create_subscription(String, "/fault_injection/state", self._fault, state_qos)
        self.create_subscription(
            Bool, "/safety/emergency_stop_state", self._stopped, state_qos
        )
        self.create_subscription(
            Twist, "/flyto/actuator_cmd_vel", self.actuator.append, 20
        )

    def _fault(self, message: String) -> None:
        self.fault = message.data
        if message.data == "lidar_dropout:active" and self.fault_active_at is None:
            self.fault_active_at = time.monotonic()

    def _stopped(self, message: Bool) -> None:
        self.stopped = message.data
        if message.data and self.stopped_at is None:
            self.stopped_at = time.monotonic()


rclpy.init()
probe = FaultProbe()
try:
    discovery_deadline = time.monotonic() + 8.0
    while time.monotonic() < discovery_deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if (
            probe.fault == "lidar_dropout:ready"
            and probe.command.get_subscription_count() >= 2
        ):
            break
    assert probe.fault == "lidar_dropout:ready", probe.fault
    assert probe.command.get_subscription_count() >= 2
    command = Twist()
    command.linear.x = 0.06
    first_command_at = time.monotonic()
    deadline = first_command_at + 5.0
    while time.monotonic() < deadline and not probe.stopped:
        probe.command.publish(command)
        rclpy.spin_once(probe, timeout_sec=0.05)
    assert probe.fault == "lidar_dropout:active", probe.fault
    assert probe.stopped, "safety supervisor did not latch"
    assert probe.fault_active_at is not None
    assert probe.stopped_at is not None
    command_to_stop = probe.stopped_at - first_command_at
    fault_to_stop = probe.stopped_at - probe.fault_active_at
    latch_zero_deadline = time.monotonic() + 1.0
    while time.monotonic() < latch_zero_deadline:
        rclpy.spin_once(probe, timeout_sec=0.05)
        if probe.actuator and abs(probe.actuator[-1].linear.x) < 1e-9:
            break
    assert probe.actuator and abs(probe.actuator[-1].linear.x) < 1e-9
    probe.actuator.clear()
    for _sample in range(8):
        probe.command.publish(command)
        rclpy.spin_once(probe, timeout_sec=0.06)
    gated = probe.actuator
    assert gated, "no post-latch actuator command observed"
    assert all(
        abs(message.linear.x) < 1e-9 and abs(message.angular.z) < 1e-9
        for message in gated
    )
    assert command_to_stop <= 1.5, command_to_stop
    assert fault_to_stop <= 0.75, fault_to_stop
    report = {
        "passed": True,
        "fault": probe.fault,
        "safety_latched": probe.stopped,
        "command_to_stop_seconds": command_to_stop,
        "stop_latency_seconds": fault_to_stop,
        "post_latch_zero_commands": len(gated),
    }
    output = Path.home() / ".local/share/flyto-robot-gazebo/runtime/fault-probe.json"
    output.write_text(json.dumps(report, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, sort_keys=True))
finally:
    probe.command.publish(Twist())
    probe.destroy_node()
    rclpy.shutdown()
PY
GUEST

limactl shell "${instance_name}" bash -lc \
  'cat "$HOME/.local/share/flyto-robot-gazebo/runtime/normal-probe.json"' \
  >"${result_directory}/normal-probe.json"
limactl shell "${instance_name}" bash -lc \
  'cat "$HOME/.local/share/flyto-robot-gazebo/runtime/fault-probe.json"' \
  >"${result_directory}/fault-probe.json"

python3 - "${result_directory}" <<'PY'
import json
import sys
from pathlib import Path

root = Path(sys.argv[1])
normal = json.loads((root / "normal-probe.json").read_text(encoding="utf-8"))
fault = json.loads((root / "fault-probe.json").read_text(encoding="utf-8"))
assert normal["passed"] and fault["passed"]
report = {
    "passed": True,
    "robot": normal,
    "fault_safety": fault,
}
(root / "report.json").write_text(
    json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
print(json.dumps(report, sort_keys=True))
PY

"${repository_root}/scripts/run-lima-gazebo.sh"
echo "Virtual robot verification: ${result_directory}/report.json"
