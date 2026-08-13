#!/usr/bin/env bash
set -euo pipefail

repository_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
instance_name="${FLYTO_GAZEBO_LIMA_INSTANCE:-flyto-robot-gazebo}"
run_id="${FLYTO_GAZEBO_VERIFY_RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}"
result_directory="${repository_root}/results/virtual-robot/${run_id}"
mkdir -p "${result_directory}"

# Any probe failure aborts under `set -e` while the runtime is still pinned to
# --no-gateway and possibly to an injected fault. Restoration therefore lives in
# a single EXIT handler so it runs exactly once on both the success and the
# failure path, without masking the original verification status.
cleanup_entered=0
readonly cleanup_contract_version="flyto.robotics.runtime-cleanup.v1"

restore_runtime() {
  local status=$?
  # Clear EXIT so the handler cannot re-enter itself, and ignore INT/TERM so a
  # second signal cannot abort restoration halfway through.
  trap - EXIT
  trap '' INT TERM
  if [[ "${cleanup_entered}" -ne 0 ]]; then
    exit "${status}"
  fi
  cleanup_entered=1
  set +e

  # Best effort: zero the commanded velocity before the runtime is recycled.
  local zero_command_status=0
  limactl shell "${instance_name}" bash -lc '
    set -eo pipefail
    source /opt/ros/jazzy/setup.bash
    source "$HOME/.local/share/flyto-robot-gazebo/workspace/install/setup.bash"
    export ROS_DOMAIN_ID=30
    timeout 20 ros2 topic pub --once /flyto/cmd_vel geometry_msgs/msg/Twist "{}"
  ' >/dev/null 2>&1
  zero_command_status=$?

  # Required: return the instance to the normal, non-fault, gateway runtime.
  "${repository_root}/scripts/run-lima-gazebo.sh"
  local restore_status=$?

  # The cleanup contract is satisfied when the normal gateway runtime is back;
  # the zero command is recorded separately because it is best effort.
  local restored=true
  local zeroed=true
  [[ "${restore_status}" -eq 0 ]] || restored=false
  [[ "${zero_command_status}" -eq 0 ]] || zeroed=false
  local cleanup_passed="${restored}"

  if [[ "${restore_status}" -ne 0 ]]; then
    if [[ "${status}" -eq 0 ]]; then
      echo "runtime restoration failed (${restore_status})" >&2
      status="${restore_status}"
    else
      echo "warning: runtime restoration failed (${restore_status}); preserving" \
        "original verification failure status ${status}" >&2
    fi
  fi

  # Contract-level record of the cleanup guarantee for this run.
  # normal_runtime_restoration_exit_code is 0 when the normal non-fault gateway
  # runtime was restored successfully, and nonzero when restoration failed.
  printf '{"contract_version": "%s", "passed": %s, "restored_normal_gateway_runtime": %s, "normal_runtime_restoration_exit_code": %s, "zero_command_published": %s, "verification_status": %s}\n' \
    "${cleanup_contract_version}" "${cleanup_passed}" "${restored}" \
    "${restore_status}" "${zeroed}" "${status}" \
    >"${result_directory}/cleanup.json" 2>/dev/null

  exit "${status}"
}

trap restore_runtime EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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
import re
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

CONTRACT_VERSION = "flyto.robotics.burger-gazebo-acceptance.v1"
EVIDENCE_SOURCE = "gazebo_harmonic_world_state+ros2_jazzy_topics"
GROUND_TRUTH_SOURCE = "gz_transport_world_pose_info"
GROUND_TRUTH_TOPIC_PATTERN = re.compile(r"^/world/[^/]+/pose/info$")
EXPECTED_GROUND_TRUTH_TOPIC = "/world/flyto_turtlebot3_fidelity/pose/info"
GROUND_TRUTH_MODEL = "burger"
GROUND_TRUTH_MIN_DISPLACEMENT_M = 0.02
GROUND_TRUTH_MAX_DISPLACEMENT_M = 0.30
GROUND_TRUTH_MIN_SIM_ADVANCE_S = 0.1
COMMAND_GRAPH_MIN_SUBSCRIPTIONS = 2
COMMAND_GRAPH_READY_TIMEOUT_S = 30.0
# Cold-start physics settle. A fully connected ROS graph can still sit in front
# of a Gazebo world that has not stepped the chassis yet, which yields odometry
# motion with ~0m world motion. Require real simulation-time advance and a
# stable chassis pose before any sample or command.
PHYSICS_SETTLE_SOURCE = "gz_transport_world_pose_info_sim_time"
PHYSICS_SETTLE_MIN_SIM_S = 10.0
PHYSICS_SETTLE_TIMEOUT_S = 90.0
PHYSICS_SETTLE_MAX_DRIFT_M = 0.01
PHYSICS_SETTLE_SPIN_INTERVAL_S = 1.0


class GroundTruthError(RuntimeError):
    """Gazebo world state was missing, stale, unparsable, or non-finite."""


class CommandGraphError(RuntimeError):
    """The /flyto/cmd_vel command graph was not fully connected before motion."""


class PhysicsSettleError(RuntimeError):
    """Gazebo physics did not settle: no sim-time advance, or the chassis drifted."""


def run_gz(arguments: list[str], timeout: float) -> str:
    """Run a Gazebo Transport CLI query without touching ROS 2 topics."""
    command = ["timeout", "-k", "1", str(int(timeout)), "gz", *arguments]
    printable = " ".join(command[4:])
    try:
        completed = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout + 5.0
        )
    except subprocess.TimeoutExpired as error:
        raise GroundTruthError(f"gazebo ground truth timed out: {printable}") from error
    except OSError as error:
        raise GroundTruthError(
            f"gazebo ground truth unavailable: {printable}: {error}"
        ) from error
    if completed.returncode != 0:
        detail = (completed.stderr or "").strip().splitlines()
        raise GroundTruthError(
            f"gazebo ground truth query failed ({completed.returncode}): "
            f"{printable}: {detail[-1] if detail else 'no stderr'}"
        )
    return completed.stdout


def resolve_ground_truth_topic() -> str:
    """Confirm the expected Gazebo world pose topic is advertised, or fail closed."""
    listing = run_gz(["topic", "-l"], 15.0)
    candidates = sorted(
        {
            line.strip()
            for line in listing.splitlines()
            if GROUND_TRUTH_TOPIC_PATTERN.match(line.strip())
        }
    )
    if EXPECTED_GROUND_TRUTH_TOPIC not in candidates:
        raise GroundTruthError(
            "gazebo ground truth missing: expected world pose topic "
            f"{EXPECTED_GROUND_TRUTH_TOPIC!r} is not advertised; "
            f"discovered candidates: {candidates if candidates else 'none'}"
        )
    return EXPECTED_GROUND_TRUTH_TOPIC


def parse_pose_v(text: str) -> tuple[float, dict[str, tuple[float, float, float]]]:
    """Parse a gz.msgs.Pose_V debug-text message into a stamp and model poses."""
    stack: list[str] = []
    stamp_sec: int | None = None
    stamp_nsec: int | None = None
    poses: dict[str, tuple[float, float, float]] = {}
    current: dict[str, object] | None = None
    for raw_line in text.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.endswith("{"):
            block = line[:-1].strip().rstrip(":").strip()
            stack.append(block)
            if len(stack) == 1 and block == "pose":
                current = {"name": None, "x": None, "y": None, "z": None}
            continue
        if line == "}":
            if not stack:
                raise GroundTruthError("gazebo ground truth unparsable: unbalanced text")
            closed = stack.pop()
            if not stack and closed == "pose" and current is not None:
                name = current["name"]
                values = (current["x"], current["y"], current["z"])
                if isinstance(name, str) and all(
                    isinstance(value, float) for value in values
                ):
                    poses[name] = values  # type: ignore[assignment]
                current = None
            continue
        if ":" not in line:
            continue
        key, _, value = line.partition(":")
        key = key.strip()
        value = value.strip().strip('"')
        path = "/".join(stack)
        try:
            if path == "header/stamp" and key == "sec":
                stamp_sec = int(value)
            elif path == "header/stamp" and key == "nsec":
                stamp_nsec = int(value)
            elif path == "pose" and key == "name" and current is not None:
                current["name"] = value
            elif path == "pose/position" and current is not None and key in ("x", "y", "z"):
                current[key] = float(value)
        except ValueError as error:
            raise GroundTruthError(
                f"gazebo ground truth unparsable: {path}.{key}={value!r}"
            ) from error
    if stamp_sec is None or stamp_nsec is None:
        raise GroundTruthError("gazebo ground truth unparsable: missing header stamp")
    if not poses:
        raise GroundTruthError("gazebo ground truth unparsable: no model pose blocks")
    stamp = stamp_sec + stamp_nsec / 1e9
    if not math.isfinite(stamp):
        raise GroundTruthError("gazebo ground truth non-finite: header stamp")
    return stamp, poses


def sample_ground_truth(topic: str, attempts: int = 3) -> tuple[float, tuple[float, float, float]]:
    """Sample the Burger chassis pose straight from Gazebo world state."""
    last_error: GroundTruthError | None = None
    for _attempt in range(attempts):
        try:
            stamp, poses = parse_pose_v(run_gz(["topic", "-e", "-t", topic, "-n", "1"], 15.0))
        except GroundTruthError as error:
            last_error = error
            continue
        if GROUND_TRUTH_MODEL not in poses:
            last_error = GroundTruthError(
                f"gazebo ground truth missing: model {GROUND_TRUTH_MODEL!r} "
                f"absent from {topic}"
            )
            continue
        position = poses[GROUND_TRUTH_MODEL]
        if not all(math.isfinite(value) for value in position):
            raise GroundTruthError(
                f"gazebo ground truth non-finite: {GROUND_TRUTH_MODEL} position {position}"
            )
        return stamp, position
    raise last_error or GroundTruthError("gazebo ground truth missing")


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


def wait_for_command_graph(node: FidelityProbe, timeout: float) -> int:
    """Fail closed unless every /flyto/cmd_vel consumer is connected first.

    Sensor readiness alone does not imply the command graph is up: from a cold
    start the teleop/actuator chain can still be discovering, so the commanded
    motion is published into a partially connected graph and Gazebo never moves
    even though odometry integrates. Gate on the same invariant the fault probe
    already enforces before sampling the start pose or publishing motion.
    """
    deadline = time.monotonic() + timeout
    observed = node.publisher.get_subscription_count()
    while observed < COMMAND_GRAPH_MIN_SUBSCRIPTIONS and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        observed = node.publisher.get_subscription_count()
    if observed < COMMAND_GRAPH_MIN_SUBSCRIPTIONS:
        raise CommandGraphError(
            "command graph not ready: /flyto/cmd_vel had "
            f"{observed} subscriber(s), expected at least "
            f"{COMMAND_GRAPH_MIN_SUBSCRIPTIONS} within {timeout:.1f}s"
        )
    return observed


def settle_physics(node: FidelityProbe, topic: str) -> tuple[float, float]:
    """Fail closed until Gazebo physics has actually been stepping, and is stable.

    Command-graph readiness is necessary but not sufficient from a cold start:
    the graph can be fully connected while the Gazebo chassis has not moved at
    all, so a commanded pulse integrates in odometry while world displacement
    stays ~0m. Require a bounded amount of real *simulation* time to elapse on
    the world pose topic, and require the Burger chassis to stay inside a small
    stability window while it does. Returns (sim seconds elapsed, max drift m).
    """
    start_stamp, start_position = sample_ground_truth(topic)
    deadline = time.monotonic() + PHYSICS_SETTLE_TIMEOUT_S
    advance = 0.0
    max_drift = 0.0
    while time.monotonic() < deadline:
        spin_deadline = time.monotonic() + PHYSICS_SETTLE_SPIN_INTERVAL_S
        while time.monotonic() < spin_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        stamp, position = sample_ground_truth(topic)
        drift = math.hypot(
            position[0] - start_position[0], position[1] - start_position[1]
        )
        advance = stamp - start_stamp
        if not math.isfinite(drift) or not math.isfinite(advance):
            raise PhysicsSettleError(
                "gazebo physics settle non-finite: "
                f"advance={advance} drift={drift}"
            )
        max_drift = max(max_drift, drift)
        if max_drift > PHYSICS_SETTLE_MAX_DRIFT_M:
            raise PhysicsSettleError(
                "gazebo chassis is not stable before the commanded motion: "
                f"drifted {max_drift:.6f}m during settle, above the allowed "
                f"{PHYSICS_SETTLE_MAX_DRIFT_M}m"
            )
        if advance >= PHYSICS_SETTLE_MIN_SIM_S:
            return advance, max_drift
    raise PhysicsSettleError(
        "gazebo physics did not settle: simulation clock advanced "
        f"{advance:.6f}s of the required {PHYSICS_SETTLE_MIN_SIM_S}s within "
        f"{PHYSICS_SETTLE_TIMEOUT_S:.1f}s"
    )


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
    command_subscriptions = wait_for_command_graph(
        probe, COMMAND_GRAPH_READY_TIMEOUT_S
    )
    ground_truth_topic = resolve_ground_truth_topic()
    settle_sim_seconds, settle_max_drift = settle_physics(probe, ground_truth_topic)
    world_start_stamp, world_start = sample_ground_truth(ground_truth_topic)
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
    world_finish_stamp, world_finish = sample_ground_truth(ground_truth_topic)
    odometry_displacement = math.hypot(finish.x - start.x, finish.y - start.y)
    final_speed = abs(probe.odom[-1].twist.twist.linear.x)
    simulation_advance = world_finish_stamp - world_start_stamp
    if not math.isfinite(simulation_advance):
        raise GroundTruthError("gazebo ground truth non-finite: simulation clock")
    if simulation_advance < GROUND_TRUTH_MIN_SIM_ADVANCE_S:
        raise GroundTruthError(
            f"gazebo ground truth stale: simulation clock advanced "
            f"{simulation_advance:.6f}s over the commanded motion"
        )
    ground_truth_displacement = math.hypot(
        world_finish[0] - world_start[0], world_finish[1] - world_start[1]
    )
    if not math.isfinite(ground_truth_displacement):
        raise GroundTruthError("gazebo ground truth non-finite: displacement")
    if not (
        GROUND_TRUTH_MIN_DISPLACEMENT_M
        <= ground_truth_displacement
        <= GROUND_TRUTH_MAX_DISPLACEMENT_M
    ):
        raise GroundTruthError(
            "gazebo ground truth displacement outside the required physical window: "
            f"{ground_truth_displacement:.6f}m not in "
            f"[{GROUND_TRUTH_MIN_DISPLACEMENT_M}, {GROUND_TRUTH_MAX_DISPLACEMENT_M}]"
        )
    assert 0.025 <= odometry_displacement <= 0.16, odometry_displacement
    assert final_speed <= 0.03, final_speed
    report = {
        "passed": True,
        "contract_version": CONTRACT_VERSION,
        "model": "turtlebot3_burger",
        "evidence_source": EVIDENCE_SOURCE,
        "command_subscription_count": command_subscriptions,
        "gazebo_physics_settle_sim_seconds": settle_sim_seconds,
        "gazebo_physics_settle_max_drift_m": settle_max_drift,
        "gazebo_physics_settle_source": PHYSICS_SETTLE_SOURCE,
        "odometry_displacement_m": odometry_displacement,
        # Backward-compatible alias for consumers predating the explicit key.
        "displacement_m": odometry_displacement,
        "gazebo_ground_truth_displacement_m": ground_truth_displacement,
        "gazebo_ground_truth_source": GROUND_TRUTH_SOURCE,
        "gazebo_ground_truth_topic": ground_truth_topic,
        "gazebo_ground_truth_model": GROUND_TRUTH_MODEL,
        "gazebo_ground_truth_start_xy": [world_start[0], world_start[1]],
        "gazebo_ground_truth_finish_xy": [world_finish[0], world_finish[1]],
        "gazebo_simulation_time_advance_s": simulation_advance,
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
import math
import sys
from pathlib import Path

GROUND_TRUTH_MIN_DISPLACEMENT_M = 0.02
GROUND_TRUTH_MAX_DISPLACEMENT_M = 0.30
GROUND_TRUTH_MIN_SIM_ADVANCE_S = 0.1
ODOMETRY_MIN_DISPLACEMENT_M = 0.025
ODOMETRY_MAX_DISPLACEMENT_M = 0.16
EXPECTED_CONTRACT_VERSION = "flyto.robotics.burger-gazebo-acceptance.v1"
EXPECTED_EVIDENCE_SOURCE = "gazebo_harmonic_world_state+ros2_jazzy_topics"
EXPECTED_GROUND_TRUTH_SOURCE = "gz_transport_world_pose_info"
EXPECTED_GROUND_TRUTH_TOPIC = "/world/flyto_turtlebot3_fidelity/pose/info"
EXPECTED_GROUND_TRUTH_MODEL = "burger"
EXPECTED_MODEL = "turtlebot3_burger"
EXPECTED_MIN_COMMAND_SUBSCRIPTIONS = 2
PHYSICS_SETTLE_MIN_SIM_S = 10.0
PHYSICS_SETTLE_MAX_DRIFT_M = 0.01
EXPECTED_PHYSICS_SETTLE_SOURCE = "gz_transport_world_pose_info_sim_time"

root = Path(sys.argv[1])
normal = json.loads((root / "normal-probe.json").read_text(encoding="utf-8"))
fault = json.loads((root / "fault-probe.json").read_text(encoding="utf-8"))
for artifact_name, artifact in (
    ("normal-probe.json", normal),
    ("fault-probe.json", fault),
):
    if artifact.get("passed") is not True:
        raise SystemExit(
            f"{artifact_name} passed is {artifact.get('passed')!r}, expected True"
        )
for key in (
    "command_subscription_count",
    "contract_version",
    "displacement_m",
    "evidence_source",
    "gazebo_ground_truth_displacement_m",
    "gazebo_ground_truth_model",
    "gazebo_ground_truth_source",
    "gazebo_ground_truth_topic",
    "gazebo_physics_settle_max_drift_m",
    "gazebo_physics_settle_sim_seconds",
    "gazebo_physics_settle_source",
    "gazebo_simulation_time_advance_s",
    "model",
    "odometry_displacement_m",
):
    if key not in normal:
        raise SystemExit(f"normal-probe.json is missing required evidence key {key!r}")
for key, expected in (
    ("contract_version", EXPECTED_CONTRACT_VERSION),
    ("evidence_source", EXPECTED_EVIDENCE_SOURCE),
    ("gazebo_ground_truth_source", EXPECTED_GROUND_TRUTH_SOURCE),
    ("gazebo_ground_truth_topic", EXPECTED_GROUND_TRUTH_TOPIC),
    ("gazebo_ground_truth_model", EXPECTED_GROUND_TRUTH_MODEL),
    ("gazebo_physics_settle_source", EXPECTED_PHYSICS_SETTLE_SOURCE),
    ("model", EXPECTED_MODEL),
):
    if normal[key] != expected:
        raise SystemExit(
            f"normal-probe.json {key} is {normal[key]!r}, expected {expected!r}"
        )
command_subscriptions = normal["command_subscription_count"]
if isinstance(command_subscriptions, bool) or not isinstance(
    command_subscriptions, int
):
    raise SystemExit(
        "normal-probe.json command_subscription_count is "
        f"{command_subscriptions!r}, expected an integer"
    )
if command_subscriptions < EXPECTED_MIN_COMMAND_SUBSCRIPTIONS:
    raise SystemExit(
        f"normal-probe.json command_subscription_count is {command_subscriptions}, "
        f"below the required minimum of {EXPECTED_MIN_COMMAND_SUBSCRIPTIONS}; the "
        "commanded motion was published into a partially connected command graph"
    )
settle_seconds = normal["gazebo_physics_settle_sim_seconds"]
if isinstance(settle_seconds, bool) or not isinstance(
    settle_seconds, (int, float)
) or not math.isfinite(settle_seconds):
    raise SystemExit("gazebo physics settle duration is missing or non-finite")
if settle_seconds < PHYSICS_SETTLE_MIN_SIM_S:
    raise SystemExit(
        f"gazebo physics settled for only {settle_seconds}s of simulation time, "
        f"below the required minimum of {PHYSICS_SETTLE_MIN_SIM_S}s; a cold world "
        "can report odometry motion with no chassis motion"
    )
settle_drift = normal["gazebo_physics_settle_max_drift_m"]
if isinstance(settle_drift, bool) or not isinstance(
    settle_drift, (int, float)
) or not math.isfinite(settle_drift):
    raise SystemExit("gazebo physics settle drift is missing or non-finite")
if not (0.0 <= settle_drift <= PHYSICS_SETTLE_MAX_DRIFT_M):
    raise SystemExit(
        f"gazebo chassis drifted {settle_drift}m before the commanded motion, "
        f"outside the required stability window [0.0, {PHYSICS_SETTLE_MAX_DRIFT_M}]"
    )
simulation_advance = normal["gazebo_simulation_time_advance_s"]
if isinstance(simulation_advance, bool) or not isinstance(
    simulation_advance, (int, float)
) or not math.isfinite(simulation_advance):
    raise SystemExit("gazebo simulation time advance is missing or non-finite")
if simulation_advance < GROUND_TRUTH_MIN_SIM_ADVANCE_S:
    raise SystemExit(
        f"gazebo simulation clock advanced {simulation_advance}s, below the required "
        f"minimum of {GROUND_TRUTH_MIN_SIM_ADVANCE_S}s"
    )
odometry = normal["odometry_displacement_m"]
if isinstance(odometry, bool) or not isinstance(odometry, (int, float)) or not math.isfinite(
    odometry
):
    raise SystemExit("odometry displacement is missing or non-finite")
if not (ODOMETRY_MIN_DISPLACEMENT_M <= odometry <= ODOMETRY_MAX_DISPLACEMENT_M):
    raise SystemExit(
        f"odometry displacement {odometry} is outside the required window "
        f"[{ODOMETRY_MIN_DISPLACEMENT_M}, {ODOMETRY_MAX_DISPLACEMENT_M}]"
    )
if normal["displacement_m"] != normal["odometry_displacement_m"]:
    raise SystemExit(
        "normal-probe.json displacement_m alias does not match odometry_displacement_m"
    )
ground_truth = normal["gazebo_ground_truth_displacement_m"]
if isinstance(ground_truth, bool) or not isinstance(
    ground_truth, (int, float)
) or not math.isfinite(ground_truth):
    raise SystemExit("gazebo ground truth displacement is missing or non-finite")
if not (
    GROUND_TRUTH_MIN_DISPLACEMENT_M <= ground_truth <= GROUND_TRUTH_MAX_DISPLACEMENT_M
):
    raise SystemExit(
        f"gazebo ground truth displacement {ground_truth} is outside the required "
        f"physical window "
        f"[{GROUND_TRUTH_MIN_DISPLACEMENT_M}, {GROUND_TRUTH_MAX_DISPLACEMENT_M}]"
    )
report = {
    "passed": True,
    "contract_version": normal["contract_version"],
    "evidence_source": normal["evidence_source"],
    "robot": normal,
    "fault_safety": fault,
}
(root / "report.json").write_text(
    json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
)
print(json.dumps(report, sort_keys=True))
PY

# The normal, non-fault, gateway runtime is restored by the EXIT handler above,
# so it happens exactly once here and on every failure path.
echo "Virtual robot verification: ${result_directory}/report.json"
