"""ROS 2 execution backend for the AI Space delivery gateway."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import TYPE_CHECKING, Any

import rclpy
from nav_msgs.msg import Odometry
from rclpy import logging as rclpy_logging
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan

from .contracts import write_json_atomic
from .mission import closest_range, sector_field, Pose2D, evaluate_sensor_gate
from .ros2_cmd_vel import (
    CMD_VEL_TYPE_AUTO,
    CMD_VEL_TYPES,
    CmdVelChannel,
    validated_topic,
)

if TYPE_CHECKING:
    from .delivery_gateway import DeliveryGateway, DeliverySession

CONTROL_PERIOD_SECONDS = 0.1
ODOMETRY_TIMEOUT_SECONDS = 1.0
SENSOR_STARTUP_GRACE_SECONDS = 10.0
SENSOR_STABILIZATION_SECONDS = 1.0

# The bundled Gazebo bridge publishes under /flyto; a vendor driver such as
# TurtleBot3 keeps the unnamespaced names. Configure the runner instead of
# remapping the vendor driver, so teleop, rviz and Nav2 keep working alongside.
DEFAULT_ODOM_TOPIC = "/flyto/odom"
DEFAULT_SCAN_TOPIC = "/flyto/scan"
DEFAULT_CMD_VEL_TOPIC = "/flyto/cmd_vel"

class Ros2DeliverySessionNode(Node):
    """Fail-safe per-session ROS wrapper mirroring the mission adapter.

    Delivery workflows never contain follow_line, so no camera subscription
    is created; the sensor gate covers odometry and lidar freshness only.
    """

    def __init__(
        self,
        *,
        session: DeliverySession,
        gateway_lock: threading.RLock,
        results_dir: Path,
        gazebo_physics: bool,
        on_terminal: Any,
        odom_topic: str = DEFAULT_ODOM_TOPIC,
        scan_topic: str = DEFAULT_SCAN_TOPIC,
        cmd_vel_topic: str = DEFAULT_CMD_VEL_TOPIC,
        cmd_vel_type: str = CMD_VEL_TYPE_AUTO,
        require_range: bool = True,
    ) -> None:
        parameter_overrides = []
        if gazebo_physics:
            # Gazebo publishes /clock; mission and step timeouts must follow
            # simulation time or they fire at the wrong physical progress.
            parameter_overrides.append(
                Parameter("use_sim_time", Parameter.Type.BOOL, True)
            )
        super().__init__(
            f"flyto_robotics_delivery_{session.session_id.replace('-', '_')}",
            parameter_overrides=parameter_overrides,
        )
        self._session = session
        self._lock = gateway_lock
        self._results_dir = results_dir
        self._gazebo_physics = gazebo_physics
        self._on_terminal = on_terminal
        self._started_at_steady = time.monotonic()
        self._last_pose: Pose2D | None = None
        self._last_odometry_at: float | None = None
        self._last_scan_at: float | None = None
        self._sensors_ready_since: float | None = None
        self._control_started = False
        self._clock_anchored = False
        self._minimum_range = math.inf
        self._range_field = None
        self._finished = False
        self._requires_range = require_range
        self._scan_snapshot: dict[str, Any] | None = None

        # Jazzy's TurtleBot3 driver subscribes with TwistStamped while the
        # bundled Gazebo bridge uses Twist. A type mismatch matches zero
        # subscribers and DDS reports no error, so the robot silently ignores
        # every command. Bind the publisher lazily to whatever the driver
        # actually subscribes with instead of hardcoding either type.
        self._cmd_vel = CmdVelChannel(
            self, topic=cmd_vel_topic, cmd_vel_type=cmd_vel_type
        )
        self.create_subscription(Odometry, odom_topic, self._on_odometry, 10)
        self.create_subscription(
            LaserScan,
            scan_topic,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"topics: cmd_vel={cmd_vel_topic} odom={odom_topic} scan={scan_topic}"
        )
        self._timer = self.create_timer(CONTROL_PERIOD_SECONDS, self._control_tick)
        self.get_logger().info(
            f"delivery session {session.session_id} accepted for "
            f"robot {session.controller.job.robot_id}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        sin_yaw = 2.0 * (
            orientation.w * orientation.z + orientation.x * orientation.y
        )
        cos_yaw = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
        self._last_pose = Pose2D(position.x, position.y, math.atan2(sin_yaw, cos_yaw))
        self._last_odometry_at = time.monotonic()

    def scan_snapshot(self) -> dict[str, Any] | None:
        """Latest range scan, or None when no sensor has reported."""
        return self._scan_snapshot

    def _on_scan(self, message: LaserScan) -> None:
        # Sectors, not one number: a global minimum cannot tell the wall the
        # robot drives alongside from the one it drives into.
        self._range_field = sector_field(
            message.ranges,
            angle_min=message.angle_min,
            angle_increment=message.angle_increment,
            range_min=message.range_min,
            range_max=message.range_max,
        )
        self._minimum_range = self._range_field.closest
        self._last_scan_at = time.monotonic()
        self._scan_snapshot = {
            "ranges": [
                round(value, 3)
                if math.isfinite(value) and message.range_min <= value <= message.range_max
                else None
                for value in message.ranges
            ],
            "angle_min": round(message.angle_min, 5),
            "angle_step": round(message.angle_increment, 6),
            "range_max": round(message.range_max, 2),
        }

    def _send_velocity(self, linear_x: float, angular_z: float) -> None:
        self._cmd_vel.send(linear_x, angular_z)

    def _publish_stop(self) -> None:
        self._cmd_vel.stop()

    def finish_locked(self, now: float) -> dict[str, Any] | None:
        """Stop, snapshot evidence, and retire; caller holds the gateway lock.

        Returns the result payload to persist with write_evidence outside the
        lock, or None when the session already finished.
        """
        if self._finished:
            return None
        self._finished = True
        self._publish_stop()
        self._timer.cancel()
        result = self._session.controller.result(
            generated_at=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            now=now,
            pose=self._last_pose,
        )
        result["simulation"] = {
            "mode": "gazebo_ros2" if self._gazebo_physics else "physical_ros2",
            "gazebo_physics": self._gazebo_physics,
        }
        self.get_logger().info(
            f"delivery session {self._session.session_id} finished with "
            f"state {self._session.controller.state.value}"
        )
        self._on_terminal(self)
        return result

    def write_evidence(self, result: dict[str, Any]) -> None:
        """Persist one result envelope; fsync stays outside the gateway lock."""
        try:
            write_json_atomic(
                self._results_dir / f"delivery-{self._session.session_id}.json",
                result,
            )
        except OSError as exc:
            self.get_logger().warning(f"delivery evidence not written: {exc}")

    def _control_tick(self) -> None:
        steady_now = time.monotonic()
        # Odometry is always required: without a trusted pose the controller
        # cannot know where it is. A range sensor is only required when the
        # workflow actually uses clearance, so a delivery on a robot with no
        # lidar still runs, it simply forfeits obstacle waiting.
        required_samples: list[float | None] = [self._last_odometry_at]
        if self._requires_range:
            required_samples.append(self._last_scan_at)
        samples_present = self._last_pose is not None and all(
            sample is not None for sample in required_samples
        )
        oldest_sample_age = max(
            (
                steady_now - sample_time
                for sample_time in required_samples
                if sample_time is not None
            ),
            default=math.inf,
        )
        if not samples_present or oldest_sample_age > ODOMETRY_TIMEOUT_SECONDS:
            self._sensors_ready_since = None
        elif self._sensors_ready_since is None:
            self._sensors_ready_since = steady_now
        ready_duration = (
            steady_now - self._sensors_ready_since
            if self._sensors_ready_since is not None
            else 0.0
        )
        finished_result: dict[str, Any] | None = None
        with self._lock:
            controller = self._session.controller
            now = self._now()
            if not self._clock_anchored:
                # The controller was constructed before the ROS clock was
                # known; anchor mission time before any terminal handling so
                # evidence never reports epoch-scale elapsed seconds.
                controller.started_at = now
                controller.state_entered_at = now
                self._session.sim_now = now
                self._clock_anchored = True
            if controller.terminal:
                finished_result = self.finish_locked(now)
            else:
                sensor_decision = evaluate_sensor_gate(
                    samples_present=samples_present,
                    oldest_sample_age=oldest_sample_age,
                    ready_duration=ready_duration,
                    startup_elapsed=steady_now - self._started_at_steady,
                    startup_grace=SENSOR_STARTUP_GRACE_SECONDS,
                    freshness_timeout=ODOMETRY_TIMEOUT_SECONDS,
                    stabilization_seconds=SENSOR_STABILIZATION_SECONDS,
                    control_started=self._control_started,
                )
                if sensor_decision == "wait":
                    self._publish_stop()
                elif sensor_decision != "ready":
                    reason = (
                        "required_sensor_stale"
                        if sensor_decision == "fail_stale"
                        else "required_sensor_not_ready"
                    )
                    controller.fail(reason, now)
                    finished_result = self.finish_locked(now)
                else:
                    self._control_started = True
                    command = controller.tick(
                        self._last_pose,
                        minimum_range=self._range_field or self._minimum_range,
                        now=now,
                    )
                    self._session.pose = self._last_pose
                    # The session exposes minimum_range, and nothing was ever
                    # writing it on this backend: the closest lidar return was
                    # computed, handed to the controller, and then dropped, so
                    # every ROS-backed mission reported minimum_range: null no
                    # matter what it drove past. Recorded here, beside pose,
                    # because it is the same kind of fact — what the robot
                    # observed while it ran — and it is what answers "was that
                    # passage actually clear" for anything reading the session.
                    self._session.minimum_range = closest_range(
                        self._range_field, self._minimum_range
                    )
                    self._session.sim_now = now
                    if controller.terminal:
                        finished_result = self.finish_locked(now)
                    else:
                        # Publish while holding the lock so a shutdown stop
                        # can never be overtaken by an in-flight command.
                        self._send_velocity(command.linear_x, command.angular_z)
        if finished_result is not None:
            self.write_evidence(finished_result)


class Ros2DeliveryRunner:
    """Delivery gateway execution backend driving ROS 2 robots."""

    mode = "ros2"

    def __init__(
        self,
        *,
        results_dir: Path | str = "results",
        gazebo_physics: bool = False,
        odom_topic: str = DEFAULT_ODOM_TOPIC,
        scan_topic: str = DEFAULT_SCAN_TOPIC,
        cmd_vel_topic: str = DEFAULT_CMD_VEL_TOPIC,
        cmd_vel_type: str = CMD_VEL_TYPE_AUTO,
        require_range: bool = True,
    ) -> None:
        self._results_dir = Path(results_dir)
        self._gazebo_physics = bool(gazebo_physics)
        self._odom_topic = validated_topic(odom_topic, "odom_topic")
        self._scan_topic = validated_topic(scan_topic, "scan_topic")
        self._cmd_vel_topic = validated_topic(cmd_vel_topic, "cmd_vel_topic")
        if cmd_vel_type not in CMD_VEL_TYPES:
            raise ValueError(f"cmd_vel_type must be one of {', '.join(CMD_VEL_TYPES)}")
        self._cmd_vel_type = cmd_vel_type
        self._require_range = bool(require_range)
        self._gateway: DeliveryGateway | None = None
        self._executor: SingleThreadedExecutor | None = None
        self._spin_thread: threading.Thread | None = None
        self._nodes_lock = threading.Lock()
        self._nodes: list[Ros2DeliverySessionNode] = []
        self._retired: list[Ros2DeliverySessionNode] = []
        self._shutdown = False

    def bind(self, gateway: DeliveryGateway) -> None:
        self._gateway = gateway
        if not rclpy.ok():
            # Keep the context alive through KeyboardInterrupt so the final
            # zero-velocity stop can still be published during teardown.
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self._executor = SingleThreadedExecutor()
        self._spin_thread = threading.Thread(
            target=self._spin,
            name="flyto-robotics-delivery-ros2-executor",
            daemon=True,
        )
        self._spin_thread.start()

    def _spin(self) -> None:
        logger = rclpy_logging.get_logger("flyto_robotics.ros2_delivery_runner")
        while not self._shutdown and rclpy.ok():
            executor = self._executor
            if executor is None:
                return
            self._destroy_retired()
            try:
                executor.spin_once(timeout_sec=0.1)
            except ExternalShutdownException:
                return
            except Exception as exc:  # noqa: BLE001 - executor must survive
                logger.error(f"delivery executor callback failed: {exc}")

    def _retire_node(self, node: Ros2DeliverySessionNode) -> None:
        with self._nodes_lock:
            if node in self._nodes:
                self._nodes.remove(node)
            if node not in self._retired:
                self._retired.append(node)

    def _destroy_retired(self) -> None:
        with self._nodes_lock:
            retired, self._retired = self._retired, []
        for node in retired:
            executor = self._executor
            if executor is not None:
                executor.remove_node(node)
            node.destroy_node()

    def start_session(self, session: DeliverySession) -> None:
        gateway = self._gateway
        executor = self._executor
        if gateway is None or executor is None:
            raise RuntimeError("ros2 delivery runner is not bound to a gateway")
        node = Ros2DeliverySessionNode(
            session=session,
            gateway_lock=gateway.lock,
            results_dir=self._results_dir,
            gazebo_physics=self._gazebo_physics,
            on_terminal=self._retire_node,
            odom_topic=self._odom_topic,
            scan_topic=self._scan_topic,
            cmd_vel_topic=self._cmd_vel_topic,
            cmd_vel_type=self._cmd_vel_type,
            require_range=self._require_range,
        )
        with self._nodes_lock:
            self._nodes.append(node)
        executor.add_node(node)

    def shutdown(self) -> None:
        # Order matters for safety: stop the executor first so no in-flight
        # tick can publish a motion command after the final stop below.
        self._shutdown = True
        if self._spin_thread is not None:
            self._spin_thread.join(timeout=2.0)
            self._spin_thread = None
        gateway = self._gateway
        with self._nodes_lock:
            active = list(self._nodes)
        for node in active:
            finished_result = None
            if gateway is not None:
                with gateway.lock:
                    controller = node._session.controller
                    if not controller.terminal:
                        controller.cancel_for_safety(
                            node._session.sim_now, reason="gateway_shutdown"
                        )
                    finished_result = node.finish_locked(node._session.sim_now)
            else:
                node._publish_stop()
                self._retire_node(node)
            if finished_result is not None:
                node.write_evidence(finished_result)
        self._destroy_retired()
        if self._executor is not None:
            self._executor.shutdown()
            self._executor = None
        if rclpy.ok():
            rclpy.shutdown()

