"""Independent latched emergency-stop supervisor for ROS 2 deployments."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any

WATCHDOG_POLL_SECONDS = 0.01
FAULT_WATCHDOG_SOURCES = {
    "lidar_dropout:active": "lidar",
    "odometry_freeze:active": "odometry",
    "nav2_lifecycle_failure:active": "command",
}


def watchdog_source_for_fault_state(state: str) -> str | None:
    """Return the receipt that a controlled fault must restart from activation."""

    return FAULT_WATCHDOG_SOURCES.get(state)


class VelocitySafetyEnvelope:
    """Clamp forwarded motion while keeping the latched stop unconditional."""

    def __init__(
        self,
        *,
        max_abs_linear_speed_mps: float,
        max_abs_angular_speed_rps: float,
    ) -> None:
        for value, label, minimum, maximum in (
            (max_abs_linear_speed_mps, "linear speed limit", 0.05, 1.0),
            (max_abs_angular_speed_rps, "angular speed limit", 0.1, 4.0),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not minimum <= float(value) <= maximum
            ):
                raise ValueError(f"{label} is outside its safe range")
        self.max_abs_linear_speed_mps = float(max_abs_linear_speed_mps)
        self.max_abs_angular_speed_rps = float(max_abs_angular_speed_rps)

    def gate(
        self,
        linear_x: float,
        angular_z: float,
        *,
        latched: bool,
    ) -> tuple[float, float]:
        if latched:
            return 0.0, 0.0
        for value, label in ((linear_x, "linear speed"), (angular_z, "angular speed")):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise ValueError(f"{label} is invalid")
        bounded_linear = max(
            -self.max_abs_linear_speed_mps,
            min(self.max_abs_linear_speed_mps, float(linear_x)),
        )
        bounded_angular = max(
            -self.max_abs_angular_speed_rps,
            min(self.max_abs_angular_speed_rps, float(angular_z)),
        )
        return bounded_linear, bounded_angular


class SafetyWatchdog:
    """Track action-time input freshness without depending on Nav2 internals."""

    def __init__(
        self,
        *,
        sensor_timeout_seconds: float,
        command_timeout_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        for value, label in (
            (sensor_timeout_seconds, "sensor timeout"),
            (command_timeout_seconds, "command timeout"),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or not 0.05 <= float(value) <= 5.0
            ):
                raise ValueError(f"{label} is outside its safe range")
        self.sensor_timeout_seconds = float(sensor_timeout_seconds)
        self.command_timeout_seconds = float(command_timeout_seconds)
        self.clock = clock
        self.goal_active = False
        self.goal_started_at: float | None = None
        self.command_observed_during_goal = False
        self.command_motion_active = False
        self.command_watchdog_forced = False
        self.latched_reason: str | None = None
        self.receipts: dict[str, float | None] = {
            "odometry": None,
            "lidar": None,
            "command": None,
        }

    @property
    def latched(self) -> bool:
        return self.latched_reason is not None

    def observe(self, source: str) -> None:
        if source not in self.receipts:
            raise ValueError("watchdog source is unsupported")
        observed_at = self.clock()
        if (
            isinstance(observed_at, bool)
            or not isinstance(observed_at, (int, float))
            or not math.isfinite(float(observed_at))
        ):
            raise ValueError("watchdog observation timestamp is invalid")
        self.receipts[source] = float(observed_at)
        if source == "command" and self.goal_active:
            self.command_observed_during_goal = True

    def observe_command(self, linear_x: float, angular_z: float) -> None:
        """Record freshness and whether the last forwarded command can move."""

        for value in (linear_x, angular_z):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(float(value))
            ):
                raise ValueError("command velocity is invalid")
        self.observe("command")
        self.command_motion_active = bool(
            abs(float(linear_x)) > 1e-6 or abs(float(angular_z)) > 1e-6
        )

    def force_command_watchdog(self) -> None:
        """Arm command loss for the controlled Nav2 lifecycle fault proof."""

        self.command_watchdog_forced = True
        self.observe("command")

    def update_goal_active(self, active: bool) -> None:
        active = bool(active)
        if active and not self.goal_active:
            started_at = self.clock()
            if (
                isinstance(started_at, bool)
                or not isinstance(started_at, (int, float))
                or not math.isfinite(float(started_at))
            ):
                raise ValueError("watchdog goal timestamp is invalid")
            self.goal_started_at = float(started_at)
            for source in ("odometry", "lidar"):
                self.receipts[source] = self.goal_started_at
            self.command_observed_during_goal = False
            self.command_motion_active = False
            self.command_watchdog_forced = False
        if not active:
            self.goal_started_at = None
            self.command_observed_during_goal = False
            self.command_motion_active = False
            self.command_watchdog_forced = False
        self.goal_active = active

    def latch(self, reason: str) -> str:
        if self.latched_reason is None:
            self.latched_reason = reason
        return self.latched_reason

    def reset(self) -> None:
        self.latched_reason = None
        self.goal_active = False
        self.goal_started_at = None
        self.command_observed_during_goal = False
        self.command_motion_active = False
        self.command_watchdog_forced = False

    def evaluate(self) -> str | None:
        if self.latched or not self.goal_active:
            return self.latched_reason
        now = self.clock()
        if (
            isinstance(now, bool)
            or not isinstance(now, (int, float))
            or not math.isfinite(float(now))
        ):
            return self.latch("watchdog_clock_invalid")
        now = float(now)
        for source, timeout, reason in (
            ("odometry", self.sensor_timeout_seconds, "odometry_stale"),
            ("lidar", self.sensor_timeout_seconds, "lidar_stale"),
            ("command", self.command_timeout_seconds, "command_stale"),
        ):
            if source == "command" and not self.command_observed_during_goal:
                continue
            receipt = self.receipts[source]
            if receipt is None:
                receipt = self.goal_started_at
            if (
                receipt is None
                or not math.isfinite(receipt)
                or receipt > now
                or now - receipt > timeout
            ):
                return self.latch(reason)
        return None

    def evaluate_transition(self) -> tuple[str | None, bool]:
        """Return the reason and whether this evaluation created the latch."""
        was_latched = self.latched
        reason = self.evaluate()
        return reason, reason is not None and not was_latched


class EmergencyStopSupervisor:
    """Own the only actuator command output and hold zero while latched."""

    def __init__(self, node: Any) -> None:
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
            qos_profile_sensor_data,
        )
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import Bool, String
        from std_srvs.srv import Trigger

        self.node = node
        self._twist_type = Twist
        self._bool_type = Bool
        self._string_type = String
        self.node.declare_parameter("cmd_vel_input_topic", "/nav2/cmd_vel")
        self.node.declare_parameter("cmd_vel_output_topic", "/cmd_vel")
        self.node.declare_parameter("state_topic", "/safety/emergency_stop_state")
        self.node.declare_parameter("reason_topic", "/safety/stop_reason")
        self.node.declare_parameter("odometry_topic", "/flyto/odom")
        self.node.declare_parameter("lidar_topic", "/flyto/scan")
        self.node.declare_parameter("execution_state_topic", "/flyto/navigation_execution_active")
        self.node.declare_parameter("fault_state_topic", "/fault_injection/state")
        self.node.declare_parameter("sensor_timeout_seconds", 0.40)
        self.node.declare_parameter("command_timeout_seconds", 0.30)
        self.node.declare_parameter("max_abs_linear_speed_mps", 0.10)
        self.node.declare_parameter("max_abs_angular_speed_rps", 0.50)
        self.velocity_envelope = VelocitySafetyEnvelope(
            max_abs_linear_speed_mps=self.node.get_parameter("max_abs_linear_speed_mps").value,
            max_abs_angular_speed_rps=self.node.get_parameter(
                "max_abs_angular_speed_rps"
            ).value,
        )
        self.watchdog = SafetyWatchdog(
            sensor_timeout_seconds=float(self.node.get_parameter("sensor_timeout_seconds").value),
            command_timeout_seconds=float(self.node.get_parameter("command_timeout_seconds").value),
        )
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.command_publisher = self.node.create_publisher(
            Twist,
            str(self.node.get_parameter("cmd_vel_output_topic").value),
            10,
        )
        self.command_subscription = self.node.create_subscription(
            Twist,
            str(self.node.get_parameter("cmd_vel_input_topic").value),
            self._gate_command,
            10,
        )
        self.state_publisher = self.node.create_publisher(
            Bool,
            str(self.node.get_parameter("state_topic").value),
            state_qos,
        )
        self.reason_publisher = self.node.create_publisher(
            String,
            str(self.node.get_parameter("reason_topic").value),
            state_qos,
        )
        self.odometry_subscription = self.node.create_subscription(
            Odometry,
            str(self.node.get_parameter("odometry_topic").value),
            lambda _message: self.watchdog.observe("odometry"),
            20,
        )
        self.lidar_subscription = self.node.create_subscription(
            LaserScan,
            str(self.node.get_parameter("lidar_topic").value),
            lambda _message: self.watchdog.observe("lidar"),
            qos_profile_sensor_data,
        )
        self.execution_subscription = self.node.create_subscription(
            Bool,
            str(self.node.get_parameter("execution_state_topic").value),
            self._observe_execution_state,
            state_qos,
        )
        self.fault_subscription = self.node.create_subscription(
            String,
            str(self.node.get_parameter("fault_state_topic").value),
            self._observe_fault_state,
            state_qos,
        )
        self.stop_service = self.node.create_service(
            Trigger,
            "emergency_stop",
            self._stop,
        )
        self.reset_service = self.node.create_service(
            Trigger,
            "reset",
            self._reset,
        )
        self._watchdog_stop = threading.Event()
        self._watchdog_thread = threading.Thread(
            target=self._run_watchdog,
            name="flyto-safety-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()
        self._publish_state()

    def _gate_command(self, command: Any) -> None:
        linear_x, angular_z = self.velocity_envelope.gate(
            command.linear.x,
            command.angular.z,
            latched=self.watchdog.latched,
        )
        self.watchdog.observe_command(linear_x, angular_z)
        safe_command = self._twist_type()
        safe_command.linear.x = linear_x
        safe_command.angular.z = angular_z
        self.command_publisher.publish(safe_command)

    def _stop(self, _request: Any, response: Any) -> Any:
        self._latch("emergency_stop")
        response.success = True
        response.message = "emergency stop latched"
        return response

    def _reset(self, _request: Any, response: Any) -> Any:
        self.watchdog.reset()
        self._publish_state()
        response.success = True
        response.message = "emergency stop reset"
        return response

    def _hold_stop(self) -> None:
        reason, newly_latched = self.watchdog.evaluate_transition()
        if reason is not None:
            self._latch(reason, was_latched=not newly_latched)

    def _run_watchdog(self) -> None:
        self.node.get_logger().info("independent safety watchdog started")
        while not self._watchdog_stop.wait(WATCHDOG_POLL_SECONDS):
            self._hold_stop()

    def shutdown(self) -> None:
        self._watchdog_stop.set()
        self._watchdog_thread.join(timeout=1.0)

    def _observe_execution_state(self, message: Any) -> None:
        was_active = self.watchdog.goal_active
        self.watchdog.update_goal_active(message.data)
        if was_active != self.watchdog.goal_active:
            self.node.get_logger().info(f"navigation execution active: {self.watchdog.goal_active}")

    def _observe_fault_state(self, message: Any) -> None:
        if message.data.endswith(":active"):
            self.watchdog.update_goal_active(True)
            source = watchdog_source_for_fault_state(message.data)
            if source == "command":
                self.watchdog.force_command_watchdog()
            elif source is not None:
                # Anchor the full watchdog window at the independently observed
                # injection boundary. Any subsequent real message advances the
                # receipt, so the stop still requires the source to remain stale.
                self.watchdog.observe(source)
            self.node.get_logger().warning(f"watchdog armed by active fault: {message.data}")

    def _latch(self, reason: str, *, was_latched: bool | None = None) -> None:
        if was_latched is None:
            was_latched = self.watchdog.latched
        latched_reason = self.watchdog.latch(reason)
        if not was_latched:
            self.node.get_logger().error(f"emergency stop latched: {latched_reason}")
            self._publish_reason(latched_reason)
            self._publish_state()
        if self.watchdog.latched:
            self.command_publisher.publish(self._twist_type())

    def _publish_state(self) -> None:
        message = self._bool_type()
        message.data = self.watchdog.latched
        self.state_publisher.publish(message)

    def _publish_reason(self, reason: str | None = None) -> None:
        message = self._string_type()
        message.data = reason or "reset"
        self.reason_publisher.publish(message)


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("emergency_supervisor")
    supervisor = EmergencyStopSupervisor(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        supervisor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
