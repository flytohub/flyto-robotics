"""Independent latched emergency-stop supervisor for ROS 2 deployments."""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from typing import Any


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
            if not math.isfinite(value) or not 0.05 <= value <= 5.0:
                raise ValueError(f"{label} is outside its safe range")
        self.sensor_timeout_seconds = float(sensor_timeout_seconds)
        self.command_timeout_seconds = float(command_timeout_seconds)
        self.clock = clock
        self.goal_active = False
        self.goal_started_at: float | None = None
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
        self.receipts[source] = self.clock()

    def update_goal_active(self, active: bool) -> None:
        active = bool(active)
        if active and not self.goal_active:
            self.goal_started_at = self.clock()
        if not active:
            self.goal_started_at = None
        self.goal_active = active

    def latch(self, reason: str) -> str:
        if self.latched_reason is None:
            self.latched_reason = reason
        return self.latched_reason

    def reset(self) -> None:
        self.latched_reason = None
        self.goal_active = False
        self.goal_started_at = None

    def evaluate(self) -> str | None:
        if self.latched or not self.goal_active:
            return self.latched_reason
        now = self.clock()
        for source, timeout, reason in (
            ("odometry", self.sensor_timeout_seconds, "odometry_stale"),
            ("lidar", self.sensor_timeout_seconds, "lidar_stale"),
            ("command", self.command_timeout_seconds, "command_stale"),
        ):
            receipt = self.receipts[source]
            if receipt is None:
                receipt = self.goal_started_at
            if receipt is None or now - receipt > timeout:
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
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.node.declare_parameter(
            "execution_state_topic", "/flyto/navigation_execution_active"
        )
        self.node.declare_parameter("fault_state_topic", "/fault_injection/state")
        self.node.declare_parameter("sensor_timeout_seconds", 0.40)
        self.node.declare_parameter("command_timeout_seconds", 0.30)
        self.watchdog = SafetyWatchdog(
            sensor_timeout_seconds=float(
                self.node.get_parameter("sensor_timeout_seconds").value
            ),
            command_timeout_seconds=float(
                self.node.get_parameter("command_timeout_seconds").value
            ),
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
            20,
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
        self.watchdog.observe("command")
        if self.watchdog.latched:
            self.command_publisher.publish(self._twist_type())
            return
        self.command_publisher.publish(command)

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
        while not self._watchdog_stop.wait(0.05):
            self._hold_stop()

    def shutdown(self) -> None:
        self._watchdog_stop.set()
        self._watchdog_thread.join(timeout=1.0)

    def _observe_execution_state(self, message: Any) -> None:
        was_active = self.watchdog.goal_active
        self.watchdog.update_goal_active(message.data)
        if was_active != self.watchdog.goal_active:
            self.node.get_logger().info(
                f"navigation execution active: {self.watchdog.goal_active}"
            )

    def _observe_fault_state(self, message: Any) -> None:
        if message.data.endswith(":active"):
            self.watchdog.update_goal_active(True)
            self.node.get_logger().warning(
                f"watchdog armed by active fault: {message.data}"
            )

    def _latch(self, reason: str, *, was_latched: bool | None = None) -> None:
        if was_latched is None:
            was_latched = self.watchdog.latched
        latched_reason = self.watchdog.latch(reason)
        if not was_latched:
            self.node.get_logger().error(
                f"emergency stop latched: {latched_reason}"
            )
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
