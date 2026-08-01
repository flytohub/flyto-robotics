"""Fail-closed ROS 2 sensor relay with deterministic fault injection."""

from __future__ import annotations

import math
import time
from collections.abc import Callable
from typing import Any

FAULT_SCENARIOS = {
    "none",
    "lidar_dropout",
    "odometry_freeze",
    "nav2_lifecycle_failure",
}


class FaultInjectionController:
    """Arm one bounded fault only after the rover receives a motion command."""

    def __init__(
        self,
        scenario: str,
        *,
        delay_seconds: float,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if scenario not in FAULT_SCENARIOS:
            raise ValueError("fault scenario is unsupported")
        if not math.isfinite(delay_seconds) or not 0.05 <= delay_seconds <= 30.0:
            raise ValueError("fault delay is outside its safe range")
        self.scenario = scenario
        self.delay_seconds = float(delay_seconds)
        self.clock = clock
        self.motion_started_at: float | None = None
        self.activated_at: float | None = None

    @property
    def active(self) -> bool:
        return self.activated_at is not None

    def observe_command(self, linear_x: float, angular_z: float) -> None:
        if self.motion_started_at is not None:
            return
        if abs(float(linear_x)) >= 0.01 or abs(float(angular_z)) >= 0.01:
            self.motion_started_at = self.clock()

    def activation_due(self) -> bool:
        return (
            self.scenario != "none"
            and self.motion_started_at is not None
            and not self.active
            and self.clock() - self.motion_started_at >= self.delay_seconds
        )

    def activate(self) -> None:
        if not self.activation_due():
            raise RuntimeError("fault activation is not due")
        self.activated_at = self.clock()

    def should_forward(self, sensor: str) -> bool:
        if sensor not in {"odometry", "lidar"}:
            raise ValueError("sensor is unsupported")
        if not self.active:
            return True
        return not (
            (self.scenario == "odometry_freeze" and sensor == "odometry")
            or (self.scenario == "lidar_dropout" and sensor == "lidar")
        )


class Ros2SensorGuard:
    """Relay raw hardware topics and inject only an explicitly selected lab fault."""

    def __init__(self, node: Any) -> None:
        from geometry_msgs.msg import Twist
        from lifecycle_msgs.msg import Transition
        from lifecycle_msgs.srv import ChangeState
        from nav_msgs.msg import Odometry
        from rclpy.qos import (
            DurabilityPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import LaserScan
        from std_msgs.msg import String

        self.node = node
        for name, default in (
            ("fault_scenario", "none"),
            ("raw_odometry_topic", "/flyto/raw_odom"),
            ("odometry_topic", "/flyto/odom"),
            ("raw_lidar_topic", "/flyto/raw_scan"),
            ("lidar_topic", "/flyto/scan"),
            ("command_topic", "/nav2/cmd_vel"),
            ("fault_state_topic", "/fault_injection/state"),
            ("lifecycle_service", "/controller_server/change_state"),
        ):
            self.node.declare_parameter(name, default)
        self.node.declare_parameter("fault_delay_seconds", 0.35)
        scenario = str(self.node.get_parameter("fault_scenario").value)
        self.controller = FaultInjectionController(
            scenario,
            delay_seconds=float(
                self.node.get_parameter("fault_delay_seconds").value
            ),
        )
        self._transition_type = Transition
        self._change_state_type = ChangeState
        self._lifecycle_requested = False
        state_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        relay_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.odometry_publisher = self.node.create_publisher(
            Odometry,
            str(self.node.get_parameter("odometry_topic").value),
            20,
        )
        self.lidar_publisher = self.node.create_publisher(
            LaserScan,
            str(self.node.get_parameter("lidar_topic").value),
            relay_qos,
        )
        self.state_publisher = self.node.create_publisher(
            String,
            str(self.node.get_parameter("fault_state_topic").value),
            state_qos,
        )
        self.odometry_subscription = self.node.create_subscription(
            Odometry,
            str(self.node.get_parameter("raw_odometry_topic").value),
            self._relay_odometry,
            20,
        )
        self.lidar_subscription = self.node.create_subscription(
            LaserScan,
            str(self.node.get_parameter("raw_lidar_topic").value),
            self._relay_lidar,
            relay_qos,
        )
        self.command_subscription = self.node.create_subscription(
            Twist,
            str(self.node.get_parameter("command_topic").value),
            self._observe_command,
            20,
        )
        self.lifecycle_client = self.node.create_client(
            ChangeState,
            str(self.node.get_parameter("lifecycle_service").value),
        )
        self.timer = self.node.create_timer(0.02, self._tick)
        self._publish_state("ready")

    def _observe_command(self, message: Any) -> None:
        self.controller.observe_command(message.linear.x, message.angular.z)

    def _relay_odometry(self, message: Any) -> None:
        self._tick()
        if self.controller.should_forward("odometry"):
            self.odometry_publisher.publish(message)

    def _relay_lidar(self, message: Any) -> None:
        self._tick()
        if self.controller.should_forward("lidar"):
            self.lidar_publisher.publish(message)

    def _tick(self) -> None:
        if not self.controller.activation_due():
            return
        if (
            self.controller.scenario == "nav2_lifecycle_failure"
            and not self.lifecycle_client.service_is_ready()
        ):
            return
        self.controller.activate()
        self.node.get_logger().warning(
            f"fault injection active: {self.controller.scenario}"
        )
        self._publish_state("active")
        if self.controller.scenario == "nav2_lifecycle_failure":
            request = self._change_state_type.Request()
            request.transition.id = self._transition_type.TRANSITION_DEACTIVATE
            self.lifecycle_client.call_async(request)
            self._lifecycle_requested = True

    def _publish_state(self, state: str) -> None:
        from std_msgs.msg import String

        message = String()
        message.data = f"{self.controller.scenario}:{state}"
        self.state_publisher.publish(message)


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("sensor_guard")
    Ros2SensorGuard(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
