"""Independent latched emergency-stop supervisor for ROS 2 deployments."""

from __future__ import annotations

from typing import Any


class EmergencyStopSupervisor:
    """Own the only actuator command output and hold zero while latched."""

    def __init__(self, node: Any) -> None:
        from geometry_msgs.msg import Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import Bool
        from std_srvs.srv import Trigger

        self.node = node
        self._twist_type = Twist
        self._bool_type = Bool
        self.latched = False
        self.node.declare_parameter("cmd_vel_input_topic", "/nav2/cmd_vel")
        self.node.declare_parameter("cmd_vel_output_topic", "/cmd_vel")
        self.node.declare_parameter("state_topic", "/safety/emergency_stop_state")
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
        self.timer = self.node.create_timer(0.05, self._hold_stop)
        self._publish_state()

    def _gate_command(self, command: Any) -> None:
        if self.latched:
            self.command_publisher.publish(self._twist_type())
            return
        self.command_publisher.publish(command)

    def _stop(self, _request: Any, response: Any) -> Any:
        self.latched = True
        self.command_publisher.publish(self._twist_type())
        self._publish_state()
        response.success = True
        response.message = "emergency stop latched"
        return response

    def _reset(self, _request: Any, response: Any) -> Any:
        self.latched = False
        self._publish_state()
        response.success = True
        response.message = "emergency stop reset"
        return response

    def _hold_stop(self) -> None:
        if self.latched:
            self.command_publisher.publish(self._twist_type())

    def _publish_state(self) -> None:
        message = self._bool_type()
        message.data = self.latched
        self.state_publisher.publish(message)


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("emergency_supervisor")
    EmergencyStopSupervisor(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
