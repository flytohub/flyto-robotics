"""Velocity publishing that matches whatever message type the driver wants.

ROS 2 Jazzy's TurtleBot3 driver subscribes to ``/cmd_vel`` with
``TwistStamped`` while the bundled Gazebo bridge uses ``Twist``. A mismatch
matches zero subscribers and DDS reports no error at all, so the robot
silently ignores every command while the logs look healthy. Resolve the type
from the topic's live subscription info instead of hardcoding either one.
"""

from __future__ import annotations

import re
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

CMD_VEL_TYPE_AUTO = "auto"
CMD_VEL_TYPE_TWIST = "twist"
CMD_VEL_TYPE_TWIST_STAMPED = "twist_stamped"
CMD_VEL_TYPES = (CMD_VEL_TYPE_AUTO, CMD_VEL_TYPE_TWIST, CMD_VEL_TYPE_TWIST_STAMPED)
TWIST_STAMPED_TYPE_NAME = "geometry_msgs/msg/TwistStamped"
TOPIC_PATTERN = re.compile(r"^/?[A-Za-z_~][A-Za-z0-9_/]{0,127}$")
DEFAULT_FRAME_ID = "base_link"


def validated_topic(value: str, field_name: str) -> str:
    """Reject anything that is not a plain absolute ROS topic name."""
    if not isinstance(value, str) or not TOPIC_PATTERN.fullmatch(value):
        raise ValueError(f"{field_name} must be a valid ROS topic name")
    return value if value.startswith("/") else f"/{value}"


class CmdVelChannel:
    """Lazily bound velocity publisher for one node and one topic."""

    def __init__(
        self,
        node: Node,
        *,
        topic: str,
        cmd_vel_type: str = CMD_VEL_TYPE_AUTO,
        frame_id: str = DEFAULT_FRAME_ID,
    ) -> None:
        if cmd_vel_type not in CMD_VEL_TYPES:
            raise ValueError(f"cmd_vel_type must be one of {', '.join(CMD_VEL_TYPES)}")
        self._node = node
        self._topic = validated_topic(topic, "cmd_vel_topic")
        self._configured = cmd_vel_type
        self._frame_id = frame_id
        self._publisher: Any = None
        self._resolved = CMD_VEL_TYPE_TWIST

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def resolved_type(self) -> str:
        return self._resolved

    def _detect(self) -> str:
        if self._configured != CMD_VEL_TYPE_AUTO:
            return self._configured
        try:
            endpoints = self._node.get_subscriptions_info_by_topic(self._topic)
        except Exception:  # noqa: BLE001 - introspection is best-effort
            endpoints = []
        for endpoint in endpoints:
            if endpoint.topic_type == TWIST_STAMPED_TYPE_NAME:
                return CMD_VEL_TYPE_TWIST_STAMPED
        if endpoints:
            return CMD_VEL_TYPE_TWIST
        # Nobody is listening yet. Say so loudly rather than command into a void.
        self._node.get_logger().warning(
            f"no subscriber on {self._topic}; assuming Twist. The robot will "
            "ignore commands if its driver expects TwistStamped."
        )
        return CMD_VEL_TYPE_TWIST

    def _channel(self) -> Any:
        if self._publisher is not None:
            return self._publisher
        self._resolved = self._detect()
        message_type = (
            TwistStamped if self._resolved == CMD_VEL_TYPE_TWIST_STAMPED else Twist
        )
        self._publisher = self._node.create_publisher(message_type, self._topic, 10)
        self._node.get_logger().info(
            f"cmd_vel bound: topic={self._topic} "
            f"type={message_type.__name__} ({self._configured})"
        )
        return self._publisher

    def send(self, linear_x: float, angular_z: float) -> None:
        if not rclpy.ok():
            return
        publisher = self._channel()
        if self._resolved == CMD_VEL_TYPE_TWIST_STAMPED:
            message = TwistStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = self._frame_id
            message.twist.linear.x = linear_x
            message.twist.angular.z = angular_z
        else:
            message = Twist()
            message.linear.x = linear_x
            message.angular.z = angular_z
        publisher.publish(message)

    def stop(self) -> None:
        self.send(0.0, 0.0)
