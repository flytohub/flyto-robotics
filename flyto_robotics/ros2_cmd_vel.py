"""Velocity publishing that matches whatever message type the driver wants.

ROS 2 Jazzy's TurtleBot3 driver subscribes to ``/cmd_vel`` with
``TwistStamped`` while the bundled Gazebo bridge uses ``Twist``. A mismatch
matches zero subscribers and DDS reports no error at all, so the robot
silently ignores every command while the logs look healthy. Resolve the type
from the topic's live subscription info instead of hardcoding either one.
"""

from __future__ import annotations

import re
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

CMD_VEL_TYPE_AUTO = "auto"
CMD_VEL_TYPE_TWIST = "twist"
CMD_VEL_TYPE_TWIST_STAMPED = "twist_stamped"
CMD_VEL_TYPES = (CMD_VEL_TYPE_AUTO, CMD_VEL_TYPE_TWIST, CMD_VEL_TYPE_TWIST_STAMPED)

# How long auto-detection waits for DDS to discover the robot's driver before
# falling back. Long enough that discovery on a Pi over Wi-Fi completes, short
# enough that a genuinely absent driver is reported rather than waited on.
DISCOVERY_GRACE_SECONDS = 3.0
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
        # DDS discovery is not instant. The first command can be requested
        # within a hundred milliseconds of the node appearing, long before the
        # robot's driver subscription has been discovered — and a guess made
        # then used to be cached forever, so the whole mission published Twist
        # at a TwistStamped topic, matched zero subscribers, and timed out with
        # no DDS error anywhere. Stay undecided for a bounded window instead.
        self._discovery_deadline = time.monotonic() + DISCOVERY_GRACE_SECONDS
        self._warned_no_subscriber = False
        # A binding made under force — a stop that could not wait for discovery
        # — is provisional, not decided. Caching it would defeat the whole
        # window, because a controller holding still publishes a stop on every
        # tick and would therefore always bind before discovery finished.
        self._provisional = False

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def resolved_type(self) -> str:
        return self._resolved

    def _resolve(self, *, force: bool) -> tuple[str, bool]:
        """The type to publish and whether that answer is provisional.

        An empty type means undecided: auto-detection has found no subscriber
        and the discovery window is still open, so the caller should retry on the
        next tick rather than commit. A guess here is silent — the wrong type
        matches no subscriber and DDS reports nothing at all — which is why this
        would rather publish nothing for a few milliseconds than publish into a
        topic the robot is not listening to.
        """
        if self._configured != CMD_VEL_TYPE_AUTO:
            return self._configured, False
        try:
            endpoints = self._node.get_subscriptions_info_by_topic(self._topic)
        except Exception:  # noqa: BLE001 - introspection is best-effort
            endpoints = []
        for endpoint in endpoints:
            if endpoint.topic_type == TWIST_STAMPED_TYPE_NAME:
                return CMD_VEL_TYPE_TWIST_STAMPED, False
        if endpoints:
            return CMD_VEL_TYPE_TWIST, False

        if time.monotonic() >= self._discovery_deadline:
            if not self._warned_no_subscriber:
                self._warned_no_subscriber = True
                self._node.get_logger().warning(
                    f"no subscriber on {self._topic} after "
                    f"{DISCOVERY_GRACE_SECONDS:.1f}s; assuming Twist. The robot "
                    "will ignore commands if its driver expects TwistStamped."
                )
            return CMD_VEL_TYPE_TWIST, False
        if force:
            # A stop cannot wait. Publish one now, and keep looking.
            return CMD_VEL_TYPE_TWIST, True
        return "", True

    def _channel(self, *, force: bool = False) -> Any:
        if self._publisher is not None and not self._provisional:
            return self._publisher

        resolved, provisional = self._resolve(force=force)
        if not resolved:
            return None

        if self._publisher is not None:
            if resolved == self._resolved:
                self._provisional = provisional
                return self._publisher
            # Discovery finished and the driver wants the other type. Replace the
            # provisional publisher rather than keeping a channel nobody reads.
            self._node.destroy_publisher(self._publisher)
            self._publisher = None

        self._resolved = resolved
        self._provisional = provisional
        message_type = (
            TwistStamped if resolved == CMD_VEL_TYPE_TWIST_STAMPED else Twist
        )
        self._publisher = self._node.create_publisher(message_type, self._topic, 10)
        self._node.get_logger().info(
            f"cmd_vel bound: topic={self._topic} type={message_type.__name__} "
            f"({self._configured}{', provisional' if provisional else ''})"
        )
        return self._publisher

    def send(self, linear_x: float, angular_z: float) -> None:
        if not rclpy.ok():
            return
        publisher = self._channel()
        if publisher is None:
            # Still discovering. The controller ticks again in milliseconds, and
            # the robot has not been told to move, so skipping is safe.
            return
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
        """Command zero velocity, binding immediately if still undecided.

        Never skipped. A stop is the one command that must go out even into a
        void: waiting for discovery before telling a moving robot to stop would
        trade a silent failure for a moving one.
        """
        if not rclpy.ok():
            return
        publisher = self._channel(force=True)
        if publisher is None:
            return
        if self._resolved == CMD_VEL_TYPE_TWIST_STAMPED:
            message = TwistStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = self._frame_id
        else:
            message = Twist()
        publisher.publish(message)
