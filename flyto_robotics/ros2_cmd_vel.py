"""Velocity publishing that matches whatever message type the driver wants.

ROS 2 Jazzy's TurtleBot3 driver subscribes to ``/cmd_vel`` with
``TwistStamped`` while the bundled Gazebo bridge uses ``Twist``. A mismatch
matches zero subscribers and DDS reports no error at all, so the robot
silently ignores every command while the logs look healthy. Resolve the type
from the topic's live subscription info instead of hardcoding either one.

This module is the ROS-facing half: it asks the graph what it can see and
publishes the result. The decision it feeds those observations to lives in
:mod:`flyto_robotics.cmd_vel_policy`, which imports no ROS and is therefore
testable on a machine that has none.
"""

from __future__ import annotations

import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node

from .cmd_vel_policy import (
    CMD_VEL_TYPE_AUTO,
    CMD_VEL_TYPE_TWIST,
    CMD_VEL_TYPE_TWIST_STAMPED,
    CMD_VEL_TYPES,
    TOPIC_PATTERN,
    TWIST_STAMPED_TYPE_NAME,
    resolve_cmd_vel_type,
    validated_topic,
)

__all__ = [
    "CMD_VEL_TYPE_AUTO",
    "CMD_VEL_TYPE_TWIST",
    "CMD_VEL_TYPE_TWIST_STAMPED",
    "CMD_VEL_TYPES",
    "DEFAULT_FRAME_ID",
    "DISCOVERY_GRACE_SECONDS",
    "TOPIC_PATTERN",
    "TWIST_STAMPED_TYPE_NAME",
    "CmdVelChannel",
    "validated_topic",
]

# How long auto-detection waits for DDS to discover the robot's driver before
# publishing a guess. Long enough that discovery on a Pi over Wi-Fi usually
# completes, short enough that a genuinely absent driver is reported rather than
# waited on. Expiry is not a decision: the guess it produces stays open to
# revision, because discovery here has been measured finishing well after it.
DISCOVERY_GRACE_SECONDS = 3.0
DEFAULT_FRAME_ID = "base_link"


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
        # robot's driver subscription has been discovered.
        self._discovery_deadline = time.monotonic() + DISCOVERY_GRACE_SECONDS
        self._warned_no_subscriber = False
        # A provisional binding is re-derived every tick rather than cached, so
        # a driver that appears late is still picked up.
        self._provisional = False

    @property
    def topic(self) -> str:
        return self._topic

    @property
    def resolved_type(self) -> str:
        return self._resolved

    def _subscriber_types(self) -> list[str]:
        """Type names currently discovered on the topic, best-effort."""
        try:
            endpoints = self._node.get_subscriptions_info_by_topic(self._topic)
        except Exception:  # noqa: BLE001 - introspection is best-effort
            return []
        return [endpoint.topic_type for endpoint in endpoints]

    def _resolve(self, *, force: bool) -> tuple[str, bool]:
        subscriber_types = self._subscriber_types()
        expired = time.monotonic() >= self._discovery_deadline
        if expired and not subscriber_types and not self._warned_no_subscriber:
            self._warned_no_subscriber = True
            self._node.get_logger().warning(
                f"no subscriber on {self._topic} after "
                f"{DISCOVERY_GRACE_SECONDS:.1f}s; publishing Twist for now and "
                "still watching. If the driver expects TwistStamped it will be "
                "rebound as soon as it is discovered."
            )
        return resolve_cmd_vel_type(
            configured=self._configured,
            subscriber_types=subscriber_types,
            discovery_expired=expired,
            stop_cannot_wait=force,
        )

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
