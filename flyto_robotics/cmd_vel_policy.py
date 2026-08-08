"""Which velocity message type to publish, decided without importing ROS.

The decision itself is arithmetic on what DDS has discovered so far, but it used
to live inside a class that imports ``rclpy`` and ``geometry_msgs`` at module
scope. That made it unimportable on any machine without a ROS installation —
which is every development machine here — so the one piece of logic whose
failure mode is *the robot silently ignores every command* had no test at all.

Everything below takes discovered facts and returns a decision. The ROS-facing
half in :mod:`flyto_robotics.ros2_cmd_vel` does the introspection and the
publishing. Same split as :func:`flyto_robotics.mission.evaluate_sensor_gate`.
"""

from __future__ import annotations

import re
from collections.abc import Sequence
from typing import NamedTuple

CMD_VEL_TYPE_AUTO = "auto"
CMD_VEL_TYPE_TWIST = "twist"
CMD_VEL_TYPE_TWIST_STAMPED = "twist_stamped"
CMD_VEL_TYPES = (CMD_VEL_TYPE_AUTO, CMD_VEL_TYPE_TWIST, CMD_VEL_TYPE_TWIST_STAMPED)

TWIST_STAMPED_TYPE_NAME = "geometry_msgs/msg/TwistStamped"

#: No answer yet. The caller should publish nothing and ask again next tick.
UNDECIDED = ""

TOPIC_PATTERN = re.compile(r"^/?[A-Za-z_~][A-Za-z0-9_/]{0,127}$")


def validated_topic(value: str, field_name: str) -> str:
    """Reject anything that is not a plain absolute ROS topic name."""
    if not isinstance(value, str) or not TOPIC_PATTERN.fullmatch(value):
        raise ValueError(f"{field_name} must be a valid ROS topic name")
    return value if value.startswith("/") else f"/{value}"


class CmdVelDecision(NamedTuple):
    """A message type, and whether the answer is still open to revision.

    ``message_type`` is :data:`UNDECIDED` when discovery has not produced an
    answer and nothing forces one. ``provisional`` marks an answer that must be
    re-derived on the next tick rather than cached.
    """

    message_type: str
    provisional: bool


def resolve_cmd_vel_type(
    *,
    configured: str,
    subscriber_types: Sequence[str],
    discovery_expired: bool,
    stop_cannot_wait: bool = False,
) -> CmdVelDecision:
    """Decide what to publish on ``/cmd_vel``.

    ROS 2 Jazzy's TurtleBot3 driver subscribes with ``TwistStamped`` while the
    bundled Gazebo bridge uses ``Twist``. Publishing the wrong one matches zero
    subscribers, and DDS reports no error whatsoever — the logs stay clean while
    the robot ignores everything. So a wrong answer here is invisible, and that
    is what shapes every rule below.

    :param configured: the operator's ``cmd_vel_type`` parameter.
    :param subscriber_types: type names currently discovered on the topic.
    :param discovery_expired: whether the discovery grace window has elapsed.
    :param stop_cannot_wait: set when the caller is publishing a stop, which
        must go out even into a void — waiting for discovery before telling a
        moving robot to halt trades a silent failure for a moving one.
    """
    if configured != CMD_VEL_TYPE_AUTO:
        return CmdVelDecision(configured, provisional=False)

    if TWIST_STAMPED_TYPE_NAME in subscriber_types:
        return CmdVelDecision(CMD_VEL_TYPE_TWIST_STAMPED, provisional=False)

    if subscriber_types:
        # Someone is listening and it is not TwistStamped.
        return CmdVelDecision(CMD_VEL_TYPE_TWIST, provisional=False)

    if not discovery_expired:
        if stop_cannot_wait:
            return CmdVelDecision(CMD_VEL_TYPE_TWIST, provisional=True)
        return CmdVelDecision(UNDECIDED, provisional=True)

    # Discovery ran out of time. This is still a guess, so it stays provisional
    # — and that is the whole point.
    #
    # It used to be returned as decided, which cached it for the life of the
    # process. Discovery on a Raspberry Pi over Wi-Fi has been measured taking
    # 2.6-9.1s against a 3.0s window, so the driver routinely appears a moment
    # after the deadline. Marking the guess final meant that arrival was never
    # read: the mission published Twist at a TwistStamped topic for its whole
    # run, matched nobody, and failed with no DDS error to point at.
    #
    # A guess that cannot be revised is worse than no guess, because the cost of
    # being wrong is silence.
    return CmdVelDecision(CMD_VEL_TYPE_TWIST, provisional=True)
