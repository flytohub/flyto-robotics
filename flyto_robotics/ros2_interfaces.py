"""The one table of standard ROS 2 interfaces Flyto2 maps capabilities onto.

Every consumer derives its view from ``INTERFACES``:

- the generic ROS 2 adapter's declared interfaces (``declared_interfaces``)
  and the ones it uses on its own behalf (``own_interfaces``);
- the action class the adapter sends a goal with (``interface_class``);
- the action types the ROS 2 pairing manifest approves per stack
  (``stack_interfaces``);
- the one action the grant-bound executor (``ros2_action_executor``) runs,
  and the interface the standard pairing profile names for it
  (``executed_interface``).

A pairing approval is therefore exactly the set of actions the adapter can
drive, plus the rows reserved for stacks no adapter here drives yet. Adding a
motion means adding a row; nothing else keeps a copy.
"""

from __future__ import annotations

import importlib
import os
from dataclasses import dataclass

#: The adapter declares this interface as one of its capabilities.
DECLARED = "declared"
#: The adapter uses this interface on its own behalf, never as a capability.
OWN = "own"
#: Approved for pairing so a later adapter can map onto it; no adapter in this
#: package drives it.
RESERVED = "reserved"


@dataclass(frozen=True)
class Ros2Interface:
    """One standard ROS 2 interface and who may use it."""

    interface_type: str
    kind: str
    use: str
    #: The pairing stack that approves this interface; None means a manifest
    #: can never map a capability onto it (topics: pairing takes cancellable
    #: actions and services only).
    stack: str | None = None
    #: The adapter's capability id for a declared or own row.
    capability_id: str | None = None
    #: The environment variable that renames the graph endpoint, and its default.
    name_env: str | None = None
    default_name: str | None = None
    #: The grant-bound executor runs goals on this interface. Exactly one row.
    executed: bool = False


INTERFACES: tuple[Ros2Interface, ...] = (
    Ros2Interface(
        "nav2_msgs/action/NavigateToPose", "action", DECLARED, "nav2",
        "motion.navigate", "FLYTO_ROS2_NAVIGATE_ACTION", "/navigate_to_pose",
        executed=True,
    ),
    Ros2Interface(
        "sensor_msgs/msg/CompressedImage", "topic", DECLARED, None,
        "vision.observe", "FLYTO_ROS2_CAMERA_COMPRESSED_TOPIC",
        "/camera/image_raw/compressed",
    ),
    Ros2Interface(
        "nav_msgs/msg/OccupancyGrid", "topic", DECLARED, None,
        "sensing.map", "FLYTO_ROS2_MAP_TOPIC", "/map",
    ),
    Ros2Interface(
        "nav2_msgs/action/DriveOnHeading", "action", DECLARED, "nav2",
        "motion.advance", "FLYTO_ROS2_ADVANCE_ACTION", "/drive_on_heading",
    ),
    Ros2Interface(
        "nav2_msgs/action/BackUp", "action", DECLARED, "nav2",
        "motion.retreat", "FLYTO_ROS2_BACKUP_ACTION", "/backup",
    ),
    Ros2Interface(
        "nav2_msgs/action/Spin", "action", DECLARED, "nav2",
        "motion.rotate", "FLYTO_ROS2_SPIN_ACTION", "/spin",
    ),
    Ros2Interface(
        "geometry_msgs/msg/Twist|geometry_msgs/msg/TwistStamped", "topic", DECLARED,
        None, "motion.halt", "FLYTO_ROS2_CMD_VEL_TOPIC", "/cmd_vel",
    ),
    # An escape from an obstacle's inflation sends its waypoint and the
    # original goal as one NavigateThroughPoses goal when the graph has it.
    Ros2Interface(
        "nav2_msgs/action/NavigateThroughPoses", "action", OWN, "nav2",
        "motion.navigate_through_poses", "FLYTO_ROS2_NAVIGATE_THROUGH_ACTION",
        "/navigate_through_poses",
    ),
    Ros2Interface("moveit_msgs/action/MoveGroup", "action", RESERVED, "moveit2"),
    Ros2Interface("moveit_msgs/action/Pickup", "action", RESERVED, "moveit2"),
    Ros2Interface("moveit_msgs/action/Place", "action", RESERVED, "moveit2"),
    Ros2Interface(
        "control_msgs/action/FollowJointTrajectory", "action", RESERVED, "ros2_control"
    ),
    Ros2Interface(
        "control_msgs/action/GripperCommand", "action", RESERVED, "ros2_control"
    ),
)


def _endpoints(use: str) -> dict[str, tuple[str, str, str]]:
    return {
        row.capability_id: (
            row.kind,
            os.getenv(row.name_env, row.default_name),
            row.interface_type,
        )
        for row in INTERFACES
        if row.use == use
    }


def declared_interfaces() -> dict[str, tuple[str, str, str]]:
    """Capability id -> (kind, graph name, type) for the adapter's capabilities."""
    return _endpoints(DECLARED)


def own_interfaces() -> dict[str, tuple[str, str, str]]:
    """Capability id -> (kind, graph name, type) the adapter uses on its own behalf."""
    return _endpoints(OWN)


def stack_interfaces() -> dict[str, frozenset[str]]:
    """Pairing stack -> the interface types a manifest may map onto it."""
    stacks: dict[str, set[str]] = {}
    for row in INTERFACES:
        if row.stack is not None:
            stacks.setdefault(row.stack, set()).add(row.interface_type)
    return {stack: frozenset(types) for stack, types in stacks.items()}


def executed_interface() -> Ros2Interface:
    """The one row the grant-bound executor runs; anything else it refuses."""
    rows = [row for row in INTERFACES if row.executed]
    if len(rows) != 1:
        raise RuntimeError(
            f"exactly one interface row must be executed, found {len(rows)}"
        )
    return rows[0]


def interface_class(interface_type: str):
    """The generated ROS 2 class named by ``package/action/Name``."""
    package, segment, name = interface_type.split("/")
    return getattr(importlib.import_module(f"{package}.{segment}"), name)
