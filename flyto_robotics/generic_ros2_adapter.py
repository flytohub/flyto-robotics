#!/usr/bin/env python3
"""Generic ROS 2 adapter for an external AI Space / Computer.

This adapter is intentionally NOT a TurtleBot3 runtime. It runs on a Mac,
laptop, Steam Deck, mini PC, or AI Space computer that can see a standard ROS 2
graph over DDS, Zenoh, or another ordinary ROS transport.

Southbound, it speaks only standard ROS 2 interfaces:
- nav2_msgs/action/NavigateToPose
- nav2_msgs/action/DriveOnHeading
- nav2_msgs/action/BackUp
- nav2_msgs/action/Spin
- geometry_msgs/msg/Twist or TwistStamped for a final zero-velocity stop
- nav_msgs/msg/Odometry and sensor_msgs/msg/LaserScan for evidence
- sensor_msgs/msg/Image + CameraInfo and tf2_msgs/msg/TFMessage for observation parity

No Flyto2 process, credential, scheduler, gateway, task database, or custom ROS
node is required on the robot. Reinstalling a TurtleBot3 from upstream ROS 2
and TurtleBot3 documentation is a resource replacement, not Flyto2
re-provisioning.

The declarations arrive as device_report / DISCOVERED. ROS graph discovery is
evidence that an interface exists, not permission to move a robot.
"""

from __future__ import annotations

import base64
import contextlib
import hashlib
import json
import math
import os
import threading
import time
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from typing import Any, Callable, Protocol

from . import adapter_contract as decl
from .adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)
from .ros2_observation_bundle import (
    build_ros2_observation_bundle as build_observation_bundle,
)
from .ros2_observation_bundle import runtime_snapshot

CMD_VEL_TYPES = frozenset(
    {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}
)
ODOM_TYPE = "nav_msgs/msg/Odometry"
SCAN_TYPE = "sensor_msgs/msg/LaserScan"

DEFAULT_INTERFACES = {
    "motion.navigate": (
        "action",
        os.getenv("FLYTO_ROS2_NAVIGATE_ACTION", "/navigate_to_pose"),
        "nav2_msgs/action/NavigateToPose",
    ),
    "motion.advance": (
        "action",
        os.getenv("FLYTO_ROS2_ADVANCE_ACTION", "/drive_on_heading"),
        "nav2_msgs/action/DriveOnHeading",
    ),
    "motion.retreat": (
        "action",
        os.getenv("FLYTO_ROS2_BACKUP_ACTION", "/backup"),
        "nav2_msgs/action/BackUp",
    ),
    "motion.rotate": (
        "action",
        os.getenv("FLYTO_ROS2_SPIN_ACTION", "/spin"),
        "nav2_msgs/action/Spin",
    ),
    "motion.halt": (
        "topic",
        os.getenv("FLYTO_ROS2_CMD_VEL_TOPIC", "/cmd_vel"),
        "geometry_msgs/msg/Twist|geometry_msgs/msg/TwistStamped",
    ),
}

ARGUMENTS: Mapping[str, tuple[decl.DeclaredArgument, ...]] = {
    "motion.navigate": (
        decl.DeclaredArgument("x", required=True, minimum=-1000.0, maximum=1000.0, unit="m"),
        decl.DeclaredArgument("y", required=True, minimum=-1000.0, maximum=1000.0, unit="m"),
        decl.DeclaredArgument(
            "yaw_radians", required=False, minimum=-math.pi, maximum=math.pi, unit="rad"
        ),
    ),
    "motion.advance": (
        decl.DeclaredArgument(
            "distance_m", required=True, minimum=0.05, maximum=2.0, unit="m"
        ),
        decl.DeclaredArgument(
            "speed_mps", required=False, minimum=0.02, maximum=0.25, unit="m/s"
        ),
    ),
    "motion.retreat": (
        decl.DeclaredArgument(
            "distance_m", required=True, minimum=0.05, maximum=2.0, unit="m"
        ),
        decl.DeclaredArgument(
            "speed_mps", required=False, minimum=0.02, maximum=0.20, unit="m/s"
        ),
    ),
    "motion.rotate": (
        decl.DeclaredArgument(
            "yaw_radians", required=True, minimum=-math.pi, maximum=math.pi, unit="rad"
        ),
    ),
    "motion.halt": (),
}


@dataclass(frozen=True)
class StandardInterface:
    kind: str
    name: str
    type: str


class ROS2Backend(Protocol):
    def discover(self) -> Sequence[StandardInterface]: ...

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult: ...

    def cancel(self, call_id: str) -> CallResult: ...

    def safe_stop(self, call_id: str) -> CallResult: ...

    def execution_count(self, call_id: str) -> int: ...

    def observation(self) -> Mapping[str, Any]: ...


def _numeric_arguments(capability_id: str, arguments: Mapping[str, Any]) -> dict[str, float]:
    schemas = {item.name: item for item in ARGUMENTS[capability_id]}
    unknown = set(arguments) - set(schemas)
    if unknown:
        raise ValueError(f"unsupported arguments: {sorted(unknown)}")
    result: dict[str, float] = {}
    for name, schema in schemas.items():
        raw = arguments.get(name)
        if raw is None:
            if schema.required:
                raise ValueError(f"{name} is required")
            continue
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f"{name} must be a number")
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"{name} must be finite")
        if schema.minimum is not None and value < schema.minimum:
            raise ValueError(f"{name} is below the declared minimum")
        if schema.maximum is not None and value > schema.maximum:
            raise ValueError(f"{name} is above the declared maximum")
        result[name] = value
    return result


class GenericROS2Adapter:
    """Flyto2 capability adapter over a standard ROS 2 graph."""

    def __init__(self, *, backend: ROS2Backend | None = None, resource_id: str = ""):
        self.backend = backend or RclpyROS2Backend()
        self.resource_id = (
            resource_id
            or os.getenv("FLYTO_ROS2_RESOURCE_ID", "").strip()
            or "ros2-resource"
        )
        self._declared: set[str] = set()

    def _discovered_capabilities(self) -> set[str]:
        interfaces = {(item.kind, item.name, item.type) for item in self.backend.discover()}
        available: set[str] = set()
        for capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if capability_id == "motion.halt":
                if any(
                    item_kind == "topic"
                    and item_name == name
                    and item_type in CMD_VEL_TYPES
                    for item_kind, item_name, item_type in interfaces
                ):
                    available.add(capability_id)
                continue
            if (kind, name, interface_type) in interfaces:
                available.add(capability_id)
        return available

    def describe(self) -> Sequence[decl.CapabilityDeclaration]:
        available = self._discovered_capabilities()
        self._declared = available
        declarations: list[decl.CapabilityDeclaration] = []
        for capability_id in sorted(available):
            kind, name, interface_type = DEFAULT_INTERFACES[capability_id]
            observations = (
                f"{os.getenv('FLYTO_ROS2_ODOM_TOPIC', '/odom')}:{ODOM_TYPE}",
                f"{os.getenv('FLYTO_ROS2_SCAN_TOPIC', '/scan')}:{SCAN_TYPE}",
            )
            declarations.append(
                decl.declare(
                    capability_id=capability_id,
                    resource_id=self.resource_id,
                    executor_kind=decl.EXECUTOR_EXTERNAL_API,
                    source=decl.SOURCE_DEVICE,
                    arguments=ARGUMENTS[capability_id],
                    required_observations=observations,
                    runtime_name=f"{kind}:{name}:{interface_type}",
                    version="1.0.0",
                )
            )
        return declarations

    def _motion_preflight(self, capability_id: str) -> str | None:
        if capability_id == "motion.halt":
            return None
        observation = self.backend.observation()
        if observation.get("pose") is None:
            return "fresh odometry is required before motion"
        range_observation = observation.get("range")
        if not isinstance(range_observation, Mapping):
            return "fresh LiDAR is required before motion"
        try:
            clearance = float(range_observation["minimum_range_m"])
        except (KeyError, TypeError, ValueError):
            return "valid LiDAR clearance is required before motion"
        required_clearance = max(
            0.1,
            float(os.getenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.35")),
        )
        if clearance < required_clearance:
            return (
                f"LiDAR clearance {clearance:.3f}m is below the "
                f"{required_clearance:.3f}m motion safety minimum"
            )
        if capability_id == "motion.navigate" and not observation.get(
            "map_tf_available", False
        ):
            return "fresh map-to-odom transform is required before navigation"
        return None

    def invoke(self, request: CallRequest) -> CallResult:
        if request.capability_id not in self._declared:
            self.describe()
        if request.capability_id not in self._declared:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"{request.capability_id} is not present on the ROS 2 graph",
            )
        if request.deadline_seconds <= 0:
            return CallResult(request.call_id, OUTCOME_TIMEOUT, detail="no time to run")
        try:
            arguments = _numeric_arguments(request.capability_id, request.arguments)
        except ValueError as error:
            return CallResult(request.call_id, OUTCOME_REFUSED, detail=str(error))
        preflight_error = self._motion_preflight(request.capability_id)
        if preflight_error is not None:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=preflight_error,
            )
        if request.capability_id == "motion.halt":
            return self.backend.safe_stop(request.call_id)
        return self.backend.invoke(
            call_id=request.call_id,
            capability_id=request.capability_id,
            arguments=arguments,
            deadline_seconds=float(request.deadline_seconds),
        )

    def cancel(self, call_id: str) -> CallResult:
        return self.backend.cancel(call_id)

    def safe_stop(self) -> CallResult:
        return self.backend.safe_stop(f"safe-stop-{time.monotonic_ns()}")

    def execution_count(self, call_id: str) -> int:
        return self.backend.execution_count(call_id)

    def observe(
        self,
        *,
        phase: str = "preflight",
        execution_id: str | None = None,
        deployment_mode: str | None = None,
        provider: str | None = None,
    ) -> dict[str, Any]:
        interfaces = tuple(self.backend.discover())
        observation = dict(self.backend.observation())
        mode = (
            deployment_mode
            or os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
        ).strip().lower()
        source = provider or ("gazebo" if mode == "simulation" else "ros2")
        return build_observation_bundle(
            resource_id=self.resource_id,
            runtime_snapshot=runtime_snapshot(interfaces),
            deployment_mode=mode,
            provider=source,
            phase=phase,
            execution_id=execution_id,
            pose=observation.get("pose"),
            range_observation=observation.get("range"),
            camera=observation.get("camera"),
            map_tf_available=bool(observation.get("map_tf_available", False)),
        )

    def disconnect(self) -> None:
        method = getattr(self.backend, "disconnect", None)
        if callable(method):
            method()

    def reconnect(self) -> None:
        method = getattr(self.backend, "reconnect", None)
        if callable(method):
            method()


class RclpyROS2Backend:
    """rclpy client for standard Nav2 actions and observation topics."""

    def __init__(self) -> None:
        try:
            import rclpy
            from nav_msgs.msg import Odometry
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image, LaserScan
            from tf2_msgs.msg import TFMessage
        except ImportError as error:
            raise RuntimeError(
                "rclpy/nav2 messages are required on the external ROS 2 adapter host"
            ) from error

        if not rclpy.ok():
            rclpy.init(args=None)
        self._rclpy = rclpy
        self._node = Node("flyto_external_generic_ros2_adapter")
        self._sensor_qos = qos_profile_sensor_data
        self._connected = True
        self._goal_handles: dict[str, Any] = {}
        self._result_futures: dict[str, Any] = {}
        self._results: dict[str, CallResult] = {}
        self._counts: dict[str, int] = {}
        self._pose: dict[str, float] | None = None
        self._pose_seen_at: float | None = None
        self._minimum_range: float | None = None
        self._range_sample_count = 0
        self._range_seen_at: float | None = None
        self._camera: dict[str, Any] | None = None
        self._camera_seen_at: float | None = None
        self._camera_calibration_snapshot: str | None = None
        self._map_tf_seen_at: float | None = None
        self._publishers: dict[str, Any] = {}

        self._node.create_subscription(
            Odometry,
            os.getenv("FLYTO_ROS2_ODOM_TOPIC", "/odom"),
            self._on_odometry,
            self._sensor_qos,
        )
        self._node.create_subscription(
            LaserScan,
            os.getenv("FLYTO_ROS2_SCAN_TOPIC", "/scan"),
            self._on_scan,
            self._sensor_qos,
        )
        self._node.create_subscription(
            Image,
            os.getenv("FLYTO_ROS2_CAMERA_TOPIC", "/camera/image_raw"),
            self._on_camera,
            self._sensor_qos,
        )
        self._node.create_subscription(
            CameraInfo,
            os.getenv("FLYTO_ROS2_CAMERA_INFO_TOPIC", "/camera/camera_info"),
            self._on_camera_info,
            self._sensor_qos,
        )
        self._node.create_subscription(
            TFMessage,
            os.getenv("FLYTO_ROS2_TF_TOPIC", "/tf"),
            self._on_tf,
            10,
        )

    def _on_odometry(self, message: Any) -> None:
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self._pose = {
            "frame": "odom",
            "x": float(message.pose.pose.position.x),
            "y": float(message.pose.pose.position.y),
            "yaw": float(yaw),
        }
        self._pose_seen_at = time.monotonic()

    def _on_scan(self, message: Any) -> None:
        usable = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
            and float(value) >= float(message.range_min)
            and float(value) <= float(message.range_max)
        ]
        if usable:
            self._minimum_range = min(usable)
            self._range_sample_count = len(usable)
            self._range_seen_at = time.monotonic()

    def _on_camera(self, message: Any) -> None:
        payload = bytes(message.data)
        frame_seed = (
            str(message.encoding).encode("utf-8")
            + b"|"
            + str(int(message.width)).encode("ascii")
            + b"x"
            + str(int(message.height)).encode("ascii")
            + b"|"
            + payload
        )
        self._camera = {
            "encoding": str(message.encoding),
            "width": int(message.width),
            "height": int(message.height),
            "calibrated": self._camera_calibration_snapshot is not None,
            "calibration_snapshot": self._camera_calibration_snapshot,
            "frame_snapshot": hashlib.sha256(frame_seed).hexdigest(),
        }
        self._camera_seen_at = time.monotonic()

    def _on_camera_info(self, message: Any) -> None:
        matrix = tuple(float(value) for value in message.k)
        calibrated = len(matrix) == 9 and matrix[0] != 0.0
        if not calibrated:
            self._camera_calibration_snapshot = None
            if self._camera is not None:
                self._camera["calibrated"] = False
                self._camera["calibration_snapshot"] = None
            return
        seed = repr(
            (
                str(message.distortion_model),
                tuple(float(value) for value in message.d),
                matrix,
                tuple(float(value) for value in message.r),
                tuple(float(value) for value in message.p),
                int(message.width),
                int(message.height),
            )
        ).encode("utf-8")
        self._camera_calibration_snapshot = hashlib.sha256(seed).hexdigest()
        if self._camera is not None:
            self._camera["calibrated"] = True
            self._camera["calibration_snapshot"] = self._camera_calibration_snapshot

    def _on_tf(self, message: Any) -> None:
        map_frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
        odom_frame = os.getenv("FLYTO_ROS2_ODOM_FRAME", "odom")
        if any(
            item.header.frame_id == map_frame and item.child_frame_id == odom_frame
            for item in message.transforms
        ):
            self._map_tf_seen_at = time.monotonic()

    def observation(self) -> Mapping[str, Any]:
        self._spin(0.25)
        now = time.monotonic()
        max_age = max(
            0.1,
            float(os.getenv("FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS", "5")),
        )

        def fresh(seen_at: float | None) -> bool:
            return seen_at is not None and now - seen_at <= max_age

        return {
            "pose": (
                dict(self._pose)
                if self._pose is not None and fresh(self._pose_seen_at)
                else None
            ),
            "range": (
                {
                    "minimum_range_m": self._minimum_range,
                    "sample_count": self._range_sample_count,
                }
                if self._minimum_range is not None and fresh(self._range_seen_at)
                else None
            ),
            "camera": (
                dict(self._camera)
                if self._camera is not None and fresh(self._camera_seen_at)
                else None
            ),
            "map_tf_available": fresh(self._map_tf_seen_at),
        }

    def discover(self) -> Sequence[StandardInterface]:
        self._spin(0.15)
        topics = {
            name: tuple(types) for name, types in self._node.get_topic_names_and_types()
        }
        actions = {
            name: tuple(types) for name, types in self._node.get_action_names_and_types()
        }
        found: list[StandardInterface] = []
        for _capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if kind == "action":
                if interface_type in actions.get(name, ()):
                    found.append(StandardInterface(kind, name, interface_type))
                continue
            for topic_type in topics.get(name, ()):
                if topic_type in CMD_VEL_TYPES:
                    found.append(StandardInterface("topic", name, topic_type))
        return found

    def _spin(self, seconds: float) -> None:
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline:
            self._rclpy.spin_once(
                self._node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic()))
            )

    def _spin_until(self, future: Any, deadline: float) -> bool:
        while not future.done() and time.monotonic() < deadline:
            self._rclpy.spin_once(
                self._node, timeout_sec=min(0.05, max(0.0, deadline - time.monotonic()))
            )
        return future.done()

    def _evidence(self, capability_id: str) -> dict[str, Any]:
        kind, name, interface_type = DEFAULT_INTERFACES[capability_id]
        evidence: dict[str, Any] = {
            "adapter": "generic_ros2",
            "standard_interface": {
                "kind": kind,
                "name": name,
                "type": interface_type,
            },
        }
        if self._pose is not None:
            evidence["odom"] = dict(self._pose)
        if self._minimum_range is not None:
            evidence["minimum_range_m"] = self._minimum_range
        return evidence

    def _action_type(self, capability_id: str):
        from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin

        return {
            "motion.navigate": NavigateToPose,
            "motion.advance": DriveOnHeading,
            "motion.retreat": BackUp,
            "motion.rotate": Spin,
        }[capability_id]

    def _goal(self, capability_id: str, arguments: Mapping[str, float]):
        action_type = self._action_type(capability_id)
        goal = action_type.Goal()
        if capability_id != "motion.navigate":
            allowance = max(
                1.0, float(os.getenv("FLYTO_ROS2_ACTION_ALLOWANCE_SECONDS", "30"))
            )
            goal.time_allowance.sec = int(allowance)
            goal.time_allowance.nanosec = int(
                (allowance - int(allowance)) * 1_000_000_000
            )
        if capability_id == "motion.navigate":
            yaw = float(arguments.get("yaw_radians", 0.0))
            goal.pose.header.frame_id = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(arguments["x"])
            goal.pose.pose.position.y = float(arguments["y"])
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        elif capability_id == "motion.rotate":
            goal.target_yaw = float(arguments["yaw_radians"])
        else:
            goal.target.x = float(arguments["distance_m"])
            goal.speed = float(
                arguments.get(
                    "speed_mps",
                    0.10 if capability_id == "motion.retreat" else 0.12,
                )
            )
        return goal

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult:
        if not self._connected:
            return CallResult(
                call_id, OUTCOME_REFUSED, detail="ROS 2 adapter disconnected"
            )
        if call_id in self._results:
            return self._results[call_id]

        from action_msgs.msg import GoalStatus
        from rclpy.action import ActionClient

        if call_id not in self._goal_handles:
            _, action_name, _ = DEFAULT_INTERFACES[capability_id]
            client = ActionClient(
                self._node, self._action_type(capability_id), action_name
            )
            if not client.wait_for_server(timeout_sec=min(1.0, deadline_seconds)):
                return CallResult(
                    call_id, OUTCOME_REFUSED, detail="ROS 2 action server unavailable"
                )
            started = client.send_goal_async(self._goal(capability_id, arguments))
            accept_deadline = time.monotonic() + min(2.0, deadline_seconds)
            if not self._spin_until(started, accept_deadline):
                return CallResult(
                    call_id, OUTCOME_TIMEOUT, detail="goal acceptance timed out"
                )
            handle = started.result()
            if handle is None or not handle.accepted:
                return CallResult(
                    call_id, OUTCOME_REFUSED, detail="ROS 2 action goal refused"
                )
            self._goal_handles[call_id] = handle
            self._result_futures[call_id] = handle.get_result_async()
            self._counts[call_id] = self._counts.get(call_id, 0) + 1

        future = self._result_futures[call_id]
        deadline = time.monotonic() + deadline_seconds

        if not self._spin_until(future, deadline):
            return CallResult(
                call_id,
                OUTCOME_TIMEOUT,
                evidence=self._evidence(capability_id),
                detail="ROS 2 action still running",
            )
        response = future.result()
        status = int(getattr(response, "status", 0))
        if status == GoalStatus.STATUS_SUCCEEDED:
            result = CallResult(
                call_id, OUTCOME_COMPLETED, evidence=self._evidence(capability_id)
            )
        elif status == GoalStatus.STATUS_CANCELED:
            result = CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        else:
            result = CallResult(
                call_id, OUTCOME_FAILED, detail=f"ROS 2 action status {status}"
            )
        self._results[call_id] = result
        return result

    def _publish_zero(self) -> bool:
        topic = DEFAULT_INTERFACES["motion.halt"][1]
        types = {
            item.type
            for item in self.discover()
            if item.kind == "topic" and item.name == topic
        }
        if len(types) != 1:
            return False
        topic_type = next(iter(types))
        if topic_type == "geometry_msgs/msg/TwistStamped":
            from geometry_msgs.msg import TwistStamped

            publisher = self._publishers.get(topic_type)
            if publisher is None:
                publisher = self._node.create_publisher(TwistStamped, topic, 10)
                self._publishers[topic_type] = publisher
            message = TwistStamped()
            message.header.stamp = self._node.get_clock().now().to_msg()
            message.header.frame_id = os.getenv("FLYTO_ROS2_BASE_FRAME", "base_link")
        elif topic_type == "geometry_msgs/msg/Twist":
            from geometry_msgs.msg import Twist

            publisher = self._publishers.get(topic_type)
            if publisher is None:
                publisher = self._node.create_publisher(Twist, topic, 10)
                self._publishers[topic_type] = publisher
            message = Twist()
        else:
            return False
        publisher.publish(message)
        self._spin(0.1)
        return True

    def cancel(self, call_id: str) -> CallResult:
        handle = self._goal_handles.get(call_id)
        if handle is None:
            return CallResult(
                call_id, OUTCOME_REFUSED, detail="no active ROS 2 goal"
            )
        future = handle.cancel_goal_async()
        if not self._spin_until(future, time.monotonic() + 2.0):
            return CallResult(
                call_id, OUTCOME_FAILED, detail="ROS 2 cancel timed out"
            )
        response = future.result()
        accepted = bool(getattr(response, "goals_canceling", ()))
        stopped = self._publish_zero()
        if not accepted or not stopped:
            return CallResult(
                call_id, OUTCOME_FAILED, detail="ROS 2 cancel/stop was not confirmed"
            )
        result = CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        self._results[call_id] = result
        return result

    def safe_stop(self, call_id: str) -> CallResult:
        for active_call in tuple(self._goal_handles):
            if active_call not in self._results:
                self.cancel(active_call)
        if not self._publish_zero():
            return CallResult(
                call_id, OUTCOME_FAILED, detail="standard cmd_vel stop unavailable"
            )
        self._counts[call_id] = self._counts.get(call_id, 0) + 1
        result = CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "adapter": "generic_ros2",
                "standard_interface": {
                    "kind": "topic",
                    "name": DEFAULT_INTERFACES["motion.halt"][1],
                    "type": "geometry_msgs/msg/Twist|TwistStamped",
                },
                "zero_velocity_published": True,
            },
        )
        self._results[call_id] = result
        return result

    def execution_count(self, call_id: str) -> int:
        return self._counts.get(call_id, 0)

    def disconnect(self) -> None:
        self._connected = False

    def reconnect(self) -> None:
        self._connected = True



class RosbridgeROS2Backend:
    """WebSocket ROS 2 backend for an external AI Space host.

    The robot runs only upstream rosbridge/rosapi. The adapter stays on the
    external computer and speaks the same standard ROS 2 capability contract
    as :class:`RclpyROS2Backend`.
    """

    def __init__(
        self,
        *,
        url: str | None = None,
        connection_factory: Callable[[str], Any] | None = None,
    ) -> None:
        self._url = (
            url
            or os.getenv("FLYTO_ROSBRIDGE_URL", "").strip()
            or "ws://127.0.0.1:19090"
        )
        self._connection_factory = connection_factory or self._connect
        self._ws: Any | None = None
        self._connected = False
        self._condition = threading.Condition()
        self._send_lock = threading.Lock()
        self._reader: threading.Thread | None = None
        self._responses: dict[str, dict[str, Any]] = {}
        self._action_results: dict[str, dict[str, Any]] = {}
        self._active_actions: dict[str, str] = {}
        self._results: dict[str, CallResult] = {}
        self._counts: dict[str, int] = {}
        self._advertised_topics: set[tuple[str, str]] = set()

        self._pose: dict[str, float | str] | None = None
        self._pose_seen_at: float | None = None
        self._minimum_range: float | None = None
        self._range_sample_count = 0
        self._range_seen_at: float | None = None
        self._camera: dict[str, Any] | None = None
        self._camera_seen_at: float | None = None
        self._camera_calibration_snapshot: str | None = None
        self._map_tf_seen_at: float | None = None
        self._topic_types: dict[str, str] = {}
        self.reconnect()

    @staticmethod
    def _connect(url: str) -> Any:
        try:
            from websockets.sync.client import connect
        except ImportError as error:
            raise RuntimeError(
                "websockets>=12 is required for the rosbridge ROS 2 backend"
            ) from error
        return connect(
            url,
            open_timeout=3,
            close_timeout=1,
            max_size=None,
        )

    def _send(self, payload: Mapping[str, Any]) -> None:
        if not self._connected or self._ws is None:
            raise RuntimeError("rosbridge adapter disconnected")
        encoded = json.dumps(payload, separators=(",", ":"), allow_nan=False)
        with self._send_lock:
            self._ws.send(encoded)

    def _reader_loop(self, websocket: Any) -> None:
        while True:
            with self._condition:
                if not self._connected or websocket is not self._ws:
                    return
            try:
                raw = websocket.recv(timeout=1.0)
            except TimeoutError:
                continue
            except Exception:
                with self._condition:
                    if websocket is self._ws:
                        self._connected = False
                        self._condition.notify_all()
                return
            try:
                message = json.loads(raw)
            except (TypeError, json.JSONDecodeError):
                continue
            if not isinstance(message, Mapping):
                continue
            self._handle_message(dict(message))

    def _handle_message(self, message: dict[str, Any]) -> None:
        operation = message.get("op")
        if operation == "publish":
            self._handle_publish(
                str(message.get("topic", "")),
                message.get("msg"),
            )
            return
        identifier = message.get("id")
        if not isinstance(identifier, str):
            return
        with self._condition:
            if operation == "service_response":
                self._responses[identifier] = message
                self._condition.notify_all()
            elif operation == "action_result":
                self._action_results[identifier] = message
                self._condition.notify_all()

    @staticmethod
    def _camera_payload(data: Any) -> bytes:
        if isinstance(data, str):
            try:
                return base64.b64decode(data, validate=False)
            except (ValueError, TypeError):
                return data.encode("utf-8", errors="ignore")
        if isinstance(data, list):
            try:
                return bytes(int(value) & 0xFF for value in data)
            except (TypeError, ValueError):
                return b""
        return b""

    def _update_odometry(self, message: Mapping[str, Any], observed: float) -> None:
        pose = message.get("pose")
        nested = pose.get("pose") if isinstance(pose, Mapping) else None
        position = nested.get("position") if isinstance(nested, Mapping) else None
        orientation = nested.get("orientation") if isinstance(nested, Mapping) else None
        if not isinstance(position, Mapping) or not isinstance(orientation, Mapping):
            return
        try:
            x = float(position.get("x", 0.0))
            y = float(position.get("y", 0.0))
            ox = float(orientation.get("x", 0.0))
            oy = float(orientation.get("y", 0.0))
            oz = float(orientation.get("z", 0.0))
            ow = float(orientation.get("w", 1.0))
        except (TypeError, ValueError):
            return
        self._pose = {
            "frame": "odom",
            "x": x,
            "y": y,
            "yaw": math.atan2(
                2.0 * (ow * oz + ox * oy),
                1.0 - 2.0 * (oy * oy + oz * oz),
            ),
        }
        self._pose_seen_at = observed
        self._condition.notify_all()

    def _update_scan(self, message: Mapping[str, Any], observed: float) -> None:
        ranges = message.get("ranges")
        if not isinstance(ranges, list):
            return
        try:
            minimum = float(message.get("range_min", 0.0))
            maximum = float(message.get("range_max", math.inf))
        except (TypeError, ValueError):
            return
        usable: list[float] = []
        for raw in ranges:
            try:
                value = float(raw)
            except (TypeError, ValueError):
                continue
            if math.isfinite(value) and minimum <= value <= maximum:
                usable.append(value)
        if not usable:
            return
        self._minimum_range = min(usable)
        self._range_sample_count = len(usable)
        self._range_seen_at = observed
        self._condition.notify_all()

    def _update_camera(self, message: Mapping[str, Any], observed: float) -> None:
        try:
            encoding = str(message["encoding"])
            width = int(message["width"])
            height = int(message["height"])
        except (KeyError, TypeError, ValueError):
            return
        payload = self._camera_payload(message.get("data", ""))
        seed = (
            encoding.encode("utf-8")
            + b"|"
            + str(width).encode("ascii")
            + b"x"
            + str(height).encode("ascii")
            + b"|"
            + payload
        )
        self._camera = {
            "encoding": encoding,
            "width": width,
            "height": height,
            "calibrated": self._camera_calibration_snapshot is not None,
            "calibration_snapshot": self._camera_calibration_snapshot,
            "frame_snapshot": hashlib.sha256(seed).hexdigest(),
        }
        self._camera_seen_at = observed
        self._condition.notify_all()

    def _update_camera_info(self, message: Mapping[str, Any]) -> None:
        matrix = message.get("k")
        if not isinstance(matrix, list):
            return
        try:
            k = tuple(float(value) for value in matrix)
        except (TypeError, ValueError):
            return
        if len(k) != 9 or k[0] == 0.0:
            self._camera_calibration_snapshot = None
            if self._camera is not None:
                self._camera["calibrated"] = False
                self._camera["calibration_snapshot"] = None
            return
        try:
            seed_value = (
                str(message.get("distortion_model", "")),
                tuple(float(value) for value in message.get("d", [])),
                k,
                tuple(float(value) for value in message.get("r", [])),
                tuple(float(value) for value in message.get("p", [])),
                int(message.get("width", 0)),
                int(message.get("height", 0)),
            )
        except (TypeError, ValueError):
            return
        self._camera_calibration_snapshot = hashlib.sha256(
            repr(seed_value).encode("utf-8")
        ).hexdigest()
        if self._camera is not None:
            self._camera["calibrated"] = True
            self._camera["calibration_snapshot"] = self._camera_calibration_snapshot

    def _update_tf(self, message: Mapping[str, Any], observed: float) -> None:
        transforms = message.get("transforms")
        if not isinstance(transforms, list):
            return
        map_frame = os.getenv("FLYTO_ROS2_MAP_FRAME", "map")
        odom_frame = os.getenv("FLYTO_ROS2_ODOM_FRAME", "odom")
        for item in transforms:
            header = item.get("header") if isinstance(item, Mapping) else None
            if not isinstance(header, Mapping):
                continue
            if (
                header.get("frame_id") == map_frame
                and item.get("child_frame_id") == odom_frame
            ):
                self._map_tf_seen_at = observed
                self._condition.notify_all()
                return

    def _handle_publish(self, topic: str, raw_message: Any) -> None:
        if not isinstance(raw_message, Mapping):
            return
        message = dict(raw_message)
        observed = time.monotonic()
        handlers = {
            os.getenv(
                "FLYTO_ROS2_ODOM_TOPIC",
                "/odom",
            ): lambda: self._update_odometry(message, observed),
            os.getenv(
                "FLYTO_ROS2_SCAN_TOPIC",
                "/scan",
            ): lambda: self._update_scan(message, observed),
            os.getenv(
                "FLYTO_ROS2_CAMERA_TOPIC",
                "/camera/image_raw",
            ): lambda: self._update_camera(message, observed),
            os.getenv(
                "FLYTO_ROS2_CAMERA_INFO_TOPIC",
                "/camera/camera_info",
            ): lambda: self._update_camera_info(message),
            os.getenv(
                "FLYTO_ROS2_TF_TOPIC",
                "/tf",
            ): lambda: self._update_tf(message, observed),
        }
        handler = handlers.get(topic)
        if handler is None:
            return
        with self._condition:
            handler()

    def _wait_for(
        self,
        store: dict[str, dict[str, Any]],
        identifier: str,
        deadline: float,
    ) -> dict[str, Any] | None:
        with self._condition:
            while identifier not in store and self._connected:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return None
                self._condition.wait(timeout=remaining)
            return store.pop(identifier, None)

    def _service(
        self,
        service: str,
        args: Mapping[str, Any],
        *,
        timeout: float = 5.0,
    ) -> dict[str, Any]:
        identifier = f"svc-{time.monotonic_ns()}"
        self._send(
            {
                "op": "call_service",
                "service": service,
                "args": dict(args),
                "id": identifier,
            }
        )
        response = self._wait_for(
            self._responses,
            identifier,
            time.monotonic() + timeout,
        )
        if response is None:
            raise RuntimeError(f"rosbridge service timed out: {service}")
        if response.get("result") is not True:
            raise RuntimeError(
                f"rosbridge service failed: {service}: {response.get('values')}"
            )
        values = response.get("values")
        if not isinstance(values, Mapping):
            raise RuntimeError(f"rosbridge service response is invalid: {service}")
        return dict(values)

    def _subscribe_standard_observations(self) -> None:
        subscriptions = (
            (
                os.getenv("FLYTO_ROS2_ODOM_TOPIC", "/odom"),
                "nav_msgs/msg/Odometry",
                "reliable",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_SCAN_TOPIC", "/scan"),
                "sensor_msgs/msg/LaserScan",
                "best_effort",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_CAMERA_TOPIC", "/camera/image_raw"),
                "sensor_msgs/msg/Image",
                "best_effort",
                100,
            ),
            (
                os.getenv(
                    "FLYTO_ROS2_CAMERA_INFO_TOPIC",
                    "/camera/camera_info",
                ),
                "sensor_msgs/msg/CameraInfo",
                "reliable",
                0,
            ),
            (
                os.getenv("FLYTO_ROS2_TF_TOPIC", "/tf"),
                "tf2_msgs/msg/TFMessage",
                "best_effort",
                0,
            ),
        )
        for index, (topic, message_type, reliability, throttle_rate) in enumerate(
            subscriptions
        ):
            self._send(
                {
                    "op": "subscribe",
                    "id": f"observation-{index}",
                    "topic": topic,
                    "type": message_type,
                    "qos": {
                        "history": "keep_last",
                        "depth": 5,
                        "reliability": reliability,
                        "durability": "volatile",
                    },
                    "throttle_rate": throttle_rate,
                }
            )

    def discover(self) -> Sequence[StandardInterface]:
        if not self._connected:
            return ()
        topics = self._service("/rosapi/topics", {})
        topic_names = topics.get("topics")
        topic_types = topics.get("types")
        if not isinstance(topic_names, list) or not isinstance(topic_types, list):
            raise RuntimeError("rosapi topics response is invalid")
        self._topic_types = {
            str(name): str(message_type)
            for name, message_type in zip(topic_names, topic_types, strict=False)
        }
        actions = self._service("/rosapi/action_servers", {})
        raw_actions = actions.get("action_servers")
        if not isinstance(raw_actions, list):
            raise RuntimeError("rosapi action server response is invalid")
        action_names = {str(name) for name in raw_actions}

        found: list[StandardInterface] = []
        for _capability_id, (kind, name, interface_type) in DEFAULT_INTERFACES.items():
            if kind == "action":
                if name in action_names:
                    found.append(StandardInterface(kind, name, interface_type))
                continue
            topic_type = self._topic_types.get(name)
            if topic_type in CMD_VEL_TYPES:
                found.append(StandardInterface("topic", name, topic_type))
        return found

    def observation(self) -> Mapping[str, Any]:
        deadline = time.monotonic() + 0.5
        with self._condition:
            while (
                self._connected
                and self._pose_seen_at is None
                and time.monotonic() < deadline
            ):
                self._condition.wait(timeout=deadline - time.monotonic())
        now = time.monotonic()
        max_age = max(
            0.1,
            float(os.getenv("FLYTO_ROS2_OBSERVATION_MAX_AGE_SECONDS", "5")),
        )

        def fresh(seen_at: float | None) -> bool:
            return seen_at is not None and now - seen_at <= max_age

        return {
            "pose": (
                dict(self._pose)
                if self._pose is not None and fresh(self._pose_seen_at)
                else None
            ),
            "range": (
                {
                    "minimum_range_m": self._minimum_range,
                    "sample_count": self._range_sample_count,
                }
                if self._minimum_range is not None and fresh(self._range_seen_at)
                else None
            ),
            "camera": (
                dict(self._camera)
                if self._camera is not None and fresh(self._camera_seen_at)
                else None
            ),
            "map_tf_available": fresh(self._map_tf_seen_at),
        }

    @staticmethod
    def _duration(seconds: float) -> dict[str, int]:
        bounded = max(0.0, seconds)
        whole = int(bounded)
        return {
            "sec": whole,
            "nanosec": int((bounded - whole) * 1_000_000_000),
        }

    def _action_goal(
        self,
        capability_id: str,
        arguments: Mapping[str, Any],
    ) -> dict[str, Any]:
        allowance = max(
            1.0,
            float(os.getenv("FLYTO_ROS2_ACTION_ALLOWANCE_SECONDS", "30")),
        )
        if capability_id == "motion.navigate":
            yaw = float(arguments.get("yaw_radians", 0.0))
            return {
                "pose": {
                    "header": {
                        "frame_id": os.getenv("FLYTO_ROS2_MAP_FRAME", "map"),
                    },
                    "pose": {
                        "position": {
                            "x": float(arguments["x"]),
                            "y": float(arguments["y"]),
                            "z": 0.0,
                        },
                        "orientation": {
                            "x": 0.0,
                            "y": 0.0,
                            "z": math.sin(yaw / 2.0),
                            "w": math.cos(yaw / 2.0),
                        },
                    },
                },
                "behavior_tree": "",
            }
        if capability_id == "motion.rotate":
            return {
                "target_yaw": float(arguments["yaw_radians"]),
                "time_allowance": self._duration(allowance),
            }
        distance = float(arguments["distance_m"])
        speed = float(
            arguments.get(
                "speed_mps",
                0.10 if capability_id == "motion.retreat" else 0.12,
            )
        )
        return {
            "target": {"x": distance, "y": 0.0, "z": 0.0},
            "speed": speed,
            "time_allowance": self._duration(allowance),
        }

    def _evidence(self, capability_id: str) -> dict[str, Any]:
        kind, name, interface_type = DEFAULT_INTERFACES[capability_id]
        evidence: dict[str, Any] = {
            "adapter": "generic_ros2",
            "transport": "rosbridge",
            "standard_interface": {
                "kind": kind,
                "name": name,
                "type": interface_type,
            },
        }
        observation = self.observation()
        if observation.get("pose") is not None:
            evidence["odom"] = dict(observation["pose"])
        range_observation = observation.get("range")
        if isinstance(range_observation, Mapping):
            evidence["minimum_range_m"] = range_observation["minimum_range_m"]
        return evidence

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ) -> CallResult:
        if not self._connected:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="rosbridge adapter disconnected",
            )
        if call_id in self._results:
            return self._results[call_id]
        if call_id not in self._active_actions:

            _, action_name, action_type = DEFAULT_INTERFACES[capability_id]
            try:
                self._send(
                    {
                        "op": "send_action_goal",
                        "id": call_id,
                        "action": action_name,
                        "action_type": action_type,
                        "args": self._action_goal(capability_id, arguments),
                        "feedback": True,
                    }
                )
            except RuntimeError as error:
                return CallResult(call_id, OUTCOME_REFUSED, detail=str(error))
            self._active_actions[call_id] = action_name
            self._counts[call_id] = self._counts.get(call_id, 0) + 1

        result_message = self._wait_for(
            self._action_results,
            call_id,
            time.monotonic() + deadline_seconds,
        )
        if result_message is None:
            return CallResult(
                call_id,
                OUTCOME_TIMEOUT,
                evidence=self._evidence(capability_id),
                detail="ROS 2 action still running",
            )

        self._active_actions.pop(call_id, None)
        status = int(result_message.get("status", 0))
        if result_message.get("result") is True and status == 4:
            result = CallResult(
                call_id,
                OUTCOME_COMPLETED,
                evidence=self._evidence(capability_id),
            )
        elif status == 5:
            result = CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        else:
            result = CallResult(
                call_id,
                OUTCOME_FAILED,
                detail=f"ROS 2 action status {status}",
            )
        self._results[call_id] = result
        return result

    def _publish_zero(self) -> bool:
        topic = DEFAULT_INTERFACES["motion.halt"][1]
        topic_type = self._topic_types.get(topic)
        if topic_type not in CMD_VEL_TYPES:
            try:
                self.discover()
            except RuntimeError:
                return False
            topic_type = self._topic_types.get(topic)
        if topic_type not in CMD_VEL_TYPES:
            return False
        key = (topic, topic_type)
        if key not in self._advertised_topics:
            self._send(
                {
                    "op": "advertise",
                    "topic": topic,
                    "type": topic_type,
                    "qos": {
                        "history": "keep_last",
                        "depth": 1,
                        "reliability": "reliable",
                        "durability": "volatile",
                    },
                }
            )
            self._advertised_topics.add(key)
        if topic_type == "geometry_msgs/msg/TwistStamped":
            message: dict[str, Any] = {
                "header": {
                    "frame_id": os.getenv("FLYTO_ROS2_BASE_FRAME", "base_link"),
                },
                "twist": {
                    "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                    "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
            }
        else:
            message = {
                "linear": {"x": 0.0, "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": 0.0},
            }
        self._send({"op": "publish", "topic": topic, "msg": message})
        return True

    def cancel(self, call_id: str) -> CallResult:
        action_name = self._active_actions.get(call_id)
        if action_name is None:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="no active ROS 2 goal",
            )
        self._send(
            {
                "op": "cancel_action_goal",
                "id": call_id,
                "action": action_name,
            }
        )
        stopped = self._publish_zero()
        result_message = self._wait_for(
            self._action_results,
            call_id,
            time.monotonic() + 3.0,
        )
        if (
            result_message is None
            or int(result_message.get("status", 0)) != 5
            or not stopped
        ):
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                detail="ROS 2 cancel/stop was not confirmed",
            )
        self._active_actions.pop(call_id, None)
        result = CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")
        self._results[call_id] = result
        return result

    def safe_stop(self, call_id: str) -> CallResult:
        for active_call in tuple(self._active_actions):
            self.cancel(active_call)
        if not self._publish_zero():
            return CallResult(
                call_id,
                OUTCOME_FAILED,
                detail="standard cmd_vel stop unavailable",
            )
        self._counts[call_id] = self._counts.get(call_id, 0) + 1
        result = CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "adapter": "generic_ros2",
                "transport": "rosbridge",
                "standard_interface": {
                    "kind": "topic",
                    "name": DEFAULT_INTERFACES["motion.halt"][1],
                    "type": "geometry_msgs/msg/Twist|TwistStamped",
                },
                "zero_velocity_published": True,
            },
        )
        self._results[call_id] = result
        return result

    def execution_count(self, call_id: str) -> int:
        return self._counts.get(call_id, 0)

    def disconnect(self) -> None:
        with self._condition:
            websocket = self._ws
            self._connected = False
            self._ws = None
            self._condition.notify_all()
        if websocket is not None:
            with contextlib.suppress(Exception):
                websocket.close()

    def reconnect(self) -> None:
        self.disconnect()
        websocket = self._connection_factory(self._url)
        with self._condition:
            self._ws = websocket
            self._connected = True
            self._responses.clear()
            self._action_results.clear()
            self._advertised_topics.clear()
        self._reader = threading.Thread(
            target=self._reader_loop,
            args=(websocket,),
            name="rosbridge-reader",
            daemon=True,
        )
        self._reader.start()
        self._subscribe_standard_observations()


def build(resource_id: str = "") -> GenericROS2Adapter:
    """Build the host-side adapter for one commanded ROS 2 resource."""
    transport = os.getenv("FLYTO_ROS2_TRANSPORT", "rclpy").strip().lower()
    if transport == "rclpy":
        backend: ROS2Backend = RclpyROS2Backend()
    elif transport == "rosbridge":
        backend = RosbridgeROS2Backend()
    else:
        raise RuntimeError(f"unsupported ROS 2 transport: {transport}")
    return GenericROS2Adapter(backend=backend, resource_id=resource_id)

