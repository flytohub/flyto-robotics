"""ROS 2 adapter for the transport-neutral mission controller."""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .ai_planner import PlanValidationError, compile_workflow, load_plan
from .contracts import JobValidationError, load_job, write_json_atomic
from .human_approval import (
    HumanDecisionAuthenticator,
    HumanDecisionValidationError,
)
from .line_perception import LineScene, detect_line_scene
from .mission import MissionController, Pose2D
from .semantic_map import SemanticLocationStore, SemanticMapValidationError
from .workflow import MissionState, PrimitiveKind


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
    return math.atan2(sin_yaw, cos_yaw)


class MissionNode(Node):
    """Fail-safe ROS wrapper for velocity, odometry, and lidar topics."""

    def __init__(
        self,
        job_path: Path | None = None,
        result_path: Path | None = None,
        plan_path: Path | None = None,
        semantic_map_path: Path | None = None,
        semantic_map_id: str | None = None,
    ) -> None:
        super().__init__("flyto_robotics_mission")
        self.declare_parameter("job_file", "")
        self.declare_parameter("result_file", "results/mission-result.json")
        self.declare_parameter("plan_file", "")
        self.declare_parameter("semantic_map_file", "")
        self.declare_parameter("semantic_map_id", "")
        self.declare_parameter("odometry_timeout_seconds", 1.0)
        self.declare_parameter("sensor_startup_grace_seconds", 5.0)
        self.declare_parameter("gazebo_physics", False)
        self.declare_parameter("obstacle_injected", False)
        self.declare_parameter("human_approval_injected", False)

        parameter_job = str(self.get_parameter("job_file").value).strip()
        if job_path is None and not parameter_job:
            raise JobValidationError("job_file parameter is required")
        configured_job = job_path or Path(parameter_job)
        configured_result = result_path or Path(str(self.get_parameter("result_file").value))
        parameter_plan = str(self.get_parameter("plan_file").value).strip()
        configured_plan = plan_path or (Path(parameter_plan) if parameter_plan else None)

        self.result_path = configured_result
        self.job = load_job(configured_job)
        workflow = None
        semantic_map_store: SemanticLocationStore | None = None
        if configured_plan is not None:
            plan = load_plan(configured_plan)
            if plan.robot_id != self.job.robot_id:
                raise PlanValidationError("plan.robot_id must match job.robot_id")
            semantic_capabilities = {
                "navigate_to_location",
                "save_current_location",
            }
            if any(
                step.capability in semantic_capabilities for step in plan.steps
            ):
                parameter_semantic_map_file = str(
                    self.get_parameter("semantic_map_file").value
                ).strip()
                semantic_map_file = semantic_map_path or (
                    Path(parameter_semantic_map_file)
                    if parameter_semantic_map_file
                    else None
                )
                configured_semantic_map_id = semantic_map_id or str(
                    self.get_parameter("semantic_map_id").value
                ).strip()
                if semantic_map_file is None or not configured_semantic_map_id:
                    raise SemanticMapValidationError(
                        "semantic_map_file and semantic_map_id are required for "
                        "semantic location capabilities"
                    )
                semantic_map_store = SemanticLocationStore(
                    semantic_map_file,
                    map_id=configured_semantic_map_id,
                )
            workflow = compile_workflow(plan, semantic_map=semantic_map_store)
        self.controller = MissionController(
            self.job,
            workflow=workflow,
            semantic_map_store=semantic_map_store,
            started_at=self._now(),
        )
        self.requires_camera = any(
            step.kind == PrimitiveKind.FOLLOW_LINE for step in self.controller.workflow.steps
        )
        self.requires_human_approval = any(
            step.kind == PrimitiveKind.ASK_HUMAN
            for step in self.controller.workflow.steps
        )
        self.human_decision_authenticator: HumanDecisionAuthenticator | None = None
        if self.requires_human_approval:
            approval_secret = os.environ.get(
                "FLYTO_ROBOTICS_APPROVAL_SECRET",
                "",
            )
            if not approval_secret:
                raise HumanDecisionValidationError(
                    "FLYTO_ROBOTICS_APPROVAL_SECRET is required for ask_human"
                )
            self.human_decision_authenticator = HumanDecisionAuthenticator(
                approval_secret
            )
        self.started_at = self._now()
        self.last_pose: Pose2D | None = None
        self.odometry_diagnostic_logged = False
        self.last_odometry_at: float | None = None
        self.last_scan_at: float | None = None
        self.last_image_at: float | None = None
        self.line_scene: LineScene | None = None
        self.camera_diagnostic_logged = False
        self.last_visible_colors: tuple[str, ...] = ()
        self.minimum_range = math.inf
        self.result_written = False
        self.exit_code = 3
        self.published_event_count = 0

        self.command_publisher = self.create_publisher(Twist, "/flyto/cmd_vel", 10)
        self.event_publisher = self.create_publisher(String, "/flyto/events", 20)
        self.create_subscription(Odometry, "/flyto/odom", self._on_odometry, 10)
        self.create_subscription(
            LaserScan,
            "/flyto/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/flyto/camera/image",
            self._on_image,
            qos_profile_sensor_data,
        )
        if self.requires_human_approval:
            self.create_subscription(
                String,
                "/flyto/human_decision",
                self._on_human_decision,
                10,
            )
        self.create_timer(0.1, self._control_tick)
        self.get_logger().info(
            f"accepted robotics job {self.job.job_id} for robot {self.job.robot_id}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.last_pose = Pose2D(position.x, position.y, _yaw_from_odometry(message))
        if not self.odometry_diagnostic_logged:
            self.get_logger().info(
                f"first odometry pose: x={position.x:.3f}, y={position.y:.3f}"
            )
            self.odometry_diagnostic_logged = True
        self.last_odometry_at = self._now()

    def _on_scan(self, message: LaserScan) -> None:
        valid = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(valid, default=math.inf)
        self.last_scan_at = self._now()

    def _on_image(self, message: Image) -> None:
        try:
            self.line_scene = detect_line_scene(
                message.data,
                width=message.width,
                height=message.height,
                encoding=message.encoding,
                step=message.step,
            )
        except ValueError as exc:
            self.get_logger().warning(f"camera frame rejected: {exc}")
            return
        if not self.camera_diagnostic_logged:
            visible = [
                (
                    f"{item.color}:confidence={item.confidence:.2f},"
                    f"error={item.lateral_error:.2f},pixels={item.pixel_count}"
                )
                for item in self.line_scene.detections
                if item.visible
            ]
            self.get_logger().info(
                "first camera line observations: " + (", ".join(visible) or "none")
            )
            self.camera_diagnostic_logged = True
        visible_colors = tuple(
            item.color for item in self.line_scene.detections if item.visible
        )
        if visible_colors != self.last_visible_colors:
            self.get_logger().info(
                "camera visible colors changed: "
                + (", ".join(visible_colors) or "none")
            )
            self.last_visible_colors = visible_colors
        self.last_image_at = self._now()

    def _on_human_decision(self, message: String) -> None:
        authenticator = self.human_decision_authenticator
        if authenticator is None:
            self.get_logger().warning(
                "human decision ignored because no approval gate is active"
            )
            return
        try:
            decision = authenticator.verify(
                message.data,
                expected_job_id=self.job.job_id,
                expected_robot_id=self.job.robot_id,
            )
            self.controller.submit_human_decision(
                approval_id=decision.approval_id,
                approved=decision.approved,
                actor_id=decision.actor_id,
                now=self._now(),
            )
        except (HumanDecisionValidationError, ValueError) as exc:
            self.controller.record_human_decision_rejection(
                reason=str(exc),
                now=self._now(),
            )
            self._publish_new_events()
            self.get_logger().warning(f"human decision rejected: {exc}")
            return
        self._publish_new_events()
        self.get_logger().info(
            f"verified human decision {decision.approval_id} "
            f"from actor {decision.actor_id}"
        )

    def _publish_new_events(self) -> None:
        """Publish each structured controller event exactly once on ROS."""
        pending = self.controller.events[self.published_event_count :]
        for event in pending:
            message = String()
            message.data = json.dumps(
                event.to_dict(),
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            self.event_publisher.publish(message)
            self.published_event_count += 1

    def _publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def _finish(self, now: float) -> None:
        if self.result_written:
            return
        self._publish_stop()
        result = self.controller.result(
            generated_at=_timestamp(),
            now=now,
            pose=self.last_pose,
        )
        gazebo_physics = bool(self.get_parameter("gazebo_physics").value)
        result["simulation"] = {
            "mode": "gazebo_ros2" if gazebo_physics else "physical_ros2",
            "gazebo_physics": gazebo_physics,
            "obstacle_injected": bool(
                self.get_parameter("obstacle_injected").value
            ),
            "human_approval_injected": bool(
                self.get_parameter("human_approval_injected").value
            ),
        }
        write_json_atomic(self.result_path, result)
        self._publish_new_events()
        self.result_written = True
        self.exit_code = 0 if self.controller.state == MissionState.COMPLETED else 3
        self.get_logger().info(
            f"mission {self.job.job_id} finished with state {self.controller.state.value}"
        )
        rclpy.shutdown()

    def _control_tick(self) -> None:
        now = self._now()
        grace = float(self.get_parameter("sensor_startup_grace_seconds").value)
        timeout = float(self.get_parameter("odometry_timeout_seconds").value)
        camera_missing = self.requires_camera and self.last_image_at is None
        if (
            self.last_pose is None
            or self.last_odometry_at is None
            or self.last_scan_at is None
            or camera_missing
        ):
            self._publish_stop()
            if now - self.started_at > grace:
                self.controller.fail("required_sensor_not_ready", now)
                self._publish_new_events()
                self._finish(now)
            return
        camera_stale = (
            self.requires_camera
            and self.last_image_at is not None
            and now - self.last_image_at > timeout
        )
        if (
            now - self.last_odometry_at > timeout
            or now - self.last_scan_at > timeout
            or camera_stale
        ):
            self._publish_stop()
            self.controller.fail("required_sensor_stale", now)
            self._publish_new_events()
            self._finish(now)
            return

        command = self.controller.tick(
            self.last_pose,
            minimum_range=self.minimum_range,
            now=now,
            line_scene=self.line_scene,
        )
        self._publish_new_events()
        velocity = Twist()
        velocity.linear.x = command.linear_x
        velocity.angular.z = command.angular_z
        self.command_publisher.publish(velocity)
        if self.controller.terminal:
            self._finish(now)


def run(
    job_path: Path,
    result_path: Path,
    *,
    ros_args: Sequence[str] | None = None,
    plan_path: Path | None = None,
    semantic_map_path: Path | None = None,
    semantic_map_id: str | None = None,
) -> int:
    """Run one ROS mission and return a process-friendly exit code."""
    rclpy.init(args=list(ros_args) if ros_args is not None else None)
    node: MissionNode | None = None
    try:
        node = MissionNode(
            job_path,
            result_path,
            plan_path,
            semantic_map_path,
            semantic_map_id,
        )
        rclpy.spin(node)
        return node.exit_code
    except (
        HumanDecisionValidationError,
        JobValidationError,
        PlanValidationError,
        SemanticMapValidationError,
        OSError,
        ValueError,
    ) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Sequence[str] | None = None) -> int:
    """ROS console-script entry point using launch-provided parameters."""
    rclpy.init(args=list(argv) if argv is not None else None)
    node: MissionNode | None = None
    try:
        node = MissionNode()
        rclpy.spin(node)
        return node.exit_code
    except (
        HumanDecisionValidationError,
        JobValidationError,
        PlanValidationError,
        SemanticMapValidationError,
        OSError,
        ValueError,
    ) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
