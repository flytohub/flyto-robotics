"""ROS 2 adapter for authenticated workflow-card shortcut input."""

from __future__ import annotations

import json
import math
import os
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .contracts import JobValidationError, load_job, write_json_atomic
from .input_gateway import InputGateway, InputGatewayError, QueuedInput
from .input_runtime import (
    InputEvent,
    InputValidationError,
    ShortcutAction,
    ShortcutBinding,
    ShortcutRuntime,
    ValidatedWorkflowCatalog,
    parse_input_event,
)
from .mission import MissionController, Pose2D
from .workflow import MissionState

SHORTCUT_RESULT_CONTRACT_VERSION = "flyto.robotics.shortcut-result.v1"
RUNTIME_EVENT_CONTRACT_VERSION = "flyto.robotics.runtime-event.v1"


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _yaw_from_odometry(message: Odometry) -> float:
    orientation = message.pose.pose.orientation
    sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
    cos_yaw = 1.0 - 2.0 * (orientation.y**2 + orientation.z**2)
    return math.atan2(sin_yaw, cos_yaw)


class ShortcutNode(Node):
    """Fail-safe bridge from input events to validated workflow execution."""

    def __init__(
        self,
        job_path: Path | None = None,
        plan_path: Path | None = None,
        result_path: Path | None = None,
    ) -> None:
        super().__init__("flyto_robotics_shortcut")
        self.declare_parameter("job_file", "")
        self.declare_parameter("plan_file", "")
        self.declare_parameter("result_file", "results/shortcut-result.json")
        self.declare_parameter("binding_id", "binding.forward")
        self.declare_parameter("input_source_id", "keyboard.main")
        self.declare_parameter("input_control_id", "ArrowUp")
        self.declare_parameter("deadman_timeout_seconds", 0.5)
        self.declare_parameter("sensor_timeout_seconds", 1.0)
        self.declare_parameter("sensor_startup_grace_seconds", 5.0)
        self.declare_parameter("gateway_enabled", True)
        self.declare_parameter("gateway_host", "127.0.0.1")
        self.declare_parameter("gateway_port", 8765)
        self.declare_parameter("exit_after_completed_workflows", 0)
        self.declare_parameter("gazebo_physics", False)
        self.declare_parameter("obstacle_injected", False)

        configured_job = job_path or self._required_path("job_file")
        configured_plan = plan_path or self._required_path("plan_file")
        self.result_path = result_path or Path(
            str(self.get_parameter("result_file").value)
        )
        self.job = load_job(configured_job)
        plan_payload = json.loads(configured_plan.read_text(encoding="utf-8"))
        catalog = ValidatedWorkflowCatalog.from_plan_payloads((plan_payload,))
        workflow_id = str(plan_payload.get("plan_id", ""))
        binding = ShortcutBinding(
            binding_id=str(self.get_parameter("binding_id").value),
            source_id=str(self.get_parameter("input_source_id").value),
            control_id=str(self.get_parameter("input_control_id").value),
            workflow_id=workflow_id,
            deadman_timeout_seconds=float(
                self.get_parameter("deadman_timeout_seconds").value
            ),
        )
        self.runtime = ShortcutRuntime(
            self.job,
            catalog=catalog,
            bindings=(binding,),
        )

        self.started_at = self._now()
        self.last_pose: Pose2D | None = None
        self.last_odometry_at: float | None = None
        self.last_scan_at: float | None = None
        self.minimum_range = math.inf
        self.archived_controller_ids: set[int] = set()
        self.missions: list[dict[str, Any]] = []
        self.published_mission_events: dict[int, int] = {}
        self.completed_workflows = 0
        self.result_written = False
        self.gateway: InputGateway | None = None

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
            String,
            "/flyto/input_event",
            self._on_ros_input_event,
            20,
        )
        self.create_timer(0.1, self._control_tick)

        if bool(self.get_parameter("gateway_enabled").value):
            token = os.environ.get("FLYTO_ROBOTICS_INPUT_TOKEN", "")
            self.gateway = InputGateway(
                token=token,
                host=str(self.get_parameter("gateway_host").value),
                port=int(self.get_parameter("gateway_port").value),
            )
            self.gateway.start()
            host, port = self.gateway.address
            self.get_logger().info(f"input gateway listening on {host}:{port}")
        self.get_logger().info(
            f"shortcut {binding.binding_id} accepts {binding.source_id}/"
            f"{binding.control_id} for validated workflow {workflow_id}"
        )

    def _required_path(self, parameter_name: str) -> Path:
        value = str(self.get_parameter(parameter_name).value).strip()
        if not value:
            raise InputValidationError(f"{parameter_name} parameter is required")
        return Path(value)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.last_pose = Pose2D(position.x, position.y, _yaw_from_odometry(message))
        self.last_odometry_at = self._now()

    def _on_scan(self, message: LaserScan) -> None:
        valid = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(valid, default=math.inf)
        self.last_scan_at = self._now()

    def _publish_json(self, payload: dict[str, object]) -> None:
        message = String()
        message.data = json.dumps(
            payload,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
        self.event_publisher.publish(message)

    def _publish_action(self, event: InputEvent, action: ShortcutAction) -> None:
        workflow_id = (
            action.workflow.workflow_id
            if action.workflow is not None
            else (
                self.runtime.controller.workflow.workflow_id
                if self.runtime.controller is not None
                else None
            )
        )
        self._publish_json(
            {
                "contract_version": RUNTIME_EVENT_CONTRACT_VERSION,
                "kind": "shortcut_action",
                "event_id": event.event_id,
                "phase": event.phase.value,
                "action": action.kind,
                "reason": action.reason,
                "binding_id": action.binding_id,
                "workflow_id": workflow_id,
            }
        )

    def _handle_input(
        self,
        event: InputEvent,
        *,
        queued: QueuedInput | None = None,
    ) -> None:
        now = self._now()
        previous = self.runtime.controller
        action = self.runtime.handle_event(event, now=now)
        if previous is not None and self.runtime.controller is not previous:
            self._archive(previous, now=now)
        self._publish_action(event, action)
        controller = self.runtime.controller
        state = controller.state.value if controller is not None else "stopped"
        workflow_id = (
            controller.workflow.workflow_id if controller is not None else None
        )
        if queued is not None:
            queued.acknowledge(
                action=action.kind,
                reason=action.reason,
                workflow_id=workflow_id,
                robot_state=state,
            )

    def _on_ros_input_event(self, message: String) -> None:
        try:
            event = parse_input_event(json.loads(message.data))
            self._handle_input(event)
        except (json.JSONDecodeError, InputValidationError) as exc:
            self.get_logger().warning(f"input event rejected: {exc}")

    def _publish_new_mission_events(self) -> None:
        controller = self.runtime.controller
        if controller is None:
            return
        controller_id = id(controller)
        published = self.published_mission_events.get(controller_id, 0)
        for event in controller.events[published:]:
            payload = event.to_dict()
            payload["contract_version"] = RUNTIME_EVENT_CONTRACT_VERSION
            payload["workflow_id"] = controller.workflow.workflow_id
            self._publish_json(payload)
            published += 1
        self.published_mission_events[controller_id] = published

    def _publish_stop(self) -> None:
        self.command_publisher.publish(Twist())

    def _archive(self, controller: MissionController, *, now: float) -> None:
        controller_id = id(controller)
        if controller_id in self.archived_controller_ids:
            return
        if not controller.terminal:
            return
        self.archived_controller_ids.add(controller_id)
        result = controller.result(
            generated_at=_timestamp(),
            now=now,
            pose=self.last_pose,
        )
        self.missions.append(result)
        if controller.state == MissionState.COMPLETED:
            self.completed_workflows += 1
        self._write_result(now=now)

    def _write_result(self, *, now: float) -> None:
        controller = self.runtime.controller
        final_pose = (
            {
                "x": round(self.last_pose.x, 4),
                "y": round(self.last_pose.y, 4),
                "yaw": round(self.last_pose.yaw, 4),
            }
            if self.last_pose is not None
            else None
        )
        active = controller is not None and not controller.terminal
        status = "running" if active else (
            "succeeded" if self.completed_workflows else "stopped"
        )
        payload: dict[str, Any] = {
            "contract_version": SHORTCUT_RESULT_CONTRACT_VERSION,
            "job_id": self.job.job_id,
            "robot_id": self.job.robot_id,
            "status": status,
            "generated_at": _timestamp(),
            "elapsed_seconds": round(max(0.0, now - self.started_at), 3),
            "completed_workflows": self.completed_workflows,
            "final_pose": final_pose,
            "input_events": [
                {
                    "sequence": item.sequence,
                    "at_seconds": item.at_seconds,
                    "kind": item.kind,
                    "reason": item.reason,
                    "binding_id": item.binding_id,
                    "workflow_id": item.workflow_id,
                }
                for item in self.runtime.events
            ],
            "missions": self.missions,
            "simulation": {
                "mode": (
                    "gazebo_ros2"
                    if bool(self.get_parameter("gazebo_physics").value)
                    else "physical_ros2"
                ),
                "gazebo_physics": bool(
                    self.get_parameter("gazebo_physics").value
                ),
                "obstacle_injected": bool(
                    self.get_parameter("obstacle_injected").value
                ),
            },
        }
        write_json_atomic(self.result_path, payload)
        self.result_written = True

    def _cancel_for_sensor_failure(self, reason: str, *, now: float) -> None:
        controller = self.runtime.controller
        if controller is not None and not controller.terminal:
            controller.cancel_for_safety(now, reason=reason)
        self._publish_stop()
        self._publish_new_mission_events()
        if controller is not None:
            self._archive(controller, now=now)

    def _control_tick(self) -> None:
        now = self._now()
        if self.gateway is not None:
            for queued in self.gateway.drain():
                self._handle_input(queued.event, queued=queued)

        deadman_action = self.runtime.poll(now=now)
        if deadman_action is not None:
            self._publish_json(
                {
                    "contract_version": RUNTIME_EVENT_CONTRACT_VERSION,
                    "kind": "shortcut_action",
                    "action": deadman_action.kind,
                    "reason": deadman_action.reason,
                    "binding_id": deadman_action.binding_id,
                    "workflow_id": (
                        self.runtime.controller.workflow.workflow_id
                        if self.runtime.controller is not None
                        else None
                    ),
                }
            )

        controller = self.runtime.controller
        if controller is None:
            self._publish_stop()
            return
        if controller.terminal:
            self._publish_stop()
            self._publish_new_mission_events()
            self._archive(controller, now=now)
            return

        grace = float(self.get_parameter("sensor_startup_grace_seconds").value)
        timeout = float(self.get_parameter("sensor_timeout_seconds").value)
        if (
            self.last_pose is None
            or self.last_odometry_at is None
            or self.last_scan_at is None
        ):
            self._publish_stop()
            if now - self.started_at > grace:
                self._cancel_for_sensor_failure(
                    "required_sensor_not_ready",
                    now=now,
                )
            return
        if (
            now - self.last_odometry_at > timeout
            or now - self.last_scan_at > timeout
        ):
            self._cancel_for_sensor_failure("required_sensor_stale", now=now)
            return

        command = self.runtime.tick(
            self.last_pose,
            minimum_range=self.minimum_range,
            now=now,
        )
        self._publish_new_mission_events()
        velocity = Twist()
        velocity.linear.x = command.linear_x
        velocity.angular.z = command.angular_z
        self.command_publisher.publish(velocity)
        if controller.terminal:
            self._archive(controller, now=now)
            exit_after = int(
                self.get_parameter("exit_after_completed_workflows").value
            )
            if exit_after > 0 and self.completed_workflows >= exit_after:
                self._publish_stop()
                rclpy.shutdown()

    def destroy_node(self) -> bool:
        now = self._now()
        if rclpy.ok():
            self._publish_stop()
        controller = self.runtime.controller
        if controller is not None:
            if not controller.terminal:
                controller.cancel_for_safety(now, reason="node_shutdown")
            self._archive(controller, now=now)
        if not self.result_written:
            self._write_result(now=now)
        if self.gateway is not None:
            self.gateway.stop()
            self.gateway = None
        return super().destroy_node()


def run(
    job_path: Path,
    plan_path: Path,
    result_path: Path,
    *,
    ros_args: Sequence[str] | None = None,
) -> int:
    """Run the shortcut adapter with explicit files for tests or deployments."""
    rclpy.init(args=list(ros_args) if ros_args is not None else None)
    node: ShortcutNode | None = None
    try:
        node = ShortcutNode(job_path, plan_path, result_path)
        rclpy.spin(node)
        return 0
    except (
        InputGatewayError,
        InputValidationError,
        JobValidationError,
        json.JSONDecodeError,
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
    node: ShortcutNode | None = None
    try:
        node = ShortcutNode()
        rclpy.spin(node)
        return 0
    except KeyboardInterrupt:
        return 0
    except (
        InputGatewayError,
        InputValidationError,
        JobValidationError,
        json.JSONDecodeError,
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
