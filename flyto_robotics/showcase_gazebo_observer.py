"""ROS 2 observer for real multi-camera Gazebo showcase evidence."""

from __future__ import annotations

import contextlib
import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .contracts import write_json_atomic
from .evidence_image import VideoFrameSequence, write_rgb_png_atomic
from .facility_resources import FacilityResourceCatalog, FacilityResourceRuntime
from .showcase_planning import build_showcase_planning_evidence

MAX_CONFIG_BYTES = 128 * 1024
MAX_MISSION_EVENTS = 256


def _load_json(path: str | Path, field_name: str) -> dict[str, object]:
    source = Path(path)
    try:
        if source.stat().st_size > MAX_CONFIG_BYTES:
            raise ValueError(f"{field_name} exceeds {MAX_CONFIG_BYTES} bytes")
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{field_name} must be readable UTF-8 JSON") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{field_name} must contain a JSON object")
    return value


class ShowcaseGazeboObserver(Node):
    """Join attested planning, physical zones, device handoff, and mission events."""

    def __init__(self) -> None:
        super().__init__("flyto_ai4all_showcase_observer")
        self.declare_parameter("resource_file", "")
        self.declare_parameter("goal_frame_file", "")
        self.declare_parameter("plan_file", "")
        self.declare_parameter("planning_session_file", "")
        self.declare_parameter("robot_id", "flyto-rover-sim-001")
        self.declare_parameter("evidence_dir", "results/ai4all-showcase/facility")
        self.declare_parameter("video_frames_dir", "")
        self.declare_parameter("video_max_frames", 900)
        self.declare_parameter("yellow_zone_x", -0.65)
        self.declare_parameter("purple_zone_x", 0.75)
        self.declare_parameter("fault_enabled", True)
        self.declare_parameter("fault_zone_id", "zone.purple")
        self.declare_parameter(
            "fault_resource_id",
            "camera.corridor.b",
        )

        resource_file = str(self.get_parameter("resource_file").value).strip()
        goal_frame_file = str(self.get_parameter("goal_frame_file").value).strip()
        plan_file = str(self.get_parameter("plan_file").value).strip()
        planning_session_file = str(
            self.get_parameter("planning_session_file").value
        ).strip()
        robot_id = str(self.get_parameter("robot_id").value).strip()
        if not all(
            (
                resource_file,
                goal_frame_file,
                plan_file,
                planning_session_file,
                robot_id,
            )
        ):
            raise ValueError(
                "resource_file, goal_frame_file, plan_file, planning_session_file, "
                "and robot_id are required"
            )
        self.evidence_dir = Path(str(self.get_parameter("evidence_dir").value))
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        frames_dir = str(self.get_parameter("video_frames_dir").value).strip()
        self.video = (
            VideoFrameSequence(
                Path(frames_dir),
                max_frames=int(self.get_parameter("video_max_frames").value),
            )
            if frames_dir
            else None
        )
        self.yellow_zone_x = float(self.get_parameter("yellow_zone_x").value)
        self.purple_zone_x = float(self.get_parameter("purple_zone_x").value)
        self.fault_enabled = bool(self.get_parameter("fault_enabled").value)
        self.fault_zone_id = str(
            self.get_parameter("fault_zone_id").value
        ).strip()
        self.fault_resource_id = str(
            self.get_parameter("fault_resource_id").value
        ).strip()
        if not -5.0 <= self.yellow_zone_x < self.purple_zone_x <= 5.0:
            raise ValueError("zone thresholds must be ordered inside the demo world")
        if self.fault_enabled and not all(
            (self.fault_zone_id, self.fault_resource_id)
        ):
            raise ValueError(
                "enabled fault injection requires zone and resource identifiers"
            )

        resource_payload = _load_json(resource_file, "resource_file")
        goal_frame = _load_json(goal_frame_file, "goal_frame_file")
        executed_plan = _load_json(plan_file, "plan_file")
        planning_session = _load_json(
            planning_session_file,
            "planning_session_file",
        )
        self.catalog = FacilityResourceCatalog.from_mapping(resource_payload)
        self.runtime = FacilityResourceRuntime(self.catalog)
        self.planning = build_showcase_planning_evidence(
            session=planning_session,
            goal_frame=goal_frame,
            executed_plan=executed_plan,
            robot_id=robot_id,
        )
        self.started_at = self._now()
        self.latest_frames: dict[str, Image] = {}
        self.latest_world_x: float | None = None
        self.current_zone = ""
        self.camera_failure_injected = False
        self.mission_events: list[dict[str, object]] = []
        self.captures: list[str] = []
        self.pending_capture: str | None = None

        for resource in self.catalog.resources:
            if resource.device_kind != "camera":
                continue
            topic = "/" + resource.endpoint_id.replace(".", "/")
            self.create_subscription(
                Image,
                topic,
                lambda message, resource_id=resource.resource_id: self._on_image(
                    resource_id, message
                ),
                qos_profile_sensor_data,
            )
        self.create_subscription(
            Odometry,
            "/flyto/ground_truth",
            self._on_world_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(String, "/flyto/events", self._on_mission_event, 20)
        self.create_timer(0.1, self._tick)
        self._write_evidence()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _elapsed(self) -> float:
        return max(0.0, self._now() - self.started_at)

    def _on_image(self, resource_id: str, message: Image) -> None:
        self.latest_frames[resource_id] = message
        self.runtime.observe_frame(resource_id, at_seconds=self._elapsed())
        active = self.runtime.active_by_kind.get("camera")
        if active == resource_id and self.video is not None:
            self.video.write(
                width=int(message.width),
                height=int(message.height),
                encoding=str(message.encoding),
                step=int(message.step),
                data=bytes(message.data),
            )
        if active == resource_id and self.pending_capture is not None:
            label = self.pending_capture
            self.pending_capture = None
            self._capture_frame(label, message)

    def _on_world_odometry(self, message: Odometry) -> None:
        self.latest_world_x = float(message.pose.pose.position.x)

    def _on_mission_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(event, dict):
            return
        kind = str(event.get("kind", ""))
        if kind not in {
            "human_approval_requested",
            "human_approved",
            "human_decision_rejected",
            "mission_completed",
            "obstacle_stop",
            "path_clear",
            "primitive_completed",
            "primitive_started",
            "resume_authorized",
        }:
            return
        if len(self.mission_events) < MAX_MISSION_EVENTS:
            self.mission_events.append(
                {
                    "sequence": len(self.mission_events) + 1,
                    "at_seconds": round(self._elapsed(), 3),
                    "kind": kind,
                    "step_id": event.get("step_id"),
                    "state": event.get("state"),
                }
            )
        if kind == "obstacle_stop":
            self.pending_capture = "obstacle-stop"
        elif kind == "human_approval_requested":
            self.pending_capture = "approval-requested"
        elif kind == "mission_completed":
            self.runtime.handoff(
                device_kind="speaker",
                zone_id="zone.purple",
                at_seconds=self._elapsed(),
                reason="delivery_completed",
            )
            self._capture_active("mission-completed")
        self._write_evidence()

    def _tick(self) -> None:
        if self.latest_world_x is None or not self.latest_frames:
            return
        zone = self._zone_for(self.latest_world_x)
        if zone != self.current_zone:
            self.current_zone = zone
            self.runtime.handoff(
                device_kind="camera",
                zone_id=zone,
                at_seconds=self._elapsed(),
                reason="robot_entered_zone",
            )
            self.pending_capture = f"handoff-{zone.replace('.', '-')}"
        if (
            self.fault_enabled
            and zone == self.fault_zone_id
            and not self.camera_failure_injected
            and self.runtime.active_by_kind.get("camera")
            == self.fault_resource_id
        ):
            self.camera_failure_injected = True
            self.runtime.set_health(
                self.fault_resource_id,
                healthy=False,
                at_seconds=self._elapsed(),
                reason="showcase_fault_injection",
            )
            self.runtime.handoff(
                device_kind="camera",
                zone_id=zone,
                at_seconds=self._elapsed(),
                reason="active_camera_unhealthy",
            )
            self.pending_capture = "camera-fallback"
        self._write_evidence()

    def _zone_for(self, world_x: float) -> str:
        if world_x < self.yellow_zone_x:
            return "zone.blue"
        if world_x < self.purple_zone_x:
            return "zone.yellow"
        return "zone.purple"

    def _capture_active(self, label: str) -> None:
        resource_id = self.runtime.active_by_kind.get("camera")
        if resource_id is None:
            return
        frame = self.latest_frames.get(resource_id)
        if frame is None:
            return
        self._capture_frame(label, frame)

    def _capture_frame(self, label: str, frame: Image) -> None:
        destination = self.evidence_dir / f"{label}.png"
        write_rgb_png_atomic(
            destination,
            width=int(frame.width),
            height=int(frame.height),
            encoding=str(frame.encoding),
            step=int(frame.step),
            data=bytes(frame.data),
        )
        if destination.name not in self.captures:
            self.captures.append(destination.name)

    def _write_evidence(self) -> None:
        write_json_atomic(
            self.evidence_dir / "showcase-evidence.json",
            {
                "contract_version": "flyto.robotics.ai4all-showcase-evidence.v1",
                "planning": self.planning,
                "facility": self.runtime.evidence(),
                "mission_events": self.mission_events,
                "latest_world_x": (
                    round(self.latest_world_x, 6)
                    if self.latest_world_x is not None
                    else None
                ),
                "current_zone": self.current_zone or None,
                "camera_failure_injected": self.camera_failure_injected,
                "captures": list(self.captures),
                "video": {
                    "enabled": self.video is not None,
                    "frame_count": self.video.frame_count if self.video else 0,
                    "dropped_frames": self.video.dropped_frames if self.video else 0,
                },
            },
        )

    def destroy_node(self) -> bool:
        """Flush final evidence during launch shutdown."""
        self._write_evidence()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node: ShowcaseGazeboObserver | None = None
    try:
        node = ShowcaseGazeboObserver()
        with contextlib.suppress(KeyboardInterrupt):
            rclpy.spin(node)
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
