"""ROS/Gazebo-only adversarial driver for repeatable evidence collection."""

from __future__ import annotations

import contextlib
import json
import math
import os
import subprocess
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String

from .contracts import load_job, write_json_atomic
from .evidence_image import VideoFrameSequence, write_rgb_png_atomic
from .human_approval import build_signed_human_decision, decision_to_json


class GazeboLabDriver(Node):
    """Inject bounded physical faults and capture audit images for one lab run."""

    def __init__(self) -> None:
        super().__init__("flyto_gazebo_lab_driver")
        self.declare_parameter("job_file", "")
        self.declare_parameter("evidence_dir", "results/gazebo-lab")
        self.declare_parameter("scenario_id", "gazebo.careflow.adversarial.v1")
        self.declare_parameter("world_name", "flyto_atomic_color_route_lab")
        self.declare_parameter("obstacle_model", "lab_obstacle")
        self.declare_parameter("obstacle_enter_seconds", 3.0)
        self.declare_parameter("obstacle_exit_seconds", 6.0)
        self.declare_parameter("robot_world_origin_x", -2.15)
        self.declare_parameter("obstacle_lead_distance", 0.80)
        self.declare_parameter("obstacle_active_y", 0.0)
        self.declare_parameter("obstacle_parked_y", 2.2)
        self.declare_parameter("approval_actor_id", "evaluator.gazebo")
        self.declare_parameter("replay_count", 8)
        self.declare_parameter("video_frames_dir", "")
        self.declare_parameter("video_max_frames", 600)

        job_file = str(self.get_parameter("job_file").value).strip()
        if not job_file:
            raise ValueError("job_file parameter is required")
        self.job = load_job(Path(job_file))
        self.evidence_dir = Path(str(self.get_parameter("evidence_dir").value))
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        self.scenario_id = str(self.get_parameter("scenario_id").value)
        self.world_name = str(self.get_parameter("world_name").value)
        self.obstacle_model = str(self.get_parameter("obstacle_model").value)
        self.obstacle_enter_seconds = float(
            self.get_parameter("obstacle_enter_seconds").value
        )
        self.obstacle_exit_seconds = float(
            self.get_parameter("obstacle_exit_seconds").value
        )
        if not 0 <= self.obstacle_enter_seconds < self.obstacle_exit_seconds <= 120:
            raise ValueError("obstacle injection window is invalid")
        self.robot_world_origin_x = float(
            self.get_parameter("robot_world_origin_x").value
        )
        self.obstacle_lead_distance = float(
            self.get_parameter("obstacle_lead_distance").value
        )
        if not 0.5 <= self.obstacle_lead_distance <= 1.0:
            raise ValueError("obstacle_lead_distance must be between 0.5 and 1.0")
        self.obstacle_active_y = float(
            self.get_parameter("obstacle_active_y").value
        )
        self.obstacle_parked_y = float(
            self.get_parameter("obstacle_parked_y").value
        )
        self.approval_actor_id = str(
            self.get_parameter("approval_actor_id").value
        )
        self.replay_count = int(self.get_parameter("replay_count").value)
        if not 1 <= self.replay_count <= 20:
            raise ValueError("replay_count must be between 1 and 20")
        video_frames_dir = str(
            self.get_parameter("video_frames_dir").value
        ).strip()
        self.video_sequence = (
            VideoFrameSequence(
                Path(video_frames_dir),
                max_frames=int(self.get_parameter("video_max_frames").value),
            )
            if video_frames_dir
            else None
        )
        self.approval_secret = os.environ.get(
            "FLYTO_ROBOTICS_APPROVAL_SECRET", ""
        )
        if len(self.approval_secret.encode("utf-8")) < 32:
            raise ValueError("a runtime-only approval secret is required")

        self.started_at = self._now()
        self.obstacle_entered = False
        self.obstacle_exited = False
        self.approval_payload: str | None = None
        self.replays_sent = 0
        self.next_replay_at = math.inf
        self.latest_image: Image | None = None
        self.latest_pose: dict[str, float] | None = None
        self.initial_world_pose: dict[str, float] | None = None
        self.latest_world_pose: dict[str, float] | None = None
        self.minimum_range = math.inf
        self.captured_labels: set[str] = set()
        self.actions: list[dict[str, object]] = []
        self.pending_capture: str | None = "startup"
        self.pending_capture_not_before = self.started_at
        self.active_obstacle_x = self.robot_world_origin_x + self.obstacle_lead_distance

        self.decision_publisher = self.create_publisher(
            String, "/flyto/human_decision", 10
        )
        self.create_subscription(
            String,
            "/flyto/events",
            self._on_event,
            20,
        )
        self.create_subscription(
            Image,
            "/flyto/evidence/overhead",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_subscription(Odometry, "/flyto/odom", self._on_odometry, 10)
        self.create_subscription(
            Odometry,
            "/flyto/ground_truth",
            self._on_world_odometry,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            LaserScan,
            "/flyto/scan",
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._tick)
        self._write_manifest()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _elapsed(self) -> float:
        return max(0.0, self._now() - self.started_at)

    def _record(self, kind: str, detail: str, **fields: object) -> None:
        action: dict[str, object] = {
            "sequence": len(self.actions) + 1,
            "at_seconds": round(self._elapsed(), 3),
            "kind": kind,
            "detail": detail[:256],
            "latest_pose": self.latest_pose,
            "latest_world_pose": self.latest_world_pose,
            "world_displacement": self._world_displacement(),
            "minimum_range": (
                round(self.minimum_range, 4)
                if math.isfinite(self.minimum_range)
                else None
            ),
        }
        action.update(fields)
        self.actions.append(action)
        self._write_manifest()

    def _write_manifest(self) -> None:
        write_json_atomic(
            self.evidence_dir / "driver-manifest.json",
            {
                "contract_version": "flyto.robotics.lab-driver-evidence.v1",
                "scenario_id": self.scenario_id,
                "job_id": self.job.job_id,
                "robot_id": self.job.robot_id,
                "actions": self.actions,
                "captures": sorted(self.captured_labels),
                "video": {
                    "enabled": self.video_sequence is not None,
                    "frames_directory": (
                        self.video_sequence.directory.name
                        if self.video_sequence is not None
                        else None
                    ),
                    "frame_count": (
                        self.video_sequence.frame_count
                        if self.video_sequence is not None
                        else 0
                    ),
                    "max_frames": (
                        self.video_sequence.max_frames
                        if self.video_sequence is not None
                        else 0
                    ),
                    "dropped_frames": (
                        self.video_sequence.dropped_frames
                        if self.video_sequence is not None
                        else 0
                    ),
                },
                "latest_pose": self.latest_pose,
                "initial_world_pose": self.initial_world_pose,
                "latest_world_pose": self.latest_world_pose,
                "world_displacement": self._world_displacement(),
                "minimum_range": (
                    round(self.minimum_range, 4)
                    if math.isfinite(self.minimum_range)
                    else None
                ),
            },
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.latest_pose = {
            "x": round(float(position.x), 4),
            "y": round(float(position.y), 4),
            "z": round(float(position.z), 4),
        }

    def _on_world_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        pose = {
            "x": round(float(position.x), 6),
            "y": round(float(position.y), 6),
            "z": round(float(position.z), 6),
        }
        if self.initial_world_pose is None:
            self.initial_world_pose = pose
        self.latest_world_pose = pose

    def _world_displacement(self) -> float | None:
        if self.initial_world_pose is None or self.latest_world_pose is None:
            return None
        return round(
            math.hypot(
                self.latest_world_pose["x"] - self.initial_world_pose["x"],
                self.latest_world_pose["y"] - self.initial_world_pose["y"],
            ),
            6,
        )

    def _on_scan(self, message: LaserScan) -> None:
        valid = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(valid, default=math.inf)

    def _on_image(self, message: Image) -> None:
        self.latest_image = message
        if self.video_sequence is not None:
            try:
                written = self.video_sequence.write(
                    width=int(message.width),
                    height=int(message.height),
                    encoding=str(message.encoding),
                    step=int(message.step),
                    data=bytes(message.data),
                )
            except (OSError, ValueError) as exc:
                self._record("video_frame_failed", str(exc))
                self.video_sequence = None
            else:
                if written is not None and self.video_sequence.frame_count % 16 == 0:
                    self._write_manifest()
        if (
            self.pending_capture is not None
            and self._now() >= self.pending_capture_not_before
        ):
            label = self.pending_capture
            self.pending_capture = None
            self._capture(label, message)

    def _capture(self, label: str, message: Image | None = None) -> None:
        if label in self.captured_labels:
            return
        frame = message or self.latest_image
        if frame is None:
            self.pending_capture = label
            return
        destination = self.evidence_dir / (
            f"gazebo-{label}-{self._elapsed():06.2f}.png"
        )
        try:
            write_rgb_png_atomic(
                destination,
                width=int(frame.width),
                height=int(frame.height),
                encoding=str(frame.encoding),
                step=int(frame.step),
                data=bytes(frame.data),
            )
        except (OSError, ValueError) as exc:
            self._record("capture_failed", f"{label}: {exc}")
            return
        self.captured_labels.add(label)
        self._record("image_captured", label, path=destination.name)

    def _set_obstacle_pose(self, *, y: float, action: str) -> bool:
        service = f"/world/{self.world_name}/set_pose"
        request = (
            f'name: "{self.obstacle_model}" '
            f"position {{ x: {self.active_obstacle_x:.6f} y: {y:.6f} z: 0.35 }} "
            "orientation { w: 1.0 }"
        )
        try:
            completed = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    service,
                    "--reqtype",
                    "gz.msgs.Pose",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "5000",
                    "--req",
                    request,
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=7,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            self._record("fault_injection_failed", f"{action}: {exc}")
            return False
        success = completed.returncode == 0 and "data: true" in completed.stdout
        self._record(
            "fault_injection",
            action,
            success=success,
            returncode=completed.returncode,
            obstacle_x=round(self.active_obstacle_x, 4),
            obstacle_y=round(y, 4),
            response=completed.stdout.strip()[:128],
        )
        return success

    def _publish_approval(self, approval_id: str) -> None:
        decision = build_signed_human_decision(
            job_id=self.job.job_id,
            robot_id=self.job.robot_id,
            approval_id=approval_id,
            approved=True,
            actor_id=self.approval_actor_id,
            secret=self.approval_secret,
            ttl_seconds=120,
        )
        self.approval_payload = decision_to_json(decision)
        message = String()
        message.data = self.approval_payload
        self.decision_publisher.publish(message)
        self.next_replay_at = self._elapsed() + 0.25
        self._record(
            "approval_published",
            approval_id,
            actor_id=self.approval_actor_id,
        )
        self.pending_capture = "approval"

    def _on_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            self._record("event_rejected", "event topic contained invalid JSON")
            return
        if not isinstance(event, dict):
            self._record("event_rejected", "event topic payload was not an object")
            return
        kind = event.get("kind")
        if kind == "human_approval_requested" and self.approval_payload is None:
            detail = str(event.get("detail", ""))
            marker = "approval requested for "
            approval_id = detail.split(marker, 1)[1].split(";", 1)[0] if marker in detail else ""
            if approval_id:
                self._publish_approval(approval_id)
        if kind == "mission_completed":
            self._capture("completed")
            self._write_manifest()

    def _tick(self) -> None:
        elapsed = self._elapsed()
        if not self.obstacle_entered and elapsed >= self.obstacle_enter_seconds:
            self.obstacle_entered = True
            if self.latest_pose is not None:
                robot_x = (
                    float(self.latest_world_pose["x"])
                    if self.latest_world_pose is not None
                    else self.robot_world_origin_x + float(self.latest_pose["x"])
                )
                self.active_obstacle_x = robot_x + self.obstacle_lead_distance
            if self._set_obstacle_pose(y=self.obstacle_active_y, action="obstacle_enter"):
                self.pending_capture = "obstacle"
                self.pending_capture_not_before = self._now() + 0.4
        if not self.obstacle_exited and elapsed >= self.obstacle_exit_seconds:
            self.obstacle_exited = True
            self._set_obstacle_pose(y=self.obstacle_parked_y, action="obstacle_exit")
        if (
            self.approval_payload is not None
            and self.replays_sent < self.replay_count
            and elapsed >= self.next_replay_at
        ):
            message = String()
            message.data = self.approval_payload
            self.decision_publisher.publish(message)
            self.replays_sent += 1
            self.next_replay_at = elapsed + 0.15
            self._record(
                "approval_replay_published",
                "reused signed nonce for rejection testing",
                replay_number=self.replays_sent,
            )


def main() -> int:
    """Run the Gazebo lab test driver until the launch system shuts it down."""
    rclpy.init()
    node: GazeboLabDriver | None = None
    try:
        node = GazeboLabDriver()
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
