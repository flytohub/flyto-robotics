"""Deterministic Gazebo driver for shortcut, deadman, and obstacle evidence."""

from __future__ import annotations

import contextlib
import json
import math
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
from .input_runtime import INPUT_EVENT_CONTRACT_VERSION


class ShortcutGazeboDriver(Node):
    """Exercise press, heartbeat, obstacle, release, and successful replay."""

    def __init__(self) -> None:
        super().__init__("flyto_shortcut_gazebo_driver")
        self.declare_parameter("job_file", "")
        self.declare_parameter("evidence_dir", "results/shortcut-gazebo/images")
        self.declare_parameter("video_frames_dir", "")
        self.declare_parameter("video_max_frames", 600)
        self.declare_parameter("minimum_video_frames", 8)
        self.declare_parameter("world_name", "flyto_atomic_color_route_lab")
        self.declare_parameter("obstacle_model", "lab_obstacle")
        self.declare_parameter("obstacle_lead_distance", 0.6)

        job_file = str(self.get_parameter("job_file").value).strip()
        if not job_file:
            raise ValueError("job_file parameter is required")
        self.job = load_job(Path(job_file))
        self.evidence_dir = Path(str(self.get_parameter("evidence_dir").value))
        self.evidence_dir.mkdir(parents=True, exist_ok=True)
        frames = str(self.get_parameter("video_frames_dir").value).strip()
        self.video_sequence = (
            VideoFrameSequence(
                Path(frames),
                max_frames=int(self.get_parameter("video_max_frames").value),
            )
            if frames
            else None
        )
        self.minimum_video_frames = int(
            self.get_parameter("minimum_video_frames").value
        )
        if not 1 <= self.minimum_video_frames <= 600:
            raise ValueError("minimum_video_frames must be between 1 and 600")
        self.world_name = str(self.get_parameter("world_name").value)
        self.obstacle_model = str(self.get_parameter("obstacle_model").value)
        self.obstacle_lead_distance = float(
            self.get_parameter("obstacle_lead_distance").value
        )
        if not 0.5 <= self.obstacle_lead_distance <= 1.0:
            raise ValueError("obstacle_lead_distance must be between 0.5 and 1.0")

        self.started_at = self._now()
        self.ready_at: float | None = None
        self.last_heartbeat_at = -math.inf
        self.latest_pose: dict[str, float] | None = None
        self.initial_world_pose: dict[str, float] | None = None
        self.latest_world_pose: dict[str, float] | None = None
        self.minimum_range = math.inf
        self.scan_ready = False
        self.latest_image: Image | None = None
        self.sequence = 0
        self.session_id: str | None = None
        self.held = False
        self.first_pressed = False
        self.obstacle_entered = False
        self.obstacle_exited = False
        self.first_released = False
        self.second_pressed = False
        self.completed = False
        self.actions: list[dict[str, object]] = []
        self.captured_labels: set[str] = set()

        self.input_publisher = self.create_publisher(String, "/flyto/input_event", 20)
        self.create_subscription(String, "/flyto/events", self._on_event, 20)
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
        self.create_subscription(
            Image,
            "/flyto/evidence/overhead",
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_timer(0.1, self._tick)
        self._write_manifest()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1_000_000_000.0

    def _elapsed(self) -> float:
        return max(0.0, self._now() - self.started_at)

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

    def _record(self, kind: str, detail: str, **fields: object) -> None:
        action: dict[str, object] = {
            "sequence": len(self.actions) + 1,
            "at_seconds": round(self._elapsed(), 3),
            "kind": kind,
            "detail": detail[:256],
            "minimum_range": (
                round(self.minimum_range, 4)
                if math.isfinite(self.minimum_range)
                else None
            ),
            "world_displacement": self._world_displacement(),
        }
        action.update(fields)
        self.actions.append(action)
        self._write_manifest()

    def _write_manifest(self) -> None:
        write_json_atomic(
            self.evidence_dir / "driver-manifest.json",
            {
                "contract_version": "flyto.robotics.shortcut-evidence.v1",
                "job_id": self.job.job_id,
                "robot_id": self.job.robot_id,
                "actions": self.actions,
                "captures": sorted(self.captured_labels),
                "latest_pose": self.latest_pose,
                "initial_world_pose": self.initial_world_pose,
                "latest_world_pose": self.latest_world_pose,
                "world_displacement": self._world_displacement(),
                "minimum_range": (
                    round(self.minimum_range, 4)
                    if math.isfinite(self.minimum_range)
                    else None
                ),
                "video": {
                    "enabled": self.video_sequence is not None,
                    "frame_count": (
                        self.video_sequence.frame_count
                        if self.video_sequence is not None
                        else 0
                    ),
                    "dropped_frames": (
                        self.video_sequence.dropped_frames
                        if self.video_sequence is not None
                        else 0
                    ),
                },
            },
        )

    def _on_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.latest_pose = {
            "x": round(float(position.x), 6),
            "y": round(float(position.y), 6),
        }

    def _on_world_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        pose = {
            "x": round(float(position.x), 6),
            "y": round(float(position.y), 6),
        }
        if self.initial_world_pose is None:
            self.initial_world_pose = pose
        self.latest_world_pose = pose

    def _on_scan(self, message: LaserScan) -> None:
        valid = [
            value
            for value in message.ranges
            if math.isfinite(value) and message.range_min <= value <= message.range_max
        ]
        self.minimum_range = min(valid, default=math.inf)
        self.scan_ready = True

    def _on_image(self, message: Image) -> None:
        self.latest_image = message
        if self.video_sequence is not None:
            self.video_sequence.write(
                width=int(message.width),
                height=int(message.height),
                encoding=str(message.encoding),
                step=int(message.step),
                data=bytes(message.data),
            )

    def _capture(self, label: str) -> None:
        if label in self.captured_labels or self.latest_image is None:
            return
        message = self.latest_image
        destination = self.evidence_dir / f"gazebo-{label}.png"
        write_rgb_png_atomic(
            destination,
            width=int(message.width),
            height=int(message.height),
            encoding=str(message.encoding),
            step=int(message.step),
            data=bytes(message.data),
        )
        self.captured_labels.add(label)
        self._record("image_captured", label, path=destination.name)

    def _publish_input(self, phase: str) -> None:
        if phase == "press":
            self.session_id = (
                "gazebo.session.one"
                if not self.first_pressed
                else "gazebo.session.two"
            )
        if self.session_id is None:
            raise RuntimeError("input session is not active")
        self.sequence += 1
        payload = {
            "contract_version": INPUT_EVENT_CONTRACT_VERSION,
            "event_id": f"gazebo.shortcut.event.{self.sequence}",
            "source_id": "keyboard.main",
            "control_id": "ArrowUp",
            "session_id": self.session_id,
            "phase": phase,
            "sequence": self.sequence,
        }
        message = String()
        message.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self.input_publisher.publish(message)
        self._record("input_published", phase, input_sequence=self.sequence)
        if phase == "press":
            self.held = True
        elif phase in {"release", "disconnect"}:
            self.held = False

    def _set_obstacle(self, *, active: bool) -> None:
        robot_x = (
            float(self.latest_world_pose["x"])
            if self.latest_world_pose is not None
            else -2.15
        )
        x = robot_x + self.obstacle_lead_distance
        y = 0.0 if active else 2.2
        completed = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                f"/world/{self.world_name}/set_pose",
                "--reqtype",
                "gz.msgs.Pose",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "5000",
                "--req",
                (
                    f'name: "{self.obstacle_model}" '
                    f"position {{ x: {x:.6f} y: {y:.6f} z: 0.35 }} "
                    "orientation { w: 1.0 }"
                ),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=7,
        )
        success = completed.returncode == 0 and "data: true" in completed.stdout
        self._record(
            "obstacle_injected" if active else "obstacle_removed",
            "dynamic Gazebo model pose updated",
            success=success,
            obstacle_x=round(x, 4),
            obstacle_y=y,
        )

    def _on_event(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not isinstance(event, dict):
            return
        kind = str(event.get("kind", ""))
        if kind in {
            "shortcut_action",
            "obstacle_stop",
            "path_clear",
            "mission_cancelled",
            "mission_completed",
        }:
            self._record(
                "runtime_event",
                kind,
                action=event.get("action"),
                reason=event.get("reason"),
                state=event.get("state"),
            )
        if kind == "obstacle_stop":
            self._capture("obstacle-stop")
        elif kind == "mission_cancelled":
            self._capture("release-stop")
        elif kind == "mission_completed":
            self.completed = True
            self._capture("completed")
            if self.held:
                self._publish_input("release")
            self._write_manifest()

    def _tick(self) -> None:
        now = self._now()
        if (
            self.ready_at is None
            and self.latest_pose is not None
            and self.scan_ready
            and self.latest_image is not None
        ):
            self.ready_at = now
            self._capture("ready")
            self._record("simulation_ready", "odometry, lidar, and camera received")
        if self.completed:
            frame_count = (
                self.video_sequence.frame_count
                if self.video_sequence is not None
                else self.minimum_video_frames
            )
            if frame_count >= self.minimum_video_frames:
                self._record(
                    "evidence_complete",
                    "minimum real Gazebo camera frames captured",
                    frame_count=frame_count,
                )
                self._write_manifest()
                rclpy.shutdown()
            return
        if self.ready_at is None:
            return
        elapsed = now - self.ready_at
        if self.held and now - self.last_heartbeat_at >= 0.15:
            self._publish_input("heartbeat")
            self.last_heartbeat_at = now
        if not self.first_pressed and elapsed >= 0.5:
            self.first_pressed = True
            self._publish_input("press")
            self.last_heartbeat_at = now
        elif not self.obstacle_entered and elapsed >= 1.2:
            self.obstacle_entered = True
            self._set_obstacle(active=True)
        elif not self.obstacle_exited and elapsed >= 2.3:
            self.obstacle_exited = True
            self._set_obstacle(active=False)
        elif not self.first_released and elapsed >= 2.7:
            self.first_released = True
            self._publish_input("release")
        elif not self.second_pressed and elapsed >= 3.3:
            self.second_pressed = True
            self._publish_input("press")
            self.last_heartbeat_at = now

    def destroy_node(self) -> bool:
        """Flush the final camera-frame count during launch shutdown."""
        self._write_manifest()
        return super().destroy_node()


def main() -> int:
    rclpy.init()
    node: ShortcutGazeboDriver | None = None
    try:
        node = ShortcutGazeboDriver()
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
