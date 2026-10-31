"""Real ROS2/Nav2 + Gazebo twin navigation; simulation graph only.

This backend is deliberately separate from the MuJoCo arm and from task policy.
Nav2 goal success is NOT sufficient: the Nav2 goal tolerance is about 0.25 m.
Every leg must independently verify a 5 cm position bound, yaw, quiet odometry,
and the simulator's scan floor; bounded ROS2 motion primitives close the last gap.
"""

from __future__ import annotations

import contextlib
import math
import os
import time
import uuid
from typing import Any

os.environ["FLYTO_ROS2_DEPLOYMENT_MODE"] = "simulation"
os.environ.pop("FLYTO_ROS2_MIN_CLEARANCE_M", None)

from flyto_robotics.adapter_contract import CallRequest
from flyto_robotics.generic_ros2_adapter import GenericROS2Adapter, RosbridgeROS2Backend
from sim.fetch_return.contract import MissionFailure, Pose


def angle_delta(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class GazeboTwinNavigator:
    def __init__(self, *, url: str = "ws://127.0.0.1:19090", ready_timeout_s: float = 65.0):
        self.backend = RosbridgeROS2Backend(url=url)
        self.adapter = GenericROS2Adapter(
            backend=self.backend, resource_id="fetch-return-gazebo-sim"
        )
        self.speed = 0.0
        self.elapsed_s = 0.0
        self.trace: list[dict[str, Any]] = []
        deadline = time.monotonic() + ready_timeout_s
        while time.monotonic() < deadline:
            state = self.adapter.reconnect()
            obs = dict(self.backend.observation())
            try:
                pose = self._observed_pose(obs)
                clearance = self._clearance(obs)
            except (KeyError, TypeError, ValueError):
                time.sleep(1.0)
                continue
            offered = {row.capability_id for row in self.adapter.describe()}
            if (
                state.get("rosbridge_ok")
                and {"motion.navigate", "motion.rotate", "motion.advance"} <= offered
                and clearance is not None
            ):
                self.home = pose
                self.pose = pose
                return
            time.sleep(1.0)
        self.close()
        raise MissionFailure("navigation", "gazebo_ros2_twin_not_ready")

    @staticmethod
    def _observed_pose(obs: dict[str, Any]) -> Pose:
        raw = obs["map_pose"]
        if not isinstance(raw, dict) or raw.get("frame") != "map":
            raise ValueError("unknown_map_frame")
        return Pose(float(raw["x"]), float(raw["y"]), float(raw["yaw"]))

    @staticmethod
    def _clearance(obs: dict[str, Any]) -> float | None:
        raw = (obs.get("range") or {}).get("minimum_range_m")
        return float(raw) if isinstance(raw, (int, float)) and math.isfinite(raw) else None

    def _pose(self) -> Pose:
        return self._observed_pose(dict(self.backend.observation()))

    def _invoke(
        self,
        stage: str,
        capability: str,
        args: dict[str, Any],
        deadline_s: float,
        called: list[dict[str, Any]],
    ) -> None:
        result = self.adapter.invoke(
            CallRequest(
                call_id=f"fetch-sim-{uuid.uuid4().hex}",
                capability_id=capability,
                arguments=args,
                deadline_seconds=deadline_s,
            )
        )
        record = {
            "capability": capability,
            "outcome": result.outcome,
            "reason": (result.evidence or {}).get("motion_outcome", {}).get("reason"),
        }
        called.append(record)
        if result.outcome != "completed":
            try:
                self.adapter.safe_stop()
            finally:
                self.speed = 0
            detail = str(result.detail or (result.evidence or {}).get("reason_code") or "unknown")[
                :180
            ]
            raise MissionFailure(
                stage, f"ros2_{capability.replace('.', '_')}_{result.outcome}: {detail}"
            )

    def go(self, destination: Pose, *, stage: str, sla_s: float = 60) -> dict[str, Any]:
        start = time.monotonic()
        called: list[dict[str, Any]] = []
        self._invoke(
            stage,
            "motion.navigate",
            {"x": destination.x, "y": destination.y, "yaw_radians": destination.yaw},
            min(sla_s, 70),
            called,
        )
        # Nav2 may legitimately say succeeded 20-25 cm from the target. This
        # extra admission gate prevents declaring docked before actual alignment.
        for _ in range(4):
            current = self._pose()
            d = math.hypot(current.x - destination.x, current.y - destination.y)
            if d <= 0.042:
                break
            if d < 0.05:
                break
            bearing = math.atan2(destination.y - current.y, destination.x - current.x)
            turn = angle_delta(bearing - current.yaw)
            if abs(turn) > 0.04:
                self._invoke(stage, "motion.rotate", {"yaw_radians": turn}, 40, called)
                current = self._pose()
                d = math.hypot(current.x - destination.x, current.y - destination.y)
            # Avoid overshoot and the adapter's 5 cm minimum step.
            move = max(0.05, min(0.45, d - 0.015))
            self._invoke(
                stage, "motion.advance", {"distance_m": move, "speed_mps": 0.10}, 45, called
            )
        # The simulator's low-level rotation may overshoot small commands.
        # Refine from fresh observed yaw with conservative half-sized commands;
        # never treat action success as proof of the final heading.
        for _ in range(10):
            current = self._pose()
            yaw_error = angle_delta(destination.yaw - current.yaw)
            if abs(yaw_error) <= 0.07:
                break
            self._invoke(
                stage,
                "motion.rotate",
                {"yaw_radians": max(-0.7, min(0.7, yaw_error * 0.65))},
                40,
                called,
            )
        # Quiet-window verification; a Nav2 action status alone is not proof.
        a = self._pose()
        time.sleep(0.55)
        b = self._pose()
        drift = math.hypot(a.x - b.x, a.y - b.y)
        pos_error = math.hypot(b.x - destination.x, b.y - destination.y)
        yaw_error = abs(angle_delta(destination.yaw - b.yaw))
        elapsed = time.monotonic() - start
        self.pose = b
        self.elapsed_s += elapsed
        self.speed = drift / 0.55
        scan = self._clearance(dict(self.backend.observation()))
        if elapsed > sla_s:
            raise MissionFailure(stage, "navigation_sla_exceeded")
        if pos_error > 0.05 or yaw_error > 0.10 or drift > 0.014:
            raise MissionFailure(
                stage,
                (
                    f"gazebo_docking_not_verified: position={pos_error:.4f}m "
                    f"yaw={yaw_error:.4f}rad quiet_drift={drift:.4f}m"
                ),
            )
        if scan is None or scan < 0.35:
            raise MissionFailure(stage, "gazebo_clearance_floor_violation")
        self.trace.append(
            {
                "stage": stage,
                "from": a.as_dict(),
                "to": b.as_dict(),
                "duration_s": round(elapsed, 3),
                "commands": called,
                "quiet_drift_m": round(drift, 5),
            }
        )
        return {
            "source": "gazebo_ros2_nav2_simulation",
            "frame": "map",
            "pose": b.as_dict(),
            "position_error_m": round(pos_error, 5),
            "yaw_error_rad": round(yaw_error, 5),
            "speed_m_s": round(self.speed, 5),
            "quiet_drift_m": round(drift, 5),
            "minimum_scan_m": round(scan, 3),
            "elapsed_s": round(elapsed, 3),
            "sla_s": sla_s,
            "arrived": True,
            "commands": called,
        }

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self.adapter.safe_stop()
        self.adapter.disconnect()
