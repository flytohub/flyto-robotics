"""Offline two-device fetch-and-return co-simulation (no hardware or Cloud credentials).

One bounded natural-language request becomes a linear, independently checked task.
Navigation is a deterministic kinematic simulation, NOT Nav2/Gazebo; manipulation
uses flyto-modules-lerobot's MuJoCo contact model and rendered perception. The
stationary AM-ARM200 and the mobile robot are TWO DIFFERENT resources. The
MuJoCo bin is a docking-bay proxy for the mobile cargo tray while parked;
cargo transport is an independent symbolic simulator, NOT coupled physics.
Each evidence entry explicitly states its simulation source.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

# Direct script execution must be able to import the sibling simulation package.
if __package__ in ("", None):
    sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sim.fetch_return.contract import WORKSPACE_FRAME, MissionFailure, Pose

GOALS = (
    re.compile(
        r"^(?:請)?(?:派)?機器人(?:去|前往)?.*?(?P<color>紅色|红色)積木(?:拿|取|撿|捡|帶|带)(?:回來|回来|回家|回原點)$"
    ),
    re.compile(
        r"^(?:please )?(?:send (?:the )?robot to )?"
        r"(?:fetch|retrieve|bring back) (?:the )?red block(?: and return)?$",
        re.I,
    ),
)
PAYLOAD = "block-red"
CARGO = "bin-blue"
MOBILE_RESOURCE = "turtlebot3-mobile-sim"
ARM_RESOURCE = "am-arm200-fixed-station-sim"
PICKUP_STATION = "station-red"


def accepted_goal(goal: str) -> bool:
    text = " ".join(goal.strip("。！？!? ").split())
    if len(text) > 200:
        return False
    return any(pattern.fullmatch(text) is not None for pattern in GOALS)


class Navigator:
    """Kinematic stop-and-turn simulation with hard clearance and arrival gates."""

    def __init__(self, start: Pose, *, blocked: bool = False):
        self.pose, self.home, self.speed, self.elapsed_s = start, start, 0.0, 0.0
        self.blocked = blocked
        self.trace: list[dict[str, Any]] = []

    def go(self, destination: Pose, *, stage: str, sla_s: float = 25.0) -> dict[str, Any]:
        x, y = self.pose.x, self.pose.y
        distance = math.hypot(destination.x - x, destination.y - y)
        # This represents a conservative stationary obstacle. No motion is commanded.
        if self.blocked and distance > 0.05:
            self.speed = 0.0
            raise MissionFailure(stage, "path_blocked_safe_stop")
        heading = (
            math.atan2(destination.y - y, destination.x - x) if distance > 1e-8 else self.pose.yaw
        )
        turn = abs(math.atan2(math.sin(heading - self.pose.yaw), math.cos(heading - self.pose.yaw)))
        final_turn = abs(
            math.atan2(math.sin(destination.yaw - heading), math.cos(destination.yaw - heading))
        )
        # Bounded primitive: turn, translate, turn; terminal speed is read back as zero.
        duration = (turn + final_turn) / 0.8 + distance / 0.16 + 0.5
        if duration > sla_s:
            self.speed = 0.0
            raise MissionFailure(stage, "navigation_sla_exceeded")
        self.elapsed_s += duration
        self.pose = destination
        self.speed = 0.0
        self.trace.append(
            {
                "stage": stage,
                "from": {"x": x, "y": y},
                "to": destination.as_dict(),
                "duration_s": round(duration, 3),
                "speed_m_s": self.speed,
            }
        )
        position_error = math.hypot(self.pose.x - destination.x, self.pose.y - destination.y)
        heading_error = abs(
            math.atan2(
                math.sin(self.pose.yaw - destination.yaw), math.cos(self.pose.yaw - destination.yaw)
            )
        )
        if position_error > 0.05 or heading_error > 0.08 or self.speed != 0.0:
            raise MissionFailure(stage, "arrival_not_verified")
        return {
            "source": "kinematic_simulation",
            "frame": WORKSPACE_FRAME,
            "pose": self.pose.as_dict(),
            "position_error_m": position_error,
            "yaw_error_rad": heading_error,
            "speed_m_s": self.speed,
            "elapsed_s": round(duration, 3),
            "sla_s": sla_s,
            "arrived": True,
        }

    def close(self) -> None:
        pass


class WorldPrediction:
    """Timestamped perception records; a prediction is never used as a pick target."""

    def __init__(self):
        self.records: list[tuple[float, float, float]] = []

    def observe(self, obj: dict[str, Any], at_s: float) -> None:
        if obj.get("source") != "simulation" or not isinstance(obj.get("pose"), dict):
            raise MissionFailure("perception", "object_not_observed_by_camera")
        pose = obj["pose"]
        self.records.append((float(at_s), float(pose["x"]), float(pose["y"])))
        self.records = self.records[-2:]

    def predict(self, at_s: float) -> dict[str, Any]:
        if not self.records:
            raise MissionFailure("perception", "no_sourced_world_observation")
        last_t, x, y = self.records[-1]
        age = at_s - last_t
        if age < 0 or age > 5.0:
            raise MissionFailure("perception", "world_observation_stale")
        if len(self.records) == 2:
            t0, x0, y0 = self.records[0]
            delta = max(1e-5, last_t - t0)
            vx, vy = (x - x0) / delta, (y - y0) / delta
        else:
            vx, vy = 0.0, 0.0
        return {
            "source": "prediction_from_sourced_camera_observations",
            "model": "constant_velocity",
            "x": round(x + vx * age, 5),
            "y": round(y + vy * age, 5),
            "age_s": round(age, 3),
            "uncertainty_m": round(0.01 + 0.02 * age, 4),
            "admission": "never_act_without_fresh_visual_reacquisition",
        }


class ArmRig:
    """Use exactly the LeRobot adapter and MuJoCo world employed by its real tests."""

    def __init__(self, config_path: Path):
        from flyto_modules_lerobot import mujoco_sim, simulation
        from flyto_modules_lerobot.arm_config import parse_arm_config
        from flyto_modules_lerobot.driver import ArmSession
        from flyto_modules_lerobot.lerobot_adapter import LeRobotArmAdapter

        data = json.loads(config_path.read_text(encoding="utf-8"))
        self.config = parse_arm_config(data)
        self.clock = simulation.VirtualClock()
        self.world = mujoco_sim.install_world(
            self.config,
            mujoco_sim.MujocoWorld(self.config, clock=self.clock.now, sleep=self.clock.sleep),
        )
        self.adapter = LeRobotArmAdapter(
            session=ArmSession(self.config),
            resource_id=self.config.resource_id,
            served_resource_id=self.config.resource_id,
            sleep=self.clock.sleep,
            clock=self.clock.now,
        )
        self._number = 0

    def call(self, capability: str, args: dict[str, Any] | None = None):
        from flyto_modules_lerobot.adapter_contract import CallRequest

        self._number += 1
        return self.adapter.invoke(
            CallRequest(f"fetch-{self._number}", capability, args or {}, 120.0)
        )

    def find(self, label: str) -> dict[str, Any]:
        looked = self.call("perception.locate_objects")
        if looked.outcome != "completed":
            raise MissionFailure("perception", "camera_unavailable")
        found = [
            row
            for row in looked.evidence.get("objects", ())
            if row.get("label", "").casefold() == label.casefold()
            or (label == "red block" and row.get("id") == "red-1")
        ]
        if len(found) != 1:
            raise MissionFailure("perception", "red_block_missing_or_ambiguous")
        return found[0]

    def inject_block_displacement(self, x: float, y: float, z: float) -> None:
        """Move the simulated physical object for a test fault, never invent a verdict."""
        import mujoco

        joint_idx = int(self.world.model.body(PAYLOAD).jntadr[0])
        qpos_adr = int(self.world.model.joint(joint_idx).qposadr[0])
        self.world.data.qpos[qpos_adr : qpos_adr + 3] = [x, y, z]
        mujoco.mj_forward(self.world.model, self.world.data)

    def independent_cargo_check(self) -> dict[str, Any]:
        block = self.world.data.body(PAYLOAD).xpos
        bin_ = self.world.data.body(CARGO).xpos
        dx, dy = float(block[0] - bin_[0]), float(block[1] - bin_[1])
        dz = float(block[2])
        in_cargo = math.hypot(dx, dy) < 0.045 and 0.0 < dz < 0.045
        return {
            "source": "mujoco_contact_physics_ground_truth",
            "in_cargo": bool(in_cargo),
            "xy_error_m": round(math.hypot(dx, dy), 5),
            "z_m": round(dz, 5),
        }


class MobileCargoSimulator:
    """Independent mobile tray state; never read the fixed arm after departure.

    The stationary arm's MuJoCo model verifies placement into the tray *proxy*
    while docked. A separate cargo simulator owns the transported object's
    presence thereafter. This boundary is NOT a physical cargo simulation.
    """

    def __init__(self):
        self._carrying = False
        self._verified_transfer = False

    def accept_station_handoff(
        self,
        *,
        is_docked: bool,
        fixed_arm_retracted: bool,
        observed_placement: bool,
        physics_placement: bool,
    ) -> dict[str, Any]:
        if not is_docked:
            raise MissionFailure("handoff", "robot_not_docked")
        if not fixed_arm_retracted:
            raise MissionFailure("handoff", "stationary_arm_not_retracted")
        if not observed_placement or not physics_placement:
            raise MissionFailure("handoff", "cargo_transfer_not_verified")
        self._carrying = True
        self._verified_transfer = True
        return {
            "source": "station_mujoco_physics_plus_simulated_mobile_cargo_sensor",
            "from_resource_id": ARM_RESOURCE,
            "to_resource_id": MOBILE_RESOURCE,
            "handoff_verified": True,
            "cargo_present": True,
            "simulation_only": True,
        }

    def inject_loss(self) -> None:
        self._carrying = False

    def observe_on_mobile(self) -> dict[str, Any]:
        return {
            "source": "mobile_cargo_sensor_simulation_not_physical",
            "resource_id": MOBILE_RESOURCE,
            "cargo_present": self._carrying,
            "handoff_verified": self._verified_transfer,
        }


class Evidence:
    def __init__(self, task_id: str):
        self.task_id = task_id
        self.entries: list[dict[str, Any]] = []
        self.chain = "genesis"

    def add(self, stage: str, detail: dict[str, Any]) -> None:
        record = {
            "task_id": self.task_id,
            "sequence": len(self.entries) + 1,
            "stage": stage,
            "detail": detail,
            "prev_digest": self.chain,
        }
        body = json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        self.chain = hashlib.sha256(body.encode()).hexdigest()
        self.entries.append({**record, "digest": self.chain})


def run_mission(
    goal: str,
    *,
    config_path: Path,
    blocked: bool = False,
    hide_target: bool = False,
    drop_on_return: bool = False,
    navigation: str = "kinematic",
    station_distance_m: float = 0.4,
) -> dict[str, Any]:
    task_id = str(uuid.uuid4())
    evidence = Evidence(task_id)
    cargo_sensor = MobileCargoSimulator()
    if navigation == "gazebo":
        if blocked:
            raise ValueError("use the Gazebo twin's obstacle scenario for real navigation blocking")
        from sim.fetch_return.twin_nav import GazeboTwinNavigator

        nav = GazeboTwinNavigator()
    elif navigation == "kinematic":
        nav = Navigator(Pose(0.0, 0.0, 0.0), blocked=blocked)
    else:
        raise ValueError("unknown navigation backend")
    report: dict[str, Any] = {
        "task_id": task_id,
        "goal": goal,
        "mode": "simulation_only",
        "fidelity": {
            "navigation": (
                "gazebo_ros2_nav2_and_docking"
                if navigation == "gazebo"
                else "deterministic_kinematic_not_Nav2"
            ),
            "manipulation": "MuJoCo_contact_physics",
            "camera": "MuJoCo_rendered",
            "mobile_arm_and_payload_physical_coupling": False,
            "devices": "two_separate_devices_fixed_arm_and_mobile_robot",
            "cargo_transport": "independent_sensor_simulation_not_physics",
            "world_prediction": "constant_velocity_not_learned",
        },
        "state": "failed",
        "failure": None,
        "steps": evidence.entries,
    }
    try:
        if not accepted_goal(goal):
            raise MissionFailure("planning", "unsupported_or_ambiguous_natural_language_goal")
        evidence.add(
            "planning",
            {
                "source": "rule_based_goal_template",
                "object": "red block",
                "pickup_station": "station-red",
                "destination": "home",
                "resources": [MOBILE_RESOURCE, ARM_RESOURCE],
                "coordination": "stationary_arm_loads_mobile_robot_at_docking_bay",
            },
        )
        target = Pose(
            nav.home.x + station_distance_m * math.cos(nav.home.yaw),
            nav.home.y + station_distance_m * math.sin(nav.home.yaw),
            nav.home.yaw,
        )
        outbound = nav.go(target, stage="outbound_nav")
        evidence.add("outbound_nav", {**outbound, "resource_id": MOBILE_RESOURCE})
        # At the station the chassis is stationary before the arm can actuate.
        if nav.speed > 0.026:
            raise MissionFailure("docking", "chassis_moving")
        evidence.add(
            "docking",
            {
                "source": (
                    "gazebo_ros2_nav2_simulation"
                    if navigation == "gazebo"
                    else "kinematic_simulation"
                ),
                "chassis_stopped": True,
                "observed_speed_m_s": round(nav.speed, 5),
                "station": PICKUP_STATION,
                "resource_id": MOBILE_RESOURCE,
                "fixed_arm_resource_id": ARM_RESOURCE,
                "stationary_arm_interlock": "armed_only_after_dock",
            },
        )
        arm = ArmRig(config_path)
        try:
            if hide_target:
                arm.inject_block_displacement(2.5, 2.5, 0.02)
            first = arm.find("red block")
            model = WorldPrediction()
            model.observe(first, 0.0)
            predicted = model.predict(0.05)
            evidence.add("world_prediction", {**predicted, "resource_id": ARM_RESOURCE})
            # A model predicts, but the final target must come from a new camera observation.
            reacquired = arm.find("red block")
            model.observe(reacquired, 0.05)
            evidence.add(
                "target_reacquired",
                {
                    "source": "MuJoCo_rendered_camera",
                    "id": reacquired["id"],
                    "pose": reacquired["pose"],
                    "resource_id": ARM_RESOURCE,
                },
            )
            picked = arm.call("manipulation.pick", {"object_id": reacquired["id"]})
            if picked.outcome != "completed" or picked.evidence.get("object_held") is not True:
                raise MissionFailure("pick", "grasp_not_verified")
            block_z = float(arm.world.data.body(PAYLOAD).xpos[2])
            if block_z <= 0.05:
                raise MissionFailure("pick", "contact_physics_did_not_lift_object")
            evidence.add(
                "pick",
                {
                    "source": "MuJoCo_contact_physics_and_gripper_readback",
                    "grip_held": True,
                    "block_height_m": round(block_z, 5),
                    "resource_id": ARM_RESOURCE,
                },
            )
            placed = arm.call("manipulation.place", {"container_id": "blue-1"})
            if (
                placed.outcome != "completed"
                or placed.evidence.get("perception_after", {}).get("object_in_container")
                is not True
            ):
                raise MissionFailure("load_cargo", "place_not_independently_verified")
            cargo_before = arm.independent_cargo_check()
            if not cargo_before["in_cargo"]:
                raise MissionFailure("load_cargo", "block_not_in_cargo")
            evidence.add(
                "load_cargo",
                {
                    **cargo_before,
                    "resource_id": ARM_RESOURCE,
                    "cargo_tray_representation": "stationary_mujoco_bin_proxy_of_docked_robot_tray",
                },
            )
            # Fixed station's arm must retract before the mobile robot can leave.
            stowed = arm.call("manipulation.move_joints", {"pose": "ready"})
            if stowed.outcome != "completed":
                raise MissionFailure("stow_arm", "arm_not_stowed")
            evidence.add(
                "stationary_arm_retracted",
                {
                    "source": "MuJoCo_joint_readback",
                    "resource_id": ARM_RESOURCE,
                    "ready_pose_confirmed": True,
                },
            )
            transfer = cargo_sensor.accept_station_handoff(
                is_docked=nav.speed <= 0.026
                and math.hypot(nav.pose.x - target.x, nav.pose.y - target.y) < 0.05,
                fixed_arm_retracted=True,
                observed_placement=placed.evidence["perception_after"]["object_in_container"]
                is True,
                physics_placement=cargo_before["in_cargo"],
            )
            evidence.add("cross_device_handoff", transfer)
            evidence.add(
                "return_nav",
                {**nav.go(nav.home, stage="return_nav"), "resource_id": MOBILE_RESOURCE},
            )
            if drop_on_return:
                cargo_sensor.inject_loss()
                evidence.add(
                    "fault_injection",
                    {
                        "source": "mobile_cargo_sensor_simulation",
                        "resource_id": MOBILE_RESOURCE,
                        "fault": "payload_lost_in_transit",
                    },
                )
            cargo_after = cargo_sensor.observe_on_mobile()
            if (
                not cargo_after["cargo_present"]
                or nav.speed > 0.026
                or math.hypot(nav.pose.x - nav.home.x, nav.pose.y - nav.home.y) > 0.05
            ):
                raise MissionFailure("final_verification", "home_or_cargo_not_verified")
            evidence.add(
                "final_verification",
                {
                    "source": "independent_simulated_sensor_combination",
                    "home": nav.pose.as_dict(),
                    "speed_m_s": nav.speed,
                    "payload_present": cargo_after["cargo_present"],
                    "resource_id": MOBILE_RESOURCE,
                    "cargo": cargo_after,
                },
            )
            report["state"] = "done"
        finally:
            from flyto_modules_lerobot import mujoco_sim

            mujoco_sim.reset_worlds()
    except MissionFailure as exc:
        evidence.add(
            "safe_failure",
            {
                "source": "simulated_policy_gate",
                "at_stage": exc.stage,
                "reason": exc.reason,
                "chassis_speed_m_s": nav.speed,
                "no_success_verdict": True,
            },
        )
        report["failure"] = {"stage": exc.stage, "reason": exc.reason}
    nav.close()
    report["digest"] = evidence.chain
    report["step_count"] = len(evidence.entries)
    report["completed_at"] = datetime.now(timezone.utc).isoformat()
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("goal", nargs="?", default="派機器人去把紅色積木拿回來")
    parser.add_argument("--arm-config", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--block-navigation", action="store_true")
    parser.add_argument("--navigation", choices=["kinematic", "gazebo"], default="kinematic")
    parser.add_argument("--station-distance-m", type=float, default=0.4)
    parser.add_argument("--hide-target", action="store_true")
    parser.add_argument("--drop-on-return", action="store_true")
    args = parser.parse_args(argv)
    verdict = run_mission(
        args.goal,
        config_path=args.arm_config,
        blocked=args.block_navigation,
        hide_target=args.hide_target,
        drop_on_return=args.drop_on_return,
        navigation=args.navigation,
        station_distance_m=args.station_distance_m,
    )
    serialized = json.dumps(verdict, ensure_ascii=False, indent=2)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(serialized + "\n", encoding="utf-8")
    print(serialized)
    return 0 if verdict["state"] == "done" else 1


if __name__ == "__main__":
    sys.exit(main())
