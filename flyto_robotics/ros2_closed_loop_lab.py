"""One-shot live Nav2/Gazebo closed-loop evidence runner."""

from __future__ import annotations

import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .contracts import write_json_atomic
from .resource_binding import load_resource_plan
from .ros2_action_executor import (
    execute_rclpy_navigation,
    prepare_authorized_navigation,
)
from .ros2_execution import authorize_ros2_execution
from .ros2_execution_evidence import (
    build_ros2_execution_evidence,
    evaluate_closed_loop_evidence,
)
from .ros2_pairing import load_ros2_adapter_manifest
from .ros2_probe_node import RclpyGraphProbe, collect_ros2_runtime_snapshot


def _load_json(path: str) -> dict[str, Any]:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def run_lab(node: Any) -> dict[str, Any]:
    """Probe, authorize, execute, and attest one real navigation scenario."""

    for name, default in (
        ("manifest_file", ""),
        ("resource_plan_file", ""),
        ("semantic_map_file", ""),
        ("semantic_map_id", ""),
        ("scenario", "success"),
        ("output_file", "results/nav2-closed-loop.json"),
        ("odometry_topic", "/flyto/odom"),
        ("safety_state_topic", "/safety/emergency_stop_state"),
        ("safety_reason_topic", "/safety/stop_reason"),
        ("fault_state_topic", "/fault_injection/state"),
        ("execution_state_topic", "/flyto/navigation_execution_active"),
        ("emergency_stop_node", "/safety/emergency_supervisor"),
        ("emergency_stop_service", "/safety/emergency_stop"),
        ("goal_frame", "map"),
    ):
        node.declare_parameter(name, default)
    node.declare_parameter("cancel_after_displacement_m", 0.25)
    values = {
        name: node.get_parameter(name).value
        for name in (
            "manifest_file",
            "resource_plan_file",
            "semantic_map_file",
            "semantic_map_id",
            "scenario",
            "output_file",
            "odometry_topic",
            "safety_state_topic",
            "safety_reason_topic",
            "fault_state_topic",
            "execution_state_topic",
            "emergency_stop_node",
            "emergency_stop_service",
            "goal_frame",
            "cancel_after_displacement_m",
        )
    }
    if not values["manifest_file"] or not values["resource_plan_file"]:
        raise ValueError("manifest_file and resource_plan_file are required")
    if not values["semantic_map_file"] or not values["semantic_map_id"]:
        raise ValueError("semantic_map_file and semantic_map_id are required")
    manifest = load_ros2_adapter_manifest(str(values["manifest_file"]))
    semantic_map = _load_json(str(values["semantic_map_file"]))
    if semantic_map.get("map_id") != values["semantic_map_id"]:
        raise ValueError("semantic map identity does not match")
    observed_at = datetime.now(timezone.utc)
    graph_probe = RclpyGraphProbe(node)
    discovery_deadline = time.monotonic() + 20.0
    while not graph_probe.external_emergency_stop_ready(
        owner_node=str(values["emergency_stop_node"]),
        service_name=str(values["emergency_stop_service"]),
        timeout_seconds=0.5,
    ):
        if time.monotonic() >= discovery_deadline:
            break
        import rclpy

        rclpy.spin_once(node, timeout_sec=0.1)
    runtime = collect_ros2_runtime_snapshot(
        manifest,
        graph_probe,
        deployment_mode="simulation",
        emergency_stop_node=str(values["emergency_stop_node"]),
        emergency_stop_service=str(values["emergency_stop_service"]),
        timeout_seconds=20.0,
        max_age_seconds=300,
        observed_at=observed_at,
    )
    grant = authorize_ros2_execution(
        resource_plan=load_resource_plan(str(values["resource_plan_file"])),
        manifest=manifest,
        runtime=runtime,
        workflow_id="hospital_delivery.v1",
        resource_id=manifest["robot_id"],
        capability_id="robotics.motion.navigate@1",
        target_space_id="gazebo-nav2-lab",
        observed_at=observed_at,
    )
    scenario = str(values["scenario"])
    location_id = "hospital.route.blue_end" if scenario == "success" else (
        "hospital.route.yellow_end"
    )
    prepared = prepare_authorized_navigation(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id=location_id,
        frame_id=str(values["goal_frame"]),
        observed_at=observed_at,
    )
    outcome = execute_rclpy_navigation(
        node,
        prepared,
        odometry_topic=str(values["odometry_topic"]),
        safety_state_topic=str(values["safety_state_topic"]),
        safety_reason_topic=str(values["safety_reason_topic"]),
        fault_state_topic=str(values["fault_state_topic"]),
        execution_state_topic=str(values["execution_state_topic"]),
        emergency_stop_service=str(values["emergency_stop_service"]),
        scenario=scenario,
        cancel_after_displacement_m=float(values["cancel_after_displacement_m"]),
    )
    evidence = build_ros2_execution_evidence(
        grant,
        prepared,
        outcome,
        scenario=scenario,
    )
    verdict = evaluate_closed_loop_evidence(evidence, expected_scenario=scenario)
    write_json_atomic(str(values["output_file"]), evidence)
    return {"evidence": evidence, "verdict": verdict}


def main() -> None:
    import rclpy
    from rclpy.node import Node

    rclpy.init()
    node = Node("flyto_nav2_closed_loop_lab")
    exit_code = 1
    try:
        report = run_lab(node)
        print(json.dumps(report, ensure_ascii=False, sort_keys=True), flush=True)
        exit_code = 0 if report["verdict"]["passed"] else 2
    except Exception as exc:
        print(f"closed-loop lab failed: {str(exc)[:500]}", file=sys.stderr, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
