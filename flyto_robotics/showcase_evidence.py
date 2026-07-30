"""Evaluate the complete AI4ALL multi-device Gazebo evidence chain."""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path

from .contracts import write_json_atomic


def _items(value: object, field_name: str) -> list[Mapping[str, object]]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes)):
        raise ValueError(f"{field_name} must be an array")
    if not all(isinstance(item, Mapping) for item in value):
        raise ValueError(f"{field_name} must contain only objects")
    return list(value)


def _check(
    checks: list[dict[str, object]],
    check_id: str,
    passed: bool,
    actual: object,
    expected: object,
) -> None:
    checks.append(
        {
            "id": check_id,
            "passed": bool(passed),
            "actual": actual,
            "expected": expected,
        }
    )


def evaluate_showcase_evidence(
    showcase: object,
    mission: object,
    driver: object,
) -> dict[str, object]:
    """Verify planning, perception handoff, safety, approval, recovery, and finish."""
    if not all(isinstance(value, Mapping) for value in (showcase, mission, driver)):
        raise ValueError("showcase, mission, and driver must be JSON objects")
    planning = showcase.get("planning", {})
    facility = showcase.get("facility", {})
    video = showcase.get("video", {})
    if not all(isinstance(value, Mapping) for value in (planning, facility, video)):
        raise ValueError("showcase planning, facility, and video must be objects")
    validated_plan = planning.get("validated_plan", {})
    routing = planning.get("capability_routing", {})
    if not isinstance(validated_plan, Mapping) or not isinstance(routing, Mapping):
        raise ValueError("showcase planning evidence is malformed")

    facility_events = _items(facility.get("events", []), "facility.events")
    mission_events = _items(mission.get("events", []), "mission.events")
    driver_actions = _items(driver.get("actions", []), "driver.actions")
    facility_kinds = [str(item.get("kind", "")) for item in facility_events]
    mission_kinds = [str(item.get("kind", "")) for item in mission_events]
    selected_resources = [
        str(item.get("resource_id", ""))
        for item in facility_events
        if item.get("kind") == "resource.router_selected"
    ]
    successful_faults = [
        item
        for item in driver_actions
        if item.get("kind") == "fault_injection" and item.get("success") is True
    ]
    selected_capabilities = validated_plan.get("selected_capabilities", [])
    shortlist = routing.get("shortlist", [])

    checks: list[dict[str, object]] = []
    _check(
        checks,
        "planning_contract",
        planning.get("contract_version")
        == "flyto.robotics.showcase-planning-evidence.v1",
        planning.get("contract_version"),
        "flyto.robotics.showcase-planning-evidence.v1",
    )
    _check(
        checks,
        "llm_plan_strictly_validated",
        validated_plan.get("source", {}).get("kind") == "llm"
        and validated_plan.get("strict_validation_passed") is True
        and validated_plan.get("direct_motor_commands_allowed") is False,
        {
            "source": validated_plan.get("source"),
            "strict_validation_passed": validated_plan.get(
                "strict_validation_passed"
            ),
            "direct_motor_commands_allowed": validated_plan.get(
                "direct_motor_commands_allowed"
            ),
        },
        "LLM plan passed strict validation without direct motor commands",
    )
    _check(
        checks,
        "capability_shortlist_enforced",
        isinstance(selected_capabilities, list)
        and isinstance(shortlist, list)
        and bool(selected_capabilities)
        and set(selected_capabilities).issubset(set(shortlist)),
        selected_capabilities,
        "every selected atom belongs to the routed shortlist",
    )
    _check(
        checks,
        "three_camera_streams_observed",
        {
            "camera.corridor.a",
            "camera.corridor.b",
            "camera.floor1.overhead",
        }.issubset(set(facility.get("seen_streams", []))),
        facility.get("seen_streams"),
        "zone A, zone B, and overhead Gazebo camera streams",
    )
    expected_handoff = [
        "camera.corridor.a",
        "camera.corridor.b",
        "camera.floor1.overhead",
    ]
    _check(
        checks,
        "camera_handoff_and_fallback",
        selected_resources[:3] == expected_handoff,
        selected_resources,
        expected_handoff,
    )
    _check(
        checks,
        "camera_failure_detected",
        any(
            item.get("kind") == "resource.health_changed"
            and item.get("resource_id") == "camera.corridor.b"
            and item.get("healthy") is False
            for item in facility_events
        )
        and any(
            item.get("kind") == "resource.dependency_assessed"
            and item.get("resource_id") == "camera.corridor.b"
            and item.get("state") == "unavailable"
            and item.get("derived_band") == "assistive"
            and item.get("action") == "switch_substitute"
            and item.get("must_stop") is False
            for item in facility_events
        ),
        {
            "health_changes": facility_kinds.count("resource.health_changed"),
            "dependency_assessments": facility_kinds.count(
                "resource.dependency_assessed"
            ),
        },
        "camera B loss is assessed as substitutable observation, then rerouted",
    )
    _check(
        checks,
        "physical_obstacle_stop_and_recovery",
        "obstacle_stop" in mission_kinds and "path_clear" in mission_kinds,
        {
            "obstacle_stop": mission_kinds.count("obstacle_stop"),
            "path_clear": mission_kinds.count("path_clear"),
        },
        "LiDAR stop followed by path-clear recovery",
    )
    _check(
        checks,
        "human_gate_and_replay_defense",
        {
            "human_approval_requested",
            "human_approved",
            "human_decision_rejected",
            "resume_authorized",
        }.issubset(set(mission_kinds)),
        sorted(set(mission_kinds)),
        "approval, signed decision, replay rejection, and resume",
    )
    _check(
        checks,
        "gazebo_fault_injection",
        len(successful_faults) >= 2,
        len(successful_faults),
        "obstacle enters and exits through Gazebo services",
    )
    _check(
        checks,
        "mission_completed_with_motion",
        mission.get("status") == "succeeded"
        and "mission_completed" in mission_kinds
        and float(driver.get("world_displacement") or 0.0) >= 4.0,
        {
            "status": mission.get("status"),
            "world_displacement": driver.get("world_displacement"),
        },
        "succeeded with at least four metres of measured world motion",
    )
    _check(
        checks,
        "completion_device_selected",
        "speaker.nurse_station.b" in selected_resources,
        selected_resources,
        "delivery completion binds the zone speaker endpoint",
    )
    frame_count = int(video.get("frame_count", 0))
    _check(
        checks,
        "active_camera_video",
        frame_count >= 20,
        frame_count,
        "at least 20 frames from the currently leased real Gazebo camera",
    )

    passed = all(bool(check["passed"]) for check in checks)
    return {
        "contract_version": "flyto.robotics.ai4all-showcase-evaluation.v1",
        "passed": passed,
        "summary": {
            "passed_checks": sum(bool(check["passed"]) for check in checks),
            "total_checks": len(checks),
            "selected_capabilities": selected_capabilities,
            "selected_resources": selected_resources,
            "world_displacement": driver.get("world_displacement"),
            "active_camera_frames": frame_count,
        },
        "checks": checks,
    }


def render_markdown(report: Mapping[str, object]) -> str:
    summary = report["summary"]
    if not isinstance(summary, Mapping):
        raise ValueError("report.summary must be an object")
    lines = [
        "# Flyto2 AI4ALL 多設備閉環測試",
        "",
        f"- 結果：{'通過' if report.get('passed') else '失敗'}",
        f"- 檢查：{summary.get('passed_checks')} / {summary.get('total_checks')}",
        f"- Gazebo 位移：{summary.get('world_displacement')} 公尺",
        f"- 主動攝影機影格：{summary.get('active_camera_frames')}",
        "",
        "## 驗證項目",
        "",
    ]
    for check in report.get("checks", []):
        if isinstance(check, Mapping):
            lines.append(
                f"- {'PASS' if check.get('passed') else 'FAIL'} `{check.get('id')}`"
            )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--showcase", required=True, type=Path)
    parser.add_argument("--mission", required=True, type=Path)
    parser.add_argument("--driver", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--markdown", required=True, type=Path)
    args = parser.parse_args()
    report = evaluate_showcase_evidence(
        json.loads(args.showcase.read_text(encoding="utf-8")),
        json.loads(args.mission.read_text(encoding="utf-8")),
        json.loads(args.driver.read_text(encoding="utf-8")),
    )
    write_json_atomic(args.report, report)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
