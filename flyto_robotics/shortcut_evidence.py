"""Strict evaluator for the Gazebo workflow-card input closed loop."""

from __future__ import annotations

import argparse
import json
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from .contracts import write_json_atomic


def _check(
    checks: list[dict[str, object]],
    name: str,
    passed: bool,
    actual: object,
    expected: str,
) -> None:
    checks.append(
        {
            "name": name,
            "passed": bool(passed),
            "actual": actual,
            "expected": expected,
        }
    )


def evaluate_shortcut_evidence(
    result: object,
    manifest: object,
) -> dict[str, object]:
    """Verify AI-space input lifecycle, safety stops, recovery, and completion."""
    if not isinstance(result, Mapping) or not isinstance(manifest, Mapping):
        raise ValueError("result and manifest must be JSON objects")
    input_events = result.get("input_events", [])
    missions = result.get("missions", [])
    actions = manifest.get("actions", [])
    captures = manifest.get("captures", [])
    if not isinstance(input_events, Sequence) or isinstance(input_events, str):
        raise ValueError("result.input_events must be an array")
    if not isinstance(missions, Sequence) or isinstance(missions, str):
        raise ValueError("result.missions must be an array")
    if not isinstance(actions, Sequence) or isinstance(actions, str):
        raise ValueError("manifest.actions must be an array")
    if not isinstance(captures, Sequence) or isinstance(captures, str):
        raise ValueError("manifest.captures must be an array")

    runtime_reasons = [
        str(item.get("reason", ""))
        for item in input_events
        if isinstance(item, Mapping)
    ]
    runtime_kinds = [
        str(item.get("kind", ""))
        for item in input_events
        if isinstance(item, Mapping)
    ]
    mission_states = [
        str(item.get("final_state", ""))
        for item in missions
        if isinstance(item, Mapping)
    ]
    mission_event_kinds = [
        str(event.get("kind", ""))
        for mission in missions
        if isinstance(mission, Mapping)
        for event in mission.get("events", [])
        if isinstance(event, Mapping)
    ]
    action_kinds = [
        str(item.get("kind", ""))
        for item in actions
        if isinstance(item, Mapping)
    ]
    successful_faults = [
        item
        for item in actions
        if isinstance(item, Mapping)
        and item.get("kind") in {"obstacle_injected", "obstacle_removed"}
        and item.get("success") is True
    ]
    video = manifest.get("video", {})
    frame_count = (
        int(video.get("frame_count", 0))
        if isinstance(video, Mapping)
        else 0
    )

    checks: list[dict[str, object]] = []
    _check(
        checks,
        "contract",
        result.get("contract_version")
        == "flyto.robotics.shortcut-result.v1",
        result.get("contract_version"),
        "flyto.robotics.shortcut-result.v1",
    )
    _check(
        checks,
        "workflow_completed",
        result.get("status") == "succeeded"
        and int(result.get("completed_workflows", 0)) >= 1,
        {
            "status": result.get("status"),
            "completed_workflows": result.get("completed_workflows"),
        },
        "at least one completed workflow",
    )
    _check(
        checks,
        "press_is_workflow_not_motor",
        runtime_kinds.count("start_workflow") >= 2,
        runtime_kinds.count("start_workflow"),
        "two validated workflow starts",
    )
    _check(
        checks,
        "heartbeat_deadman",
        "input_heartbeat" in runtime_reasons,
        runtime_reasons.count("input_heartbeat"),
        "one or more accepted heartbeats",
    )
    _check(
        checks,
        "release_safe_stop",
        "input_released" in runtime_reasons and "cancelled" in mission_states,
        {
            "input_released": runtime_reasons.count("input_released"),
            "mission_states": mission_states,
        },
        "release cancels the active mission with zero velocity",
    )
    _check(
        checks,
        "physical_obstacle_stop_and_recovery",
        "obstacle_stop" in mission_event_kinds
        and "path_clear" in mission_event_kinds,
        {
            "obstacle_stop": mission_event_kinds.count("obstacle_stop"),
            "path_clear": mission_event_kinds.count("path_clear"),
        },
        "Gazebo lidar stop followed by path-clear recovery",
    )
    _check(
        checks,
        "gazebo_fault_injection",
        len(successful_faults) == 2,
        len(successful_faults),
        "obstacle model entered and exited successfully",
    )
    _check(
        checks,
        "runtime_audit",
        "runtime_event" in action_kinds and len(actions) >= 8,
        len(actions),
        "bounded driver and runtime timeline",
    )
    required_captures = {"ready", "obstacle-stop", "release-stop", "completed"}
    _check(
        checks,
        "visual_evidence",
        required_captures.issubset(set(str(item) for item in captures)),
        sorted(str(item) for item in captures),
        ", ".join(sorted(required_captures)),
    )
    _check(
        checks,
        "video_frames",
        frame_count >= 8,
        frame_count,
        "at least 8 real Gazebo camera frames",
    )
    passed = all(bool(item["passed"]) for item in checks)
    return {
        "contract_version": "flyto.robotics.shortcut-evaluation.v1",
        "passed": passed,
        "summary": {
            "passed_checks": sum(bool(item["passed"]) for item in checks),
            "total_checks": len(checks),
            "completed_workflows": result.get("completed_workflows"),
            "mission_states": mission_states,
            "world_displacement": manifest.get("world_displacement"),
            "video_frames": frame_count,
        },
        "checks": checks,
    }


def render_markdown(report: Mapping[str, object]) -> str:
    summary = report.get("summary", {})
    checks = report.get("checks", [])
    status = "PASS" if report.get("passed") else "FAIL"
    lines = [
        "# Flyto2 工作流程快捷鍵 Gazebo 閉環驗證",
        "",
        f"結論：**{status}**",
        "",
    ]
    if isinstance(summary, Mapping):
        lines.extend(
            [
                f"- 通過：{summary.get('passed_checks')}/{summary.get('total_checks')}",
                f"- 完成工作流程：{summary.get('completed_workflows')}",
                f"- 任務狀態：{summary.get('mission_states')}",
                f"- Gazebo 真實位移：{summary.get('world_displacement')} m",
                f"- Gazebo 相機影格：{summary.get('video_frames')}",
                "",
            ]
        )
    lines.extend(["| 驗證項目 | 結果 | 實際值 |", "|---|---:|---|"])
    if isinstance(checks, Sequence):
        for item in checks:
            if not isinstance(item, Mapping):
                continue
            result = "PASS" if item.get("passed") else "FAIL"
            actual = json.dumps(item.get("actual"), ensure_ascii=False)
            lines.append(f"| {item.get('name')} | {result} | `{actual}` |")
    lines.append("")
    return "\n".join(lines)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--markdown", required=True, type=Path)
    args = parser.parse_args(argv)
    result: Any = json.loads(args.result.read_text(encoding="utf-8"))
    manifest: Any = json.loads(args.manifest.read_text(encoding="utf-8"))
    report = evaluate_shortcut_evidence(result, manifest)
    write_json_atomic(args.report, report)
    args.markdown.write_text(render_markdown(report), encoding="utf-8")
    return 0 if report["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
