from __future__ import annotations

from flyto_robotics.shortcut_evidence import (
    evaluate_shortcut_evidence,
    render_markdown,
)


def test_shortcut_evidence_requires_the_complete_physical_loop() -> None:
    result = {
        "contract_version": "flyto.robotics.shortcut-result.v1",
        "status": "succeeded",
        "completed_workflows": 1,
        "input_events": [
            {"kind": "start_workflow", "reason": "input_pressed"},
            {"kind": "keepalive", "reason": "input_heartbeat"},
            {"kind": "safe_stop", "reason": "input_released"},
            {"kind": "start_workflow", "reason": "input_pressed"},
        ],
        "missions": [
            {
                "final_state": "cancelled",
                "events": [
                    {"kind": "obstacle_stop"},
                    {"kind": "path_clear"},
                ],
            },
            {"final_state": "completed", "events": [{"kind": "mission_completed"}]},
        ],
    }
    manifest = {
        "actions": [
            {"kind": "runtime_event"},
            {"kind": "runtime_event"},
            {"kind": "runtime_event"},
            {"kind": "runtime_event"},
            {"kind": "runtime_event"},
            {"kind": "runtime_event"},
            {"kind": "obstacle_injected", "success": True},
            {"kind": "obstacle_removed", "success": True},
        ],
        "captures": ["ready", "obstacle-stop", "release-stop", "completed"],
        "world_displacement": 0.41,
        "video": {"frame_count": 40},
    }

    report = evaluate_shortcut_evidence(result, manifest)

    assert report["passed"] is True
    assert "結論：**PASS**" in render_markdown(report)


def test_shortcut_evidence_fails_if_release_or_video_is_missing() -> None:
    report = evaluate_shortcut_evidence(
        {
            "contract_version": "flyto.robotics.shortcut-result.v1",
            "status": "succeeded",
            "completed_workflows": 1,
            "input_events": [
                {"kind": "start_workflow", "reason": "input_pressed"},
                {"kind": "start_workflow", "reason": "input_pressed"},
                {"kind": "keepalive", "reason": "input_heartbeat"},
            ],
            "missions": [
                {
                    "final_state": "completed",
                    "events": [
                        {"kind": "obstacle_stop"},
                        {"kind": "path_clear"},
                    ],
                }
            ],
        },
        {
            "actions": [
                {"kind": "runtime_event"},
                {"kind": "runtime_event"},
                {"kind": "runtime_event"},
                {"kind": "runtime_event"},
                {"kind": "runtime_event"},
                {"kind": "runtime_event"},
                {"kind": "obstacle_injected", "success": True},
                {"kind": "obstacle_removed", "success": True},
            ],
            "captures": ["ready", "obstacle-stop", "completed"],
            "video": {"frame_count": 0},
        },
    )

    failed = {
        item["name"]
        for item in report["checks"]
        if not item["passed"]
    }
    assert {"release_safe_stop", "visual_evidence", "video_frames"} <= failed
