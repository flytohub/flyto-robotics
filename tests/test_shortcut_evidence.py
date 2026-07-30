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


def test_shortcut_evidence_can_require_exact_resource_plan_binding() -> None:
    result = {
        "contract_version": "flyto.robotics.shortcut-result.v1",
        "robot_id": "flyto-rover-sim-001",
        "status": "succeeded",
        "completed_workflows": 1,
        "resource_binding": {
            "plan_snapshot": "a" * 64,
            "resource_id": "flyto-rover-sim-001",
            "workflow_id": "shortcut.forward.30cm.v1",
            "adapter_id": "robotics.gazebo",
            "endpoint_id": "gazebo-rover-motion",
            "capability_id": "mobility.move_relative",
        },
        "input_events": [
            {
                "kind": "start_workflow",
                "reason": "input_pressed",
                "workflow_id": "shortcut.forward.30cm.v1",
            },
            {
                "kind": "keepalive",
                "reason": "input_heartbeat",
                "workflow_id": "shortcut.forward.30cm.v1",
            },
            {
                "kind": "safe_stop",
                "reason": "input_released",
                "workflow_id": "shortcut.forward.30cm.v1",
            },
            {
                "kind": "start_workflow",
                "reason": "input_pressed",
                "workflow_id": "shortcut.forward.30cm.v1",
            },
        ],
        "missions": [
            {
                "final_state": "cancelled",
                "events": [
                    {"kind": "obstacle_stop"},
                    {"kind": "path_clear"},
                ],
            },
            {"final_state": "completed", "events": []},
        ],
    }
    manifest = {
        "actions": [
            *[{"kind": "runtime_event"} for _ in range(6)],
            {"kind": "obstacle_injected", "success": True},
            {"kind": "obstacle_removed", "success": True},
        ],
        "captures": ["ready", "obstacle-stop", "release-stop", "completed"],
        "video": {"frame_count": 20},
    }

    report = evaluate_shortcut_evidence(
        result,
        manifest,
        require_resource_binding=True,
        expected_resource_plan_snapshot="a" * 64,
        expected_resource_adapter="robotics.gazebo",
    )

    exact = next(
        item for item in report["checks"] if item["name"] == "exact_resource_binding"
    )
    assert exact["passed"] is True
    assert report["passed"] is True
