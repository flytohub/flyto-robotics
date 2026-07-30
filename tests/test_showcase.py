from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from flyto_robotics.facility_resources import (
    DependencyContract,
    FacilityResourceCatalog,
    FacilityResourceError,
    FacilityResourceRuntime,
    assess_device_dependency,
)
from flyto_robotics.showcase_evidence import evaluate_showcase_evidence
from flyto_robotics.showcase_planning import build_showcase_planning_evidence

ROOT = Path(__file__).resolve().parents[1]
RESOURCE_FILE = (
    ROOT / "examples/facility-resources/ai4all-showcase-facility.json"
)
GOAL_FRAME_FILE = (
    ROOT / "examples/goal-frames/ai4all-careflow-showcase.json"
)
PLAN_FILE = ROOT / "examples/plans/careflow-waypoints-human-gate.json"


def load_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_facility_runtime_handoffs_to_local_camera_then_declared_fallback() -> None:
    catalog = FacilityResourceCatalog.from_mapping(load_json(RESOURCE_FILE))
    runtime = FacilityResourceRuntime(catalog)

    blue = runtime.handoff(
        device_kind="camera",
        zone_id="zone.blue",
        at_seconds=0.5,
        reason="robot_entered_zone",
    )
    assert blue.resource.resource_id == "camera.corridor.a"
    yellow = runtime.handoff(
        device_kind="camera",
        zone_id="zone.yellow",
        at_seconds=4.0,
        reason="robot_entered_zone",
    )
    assert yellow.resource.resource_id == "camera.corridor.b"

    runtime.set_health(
        "camera.corridor.b",
        healthy=False,
        at_seconds=7.0,
        reason="showcase_fault_injection",
    )
    fallback = runtime.handoff(
        device_kind="camera",
        zone_id="zone.purple",
        at_seconds=7.1,
        reason="active_camera_unhealthy",
    )
    assert fallback.resource.resource_id == "camera.floor1.overhead"
    assert fallback.reason == "declared_fallback"

    evidence = runtime.evidence()
    event_kinds = [event["kind"] for event in evidence["events"]]
    assert event_kinds.count("resource.router_selected") == 3
    assert "resource.health_changed" in event_kinds
    loss_assessment = next(
        event
        for event in evidence["events"]
        if event["kind"] == "resource.dependency_assessed"
        and event["resource_id"] == "camera.corridor.b"
        and event["state"] == "unavailable"
    )
    assert loss_assessment["derived_band"] == "assistive"
    assert loss_assessment["action"] == "switch_substitute"
    assert loss_assessment["must_stop"] is False
    assert evidence["active_resources"]["camera"] == "camera.floor1.overhead"
    assert evidence["active_dependencies"]["camera"]["safety_impact"] == "none"


@pytest.mark.parametrize(
    (
        "contract",
        "fallback_available",
        "fallback_equivalent",
        "fallback_validated",
        "expected_band",
        "expected_action",
        "expected_stop",
    ),
    [
        (
            {},
            False,
            False,
            False,
            "assistive",
            "continue_degraded",
            False,
        ),
        (
            {"task_impact": "block", "substitution_mode": "equivalent"},
            True,
            False,
            False,
            "required",
            "pause_and_escalate",
            False,
        ),
        (
            {"task_impact": "block", "substitution_mode": "equivalent"},
            True,
            True,
            False,
            "required",
            "switch_substitute",
            False,
        ),
        (
            {"safety_impact": "stop", "substitution_mode": "validated"},
            True,
            True,
            False,
            "safety_critical",
            "safe_stop_and_escalate",
            True,
        ),
        (
            {"safety_impact": "stop", "substitution_mode": "validated"},
            True,
            True,
            True,
            "safety_critical",
            "safe_stop_then_switch_substitute",
            True,
        ),
        (
            {"task_impact": "invalidate", "substitution_mode": "none"},
            False,
            False,
            False,
            "mission_critical",
            "abort_and_invalidate",
            True,
        ),
    ],
)
def test_dependency_contract_derives_actions_from_independent_axes(
    contract: dict[str, object],
    fallback_available: bool,
    fallback_equivalent: bool,
    fallback_validated: bool,
    expected_band: str,
    expected_action: str,
    expected_stop: bool,
) -> None:
    assessment = assess_device_dependency(
        DependencyContract.from_mapping(contract),
        healthy=False,
        confidence=0.0,
        observation_age_seconds=0.0,
        fallback_available=fallback_available,
        fallback_equivalent=fallback_equivalent,
        fallback_validated=fallback_validated,
    )
    assert assessment.derived_band == expected_band
    assert assessment.action == expected_action
    assert assessment.must_stop is expected_stop


def test_dependency_contract_keeps_quality_separate_from_loss_consequence() -> None:
    observation = {
        "healthy": True,
        "confidence": 0.3,
        "observation_age_seconds": 0.1,
        "fallback_available": False,
    }
    observer = assess_device_dependency(
        DependencyContract.from_mapping({"minimum_confidence": 0.8}),
        **observation,
    )
    safety = assess_device_dependency(
        DependencyContract.from_mapping(
            {
                "minimum_confidence": 0.8,
                "safety_impact": "stop",
                "task_impact": "block",
            }
        ),
        **observation,
    )
    assert observer.reason == safety.reason == "confidence_below_requirement"
    assert observer.action == "continue_degraded"
    assert safety.action == "safe_stop_and_escalate"
    assert observer.must_stop is False
    assert safety.must_stop is True


def test_dependency_contract_honours_phase_scope_and_required_evidence() -> None:
    contract = DependencyContract.from_mapping(
        {
            "task_impact": "block",
            "evidence_requirement": "required",
            "active_phases": ["delivery"],
        }
    )
    outside = assess_device_dependency(
        contract,
        healthy=False,
        confidence=0.0,
        observation_age_seconds=99.0,
        fallback_available=False,
        evidence_present=False,
        phase="navigation",
    )
    missing = assess_device_dependency(
        contract,
        healthy=True,
        confidence=1.0,
        observation_age_seconds=0.0,
        fallback_available=False,
        evidence_present=False,
        phase="delivery",
    )
    assert outside.action == "ignore_outside_scope"
    assert missing.reason == "required_evidence_missing"
    assert missing.action == "pause_and_escalate"


def test_facility_runtime_fails_closed_without_a_healthy_declared_resource() -> None:
    catalog = FacilityResourceCatalog.from_mapping(load_json(RESOURCE_FILE))
    runtime = FacilityResourceRuntime(catalog)
    runtime.set_health(
        "camera.corridor.a",
        healthy=False,
        at_seconds=1.0,
        reason="sensor_timeout",
    )
    runtime.set_health(
        "camera.floor1.overhead",
        healthy=False,
        at_seconds=1.1,
        reason="sensor_timeout",
    )

    with pytest.raises(FacilityResourceError, match="no healthy camera resource"):
        runtime.handoff(
            device_kind="camera",
            zone_id="zone.blue",
            at_seconds=1.2,
            reason="robot_entered_zone",
        )


def test_showcase_planning_proves_shortlist_and_strict_atomic_plan() -> None:
    plan_payload = load_json(PLAN_FILE)
    evidence = build_showcase_planning_evidence(
        goal=str(plan_payload["goal"]),
        goal_frame=load_json(GOAL_FRAME_FILE),
        plan_file=PLAN_FILE,
        robot_id="flyto-rover-sim-001",
    )

    routing = evidence["capability_routing"]
    validated = evidence["validated_plan"]
    assert routing["confidence"] == 1.0
    assert set(validated["selected_capabilities"]) == {
        "navigate",
        "wait_until_clear",
        "ask_human",
        "resume",
        "safe_stop",
    }
    assert set(validated["selected_capabilities"]).issubset(set(routing["shortlist"]))
    assert validated["strict_validation_passed"] is True
    assert validated["direct_motor_commands_allowed"] is False
    assert validated["source"]["kind"] == "llm"


def test_showcase_world_exposes_three_real_gazebo_camera_streams() -> None:
    world = ET.parse(ROOT / "worlds/atomic-color-route-lab.sdf")
    topics = {
        element.text
        for element in world.findall(".//sensor/topic")
        if element.text
    }
    assert {
        "/flyto/evidence/overhead",
        "/flyto/evidence/zone_a",
        "/flyto/evidence/zone_b",
    }.issubset(topics)

    bridge = (ROOT / "config/bridge.yaml").read_text(encoding="utf-8")
    for topic in topics:
        assert f'ros_topic_name: "{topic}"' in bridge


def test_showcase_evaluator_requires_the_whole_multidevice_physical_loop() -> None:
    plan_payload = load_json(PLAN_FILE)
    planning = build_showcase_planning_evidence(
        goal=str(plan_payload["goal"]),
        goal_frame=load_json(GOAL_FRAME_FILE),
        plan_file=PLAN_FILE,
        robot_id="flyto-rover-sim-001",
    )
    facility_events = [
        {
            "kind": "resource.router_selected",
            "resource_id": "camera.corridor.a",
        },
        {
            "kind": "resource.router_selected",
            "resource_id": "camera.corridor.b",
        },
        {
            "kind": "resource.health_changed",
            "resource_id": "camera.corridor.b",
            "healthy": False,
        },
        {
            "kind": "resource.dependency_assessed",
            "resource_id": "camera.corridor.b",
            "state": "unavailable",
            "derived_band": "assistive",
            "action": "switch_substitute",
            "must_stop": False,
            "dependency": {
                "safety_impact": "none",
                "task_impact": "degrade",
            },
        },
        {
            "kind": "resource.router_selected",
            "resource_id": "camera.floor1.overhead",
        },
        {
            "kind": "resource.router_selected",
            "resource_id": "speaker.nurse_station.b",
        },
    ]
    showcase = {
        "planning": planning,
        "facility": {
            "events": facility_events,
            "seen_streams": [
                "camera.corridor.a",
                "camera.corridor.b",
                "camera.floor1.overhead",
            ],
        },
        "video": {"frame_count": 80},
    }
    mission = {
        "status": "succeeded",
        "events": [
            {"kind": "obstacle_stop"},
            {"kind": "path_clear"},
            {"kind": "human_approval_requested"},
            {"kind": "human_approved"},
            {"kind": "human_decision_rejected"},
            {"kind": "resume_authorized"},
            {"kind": "mission_completed"},
        ],
    }
    driver = {
        "world_displacement": 4.24,
        "actions": [
            {"kind": "fault_injection", "success": True},
            {"kind": "fault_injection", "success": True},
        ],
    }

    report = evaluate_showcase_evidence(showcase, mission, driver)
    assert report["passed"] is True
    assert report["summary"]["passed_checks"] == report["summary"]["total_checks"]

    facility_events.pop(2)
    failed = evaluate_showcase_evidence(showcase, mission, driver)
    assert failed["passed"] is False
    assert next(
        check
        for check in failed["checks"]
        if check["id"] == "camera_failure_detected"
    )["passed"] is False
