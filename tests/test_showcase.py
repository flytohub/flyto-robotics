from __future__ import annotations

import hashlib
import json
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path

import pytest

from flyto_robotics.ai_planner import PlanValidationError, planner_request
from flyto_robotics.capabilities import GoalFrame
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
    ROOT / "examples/goal-frames/ai4all-branching-careflow.json"
)
PLAN_FILE = ROOT / "examples/plans/careflow-waypoints-human-gate.json"


def load_json(path: Path) -> dict[str, object]:
    value = json.loads(path.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def snapshot(value: object) -> str:
    raw = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(raw).hexdigest()


def semantic_showcase_plan() -> dict[str, object]:
    plan = load_json(PLAN_FILE)
    locations = [
        "route.orange.1",
        "route.merge.1",
        "route.purple.1",
    ]
    location_index = 0
    for step in plan["steps"]:
        if not isinstance(step, dict) or step.get("capability") != "navigate":
            continue
        step["capability"] = "navigate_to_location"
        step["arguments"] = {"location_id": locations[location_index]}
        location_index += 1
    plan["plan_id"] = "test.semantic.branch.plan"
    return plan


def synthetic_live_session(
    plan: dict[str, object],
) -> dict[str, object]:
    goal = str(plan["goal"])
    request = planner_request(
        goal=goal,
        robot_id="flyto-rover-sim-001",
        goal_frame=GoalFrame.from_mapping(load_json(GOAL_FRAME_FILE)),
    )
    selected_locations = [
        str(step["arguments"]["location_id"])
        for step in plan["steps"]
        if isinstance(step, dict)
        and step.get("capability") == "navigate_to_location"
        and isinstance(step.get("arguments"), dict)
    ]
    request["observations"] = {
        "semantic_map": {
            "locations": [
                {"location_id": location_id}
                for location_id in {
                    *selected_locations,
                    "route.yellow.1",
                }
            ]
        },
        "route_candidates": [
            {
                "route_id": "route.orange-purple",
                "location_ids": selected_locations,
                "score": 80,
                "reason_codes": ["dependency_healthy"],
            },
            {
                "route_id": "route.yellow-purple",
                "location_ids": [
                    "route.yellow.1",
                    "route.merge.1",
                    "route.purple.1",
                ],
                "score": 90,
                "reason_codes": ["shorter"],
            },
        ],
    }
    attestation: dict[str, object] = {
        "contract_version": "flyto.ai.robotics-planning-attestation.v1",
        "run_id": "test-live-plan",
        "mode": "live_llm",
        "provider": "flyto-ai",
        "model": "test-model",
        "transport": "fake-test-provider",
        "request_sha256": snapshot(request),
        "plan_sha256": snapshot(plan),
        "schema_sha256": "a" * 64,
        "started_at": "2026-07-30T00:00:00+00:00",
        "finished_at": "2026-07-30T00:00:01+00:00",
        "latency_ms": 1000.0,
        "attempt_count": 1,
        "attempts": [],
        "selected_route_id": "route.orange-purple",
    }
    attestation["snapshot"] = snapshot(attestation)
    response = {
        "contract_version": "flyto.ai.robotics-plan-response.v1",
        "plan": plan,
        "attestation": attestation,
    }
    session: dict[str, object] = {
        "contract_version": "flyto.robotics.planning-session.v1",
        "session_id": "planning-session-test",
        "planning_mode": "live_llm",
        "goal": goal,
        "robot_id": "flyto-rover-sim-001",
        "rounds": [
            {
                "sequence": 1,
                "status": "superseded",
                "trigger": "initial_goal",
                "request": request,
                "response": response,
                "route_evaluation": {},
            },
            {
                "sequence": 2,
                "status": "selected",
                "trigger": "resource_dependency_changed",
                "request": request,
                "response": response,
                "route_evaluation": {},
            },
        ],
        "final_round": 2,
    }
    session["snapshot"] = snapshot(session)
    return session


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
    plan_payload = semantic_showcase_plan()
    evidence = build_showcase_planning_evidence(
        session=synthetic_live_session(plan_payload),
        goal_frame=load_json(GOAL_FRAME_FILE),
        executed_plan=plan_payload,
        robot_id="flyto-rover-sim-001",
    )

    routing = evidence["capability_routing"]
    validated = evidence["validated_plan"]
    assert routing["confidence"] >= 0.9
    assert set(validated["selected_capabilities"]) == {
        "navigate_to_location",
        "wait_until_clear",
        "ask_human",
        "resume",
        "safe_stop",
    }
    assert set(validated["selected_capabilities"]).issubset(set(routing["shortlist"]))
    assert validated["strict_validation_passed"] is True
    assert validated["direct_motor_commands_allowed"] is False
    assert validated["source"]["kind"] == "llm"


def test_showcase_planning_rejects_goal_frame_drift() -> None:
    plan_payload = semantic_showcase_plan()
    goal_frame = load_json(GOAL_FRAME_FILE)
    goal_frame["constraints"][0]["value"] = "unattested_policy"

    with pytest.raises(PlanValidationError, match="goal frame does not match"):
        build_showcase_planning_evidence(
            session=synthetic_live_session(plan_payload),
            goal_frame=goal_frame,
            executed_plan=plan_payload,
            robot_id="flyto-rover-sim-001",
        )


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
    plan_payload = semantic_showcase_plan()
    planning = build_showcase_planning_evidence(
        session=synthetic_live_session(plan_payload),
        goal_frame=load_json(GOAL_FRAME_FILE),
        executed_plan=plan_payload,
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
        "qr_confirmation": {
            "token_sha256": "a" * 64,
            "replay_rejected": True,
            "raw_token_persisted": False,
        },
        "actions": [
            {"kind": "fault_injection", "success": True},
            {"kind": "fault_injection", "success": True},
            {
                "kind": "qr_confirmation_verified",
                "token_sha256": "a" * 64,
                "raw_token_persisted": False,
            },
            {"kind": "qr_confirmation_replay_rejected"},
        ],
    }

    report = evaluate_showcase_evidence(showcase, mission, driver)
    assert report["passed"] is True
    assert report["summary"]["passed_checks"] == report["summary"]["total_checks"]
    assert report["summary"]["total_checks"] == 16
    assert report["summary"]["guarded_handoff_enabled"] is False

    facility_events.pop(2)
    failed = evaluate_showcase_evidence(showcase, mission, driver)
    assert failed["passed"] is False
    assert next(
        check
        for check in failed["checks"]
        if check["id"] == "camera_failure_detected"
    )["passed"] is False


def test_showcase_evaluator_proves_guarded_handoff_fail_closed_order() -> None:
    plan_payload = semantic_showcase_plan()
    planning = build_showcase_planning_evidence(
        session=synthetic_live_session(plan_payload),
        goal_frame=load_json(GOAL_FRAME_FILE),
        executed_plan=plan_payload,
        robot_id="flyto-rover-sim-001",
    )
    showcase = {
        "planning": planning,
        "facility": {
            "events": [
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
                },
                {
                    "kind": "resource.router_selected",
                    "resource_id": "camera.floor1.overhead",
                },
                {
                    "kind": "resource.router_selected",
                    "resource_id": "speaker.nurse_station.b",
                },
            ],
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
    guarded_events = [
        {
            "kind": "handoff_started",
            "container_locked": True,
        },
        {
            "kind": "precondition_verified",
            "container_locked": True,
        },
        {
            "kind": "item_rejected",
            "expected": "A12",
            "actual": "B13",
            "container_locked": True,
        },
        {
            "kind": "item_verified",
            "actual": "A12",
            "container_locked": True,
        },
        {
            "kind": "checkpoint_resumed",
            "checkpoint": "verify_item",
            "container_locked": True,
        },
        {
            "kind": "recipient_rejected",
            "expected": "patient-12",
            "actual": "patient-13",
            "container_locked": True,
        },
        {
            "kind": "recipient_verified",
            "actual": "patient-12",
            "container_locked": True,
        },
        {
            "kind": "container_unlocked",
            "container_locked": False,
        },
        {
            "kind": "handoff_completed",
            "container_locked": False,
        },
    ]
    driver = {
        "contract_version": "flyto.robotics.lab-driver-evidence.v2",
        "world_displacement": 4.24,
        "observed_motion": {
            "before_obstacle": True,
            "safety_stop": True,
            "resumed_after_clear": True,
        },
        "qr_confirmation": {
            "token_sha256": "a" * 64,
            "replay_rejected": True,
            "raw_token_persisted": False,
        },
        "guarded_handoff": {
            "enabled": True,
            "failed": False,
            "evidence": {
                "state": "completed",
                "container_locked": False,
                "checkpoint": None,
                "preconditions_verified": ["billing_status"],
                "item_verified": True,
                "recipient_verified": True,
                "events": guarded_events,
            },
        },
        "actions": [
            {"kind": "fault_injection", "success": True},
            {
                "kind": "safety_stop_observed",
                "minimum_range": 0.4,
                "configured_stop_distance": 0.55,
                "latest_command_velocity": {
                    "linear_x": 0.0,
                    "angular_z": 0.0,
                    "is_zero": True,
                },
            },
            {"kind": "fault_injection", "success": True},
            {
                "kind": "motion_resumed_observed",
                "latest_command_velocity": {
                    "linear_x": 0.2,
                    "angular_z": 0.0,
                    "is_zero": False,
                },
            },
            *guarded_events,
            {"kind": "guarded_handoff_approved"},
            {
                "kind": "qr_confirmation_verified",
                "token_sha256": "a" * 64,
                "raw_token_persisted": False,
            },
            {"kind": "qr_confirmation_replay_rejected"},
            {"kind": "approval_published"},
        ],
    }

    report = evaluate_showcase_evidence(showcase, mission, driver)
    assert report["passed"] is True
    assert report["summary"]["passed_checks"] == 22
    assert report["summary"]["total_checks"] == 22
    assert report["summary"]["guarded_handoff_enabled"] is True

    missing_independent_stop = deepcopy(driver)
    missing_independent_stop["observed_motion"]["safety_stop"] = False
    failed_stop = evaluate_showcase_evidence(
        showcase,
        mission,
        missing_independent_stop,
    )
    assert next(
        check
        for check in failed_stop["checks"]
        if check["id"] == "independent_command_stop_and_resume_observed"
    )["passed"] is False

    missing_checkpoint = deepcopy(driver)
    missing_checkpoint["guarded_handoff"]["evidence"]["events"].pop(4)
    failed_checkpoint = evaluate_showcase_evidence(
        showcase,
        mission,
        missing_checkpoint,
    )
    assert next(
        check
        for check in failed_checkpoint["checks"]
        if check["id"] == "wrong_item_blocked_then_checkpoint_resumed"
    )["passed"] is False

    unlocked_wrong_item = deepcopy(driver)
    unlocked_wrong_item["guarded_handoff"]["evidence"]["events"][2][
        "container_locked"
    ] = False
    failed_lock = evaluate_showcase_evidence(
        showcase,
        mission,
        unlocked_wrong_item,
    )
    assert next(
        check
        for check in failed_lock["checks"]
        if check["id"] == "wrong_item_blocked_then_checkpoint_resumed"
    )["passed"] is False


def test_branching_semantic_map_is_in_robot_odometry_frame() -> None:
    semantic_map = load_json(
        ROOT / "examples/maps/ai4all-branching-route.json"
    )
    poses = {
        item["location_id"]: item["pose"]
        for item in semantic_map["locations"]
    }
    robot_world_origin_x = -2.15
    expected_world_x = {
        "route.yellow.entry": -0.35,
        "route.orange.entry": -0.35,
        "route.merge.center": 1.1,
        "route.blue.branch": 2.35,
        "route.green.branch": 2.35,
        "route.purple.branch": 2.35,
        "route.red.branch": 2.35,
        "destination.blue": 3.25,
        "destination.green": 3.25,
        "destination.purple": 3.25,
        "destination.red": 3.25,
    }

    assert {
        location_id: round(pose["x"] + robot_world_origin_x, 2)
        for location_id, pose in poses.items()
    } == expected_world_x
