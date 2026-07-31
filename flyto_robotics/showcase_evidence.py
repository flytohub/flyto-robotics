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
    driver_kinds = [str(item.get("kind", "")) for item in driver_actions]
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
    selected_route = planning.get("selected_route", {})
    attestation = validated_plan.get("attestation", {})
    measured_motion = float(
        driver.get("world_path_length")
        or driver.get("world_displacement")
        or 0.0
    )
    guarded_handoff = driver.get("guarded_handoff", {})
    if not isinstance(guarded_handoff, Mapping):
        raise ValueError("driver.guarded_handoff must be an object")
    guarded_handoff_enabled = guarded_handoff.get("enabled") is True

    checks: list[dict[str, object]] = []
    _check(
        checks,
        "planning_contract",
        planning.get("contract_version")
        == "flyto.robotics.showcase-planning-evidence.v2",
        planning.get("contract_version"),
        "flyto.robotics.showcase-planning-evidence.v2",
    )
    _check(
        checks,
        "live_llm_plan_attested_and_strictly_validated",
        planning.get("planning_mode") == "live_llm"
        and validated_plan.get("source", {}).get("kind") == "llm"
        and validated_plan.get("strict_validation_passed") is True
        and validated_plan.get("executed_plan_matches_attestation") is True
        and validated_plan.get("direct_motor_commands_allowed") is False
        and isinstance(attestation, Mapping)
        and bool(attestation.get("run_id"))
        and isinstance(attestation.get("request_sha256"), str)
        and len(attestation.get("request_sha256", "")) == 64
        and isinstance(attestation.get("plan_sha256"), str)
        and len(attestation.get("plan_sha256", "")) == 64
        and isinstance(attestation.get("snapshot"), str)
        and len(attestation.get("snapshot", "")) == 64,
        {
            "planning_mode": planning.get("planning_mode"),
            "source": validated_plan.get("source"),
            "strict_validation_passed": validated_plan.get(
                "strict_validation_passed"
            ),
            "executed_plan_matches_attestation": validated_plan.get(
                "executed_plan_matches_attestation"
            ),
            "direct_motor_commands_allowed": validated_plan.get(
                "direct_motor_commands_allowed"
            ),
            "attestation": attestation,
        },
        "live model response is attested, independently validated, and identical "
        "to the executed plan",
    )
    _check(
        checks,
        "resource_failure_triggered_verified_replan",
        int(planning.get("round_count") or 0) >= 2
        and int(planning.get("replan_count") or 0) >= 1
        and any(
            isinstance(item, Mapping)
            and item.get("trigger") == "resource_dependency_changed"
            and item.get("status") == "selected"
            for item in planning.get("planning_rounds", [])
        ),
        {
            "round_count": planning.get("round_count"),
            "replan_count": planning.get("replan_count"),
            "planning_rounds": planning.get("planning_rounds"),
        },
        "initial route is superseded after a dependency change and a verified "
        "replan is selected",
    )
    _check(
        checks,
        "branch_route_selected",
        isinstance(selected_route, Mapping)
        and bool(selected_route.get("route_id"))
        and isinstance(selected_route.get("location_ids"), list)
        and len(selected_route.get("location_ids", [])) >= 2,
        selected_route,
        "one named branch with at least two trusted semantic locations",
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
    qr_confirmation = driver.get("qr_confirmation", {})
    _check(
        checks,
        "signed_qr_delivery_confirmation",
        isinstance(qr_confirmation, Mapping)
        and isinstance(qr_confirmation.get("token_sha256"), str)
        and len(str(qr_confirmation.get("token_sha256"))) == 64
        and qr_confirmation.get("raw_token_persisted") is False
        and "qr_confirmation_verified" in driver_kinds,
        {
            "manifest": qr_confirmation,
            "verified_events": driver_kinds.count("qr_confirmation_verified"),
        },
        "signed QR is verified while only its SHA-256 fingerprint is persisted",
    )
    _check(
        checks,
        "qr_nonce_replay_rejected",
        isinstance(qr_confirmation, Mapping)
        and qr_confirmation.get("replay_rejected") is True
        and "qr_confirmation_replay_rejected" in driver_kinds
        and "qr_confirmation_replay_accepted" not in driver_kinds,
        {
            "manifest": qr_confirmation,
            "rejected_events": driver_kinds.count(
                "qr_confirmation_replay_rejected"
            ),
            "accepted_events": driver_kinds.count(
                "qr_confirmation_replay_accepted"
            ),
        },
        "the same delivery QR nonce is rejected on its second use",
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
        and measured_motion >= 4.0,
        {
            "status": mission.get("status"),
            "world_displacement": driver.get("world_displacement"),
            "world_path_length": driver.get("world_path_length"),
        },
        "succeeded with at least four metres of measured Gazebo path motion",
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
    if guarded_handoff_enabled:
        guarded_evidence = guarded_handoff.get("evidence", {})
        if not isinstance(guarded_evidence, Mapping):
            raise ValueError("driver.guarded_handoff.evidence must be an object")
        guarded_events = _items(
            guarded_evidence.get("events", []),
            "driver.guarded_handoff.evidence.events",
        )
        guarded_kinds = [
            str(item.get("kind", "")) for item in guarded_events
        ]

        def _guarded_event(kind: str) -> Mapping[str, object] | None:
            return next(
                (
                    item
                    for item in guarded_events
                    if item.get("kind") == kind
                ),
                None,
            )

        precondition_verified = _guarded_event("precondition_verified")
        item_rejected = _guarded_event("item_rejected")
        item_verified = _guarded_event("item_verified")
        checkpoint_resumed = _guarded_event("checkpoint_resumed")
        recipient_rejected = _guarded_event("recipient_rejected")
        recipient_verified = _guarded_event("recipient_verified")
        container_unlocked = _guarded_event("container_unlocked")
        handoff_completed = _guarded_event("handoff_completed")
        event_positions = {
            kind: guarded_kinds.index(kind)
            for kind in {
                "precondition_verified",
                "item_rejected",
                "item_verified",
                "checkpoint_resumed",
                "recipient_rejected",
                "recipient_verified",
                "container_unlocked",
                "handoff_completed",
            }
            if kind in guarded_kinds
        }
        _check(
            checks,
            "guarded_preconditions_verified",
            precondition_verified is not None
            and bool(guarded_evidence.get("preconditions_verified"))
            and precondition_verified.get("container_locked") is True,
            {
                "preconditions_verified": guarded_evidence.get(
                    "preconditions_verified"
                ),
                "event": precondition_verified,
            },
            "declared preconditions pass while the container remains locked",
        )
        _check(
            checks,
            "wrong_item_blocked_then_checkpoint_resumed",
            item_rejected is not None
            and item_rejected.get("expected") != item_rejected.get("actual")
            and item_rejected.get("container_locked") is True
            and item_verified is not None
            and item_verified.get("container_locked") is True
            and checkpoint_resumed is not None
            and checkpoint_resumed.get("container_locked") is True
            and event_positions.get("item_rejected", -1)
            < event_positions.get("item_verified", -1)
            < event_positions.get("checkpoint_resumed", -1),
            {
                "item_rejected": item_rejected,
                "item_verified": item_verified,
                "checkpoint_resumed": checkpoint_resumed,
            },
            "wrong payload is blocked; the corrected payload resumes only "
            "from its verified checkpoint",
        )
        _check(
            checks,
            "wrong_recipient_blocked_then_verified",
            recipient_rejected is not None
            and recipient_rejected.get("expected")
            != recipient_rejected.get("actual")
            and recipient_rejected.get("container_locked") is True
            and recipient_verified is not None
            and recipient_verified.get("container_locked") is True
            and event_positions.get("recipient_rejected", -1)
            < event_positions.get("recipient_verified", -1),
            {
                "recipient_rejected": recipient_rejected,
                "recipient_verified": recipient_verified,
            },
            "wrong recipient is rejected without unlocking, then the intended "
            "recipient is verified",
        )
        _check(
            checks,
            "unlock_after_all_guarded_gates",
            guarded_evidence.get("state") == "completed"
            and guarded_evidence.get("container_locked") is False
            and guarded_evidence.get("item_verified") is True
            and guarded_evidence.get("recipient_verified") is True
            and guarded_evidence.get("checkpoint") is None
            and container_unlocked is not None
            and handoff_completed is not None
            and event_positions.get("checkpoint_resumed", -1)
            < event_positions.get("recipient_verified", -1)
            < event_positions.get("container_unlocked", -1)
            < event_positions.get("handoff_completed", -1),
            {
                "state": guarded_evidence.get("state"),
                "container_locked": guarded_evidence.get("container_locked"),
                "item_verified": guarded_evidence.get("item_verified"),
                "recipient_verified": guarded_evidence.get(
                    "recipient_verified"
                ),
                "checkpoint": guarded_evidence.get("checkpoint"),
                "container_unlocked": container_unlocked,
                "handoff_completed": handoff_completed,
            },
            "unlock occurs only after every declared gate, then the handoff "
            "reaches a terminal state",
        )
        driver_guarded_positions = {
            kind: driver_kinds.index(kind)
            for kind in {
                "handoff_completed",
                "guarded_handoff_approved",
                "approval_published",
            }
            if kind in driver_kinds
        }
        _check(
            checks,
            "guarded_handoff_authorizes_delivery",
            guarded_handoff.get("failed") is False
            and {
                "handoff_completed",
                "guarded_handoff_approved",
                "approval_published",
            }.issubset(set(driver_kinds))
            and driver_guarded_positions.get("handoff_completed", -1)
            < driver_guarded_positions.get("guarded_handoff_approved", -1)
            < driver_guarded_positions.get("approval_published", -1),
            {
                "failed": guarded_handoff.get("failed"),
                "driver_event_positions": driver_guarded_positions,
            },
            "the deterministic handoff completes before delivery approval is "
            "published",
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
            "world_path_length": driver.get("world_path_length"),
            "active_camera_frames": frame_count,
            "guarded_handoff_enabled": guarded_handoff_enabled,
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
