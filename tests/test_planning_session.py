from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from copy import deepcopy
from pathlib import Path
from typing import Any

import pytest

from flyto_robotics.planning_session import (
    PlanningSessionError,
    run_planning_session,
)
from flyto_robotics.semantic_map import SemanticLocationStore

ROOT = Path(__file__).resolve().parents[1]
SCENARIO_FILE = ROOT / "examples/routes/ai4all-branching-routes.json"
MAP_FILE = ROOT / "examples/maps/ai4all-branching-route.json"
MAP_ID = "gazebo.ai4all-branching-route.v1"
GOAL = "把補給品送到紫區護理站；設備失效時改走可驗證的安全路線。"
ROBOT_ID = "flyto-rover-sim-001"


def canonical(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def snapshot(value: object) -> str:
    return hashlib.sha256(canonical(value)).hexdigest()


def load_scenario() -> dict[str, Any]:
    value = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


class FakeLivePlanner:
    def __init__(self, *, tamper_snapshot: bool = False) -> None:
        self.requests: list[dict[str, Any]] = []
        self.tamper_snapshot = tamper_snapshot

    def complete_attested(
        self,
        request: dict[str, Any],
    ) -> tuple[object, dict[str, Any]]:
        self.requests.append(deepcopy(request))
        candidates = request["observations"]["route_candidates"]
        selected = candidates[0]
        plan = {
            "contract_version": "flyto.robotics.plan.v1",
            "plan_id": f"test-live-plan-{len(self.requests)}",
            "robot_id": request["robot_id"],
            "goal": request["goal"],
            "generated_by": {
                "kind": "llm",
                "provider": "flyto-ai",
                "model": "test-structured-model",
            },
            "steps": [
                *[
                    {
                        "step_id": f"navigate-{index + 1}",
                        "capability": "navigate_to_location",
                        "arguments": {"location_id": location_id},
                        "timeout_seconds": 60,
                        "on_failure": "request_replan",
                    }
                    for index, location_id in enumerate(selected["location_ids"])
                ],
                {
                    "step_id": "safe-stop",
                    "capability": "safe_stop",
                    "arguments": {"seconds": 0.5},
                    "timeout_seconds": 5,
                    "on_failure": "abort",
                },
            ],
        }
        attestation: dict[str, Any] = {
            "contract_version": "flyto.ai.robotics-planning-attestation.v1",
            "run_id": f"test-run-{len(self.requests)}",
            "mode": "live_llm",
            "provider": "flyto-ai",
            "model": "test-structured-model",
            "transport": "fake-test-transport",
            "request_sha256": snapshot(request),
            "plan_sha256": snapshot(plan),
            "schema_sha256": "a" * 64,
            "started_at": "2026-07-30T00:00:00+00:00",
            "finished_at": "2026-07-30T00:00:01+00:00",
            "latency_ms": 1000.0,
            "attempt_count": 1,
            "attempts": [],
            "selected_route_id": selected["route_id"],
        }
        attestation["snapshot"] = snapshot(attestation)
        if self.tamper_snapshot:
            attestation["snapshot"] = "0" * 64
        return plan, attestation


def semantic_store() -> SemanticLocationStore:
    return SemanticLocationStore(MAP_FILE, map_id=MAP_ID)


def test_session_proves_ai_replanned_from_yellow_to_orange() -> None:
    planner = FakeLivePlanner()

    session, final_plan = run_planning_session(
        scenario=load_scenario(),
        goal=GOAL,
        robot_id=ROBOT_ID,
        semantic_map=semantic_store(),
        transport=planner,
    )

    assert session["planning_mode"] == "live_llm"
    assert session["final_round"] == 2
    assert len(planner.requests) == 2
    assert session["rounds"][0]["response"]["attestation"][
        "selected_route_id"
    ] == "yellow-purple"
    assert session["rounds"][1]["response"]["attestation"][
        "selected_route_id"
    ] == "orange-purple"
    assert {
        item["attributes"]["first_branch"]
        for item in session["rounds"][1]["route_evaluation"]["excluded"]
    } == {"yellow"}
    assert [
        step["arguments"]["location_id"]
        for step in final_plan["steps"]
        if step["capability"] == "navigate_to_location"
    ] == [
        "route.orange.entry",
        "route.merge.center",
        "route.purple.branch",
        "destination.purple",
    ]
    unsigned_session = {
        key: value for key, value in session.items() if key != "snapshot"
    }
    assert session["snapshot"] == snapshot(unsigned_session)


def test_session_rejects_tampered_model_attestation() -> None:
    with pytest.raises(PlanningSessionError, match="snapshot does not match"):
        run_planning_session(
            scenario=load_scenario(),
            goal=GOAL,
            robot_id=ROBOT_ID,
            semantic_map=semantic_store(),
            transport=FakeLivePlanner(tamper_snapshot=True),
        )


def test_session_rejects_change_that_does_not_affect_selected_route() -> None:
    scenario = load_scenario()
    scenario["preflight_change"]["resource_id"] = "camera.floor1.overhead"

    with pytest.raises(
        PlanningSessionError,
        match="does not affect the AI-selected route",
    ):
        run_planning_session(
            scenario=scenario,
            goal=GOAL,
            robot_id=ROBOT_ID,
            semantic_map=semantic_store(),
            transport=FakeLivePlanner(),
        )


def test_session_requires_real_attestation_not_legacy_plan_wrapper() -> None:
    class LegacyPlanner:
        def complete_attested(
            self,
            request: dict[str, Any],
        ) -> tuple[Mapping[str, object], None]:
            return {}, None

    with pytest.raises(PlanningSessionError, match="cannot be called live AI"):
        run_planning_session(
            scenario=load_scenario(),
            goal=GOAL,
            robot_id=ROBOT_ID,
            semantic_map=semantic_store(),
            transport=LegacyPlanner(),
        )
