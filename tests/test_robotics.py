from __future__ import annotations

import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from threading import Thread

import pytest

from flyto_robotics.ai_planner import (
    CallablePlannerTransport,
    HTTPJsonPlannerTransport,
    PlanValidationError,
    compile_workflow,
    load_plan,
    parse_plan,
    planner_request,
    request_ai_plan,
)
from flyto_robotics.capabilities import (
    ArgumentSpec,
    CapabilityDefinition,
    CapabilityRegistry,
    CapabilityRoutingContext,
    GoalFrame,
    default_capability_registry,
)
from flyto_robotics.cli import (
    PROJECT_ROOT,
    dry_run,
    dry_run_plan,
    main,
    validate_assets,
)
from flyto_robotics.contracts import (
    JOB_CONTRACT_VERSION,
    JobValidationError,
    job_to_dict,
    load_job,
    parse_job,
    write_json_atomic,
)
from flyto_robotics.human_approval import (
    HumanDecisionAuthenticator,
    HumanDecisionValidationError,
    build_signed_human_decision,
    decision_to_json,
)
from flyto_robotics.line_perception import detect_line_scene
from flyto_robotics.mission import MissionController, Pose2D
from flyto_robotics.semantic_map import (
    SEMANTIC_CATALOG_CONTRACT_VERSION,
    SEMANTIC_MAP_CONTRACT_VERSION,
    SemanticLocationStore,
    SemanticMapValidationError,
)
from flyto_robotics.workflow import (
    MissionState,
    PrimitiveKind,
    WorkflowPlan,
    WorkflowStep,
    hospital_delivery_workflow,
)

EXAMPLE_JOB = PROJECT_ROOT / "examples/jobs/pharmacy-to-ward.json"
EXAMPLE_PLAN = PROJECT_ROOT / "examples/plans/blue-yellow-purple.json"
CAREFLOW_PLAN = PROJECT_ROOT / "examples/plans/careflow-human-gate.json"
CAREFLOW_WAYPOINT_PLAN = (
    PROJECT_ROOT / "examples/plans/careflow-waypoints-human-gate.json"
)
SEMANTIC_MAP = PROJECT_ROOT / "examples/maps/atomic-color-route.json"
SEMANTIC_PLAN = PROJECT_ROOT / "examples/plans/semantic-location-sequence.json"
APPROVAL_SECRET = "test-only-approval-secret-with-at-least-32-bytes"


def test_example_job_and_atomic_workflow_compile() -> None:
    job = load_job(EXAMPLE_JOB)
    workflow = hospital_delivery_workflow(job)

    assert job.contract_version == JOB_CONTRACT_VERSION
    assert workflow.workflow_id == "hospital_delivery.v1"
    assert [step.kind for step in workflow.steps] == [
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.DWELL,
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.DWELL,
    ]
    assert [step.step_id for step in workflow.steps] == [
        "navigate.pickup",
        "dwell.pickup",
        "navigate.dropoff",
        "dwell.dropoff",
    ]


def test_controller_accepts_a_custom_composition() -> None:
    job = load_job(EXAMPLE_JOB)
    workflow = WorkflowPlan(
        workflow_id="test.single_dwell.v1",
        steps=(
            WorkflowStep(
                step_id="dwell.health_check",
                kind=PrimitiveKind.DWELL,
                active_state=MissionState.WAITING_FOR_PICKUP,
                station=job.pickup,
                dwell_seconds=0.0,
            ),
        ),
    )
    controller = MissionController(job, workflow=workflow)

    command = controller.tick(Pose2D(0.0, 0.0, 0.0), minimum_range=math.inf, now=0.0)

    assert controller.state == MissionState.COMPLETED
    assert command.linear_x == 0.0
    assert command.angular_z == 0.0
    assert any(event.kind == "primitive_completed" for event in controller.events)


def test_unknown_or_sensitive_job_fields_are_rejected() -> None:
    decoded = job_to_dict(load_job(EXAMPLE_JOB))
    decoded["patient_name"] = "must-not-enter-robot-contract"

    with pytest.raises(JobValidationError, match="unsupported fields"):
        parse_job(decoded)


def test_unsafe_speed_is_rejected() -> None:
    decoded = job_to_dict(load_job(EXAMPLE_JOB))
    decoded["safety"]["max_linear_speed"] = 4.0

    with pytest.raises(JobValidationError, match="max_linear_speed"):
        parse_job(decoded)


def test_obstacle_atom_stops_and_recovers() -> None:
    job = load_job(EXAMPLE_JOB)
    controller = MissionController(job)
    pose = Pose2D(0.0, 0.0, 0.0)

    stopped = controller.tick(pose, minimum_range=0.2, now=0.0)
    resumed = controller.tick(pose, minimum_range=math.inf, now=0.1)

    assert stopped.reason == "obstacle_stop"
    assert stopped.linear_x == 0.0
    assert controller.safety_stop_count == 1
    assert resumed.reason in {"turning_to_target", "advancing_to_target"}
    assert [event.kind for event in controller.events].count("obstacle_stop") == 1
    assert [event.kind for event in controller.events].count("path_clear") == 1


def test_dry_run_completes_with_stop_evidence() -> None:
    result = dry_run(EXAMPLE_JOB)

    assert result["status"] == "succeeded"
    assert result["final_state"] == "completed"
    assert result["safety_stop_count"] == 1
    assert result["simulation"] == {
        "mode": "deterministic_planar_dry_run",
        "obstacle_injected": True,
        "gazebo_physics": False,
    }
    kinds = [event["kind"] for event in result["events"]]
    assert kinds.count("primitive_started") == 4
    assert kinds.count("primitive_completed") == 4


def test_result_write_is_atomic_and_valid_json(tmp_path: Path) -> None:
    destination = tmp_path / "nested/result.json"
    value = {"ok": True, "sequence": [1, 2, 3]}

    write_json_atomic(destination, value)

    assert json.loads(destination.read_text(encoding="utf-8")) == value
    assert not list(destination.parent.glob("*.tmp"))


def test_static_assets_are_parseable_and_self_contained() -> None:
    checked = validate_assets()
    world = (PROJECT_ROOT / "worlds/hospital-logistics.sdf").read_text(encoding="utf-8")

    assert "worlds/hospital-logistics.sdf" in checked
    assert "models/flyto_rover/model.sdf" in checked
    assert "worlds/atomic-color-route.sdf" in checked
    assert "contracts/plan-v1.schema.json" in checked
    assert "contracts/human-decision-v1.schema.json" in checked
    assert "contracts/capability-manifest-v1.schema.json" in checked
    assert "contracts/capability-route-v1.schema.json" in checked
    assert "contracts/goal-frame-v1.schema.json" in checked
    assert "contracts/semantic-location-catalog-v1.schema.json" in checked
    assert "contracts/semantic-location-map-v1.schema.json" in checked
    assert "examples/maps/atomic-color-route.json" in checked
    assert "examples/goal-frames/semantic-location-sequence.json" in checked
    assert "examples/plans/blue-yellow-purple.json" in checked
    assert "examples/plans/blue-yellow-purple-waypoints.json" in checked
    assert "examples/plans/careflow-human-gate.json" in checked
    assert "examples/plans/careflow-waypoints-human-gate.json" in checked
    assert "examples/plans/semantic-location-sequence.json" in checked
    assert "examples/plans/teach-current-location.json" in checked
    assert "https://" not in world
    assert "fuel.gazebosim.org" not in world


def test_capability_manifest_has_stable_namespaced_ids_and_snapshot() -> None:
    registry = default_capability_registry()
    catalog = registry.catalog()

    assert {item["canonical_id"] for item in catalog} >= {
        "robotics.motion.navigate@1",
        "robotics.vision.follow_line@1",
        "robotics.safety.safe_stop@1",
        "robotics.motion.navigate_to_location@1",
        "robotics.memory.save_current_location@1",
    }
    assert all(item["manifest_contract"] == "flyto.capability-manifest.v1" for item in catalog)
    assert all(item["runtime_name"] == item["name"] for item in catalog)
    assert all("affordances" in item for item in catalog)
    assert all("effects" in item for item in catalog)
    assert registry.snapshot_hash() == default_capability_registry().snapshot_hash()


def test_semantic_frame_excludes_unmatched_atoms_without_changing_legacy_atoms() -> None:
    registry = default_capability_registry()
    frame = GoalFrame(
        intent_ids=("location.remember.current_pose",),
        required_affordances=("map.semantic_location.write",),
        desired_effects=("location.pose.saved",),
    )

    route = registry.route(
        "記住這裡",
        goal_frame=frame,
        limit=8,
    )

    assert route.names == ("save_current_location",)
    assert route.semantic_missing == ()
    assert route.confidence == 1.0
    legacy = registry.route("沿藍線前進", limit=3)
    assert legacy.names[0] == "follow_line"
    assert "safe_stop" in legacy.names
    assert "navigate_to_location" not in legacy.names
    assert "save_current_location" not in legacy.names


def test_semantic_location_store_is_language_neutral_and_hides_pose_from_llm(
    tmp_path: Path,
) -> None:
    store = SemanticLocationStore(
        tmp_path / "semantic-map.json",
        map_id="hospital.demo.1",
    )

    first = store.remember(
        location_id="hospital.nurse_station.1",
        label="護理站",
        pose=Pose2D(4.25, 2.1, 1.57),
    )
    second = store.remember(
        location_id="hospital.nurse_station.1",
        label="محطة التمريض",
        pose=Pose2D(4.5, 2.25, 1.5),
        expected_revision=first.revision,
    )
    third = store.remember(
        location_id="hospital.nurse_station.1",
        label="ナースステーション",
        pose=Pose2D(4.5, 2.25, 1.5),
        expected_revision=second.revision,
    )

    location = store.resolve("hospital.nurse_station.1")
    assert third.to_dict()["contract_version"] == SEMANTIC_MAP_CONTRACT_VERSION
    assert location.labels == (
        "護理站",
        "محطة التمريض",
        "ナースステーション",
    )
    assert location.pose.x == 4.5
    planner_view = store.planner_view()
    assert planner_view["contract_version"] == SEMANTIC_CATALOG_CONTRACT_VERSION
    assert planner_view["revision"] == 3
    assert "pose" not in planner_view["locations"][0]
    saved = json.loads((tmp_path / "semantic-map.json").read_text(encoding="utf-8"))
    assert saved["locations"][0]["pose"]["x"] == 4.5
    assert not list(tmp_path.glob("*.tmp"))


def test_semantic_location_store_fails_closed_on_map_mismatch_and_unknown_id(
    tmp_path: Path,
) -> None:
    path = tmp_path / "semantic-map.json"
    SemanticLocationStore(path, map_id="hospital.map.a").remember(
        location_id="hospital.nurse_station.1",
        label="護理站",
        pose=Pose2D(1.0, 2.0, 0.0),
    )

    with pytest.raises(SemanticMapValidationError, match="map_id mismatch"):
        SemanticLocationStore(path, map_id="hospital.map.b").load()
    with pytest.raises(SemanticMapValidationError, match="not registered"):
        SemanticLocationStore(path, map_id="hospital.map.a").resolve(
            "hospital.unknown.1"
        )


def test_semantic_navigation_routing_is_identical_across_languages_and_hides_pose(
    tmp_path: Path,
) -> None:
    store = SemanticLocationStore(
        tmp_path / "semantic-map.json",
        map_id="hospital.demo.1",
    )
    store.remember(
        location_id="hospital.nurse_station.1",
        label="護理站",
        pose=Pose2D(4.25, 2.1, 1.57),
    )
    frame = {
        "contract_version": "flyto.goal-frame.v1",
        "intent_ids": ["route.navigate.location"],
        "required_affordances": ["motion.navigate.semantic_location"],
        "desired_effects": ["robot.location.reached", "robot.motion.stopped"],
        "trigger_events": [],
        "constraints": [
            {
                "key": "target.location_id",
                "operator": "equals",
                "value": "hospital.nurse_station.1",
            }
        ],
    }

    requests = [
        planner_request(
            goal=goal,
            goal_frame=frame,
            robot_id="flyto-rover-sim-001",
            semantic_map=store,
            route_limit=4,
        )
        for goal in (
            "去護理站並安全停止",
            "اذهب إلى محطة التمريض وتوقف بأمان",
            "ナースステーションへ行き、安全に停止する",
        )
    ]

    routed_names = [
        tuple(
            candidate["runtime_name"]
            for candidate in request["capability_route"]["candidates"]
        )
        for request in requests
    ]
    assert routed_names == [
        ("navigate_to_location", "safe_stop"),
        ("navigate_to_location", "safe_stop"),
        ("navigate_to_location", "safe_stop"),
    ]
    assert all(
        request["capability_route"]["semantic_coverage"]["ratio"] == 1.0
        for request in requests
    )
    catalog = requests[0]["observations"]["semantic_map"]
    assert catalog["contract_version"] == SEMANTIC_CATALOG_CONTRACT_VERSION
    assert catalog["locations"][0]["location_id"] == "hospital.nurse_station.1"
    assert "pose" not in catalog["locations"][0]


def test_named_navigation_fails_closed_when_location_is_missing(tmp_path: Path) -> None:
    store = SemanticLocationStore(
        tmp_path / "semantic-map.json",
        map_id="hospital.demo.1",
    )
    decoded = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "semantic-navigation.missing",
        "robot_id": "flyto-rover-sim-001",
        "goal": "去不存在的地點",
        "generated_by": {
            "kind": "llm",
            "provider": "test",
            "model": "fake",
        },
        "steps": [
            {
                "step_id": "navigate.unknown",
                "capability": "navigate_to_location",
                "arguments": {"location_id": "hospital.unknown.1"},
                "timeout_seconds": 120,
                "on_failure": "request_replan",
            },
            {
                "step_id": "stop.final",
                "capability": "safe_stop",
                "arguments": {"seconds": 0},
                "timeout_seconds": 2,
                "on_failure": "abort",
            },
        ],
    }

    with pytest.raises(PlanValidationError, match="not registered"):
        compile_workflow(parse_plan(decoded), semantic_map=store)


def test_named_navigation_uses_trusted_store_pose_not_llm_coordinates(
    tmp_path: Path,
) -> None:
    store = SemanticLocationStore(
        tmp_path / "semantic-map.json",
        map_id="hospital.demo.1",
    )
    store.remember(
        location_id="hospital.nurse_station.1",
        label="護理站",
        pose=Pose2D(4.25, 2.1, 1.57),
    )
    decoded = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "semantic-navigation.test",
        "robot_id": "flyto-rover-sim-001",
        "goal": "去護理站",
        "generated_by": {
            "kind": "llm",
            "provider": "test",
            "model": "fake",
        },
        "steps": [
            {
                "step_id": "navigate.nurse_station",
                "capability": "navigate_to_location",
                "arguments": {
                    "location_id": "hospital.nurse_station.1",
                },
                "timeout_seconds": 120,
                "on_failure": "request_replan",
            },
            {
                "step_id": "stop.final",
                "capability": "safe_stop",
                "arguments": {"seconds": 0},
                "timeout_seconds": 2,
                "on_failure": "abort",
            },
        ],
    }

    plan = parse_plan(decoded)
    workflow = compile_workflow(plan, semantic_map=store)

    assert workflow.steps[0].kind == PrimitiveKind.NAVIGATE_TO_LOCATION
    assert workflow.steps[0].station is not None
    assert workflow.steps[0].station.station_id == "hospital.nurse_station.1"
    assert workflow.steps[0].station.x == 4.25
    tampered = json.loads(json.dumps(decoded))
    tampered["steps"][0]["arguments"]["x"] = 999.0
    with pytest.raises(PlanValidationError, match="unsupported fields"):
        parse_plan(tampered)


def test_save_current_location_atom_persists_current_odometry_pose(
    tmp_path: Path,
) -> None:
    store = SemanticLocationStore(
        tmp_path / "semantic-map.json",
        map_id="hospital.demo.1",
    )
    decoded = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "teach-location.test",
        "robot_id": "flyto-rover-sim-001",
        "goal": "記住這裡是護理站",
        "generated_by": {
            "kind": "llm",
            "provider": "test",
            "model": "fake",
        },
        "steps": [
            {
                "step_id": "remember.nurse_station",
                "capability": "save_current_location",
                "arguments": {
                    "location_id": "hospital.nurse_station.1",
                    "label": "護理站",
                },
                "timeout_seconds": 5,
                "on_failure": "abort",
            },
            {
                "step_id": "stop.final",
                "capability": "safe_stop",
                "arguments": {"seconds": 0},
                "timeout_seconds": 2,
                "on_failure": "abort",
            },
        ],
    }
    workflow = compile_workflow(parse_plan(decoded), semantic_map=store)
    controller = MissionController(
        load_job(EXAMPLE_JOB),
        workflow=workflow,
        semantic_map_store=store,
    )
    pose = Pose2D(3.2, -1.4, 0.75)

    controller.tick(pose, minimum_range=math.inf, now=0.0)
    controller.tick(pose, minimum_range=math.inf, now=0.1)

    saved = store.resolve("hospital.nurse_station.1")
    assert saved.labels == ("護理站",)
    assert (saved.pose.x, saved.pose.y, saved.pose.yaw) == (3.2, -1.4, 0.75)
    assert controller.state == MissionState.COMPLETED
    assert "semantic_location_saved" in {
        event.kind for event in controller.events
    }


def test_multilingual_router_selects_line_and_obstacle_atoms_reproducibly() -> None:
    registry = default_capability_registry()
    goal = "先走藍線，再走黃線，最後走紫線；遇到人就停下來等待淨空。"

    first = registry.route(goal, limit=5)
    second = registry.route(goal, limit=5)

    assert first.names == second.names
    assert first.registry_snapshot == second.registry_snapshot
    assert first.names[0] == "follow_line"
    assert {"follow_line", "wait_until_clear", "safe_stop"}.issubset(first.names)
    assert first.needs_clarification is False
    assert first.confidence >= 0.8


def test_goal_frame_selection_is_identical_across_unrelated_languages() -> None:
    registry = default_capability_registry()
    frame = GoalFrame(
        intent_ids=("route.follow.sequence",),
        required_affordances=(
            "motion.follow.visual_line",
            "safety.wait_until_clear",
        ),
        desired_effects=(
            "robot.motion.stopped",
            "route.sequence.completed",
        ),
        trigger_events=("human.detected",),
        constraints=(
            {
                "key": "route.sequence",
                "operator": "ordered",
                "value": ["blue", "yellow", "purple"],
            },
        ),
    )

    routes = [
        registry.route(text, goal_frame=frame, limit=5)
        for text in (
            "先走藍線，再走黃線，最後走紫線；遇到人就等待。",
            "اتبع الخطوط بالترتيب وانتظر عند وجود شخص.",
            "青、黄、紫の順で進み、人がいたら待機する。",
        )
    ]

    assert routes[0] == routes[1] == routes[2]
    assert routes[0].names[:3] == (
        "follow_line",
        "wait_until_clear",
        "safe_stop",
    )
    assert routes[0].semantic_missing == ()
    assert routes[0].confidence == 1.0
    assert routes[0].needs_clarification is False
    assert routes[0].to_dict()["selection_method"] == (
        "hard_filter_then_semantic_frame_rank_v1"
    )


def test_planner_request_carries_goal_frame_without_language_metadata() -> None:
    frame = {
        "contract_version": "flyto.goal-frame.v1",
        "intent_ids": ["route.follow.sequence"],
        "required_affordances": ["motion.follow.visual_line"],
        "desired_effects": [
            "robot.motion.stopped",
            "route.sequence.completed",
        ],
        "trigger_events": [],
        "constraints": [
            {
                "key": "route.sequence",
                "operator": "ordered",
                "value": ["blue", "yellow", "purple"],
            }
        ],
    }

    request = planner_request(
        goal="輸入文字可以是任何語言",
        goal_frame=frame,
        robot_id="flyto-rover-sim-001",
        route_limit=4,
    )

    assert request["goal_frame"] == frame
    assert "language" not in request["goal_frame"]
    assert request["capability_route"]["semantic_coverage"]["ratio"] == 1.0
    assert {
        item["runtime_name"] for item in request["capabilities"]
    } >= {"follow_line", "safe_stop"}


@pytest.mark.parametrize(
    ("goal", "expected"),
    [
        ("沿藍線前進", "follow_line"),
        ("先循線到下一個交叉點", "follow_line"),
        ("follow the purple line", "follow_line"),
        ("前往護理站", "navigate"),
        ("navigate to waypoint", "navigate"),
        ("移動到座標", "navigate"),
        ("等待淨空後再走", "wait_until_clear"),
        ("wait until the obstacle clears", "wait_until_clear"),
        ("有人擋住就等待", "wait_until_clear"),
        ("不確定就問我", "ask_human"),
        ("request human approval", "ask_human"),
        ("人工確認後再執行", "ask_human"),
        ("停留十秒", "dwell"),
        ("wait for five seconds", "dwell"),
        ("安全停止", "safe_stop"),
        ("emergency stop", "safe_stop"),
    ],
)
def test_router_top1_paraphrase_recall(goal: str, expected: str) -> None:
    route = default_capability_registry().route(goal, limit=3)

    assert route.names[0] == expected


def test_router_hard_filters_missing_observations_before_ranking() -> None:
    route = default_capability_registry().route(
        "沿藍線前進",
        context=CapabilityRoutingContext(
            available_observations=frozenset({"odometry", "minimum_range"})
        ),
    )

    assert "follow_line" not in route.names
    excluded = dict(route.excluded)
    assert excluded["follow_line"] == ("missing_observation",)


def test_large_registry_is_bounded_before_the_llm_sees_it() -> None:
    base = default_capability_registry()
    distractors = tuple(
        CapabilityDefinition(
            name=f"noise_{index}",
            canonical_id=f"robotics.test.noise_{index}@1",
            description=f"Unrelated synthetic capability {index}.",
            control_class="timed",
            required_observations=(),
            arguments=(ArgumentSpec("seconds", "number", minimum=0.0, maximum=1.0),),
            safety_notes="Test-only capability.",
            tags=("unrelated",),
        )
        for index in range(40)
    )
    definitions = tuple(base.definition(name) for name in sorted(base.names)) + distractors
    registry = CapabilityRegistry(definitions)

    request = planner_request(
        goal="先走藍線再安全停止",
        robot_id="flyto-rover-sim-001",
        registry=registry,
        route_limit=6,
    )

    assert len(request["capabilities"]) == 6
    assert len(request["capability_route"]["candidates"]) == 6
    assert request["capability_route"]["registry_snapshot"] == registry.snapshot_hash()
    assert {"follow_line", "safe_stop"}.issubset({
        capability["runtime_name"] for capability in request["capabilities"]
    })


def test_ai_selects_and_orders_registered_capabilities() -> None:
    response = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))

    def fake_llm(request: dict[str, object]) -> object:
        capabilities = request["capabilities"]
        assert isinstance(capabilities, list)
        assert {item["name"] for item in capabilities} >= {
            "navigate",
            "follow_line",
            "dwell",
            "wait_until_clear",
            "ask_human",
            "resume",
            "safe_stop",
        }
        assert request["goal"] == "先走藍線，再走黃線，最後走紫線並安全停止。"
        assert request["capability_route"]["needs_clarification"] is False
        assert request["capability_route"]["registry_snapshot"].startswith("sha256:")
        return response

    plan = request_ai_plan(
        CallablePlannerTransport(fake_llm),
        goal="先走藍線，再走黃線，最後走紫線並安全停止。",
        robot_id="flyto-rover-sim-001",
    )
    workflow = compile_workflow(plan)

    assert plan.generated_by.kind == "llm"
    assert plan.registry_snapshot.startswith("sha256:")
    assert "follow_line" in plan.allowed_capabilities
    assert [step.kind for step in workflow.steps] == [
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.SAFE_STOP,
    ]
    assert [step.argument("color") for step in workflow.steps[:3]] == [
        "blue",
        "yellow",
        "purple",
    ]


def test_ai_cannot_inject_raw_motor_or_unregistered_capability() -> None:
    decoded = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))
    decoded["steps"][0]["capability"] = "set_wheel_pwm"
    decoded["steps"][0]["arguments"] = {"left": 255, "right": 255}

    with pytest.raises(PlanValidationError, match="not registered"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="全速前進",
            robot_id="flyto-rover-sim-001",
        )


def test_ai_plan_must_target_the_requested_robot() -> None:
    decoded = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))

    with pytest.raises(PlanValidationError, match="requested robot"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="巡檢 A 區",
            robot_id="different-robot",
        )


def test_https_adapter_contract_works_with_loopback_planner() -> None:
    response = EXAMPLE_PLAN.read_bytes()
    requests: list[dict[str, object]] = []

    class PlannerHandler(BaseHTTPRequestHandler):
        def do_POST(self) -> None:  # noqa: N802
            content_length = int(self.headers["Content-Length"])
            requests.append(json.loads(self.rfile.read(content_length)))
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(response)))
            self.end_headers()
            self.wfile.write(response)

        def log_message(self, _format: str, *_args: object) -> None:
            return

    server = ThreadingHTTPServer(("127.0.0.1", 0), PlannerHandler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        plan = request_ai_plan(
            HTTPJsonPlannerTransport(
                f"http://127.0.0.1:{server.server_port}/robot-plan"
            ),
            goal="先走藍線，再走黃線，最後走紫線並安全停止。",
            robot_id="flyto-rover-sim-001",
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)

    assert plan.plan_id == "color-route-demo.v1"
    assert requests[0]["planner_contract"] == "flyto.robotics.planner-request.v1"
    assert requests[0]["robot_id"] == "flyto-rover-sim-001"


def test_conditional_capability_arguments_are_enforced() -> None:
    decoded = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))
    del decoded["steps"][0]["arguments"]["next_color"]

    with pytest.raises(PlanValidationError, match="next_color is required"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="先走藍線",
            robot_id="flyto-rover-sim-001",
        )


def test_motion_plan_requires_terminal_safe_stop() -> None:
    decoded = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))
    decoded["steps"].pop()

    with pytest.raises(PlanValidationError, match="must end with safe_stop"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="沿路線前進但沒有停止條件",
            robot_id="flyto-rover-sim-001",
        )


def test_contradictory_line_transition_is_rejected() -> None:
    decoded = json.loads(EXAMPLE_PLAN.read_text(encoding="utf-8"))
    decoded["steps"][0]["arguments"]["next_color"] = "blue"

    with pytest.raises(PlanValidationError, match="same color"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="模糊且矛盾的路線",
            robot_id="flyto-rover-sim-001",
        )


def test_orphan_resume_is_rejected() -> None:
    decoded = json.loads(CAREFLOW_PLAN.read_text(encoding="utf-8"))
    decoded["steps"] = [
        step for step in decoded["steps"] if step["capability"] != "ask_human"
    ]

    with pytest.raises(PlanValidationError, match="no preceding ask_human"):
        request_ai_plan(
            CallablePlannerTransport(lambda _request: decoded),
            goal="未取得核准就恢復",
            robot_id="flyto-rover-sim-001",
        )


def test_human_gate_requires_matching_external_decision() -> None:
    job = load_job(EXAMPLE_JOB)
    workflow = WorkflowPlan(
        workflow_id="test.human_gate.v1",
        steps=(
            WorkflowStep(
                step_id="approval.ask",
                kind=PrimitiveKind.ASK_HUMAN,
                active_state=MissionState.WAITING_FOR_HUMAN,
                arguments=(
                    ("approval_id", "delivery.test"),
                    ("prompt_key", "confirm.delivery"),
                ),
                timeout_seconds=5.0,
            ),
            WorkflowStep(
                step_id="approval.resume",
                kind=PrimitiveKind.RESUME,
                active_state=MissionState.RESUMING,
                arguments=(("approval_id", "delivery.test"),),
                timeout_seconds=1.0,
            ),
        ),
    )
    controller = MissionController(job, workflow=workflow)
    pose = Pose2D(0.0, 0.0, 0.0)

    waiting = controller.tick(pose, minimum_range=math.inf, now=0.0)
    with pytest.raises(ValueError, match="does not match"):
        controller.submit_human_decision(
            approval_id="delivery.wrong",
            approved=True,
            actor_id="operator.test",
            now=0.1,
        )
    controller.submit_human_decision(
        approval_id="delivery.test",
        approved=True,
        actor_id="operator.test",
        now=0.1,
    )
    controller.tick(pose, minimum_range=math.inf, now=0.1)
    resumed = controller.tick(pose, minimum_range=math.inf, now=0.15)

    assert waiting.reason == "waiting_for_human"
    assert resumed.linear_x == 0.0
    assert controller.state == MissionState.COMPLETED
    kinds = [event.kind for event in controller.events]
    assert "human_approval_requested" in kinds
    assert "human_approved" in kinds
    assert "resume_authorized" in kinds
    assert [event.sequence for event in controller.events] == list(
        range(1, len(controller.events) + 1)
    )


def test_human_decision_rejection_audit_is_bounded() -> None:
    controller = MissionController(load_job(EXAMPLE_JOB))

    for index in range(25):
        controller.record_human_decision_rejection(
            reason=f"replay rejected {index}",
            now=float(index),
        )

    rejected = [
        event
        for event in controller.events
        if event.kind == "human_decision_rejected"
    ]
    assert len(rejected) == 20
    assert controller.human_decision_rejection_count == 20


def test_signed_human_decision_is_bound_fresh_and_single_use() -> None:
    decision = build_signed_human_decision(
        job_id="job.test",
        robot_id="robot.test",
        approval_id="delivery.test",
        approved=True,
        actor_id="operator.test",
        secret=APPROVAL_SECRET,
        issued_at_epoch_seconds=1_000,
        ttl_seconds=60,
        nonce="nonce.test.001",
    )
    authenticator = HumanDecisionAuthenticator(APPROVAL_SECRET)

    verified = authenticator.verify(
        decision_to_json(decision),
        expected_job_id="job.test",
        expected_robot_id="robot.test",
        now_epoch_seconds=1_010,
    )

    assert verified.approval_id == "delivery.test"
    assert verified.actor_id == "operator.test"
    assert verified.approved is True
    with pytest.raises(HumanDecisionValidationError, match="already used"):
        authenticator.verify(
            decision,
            expected_job_id="job.test",
            expected_robot_id="robot.test",
            now_epoch_seconds=1_011,
        )


def test_signed_human_decision_rejects_tampering_scope_and_expiry() -> None:
    decision = build_signed_human_decision(
        job_id="job.test",
        robot_id="robot.test",
        approval_id="delivery.test",
        approved=True,
        actor_id="operator.test",
        secret=APPROVAL_SECRET,
        issued_at_epoch_seconds=1_000,
        ttl_seconds=30,
        nonce="nonce.test.002",
    )
    tampered = dict(decision)
    tampered["approved"] = False

    with pytest.raises(HumanDecisionValidationError, match="signature is invalid"):
        HumanDecisionAuthenticator(APPROVAL_SECRET).verify(
            tampered,
            expected_job_id="job.test",
            expected_robot_id="robot.test",
            now_epoch_seconds=1_010,
        )
    with pytest.raises(HumanDecisionValidationError, match="job_id does not match"):
        HumanDecisionAuthenticator(APPROVAL_SECRET).verify(
            decision,
            expected_job_id="job.other",
            expected_robot_id="robot.test",
            now_epoch_seconds=1_010,
        )
    with pytest.raises(HumanDecisionValidationError, match="has expired"):
        HumanDecisionAuthenticator(APPROVAL_SECRET).verify(
            decision,
            expected_job_id="job.test",
            expected_robot_id="robot.test",
            now_epoch_seconds=1_031,
        )


def test_cli_signs_human_decision_without_secret_in_output(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    capsys: pytest.CaptureFixture[str],
) -> None:
    output = tmp_path / "signed-decision.json"
    monkeypatch.setenv("FLYTO_ROBOTICS_APPROVAL_SECRET", APPROVAL_SECRET)

    exit_code = main(
        [
            "sign-human-decision",
            "--job",
            str(EXAMPLE_JOB),
            "--approval-id",
            "delivery.nurse_station",
            "--actor-id",
            "operator.test",
            "--approve",
            "--output",
            str(output),
        ]
    )
    printed = json.loads(capsys.readouterr().out)
    saved = json.loads(output.read_text(encoding="utf-8"))
    verified = HumanDecisionAuthenticator(APPROVAL_SECRET).verify(
        saved,
        expected_job_id="demo-pharmacy-to-ward-001",
        expected_robot_id="flyto-rover-sim-001",
    )

    assert exit_code == 0
    assert printed == saved
    assert APPROVAL_SECRET not in output.read_text(encoding="utf-8")
    assert verified.actor_id == "operator.test"


def test_color_line_perception_returns_normalized_lateral_error() -> None:
    width = 40
    height = 30
    image = bytearray([210, 210, 210] * width * height)
    for y in range(15, height):
        for x in range(27, 33):
            offset = (y * width + x) * 3
            image[offset : offset + 3] = bytes((8, 30, 245))

    scene = detect_line_scene(image, width=width, height=height, encoding="rgb8")
    blue = scene.get("blue")

    assert blue is not None
    assert blue.visible
    assert blue.confidence > 0.1
    assert 0.3 < blue.lateral_error < 0.7


def test_line_perception_excludes_robot_nose_at_image_bottom() -> None:
    width = 40
    height = 30
    image = bytearray([220, 220, 220] * width * height)
    for y in range(27, height):
        for x in range(width):
            offset = (y * width + x) * 3
            image[offset : offset + 3] = bytes((0, 35, 120))

    scene = detect_line_scene(image, width=width, height=height, encoding="rgb8")
    blue = scene.get("blue")

    assert blue is not None
    assert not blue.visible
    assert blue.pixel_count == 0


def test_ai_plan_dry_run_executes_feedback_and_recovery() -> None:
    plan = load_plan(EXAMPLE_PLAN)
    result = dry_run_plan(EXAMPLE_JOB, EXAMPLE_PLAN)

    assert plan.goal.startswith("先走藍線")
    assert result["status"] == "succeeded"
    assert result["safety_stop_count"] == 1
    assert result["simulation"]["mode"] == "deterministic_capability_dry_run"
    kinds = [event["kind"] for event in result["events"]]
    assert kinds.count("line_acquired") == 3
    assert kinds.count("primitive_completed") == 4
    assert "obstacle_stop" in kinds
    assert "path_clear" in kinds


def test_careflow_plan_executes_clearance_human_gate_and_audit() -> None:
    plan = load_plan(CAREFLOW_PLAN)
    workflow = compile_workflow(plan)
    result = dry_run_plan(EXAMPLE_JOB, CAREFLOW_PLAN)

    assert [step.kind for step in workflow.steps] == [
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.WAIT_UNTIL_CLEAR,
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.ASK_HUMAN,
        PrimitiveKind.RESUME,
        PrimitiveKind.FOLLOW_LINE,
        PrimitiveKind.SAFE_STOP,
    ]
    assert result["status"] == "succeeded"
    assert result["simulation"]["obstacle_injected"] is True
    assert result["simulation"]["human_approval_injected"] is True
    kinds = [event["kind"] for event in result["events"]]
    for expected in (
        "clearance_blocked",
        "clearance_window_started",
        "human_approval_requested",
        "human_approved",
        "resume_authorized",
    ):
        assert expected in kinds
    assert [event["sequence"] for event in result["events"]] == list(
        range(1, len(result["events"]) + 1)
    )
    approval_event = next(
        event for event in result["events"] if event["kind"] == "human_approved"
    )
    assert approval_event["step_id"] == "delivery.ask_nurse"
    assert approval_event["capability"] == "ask_human"
    assert approval_event["actor_id"] == "demo.operator"


def test_waypoint_careflow_plan_is_physics_ready_and_deterministic() -> None:
    plan = load_plan(CAREFLOW_WAYPOINT_PLAN)
    workflow = compile_workflow(plan)
    result = dry_run_plan(EXAMPLE_JOB, CAREFLOW_WAYPOINT_PLAN)

    assert [step.kind for step in workflow.steps] == [
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.WAIT_UNTIL_CLEAR,
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.ASK_HUMAN,
        PrimitiveKind.RESUME,
        PrimitiveKind.NAVIGATE,
        PrimitiveKind.SAFE_STOP,
    ]
    assert result["status"] == "succeeded"
    assert result["simulation"]["human_approval_injected"] is True
    kinds = [event["kind"] for event in result["events"]]
    assert "human_approved" in kinds
    assert "resume_authorized" in kinds


def test_semantic_location_plan_executes_without_llm_coordinates() -> None:
    plan = load_plan(SEMANTIC_PLAN)
    workflow = compile_workflow(
        plan,
        semantic_map=SemanticLocationStore(
            SEMANTIC_MAP,
            map_id="gazebo.atomic-color-route.v1",
        ),
    )
    result = dry_run_plan(
        EXAMPLE_JOB,
        SEMANTIC_PLAN,
        semantic_map_path=SEMANTIC_MAP,
        semantic_map_id="gazebo.atomic-color-route.v1",
    )

    assert [step.kind for step in workflow.steps] == [
        PrimitiveKind.NAVIGATE_TO_LOCATION,
        PrimitiveKind.NAVIGATE_TO_LOCATION,
        PrimitiveKind.NAVIGATE_TO_LOCATION,
        PrimitiveKind.SAFE_STOP,
    ]
    assert all(
        "x" not in call.arguments and "y" not in call.arguments
        for call in plan.steps[:3]
    )
    assert result["status"] == "succeeded"
    assert result["final_pose"]["x"] > 4.2
