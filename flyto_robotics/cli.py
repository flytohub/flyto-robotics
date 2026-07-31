"""Command-line entry points for contract checks and mission execution."""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import xml.etree.ElementTree as ET
from collections.abc import Sequence
from datetime import datetime, timezone
from pathlib import Path

from .ai_planner import (
    HTTPJsonPlannerTransport,
    PlanValidationError,
    compile_workflow,
    load_plan,
    plan_to_dict,
    planner_request,
    request_ai_plan,
)
from .capabilities import GoalFrame, default_capability_registry
from .contracts import JobValidationError, load_job, write_json_atomic
from .guarded_handoff import load_policy, load_script
from .human_approval import (
    HumanDecisionValidationError,
    build_signed_human_decision,
    decision_to_json,
)
from .lab import (
    evaluate_lab_result,
    load_lab_scenario,
    render_lab_junit,
    render_lab_markdown,
    write_text_atomic,
)
from .line_perception import LineObservation, LineScene
from .matrix import (
    aggregate_lab_reports,
    render_matrix_junit,
    render_matrix_markdown,
)
from .mission import MissionController, Pose2D, normalize_angle
from .qr_confirmation import (
    QRConfirmationAuthenticator,
    QRConfirmationValidationError,
    build_signed_qr_confirmation,
    qr_confirmation_to_human_decision,
    qr_token_sha256,
)
from .resource_binding import load_resource_plan
from .semantic_map import SemanticLocationStore, parse_semantic_location_map
from .soak import (
    render_soak_junit,
    render_soak_markdown,
    run_deterministic_soak,
)
from .workflow import PrimitiveKind

PROJECT_ROOT = Path(__file__).resolve().parent.parent


def _timestamp() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _load_json(path: Path) -> object:
    return json.loads(path.read_text(encoding="utf-8"))


def _load_goal_frame(path: Path) -> dict[str, object]:
    try:
        if path.stat().st_size > 64 * 1024:
            raise PlanValidationError("goal frame file exceeds 65536 bytes")
        decoded = _load_json(path)
    except PlanValidationError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise PlanValidationError(
            "goal frame file must contain readable UTF-8 JSON"
        ) from exc
    if not isinstance(decoded, dict):
        raise PlanValidationError("goal frame file must contain one JSON object")
    try:
        GoalFrame.from_mapping(decoded)
    except ValueError as exc:
        raise PlanValidationError(str(exc)) from exc
    return decoded


def validate_assets(root: Path = PROJECT_ROOT) -> list[str]:
    """Validate static assets without importing ROS or Gazebo."""
    checked: list[str] = []
    xml_paths = [
        root / "package.xml",
        root / "models/flyto_rover/model.config",
        root / "models/flyto_rover/model.sdf",
        root / "worlds/hospital-logistics.sdf",
        root / "worlds/atomic-color-route.sdf",
        root / "worlds/atomic-color-route-lab.sdf",
    ]
    for path in xml_paths:
        ET.parse(path)
        checked.append(str(path.relative_to(root)))

    json_paths = [
        root / "contracts/job-v1.schema.json",
        root / "contracts/result-v1.schema.json",
        root / "contracts/shortcut-result-v1.schema.json",
        root / "contracts/facility-resource-plan-v1.schema.json",
        root / "contracts/plan-v1.schema.json",
        root / "contracts/input-event-v1.schema.json",
        root / "contracts/human-decision-v1.schema.json",
        root / "contracts/qr-confirmation-v1.schema.json",
        root / "contracts/guarded-handoff-policy-v1.schema.json",
        root / "contracts/guarded-handoff-script-v1.schema.json",
        root / "contracts/guarded-handoff-evidence-v1.schema.json",
        root / "contracts/capability-manifest-v1.schema.json",
        root / "contracts/capability-route-v1.schema.json",
        root / "contracts/goal-frame-v1.schema.json",
        root / "contracts/semantic-location-catalog-v1.schema.json",
        root / "contracts/semantic-location-map-v1.schema.json",
        root / "contracts/lab-scenario-v1.schema.json",
        root / "contracts/lab-matrix-v1.schema.json",
        root / "contracts/soak-report-v1.schema.json",
    ]
    for path in json_paths:
        decoded = _load_json(path)
        if not isinstance(decoded, dict) or "$schema" not in decoded:
            raise ValueError(f"{path.name} must be a JSON Schema object")
        checked.append(str(path.relative_to(root)))

    for semantic_map in sorted((root / "examples/maps").glob("*.json")):
        parse_semantic_location_map(_load_json(semantic_map))
        checked.append(str(semantic_map.relative_to(root)))
    for goal_frame in sorted((root / "examples/goal-frames").glob("*.json")):
        decoded_frame = _load_json(goal_frame)
        if not isinstance(decoded_frame, dict):
            raise ValueError(f"{goal_frame.name} must contain a Goal Frame object")
        GoalFrame.from_mapping(decoded_frame)
        checked.append(str(goal_frame.relative_to(root)))
    for scenario_path in sorted((root / "scenarios/gazebo").glob("*.json")):
        load_lab_scenario(scenario_path, project_root=root)
        checked.append(str(scenario_path.relative_to(root)))

    example = root / "examples/jobs/pharmacy-to-ward.json"
    load_job(example)
    checked.append(str(example.relative_to(root)))
    for example_plan in sorted((root / "examples/plans").glob("*.json")):
        load_plan(example_plan)
        checked.append(str(example_plan.relative_to(root)))
    for resource_plan in sorted((root / "examples/resource-plans").glob("*.json")):
        load_resource_plan(resource_plan)
        checked.append(str(resource_plan.relative_to(root)))
    for policy_path in sorted(
        (root / "examples/guarded-handoff").glob("*-policy.json")
    ):
        load_policy(policy_path)
        checked.append(str(policy_path.relative_to(root)))
    for script_path in sorted(
        (root / "examples/guarded-handoff").glob("*-script.json")
    ):
        load_script(script_path)
        checked.append(str(script_path.relative_to(root)))

    bridge = (root / "config/bridge.yaml").read_text(encoding="utf-8")
    for required in (
        "/clock",
        "/flyto/cmd_vel",
        "/flyto/odom",
        "/flyto/scan",
        "/flyto/camera/image",
        "/flyto/evidence/overhead",
        "/flyto/ground_truth",
        "ros_type_name",
        "gz_type_name",
    ):
        if required not in bridge:
            raise ValueError(f"bridge.yaml is missing {required}")
    checked.append("config/bridge.yaml")

    launch_path = root / "launch/hospital_demo.launch.py"
    ai_launch_path = root / "launch/atomic_ai_demo.launch.py"
    lab_launch_path = root / "launch/gazebo_lab.launch.py"
    shortcut_launch_path = root / "launch/shortcut_gazebo_demo.launch.py"
    showcase_launch_path = root / "launch/ai4all_showcase.launch.py"
    medication_showcase_launch_path = (
        root / "launch/ai4all_medication_showcase.launch.py"
    )
    for path in (
        launch_path,
        ai_launch_path,
        lab_launch_path,
        shortcut_launch_path,
        showcase_launch_path,
        medication_showcase_launch_path,
    ):
        compile(path.read_text(encoding="utf-8"), str(path), "exec")
        checked.append(str(path.relative_to(root)))
    return checked


def dry_run(job_path: Path, output_path: Path | None = None) -> dict[str, object]:
    """Run the real controller against deterministic planar kinematics."""
    job = load_job(job_path)
    controller = MissionController(job)
    pose = Pose2D(0.0, 0.0, 0.0)
    now = 0.0
    timestep = 0.05
    obstacle_injected = False
    maximum_steps = math.ceil((job.safety.mission_timeout_seconds + 1.0) / timestep)

    for _ in range(maximum_steps):
        # One deterministic two-second obstruction proves stop and recovery.
        obstructed = 2.0 <= now < 4.0
        obstacle_injected = obstacle_injected or obstructed
        minimum_range = 0.25 if obstructed else math.inf
        command = controller.tick(pose, minimum_range=minimum_range, now=now)
        pose = Pose2D(
            x=pose.x + command.linear_x * math.cos(pose.yaw) * timestep,
            y=pose.y + command.linear_x * math.sin(pose.yaw) * timestep,
            yaw=normalize_angle(pose.yaw + command.angular_z * timestep),
        )
        now += timestep
        if controller.terminal:
            break

    if not controller.terminal:
        controller.fail("dry_run_step_limit", now)
    result = controller.result(generated_at=_timestamp(), now=now, pose=pose)
    result["simulation"] = {
        "mode": "deterministic_planar_dry_run",
        "obstacle_injected": obstacle_injected,
        "gazebo_physics": False,
    }
    if output_path:
        write_json_atomic(output_path, result)
    return result


def _synthetic_line_scene(
    target_color: str,
    *,
    lateral_error: float,
    next_color: str | None = None,
    visible: bool = True,
) -> LineScene:
    detections = [
        LineObservation(
            color=target_color,
            visible=visible,
            confidence=0.8 if visible else 0.0,
            lateral_error=lateral_error,
            pixel_count=120 if visible else 0,
        )
    ]
    if next_color is not None:
        detections.append(
            LineObservation(
                color=next_color,
                visible=True,
                confidence=0.35,
                lateral_error=0.1,
                pixel_count=52,
            )
        )
    return LineScene(tuple(detections))


def dry_run_plan(
    job_path: Path,
    plan_path: Path,
    output_path: Path | None = None,
    *,
    semantic_map_path: Path | None = None,
    semantic_map_id: str | None = None,
) -> dict[str, object]:
    """Exercise a validated AI plan against deterministic capability observations."""
    job = load_job(job_path)
    plan = load_plan(plan_path)
    if plan.robot_id != job.robot_id:
        raise PlanValidationError("plan.robot_id must match job.robot_id")
    semantic_map = _semantic_map_store(semantic_map_path, semantic_map_id)
    controller = MissionController(
        job,
        workflow=compile_workflow(plan, semantic_map=semantic_map),
        semantic_map_store=semantic_map,
    )
    pose = Pose2D(0.0, 0.0, 0.0)
    now = 0.0
    timestep = 0.05
    maximum_steps = math.ceil((job.safety.mission_timeout_seconds + 1.0) / timestep)
    obstacle_injected = False
    human_approval_injected = False

    for _ in range(maximum_steps):
        line_scene = None
        minimum_range = math.inf
        if controller.step_index >= 0 and not controller.terminal:
            step = controller.workflow.steps[controller.step_index]
            if step.kind == PrimitiveKind.FOLLOW_LINE:
                active_for = now - controller.state_entered_at
                color = str(step.argument("color"))
                completion = str(step.argument("completion", "line_end"))
                next_color = step.argument("next_color")
                show_transition = completion == "next_color" and active_for >= 1.1
                show_target = not (completion == "line_end" and active_for >= 1.1)
                line_scene = _synthetic_line_scene(
                    color,
                    lateral_error=0.18 * math.sin(now * 2.0),
                    next_color=(
                        str(next_color)
                        if show_transition and isinstance(next_color, str)
                        else None
                    ),
                    visible=show_target,
                )
                if 0.45 <= now <= 0.7:
                    minimum_range = 0.25
                    obstacle_injected = True
            elif step.kind == PrimitiveKind.WAIT_UNTIL_CLEAR:
                active_for = now - controller.state_entered_at
                if active_for < 0.45:
                    minimum_range = 0.25
                    obstacle_injected = True
            elif step.kind == PrimitiveKind.ASK_HUMAN:
                active_for = now - controller.state_entered_at
                approval_id = str(step.argument("approval_id"))
                if active_for >= 0.25 and approval_id not in controller.human_decisions:
                    controller.submit_human_decision(
                        approval_id=approval_id,
                        approved=True,
                        actor_id="demo.operator",
                        now=now,
                    )
                    human_approval_injected = True
        command = controller.tick(
            pose,
            minimum_range=minimum_range,
            now=now,
            line_scene=line_scene,
        )
        pose = Pose2D(
            x=pose.x + command.linear_x * math.cos(pose.yaw) * timestep,
            y=pose.y + command.linear_x * math.sin(pose.yaw) * timestep,
            yaw=normalize_angle(pose.yaw + command.angular_z * timestep),
        )
        now += timestep
        if controller.terminal:
            break

    if not controller.terminal:
        controller.fail("dry_run_step_limit", now)
    result = controller.result(generated_at=_timestamp(), now=now, pose=pose)
    result["simulation"] = {
        "mode": "deterministic_capability_dry_run",
        "obstacle_injected": obstacle_injected,
        "human_approval_injected": human_approval_injected,
        "gazebo_physics": False,
    }
    if output_path:
        write_json_atomic(output_path, result)
    return result


def _semantic_map_store(
    path: Path | None,
    map_id: str | None,
) -> SemanticLocationStore | None:
    if (path is None) != (map_id is None):
        raise PlanValidationError(
            "semantic map path and semantic map ID must be provided together"
        )
    if path is None or map_id is None:
        return None
    store = SemanticLocationStore(path, map_id=map_id)
    store.load()
    return store


def _add_semantic_planning_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument(
        "--semantic-map",
        type=Path,
        help="Trusted full semantic-location map; poses are hidden from the planner",
    )
    parser.add_argument(
        "--semantic-map-id",
        help="Expected physical map identity",
    )
    parser.add_argument(
        "--goal-frame",
        type=Path,
        help="Language-neutral flyto.goal-frame.v1 JSON emitted by Flyto AI",
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="flyto-robotics")
    subcommands = parser.add_subparsers(dest="command", required=True)

    validate_job = subcommands.add_parser("validate-job", help="validate a job contract")
    validate_job.add_argument("job", type=Path)

    subcommands.add_parser("validate-assets", help="validate bundled simulation assets")

    dry = subcommands.add_parser("dry-run", help="run deterministic closed-loop kinematics")
    dry.add_argument("job", type=Path)
    dry.add_argument("--output", type=Path)

    validate_plan = subcommands.add_parser(
        "validate-plan",
        help="validate untrusted AI plan JSON and registered capability calls",
    )
    validate_plan.add_argument("plan", type=Path)

    subcommands.add_parser(
        "show-capabilities",
        help="print the machine-readable capability catalog exposed to AI",
    )

    request = subcommands.add_parser(
        "planner-request",
        help="build the provider-neutral request payload for an LLM",
    )
    request.add_argument("--goal", required=True)
    request.add_argument("--robot-id", required=True)
    _add_semantic_planning_arguments(request)

    plan_ai = subcommands.add_parser(
        "plan-ai",
        help="ask a Flyto or local HTTPS planner to compose and validate a plan",
    )
    plan_ai.add_argument("--goal", required=True)
    plan_ai.add_argument("--robot-id", required=True)
    plan_ai.add_argument("--output", required=True, type=Path)
    plan_ai.add_argument(
        "--planner-url",
        default=os.environ.get("FLYTO_ROBOTICS_PLANNER_URL", ""),
    )
    _add_semantic_planning_arguments(plan_ai)

    dry_plan = subcommands.add_parser(
        "dry-run-plan",
        help="run an AI-composed plan with deterministic observations",
    )
    dry_plan.add_argument("--job", required=True, type=Path)
    dry_plan.add_argument("--plan", required=True, type=Path)
    dry_plan.add_argument("--output", type=Path)
    dry_plan.add_argument("--semantic-map", type=Path)
    dry_plan.add_argument("--semantic-map-id")

    sign_decision = subcommands.add_parser(
        "sign-human-decision",
        help="create a short-lived signed approval for the ROS human gate",
    )
    sign_decision.add_argument("--job", required=True, type=Path)
    sign_decision.add_argument("--approval-id", required=True)
    sign_decision.add_argument("--actor-id", required=True)
    decision = sign_decision.add_mutually_exclusive_group(required=True)
    decision.add_argument("--approve", dest="approved", action="store_true")
    decision.add_argument("--deny", dest="approved", action="store_false")
    sign_decision.add_argument("--ttl-seconds", type=int, default=60)
    sign_decision.add_argument("--output", type=Path)

    sign_qr = subcommands.add_parser(
        "sign-delivery-qr",
        help="create a short-lived, signed delivery confirmation QR payload",
    )
    sign_qr.add_argument("--job", required=True, type=Path)
    sign_qr.add_argument("--approval-id", required=True)
    sign_qr.add_argument("--recipient-ref", required=True)
    sign_qr.add_argument("--ttl-seconds", type=int, default=120)
    sign_qr.add_argument("--output", type=Path)

    verify_qr = subcommands.add_parser(
        "verify-delivery-qr",
        help="verify a scanned QR and convert it to the existing human gate",
    )
    verify_qr.add_argument("--job", required=True, type=Path)
    verify_qr.add_argument("--approval-id", required=True)
    verify_qr.add_argument("--recipient-ref")
    verify_qr.add_argument("--token-file", required=True, type=Path)
    verify_qr.add_argument("--output", type=Path)

    validate_lab = subcommands.add_parser(
        "validate-lab-scenario",
        help="validate a Gazebo lab scenario and every referenced asset",
    )
    validate_lab.add_argument("scenario", type=Path)

    evaluate_lab = subcommands.add_parser(
        "evaluate-lab",
        help="evaluate Gazebo result, safety evidence, images, and provenance",
    )
    evaluate_lab.add_argument("--scenario", required=True, type=Path)
    evaluate_lab.add_argument("--result", required=True, type=Path)
    evaluate_lab.add_argument("--evidence-dir", required=True, type=Path)
    evaluate_lab.add_argument("--report", required=True, type=Path)
    evaluate_lab.add_argument("--markdown", required=True, type=Path)
    evaluate_lab.add_argument("--junit", required=True, type=Path)

    soak_plan = subcommands.add_parser(
        "soak-plan",
        help="repeat one AI-composed plan and prove deterministic results",
    )
    soak_plan.add_argument("--job", required=True, type=Path)
    soak_plan.add_argument("--plan", required=True, type=Path)
    soak_plan.add_argument("--runs", type=int, default=50)
    soak_plan.add_argument("--output-dir", required=True, type=Path)

    aggregate_lab = subcommands.add_parser(
        "aggregate-lab",
        help="aggregate independent Gazebo lab reports into one strict matrix",
    )
    aggregate_lab.add_argument("--reports", required=True, nargs="+", type=Path)
    aggregate_lab.add_argument("--report", required=True, type=Path)
    aggregate_lab.add_argument("--markdown", required=True, type=Path)
    aggregate_lab.add_argument("--junit", required=True, type=Path)

    ros = subcommands.add_parser("run-ros", help="run the ROS 2 mission adapter")
    ros.add_argument("--job", required=True, type=Path)
    ros.add_argument("--result", required=True, type=Path)
    ros.add_argument("--plan", type=Path)
    ros.add_argument("--semantic-map", type=Path)
    ros.add_argument("--semantic-map-id")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        if args.command == "validate-job":
            job = load_job(args.job)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "contract_version": job.contract_version,
                        "job_id": job.job_id,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "validate-assets":
            checked = validate_assets()
            print(json.dumps({"ok": True, "checked": checked}, sort_keys=True))
            return 0
        if args.command == "validate-plan":
            plan = load_plan(args.plan)
            print(json.dumps({"ok": True, "plan": plan_to_dict(plan)}, ensure_ascii=False))
            return 0
        if args.command == "show-capabilities":
            print(
                json.dumps(
                    {"capabilities": default_capability_registry().catalog()},
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "planner-request":
            semantic_map = _semantic_map_store(
                args.semantic_map,
                args.semantic_map_id,
            )
            goal_frame = _load_goal_frame(args.goal_frame) if args.goal_frame else None
            print(
                json.dumps(
                    planner_request(
                        goal=args.goal,
                        robot_id=args.robot_id,
                        goal_frame=goal_frame,
                        semantic_map=semantic_map,
                    ),
                    ensure_ascii=False,
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "plan-ai":
            if not args.planner_url:
                raise PlanValidationError(
                    "planner URL is required via --planner-url or "
                    "FLYTO_ROBOTICS_PLANNER_URL"
                )
            transport = HTTPJsonPlannerTransport(
                args.planner_url,
                bearer_token=os.environ.get("FLYTO_ROBOTICS_PLANNER_TOKEN") or None,
            )
            semantic_map = _semantic_map_store(
                args.semantic_map,
                args.semantic_map_id,
            )
            goal_frame = _load_goal_frame(args.goal_frame) if args.goal_frame else None
            plan = request_ai_plan(
                transport,
                goal=args.goal,
                robot_id=args.robot_id,
                goal_frame=goal_frame,
                semantic_map=semantic_map,
            )
            decoded_plan = plan_to_dict(plan)
            write_json_atomic(args.output, decoded_plan)
            print(json.dumps({"ok": True, "plan": decoded_plan}, ensure_ascii=False))
            return 0
        if args.command == "dry-run":
            result = dry_run(args.job, args.output)
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            return 0 if result["status"] == "succeeded" else 3
        if args.command == "dry-run-plan":
            result = dry_run_plan(
                args.job,
                args.plan,
                args.output,
                semantic_map_path=args.semantic_map,
                semantic_map_id=args.semantic_map_id,
            )
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            return 0 if result["status"] == "succeeded" else 3
        if args.command == "sign-human-decision":
            secret = os.environ.get("FLYTO_ROBOTICS_APPROVAL_SECRET", "")
            if not secret:
                raise HumanDecisionValidationError(
                    "FLYTO_ROBOTICS_APPROVAL_SECRET is required"
                )
            job = load_job(args.job)
            decision = build_signed_human_decision(
                job_id=job.job_id,
                robot_id=job.robot_id,
                approval_id=args.approval_id,
                approved=args.approved,
                actor_id=args.actor_id,
                secret=secret,
                ttl_seconds=args.ttl_seconds,
            )
            if args.output:
                write_json_atomic(args.output, decision)
            print(decision_to_json(decision))
            return 0
        if args.command == "sign-delivery-qr":
            secret = os.environ.get("FLYTO_ROBOTICS_QR_SECRET", "")
            if not secret:
                raise QRConfirmationValidationError(
                    "FLYTO_ROBOTICS_QR_SECRET is required"
                )
            job = load_job(args.job)
            token = build_signed_qr_confirmation(
                job_id=job.job_id,
                robot_id=job.robot_id,
                approval_id=args.approval_id,
                recipient_ref=args.recipient_ref,
                secret=secret,
                ttl_seconds=args.ttl_seconds,
            )
            if args.output:
                write_text_atomic(args.output, token + "\n")
            print(token)
            return 0
        if args.command == "verify-delivery-qr":
            qr_secret = os.environ.get("FLYTO_ROBOTICS_QR_SECRET", "")
            approval_secret = os.environ.get(
                "FLYTO_ROBOTICS_APPROVAL_SECRET",
                "",
            )
            if not qr_secret:
                raise QRConfirmationValidationError(
                    "FLYTO_ROBOTICS_QR_SECRET is required"
                )
            if not approval_secret:
                raise HumanDecisionValidationError(
                    "FLYTO_ROBOTICS_APPROVAL_SECRET is required"
                )
            if args.token_file.stat().st_size > 16 * 1024:
                raise QRConfirmationValidationError(
                    "QR confirmation is too large"
                )
            token = args.token_file.read_text(encoding="utf-8").strip()
            job = load_job(args.job)
            confirmation = QRConfirmationAuthenticator(qr_secret).verify(
                token,
                expected_job_id=job.job_id,
                expected_robot_id=job.robot_id,
                expected_approval_id=args.approval_id,
                expected_recipient_ref=args.recipient_ref,
            )
            decision = qr_confirmation_to_human_decision(
                confirmation,
                approval_secret=approval_secret,
            )
            result = {
                "ok": True,
                "confirmation": {
                    "contract_version": "flyto.robotics.qr-verification.v1",
                    "confirmation_id": confirmation.confirmation_id,
                    "job_id": confirmation.job_id,
                    "robot_id": confirmation.robot_id,
                    "approval_id": confirmation.approval_id,
                    "recipient_ref": confirmation.recipient_ref,
                    "expires_at_epoch_seconds": (
                        confirmation.expires_at_epoch_seconds
                    ),
                    "token_sha256": qr_token_sha256(token),
                },
                "human_decision": decision,
            }
            if args.output:
                write_json_atomic(args.output, result)
            print(json.dumps(result, ensure_ascii=False, sort_keys=True))
            return 0
        if args.command == "validate-lab-scenario":
            scenario = load_lab_scenario(args.scenario, project_root=PROJECT_ROOT)
            print(
                json.dumps(
                    {
                        "ok": True,
                        "contract_version": scenario.contract_version,
                        "scenario_id": scenario.scenario_id,
                        "soak_runs": scenario.soak_runs,
                    },
                    sort_keys=True,
                )
            )
            return 0
        if args.command == "evaluate-lab":
            scenario = load_lab_scenario(args.scenario, project_root=PROJECT_ROOT)
            result = _load_json(args.result)
            report = evaluate_lab_result(
                scenario,
                result,
                project_root=PROJECT_ROOT,
                result_path=args.result,
                evidence_dir=args.evidence_dir,
            )
            write_json_atomic(args.report, report)
            write_text_atomic(args.markdown, render_lab_markdown(report))
            write_text_atomic(args.junit, render_lab_junit(report))
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
            return 0 if report["passed"] is True else 4
        if args.command == "soak-plan":
            report = run_deterministic_soak(
                runs=args.runs,
                run_once=lambda: dry_run_plan(args.job, args.plan),
            )
            write_json_atomic(args.output_dir / "report.json", report)
            write_text_atomic(
                args.output_dir / "report.md",
                render_soak_markdown(report),
            )
            write_text_atomic(
                args.output_dir / "junit.xml",
                render_soak_junit(report),
            )
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
            return 0 if report["passed"] is True else 4
        if args.command == "aggregate-lab":
            report = aggregate_lab_reports(args.reports)
            write_json_atomic(args.report, report)
            write_text_atomic(args.markdown, render_matrix_markdown(report))
            write_text_atomic(args.junit, render_matrix_junit(report))
            print(json.dumps(report, ensure_ascii=False, sort_keys=True))
            return 0 if report["passed"] is True else 4
        if args.command == "run-ros":
            from .ros2_node import run

            return run(
                args.job,
                args.result,
                plan_path=args.plan,
                semantic_map_path=args.semantic_map,
                semantic_map_id=args.semantic_map_id,
            )
    except (
        HumanDecisionValidationError,
        QRConfirmationValidationError,
        JobValidationError,
        PlanValidationError,
        OSError,
        ValueError,
    ) as exc:
        print(json.dumps({"ok": False, "error": str(exc)}), file=sys.stderr)
        return 2
    parser.error("unsupported command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
