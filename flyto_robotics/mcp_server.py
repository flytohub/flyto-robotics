"""Detachable, fail-closed MCP boundary for Flyto2 Robotics.

The server exposes semantic planning and deterministic simulation only. It has
no raw actuator, ROS topic, shell, arbitrary file, or network tool.
"""

from __future__ import annotations

import json
import sys
import tempfile
from collections.abc import Callable
from pathlib import Path
from typing import Any

from .ai_planner import (
    PlanValidationError,
    compile_workflow,
    parse_planner_response,
    plan_to_dict,
    planner_request,
)
from .capabilities import CapabilityValidationError, default_capability_registry
from .cli import dry_run_plan
from .contracts import JobValidationError, parse_job
from .ros2_execution import authorize_ros2_execution
from .ros2_pairing import (
    parse_ros2_runtime_snapshot,
    ros2_profile_summary,
    standard_ros2_adapter_manifest,
    verify_ros2_pairing,
)
from .semantic_map import SemanticMapValidationError

PROTOCOL_VERSION = "2025-06-18"
SERVER_NAME = "flyto2-robotics"
SERVER_VERSION = "0.3.0"
MAX_REQUEST_BYTES = 512 * 1024
MAX_ARGUMENT_BYTES = 384 * 1024
MAX_OBSERVATION_ITEMS = 64


def _tool(
    name: str,
    description: str,
    properties: dict[str, Any],
    required: list[str] | None = None,
) -> dict[str, Any]:
    schema: dict[str, Any] = {
        "type": "object",
        "properties": properties,
        "additionalProperties": False,
    }
    if required:
        schema["required"] = required
    return {"name": name, "description": description, "inputSchema": schema}


def tool_definitions() -> list[dict[str, Any]]:
    """Return the stable, bounded Robot MCP surface."""
    object_schema = {"type": "object"}
    return [
        _tool(
            "robot.capabilities.list",
            "List registered semantic robot capabilities and their safety contracts.",
            {},
        ),
        _tool(
            "robot.plan.prepare",
            "Build a provider-neutral, coordinate-bounded request for an AI planner.",
            {
                "goal": {"type": "string", "minLength": 1, "maxLength": 2000},
                "robot_id": {"type": "string", "minLength": 1, "maxLength": 128},
                "observations": object_schema,
                "goal_frame": object_schema,
                "routing_context": object_schema,
                "route_limit": {"type": "integer", "minimum": 1, "maximum": 32},
            },
            ["goal", "robot_id"],
        ),
        _tool(
            "robot.plan.validate",
            "Strictly validate and compile an untrusted semantic robot plan.",
            {"plan": object_schema},
            ["plan"],
        ),
        _tool(
            "robot.mission.dry_run",
            "Run a validated job and plan through the real deterministic mission controller.",
            {"job": object_schema, "plan": object_schema},
            ["job", "plan"],
        ),
        _tool(
            "robot.ros2.profile",
            "Return the safe public view of the standard semantic ROS 2 profile.",
            {
                "robot_id": {"type": "string", "minLength": 1, "maxLength": 128},
            },
            ["robot_id"],
        ),
        _tool(
            "robot.ros2.readiness.verify",
            "Fail closed unless the standard semantic ROS 2 profile is ready.",
            {"runtime": object_schema},
            ["runtime"],
        ),
        _tool(
            "robot.ros2.execution.authorize",
            "Issue a short-lived grant for one pre-authorized semantic binding.",
            {
                "resource_plan": object_schema,
                "runtime": object_schema,
                "workflow_id": {"type": "string", "minLength": 1, "maxLength": 192},
                "resource_id": {"type": "string", "minLength": 1, "maxLength": 192},
                "capability_id": {"type": "string", "minLength": 1, "maxLength": 192},
                "target_space_id": {"type": "string", "minLength": 1, "maxLength": 192},
                "confirmed": {"type": "boolean"},
            },
            [
                "resource_plan",
                "runtime",
                "workflow_id",
                "resource_id",
                "capability_id",
                "target_space_id",
            ],
        ),
    ]


def _arguments(value: object) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ValueError("arguments must be an object")
    encoded = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode()
    if len(encoded) > MAX_ARGUMENT_BYTES:
        raise ValueError("arguments are too large")
    return value


def _exact_fields(
    data: dict[str, Any],
    *,
    allowed: set[str],
    required: set[str],
) -> None:
    unknown = sorted(set(data) - allowed)
    missing = sorted(required - set(data))
    if unknown:
        raise ValueError("unsupported argument fields: " + ", ".join(unknown))
    if missing:
        raise ValueError("missing argument fields: " + ", ".join(missing))


def _capabilities(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(arguments, allowed=set(), required=set())
    registry = default_capability_registry()
    return {
        "contract_version": "flyto.robotics.mcp-capabilities.v1",
        "registry_snapshot": registry.snapshot_hash(),
        "capabilities": registry.catalog(),
    }


def _prepare(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(
        arguments,
        allowed={
            "goal",
            "robot_id",
            "observations",
            "goal_frame",
            "routing_context",
            "route_limit",
        },
        required={"goal", "robot_id"},
    )
    observations = arguments.get("observations")
    if observations is not None:
        if not isinstance(observations, dict):
            raise ValueError("observations must be an object")
        if len(observations) > MAX_OBSERVATION_ITEMS:
            raise ValueError("observations has too many fields")
    route_limit = arguments.get("route_limit", 8)
    if isinstance(route_limit, bool) or not isinstance(route_limit, int):
        raise ValueError("route_limit must be an integer")
    request = planner_request(
        goal=arguments["goal"],
        robot_id=arguments["robot_id"],
        observations=observations,
        goal_frame=arguments.get("goal_frame"),
        routing_context=arguments.get("routing_context"),
        route_limit=route_limit,
    )
    return {
        "contract_version": "flyto.robotics.mcp-plan-request.v1",
        "request": request,
    }


def _workflow_payload(plan_value: object) -> tuple[dict[str, Any], object]:
    plan = parse_planner_response(plan_value)
    workflow = compile_workflow(plan)
    return (
        {
            "contract_version": "flyto.robotics.mcp-plan-validation.v1",
            "plan": plan_to_dict(plan),
            "workflow": {
                "workflow_id": workflow.workflow_id,
                "goal": workflow.goal,
                "source_kind": workflow.source_kind,
                "steps": [
                    {
                        "step_id": step.step_id,
                        "kind": step.kind.value,
                        "timeout_seconds": step.timeout_seconds,
                        "on_failure": step.on_failure,
                    }
                    for step in workflow.steps
                ],
            },
        },
        plan,
    )


def _validate(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(arguments, allowed={"plan"}, required={"plan"})
    payload, _ = _workflow_payload(arguments["plan"])
    return payload


def _dry_run(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(arguments, allowed={"job", "plan"}, required={"job", "plan"})
    job = parse_job(arguments["job"])
    validation, plan = _workflow_payload(arguments["plan"])
    if plan.robot_id != job.robot_id:
        raise PlanValidationError("plan.robot_id must match job.robot_id")

    with tempfile.TemporaryDirectory(prefix="flyto2-robot-mcp-") as directory:
        root = Path(directory)
        job_path = root / "job.json"
        plan_path = root / "plan.json"
        job_path.write_text(
            json.dumps(arguments["job"], ensure_ascii=False), encoding="utf-8"
        )
        plan_path.write_text(
            json.dumps(arguments["plan"], ensure_ascii=False), encoding="utf-8"
        )
        result = dry_run_plan(job_path, plan_path)

    return {
        "contract_version": "flyto.robotics.mcp-dry-run.v1",
        "validation": validation,
        "result": result,
    }


def _ros2_profile(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(arguments, allowed={"robot_id"}, required={"robot_id"})
    return ros2_profile_summary(
        standard_ros2_adapter_manifest(arguments["robot_id"])
    )


def _ros2_readiness(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(arguments, allowed={"runtime"}, required={"runtime"})
    runtime = parse_ros2_runtime_snapshot(arguments["runtime"])
    manifest = standard_ros2_adapter_manifest(runtime["robot_id"])
    return verify_ros2_pairing(manifest, runtime)


def _ros2_authorize(arguments: dict[str, Any]) -> dict[str, Any]:
    _exact_fields(
        arguments,
        allowed={
            "resource_plan",
            "runtime",
            "workflow_id",
            "resource_id",
            "capability_id",
            "target_space_id",
            "confirmed",
        },
        required={
            "resource_plan",
            "runtime",
            "workflow_id",
            "resource_id",
            "capability_id",
            "target_space_id",
        },
    )
    confirmed = arguments.get("confirmed", False)
    if type(confirmed) is not bool:
        raise ValueError("confirmed must be boolean")
    runtime = parse_ros2_runtime_snapshot(arguments["runtime"])
    manifest = standard_ros2_adapter_manifest(runtime["robot_id"])
    return authorize_ros2_execution(
        resource_plan=arguments["resource_plan"],
        manifest=manifest,
        runtime=runtime,
        workflow_id=arguments["workflow_id"],
        resource_id=arguments["resource_id"],
        capability_id=arguments["capability_id"],
        target_space_id=arguments["target_space_id"],
        confirmed=confirmed,
    )


TOOLS: dict[str, Callable[[dict[str, Any]], dict[str, Any]]] = {
    "robot.capabilities.list": _capabilities,
    "robot.plan.prepare": _prepare,
    "robot.plan.validate": _validate,
    "robot.mission.dry_run": _dry_run,
    "robot.ros2.profile": _ros2_profile,
    "robot.ros2.readiness.verify": _ros2_readiness,
    "robot.ros2.execution.authorize": _ros2_authorize,
}


def _result(request_id: object, result: object) -> dict[str, Any]:
    return {"jsonrpc": "2.0", "id": request_id, "result": result}


def _error(request_id: object, code: int, message: str) -> dict[str, Any]:
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "error": {"code": code, "message": message},
    }


def _tool_result(payload: object, *, is_error: bool = False) -> dict[str, Any]:
    content = json.dumps(payload, ensure_ascii=False, sort_keys=True)
    result: dict[str, Any] = {
        "content": [{"type": "text", "text": content}],
        "isError": is_error,
    }
    if not is_error:
        result["structuredContent"] = payload
    return result


def handle_request(request: object) -> dict[str, Any] | None:
    """Handle one decoded JSON-RPC request without performing transport I/O."""
    if not isinstance(request, dict):
        return _error(None, -32600, "invalid request")
    request_id = request.get("id")
    if request.get("jsonrpc") != "2.0" or not isinstance(request.get("method"), str):
        return _error(request_id, -32600, "invalid request")
    method = request["method"]
    params = request.get("params", {})
    if method.startswith("notifications/"):
        return None
    if not isinstance(params, dict):
        return _error(request_id, -32602, "params must be an object")
    if method == "initialize":
        version = params.get("protocolVersion")
        if version != PROTOCOL_VERSION:
            return _error(request_id, -32602, "unsupported protocol version")
        return _result(
            request_id,
            {
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {"tools": {"listChanged": False}},
                "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
            },
        )
    if method == "tools/list":
        return _result(request_id, {"tools": tool_definitions()})
    if method != "tools/call":
        return _error(request_id, -32601, "method not found")
    name = params.get("name")
    if not isinstance(name, str) or name not in TOOLS:
        return _error(request_id, -32602, "unknown tool")
    try:
        arguments = _arguments(params.get("arguments", {}))
        payload = TOOLS[name](arguments)
    except (
        CapabilityValidationError,
        JobValidationError,
        PlanValidationError,
        SemanticMapValidationError,
        TypeError,
        ValueError,
    ) as exc:
        return _result(
            request_id,
            _tool_result(
                {"error": str(exc), "error_type": type(exc).__name__},
                is_error=True,
            ),
        )
    except Exception:
        return _result(
            request_id,
            _tool_result(
                {"error": "internal tool failure", "error_type": "InternalError"},
                is_error=True,
            ),
        )
    return _result(request_id, _tool_result(payload))


def serve() -> int:
    """Serve newline-delimited MCP JSON-RPC over stdin/stdout."""
    while True:
        line = sys.stdin.buffer.readline(MAX_REQUEST_BYTES + 1)
        if not line:
            return 0
        if len(line) > MAX_REQUEST_BYTES:
            while line and not line.endswith(b"\n"):
                line = sys.stdin.buffer.readline(MAX_REQUEST_BYTES + 1)
            response = _error(None, -32700, "request is too large")
        else:
            try:
                request = json.loads(line)
            except (UnicodeDecodeError, json.JSONDecodeError):
                response = _error(None, -32700, "parse error")
            else:
                response = handle_request(request)
        if response is not None:
            sys.stdout.write(json.dumps(response, ensure_ascii=False) + "\n")
            sys.stdout.flush()


def main() -> None:
    raise SystemExit(serve())


if __name__ == "__main__":
    main()
