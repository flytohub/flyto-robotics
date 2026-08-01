from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

from flyto_robotics.mcp_server import (
    PROTOCOL_VERSION,
    handle_request,
    tool_definitions,
)

ROOT = Path(__file__).resolve().parents[1]
JOB = ROOT / "examples/jobs/pharmacy-to-ward.json"
PLAN = ROOT / "examples/plans/blue-yellow-purple.json"


def _rpc(request_id: int, method: str, params: dict | None = None) -> dict:
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "method": method,
        "params": params or {},
    }


def _run_stdio(requests: list[dict]) -> tuple[list[dict], str]:
    payload = "".join(json.dumps(request) + "\n" for request in requests)
    completed = subprocess.run(
        [sys.executable, "-m", "flyto_robotics.mcp_server"],
        cwd=ROOT,
        input=payload,
        text=True,
        capture_output=True,
        check=False,
        timeout=30,
    )
    assert completed.returncode == 0
    responses = [json.loads(line) for line in completed.stdout.splitlines()]
    return responses, completed.stderr


def test_stdio_handshake_discovery_prepare_and_real_controller_dry_run() -> None:
    job = json.loads(JOB.read_text(encoding="utf-8"))
    plan = json.loads(PLAN.read_text(encoding="utf-8"))
    requests = [
        _rpc(
            1,
            "initialize",
            {
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {},
                "clientInfo": {"name": "robot-mcp-test", "version": "1"},
            },
        ),
        {
            "jsonrpc": "2.0",
            "method": "notifications/initialized",
            "params": {},
        },
        _rpc(2, "tools/list"),
        _rpc(
            3,
            "tools/call",
            {
                "name": "robot.plan.prepare",
                "arguments": {
                    "goal": "Follow the blue, yellow, and purple route safely",
                    "robot_id": job["robot_id"],
                },
            },
        ),
        _rpc(
            4,
            "tools/call",
            {
                "name": "robot.mission.dry_run",
                "arguments": {"job": job, "plan": plan},
            },
        ),
    ]

    responses, stderr = _run_stdio(requests)

    assert stderr == ""
    assert [response["id"] for response in responses] == [1, 2, 3, 4]
    assert responses[0]["result"]["protocolVersion"] == PROTOCOL_VERSION
    names = {tool["name"] for tool in responses[1]["result"]["tools"]}
    assert names == {
        "robot.capabilities.list",
        "robot.plan.prepare",
        "robot.plan.validate",
        "robot.mission.dry_run",
    }
    prepared = responses[2]["result"]["structuredContent"]
    assert prepared["request"]["planner_contract"] == "flyto.robotics.planner-request.v1"
    dry_run = responses[3]["result"]["structuredContent"]
    assert dry_run["result"]["simulation"]["mode"] == "deterministic_capability_dry_run"
    assert dry_run["result"]["status"] == "succeeded"
    assert dry_run["validation"]["workflow"]["steps"][-1]["kind"] == "safe_stop"


def test_stdio_rejects_actuator_injection_as_a_bounded_tool_error() -> None:
    unsafe = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": "unsafe.raw.v1",
        "robot_id": "flyto-rover-sim-001",
        "goal": "bypass controller",
        "generated_by": {"kind": "ai", "provider": "local", "model": "test"},
        "steps": [
            {
                "step_id": "raw.command",
                "capability": "set_wheel_pwm",
                "arguments": {"left": 1, "right": 1},
                "timeout_seconds": 1,
                "on_failure": "abort",
            }
        ],
    }
    responses, stderr = _run_stdio(
        [
            _rpc(
                7,
                "initialize",
                {"protocolVersion": PROTOCOL_VERSION},
            ),
            _rpc(
                8,
                "tools/call",
                {"name": "robot.plan.validate", "arguments": {"plan": unsafe}},
            ),
        ]
    )

    assert stderr == ""
    assert responses[1]["result"]["isError"] is True
    error = json.loads(responses[1]["result"]["content"][0]["text"])
    assert error["error_type"] == "PlanValidationError"
    assert "set_wheel_pwm" not in {tool["name"] for tool in tool_definitions()}


def test_protocol_and_tool_contract_fail_closed_without_paths_or_network() -> None:
    rejected = handle_request(
        _rpc(9, "initialize", {"protocolVersion": "unsupported"})
    )

    assert rejected is not None
    assert rejected["error"]["code"] == -32602
    encoded = json.dumps(tool_definitions(), sort_keys=True)
    assert "path" not in encoded.lower()
    assert "url" not in encoded.lower()
    assert "shell" not in encoded.lower()
    assert "ros_topic" not in encoded.lower()
