from __future__ import annotations

import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
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
        "robot.ros2.profile",
        "robot.ros2.readiness.verify",
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


def test_ros2_mcp_profile_is_redacted_and_readiness_is_content_addressed() -> None:
    profile_response = handle_request(
        _rpc(
            10,
            "tools/call",
            {
                "name": "robot.ros2.profile",
                "arguments": {"robot_id": "flyto-rover-sim-001"},
            },
        )
    )
    assert profile_response is not None
    profile = profile_response["result"]["structuredContent"]
    encoded_profile = json.dumps(profile, sort_keys=True)
    assert "nav2_msgs" not in encoded_profile
    assert "/navigate_to_pose" not in encoded_profile
    assert "interface" not in encoded_profile

    observed_at = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
    runtime = {
        "contract_version": "flyto.robotics.ros2-runtime-snapshot.v1",
        "profile_id": profile["profile_id"],
        "profile_snapshot": profile["profile_snapshot"],
        "robot_id": profile["robot_id"],
        "deployment_mode": "simulation",
        "observed_at": observed_at,
        "max_age_seconds": 60,
        "emergency_stop_ready": True,
        "adapters": [
            {
                "adapter_id": profile["adapters"][0]["adapter_id"],
                "status": "ready",
                "interface_available": True,
                "lifecycle_state": "active",
                "observation_sequence": "mcp-test:ready:1",
            }
        ],
    }
    runtime["snapshot"] = hashlib.sha256(
        json.dumps(runtime, separators=(",", ":"), sort_keys=True).encode("utf-8")
    ).hexdigest()
    readiness_response = handle_request(
        _rpc(
            11,
            "tools/call",
            {
                "name": "robot.ros2.readiness.verify",
                "arguments": {"runtime": runtime},
            },
        )
    )

    assert readiness_response is not None
    readiness = readiness_response["result"]["structuredContent"]
    assert readiness["passed"] is True
    assert readiness["ready_capability_ids"] == [
        "robotics.motion.navigate@1",
        "robotics.motion.navigate_to_location@1",
    ]
