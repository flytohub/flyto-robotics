#!/usr/bin/env python3
"""Run a real, repeatable Robot MCP closed-loop benchmark.

The benchmark launches the production stdio server, negotiates MCP, and sends
every case through planner request preparation, strict plan compilation, and
the deterministic MissionController. It does not replace any boundary with a
mock or call implementation helpers in-process.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import select
import subprocess
import sys
import tempfile
import time
from collections import Counter, defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
BASE_JOB = ROOT / "examples/jobs/pharmacy-to-ward.json"
PROTOCOL_VERSION = "2025-06-18"
CONTRACT_VERSION = "flyto.robotics.mcp-benchmark.v1"
MINIMUM_CASES = 101
MINIMUM_SUCCESS_RATE = 0.90
EXPECTED_TOOLS = {
    "robot.capabilities.list",
    "robot.plan.prepare",
    "robot.plan.validate",
    "robot.mission.dry_run",
}
GOALS = (
    "Hold position and finish with a verified safe stop",
    "確認通道淨空，完成任務後安全停止",
    "通路の安全を確認し、承認後に短距離移動して停止する",
    "Verifique el paso, solicite aprobación y deténgase con seguridad",
    "Vérifier le passage, obtenir l’approbation et terminer en arrêt sûr",
    "افحص المسار وتوقف بأمان بعد موافقة المشغل",
)


def _canonical(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _digest(value: object) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _rpc(request_id: int, method: str, params: dict[str, Any] | None = None) -> dict:
    return {
        "jsonrpc": "2.0",
        "id": request_id,
        "method": method,
        "params": params or {},
    }


class RobotMCPClient:
    """Synchronous client for the actual newline-delimited stdio server."""

    def __init__(self, *, timeout_seconds: float = 30.0) -> None:
        self.timeout_seconds = timeout_seconds
        self.process: subprocess.Popen[str] | None = None
        self.request_id = 0

    def __enter__(self) -> RobotMCPClient:
        self.process = subprocess.Popen(
            [sys.executable, "-m", "flyto_robotics.mcp_server"],
            cwd=ROOT,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        return self

    def __exit__(self, exc_type: object, exc: object, traceback: object) -> None:
        if self.process is None:
            return
        if self.process.stdin is not None:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)

    def notify(self, method: str, params: dict[str, Any] | None = None) -> None:
        self._write({"jsonrpc": "2.0", "method": method, "params": params or {}})

    def call(self, method: str, params: dict[str, Any] | None = None) -> dict:
        self.request_id += 1
        expected_id = self.request_id
        self._write(_rpc(expected_id, method, params))
        response = self._read()
        if response.get("id") != expected_id:
            raise RuntimeError("Robot MCP returned an unexpected response id")
        if "error" in response:
            error = response["error"]
            message = (
                error.get("message", "protocol error")
                if isinstance(error, dict)
                else "protocol error"
            )
            raise RuntimeError(str(message)[:256])
        result = response.get("result")
        if not isinstance(result, dict):
            raise RuntimeError("Robot MCP returned a non-object result")
        return result

    def tool(self, name: str, arguments: dict[str, Any]) -> dict:
        result = self.call("tools/call", {"name": name, "arguments": arguments})
        if result.get("isError") is True:
            error_type = "ToolError"
            message = "Robot MCP tool rejected the case"
            content = result.get("content")
            if isinstance(content, list) and content and isinstance(content[0], dict):
                try:
                    decoded = json.loads(str(content[0].get("text", "{}")))
                except json.JSONDecodeError:
                    decoded = {}
                if isinstance(decoded, dict):
                    error_type = str(decoded.get("error_type", error_type))[:64]
                    message = str(decoded.get("error", message))[:256]
            raise RuntimeError(f"{error_type}: {message}")
        payload = result.get("structuredContent")
        if not isinstance(payload, dict):
            raise RuntimeError("Robot MCP omitted structuredContent")
        return payload

    def stderr(self) -> str:
        if self.process is None or self.process.stderr is None:
            return ""
        if self.process.poll() is None:
            return ""
        return self.process.stderr.read(4096)

    def _write(self, payload: dict[str, Any]) -> None:
        if self.process is None or self.process.stdin is None:
            raise RuntimeError("Robot MCP process is not running")
        if self.process.poll() is not None:
            raise RuntimeError("Robot MCP process exited unexpectedly")
        self.process.stdin.write(json.dumps(payload, ensure_ascii=False) + "\n")
        self.process.stdin.flush()

    def _read(self) -> dict:
        if self.process is None or self.process.stdout is None:
            raise RuntimeError("Robot MCP process is not running")
        ready, _, _ = select.select(
            [self.process.stdout],
            [],
            [],
            self.timeout_seconds,
        )
        if not ready:
            raise TimeoutError("Robot MCP response timed out")
        line = self.process.stdout.readline()
        if not line:
            detail = self.stderr().strip()
            suffix = f": {detail[:256]}" if detail else ""
            raise RuntimeError(f"Robot MCP closed stdout{suffix}")
        try:
            response = json.loads(line)
        except json.JSONDecodeError as exc:
            raise RuntimeError("Robot MCP returned invalid JSON") from exc
        if not isinstance(response, dict):
            raise RuntimeError("Robot MCP returned a non-object response")
        return response


def _step(
    case_id: str,
    index: int,
    capability: str,
    arguments: dict[str, object],
    *,
    timeout_seconds: float,
) -> dict[str, object]:
    return {
        "step_id": f"{case_id}.step.{index:02d}",
        "capability": capability,
        "arguments": arguments,
        "timeout_seconds": timeout_seconds,
        "on_failure": "abort",
    }


def _case_payload(index: int, base_job: dict[str, Any]) -> dict[str, Any]:
    case_id = f"robot-mcp-{index + 1:03d}"
    lane = index % 3
    if lane == 0:
        tier = "standard"
        dwell_count = 1 + (index % 2)
        route_limit = 4 + (index % 3)
        prefix: list[tuple[str, dict[str, object], float]] = []
    elif lane == 1:
        tier = "intermediate"
        dwell_count = 2 + (index % 3)
        route_limit = 8 + (index % 5)
        prefix = [("wait_until_clear", {"clear_seconds": 0.1}, 2.0)]
    else:
        tier = "advanced"
        dwell_count = 3 + (index % 5)
        route_limit = 16 + (index % 9)
        approval_id = f"approval.{index + 1:03d}"
        prefix = [
            (
                "ask_human",
                {
                    "approval_id": approval_id,
                    "prompt_key": "benchmark.motion.approve",
                },
                2.0,
            ),
            ("resume", {"approval_id": approval_id}, 2.0),
            ("wait_until_clear", {"clear_seconds": 0.1}, 2.0),
            (
                "move_relative",
                {"distance_m": 0.02 + (index % 4) * 0.01, "speed": 0.2},
                3.0,
            ),
        ]

    raw_steps = prefix + [
        ("dwell", {"seconds": 0.0}, 1.0) for _ in range(dwell_count)
    ]
    raw_steps.append(("safe_stop", {"seconds": 0.0}, 1.0))
    steps = [
        _step(case_id, position, capability, arguments, timeout_seconds=timeout)
        for position, (capability, arguments, timeout) in enumerate(raw_steps, start=1)
    ]
    goal = f"{GOALS[index % len(GOALS)]} [{case_id}]"
    job = copy.deepcopy(base_job)
    job["job_id"] = case_id
    job["metadata"] = {
        "scenario": "robot-mcp-benchmark",
        "case_id": case_id,
        "tier": tier,
    }
    plan = {
        "contract_version": "flyto.robotics.plan.v1",
        "plan_id": f"{case_id}.plan.v1",
        "robot_id": job["robot_id"],
        "goal": goal,
        "generated_by": {
            "kind": "deterministic_demo",
            "provider": "flyto-ai",
            "model": "mcp-benchmark-generator",
        },
        "steps": steps,
    }
    prepare = {
        "goal": goal,
        "robot_id": job["robot_id"],
        "observations": {
            "case_id": case_id,
            "difficulty_tier": tier,
            "requested_depth": len(steps),
        },
        "routing_context": {
            "runtime": "robotics",
            "robot_id": job["robot_id"],
            "available_observations": ["minimum_range", "human_decision"],
            "permissions": ["robot.motion", "operator.approval"],
        },
        "route_limit": route_limit,
    }
    return {
        "case_id": case_id,
        "tier": tier,
        "depth": len(steps),
        "job": job,
        "plan": plan,
        "prepare": prepare,
    }


def _run_case(client: RobotMCPClient, case: dict[str, Any]) -> dict[str, Any]:
    started = time.monotonic()
    record: dict[str, Any] = {
        "case_id": case["case_id"],
        "tier": case["tier"],
        "depth": case["depth"],
        "job_sha256": _digest(case["job"]),
        "plan_sha256": _digest(case["plan"]),
        "stages": {"prepare": False, "validate": False, "dry_run": False},
        "success": False,
    }
    try:
        prepared = client.tool("robot.plan.prepare", case["prepare"])
        record["stages"]["prepare"] = True
        record["prepared_request_sha256"] = _digest(prepared)

        validated = client.tool("robot.plan.validate", {"plan": case["plan"]})
        record["stages"]["validate"] = True
        record["validation_sha256"] = _digest(validated)

        executed = client.tool(
            "robot.mission.dry_run",
            {"job": case["job"], "plan": case["plan"]},
        )
        record["stages"]["dry_run"] = True
        record["dry_run_sha256"] = _digest(executed)
        result = executed.get("result")
        validation = executed.get("validation")
        workflow = validation.get("workflow") if isinstance(validation, dict) else None
        workflow_steps = workflow.get("steps") if isinstance(workflow, dict) else None
        last_kind = (
            workflow_steps[-1].get("kind")
            if isinstance(workflow_steps, list) and workflow_steps
            else None
        )
        status = result.get("status") if isinstance(result, dict) else None
        simulation = result.get("simulation") if isinstance(result, dict) else None
        mode = simulation.get("mode") if isinstance(simulation, dict) else None
        record["terminal_status"] = status
        record["terminal_step"] = last_kind
        record["controller_mode"] = mode
        record["success"] = (
            status == "succeeded"
            and last_kind == "safe_stop"
            and mode == "deterministic_capability_dry_run"
        )
        if not record["success"]:
            record["error_type"] = "OutcomeAssertionError"
            record["error"] = "controller outcome did not satisfy the benchmark contract"
    except (RuntimeError, TimeoutError, OSError, ValueError) as exc:
        record["error_type"] = type(exc).__name__
        record["error"] = str(exc)[:256]
    record["elapsed_ms"] = round((time.monotonic() - started) * 1000, 3)
    return record


def _rate(successes: int, total: int) -> float:
    return round(successes / total, 6) if total else 0.0


def run_benchmark(
    *,
    case_count: int = MINIMUM_CASES,
    minimum_cases: int = MINIMUM_CASES,
    minimum_success_rate: float = MINIMUM_SUCCESS_RATE,
    timeout_seconds: float = 30.0,
) -> dict[str, Any]:
    """Execute cases through the production subprocess and return an attested report."""
    if case_count < minimum_cases:
        raise ValueError(f"case_count must be at least {minimum_cases}")
    if not 0.9 <= minimum_success_rate <= 1.0:
        raise ValueError("minimum_success_rate must be between 0.9 and 1.0")

    base_job = json.loads(BASE_JOB.read_text(encoding="utf-8"))
    cases = [_case_payload(index, base_job) for index in range(case_count)]
    signatures = {
        _digest({"job": case["job"], "plan": case["plan"], "prepare": case["prepare"]})
        for case in cases
    }
    if len(signatures) != case_count:
        raise RuntimeError("benchmark cases are not distinct")

    started_at = _utc_now()
    started = time.monotonic()
    server_info: dict[str, Any] = {}
    tool_names: list[str] = []
    records: list[dict[str, Any]] = []
    capability_snapshot: object = None
    with RobotMCPClient(timeout_seconds=timeout_seconds) as client:
        initialized = client.call(
            "initialize",
            {
                "protocolVersion": PROTOCOL_VERSION,
                "capabilities": {},
                "clientInfo": {
                    "name": "flyto-robot-mcp-benchmark",
                    "version": "1",
                },
            },
        )
        if initialized.get("protocolVersion") != PROTOCOL_VERSION:
            raise RuntimeError("Robot MCP negotiated an unexpected protocol")
        raw_server_info = initialized.get("serverInfo")
        if isinstance(raw_server_info, dict):
            server_info = raw_server_info
        client.notify("notifications/initialized")
        listing = client.call("tools/list")
        tools = listing.get("tools")
        if not isinstance(tools, list):
            raise RuntimeError("Robot MCP tools/list returned an invalid contract")
        tool_names = sorted(
            tool["name"]
            for tool in tools
            if isinstance(tool, dict) and isinstance(tool.get("name"), str)
        )
        if set(tool_names) != EXPECTED_TOOLS:
            raise RuntimeError("Robot MCP tool surface drifted from the benchmark contract")
        capabilities = client.tool("robot.capabilities.list", {})
        capability_snapshot = capabilities.get("registry_snapshot")
        for case in cases:
            records.append(_run_case(client, case))

    tier_records: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        tier_records[record["tier"]].append(record)
    tiers: dict[str, dict[str, Any]] = {}
    for tier in ("standard", "intermediate", "advanced"):
        group = tier_records[tier]
        tier_successes = sum(record["success"] is True for record in group)
        tier_rate = _rate(tier_successes, len(group))
        tiers[tier] = {
            "case_count": len(group),
            "successes": tier_successes,
            "failures": len(group) - tier_successes,
            "success_rate": tier_rate,
            "passed": bool(group) and tier_rate >= minimum_success_rate,
            "depth_min": min((record["depth"] for record in group), default=0),
            "depth_max": max((record["depth"] for record in group), default=0),
        }

    successes = sum(record["success"] is True for record in records)
    success_rate = _rate(successes, case_count)
    errors = Counter(
        record.get("error_type", "none")
        for record in records
        if not record["success"]
    )
    report: dict[str, Any] = {
        "contract_version": CONTRACT_VERSION,
        "benchmark_family": "robot-mcp",
        "real_execution": {
            "mocked": False,
            "transport": "production-stdio-subprocess",
            "controller": "MissionController",
            "controller_mode": "deterministic_capability_dry_run",
        },
        "policy": {
            "minimum_cases": minimum_cases,
            "minimum_success_rate": minimum_success_rate,
            "require_every_tier": True,
        },
        "started_at": started_at,
        "completed_at": _utc_now(),
        "elapsed_ms": round((time.monotonic() - started) * 1000, 3),
        "protocol_version": PROTOCOL_VERSION,
        "server_info": server_info,
        "tools": tool_names,
        "capability_registry_snapshot": capability_snapshot,
        "distinct_case_count": len(signatures),
        "case_count": case_count,
        "successes": successes,
        "failures": case_count - successes,
        "success_rate": success_rate,
        "tiers": tiers,
        "failure_types": dict(sorted(errors.items())),
        "cases": records,
    }
    report["passed"] = (
        case_count >= minimum_cases
        and len(signatures) == case_count
        and success_rate >= minimum_success_rate
        and all(tier["passed"] for tier in tiers.values())
    )
    report["evidence_sha256"] = _digest(report)
    return report


def write_report_atomic(report: dict[str, Any], output_dir: Path) -> Path:
    """Write one content-addressed, owner-readable report atomically."""
    output_dir.mkdir(parents=True, exist_ok=True, mode=0o700)
    digest = report.get("evidence_sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise ValueError("report is missing a valid evidence digest")
    destination = output_dir / f"robot-mcp-benchmark-{digest}.json"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".robot-mcp-benchmark-",
        suffix=".tmp",
        dir=output_dir,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        os.replace(temporary, destination)
    finally:
        if temporary.exists():
            temporary.unlink()
    return destination


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cases", type=int, default=MINIMUM_CASES)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "output/benchmarks",
    )
    parser.add_argument("--timeout-seconds", type=float, default=30.0)
    return parser


def main() -> int:
    arguments = _parser().parse_args()
    try:
        report = run_benchmark(
            case_count=arguments.cases,
            timeout_seconds=arguments.timeout_seconds,
        )
        output = write_report_atomic(report, arguments.output_dir)
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        print(f"robot MCP benchmark failed: {exc}", file=sys.stderr)
        return 2
    print(
        json.dumps(
            {
                "passed": report["passed"],
                "case_count": report["case_count"],
                "successes": report["successes"],
                "success_rate": report["success_rate"],
                "tiers": report["tiers"],
                "evidence_sha256": report["evidence_sha256"],
                "report": str(output),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
