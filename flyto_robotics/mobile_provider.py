"""Read-only, local ROS2 readiness provider for the Runtime adapter contract.

This is NOT a motion controller. It observes the existing ROS2 readiness
adapter's file from an explicit host path, bounds its freshness, hashes the
actual bytes and never fabricates a successful robot mission.

Usage as an installed mobile adapter (example):
    python -m flyto_robotics.mobile_provider --status-file /path/status.json

One flyto2.execution.v1 invocation is read from stdin and a typed capability
result is written to stdout. Cloud, Core and security Engine are not imported.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import stat
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .ros2_adapter import STATUS_SCHEMA

CAPABILITY = "robot.ros2.readiness"
MAX_OBSERVATION_AGE_SECONDS = 15
MAX_STATUS_BYTES = 4096
MAX_REQUEST_BYTES = 12_000
_SAFE_ID = re.compile(r"^[A-Za-z0-9._:-]{1,128}$")


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def read_status_file(path: Path, now: float | None = None) -> tuple[dict[str, Any], bytes]:
    if not path.is_absolute():
        raise ValueError("status_file_must_be_absolute")
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode) or stat.S_ISLNK(info.st_mode):
        raise ValueError("status_file_not_regular")
    if info.st_size > MAX_STATUS_BYTES:
        raise ValueError("status_file_oversized")
    if (now if now is not None else time.time()) - info.st_mtime > MAX_OBSERVATION_AGE_SECONDS:
        raise ValueError("ros2_graph_observation_stale")
    data = path.read_bytes()
    if len(data) > MAX_STATUS_BYTES:
        raise ValueError("status_file_oversized")
    status = json.loads(data)
    if not isinstance(status, dict) or status.get("schema") != STATUS_SCHEMA or \
            status.get("service") != "ros2_readiness_adapter" or \
            not isinstance(status.get("ready"), bool) or \
            not isinstance(status.get("state"), str):
        raise ValueError("ros2_graph_document_invalid")
    return status, data


def execute(request: Any, status_path: Path) -> dict[str, Any]:
    if not isinstance(request, dict) or request.get("schema") != "flyto2.execution.v1":
        raise ValueError("invalid_invocation")
    capability = request.get("capability")
    invocation_id = request.get("invocation_id")
    operation_id = request.get("operation_id")
    if capability != CAPABILITY or request.get("revision") != 1 or \
            not isinstance(invocation_id, str) or not _SAFE_ID.fullmatch(invocation_id) or \
            not isinstance(operation_id, str) or not _SAFE_ID.fullmatch(operation_id) or \
            not isinstance(request.get("input"), dict) or request["input"]:
        raise ValueError("invalid_invocation")

    started_at = _now_iso()
    try:
        status, raw = read_status_file(status_path)
        digest = hashlib.sha256(raw).hexdigest()
        return {
            "schema": "flyto2.execution.v1",
            "invocation_id": invocation_id,
            "capability": CAPABILITY,
            "revision": 1,
            "status": "success",
            "started_at": started_at,
            "completed_at": _now_iso(),
            "output": {
                "graph_state": status["state"],
                "ready": status["ready"],
                "reason": str(status.get("reason", ""))[:256],
                "missing_topics": list(status.get("missing_topics", []))[:16],
                "mismatched_topics": list(status.get("mismatched_topics", []))[:16],
            },
            "evidence": [{
                "kind": "ros2_graph_status",
                "ref": "sha256:" + digest,
                "sha256": digest,
                "size": len(raw),
            }],
        }
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return {
            "schema": "flyto2.execution.v1",
            "invocation_id": invocation_id,
            "capability": CAPABILITY,
            "revision": 1,
            "status": "failed",
            "started_at": started_at,
            "completed_at": _now_iso(),
            "output": {},
            "evidence": [],
            "failure": {"code": "ros2_graph_unavailable", "retryable": False,
                        "detail": str(error)[:256]},
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Read current ROS2 readiness evidence")
    parser.add_argument("--status-file", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        request_bytes = sys.stdin.buffer.read(MAX_REQUEST_BYTES + 1)
        if len(request_bytes) > MAX_REQUEST_BYTES:
            raise ValueError("invocation_too_large")
        result = execute(json.loads(request_bytes), args.status_file)
    except (ValueError, json.JSONDecodeError) as error:
        print(json.dumps({"error": "invalid_input", "detail": str(error)[:256]}))
        return 2
    print(json.dumps(result, separators=(",", ":"), sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
