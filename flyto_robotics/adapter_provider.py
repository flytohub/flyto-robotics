"""Flyto2 external-adapter provider for standard ROS 2 equipment.

The provider is installed on the execution computer, never on the robot.  It
offers both Python entry points for current hosts and a tiny JSON-lines process
protocol for Flyto2 Runtime, so Cloud never imports ROS, rosbridge, or vendor
transport code.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import socket
import sys
from collections.abc import Mapping
from datetime import datetime, timezone
from typing import Any

from . import adapter_contract as contract
from .generic_ros2_adapter import GenericROS2Adapter, build

ADAPTER_ID = "ros2.generic"
PROVIDER_PROTOCOL = "flyto2.adapter-provider.v1"


def _safe_fragment(value: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "-", str(value or "").strip())
    return text.strip("-._")[:48] or "resource"


def _resource_identity() -> tuple[str, str]:
    domain = os.getenv("ROS_DOMAIN_ID", "0").strip() or "0"
    hostname = socket.gethostname()
    resource_id = (
        os.getenv("FLYTO_ROS2_RESOURCE_ID", "").strip()
        or f"ros2-{_safe_fragment(hostname)}-{_safe_fragment(domain)}"
    )[:128]
    resource_name = (
        os.getenv("FLYTO_ROS2_RESOURCE_NAME", "").strip()
        or f"ROS 2 robot ({hostname})"
    )[:200]
    return resource_id, resource_name


def _manifest(adapter: GenericROS2Adapter, *, resource_name: str) -> dict[str, Any]:
    declarations = tuple(adapter.describe())
    contracts: list[dict[str, Any]] = []
    for item in declarations:
        metadata = contract.capability_metadata(item.capability_id)
        contracts.append(
            {
                "capability_id": item.capability_id,
                "display_name": str(metadata.get("display_name") or item.capability_id),
                "description": str(metadata.get("description") or "")[:1000],
                "input_schema": contract.arguments_to_json_schema(item.arguments),
                "safety_class": item.safety_class,
                "required_permissions": list(item.required_permissions),
                "requires_safe_stop": item.requires_safe_stop,
            }
        )
    return {
        "contract": "flyto.resource-manifest.v1",
        "resource_id": adapter.resource_id,
        "resource_type": "robot",
        "display_name": resource_name,
        "revision": 1,
        "adapter": {
            "adapter_id": ADAPTER_ID,
            "version": "1.0.0",
            "provider": "Flyto2 Robotics",
        },
        "deployment_mode": (
            "simulation"
            if os.getenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware").strip().lower()
            == "simulation"
            else "real"
        ),
        "capability_ids": [item["capability_id"] for item in contracts],
        "capability_contracts": contracts,
        "settings": [],
        "telemetry_channels": [],
        "observed_at": datetime.now(timezone.utc).isoformat(),
        "contract_hash": "",
    }


def discover_resource_manifests(*, execution_host_id: str) -> list[dict[str, Any]]:
    """Discover one reachable standard ROS 2 graph without granting authority."""

    _ = execution_host_id
    disabled = os.getenv("FLYTO_ROS2_AUTODISCOVER", "").strip().lower()
    if disabled in {"0", "false", "off", "no"}:
        return []

    transport = os.getenv("FLYTO_ROS2_TRANSPORT", "rclpy").strip().lower()
    if transport == "rosbridge" and not os.getenv("FLYTO_ROSBRIDGE_URL", "").strip():
        return []
    if transport not in {"rclpy", "rosbridge"}:
        return []

    resource_id, resource_name = _resource_identity()
    adapter: GenericROS2Adapter | None = None
    try:
        adapter = build(resource_id)
        if not adapter.describe():
            return []
        return [_manifest(adapter, resource_name=resource_name)]
    except Exception:
        # Discovery is best effort.  An unavailable transport must not become
        # an authoritative "no equipment exists" statement.
        return []
    finally:
        if adapter is not None:
            adapter.disconnect()


def build_adapter(resource_id: str) -> GenericROS2Adapter:
    """Python entry point used by hosts that load adapters in-process."""

    return build(resource_id)


def _response(request_id: Any, *, ok: bool, result: Any = None, error: str = "") -> dict[str, Any]:
    payload: dict[str, Any] = {
        "protocol": PROVIDER_PROTOCOL,
        "id": request_id,
        "ok": ok,
    }
    if ok:
        payload["result"] = result
    else:
        payload["error"] = error[:500] or "adapter provider request failed"
    return payload


def _serve(adapter_id: str, resource_id: str) -> int:
    if adapter_id != ADAPTER_ID:
        print(
            json.dumps(
                _response(None, ok=False, error=f"unsupported adapter: {adapter_id}"),
                separators=(",", ":"),
            ),
            flush=True,
        )
        return 2

    adapter = build(resource_id)
    try:
        for raw in sys.stdin:
            raw = raw.strip()
            if not raw:
                continue
            try:
                request = json.loads(raw)
                if not isinstance(request, Mapping):
                    raise ValueError("request must be an object")
                request_id = request.get("id")
                op = str(request.get("op") or "")
                if op == "invoke":
                    call = request.get("request")
                    if not isinstance(call, Mapping):
                        raise ValueError("invoke.request must be an object")
                    result = adapter.invoke(
                        contract.CallRequest(
                            call_id=str(call.get("call_id") or ""),
                            capability_id=str(call.get("capability_id") or ""),
                            arguments=dict(call.get("arguments") or {}),
                            deadline_seconds=float(call.get("deadline_seconds") or 30.0),
                        )
                    )
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "cancel":
                    result = adapter.cancel(str(request.get("call_id") or ""))
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "safe_stop":
                    result = adapter.safe_stop()
                    value = {
                        "call_id": result.call_id,
                        "outcome": result.outcome,
                        "evidence": dict(result.evidence),
                        "detail": result.detail,
                    }
                elif op == "observe":
                    value = adapter.observe(
                        phase=str(request.get("phase") or "preflight"),
                        execution_id=(
                            str(request["execution_id"])
                            if request.get("execution_id") is not None
                            else None
                        ),
                    )
                elif op == "describe":
                    value = _manifest(adapter, resource_name=_resource_identity()[1])
                elif op == "execution_count":
                    value = {
                        "count": adapter.execution_count(str(request.get("call_id") or ""))
                    }
                elif op == "close":
                    print(
                        json.dumps(
                            _response(
                                request_id,
                                ok=True,
                                result={"closed": True},
                            ),
                            separators=(",", ":"),
                        ),
                        flush=True,
                    )
                    return 0
                else:
                    raise ValueError(f"unsupported provider operation: {op}")
                response = _response(request_id, ok=True, result=value)
            except Exception as error:  # provider boundary must return typed failure
                response = _response(
                    locals().get("request_id"),
                    ok=False,
                    error=f"{type(error).__name__}: {error}",
                )
            print(json.dumps(response, separators=(",", ":"), ensure_ascii=True), flush=True)
    finally:
        adapter.disconnect()
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="flyto2-adapter-provider-ros2-generic")
    parser.add_argument("--discover", action="store_true")
    parser.add_argument("--execution-host-id", default="")
    parser.add_argument("--adapter-id", default=ADAPTER_ID)
    parser.add_argument("--resource-id", default="")
    args = parser.parse_args(argv)

    if args.discover:
        print(
            json.dumps(
                {
                    "protocol": PROVIDER_PROTOCOL,
                    "resources": discover_resource_manifests(
                        execution_host_id=args.execution_host_id
                    ),
                },
                separators=(",", ":"),
                ensure_ascii=True,
            )
        )
        return 0

    resource_id = args.resource_id.strip() or _resource_identity()[0]
    return _serve(args.adapter_id, resource_id)


if __name__ == "__main__":
    raise SystemExit(main())
