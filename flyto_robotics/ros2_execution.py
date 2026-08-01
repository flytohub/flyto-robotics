"""Short-lived execution authority for one exact Flyto2-to-ROS 2 binding."""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any

from .resource_binding import ResourceBindingError, select_resource_binding
from .ros2_pairing import (
    parse_observed_at,
    parse_ros2_adapter_manifest,
    parse_ros2_runtime_snapshot,
    verify_ros2_pairing,
)

ROS2_EXECUTION_GRANT_CONTRACT_VERSION = "flyto.robotics.ros2-execution-grant.v1"
MAX_GRANT_LIFETIME_SECONDS = 300
_GRANT_FIELDS = {
    "contract_version",
    "grant_id",
    "issued_at",
    "expires_at",
    "profile_id",
    "robot_id",
    "deployment_mode",
    "workflow_id",
    "target_space_id",
    "endpoint_id",
    "resource_id",
    "adapter_id",
    "capability_id",
    "resource_plan_snapshot",
    "profile_snapshot",
    "runtime_snapshot",
    "resource_observation_sequence",
    "runtime_observation_sequence",
    "require_confirmation",
    "snapshot",
}
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,191}$")


class Ros2ExecutionError(ValueError):
    """Raised when semantic execution authority cannot be issued or resolved."""


@dataclass(frozen=True)
class Ros2ExecutionTarget:
    """Private deterministic-adapter view; never return this through MCP."""

    adapter_id: str
    capability_id: str
    interface_kind: str
    interface_name: str
    interface_type: str
    timeout_seconds: float
    grant_snapshot: str
    resource_plan_snapshot: str
    profile_snapshot: str
    runtime_snapshot: str


def authorize_ros2_execution(
    *,
    resource_plan: Mapping[str, Any],
    manifest: Mapping[str, Any],
    runtime: Mapping[str, Any],
    workflow_id: str,
    resource_id: str,
    capability_id: str,
    target_space_id: str,
    confirmed: bool = False,
    observed_at: datetime | None = None,
) -> dict[str, Any]:
    """Issue one redacted grant only after resource and ROS readiness agree."""

    validated_manifest = parse_ros2_adapter_manifest(manifest)
    validated_runtime = parse_ros2_runtime_snapshot(runtime)
    checked_at = _utc(observed_at or datetime.now(timezone.utc), "observed_at")
    pairing = verify_ros2_pairing(
        validated_manifest,
        validated_runtime,
        observed_at=checked_at,
    )
    if pairing["passed"] is not True:
        failed = [
            check["code"] for check in pairing["checks"] if check["passed"] is False
        ]
        raise Ros2ExecutionError(
            "ROS 2 pairing is not ready: " + ", ".join(failed)
        )

    normalized_capability = _identifier(capability_id, "capability_id")
    matching_adapters = [
        adapter
        for adapter in validated_manifest["adapters"]
        if normalized_capability in adapter["capability_ids"]
    ]
    if len(matching_adapters) != 1:
        raise Ros2ExecutionError(
            "capability must resolve to exactly one declared ROS 2 adapter"
        )
    adapter = matching_adapters[0]
    try:
        binding = select_resource_binding(
            resource_plan,
            workflow_id=_identifier(workflow_id, "workflow_id"),
            resource_id=_identifier(resource_id, "resource_id"),
            capability_id=normalized_capability,
            allowed_adapter_ids=(adapter["adapter_id"],),
            target_space_id=_identifier(target_space_id, "target_space_id"),
            confirmed=confirmed,
        )
    except ResourceBindingError as exc:
        raise Ros2ExecutionError(str(exc)) from exc
    if binding.adapter_id not in pairing["ready_adapter_ids"]:
        raise Ros2ExecutionError("selected ROS 2 adapter is not ready")
    if normalized_capability not in pairing["ready_capability_ids"]:
        raise Ros2ExecutionError("selected semantic capability is not ready")

    runtime_state = next(
        state
        for state in validated_runtime["adapters"]
        if state["adapter_id"] == binding.adapter_id
    )
    runtime_time = parse_observed_at(validated_runtime["observed_at"])
    expires_at = runtime_time.timestamp() + validated_runtime["max_age_seconds"]
    seed = {
        "workflow_id": binding.workflow_id,
        "resource_id": binding.resource_id,
        "adapter_id": binding.adapter_id,
        "capability_id": binding.capability_id,
        "resource_plan_snapshot": binding.plan_snapshot,
        "profile_snapshot": validated_manifest["snapshot"],
        "runtime_snapshot": validated_runtime["snapshot"],
    }
    grant: dict[str, Any] = {
        "contract_version": ROS2_EXECUTION_GRANT_CONTRACT_VERSION,
        "grant_id": "ros2-grant-" + _snapshot(seed)[:24],
        "issued_at": _format_datetime(checked_at),
        "expires_at": _format_datetime(datetime.fromtimestamp(expires_at, timezone.utc)),
        "profile_id": validated_manifest["profile_id"],
        "robot_id": validated_manifest["robot_id"],
        "deployment_mode": validated_runtime["deployment_mode"],
        "workflow_id": binding.workflow_id,
        "target_space_id": binding.target_space_id,
        "endpoint_id": binding.endpoint_id,
        "resource_id": binding.resource_id,
        "adapter_id": binding.adapter_id,
        "capability_id": binding.capability_id,
        "resource_plan_snapshot": binding.plan_snapshot,
        "profile_snapshot": validated_manifest["snapshot"],
        "runtime_snapshot": validated_runtime["snapshot"],
        "resource_observation_sequence": binding.observation_sequence,
        "runtime_observation_sequence": runtime_state["observation_sequence"],
        "require_confirmation": binding.require_confirmation,
    }
    grant["snapshot"] = _snapshot(grant)
    return parse_ros2_execution_grant(grant)


def parse_ros2_execution_grant(value: Any) -> dict[str, Any]:
    """Validate a redacted, content-addressed, short-lived execution grant."""

    if not isinstance(value, Mapping):
        raise Ros2ExecutionError("ROS 2 execution grant must be an object")
    missing = sorted(_GRANT_FIELDS - set(value))
    unsupported = sorted(set(value) - _GRANT_FIELDS)
    if missing:
        raise Ros2ExecutionError("ROS 2 execution grant is missing: " + ", ".join(missing))
    if unsupported:
        raise Ros2ExecutionError(
            "ROS 2 execution grant has unsupported fields: " + ", ".join(unsupported)
        )
    if value["contract_version"] != ROS2_EXECUTION_GRANT_CONTRACT_VERSION:
        raise Ros2ExecutionError(
            f"contract_version must be {ROS2_EXECUTION_GRANT_CONTRACT_VERSION}"
        )
    for field in (
        "grant_id",
        "profile_id",
        "robot_id",
        "workflow_id",
        "target_space_id",
        "endpoint_id",
        "resource_id",
        "adapter_id",
        "capability_id",
    ):
        _identifier(value[field], field)
    if value["deployment_mode"] not in {"simulation", "hardware"}:
        raise Ros2ExecutionError("deployment_mode must be simulation or hardware")
    issued_at = parse_observed_at(value["issued_at"], "issued_at")
    expires_at = parse_observed_at(value["expires_at"], "expires_at")
    lifetime = (expires_at - issued_at).total_seconds()
    if not 0.0 < lifetime <= MAX_GRANT_LIFETIME_SECONDS:
        raise Ros2ExecutionError("execution grant lifetime must be 1 to 300 seconds")
    for field in (
        "resource_plan_snapshot",
        "profile_snapshot",
        "runtime_snapshot",
        "snapshot",
    ):
        _snapshot_text(value[field], field)
    for field in (
        "resource_observation_sequence",
        "runtime_observation_sequence",
    ):
        sequence = value[field]
        if sequence is not None:
            _text(sequence, field, maximum=256)
    if type(value["require_confirmation"]) is not bool:
        raise Ros2ExecutionError("require_confirmation must be boolean")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _snapshot(unsigned):
        raise Ros2ExecutionError("ROS 2 execution grant snapshot does not match")
    return dict(value)


def resolve_ros2_execution_target(
    grant: Mapping[str, Any],
    manifest: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    observed_at: datetime | None = None,
) -> Ros2ExecutionTarget:
    """Resolve private graph details only inside the deterministic adapter."""

    validated_grant = parse_ros2_execution_grant(grant)
    validated_manifest = parse_ros2_adapter_manifest(manifest)
    validated_runtime = parse_ros2_runtime_snapshot(runtime)
    checked_at = _utc(observed_at or datetime.now(timezone.utc), "observed_at")
    if checked_at > parse_observed_at(validated_grant["expires_at"], "expires_at"):
        raise Ros2ExecutionError("ROS 2 execution grant has expired")
    exact_matches = {
        "profile_id": validated_manifest["profile_id"],
        "robot_id": validated_manifest["robot_id"],
        "deployment_mode": validated_runtime["deployment_mode"],
        "profile_snapshot": validated_manifest["snapshot"],
        "runtime_snapshot": validated_runtime["snapshot"],
    }
    for field, expected in exact_matches.items():
        if validated_grant[field] != expected:
            raise Ros2ExecutionError(f"execution grant {field} does not match")
    pairing = verify_ros2_pairing(
        validated_manifest,
        validated_runtime,
        observed_at=checked_at,
    )
    if pairing["passed"] is not True:
        raise Ros2ExecutionError("ROS 2 runtime is no longer ready")
    adapters = [
        adapter
        for adapter in validated_manifest["adapters"]
        if adapter["adapter_id"] == validated_grant["adapter_id"]
        and validated_grant["capability_id"] in adapter["capability_ids"]
    ]
    if len(adapters) != 1:
        raise Ros2ExecutionError("execution grant adapter binding is unavailable")
    adapter = adapters[0]
    return Ros2ExecutionTarget(
        adapter_id=adapter["adapter_id"],
        capability_id=validated_grant["capability_id"],
        interface_kind=adapter["interface"]["kind"],
        interface_name=adapter["interface"]["name"],
        interface_type=adapter["interface"]["type"],
        timeout_seconds=float(adapter["timeout_seconds"]),
        grant_snapshot=validated_grant["snapshot"],
        resource_plan_snapshot=validated_grant["resource_plan_snapshot"],
        profile_snapshot=validated_grant["profile_snapshot"],
        runtime_snapshot=validated_grant["runtime_snapshot"],
    )


def _snapshot(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    ).hexdigest()


def _snapshot_text(value: Any, label: str) -> str:
    text = str(value or "")
    if len(text) != 64 or any(char not in "0123456789abcdef" for char in text):
        raise Ros2ExecutionError(f"{label} must be a lowercase SHA-256 digest")
    return text


def _identifier(value: Any, label: str) -> str:
    text = _text(value, label)
    if _IDENTIFIER.fullmatch(text) is None:
        raise Ros2ExecutionError(f"{label} is invalid")
    return text


def _text(value: Any, label: str, *, maximum: int = 192) -> str:
    if not isinstance(value, str):
        raise Ros2ExecutionError(f"{label} must be text")
    text = value.strip()
    if not text or len(text) > maximum or any(ord(char) < 32 for char in text):
        raise Ros2ExecutionError(f"{label} must be bounded non-empty text")
    return text


def _utc(value: datetime, label: str) -> datetime:
    if value.tzinfo is None:
        raise Ros2ExecutionError(f"{label} must include a UTC offset")
    return value.astimezone(timezone.utc)


def _format_datetime(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")
