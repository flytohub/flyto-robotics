"""Strict Flyto2 resource-plan boundary for ROS, Gazebo, and physical adapters."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections.abc import Mapping, Sequence
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

RESOURCE_PLAN_CONTRACT_VERSION = "ai-space-resource-plan.v1"
MAX_RESOURCE_PLAN_BYTES = 256 * 1024
MAX_SELECTED_ENDPOINTS = 128
_PLAN_FIELDS = {
    "schema_version",
    "request_id",
    "workflow_id",
    "target_space_id",
    "fail_closed",
    "executable",
    "configured_endpoint_count",
    "requirement_count",
    "selected",
    "missing",
    "excluded",
    "snapshot",
}
_STEP_FIELDS = {
    "step",
    "requirement_id",
    "unit",
    "capability_id",
    "lease_only",
    "endpoint_id",
    "resource_id",
    "adapter_id",
    "target_space_id",
    "score",
    "observation_sequence",
    "require_confirmation",
}


class ResourceBindingError(ValueError):
    """Raised when a resource plan cannot safely authorize this adapter."""


@dataclass(frozen=True)
class ResourceBinding:
    """Exact resource slice authorized for one workflow and adapter."""

    plan_snapshot: str
    request_id: str
    workflow_id: str
    target_space_id: str
    endpoint_id: str
    resource_id: str
    adapter_id: str
    capability_id: str
    observation_sequence: str | None
    require_confirmation: bool

    def evidence(self) -> dict[str, Any]:
        """Return a payload-free audit record suitable for result envelopes."""

        return asdict(self)


def parse_resource_plan(value: Any) -> dict[str, Any]:
    """Validate an immutable AI Space resource plan without importing Cloud."""

    plan = _object(value, "resource plan")
    _require_exact_fields(plan, _PLAN_FIELDS, "resource plan")
    if plan["schema_version"] != RESOURCE_PLAN_CONTRACT_VERSION:
        raise ResourceBindingError(
            f"schema_version must be {RESOURCE_PLAN_CONTRACT_VERSION}"
        )
    _text(plan["request_id"], "request_id")
    _text(plan["workflow_id"], "workflow_id")
    _text(plan["target_space_id"], "target_space_id")
    if plan["fail_closed"] is not True:
        raise ResourceBindingError("resource plan must fail closed")
    if plan["executable"] is not True:
        raise ResourceBindingError("resource plan is not executable")
    _integer(plan["configured_endpoint_count"], "configured_endpoint_count", 1, 4096)
    _integer(plan["requirement_count"], "requirement_count", 1, 64)
    if plan["missing"] != []:
        raise ResourceBindingError("executable resource plan cannot have missing items")
    if not isinstance(plan["excluded"], list) or len(plan["excluded"]) > 8192:
        raise ResourceBindingError("excluded must be a bounded array")

    selected = plan["selected"]
    if (
        not isinstance(selected, list)
        or not selected
        or len(selected) > MAX_SELECTED_ENDPOINTS
    ):
        raise ResourceBindingError("selected must contain 1 to 128 endpoints")
    for index, raw_step in enumerate(selected):
        step = _object(raw_step, f"selected[{index}]")
        _require_exact_fields(step, _STEP_FIELDS, f"selected[{index}]")
        _integer(step["step"], f"selected[{index}].step", 1, MAX_SELECTED_ENDPOINTS)
        _integer(step["unit"], f"selected[{index}].unit", 1, 4)
        _integer(step["score"], f"selected[{index}].score", -1_000_000, 1_000_000)
        for field in (
            "requirement_id",
            "capability_id",
            "endpoint_id",
            "resource_id",
            "adapter_id",
            "target_space_id",
        ):
            _text(step[field], f"selected[{index}].{field}")
        if type(step["lease_only"]) is not bool:
            raise ResourceBindingError(
                f"selected[{index}].lease_only must be boolean"
            )
        if type(step["require_confirmation"]) is not bool:
            raise ResourceBindingError(
                f"selected[{index}].require_confirmation must be boolean"
            )
        sequence = step["observation_sequence"]
        if sequence is not None:
            _text(sequence, f"selected[{index}].observation_sequence", maximum=256)

    supplied_snapshot = _snapshot_text(plan["snapshot"])
    unsigned = {key: item for key, item in plan.items() if key != "snapshot"}
    if supplied_snapshot != _snapshot(unsigned):
        raise ResourceBindingError("resource plan snapshot does not match")
    return dict(plan)


def load_resource_plan(path: str | Path) -> dict[str, Any]:
    """Load and validate a bounded UTF-8 resource-plan document."""

    source = Path(path)
    try:
        size = source.stat().st_size
    except OSError as exc:
        raise ResourceBindingError("resource plan file is not readable") from exc
    if size > MAX_RESOURCE_PLAN_BYTES:
        raise ResourceBindingError(
            f"resource plan file exceeds {MAX_RESOURCE_PLAN_BYTES} bytes"
        )
    try:
        decoded = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ResourceBindingError(
            "resource plan file must contain valid UTF-8 JSON"
        ) from exc
    return parse_resource_plan(decoded)


def select_resource_binding(
    plan: Mapping[str, Any],
    *,
    workflow_id: str,
    resource_id: str,
    capability_id: str,
    allowed_adapter_ids: Sequence[str],
    target_space_id: str = "",
    confirmed: bool = False,
) -> ResourceBinding:
    """Select exactly one pre-authorized endpoint; never infer an adapter."""

    validated = parse_resource_plan(plan)
    expected_workflow = _text(workflow_id, "expected workflow_id")
    expected_resource = _text(resource_id, "expected resource_id")
    expected_capability = _text(capability_id, "expected capability_id")
    expected_space = str(target_space_id or "").strip()
    allowed = {
        _text(item, "allowed adapter_id")
        for item in allowed_adapter_ids
        if str(item or "").strip()
    }
    if not allowed:
        raise ResourceBindingError("at least one allowed adapter_id is required")
    if validated["workflow_id"] != expected_workflow:
        raise ResourceBindingError("resource plan workflow_id does not match")
    if expected_space and validated["target_space_id"] != expected_space:
        raise ResourceBindingError("resource plan target_space_id does not match")

    matches = [
        step
        for step in validated["selected"]
        if step["resource_id"] == expected_resource
        and step["capability_id"] == expected_capability
        and step["adapter_id"] in allowed
        and step["lease_only"] is False
        and (not expected_space or step["target_space_id"] == expected_space)
    ]
    if len(matches) != 1:
        raise ResourceBindingError(
            "resource plan must select exactly one matching executable endpoint"
        )
    selected = matches[0]
    if selected["require_confirmation"] and not confirmed:
        raise ResourceBindingError("resource endpoint requires confirmation")
    return ResourceBinding(
        plan_snapshot=validated["snapshot"],
        request_id=validated["request_id"],
        workflow_id=validated["workflow_id"],
        target_space_id=selected["target_space_id"],
        endpoint_id=selected["endpoint_id"],
        resource_id=selected["resource_id"],
        adapter_id=selected["adapter_id"],
        capability_id=selected["capability_id"],
        observation_sequence=selected["observation_sequence"],
        require_confirmation=selected["require_confirmation"],
    )


def _snapshot(value: Mapping[str, Any]) -> str:
    return hashlib.sha256(
        json.dumps(
            value,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
            default=str,
        ).encode("utf-8")
    ).hexdigest()


def _snapshot_text(value: Any) -> str:
    text = str(value or "")
    if len(text) != 64 or any(character not in "0123456789abcdef" for character in text):
        raise ResourceBindingError("snapshot must be a lowercase SHA-256 digest")
    return text


def _object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ResourceBindingError(f"{label} must be an object")
    return value


def _require_exact_fields(
    value: Mapping[str, Any],
    expected: set[str],
    label: str,
) -> None:
    missing = sorted(expected - set(value))
    unsupported = sorted(set(value) - expected)
    if missing:
        raise ResourceBindingError(f"{label} is missing: {', '.join(missing)}")
    if unsupported:
        raise ResourceBindingError(
            f"{label} has unsupported fields: {', '.join(unsupported)}"
        )


def _text(value: Any, label: str, *, maximum: int = 128) -> str:
    if not isinstance(value, str):
        raise ResourceBindingError(f"{label} must be text")
    text = value.strip()
    if (
        not text
        or len(text) > maximum
        or any(character.isspace() or ord(character) < 32 for character in text)
    ):
        raise ResourceBindingError(
            f"{label} must be a non-empty identifier of at most {maximum} characters"
        )
    return text


def _integer(value: Any, label: str, minimum: int, maximum: int) -> int:
    if type(value) is not int or not minimum <= value <= maximum:
        raise ResourceBindingError(
            f"{label} must be an integer between {minimum} and {maximum}"
        )
    return value


def main(argv: Sequence[str] | None = None) -> int:
    """Validate one plan and print its exact adapter evidence."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan")
    parser.add_argument("--workflow", required=True)
    parser.add_argument("--resource", required=True)
    parser.add_argument("--capability", required=True)
    parser.add_argument("--adapter", action="append", required=True)
    parser.add_argument("--space", default="")
    parser.add_argument("--confirmed", action="store_true")
    args = parser.parse_args(argv)
    try:
        plan = load_resource_plan(args.plan)
        binding = select_resource_binding(
            plan,
            workflow_id=args.workflow,
            resource_id=args.resource,
            capability_id=args.capability,
            allowed_adapter_ids=args.adapter,
            target_space_id=args.space,
            confirmed=args.confirmed,
        )
    except ResourceBindingError as exc:
        parser.error(str(exc))
    print(json.dumps(binding.evidence(), ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
