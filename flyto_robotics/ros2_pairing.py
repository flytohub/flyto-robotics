"""Fail-closed semantic pairing contracts for Flyto2 and ROS 2.

The manifest is deployment-only configuration. AI-facing callers receive a
redacted profile summary and runtime readiness evidence, never ROS interface
names or actuator values.
"""

from __future__ import annotations

import hashlib
import json
import re
from collections.abc import Mapping
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .capabilities import default_capability_registry

ROS2_ADAPTER_CONTRACT_VERSION = "flyto.robotics.ros2-adapter-manifest.v1"
ROS2_RUNTIME_CONTRACT_VERSION = "flyto.robotics.ros2-runtime-snapshot.v1"
ROS2_PAIRING_CONTRACT_VERSION = "flyto.robotics.ros2-pairing-report.v1"
ROS2_PROFILE_CONTRACT_VERSION = "flyto.robotics.mcp-ros2-profile.v1"
MAX_DOCUMENT_BYTES = 256 * 1024
MAX_ADAPTERS = 64
PORTABLE_MODES = frozenset({"simulation", "hardware"})

_MANIFEST_FIELDS = {
    "contract_version",
    "profile_id",
    "robot_id",
    "ros_version",
    "fail_closed",
    "direct_actuation",
    "emergency_stop_required",
    "adapters",
    "snapshot",
}
_ADAPTER_FIELDS = {
    "adapter_id",
    "capability_ids",
    "stack",
    "interface",
    "managed_nodes",
    "required_observations",
    "timeout_seconds",
    "supported_modes",
    "feedback_required",
    "cancel_on_timeout",
    "lifecycle_required",
    "goal_policy",
}
_INTERFACE_FIELDS = {"kind", "name", "type"}
_RUNTIME_FIELDS = {
    "contract_version",
    "profile_id",
    "profile_snapshot",
    "robot_id",
    "deployment_mode",
    "observed_at",
    "max_age_seconds",
    "emergency_stop_ready",
    "adapters",
    "snapshot",
}
_RUNTIME_ADAPTER_FIELDS = {
    "adapter_id",
    "status",
    "interface_available",
    "lifecycle_state",
    "observation_sequence",
}
_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,191}$")
_INTERFACE_TYPE = re.compile(
    r"^[A-Za-z][A-Za-z0-9_]*/(action|srv)/[A-Za-z][A-Za-z0-9_]*$"
)
_ROS_GRAPH_NAME = re.compile(r"^/[A-Za-z0-9_]+(?:/[A-Za-z0-9_]+)*$")
_UNSAFE_INTERFACE_MARKERS = (
    "cmd_vel",
    "motor_command",
    "motor_pwm",
    "wheel_pwm",
    "wheel_speed",
    "joint_jog",
)
_STACK_INTERFACES = {
    "nav2": frozenset({"nav2_msgs/action/NavigateToPose"}),
    "moveit2": frozenset(
        {
            "moveit_msgs/action/MoveGroup",
            "moveit_msgs/action/Pickup",
            "moveit_msgs/action/Place",
        }
    ),
    "ros2_control": frozenset(
        {
            "control_msgs/action/FollowJointTrajectory",
            "control_msgs/action/GripperCommand",
        }
    ),
}


class Ros2PairingError(ValueError):
    """Raised when a ROS 2 manifest or readiness snapshot is unsafe."""


def parse_ros2_adapter_manifest(value: Any) -> dict[str, Any]:
    """Validate a deployment-only ROS 2 semantic adapter manifest."""

    manifest = _object(value, "ROS 2 adapter manifest")
    _require_exact_fields(manifest, _MANIFEST_FIELDS, "ROS 2 adapter manifest")
    if manifest["contract_version"] != ROS2_ADAPTER_CONTRACT_VERSION:
        raise Ros2PairingError(
            f"contract_version must be {ROS2_ADAPTER_CONTRACT_VERSION}"
        )
    _identifier(manifest["profile_id"], "profile_id")
    _identifier(manifest["robot_id"], "robot_id")
    _integer(manifest["ros_version"], "ros_version", 2, 2)
    if manifest["fail_closed"] is not True:
        raise Ros2PairingError("ROS 2 adapter manifest must fail closed")
    if manifest["direct_actuation"] is not False:
        raise Ros2PairingError("direct_actuation must be false")
    if manifest["emergency_stop_required"] is not True:
        raise Ros2PairingError("emergency_stop_required must be true")

    adapters = manifest["adapters"]
    if not isinstance(adapters, list) or not 1 <= len(adapters) <= MAX_ADAPTERS:
        raise Ros2PairingError("adapters must contain 1 to 64 entries")
    catalog = {
        capability["canonical_id"]: capability
        for capability in default_capability_registry().catalog()
    }
    adapter_ids: set[str] = set()
    capability_ids: set[str] = set()
    for index, raw_adapter in enumerate(adapters):
        label = f"adapters[{index}]"
        adapter = _object(raw_adapter, label)
        _require_exact_fields(adapter, _ADAPTER_FIELDS, label)
        adapter_id = _identifier(adapter["adapter_id"], f"{label}.adapter_id")
        if adapter_id in adapter_ids:
            raise Ros2PairingError("adapter_id values must be unique")
        adapter_ids.add(adapter_id)

        declared_capabilities = _text_list(
            adapter["capability_ids"],
            f"{label}.capability_ids",
            minimum=1,
            maximum=16,
            identifiers=True,
        )
        for capability_id in declared_capabilities:
            if capability_id not in catalog:
                raise Ros2PairingError(
                    f"{label}.capability_ids contains an unregistered capability"
                )
            if capability_id in capability_ids:
                raise Ros2PairingError(
                    "each capability_id must be owned by exactly one adapter"
                )
            capability_ids.add(capability_id)

        stack = _text(adapter["stack"], f"{label}.stack", maximum=32)
        if stack not in {*_STACK_INTERFACES, "flyto"}:
            raise Ros2PairingError(f"{label}.stack is unsupported")
        interface = _object(adapter["interface"], f"{label}.interface")
        _require_exact_fields(interface, _INTERFACE_FIELDS, f"{label}.interface")
        interface_kind = _text(
            interface["kind"], f"{label}.interface.kind", maximum=16
        )
        if interface_kind not in {"action", "service"}:
            raise Ros2PairingError(
                f"{label}.interface.kind must be action or service"
            )
        interface_name = _text(
            interface["name"], f"{label}.interface.name", maximum=192
        )
        interface_type = _text(
            interface["type"], f"{label}.interface.type", maximum=192
        )
        if _ROS_GRAPH_NAME.fullmatch(interface_name) is None:
            raise Ros2PairingError(
                f"{label}.interface.name must be an absolute ROS graph name"
            )
        unsafe_surface = f"{interface_name} {interface_type}".lower()
        if any(marker in unsafe_surface for marker in _UNSAFE_INTERFACE_MARKERS):
            raise Ros2PairingError(
                f"{label}.interface exposes a raw actuator surface"
            )
        if _INTERFACE_TYPE.fullmatch(interface_type) is None:
            raise Ros2PairingError(f"{label}.interface.type is invalid")
        expected_segment = "/action/" if interface_kind == "action" else "/srv/"
        if expected_segment not in interface_type:
            raise Ros2PairingError(
                f"{label}.interface kind and type do not match"
            )
        if stack in _STACK_INTERFACES and interface_type not in _STACK_INTERFACES[stack]:
            raise Ros2PairingError(
                f"{label}.interface.type is not approved for {stack}"
            )
        if stack == "flyto" and not interface_type.startswith("flyto_robotics/"):
            raise Ros2PairingError(
                f"{label}.interface.type must use the flyto_robotics package"
            )

        managed_nodes = _text_list(
            adapter["managed_nodes"],
            f"{label}.managed_nodes",
            minimum=1,
            maximum=16,
            identifiers=False,
        )
        if any(_ROS_GRAPH_NAME.fullmatch(node) is None for node in managed_nodes):
            raise Ros2PairingError(
                f"{label}.managed_nodes must contain absolute ROS node names"
            )
        required_observations = _text_list(
            adapter["required_observations"],
            f"{label}.required_observations",
            minimum=0,
            maximum=32,
            identifiers=True,
        )
        required_by_capabilities = {
            observation
            for capability_id in declared_capabilities
            for observation in catalog[capability_id]["required_observations"]
        }
        if not required_by_capabilities.issubset(required_observations):
            raise Ros2PairingError(
                f"{label}.required_observations omits capability safety inputs"
            )
        timeout = _number(
            adapter["timeout_seconds"], f"{label}.timeout_seconds", 0.1, 3600.0
        )
        if timeout <= 0.0:
            raise Ros2PairingError(f"{label}.timeout_seconds must be positive")
        modes = set(
            _text_list(
                adapter["supported_modes"],
                f"{label}.supported_modes",
                minimum=2,
                maximum=2,
                identifiers=False,
            )
        )
        if modes != PORTABLE_MODES:
            raise Ros2PairingError(
                f"{label}.supported_modes must include simulation and hardware"
            )
        if adapter["goal_policy"] != "semantic_only":
            raise Ros2PairingError(f"{label}.goal_policy must be semantic_only")
        for field in (
            "feedback_required",
            "cancel_on_timeout",
            "lifecycle_required",
        ):
            if adapter[field] is not True:
                raise Ros2PairingError(f"{label}.{field} must be true")
        if any(
            catalog[capability_id]["control_class"] == "motion"
            for capability_id in declared_capabilities
        ) and interface_kind != "action":
            raise Ros2PairingError(
                f"{label} motion capabilities require a cancellable action"
            )

    supplied_snapshot = _snapshot_text(manifest["snapshot"], "snapshot")
    unsigned = {key: item for key, item in manifest.items() if key != "snapshot"}
    if supplied_snapshot != _snapshot(unsigned):
        raise Ros2PairingError("ROS 2 adapter manifest snapshot does not match")
    return dict(manifest)


def parse_ros2_runtime_snapshot(value: Any) -> dict[str, Any]:
    """Validate bounded, content-addressed ROS 2 readiness evidence."""

    runtime = _object(value, "ROS 2 runtime snapshot")
    _require_exact_fields(runtime, _RUNTIME_FIELDS, "ROS 2 runtime snapshot")
    if runtime["contract_version"] != ROS2_RUNTIME_CONTRACT_VERSION:
        raise Ros2PairingError(
            f"contract_version must be {ROS2_RUNTIME_CONTRACT_VERSION}"
        )
    _identifier(runtime["profile_id"], "profile_id")
    _snapshot_text(runtime["profile_snapshot"], "profile_snapshot")
    _identifier(runtime["robot_id"], "robot_id")
    if runtime["deployment_mode"] not in PORTABLE_MODES:
        raise Ros2PairingError("deployment_mode must be simulation or hardware")
    parse_observed_at(runtime["observed_at"], "observed_at")
    _integer(runtime["max_age_seconds"], "max_age_seconds", 1, 300)
    if type(runtime["emergency_stop_ready"]) is not bool:
        raise Ros2PairingError("emergency_stop_ready must be boolean")

    adapters = runtime["adapters"]
    if not isinstance(adapters, list) or not 1 <= len(adapters) <= MAX_ADAPTERS:
        raise Ros2PairingError("adapters must contain 1 to 64 readiness entries")
    adapter_ids: set[str] = set()
    for index, raw_adapter in enumerate(adapters):
        label = f"adapters[{index}]"
        adapter = _object(raw_adapter, label)
        _require_exact_fields(adapter, _RUNTIME_ADAPTER_FIELDS, label)
        adapter_id = _identifier(adapter["adapter_id"], f"{label}.adapter_id")
        if adapter_id in adapter_ids:
            raise Ros2PairingError("runtime adapter_id values must be unique")
        adapter_ids.add(adapter_id)
        if adapter["status"] not in {"ready", "degraded", "unavailable"}:
            raise Ros2PairingError(f"{label}.status is unsupported")
        if type(adapter["interface_available"]) is not bool:
            raise Ros2PairingError(f"{label}.interface_available must be boolean")
        if adapter["lifecycle_state"] not in {
            "active",
            "inactive",
            "unconfigured",
            "finalized",
            "error",
            "unknown",
        }:
            raise Ros2PairingError(f"{label}.lifecycle_state is unsupported")
        _text(
            adapter["observation_sequence"],
            f"{label}.observation_sequence",
            maximum=256,
        )

    supplied_snapshot = _snapshot_text(runtime["snapshot"], "snapshot")
    unsigned = {key: item for key, item in runtime.items() if key != "snapshot"}
    if supplied_snapshot != _snapshot(unsigned):
        raise Ros2PairingError("ROS 2 runtime snapshot does not match")
    return dict(runtime)


def load_ros2_adapter_manifest(path: str | Path) -> dict[str, Any]:
    """Load one bounded UTF-8 adapter manifest."""

    return parse_ros2_adapter_manifest(_load_json(path, "ROS 2 adapter manifest"))


def load_ros2_runtime_snapshot(path: str | Path) -> dict[str, Any]:
    """Load one bounded UTF-8 runtime snapshot."""

    return parse_ros2_runtime_snapshot(_load_json(path, "ROS 2 runtime snapshot"))


def standard_ros2_adapter_manifest(robot_id: str) -> dict[str, Any]:
    """Build the portable Nav2 profile for the currently executable atoms."""

    normalized_robot_id = _identifier(robot_id, "robot_id")
    manifest: dict[str, Any] = {
        "contract_version": ROS2_ADAPTER_CONTRACT_VERSION,
        "profile_id": "flyto2.ros2.standard.v1",
        "robot_id": normalized_robot_id,
        "ros_version": 2,
        "fail_closed": True,
        "direct_actuation": False,
        "emergency_stop_required": True,
        "adapters": [
            {
                "adapter_id": "ros2.nav2.navigate_to_pose.v1",
                "capability_ids": [
                    "robotics.motion.navigate@1",
                    "robotics.motion.navigate_to_location@1",
                ],
                "stack": "nav2",
                "interface": {
                    "kind": "action",
                    "name": "/navigate_to_pose",
                    "type": "nav2_msgs/action/NavigateToPose",
                },
                "managed_nodes": [
                    "/bt_navigator",
                    "/controller_server",
                    "/planner_server",
                ],
                "required_observations": ["minimum_range", "odometry"],
                "timeout_seconds": 300.0,
                "supported_modes": ["hardware", "simulation"],
                "feedback_required": True,
                "cancel_on_timeout": True,
                "lifecycle_required": True,
                "goal_policy": "semantic_only",
            }
        ],
    }
    manifest["snapshot"] = _snapshot(manifest)
    return parse_ros2_adapter_manifest(manifest)


def ros2_profile_summary(manifest: Mapping[str, Any]) -> dict[str, Any]:
    """Return an AI-safe profile view with all ROS graph details removed."""

    validated = parse_ros2_adapter_manifest(manifest)
    return {
        "contract_version": ROS2_PROFILE_CONTRACT_VERSION,
        "profile_id": validated["profile_id"],
        "profile_snapshot": validated["snapshot"],
        "robot_id": validated["robot_id"],
        "portable_modes": sorted(PORTABLE_MODES),
        "emergency_stop_required": validated["emergency_stop_required"],
        "adapters": [
            {
                "adapter_id": adapter["adapter_id"],
                "capability_ids": list(adapter["capability_ids"]),
                "stack": adapter["stack"],
                "goal_policy": adapter["goal_policy"],
                "feedback_required": adapter["feedback_required"],
                "cancel_on_timeout": adapter["cancel_on_timeout"],
                "lifecycle_required": adapter["lifecycle_required"],
            }
            for adapter in validated["adapters"]
        ],
    }


def verify_ros2_pairing(
    manifest: Mapping[str, Any],
    runtime: Mapping[str, Any],
    *,
    observed_at: datetime | None = None,
) -> dict[str, Any]:
    """Return deterministic readiness evidence without exposing ROS graph names."""

    validated_manifest = parse_ros2_adapter_manifest(manifest)
    validated_runtime = parse_ros2_runtime_snapshot(runtime)
    checked_at = observed_at or datetime.now(timezone.utc)
    if checked_at.tzinfo is None:
        raise Ros2PairingError("observed_at must include a UTC offset")
    checked_at = checked_at.astimezone(timezone.utc)
    runtime_time = parse_observed_at(
        validated_runtime["observed_at"], "runtime observed_at"
    )
    age_seconds = (checked_at - runtime_time).total_seconds()

    checks: list[dict[str, Any]] = []

    def add(code: str, passed: bool, detail: str) -> None:
        checks.append({"code": code, "passed": passed, "detail": detail})

    add(
        "profile_id_match",
        validated_runtime["profile_id"] == validated_manifest["profile_id"],
        "runtime profile identity matches the deployment manifest",
    )
    add(
        "profile_snapshot_match",
        validated_runtime["profile_snapshot"] == validated_manifest["snapshot"],
        "runtime evidence is bound to the exact deployment manifest",
    )
    add(
        "robot_id_match",
        validated_runtime["robot_id"] == validated_manifest["robot_id"],
        "runtime evidence belongs to the requested robot",
    )
    fresh = -5.0 <= age_seconds <= validated_runtime["max_age_seconds"]
    add(
        "runtime_fresh",
        fresh,
        "runtime evidence is recent and not materially future-dated",
    )
    add(
        "emergency_stop_ready",
        validated_runtime["emergency_stop_ready"] is True,
        "an independent emergency stop is ready",
    )

    expected = {
        adapter["adapter_id"]: adapter for adapter in validated_manifest["adapters"]
    }
    actual = {
        adapter["adapter_id"]: adapter for adapter in validated_runtime["adapters"]
    }
    add(
        "adapter_set_exact",
        set(expected) == set(actual),
        "runtime evidence contains exactly the declared adapters",
    )
    ready_adapter_ids: list[str] = []
    ready_capability_ids: list[str] = []
    for adapter_id, adapter in expected.items():
        state = actual.get(adapter_id)
        available = state is not None and state["interface_available"] is True
        ready = state is not None and state["status"] == "ready"
        active = state is not None and state["lifecycle_state"] == "active"
        add(
            f"adapter.{adapter_id}.interface_available",
            available,
            "declared semantic interface is available",
        )
        add(
            f"adapter.{adapter_id}.ready",
            ready,
            "adapter reports ready instead of degraded or unavailable",
        )
        add(
            f"adapter.{adapter_id}.lifecycle_active",
            active,
            "all managed ROS 2 lifecycle nodes report active",
        )
        if available and ready and active:
            ready_adapter_ids.append(adapter_id)
            ready_capability_ids.extend(adapter["capability_ids"])

    passed = all(check["passed"] is True for check in checks)
    return {
        "contract_version": ROS2_PAIRING_CONTRACT_VERSION,
        "passed": passed,
        "profile_id": validated_manifest["profile_id"],
        "profile_snapshot": validated_manifest["snapshot"],
        "runtime_snapshot": validated_runtime["snapshot"],
        "robot_id": validated_manifest["robot_id"],
        "deployment_mode": validated_runtime["deployment_mode"],
        "checked_at": _format_datetime(checked_at),
        "ready_adapter_ids": sorted(ready_adapter_ids) if passed else [],
        "ready_capability_ids": sorted(set(ready_capability_ids)) if passed else [],
        "checks": checks,
    }


def parse_observed_at(value: Any, label: str = "observed_at") -> datetime:
    """Parse one timezone-aware ISO-8601 timestamp and normalize it to UTC."""

    text = _text(value, label, maximum=64)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise Ros2PairingError(f"{label} must be an ISO-8601 timestamp") from exc
    if parsed.tzinfo is None:
        raise Ros2PairingError(f"{label} must include a UTC offset")
    return parsed.astimezone(timezone.utc)


def _load_json(path: str | Path, label: str) -> Any:
    source = Path(path)
    try:
        if source.stat().st_size > MAX_DOCUMENT_BYTES:
            raise Ros2PairingError(
                f"{label} file exceeds {MAX_DOCUMENT_BYTES} bytes"
            )
        return json.loads(source.read_text(encoding="utf-8"))
    except Ros2PairingError:
        raise
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise Ros2PairingError(
            f"{label} file must contain readable UTF-8 JSON"
        ) from exc


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
        raise Ros2PairingError(f"{label} must be a lowercase SHA-256 digest")
    return text


def _format_datetime(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _object(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise Ros2PairingError(f"{label} must be an object")
    return value


def _require_exact_fields(
    value: Mapping[str, Any], expected: set[str], label: str
) -> None:
    missing = sorted(expected - set(value))
    unsupported = sorted(set(value) - expected)
    if missing:
        raise Ros2PairingError(f"{label} is missing: {', '.join(missing)}")
    if unsupported:
        raise Ros2PairingError(
            f"{label} has unsupported fields: {', '.join(unsupported)}"
        )


def _text(value: Any, label: str, *, maximum: int = 192) -> str:
    if not isinstance(value, str):
        raise Ros2PairingError(f"{label} must be text")
    text = value.strip()
    if not text or len(text) > maximum or any(ord(char) < 32 for char in text):
        raise Ros2PairingError(f"{label} must be bounded non-empty text")
    return text


def _identifier(value: Any, label: str) -> str:
    text = _text(value, label)
    if _IDENTIFIER.fullmatch(text) is None:
        raise Ros2PairingError(f"{label} is invalid")
    return text


def _text_list(
    value: Any,
    label: str,
    *,
    minimum: int,
    maximum: int,
    identifiers: bool,
) -> list[str]:
    if not isinstance(value, list) or not minimum <= len(value) <= maximum:
        raise Ros2PairingError(
            f"{label} must contain {minimum} to {maximum} text values"
        )
    parsed = [
        _identifier(item, f"{label}[{index}]")
        if identifiers
        else _text(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    ]
    if len(set(parsed)) != len(parsed):
        raise Ros2PairingError(f"{label} values must be unique")
    return parsed


def _integer(value: Any, label: str, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise Ros2PairingError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise Ros2PairingError(f"{label} must be between {minimum} and {maximum}")
    return value


def _number(
    value: Any, label: str, minimum: float, maximum: float
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise Ros2PairingError(f"{label} must be a number")
    number = float(value)
    if not minimum <= number <= maximum:
        raise Ros2PairingError(f"{label} must be between {minimum} and {maximum}")
    return number
