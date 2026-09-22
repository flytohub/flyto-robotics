"""Provider-neutral external adapter contract owned by Flyto2 Robotics.

This module deliberately imports no Cloud/Core source.  It mirrors only the
small JSON-safe capability surface that an execution host needs to discover and
invoke an installed adapter.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

OUTCOME_COMPLETED = "completed"
OUTCOME_REFUSED = "refused"
OUTCOME_TIMEOUT = "timeout"
OUTCOME_CANCELLED = "cancelled"
OUTCOME_FAILED = "failed"

EXECUTOR_EXTERNAL_API = "external-api"
EXECUTOR_OPEN_RMF = "open-rmf"
SOURCE_DEVICE = "device_report"
STATUS_DISCOVERED = "DISCOVERED"
STATUS_APPROVED = "APPROVED"


class DeclarationRefused(ValueError):
    """An adapter declaration that contradicts the shared capability contract."""



_CAPABILITY_METADATA: Mapping[str, Mapping[str, Any]] = {
    "motion.navigate": {
        "display_name": "Navigate",
        "description": "Travel to a map coordinate or resolved destination and arrive there",
        "safety_class": "movement",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": True,
    },
    "motion.advance": {
        "display_name": "Advance",
        "description": "Move forward by a bounded relative distance",
        "safety_class": "movement",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": True,
    },
    "motion.retreat": {
        "display_name": "Retreat",
        "description": "Move backward by a bounded relative distance",
        "safety_class": "movement",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": True,
    },
    "motion.rotate": {
        "display_name": "Rotate",
        "description": "Rotate in place by a bounded angle",
        "safety_class": "movement",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": True,
    },
    "motion.halt": {
        "display_name": "Halt",
        "description": "Stop the machine's motion immediately",
        "safety_class": "controlled",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": False,
        "cancellable": False,
        "revision": 1,
    },
    "motion.dock": {
        "display_name": "Dock",
        "description": "Travel to an approved docking waypoint",
        "safety_class": "movement",
        "required_permissions": ("robot.motion",),
        "requires_safe_stop": True,
        "cancellable": True,
        "revision": 1,
    },
    "transport.load": {
        "display_name": "Load",
        "description": "Collect an approved payload at a named facility waypoint",
        "safety_class": "movement",
        "required_permissions": ("robot.transport",),
        "requires_safe_stop": True,
        "cancellable": True,
        "revision": 1,
    },
    "transport.unload": {
        "display_name": "Unload",
        "description": "Deliver an approved payload at a named facility waypoint",
        "safety_class": "movement",
        "required_permissions": ("robot.transport",),
        "requires_safe_stop": True,
        "cancellable": True,
        "revision": 1,
    },
    "vision.stream": {
        "display_name": "Vision Stream",
        "description": "Open an approved expiring live-view reference",
        "safety_class": "read_only",
        "required_permissions": (),
        "requires_safe_stop": False,
        "cancellable": False,
        "revision": 2,
    },
}

for _item in _CAPABILITY_METADATA.values():
    _item.setdefault("cancellable", True)
    _item.setdefault("revision", 1)

KNOWN_CAPABILITY_IDS = frozenset({
    *_CAPABILITY_METADATA,
    "motion.hop",
    "motion.ascend",
    "vision.observe",
})


@dataclass(frozen=True)
class DeclaredArgument:
    name: str
    type: str = "number"
    required: bool = False
    description: str = ""
    minimum: float | None = None
    maximum: float | None = None
    unit: str = ""

    @property
    def narrows(self) -> bool:
        return self.type == "number" and (
            self.minimum is not None or self.maximum is not None
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "name": self.name,
            "type": self.type,
            "required": self.required,
            "description": self.description,
            "minimum": self.minimum,
            "maximum": self.maximum,
            "unit": self.unit,
        }


@dataclass(frozen=True)
class CapabilityDeclaration:
    capability_id: str
    resource_id: str
    executor_kind: str
    source: str
    arguments: tuple[DeclaredArgument, ...]
    required_observations: tuple[str, ...]
    runtime_name: str
    version: str
    safety_class: str
    required_permissions: tuple[str, ...]
    requires_safe_stop: bool
    cancellable: bool
    revision: int
    schema_hash: str
    approval_status: str = STATUS_DISCOVERED

    @property
    def usable(self) -> bool:
        return self.approval_status == STATUS_APPROVED


@dataclass(frozen=True)
class CallRequest:
    call_id: str
    capability_id: str
    arguments: Mapping[str, Any] = field(default_factory=dict)
    deadline_seconds: float = 30.0


@dataclass(frozen=True)
class CallResult:
    call_id: str
    outcome: str
    evidence: Mapping[str, Any] = field(default_factory=dict)
    detail: str = ""


def declare(
    *,
    capability_id: str,
    resource_id: str,
    executor_kind: str,
    source: str,
    arguments: Sequence[DeclaredArgument] = (),
    required_observations: Sequence[str] = (),
    runtime_name: str = "",
    version: str = "1.0.0",
) -> CapabilityDeclaration:
    metadata = _CAPABILITY_METADATA.get(capability_id)
    if metadata is None:
        raise DeclarationRefused(f"unsupported standard capability: {capability_id}")
    normalized_arguments = tuple(arguments)
    payload = {
        "capability_id": capability_id,
        "resource_id": resource_id,
        "executor_kind": executor_kind,
        "source": source,
        "arguments": [item.to_dict() for item in normalized_arguments],
        "required_observations": list(required_observations),
        "runtime_name": runtime_name,
        "version": version,
        "safety_class": metadata["safety_class"],
        "required_permissions": list(metadata["required_permissions"]),
        "requires_safe_stop": metadata["requires_safe_stop"],
        "cancellable": metadata["cancellable"],
        "revision": metadata["revision"],
    }
    schema_hash = hashlib.sha256(
        json.dumps(
            payload,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=True,
            allow_nan=False,
        ).encode("ascii")
    ).hexdigest()
    return CapabilityDeclaration(
        capability_id=capability_id,
        resource_id=resource_id,
        executor_kind=executor_kind,
        source=source,
        arguments=normalized_arguments,
        required_observations=tuple(required_observations),
        runtime_name=runtime_name,
        version=version,
        safety_class=str(metadata["safety_class"]),
        required_permissions=tuple(metadata["required_permissions"]),
        requires_safe_stop=bool(metadata["requires_safe_stop"]),
        cancellable=bool(metadata["cancellable"]),
        revision=int(metadata["revision"]),
        schema_hash=schema_hash,
    )


def approve(
    declaration: CapabilityDeclaration,
    *,
    actor: str = "",
) -> CapabilityDeclaration:
    _ = actor
    return replace(declaration, approval_status=STATUS_APPROVED)


def arguments_to_json_schema(arguments: Sequence[DeclaredArgument]) -> dict[str, Any]:
    properties: dict[str, Any] = {}
    required: list[str] = []
    for item in arguments:
        schema: dict[str, Any] = {"type": "number" if item.type == "number" else item.type}
        if item.minimum is not None:
            schema["minimum"] = item.minimum
        if item.maximum is not None:
            schema["maximum"] = item.maximum
        if item.description:
            schema["description"] = item.description
        if item.unit:
            schema["x-unit"] = item.unit
        properties[item.name] = schema
        if item.required:
            required.append(item.name)
    result: dict[str, Any] = {
        "type": "object",
        "properties": properties,
        "additionalProperties": False,
    }
    if required:
        result["required"] = required
    return result


def capability_metadata(capability_id: str) -> Mapping[str, Any]:
    return dict(_CAPABILITY_METADATA[capability_id])
