"""Strict Mission Stations adapter in front of the deterministic robot runtime."""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import math
import os
import re
import threading
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from .capabilities import CapabilityRegistry, CapabilityValidationError
from .safety_profile import (
    CONSTRAINABLE,
    DEFAULT_AUDIT_PATH,
    DEFAULT_PROFILE_PATH,
    PROFILE_CONTRACT_VERSION,
    SafetyProfileError,
    audit_tail,
    load_profile,
    update_profile,
)

MISSION_DISPATCH_CONTRACT_VERSION = "flyto.robotics.mission-dispatch.v1"
CALIBRATION_CONTRACT_VERSION = "flyto.space-calibration.v1"
MISSION_GRANT_CONTRACT_VERSION = "flyto.robotics.mission-grant.v1"
MISSION_ACTION_EVIDENCE_CONTRACT_VERSION = (
    "flyto.robotics.mission-action-evidence.v1"
)

SAFE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")
SHA256 = re.compile(r"^[a-f0-9]{64}$")
EVIDENCE_KINDS = frozenset(
    {
        "action.execution",
        "zone.overview",
        "passage.clearance",
        "device.identifier",
        "arrival.pose",
        "handover.confirmation",
        "human.approval",
    }
)
CAPABILITY_EVIDENCE = {
    "navigate": frozenset({"arrival.pose"}),
    "navigate_to_location": frozenset({"arrival.pose"}),
    "wait_until_clear": frozenset({"passage.clearance"}),
    "ask_human": frozenset({"human.approval"}),
}
RAW_ACTUATOR_FIELDS = frozenset(
    {"cmd_vel", "velocity", "linear_velocity", "angular_velocity", "pwm", "topic"}
)


class MissionGatewayError(ValueError):
    """Raised when an untrusted dispatch violates the execution contract."""


@dataclass(frozen=True)
class CalibrationMarker:
    marker_id: str
    x: float
    y: float
    yaw: float
    confidence: float
    source: str
    observed_at: str


@dataclass(frozen=True)
class MissionCalibration:
    calibration_id: str
    revision: int
    coordinate_frame: str
    markers: tuple[CalibrationMarker, ...]
    contract_hash: str


@dataclass(frozen=True)
class MissionDispatchStep:
    step_id: str
    capability_id: str
    runtime_name: str
    arguments: tuple[tuple[str, object], ...]
    evidence_outputs: tuple[str, ...]


@dataclass(frozen=True)
class MissionExecutionGrant:
    task_id: str
    space_id: str
    objective_id: str
    zone_id: str
    resource_id: str
    calibration: MissionCalibration
    plan_revision: int
    plan_hash: str
    assignment_revision: int
    registry_hash: str
    evidence_requirements: tuple[str, ...]
    steps: tuple[MissionDispatchStep, ...]
    dispatch_hash: str

    def to_dict(self) -> dict[str, object]:
        return {
            "contract_version": MISSION_GRANT_CONTRACT_VERSION,
            "task_id": self.task_id,
            "space_id": self.space_id,
            "objective_id": self.objective_id,
            "zone_id": self.zone_id,
            "resource_id": self.resource_id,
            "calibration_id": self.calibration.calibration_id,
            "calibration_revision": self.calibration.revision,
            "calibration_hash": self.calibration.contract_hash,
            "plan_revision": self.plan_revision,
            "plan_hash": self.plan_hash,
            "assignment_revision": self.assignment_revision,
            "capability_registry_hash": self.registry_hash,
            "evidence_requirements": list(self.evidence_requirements),
            "steps": [
                {
                    "step_id": step.step_id,
                    "capability_id": step.capability_id,
                    "runtime_name": step.runtime_name,
                    "arguments": dict(step.arguments),
                    "evidence_outputs": list(step.evidence_outputs),
                }
                for step in self.steps
            ],
            "dispatch_hash": self.dispatch_hash,
        }


def parse_mission_dispatch(
    raw: object,
    registry: CapabilityRegistry,
    *,
    required_marker_ids: frozenset[str] = frozenset(
        {"Z1", "Z2", "Z3", "Z4", "START"}
    ),
) -> MissionExecutionGrant:
    """Validate one scheduler dispatch before any controller or adapter starts."""
    value = _mapping(raw, "mission dispatch")
    _exact_fields(
        value,
        {
            "contract_version",
            "task_id",
            "space_id",
            "objective_id",
            "zone_id",
            "zone_marker_id",
            "card_source",
            "resource_id",
            "evidence_requirements",
            "calibration",
            "plan_revision",
            "plan_hash",
            "assignment_revision",
            "capability_registry_hash",
            "steps",
        },
        "mission dispatch",
    )
    if value["contract_version"] != MISSION_DISPATCH_CONTRACT_VERSION:
        raise MissionGatewayError("mission dispatch contract_version is unsupported")
    if value["card_source"] != "judge_draw":
        raise MissionGatewayError("mission cards must come from a physical judge draw")

    identifiers = {
        name: _identifier(value[name], f"mission dispatch.{name}")
        for name in (
            "task_id",
            "space_id",
            "objective_id",
            "zone_id",
            "zone_marker_id",
            "resource_id",
        )
    }
    if identifiers["zone_marker_id"] not in required_marker_ids - {"START"}:
        raise MissionGatewayError("zone_marker_id is not a configured Mission Station")

    requirements = _evidence_list(
        value["evidence_requirements"],
        "mission dispatch.evidence_requirements",
        minimum=1,
    )
    calibration = _parse_calibration(value["calibration"], required_marker_ids)
    plan_revision = _positive_int(value["plan_revision"], "plan_revision")
    assignment_revision = _positive_int(
        value["assignment_revision"], "assignment_revision"
    )
    plan_hash = _hash(value["plan_hash"], "plan_hash")

    catalog = registry.execution_catalog()
    registry_hash = _hash(
        value["capability_registry_hash"], "capability_registry_hash"
    )
    if registry_hash != catalog["contract_hash"]:
        raise MissionGatewayError("capability registry snapshot is stale or unknown")
    by_id = {
        item["capability_id"]: item
        for item in catalog["capabilities"]
        if item["approval_status"] == "APPROVED"
    }

    raw_steps = value["steps"]
    if not isinstance(raw_steps, list) or not 1 <= len(raw_steps) <= 100:
        raise MissionGatewayError("mission dispatch.steps must contain 1 to 100 items")
    steps = tuple(_parse_step(item, registry, by_id, requirements) for item in raw_steps)
    step_ids = [step.step_id for step in steps]
    if len(step_ids) != len(set(step_ids)):
        raise MissionGatewayError("mission dispatch step IDs must be unique")

    moving = any(bool(by_id[step.capability_id]["requires_safe_stop"]) for step in steps)
    if moving and steps[-1].runtime_name != "safe_stop":
        raise MissionGatewayError("movement dispatch must end with safe_stop")

    canonical = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return MissionExecutionGrant(
        task_id=identifiers["task_id"],
        space_id=identifiers["space_id"],
        objective_id=identifiers["objective_id"],
        zone_id=identifiers["zone_id"],
        resource_id=identifiers["resource_id"],
        calibration=calibration,
        plan_revision=plan_revision,
        plan_hash=plan_hash,
        assignment_revision=assignment_revision,
        registry_hash=registry_hash,
        evidence_requirements=requirements,
        steps=steps,
        dispatch_hash=hashlib.sha256(canonical.encode("utf-8")).hexdigest(),
    )


def build_action_evidence(
    grant: MissionExecutionGrant,
    *,
    step_id: str,
    status: str,
    outcome: str,
    observed_at: str,
) -> dict[str, object]:
    """Emit an action receipt that is explicitly ineligible to complete a task."""
    step = next((item for item in grant.steps if item.step_id == step_id), None)
    if step is None:
        raise MissionGatewayError("action evidence step is not in the granted dispatch")
    if status not in {"VALID", "INSUFFICIENT", "INVALID"}:
        raise MissionGatewayError("action evidence status is unsupported")
    if not isinstance(outcome, str) or not 1 <= len(outcome) <= 128:
        raise MissionGatewayError("action evidence outcome must be bounded text")
    evidence_key = hashlib.sha256(
        f"{grant.task_id}:{step_id}:{grant.dispatch_hash}".encode()
    ).hexdigest()[:24]
    evidence: dict[str, object] = {
        "evidence_id": f"action-{evidence_key}",
        "kind": "action.execution",
        "status": status,
        "outcome": outcome,
        "source_resource_id": grant.resource_id,
        "value": {
            "capability_id": step.capability_id,
            "plan_revision": grant.plan_revision,
            "assignment_revision": grant.assignment_revision,
            "dispatch_hash": grant.dispatch_hash,
            "task_completion_eligible": False,
        },
        "observed_at": observed_at,
    }
    evidence["digest"] = _content_hash(evidence)
    payload: dict[str, object] = {
        "contract_version": MISSION_ACTION_EVIDENCE_CONTRACT_VERSION,
        "task_id": grant.task_id,
        "step_id": step_id,
        "evidence": evidence,
    }
    payload["contract_hash"] = _content_hash(payload)
    return payload


class MissionGateway:
    """Local HTTP discovery and validation adapter with no raw motor endpoint."""

    def __init__(
        self,
        registry: CapabilityRegistry,
        token: str,
        *,
        host: str = "127.0.0.1",
        port: int = 0,
    ) -> None:
        if not token:
            raise MissionGatewayError("mission gateway token is required")
        if not ipaddress.ip_address(host).is_loopback:
            raise MissionGatewayError("mission gateway must bind to a loopback address")
        self.registry = registry
        self.token = token
        self.host = host
        self.port = port
        self._server: ThreadingHTTPServer | None = None
        self._thread: threading.Thread | None = None
        # Outside the deployed tree: a site limit inside it would be replaced
        # by the next git pull, which is the opposite of what it is for.
        self.safety_profile_path = Path(
            os.getenv("FLYTO_SAFETY_PROFILE", DEFAULT_PROFILE_PATH)
        )
        self.safety_audit_path = Path(
            os.getenv("FLYTO_SAFETY_PROFILE_AUDIT", DEFAULT_AUDIT_PATH)
        )

    @property
    def address(self) -> tuple[str, int]:
        if self._server is None:
            raise MissionGatewayError("mission gateway is not running")
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        if self._server is not None:
            return
        gateway = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, _format: str, *_args: object) -> None:
                return

            def _authorized(self) -> bool:
                header = self.headers.get("Authorization", "")
                return hmac.compare_digest(header, f"Bearer {gateway.token}")

            def _send(self, status: int, payload: Mapping[str, object]) -> None:
                body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802 - http.server contract
                if not self._authorized():
                    self._send(401, {"error": "unauthorized"})
                elif self.path == "/v1/capabilities":
                    self._send(200, gateway.registry.execution_catalog())
                elif self.path == "/v1/safety-profile":
                    self._send(200, gateway.safety_profile_view())
                else:
                    self._send(404, {"error": "not_found"})

            def do_POST(self) -> None:  # noqa: N802 - http.server contract
                if not self._authorized():
                    self._send(401, {"error": "unauthorized"})
                    return
                if self.path not in ("/v1/missions/validate", "/v1/safety-profile"):
                    self._send(404, {"error": "not_found"})
                    return
                try:
                    length = int(self.headers.get("Content-Length", "0"))
                except ValueError:
                    length = 0
                if not 1 <= length <= 262_144:
                    self._send(400, {"error": "invalid_body_length"})
                    return
                if self.path == "/v1/safety-profile":
                    try:
                        payload = json.loads(self.rfile.read(length))
                        record = gateway.update_safety_profile(payload)
                    except (json.JSONDecodeError, SafetyProfileError) as exc:
                        self._send(
                            400,
                            {"error": "safety_profile_invalid", "detail": str(exc)[:200]},
                        )
                        return
                    self._send(200, {"ok": True, "change": record})
                    return
                try:
                    payload = json.loads(self.rfile.read(length))
                    grant = parse_mission_dispatch(payload, gateway.registry)
                except (json.JSONDecodeError, MissionGatewayError) as exc:
                    self._send(
                        400,
                        {"error": "mission_dispatch_invalid", "detail": str(exc)[:200]},
                    )
                    return
                self._send(200, {"ok": True, "grant": grant.to_dict()})

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    # -- the installation's own safety limits ----------------------------

    def safety_profile_view(self) -> dict[str, object]:
        """The limits in force here, and how they came to be that way.

        Returned together on purpose: a limit without its history invites
        "who set this to 0.2?" and no way to answer.
        """
        return {
            "contract_version": PROFILE_CONTRACT_VERSION,
            "limits": load_profile(self.safety_profile_path),
            "constrainable": dict(CONSTRAINABLE),
            "recent_changes": audit_tail(self.safety_audit_path),
        }

    def update_safety_profile(self, payload: object) -> dict[str, object]:
        """Set the site limits, recording who and why.

        Takes effect at the next job load. A mission already running resolved
        its limits when it started, so this cannot reach into one in flight —
        which is why nothing here needs to take a lock.
        """
        if not isinstance(payload, Mapping):
            raise SafetyProfileError("body must be a JSON object")
        unknown = sorted(set(payload) - {"limits", "changed_by", "reason"})
        if unknown:
            raise SafetyProfileError(f"unexpected fields: {', '.join(unknown)}")
        limits = payload.get("limits")
        if not isinstance(limits, Mapping):
            raise SafetyProfileError("limits must be an object")
        return update_profile(
            self.safety_profile_path,
            self.safety_audit_path,
            limits=limits,
            changed_by=str(payload.get("changed_by", "")),
            reason=str(payload.get("reason", "")),
            at=datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
        )

    def stop(self) -> None:
        if self._server is None:
            return
        self._server.shutdown()
        self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None

    def __enter__(self) -> MissionGateway:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.stop()


def _parse_step(
    raw: object,
    registry: CapabilityRegistry,
    approved: Mapping[str, Mapping[str, object]],
    requirements: tuple[str, ...],
) -> MissionDispatchStep:
    value = _mapping(raw, "mission dispatch step")
    if RAW_ACTUATOR_FIELDS.intersection(value):
        raise MissionGatewayError("raw actuator fields are forbidden")
    _exact_fields(
        value,
        {"step_id", "capability_id", "executor_kind", "arguments", "evidence_outputs"},
        "mission dispatch step",
    )
    if value["executor_kind"] != "flyto-robotics":
        raise MissionGatewayError("mission dispatch step executor_kind is not Robotics")
    step_id = _identifier(value["step_id"], "mission dispatch step.step_id")
    capability_id = _identifier(
        value["capability_id"], "mission dispatch step.capability_id"
    )
    manifest = approved.get(capability_id)
    if manifest is None:
        raise MissionGatewayError("mission dispatch capability is not approved")
    runtime_name = str(manifest["runtime_name"])
    try:
        arguments = registry.validate_call(runtime_name, value["arguments"])
    except CapabilityValidationError as exc:
        raise MissionGatewayError(str(exc)) from exc
    evidence_outputs = _evidence_list(
        value["evidence_outputs"],
        "mission dispatch step.evidence_outputs",
        minimum=0,
    )
    allowed_outputs = CAPABILITY_EVIDENCE.get(runtime_name, frozenset())
    if not set(evidence_outputs).issubset(allowed_outputs):
        raise MissionGatewayError("capability declares an unsupported evidence output")
    if not set(evidence_outputs).issubset(requirements):
        raise MissionGatewayError("step evidence output is outside the judge-card contract")
    return MissionDispatchStep(
        step_id=step_id,
        capability_id=capability_id,
        runtime_name=runtime_name,
        arguments=tuple(arguments.items()),
        evidence_outputs=evidence_outputs,
    )


def _parse_calibration(
    raw: object,
    required_marker_ids: frozenset[str],
) -> MissionCalibration:
    value = _mapping(raw, "calibration")
    _exact_fields(
        value,
        {
            "contract_version",
            "calibration_id",
            "revision",
            "coordinate_frame",
            "status",
            "markers",
            "created_at",
            "created_by",
            "contract_hash",
        },
        "calibration",
    )
    if value["contract_version"] != CALIBRATION_CONTRACT_VERSION:
        raise MissionGatewayError("calibration contract_version is unsupported")
    if value["status"] != "READY":
        raise MissionGatewayError("calibration is not READY")
    calibration_id = _identifier(value["calibration_id"], "calibration_id")
    coordinate_frame = _identifier(value["coordinate_frame"], "coordinate_frame")
    revision = _positive_int(value["revision"], "calibration revision")
    raw_markers = value["markers"]
    if not isinstance(raw_markers, list) or not 1 <= len(raw_markers) <= 65:
        raise MissionGatewayError("calibration.markers must contain 1 to 65 items")
    markers = tuple(_parse_marker(item) for item in raw_markers)
    marker_ids = [item.marker_id for item in markers]
    if len(marker_ids) != len(set(marker_ids)):
        raise MissionGatewayError("calibration marker IDs must be unique")
    missing = sorted(required_marker_ids - set(marker_ids))
    if missing:
        raise MissionGatewayError("calibration is missing markers: " + ", ".join(missing))
    supplied_hash = _hash(value["contract_hash"], "calibration contract_hash")
    hash_payload = {
        key: item
        for key, item in value.items()
        if key not in {"contract_version", "contract_hash"}
    }
    if supplied_hash != _content_hash(hash_payload):
        raise MissionGatewayError("calibration contract_hash does not match markers")
    return MissionCalibration(
        calibration_id=calibration_id,
        revision=revision,
        coordinate_frame=coordinate_frame,
        markers=markers,
        contract_hash=supplied_hash,
    )


def _parse_marker(raw: object) -> CalibrationMarker:
    value = _mapping(raw, "calibration marker")
    _exact_fields(
        value,
        {"marker_id", "x", "y", "yaw", "confidence", "source", "observed_at"},
        "calibration marker",
    )
    source = value["source"]
    if source not in {"apriltag", "overhead_camera", "manual"}:
        raise MissionGatewayError("calibration marker source is unsupported")
    return CalibrationMarker(
        marker_id=_identifier(value["marker_id"], "calibration marker.marker_id"),
        x=_finite(value["x"], "calibration marker.x", -100_000.0, 100_000.0),
        y=_finite(value["y"], "calibration marker.y", -100_000.0, 100_000.0),
        yaw=_finite(value["yaw"], "calibration marker.yaw", -6.284, 6.284),
        confidence=_finite(value["confidence"], "calibration marker.confidence", 0.0, 1.0),
        source=str(source),
        observed_at=_bounded_text(value["observed_at"], "observed_at", 64),
    )


def _mapping(value: object, field: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise MissionGatewayError(f"{field} must be an object")
    return value


def _exact_fields(value: Mapping[str, object], expected: set[str], field: str) -> None:
    missing = sorted(expected - set(value))
    unknown = sorted(set(value) - expected)
    if missing or unknown:
        details = []
        if missing:
            details.append("missing " + ", ".join(missing))
        if unknown:
            details.append("unsupported " + ", ".join(unknown))
        raise MissionGatewayError(f"{field} fields are invalid: {'; '.join(details)}")


def _identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or not SAFE_IDENTIFIER.fullmatch(value):
        raise MissionGatewayError(f"{field} must be a safe identifier")
    return value


def _hash(value: object, field: str) -> str:
    if not isinstance(value, str) or not SHA256.fullmatch(value):
        raise MissionGatewayError(f"{field} must be a lowercase SHA-256 digest")
    return value


def _positive_int(value: object, field: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 1 <= value <= 1_000_000:
        raise MissionGatewayError(f"{field} must be a positive bounded integer")
    return value


def _finite(value: object, field: str, minimum: float, maximum: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionGatewayError(f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number) or not minimum <= number <= maximum:
        raise MissionGatewayError(f"{field} is outside its bounded range")
    return number


def _bounded_text(value: object, field: str, maximum: int) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= maximum:
        raise MissionGatewayError(f"{field} must be bounded text")
    return value


def _evidence_list(value: object, field: str, *, minimum: int) -> tuple[str, ...]:
    if not isinstance(value, list) or not minimum <= len(value) <= 16:
        raise MissionGatewayError(f"{field} must contain {minimum} to 16 items")
    parsed = tuple(str(item) for item in value)
    if len(parsed) != len(set(parsed)):
        raise MissionGatewayError(f"{field} must contain unique values")
    if any(item not in EVIDENCE_KINDS for item in parsed):
        raise MissionGatewayError(f"{field} contains an unsupported evidence kind")
    return parsed


def _content_hash(value: Mapping[str, object]) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    )
    return hashlib.sha256(payload.encode("utf-8")).hexdigest()
