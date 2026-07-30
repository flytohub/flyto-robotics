"""Deterministic facility-resource selection and handoff evidence."""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass

from .dependency_policy import (
    DependencyAssessment,
    DependencyContract,
    assess_device_dependency,
)

FACILITY_RESOURCE_CONTRACT_VERSION = "flyto.robotics.facility-resources.v1"
FACILITY_EVIDENCE_CONTRACT_VERSION = "flyto.robotics.facility-evidence.v1"
MAX_RESOURCES = 64
MAX_EVENTS = 512
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@*-]{0,191}$")


class FacilityResourceError(ValueError):
    """Raised when resource configuration or a handoff request is invalid."""


def _identifier(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise FacilityResourceError(f"{field_name} must be a safe identifier")
    return value


def _fallback_zone(value: object, field_name: str) -> str:
    if value == "*":
        return "*"
    return _identifier(value, field_name)


@dataclass(frozen=True)
class FacilityResource:
    """One exact device endpoint available inside an AI Space."""

    resource_id: str
    device_kind: str
    zone_id: str
    adapter_id: str
    endpoint_id: str
    priority: int
    fallback_zones: tuple[str, ...]
    default_dependency: DependencyContract


@dataclass(frozen=True)
class ResourceSelection:
    """A deterministic selection result suitable for audit and replay."""

    resource: FacilityResource
    candidates: tuple[str, ...]
    reason: str
    dependency: DependencyContract


@dataclass(frozen=True)
class FacilityResourceCatalog:
    """Validated resource catalog; policy stays data-driven rather than hard-coded."""

    space_id: str
    resources: tuple[FacilityResource, ...]

    @classmethod
    def from_mapping(cls, value: object) -> FacilityResourceCatalog:
        if not isinstance(value, Mapping):
            raise FacilityResourceError("facility resource document must be an object")
        allowed = {"contract_version", "space_id", "resources"}
        unknown = sorted(set(value) - allowed)
        if unknown:
            raise FacilityResourceError(
                "facility resource document contains unsupported fields: "
                + ", ".join(unknown)
            )
        if value.get("contract_version") != FACILITY_RESOURCE_CONTRACT_VERSION:
            raise FacilityResourceError(
                f"contract_version must be {FACILITY_RESOURCE_CONTRACT_VERSION}"
            )
        raw_resources = value.get("resources")
        if not isinstance(raw_resources, list) or not 1 <= len(raw_resources) <= MAX_RESOURCES:
            raise FacilityResourceError(
                f"resources must contain 1 to {MAX_RESOURCES} items"
            )
        resources: list[FacilityResource] = []
        for index, raw in enumerate(raw_resources):
            field_name = f"resources[{index}]"
            if not isinstance(raw, Mapping):
                raise FacilityResourceError(f"{field_name} must be an object")
            required_fields = {
                "resource_id",
                "device_kind",
                "zone_id",
                "adapter_id",
                "endpoint_id",
                "priority",
                "fallback_zones",
            }
            allowed_fields = required_fields | {"default_dependency"}
            if not required_fields.issubset(raw) or not set(raw).issubset(
                allowed_fields
            ):
                raise FacilityResourceError(
                    f"{field_name} requires {', '.join(sorted(required_fields))}; "
                    "default_dependency is optional"
                )
            priority = raw["priority"]
            if isinstance(priority, bool) or not isinstance(priority, int):
                raise FacilityResourceError(f"{field_name}.priority must be an integer")
            if not 0 <= priority <= 10_000:
                raise FacilityResourceError(
                    f"{field_name}.priority must be between 0 and 10000"
                )
            fallback_zones = raw["fallback_zones"]
            if not isinstance(fallback_zones, list) or len(fallback_zones) > 32:
                raise FacilityResourceError(
                    f"{field_name}.fallback_zones must contain at most 32 items"
                )
            resources.append(
                FacilityResource(
                    resource_id=_identifier(
                        raw["resource_id"], f"{field_name}.resource_id"
                    ),
                    device_kind=_identifier(
                        raw["device_kind"], f"{field_name}.device_kind"
                    ),
                    zone_id=_identifier(raw["zone_id"], f"{field_name}.zone_id"),
                    adapter_id=_identifier(
                        raw["adapter_id"], f"{field_name}.adapter_id"
                    ),
                    endpoint_id=_identifier(
                        raw["endpoint_id"], f"{field_name}.endpoint_id"
                    ),
                    priority=priority,
                    fallback_zones=tuple(
                        _fallback_zone(item, f"{field_name}.fallback_zones")
                        for item in fallback_zones
                    ),
                    default_dependency=DependencyContract.from_mapping(
                        raw.get("default_dependency"),
                    ),
                )
            )
        resource_ids = [item.resource_id for item in resources]
        if len(resource_ids) != len(set(resource_ids)):
            raise FacilityResourceError("resource_id values must be unique")
        return cls(
            space_id=_identifier(value.get("space_id"), "space_id"),
            resources=tuple(resources),
        )

    def select(
        self,
        *,
        device_kind: str,
        zone_id: str,
        healthy: Mapping[str, bool],
    ) -> ResourceSelection:
        """Choose the most local healthy resource, then a declared fallback."""
        requested_kind = _identifier(device_kind, "device_kind")
        requested_zone = _identifier(zone_id, "zone_id")
        candidates = tuple(
            resource
            for resource in self.resources
            if resource.device_kind == requested_kind
            and healthy.get(resource.resource_id, True)
            and (
                resource.zone_id == requested_zone
                or requested_zone in resource.fallback_zones
                or "*" in resource.fallback_zones
            )
        )
        ordered = tuple(
            sorted(
                candidates,
                key=lambda resource: (
                    resource.zone_id != requested_zone,
                    resource.priority,
                    resource.resource_id,
                ),
            )
        )
        if not ordered:
            raise FacilityResourceError(
                f"no healthy {requested_kind} resource covers {requested_zone}"
            )
        selected = ordered[0]
        reason = (
            "exact_zone"
            if selected.zone_id == requested_zone
            else "declared_fallback"
        )
        return ResourceSelection(
            resource=selected,
            candidates=tuple(item.resource_id for item in ordered),
            reason=reason,
            dependency=selected.default_dependency,
        )


class FacilityResourceRuntime:
    """Small event-sourced runtime for resource health, leases, and handoff."""

    def __init__(self, catalog: FacilityResourceCatalog) -> None:
        self.catalog = catalog
        self.healthy = {resource.resource_id: True for resource in catalog.resources}
        self.active_by_kind: dict[str, str] = {}
        self.active_zone_by_kind: dict[str, str] = {}
        self.active_dependency_by_kind: dict[str, DependencyContract] = {}
        self.seen_frames: set[str] = set()
        self.events: list[dict[str, object]] = []

    def _record(self, kind: str, *, at_seconds: float, **fields: object) -> None:
        if not 0 <= at_seconds <= 86_400:
            raise FacilityResourceError("at_seconds is outside the bounded run window")
        if len(self.events) >= MAX_EVENTS:
            raise FacilityResourceError("facility evidence event limit reached")
        self.events.append(
            {
                "sequence": len(self.events) + 1,
                "at_seconds": round(float(at_seconds), 3),
                "kind": _identifier(kind, "kind"),
                **fields,
            }
        )

    def observe_frame(self, resource_id: str, *, at_seconds: float) -> None:
        """Record first proof that a configured camera stream is live."""
        resource = self._resource(resource_id)
        if resource.resource_id in self.seen_frames:
            return
        self.seen_frames.add(resource.resource_id)
        self._record(
            "resource.stream_ready",
            at_seconds=at_seconds,
            resource_id=resource.resource_id,
            device_kind=resource.device_kind,
            zone_id=resource.zone_id,
        )

    def set_health(
        self,
        resource_id: str,
        *,
        healthy: bool,
        at_seconds: float,
        reason: str,
    ) -> None:
        """Update trusted health and release an unhealthy active resource."""
        resource = self._resource(resource_id)
        if not isinstance(healthy, bool):
            raise FacilityResourceError("healthy must be a boolean")
        previous = self.healthy[resource.resource_id]
        self.healthy[resource.resource_id] = healthy
        if previous != healthy:
            self._record(
                "resource.health_changed",
                at_seconds=at_seconds,
                resource_id=resource.resource_id,
                healthy=healthy,
                reason=_identifier(reason, "reason"),
            )
        if not healthy and self.active_by_kind.get(resource.device_kind) == resource.resource_id:
            active_zone = self.active_zone_by_kind.get(
                resource.device_kind, resource.zone_id
            )
            fallback_available = self._fallback_available(
                resource=resource,
                zone_id=active_zone,
            )
            requirement = self.active_dependency_by_kind.get(
                resource.device_kind,
                resource.default_dependency,
            )
            self._record_dependency_assessment(
                resource=resource,
                requirement=requirement,
                healthy=False,
                confidence=0.0,
                observation_age_seconds=0.0,
                fallback_available=fallback_available,
                at_seconds=at_seconds,
            )
            self.active_by_kind.pop(resource.device_kind, None)
            self.active_zone_by_kind.pop(resource.device_kind, None)
            self.active_dependency_by_kind.pop(resource.device_kind, None)
            self._record(
                "resource.lease_released",
                at_seconds=at_seconds,
                resource_id=resource.resource_id,
                device_kind=resource.device_kind,
                reason="active_resource_unhealthy",
            )

    def handoff(
        self,
        *,
        device_kind: str,
        zone_id: str,
        at_seconds: float,
        reason: str,
        dependency: object = None,
    ) -> ResourceSelection:
        """Release the old lease and bind the best healthy resource for a zone."""
        selection = self.catalog.select(
            device_kind=device_kind,
            zone_id=zone_id,
            healthy=self.healthy,
        )
        requirement = DependencyContract.from_mapping(
            dependency,
            default=selection.resource.default_dependency,
        )
        selection = ResourceSelection(
            resource=selection.resource,
            candidates=selection.candidates,
            reason=selection.reason,
            dependency=requirement,
        )
        active = self.active_by_kind.get(device_kind)
        if active == selection.resource.resource_id:
            self.active_zone_by_kind[device_kind] = zone_id
            self.active_dependency_by_kind[device_kind] = requirement
            self._record(
                "resource.lease_retained",
                at_seconds=at_seconds,
                resource_id=active,
                device_kind=device_kind,
                zone_id=zone_id,
                reason=_identifier(reason, "reason"),
                dependency_band=requirement.derived_band(),
            )
            self._record_dependency_assessment(
                resource=selection.resource,
                requirement=requirement,
                healthy=True,
                confidence=1.0,
                observation_age_seconds=0.0,
                fallback_available=True,
                at_seconds=at_seconds,
            )
            return selection
        if active is not None:
            self._record(
                "resource.lease_released",
                at_seconds=at_seconds,
                resource_id=active,
                device_kind=device_kind,
                reason="zone_handoff",
            )
        self.active_by_kind[device_kind] = selection.resource.resource_id
        self.active_zone_by_kind[device_kind] = zone_id
        self.active_dependency_by_kind[device_kind] = requirement
        self._record(
            "resource.router_selected",
            at_seconds=at_seconds,
            resource_id=selection.resource.resource_id,
            device_kind=device_kind,
            zone_id=zone_id,
            adapter_id=selection.resource.adapter_id,
            endpoint_id=selection.resource.endpoint_id,
            candidates=list(selection.candidates),
            selection_reason=selection.reason,
            trigger_reason=_identifier(reason, "reason"),
            dependency_band=requirement.derived_band(),
        )
        self._record(
            "resource.lease_acquired",
            at_seconds=at_seconds,
            resource_id=selection.resource.resource_id,
            device_kind=device_kind,
            zone_id=zone_id,
            dependency_band=requirement.derived_band(),
        )
        self._record_dependency_assessment(
            resource=selection.resource,
            requirement=requirement,
            healthy=True,
            confidence=1.0,
            observation_age_seconds=0.0,
            fallback_available=True,
            at_seconds=at_seconds,
        )
        return selection

    def assess_dependency(
        self,
        resource_id: str,
        *,
        at_seconds: float,
        dependency: object = None,
        confidence: float = 1.0,
        observation_age_seconds: float = 0.0,
        fallback_available: bool = False,
    ) -> DependencyAssessment:
        """Assess a workflow binding without changing its active lease."""
        resource = self._resource(resource_id)
        requirement = DependencyContract.from_mapping(
            dependency,
            default=resource.default_dependency,
        )
        return self._record_dependency_assessment(
            resource=resource,
            requirement=requirement,
            healthy=self.healthy[resource.resource_id],
            confidence=confidence,
            observation_age_seconds=observation_age_seconds,
            fallback_available=fallback_available,
            at_seconds=at_seconds,
        )

    def evidence(self) -> dict[str, object]:
        """Return a bounded replay document without embedding video frames."""
        return {
            "contract_version": FACILITY_EVIDENCE_CONTRACT_VERSION,
            "space_id": self.catalog.space_id,
            "active_resources": dict(sorted(self.active_by_kind.items())),
            "active_dependencies": {
                device_kind: requirement.to_mapping()
                for device_kind, requirement in sorted(
                    self.active_dependency_by_kind.items()
                )
            },
            "health": dict(sorted(self.healthy.items())),
            "seen_streams": sorted(self.seen_frames),
            "events": list(self.events),
        }

    def _resource(self, resource_id: str) -> FacilityResource:
        requested = _identifier(resource_id, "resource_id")
        for resource in self.catalog.resources:
            if resource.resource_id == requested:
                return resource
        raise FacilityResourceError(f"unknown resource_id: {requested}")

    def _fallback_available(
        self,
        *,
        resource: FacilityResource,
        zone_id: str,
    ) -> bool:
        try:
            selection = self.catalog.select(
                device_kind=resource.device_kind,
                zone_id=zone_id,
                healthy=self.healthy,
            )
        except FacilityResourceError:
            return False
        return selection.resource.resource_id != resource.resource_id

    def _record_dependency_assessment(
        self,
        *,
        resource: FacilityResource,
        requirement: DependencyContract,
        healthy: bool,
        confidence: float,
        observation_age_seconds: float,
        fallback_available: bool,
        at_seconds: float,
    ) -> DependencyAssessment:
        assessment = assess_device_dependency(
            requirement,
            healthy=healthy,
            confidence=confidence,
            observation_age_seconds=observation_age_seconds,
            fallback_available=fallback_available,
        )
        self._record(
            "resource.dependency_assessed",
            at_seconds=at_seconds,
            resource_id=resource.resource_id,
            device_kind=resource.device_kind,
            dependency=requirement.to_mapping(),
            derived_band=assessment.derived_band,
            state=assessment.state,
            action=assessment.action,
            must_stop=assessment.must_stop,
            requires_human=assessment.requires_human,
            assessment_reason=assessment.reason,
            fallback_available=fallback_available,
        )
        return assessment
