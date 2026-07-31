"""Data-driven branch routing over generic resource dependency contracts."""

from __future__ import annotations

import hashlib
import json
import math
import re
from collections.abc import Mapping
from dataclasses import dataclass
from pathlib import Path

from .dependency_policy import (
    DependencyContract,
    DependencyPolicyError,
    assess_device_dependency,
)

ROUTE_GRAPH_CONTRACT = "flyto.robotics.route-graph.v1"
ROUTE_EVALUATION_CONTRACT = "flyto.robotics.route-evaluation.v1"
MAX_ROUTE_FILE_BYTES = 256 * 1024
IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,255}$")
BLOCKING_ACTIONS = frozenset(
    {
        "abort_and_invalidate",
        "pause_and_escalate",
        "safe_stop_and_abort",
        "safe_stop_and_escalate",
    }
)
ACTION_PENALTIES = {
    "use_resource": 0,
    "switch_substitute": 8,
    "safe_stop_then_switch_substitute": 12,
    "pause_then_switch_substitute": 12,
    "continue_degraded": 20,
    "continue_without_resource": 30,
    "ignore_outside_scope": 0,
}


class RouteGraphError(ValueError):
    """Raised when a route graph or runtime observation is invalid."""


def _identifier(value: object, field: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise RouteGraphError(f"{field} must be a safe identifier")
    return value


def _number(
    value: object,
    field: str,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RouteGraphError(f"{field} must be numeric")
    parsed = float(value)
    if not math.isfinite(parsed) or not minimum <= parsed <= maximum:
        raise RouteGraphError(
            f"{field} must be between {minimum} and {maximum}"
        )
    return parsed


def _canonical(value: object) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def _snapshot(value: object) -> str:
    return hashlib.sha256(_canonical(value)).hexdigest()


@dataclass(frozen=True)
class RouteDependency:
    """One resource dependency attached to one complete route option."""

    resource_id: str
    contract: DependencyContract


@dataclass(frozen=True)
class RouteOption:
    """One complete semantic-location path through the branching graph."""

    route_id: str
    location_ids: tuple[str, ...]
    base_score: float
    reason_codes: tuple[str, ...]
    attributes: dict[str, object]
    dependencies: tuple[RouteDependency, ...]


@dataclass(frozen=True)
class RouteGraph:
    """Validated route alternatives; selection remains separate runtime state."""

    graph_id: str
    revision: int
    routes: tuple[RouteOption, ...]
    snapshot: str

    @classmethod
    def from_mapping(cls, value: object) -> RouteGraph:
        if not isinstance(value, Mapping):
            raise RouteGraphError("route graph must be an object")
        raw = dict(value)
        allowed = {"contract_version", "graph_id", "revision", "routes"}
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise RouteGraphError(
                "route graph contains unsupported fields: "
                + ", ".join(unknown)
            )
        if raw.get("contract_version") != ROUTE_GRAPH_CONTRACT:
            raise RouteGraphError(
                f"contract_version must be {ROUTE_GRAPH_CONTRACT}"
            )
        graph_id = _identifier(raw.get("graph_id"), "graph_id")
        revision = raw.get("revision")
        if (
            isinstance(revision, bool)
            or not isinstance(revision, int)
            or not 1 <= revision <= 1_000_000
        ):
            raise RouteGraphError("revision must be a positive integer")
        raw_routes = raw.get("routes")
        if (
            not isinstance(raw_routes, list)
            or not 2 <= len(raw_routes) <= 64
        ):
            raise RouteGraphError("routes must contain 2 to 64 alternatives")
        routes = []
        route_ids = set()
        for index, raw_route in enumerate(raw_routes):
            if not isinstance(raw_route, Mapping):
                raise RouteGraphError(f"routes[{index}] must be an object")
            route = dict(raw_route)
            route_allowed = {
                "route_id",
                "location_ids",
                "base_score",
                "reason_codes",
                "attributes",
                "dependencies",
            }
            route_unknown = sorted(set(route) - route_allowed)
            if route_unknown:
                raise RouteGraphError(
                    f"routes[{index}] contains unsupported fields: "
                    + ", ".join(route_unknown)
                )
            route_id = _identifier(
                route.get("route_id"),
                f"routes[{index}].route_id",
            )
            if route_id in route_ids:
                raise RouteGraphError("route_id values must be unique")
            route_ids.add(route_id)
            location_ids = route.get("location_ids")
            if (
                not isinstance(location_ids, list)
                or not 2 <= len(location_ids) <= 64
            ):
                raise RouteGraphError(
                    f"routes[{index}].location_ids must contain 2 to 64 items"
                )
            normalized_locations = tuple(
                _identifier(
                    location,
                    f"routes[{index}].location_ids",
                )
                for location in location_ids
            )
            if len(set(normalized_locations)) != len(normalized_locations):
                raise RouteGraphError(
                    f"routes[{index}].location_ids must not repeat"
                )
            reason_codes = route.get("reason_codes", [])
            if (
                not isinstance(reason_codes, list)
                or len(reason_codes) > 32
            ):
                raise RouteGraphError(
                    f"routes[{index}].reason_codes must be a bounded array"
                )
            attributes = route.get("attributes", {})
            if not isinstance(attributes, Mapping):
                raise RouteGraphError(
                    f"routes[{index}].attributes must be an object"
                )
            dependencies = []
            raw_dependencies = route.get("dependencies", [])
            if (
                not isinstance(raw_dependencies, list)
                or len(raw_dependencies) > 32
            ):
                raise RouteGraphError(
                    f"routes[{index}].dependencies must be a bounded array"
                )
            for dependency_index, raw_dependency in enumerate(
                raw_dependencies
            ):
                if not isinstance(raw_dependency, Mapping):
                    raise RouteGraphError(
                        f"routes[{index}].dependencies[{dependency_index}] "
                        "must be an object"
                    )
                if set(raw_dependency) != {"resource_id", "contract"}:
                    raise RouteGraphError(
                        f"routes[{index}].dependencies[{dependency_index}] "
                        "requires resource_id and contract"
                    )
                try:
                    contract = DependencyContract.from_mapping(
                        raw_dependency["contract"]
                    )
                except DependencyPolicyError as exc:
                    raise RouteGraphError(str(exc)) from exc
                dependencies.append(
                    RouteDependency(
                        resource_id=_identifier(
                            raw_dependency["resource_id"],
                            "route dependency resource_id",
                        ),
                        contract=contract,
                    )
                )
            routes.append(
                RouteOption(
                    route_id=route_id,
                    location_ids=normalized_locations,
                    base_score=_number(
                        route.get("base_score"),
                        f"routes[{index}].base_score",
                        -1_000_000,
                        1_000_000,
                    ),
                    reason_codes=tuple(
                        _identifier(code, "route reason code")
                        for code in reason_codes
                    ),
                    attributes=dict(attributes),
                    dependencies=tuple(dependencies),
                )
            )
        return cls(
            graph_id=graph_id,
            revision=revision,
            routes=tuple(routes),
            snapshot=_snapshot(raw),
        )

    def evaluate(
        self,
        resource_observations: Mapping[str, object],
        *,
        phase: str = "preflight",
        limit: int = 8,
    ) -> dict[str, object]:
        """Hard-filter unusable branches, then rank the bounded candidates."""

        if not 1 <= limit <= 32:
            raise RouteGraphError("limit must be between 1 and 32")
        active_phase = _identifier(phase, "phase")
        candidates = []
        excluded = []
        for route in self.routes:
            score = route.base_score
            dependency_evidence = []
            exclusion_reasons = []
            reason_codes = list(route.reason_codes)
            for dependency in route.dependencies:
                observation = resource_observations.get(
                    dependency.resource_id,
                    {},
                )
                if not isinstance(observation, Mapping):
                    raise RouteGraphError(
                        f"resource observation {dependency.resource_id} "
                        "must be an object"
                    )
                assessment = assess_device_dependency(
                    dependency.contract,
                    healthy=_boolean(observation, "healthy", False),
                    confidence=_observation_number(
                        observation,
                        "confidence",
                        0.0,
                        0.0,
                        1.0,
                    ),
                    observation_age_seconds=_observation_number(
                        observation,
                        "observation_age_seconds",
                        86_400.0,
                        0.0,
                        86_400.0,
                    ),
                    fallback_available=_boolean(
                        observation,
                        "fallback_available",
                        False,
                    ),
                    fallback_equivalent=_boolean(
                        observation,
                        "fallback_equivalent",
                        False,
                    ),
                    fallback_validated=_boolean(
                        observation,
                        "fallback_validated",
                        False,
                    ),
                    evidence_present=_boolean(
                        observation,
                        "evidence_present",
                        False,
                    ),
                    phase=active_phase,
                )
                evidence = {
                    "resource_id": dependency.resource_id,
                    "derived_band": assessment.derived_band,
                    "state": assessment.state,
                    "action": assessment.action,
                    "must_stop": assessment.must_stop,
                    "requires_human": assessment.requires_human,
                    "reason": assessment.reason,
                    "dependency": dependency.contract.to_mapping(),
                }
                dependency_evidence.append(evidence)
                if assessment.action in BLOCKING_ACTIONS:
                    exclusion_reasons.append(
                        f"{dependency.resource_id}:{assessment.reason}"
                    )
                    continue
                penalty = ACTION_PENALTIES.get(assessment.action, 50)
                score -= penalty
                reason_codes.append(
                    f"dependency.{dependency.resource_id}.{assessment.action}"
                )
            item = {
                "route_id": route.route_id,
                "location_ids": list(route.location_ids),
                "score": round(score, 3),
                "reason_codes": list(dict.fromkeys(reason_codes)),
                "attributes": route.attributes,
                "dependencies": dependency_evidence,
            }
            if exclusion_reasons:
                item["exclusion_reasons"] = exclusion_reasons
                excluded.append(item)
            else:
                candidates.append(item)
        candidates.sort(
            key=lambda item: (-float(item["score"]), str(item["route_id"]))
        )
        excluded.sort(key=lambda item: str(item["route_id"]))
        selected_candidates = candidates[:limit]
        if not selected_candidates:
            raise RouteGraphError(
                "no executable route remains after dependency checks"
            )
        return {
            "contract_version": ROUTE_EVALUATION_CONTRACT,
            "graph_id": self.graph_id,
            "graph_revision": self.revision,
            "graph_snapshot": self.snapshot,
            "phase": active_phase,
            "candidates": selected_candidates,
            "excluded": excluded,
            "candidate_count": len(selected_candidates),
            "excluded_count": len(excluded),
        }


def _boolean(
    observation: Mapping[str, object],
    field: str,
    default: bool,
) -> bool:
    value = observation.get(field, default)
    if not isinstance(value, bool):
        raise RouteGraphError(f"resource observation {field} must be boolean")
    return value


def _observation_number(
    observation: Mapping[str, object],
    field: str,
    default: float,
    minimum: float,
    maximum: float,
) -> float:
    return _number(observation.get(field, default), field, minimum, maximum)


def load_route_graph(path: str | Path) -> RouteGraph:
    """Load one bounded UTF-8 route graph file."""

    source = Path(path)
    try:
        size = source.stat().st_size
    except OSError as exc:
        raise RouteGraphError("route graph file is not readable") from exc
    if size > MAX_ROUTE_FILE_BYTES:
        raise RouteGraphError("route graph file exceeds the byte limit")
    try:
        value = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise RouteGraphError("route graph file must contain UTF-8 JSON") from exc
    return RouteGraph.from_mapping(value)
