"""Declarative robot capability registry exposed to AI planners."""

from __future__ import annotations

import hashlib
import json
import math
import re
import unicodedata
from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

SAFE_TEXT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
CANONICAL_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@-]{0,191}$")
WORD = re.compile(r"[^\W_]+", re.UNICODE)
MANIFEST_CONTRACT_VERSION = "flyto.capability-manifest.v1"
ROUTE_CONTRACT_VERSION = "flyto.capability-route.v1"
GOAL_FRAME_CONTRACT_VERSION = "flyto.goal-frame.v1"


class CapabilityValidationError(ValueError):
    """Raised when an AI-selected capability call is unavailable or unsafe."""


def _semantic_ids(value: object, field_name: str) -> tuple[str, ...]:
    if not isinstance(value, (list, tuple, set, frozenset)):
        raise CapabilityValidationError(f"goal_frame.{field_name} must be an array")
    parsed = tuple(sorted({str(item) for item in value}))
    if len(parsed) > 128:
        raise CapabilityValidationError(f"goal_frame.{field_name} exceeds 128 items")
    if any(not CANONICAL_ID.fullmatch(item) for item in parsed):
        raise CapabilityValidationError(
            f"goal_frame.{field_name} contains an unsafe semantic identifier"
        )
    return parsed


@dataclass(frozen=True)
class GoalFrame:
    """Language-neutral intent, affordance, effect, event, and constraint contract."""

    intent_ids: tuple[str, ...] = ()
    required_affordances: tuple[str, ...] = ()
    desired_effects: tuple[str, ...] = ()
    trigger_events: tuple[str, ...] = ()
    constraints: tuple[dict[str, object], ...] = ()

    @classmethod
    def from_mapping(cls, value: Mapping[str, object]) -> GoalFrame:
        allowed = {
            "contract_version",
            "intent_ids",
            "required_affordances",
            "desired_effects",
            "trigger_events",
            "constraints",
        }
        unknown = sorted(set(value) - allowed)
        if unknown:
            raise CapabilityValidationError(
                "goal_frame contains unsupported fields: " + ", ".join(unknown)
            )
        if value.get("contract_version") != GOAL_FRAME_CONTRACT_VERSION:
            raise CapabilityValidationError(
                "goal_frame.contract_version is not supported"
            )
        raw_constraints = value.get("constraints", [])
        if not isinstance(raw_constraints, list) or len(raw_constraints) > 64:
            raise CapabilityValidationError(
                "goal_frame.constraints must contain at most 64 items"
            )
        constraints: list[dict[str, object]] = []
        for item in raw_constraints:
            if not isinstance(item, Mapping) or set(item) != {
                "key",
                "operator",
                "value",
            }:
                raise CapabilityValidationError(
                    "goal_frame constraint requires only key, operator, and value"
                )
            key = str(item["key"])
            operator = str(item["operator"])
            if not CANONICAL_ID.fullmatch(key) or not SAFE_TEXT.fullmatch(operator):
                raise CapabilityValidationError(
                    "goal_frame constraint key and operator must be safe identifiers"
                )
            constraints.append(
                {"key": key, "operator": operator, "value": item["value"]}
            )
        try:
            payload = json.dumps(
                constraints,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
            )
        except (TypeError, ValueError) as exc:
            raise CapabilityValidationError(
                "goal_frame.constraints must be JSON serializable"
            ) from exc
        if len(payload.encode("utf-8")) > 16_384:
            raise CapabilityValidationError(
                "goal_frame.constraints exceeds 16384 bytes"
            )
        return cls(
            intent_ids=_semantic_ids(value.get("intent_ids", []), "intent_ids"),
            required_affordances=_semantic_ids(
                value.get("required_affordances", []),
                "required_affordances",
            ),
            desired_effects=_semantic_ids(
                value.get("desired_effects", []),
                "desired_effects",
            ),
            trigger_events=_semantic_ids(
                value.get("trigger_events", []),
                "trigger_events",
            ),
            constraints=tuple(constraints),
        )

    def to_dict(self) -> dict[str, object]:
        return {
            "contract_version": GOAL_FRAME_CONTRACT_VERSION,
            "intent_ids": list(self.intent_ids),
            "required_affordances": list(self.required_affordances),
            "desired_effects": list(self.desired_effects),
            "trigger_events": list(self.trigger_events),
            "constraints": [dict(item) for item in self.constraints],
        }


@dataclass(frozen=True)
class ArgumentSpec:
    """Machine-readable constraint for one atomic capability argument."""

    name: str
    value_type: str
    required: bool = True
    description: str = ""
    minimum: float | None = None
    maximum: float | None = None
    choices: tuple[str, ...] = ()
    default: object | None = None
    required_when: tuple[str, object] | None = None

    def validate(self, value: object, field_name: str) -> object:
        if self.value_type == "number":
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise CapabilityValidationError(f"{field_name} must be a number")
            parsed = float(value)
            if not math.isfinite(parsed):
                raise CapabilityValidationError(f"{field_name} must be finite")
            if self.minimum is not None and parsed < self.minimum:
                raise CapabilityValidationError(
                    f"{field_name} must be at least {self.minimum}"
                )
            if self.maximum is not None and parsed > self.maximum:
                raise CapabilityValidationError(
                    f"{field_name} must be at most {self.maximum}"
                )
            return parsed
        if self.value_type == "boolean":
            if not isinstance(value, bool):
                raise CapabilityValidationError(f"{field_name} must be a boolean")
            return value
        if self.value_type == "string":
            if not isinstance(value, str) or not SAFE_TEXT.fullmatch(value):
                raise CapabilityValidationError(f"{field_name} must be a safe identifier")
            if self.choices and value not in self.choices:
                raise CapabilityValidationError(
                    f"{field_name} must be one of {', '.join(self.choices)}"
                )
            return value
        if self.value_type == "text":
            if not isinstance(value, str):
                raise CapabilityValidationError(f"{field_name} must be text")
            parsed_text = unicodedata.normalize("NFKC", value).strip()
            if (
                not parsed_text
                or len(parsed_text) > 128
                or len(parsed_text.encode("utf-8")) > 512
                or any(
                    unicodedata.category(character) in {"Cc", "Cs"}
                    for character in parsed_text
                )
            ):
                raise CapabilityValidationError(
                    f"{field_name} must be bounded printable Unicode text"
                )
            return parsed_text
        raise RuntimeError(f"unsupported ArgumentSpec value_type: {self.value_type}")

    def to_catalog(self) -> dict[str, Any]:
        result: dict[str, Any] = {
            "name": self.name,
            "type": self.value_type,
            "required": self.required,
            "description": self.description,
        }
        if self.minimum is not None:
            result["minimum"] = self.minimum
        if self.maximum is not None:
            result["maximum"] = self.maximum
        if self.choices:
            result["choices"] = list(self.choices)
        if self.default is not None:
            result["default"] = self.default
        if self.required_when is not None:
            result["required_when"] = {
                "argument": self.required_when[0],
                "equals": self.required_when[1],
            }
        return result


@dataclass(frozen=True)
class CapabilityDefinition:
    """One executable atom and the contract an AI may use to call it."""

    name: str
    description: str
    control_class: str
    required_observations: tuple[str, ...]
    arguments: tuple[ArgumentSpec, ...]
    safety_notes: str
    canonical_id: str = ""
    version: str = "1.0.0"
    domain: str = "robotics"
    tags: tuple[str, ...] = ()
    aliases: tuple[str, ...] = ()
    required_resources: tuple[str, ...] = ()
    required_permissions: tuple[str, ...] = ()
    compatible_robots: tuple[str, ...] = ("*",)
    safety_class: str = "controlled"
    side_effects: tuple[str, ...] = ()
    positive_examples: tuple[str, ...] = ()
    negative_examples: tuple[str, ...] = ()
    recovery_capabilities: tuple[str, ...] = ()
    intent_ids: tuple[str, ...] = ()
    affordances: tuple[str, ...] = ()
    effects: tuple[str, ...] = ()
    handled_events: tuple[str, ...] = ()
    legacy_zero_score_fallback: bool = True

    def validate_arguments(self, raw: object) -> dict[str, object]:
        if not isinstance(raw, dict):
            raise CapabilityValidationError(f"{self.name}.arguments must be an object")
        specs = {spec.name: spec for spec in self.arguments}
        unknown = sorted(set(raw) - set(specs))
        if unknown:
            raise CapabilityValidationError(
                f"{self.name}.arguments contains unsupported fields: {', '.join(unknown)}"
            )
        normalized: dict[str, object] = {}
        for name, spec in specs.items():
            if name in raw:
                normalized[name] = spec.validate(raw[name], f"{self.name}.arguments.{name}")
            elif spec.required:
                raise CapabilityValidationError(
                    f"{self.name}.arguments is missing required field: {name}"
                )
            elif spec.default is not None:
                normalized[name] = spec.default
        for name, spec in specs.items():
            if (
                spec.required_when is not None
                and normalized.get(spec.required_when[0]) == spec.required_when[1]
                and name not in normalized
            ):
                raise CapabilityValidationError(
                    f"{self.name}.arguments.{name} is required when "
                    f"{spec.required_when[0]} is {spec.required_when[1]}"
                )
        return normalized

    def to_catalog(self) -> dict[str, Any]:
        return {
            "manifest_contract": MANIFEST_CONTRACT_VERSION,
            "canonical_id": self.canonical_id
            or f"robotics.{self.control_class}.{self.name}@1",
            "runtime_name": self.name,
            "name": self.name,
            "version": self.version,
            "domain": self.domain,
            "description": self.description,
            "tags": list(self.tags),
            "aliases": list(self.aliases),
            "control_class": self.control_class,
            "safety_class": self.safety_class,
            "required_observations": list(self.required_observations),
            "required_resources": list(self.required_resources),
            "required_permissions": list(self.required_permissions),
            "compatible_robots": list(self.compatible_robots),
            "arguments": [argument.to_catalog() for argument in self.arguments],
            "safety_notes": self.safety_notes,
            "side_effects": list(self.side_effects),
            "positive_examples": list(self.positive_examples),
            "negative_examples": list(self.negative_examples),
            "recovery_capabilities": list(self.recovery_capabilities),
            "intent_ids": list(self.intent_ids),
            "affordances": list(self.affordances),
            "effects": list(self.effects),
            "handled_events": list(self.handled_events),
        }


@dataclass(frozen=True)
class CapabilityRoutingContext:
    """Known runtime constraints used for deterministic hard filtering."""

    robot_model: str = ""
    available_observations: frozenset[str] | None = None
    available_resources: frozenset[str] | None = None
    granted_permissions: frozenset[str] | None = None
    enabled_capabilities: frozenset[str] | None = None

    @classmethod
    def from_mapping(
        cls,
        value: Mapping[str, object] | None,
    ) -> CapabilityRoutingContext:
        if value is None:
            return cls()

        def optional_set(name: str) -> frozenset[str] | None:
            if name not in value:
                return None
            raw = value[name]
            if not isinstance(raw, (list, tuple, set, frozenset)):
                raise CapabilityValidationError(f"routing_context.{name} must be an array")
            parsed = frozenset(str(item) for item in raw)
            if any(not SAFE_TEXT.fullmatch(item) for item in parsed):
                raise CapabilityValidationError(
                    f"routing_context.{name} contains an unsafe identifier"
                )
            return parsed

        robot_model = str(value.get("robot_model", ""))
        if robot_model and not SAFE_TEXT.fullmatch(robot_model):
            raise CapabilityValidationError(
                "routing_context.robot_model must be a safe identifier"
            )
        return cls(
            robot_model=robot_model,
            available_observations=optional_set("available_observations"),
            available_resources=optional_set("available_resources"),
            granted_permissions=optional_set("granted_permissions"),
            enabled_capabilities=optional_set("enabled_capabilities"),
        )

    def to_dict(self) -> dict[str, object]:
        result: dict[str, object] = {}
        if self.robot_model:
            result["robot_model"] = self.robot_model
        for field_name in (
            "available_observations",
            "available_resources",
            "granted_permissions",
            "enabled_capabilities",
        ):
            value = getattr(self, field_name)
            if value is not None:
                result[field_name] = sorted(value)
        return result


@dataclass(frozen=True)
class CapabilityCandidate:
    """One shortlisted capability with reproducible selection evidence."""

    canonical_id: str
    runtime_name: str
    score: float
    reasons: tuple[str, ...]
    selected_by: str = "deterministic_hybrid_v1"

    def to_dict(self) -> dict[str, object]:
        return {
            "canonical_id": self.canonical_id,
            "runtime_name": self.runtime_name,
            "score": round(self.score, 4),
            "reasons": list(self.reasons),
            "selected_by": self.selected_by,
        }


@dataclass(frozen=True)
class CapabilityRoute:
    """Bounded shortlist sent to an LLM instead of the complete registry."""

    registry_snapshot: str
    candidates: tuple[CapabilityCandidate, ...]
    excluded: tuple[tuple[str, tuple[str, ...]], ...]
    confidence: float
    needs_clarification: bool
    context: CapabilityRoutingContext
    goal_frame: GoalFrame | None
    semantic_required: tuple[str, ...]
    semantic_matched: tuple[str, ...]
    semantic_missing: tuple[str, ...]

    @property
    def names(self) -> tuple[str, ...]:
        return tuple(candidate.runtime_name for candidate in self.candidates)

    def to_dict(self) -> dict[str, object]:
        return {
            "contract_version": ROUTE_CONTRACT_VERSION,
            "registry_snapshot": self.registry_snapshot,
            "selection_method": (
                "hard_filter_then_semantic_frame_rank_v1"
                if self.goal_frame is not None
                else "hard_filter_then_deterministic_hybrid_rank_v1"
            ),
            "confidence": round(self.confidence, 4),
            "needs_clarification": self.needs_clarification,
            "candidates": [candidate.to_dict() for candidate in self.candidates],
            "excluded_count": len(self.excluded),
            "excluded": [
                {"runtime_name": name, "reasons": list(reasons)}
                for name, reasons in self.excluded
            ],
            "routing_context": self.context.to_dict(),
            "goal_frame": (
                self.goal_frame.to_dict() if self.goal_frame is not None else None
            ),
            "semantic_coverage": {
                "required": list(self.semantic_required),
                "matched": list(self.semantic_matched),
                "missing": list(self.semantic_missing),
                "ratio": round(
                    len(self.semantic_matched) / len(self.semantic_required)
                    if self.semantic_required
                    else (1.0 if self.goal_frame is not None else 0.0),
                    4,
                ),
            },
        }


def _normalized(value: str) -> str:
    return unicodedata.normalize("NFKC", value).casefold()


def _words(value: str) -> set[str]:
    return set(WORD.findall(_normalized(value)))


class CapabilityRegistry:
    """Validated vocabulary separating semantic AI planning from motor control."""

    def __init__(self, definitions: tuple[CapabilityDefinition, ...]) -> None:
        self._definitions: dict[str, CapabilityDefinition] = {}
        canonical_ids: set[str] = set()
        for definition in definitions:
            if not SAFE_TEXT.fullmatch(definition.name):
                raise ValueError(f"unsafe capability name: {definition.name}")
            if definition.name in self._definitions:
                raise ValueError(f"duplicate capability: {definition.name}")
            canonical_id = definition.to_catalog()["canonical_id"]
            if not isinstance(canonical_id, str) or not CANONICAL_ID.fullmatch(canonical_id):
                raise ValueError(f"unsafe canonical capability ID: {canonical_id}")
            if canonical_id in canonical_ids:
                raise ValueError(f"duplicate canonical capability ID: {canonical_id}")
            canonical_ids.add(canonical_id)
            self._definitions[definition.name] = definition

    def definition(self, name: str) -> CapabilityDefinition:
        try:
            return self._definitions[name]
        except KeyError as exc:
            raise CapabilityValidationError(f"capability is not registered: {name}") from exc

    def validate_call(self, name: str, arguments: object) -> dict[str, object]:
        return self.definition(name).validate_arguments(arguments)

    def catalog(self) -> list[dict[str, Any]]:
        return [
            self._definitions[name].to_catalog() for name in sorted(self._definitions)
        ]

    def catalog_for(self, names: tuple[str, ...]) -> list[dict[str, Any]]:
        return [self.definition(name).to_catalog() for name in names]

    def snapshot_hash(self) -> str:
        payload = json.dumps(
            self.catalog(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        )
        return f"sha256:{hashlib.sha256(payload.encode('utf-8')).hexdigest()}"

    def route(
        self,
        goal: str,
        *,
        goal_frame: GoalFrame | Mapping[str, object] | None = None,
        context: CapabilityRoutingContext | Mapping[str, object] | None = None,
        limit: int = 8,
    ) -> CapabilityRoute:
        """Hard-filter and rank by a language-neutral frame or lexical fallback."""
        if not 1 <= limit <= 32:
            raise CapabilityValidationError("capability route limit must be between 1 and 32")
        active_goal_frame = (
            goal_frame
            if isinstance(goal_frame, GoalFrame)
            else GoalFrame.from_mapping(goal_frame)
            if goal_frame is not None
            else None
        )
        active_context = (
            context
            if isinstance(context, CapabilityRoutingContext)
            else CapabilityRoutingContext.from_mapping(context)
        )
        query = _normalized(goal)
        query_words = _words(goal)
        ranked: list[tuple[float, str, CapabilityCandidate]] = []
        excluded: list[tuple[str, tuple[str, ...]]] = []

        for name, definition in self._definitions.items():
            hard_failures = self._hard_filter(definition, active_context)
            if hard_failures:
                excluded.append((name, hard_failures))
                continue
            score, reasons = self._score(
                definition,
                query,
                query_words,
                active_goal_frame,
            )
            canonical_id = str(definition.to_catalog()["canonical_id"])
            candidate = CapabilityCandidate(
                canonical_id=canonical_id,
                runtime_name=name,
                score=score,
                reasons=reasons or ("deterministic_tiebreak",),
                selected_by=(
                    "deterministic_semantic_frame_v1"
                    if active_goal_frame is not None
                    else "deterministic_hybrid_v1"
                ),
            )
            ranked.append((score, canonical_id, candidate))

        ranked.sort(key=lambda item: (-item[0], item[1]))
        selection_pool = (
            [item for item in ranked if item[0] > 0.0]
            if active_goal_frame is not None
            else [
                item
                for item in ranked
                if item[0] > 0.0
                or self.definition(item[2].runtime_name).legacy_zero_score_fallback
            ]
        )
        selected = [item[2] for item in selection_pool[:limit]]
        motion_selected = any(
            self.definition(item.runtime_name).control_class == "motion"
            and item.score > 0
            for item in selected
        )
        if motion_selected and "safe_stop" in self._definitions:
            selected = self._ensure_candidate(selected, ranked, "safe_stop", limit)

        top_score = ranked[0][0] if ranked else 0.0
        relevant_scores = [
            score
            for score, _canonical_id, candidate in ranked
            if self.definition(candidate.runtime_name).control_class
            not in {"safety", "human_gate"}
        ]
        top_relevant = relevant_scores[0] if relevant_scores else top_score
        second_relevant = relevant_scores[1] if len(relevant_scores) > 1 else 0.0
        semantic_required, semantic_matched, semantic_missing = (
            self._semantic_coverage(active_goal_frame, selected)
        )
        if active_goal_frame is not None:
            confidence = (
                len(semantic_matched) / len(semantic_required)
                if semantic_required
                else 1.0
            )
            needs_clarification = not selected or bool(semantic_missing)
        else:
            confidence = min(1.0, max(0.0, top_relevant / 10.0))
            needs_clarification = not selected or top_relevant < 2.0
            if top_relevant >= 2.0 and second_relevant >= 2.0:
                needs_clarification = top_relevant - second_relevant < 0.35

        return CapabilityRoute(
            registry_snapshot=self.snapshot_hash(),
            candidates=tuple(selected),
            excluded=tuple(sorted(excluded)),
            confidence=confidence,
            needs_clarification=needs_clarification,
            context=active_context,
            goal_frame=active_goal_frame,
            semantic_required=semantic_required,
            semantic_matched=semantic_matched,
            semantic_missing=semantic_missing,
        )

    def _hard_filter(
        self,
        definition: CapabilityDefinition,
        context: CapabilityRoutingContext,
    ) -> tuple[str, ...]:
        failures: list[str] = []
        if (
            context.enabled_capabilities is not None
            and definition.name not in context.enabled_capabilities
        ):
            failures.append("not_enabled")
        if (
            context.robot_model
            and "*" not in definition.compatible_robots
            and context.robot_model not in definition.compatible_robots
        ):
            failures.append("robot_incompatible")
        checks = (
            (
                context.available_observations,
                definition.required_observations,
                "missing_observation",
            ),
            (
                context.available_resources,
                definition.required_resources,
                "missing_resource",
            ),
            (
                context.granted_permissions,
                definition.required_permissions,
                "permission_denied",
            ),
        )
        for available, required, reason in checks:
            if available is not None and not set(required).issubset(available):
                failures.append(reason)
        return tuple(failures)

    @staticmethod
    def _score(
        definition: CapabilityDefinition,
        query: str,
        query_words: set[str],
        goal_frame: GoalFrame | None,
    ) -> tuple[float, tuple[str, ...]]:
        score = 0.0
        reasons: list[str] = []
        if goal_frame is not None:
            semantic_pairs = (
                (goal_frame.intent_ids, definition.intent_ids, 12.0, "intent_match"),
                (
                    goal_frame.required_affordances,
                    definition.affordances,
                    16.0,
                    "affordance_match",
                ),
                (
                    goal_frame.desired_effects,
                    definition.effects,
                    8.0,
                    "effect_match",
                ),
                (
                    goal_frame.trigger_events,
                    definition.handled_events,
                    10.0,
                    "event_match",
                ),
            )
            for required, provided, weight, reason in semantic_pairs:
                overlap = set(required) & set(provided)
                if overlap:
                    score += weight * len(overlap)
                    reasons.append(reason)
            return score, tuple(reasons)

        searchable_names = (
            definition.name,
            definition.canonical_id,
            definition.name.replace("_", " "),
        )
        if any(term and _normalized(term) in query for term in searchable_names):
            score += 8.0
            reasons.append("identifier_match")

        alias_hits = [
            alias for alias in definition.aliases if _normalized(alias) in query
        ]
        if alias_hits:
            score += min(12.0, 6.0 + 2.0 * (len(alias_hits) - 1))
            reasons.append("alias_phrase_match")

        tag_hits = [tag for tag in definition.tags if _normalized(tag) in query]
        if tag_hits:
            score += min(6.0, 2.0 * len(tag_hits))
            reasons.append("tag_match")

        candidate_words = _words(
            " ".join(
                (
                    definition.name,
                    definition.description,
                    *definition.aliases,
                    *definition.tags,
                    *definition.positive_examples,
                )
            )
        )
        overlap = query_words & candidate_words
        if overlap:
            score += min(6.0, 1.25 * len(overlap))
            reasons.append("token_overlap")

        if any(_normalized(example) in query for example in definition.positive_examples):
            score += 3.0
            reasons.append("positive_example_match")
        if any(_normalized(example) in query for example in definition.negative_examples):
            score -= 8.0
            reasons.append("negative_example_match")
        return max(0.0, score), tuple(reasons)

    def _semantic_coverage(
        self,
        goal_frame: GoalFrame | None,
        selected: list[CapabilityCandidate],
    ) -> tuple[tuple[str, ...], tuple[str, ...], tuple[str, ...]]:
        if goal_frame is None:
            return (), (), ()
        required = {
            *(f"intent:{item}" for item in goal_frame.intent_ids),
            *(f"affordance:{item}" for item in goal_frame.required_affordances),
            *(f"effect:{item}" for item in goal_frame.desired_effects),
            *(f"event:{item}" for item in goal_frame.trigger_events),
        }
        provided: set[str] = set()
        for candidate in selected:
            definition = self.definition(candidate.runtime_name)
            provided.update(f"intent:{item}" for item in definition.intent_ids)
            provided.update(f"affordance:{item}" for item in definition.affordances)
            provided.update(f"effect:{item}" for item in definition.effects)
            provided.update(f"event:{item}" for item in definition.handled_events)
        matched = required & provided
        return (
            tuple(sorted(required)),
            tuple(sorted(matched)),
            tuple(sorted(required - provided)),
        )

    @staticmethod
    def _ensure_candidate(
        selected: list[CapabilityCandidate],
        ranked: list[tuple[float, str, CapabilityCandidate]],
        runtime_name: str,
        limit: int,
    ) -> list[CapabilityCandidate]:
        if any(item.runtime_name == runtime_name for item in selected):
            return selected
        required = next(
            (item[2] for item in ranked if item[2].runtime_name == runtime_name),
            None,
        )
        if required is None:
            return selected
        if len(selected) >= limit:
            selected = selected[:-1]
        return [*selected, required]

    @property
    def names(self) -> frozenset[str]:
        return frozenset(self._definitions)


def default_capability_registry() -> CapabilityRegistry:
    """Return the executable reference atoms; adapters may register more later."""
    return CapabilityRegistry(
        (
            CapabilityDefinition(
                name="navigate",
                description="Navigate to one bounded two-dimensional target pose.",
                canonical_id="robotics.motion.navigate@1",
                tags=("navigation", "waypoint", "station", "pose", "座標", "站點"),
                aliases=(
                    "navigate",
                    "go to station",
                    "waypoint",
                    "導航",
                    "前往站點",
                    "移動到座標",
                ),
                control_class="motion",
                required_observations=("odometry", "minimum_range"),
                required_resources=("base_controller",),
                safety_class="motion_bounded",
                side_effects=("robot_motion",),
                intent_ids=("route.navigate.pose",),
                affordances=("motion.navigate.pose",),
                effects=("robot.pose.reached",),
                handled_events=("obstacle.detected",),
                positive_examples=("前往護理站", "navigate to waypoint"),
                negative_examples=("沿藍線", "follow the blue line"),
                recovery_capabilities=("safe_stop", "wait_until_clear", "ask_human"),
                arguments=(
                    ArgumentSpec("station_id", "string", description="Semantic target name."),
                    ArgumentSpec("x", "number", minimum=-1000.0, maximum=1000.0),
                    ArgumentSpec("y", "number", minimum=-1000.0, maximum=1000.0),
                    ArgumentSpec(
                        "yaw",
                        "number",
                        required=False,
                        minimum=-math.pi,
                        maximum=math.pi,
                        default=0.0,
                    ),
                ),
                safety_notes="Controller clamps velocity and stops for lidar obstacles.",
            ),
            CapabilityDefinition(
                name="navigate_to_location",
                description=(
                    "Resolve one stable semantic location ID through the trusted map "
                    "store, then navigate to its stored pose."
                ),
                canonical_id="robotics.motion.navigate_to_location@1",
                tags=("semantic-map", "saved-location", "named-place"),
                aliases=(
                    "navigate to saved location",
                    "go to remembered place",
                    "navigate by location id",
                ),
                control_class="motion",
                required_observations=("odometry", "minimum_range"),
                required_resources=("base_controller", "semantic_map"),
                required_permissions=("location.read",),
                safety_class="motion_bounded",
                side_effects=("robot_motion",),
                intent_ids=("route.navigate.location",),
                affordances=("motion.navigate.semantic_location",),
                effects=("robot.location.reached",),
                handled_events=("obstacle.detected", "location.missing"),
                positive_examples=(
                    "navigate to hospital.nurse_station.1",
                    "go to a previously remembered location",
                ),
                negative_examples=(
                    "navigate to raw x y coordinates",
                    "follow a colored line",
                ),
                recovery_capabilities=("safe_stop", "ask_human"),
                legacy_zero_score_fallback=False,
                arguments=(
                    ArgumentSpec(
                        "location_id",
                        "string",
                        description=(
                            "Stable map-scoped ID; trusted coordinates are resolved "
                            "outside the planner."
                        ),
                    ),
                ),
                safety_notes=(
                    "The planner cannot provide x, y, or yaw. Compilation resolves the "
                    "location_id against the configured map and fails closed on mismatch."
                ),
            ),
            CapabilityDefinition(
                name="save_current_location",
                description=(
                    "Atomically associate a stable location ID and multilingual label "
                    "with the robot's current trusted odometry pose."
                ),
                canonical_id="robotics.memory.save_current_location@1",
                tags=("semantic-map", "location-memory", "teach-location"),
                aliases=(
                    "remember this location",
                    "save current location",
                    "teach this place",
                ),
                control_class="memory",
                required_observations=("odometry",),
                required_resources=("semantic_map",),
                required_permissions=("location.write",),
                safety_class="stationary",
                side_effects=("semantic_map_write",),
                intent_ids=("location.remember.current_pose",),
                affordances=("map.semantic_location.write",),
                effects=("location.pose.saved",),
                handled_events=("location.teach.requested",),
                positive_examples=(
                    "remember the current pose as hospital.nurse_station.1",
                    "store this place using a stable location id",
                ),
                negative_examples=(
                    "overwrite the location with model supplied coordinates",
                ),
                recovery_capabilities=("ask_human", "safe_stop"),
                legacy_zero_score_fallback=False,
                arguments=(
                    ArgumentSpec(
                        "location_id",
                        "string",
                        description=(
                            "Stable map-scoped ID created independently from its label."
                        ),
                    ),
                    ArgumentSpec(
                        "label",
                        "text",
                        description=(
                            "Bounded Unicode display label; never used as the trusted key."
                        ),
                    ),
                ),
                safety_notes=(
                    "Always emits zero velocity. The stored pose comes only from current "
                    "odometry and is written through the atomic semantic map store."
                ),
            ),
            CapabilityDefinition(
                name="follow_line",
                description=(
                    "Follow a visually observed line until the next line appears or the "
                    "current line ends."
                ),
                canonical_id="robotics.vision.follow_line@1",
                tags=("line", "route", "color", "camera", "循線", "彩色路線"),
                aliases=(
                    "follow line",
                    "colored line",
                    "blue line",
                    "yellow line",
                    "purple line",
                    "循線",
                    "沿線",
                    "藍線",
                    "黃線",
                    "紫線",
                    "彩色路線",
                ),
                control_class="motion",
                required_observations=("camera.line_scene", "minimum_range"),
                required_resources=("base_controller", "camera"),
                safety_class="motion_bounded",
                side_effects=("robot_motion",),
                intent_ids=("route.follow.sequence",),
                affordances=("motion.follow.visual_line",),
                effects=("route.segment.completed", "route.sequence.completed"),
                handled_events=(
                    "line.changed",
                    "line.detected",
                    "obstacle.detected",
                ),
                positive_examples=("先走藍線再走黃線", "follow blue then purple"),
                negative_examples=("移動到座標", "navigate to waypoint"),
                recovery_capabilities=("safe_stop", "wait_until_clear", "ask_human"),
                arguments=(
                    ArgumentSpec("line_id", "string", description="Semantic line or route ID."),
                    ArgumentSpec(
                        "color",
                        "string",
                        choices=("black", "blue", "green", "purple", "red", "white", "yellow"),
                    ),
                    ArgumentSpec(
                        "speed",
                        "number",
                        required=False,
                        minimum=0.02,
                        maximum=0.5,
                        default=0.16,
                    ),
                    ArgumentSpec(
                        "steering_gain",
                        "number",
                        required=False,
                        minimum=0.0,
                        maximum=3.0,
                        default=1.2,
                    ),
                    ArgumentSpec(
                        "completion",
                        "string",
                        required=False,
                        choices=("line_end", "next_color"),
                        default="line_end",
                    ),
                    ArgumentSpec(
                        "next_color",
                        "string",
                        required=False,
                        choices=("black", "blue", "green", "purple", "red", "white", "yellow"),
                        required_when=("completion", "next_color"),
                    ),
                    ArgumentSpec(
                        "minimum_follow_seconds",
                        "number",
                        required=False,
                        minimum=0.0,
                        maximum=300.0,
                        default=0.5,
                    ),
                    ArgumentSpec(
                        "transition_search_seconds",
                        "number",
                        required=False,
                        minimum=0.0,
                        maximum=6.0,
                        default=1.5,
                    ),
                ),
                safety_notes=(
                    "AI chooses route semantics and bounded gains; the deterministic follower "
                    "owns steering, obstacle stop, and lost-line behavior."
                ),
            ),
            CapabilityDefinition(
                name="dwell",
                description="Remain stopped for a bounded duration.",
                canonical_id="robotics.time.dwell@1",
                tags=("wait", "timer", "停留", "等待"),
                aliases=("dwell", "wait for seconds", "停留", "等待幾秒"),
                control_class="timed",
                required_observations=(),
                safety_class="stationary",
                intent_ids=("time.dwell",),
                affordances=("time.wait.bounded",),
                effects=("time.elapsed",),
                positive_examples=("停留十秒", "wait for five seconds"),
                arguments=(
                    ArgumentSpec("seconds", "number", minimum=0.0, maximum=300.0),
                ),
                safety_notes="Always emits zero velocity.",
            ),
            CapabilityDefinition(
                name="wait_until_clear",
                description=(
                    "Remain stopped until lidar clearance has been continuously safe "
                    "for a bounded verification period."
                ),
                canonical_id="robotics.safety.wait_until_clear@1",
                tags=("obstacle", "clearance", "person", "避障", "有人", "障礙"),
                aliases=(
                    "wait until clear",
                    "obstacle",
                    "person blocking",
                    "有人擋住",
                    "遇到人",
                    "等待淨空",
                    "障礙物",
                ),
                control_class="safety",
                required_observations=("minimum_range",),
                required_resources=("range_sensor",),
                safety_class="safety_stop",
                intent_ids=("safety.wait.clearance",),
                affordances=("safety.wait_until_clear",),
                effects=("path.clear",),
                handled_events=("human.detected", "obstacle.detected"),
                positive_examples=("有人擋住就等待", "wait until the obstacle clears"),
                recovery_capabilities=("ask_human", "safe_stop"),
                arguments=(
                    ArgumentSpec(
                        "clear_seconds",
                        "number",
                        required=False,
                        minimum=0.1,
                        maximum=30.0,
                        default=0.5,
                    ),
                ),
                safety_notes=(
                    "Always emits zero velocity. Clearance uses the job safety distance, "
                    "and the primitive timeout bounds the maximum wait."
                ),
            ),
            CapabilityDefinition(
                name="ask_human",
                description=(
                    "Stop and request an explicit external approval for one named gate."
                ),
                canonical_id="robotics.human.ask@1",
                tags=("approval", "clarify", "operator", "人工", "核准", "詢問"),
                aliases=(
                    "ask human",
                    "request approval",
                    "clarify",
                    "詢問人員",
                    "請求核准",
                    "人工確認",
                    "不確定",
                ),
                control_class="human_gate",
                required_observations=("human_decision",),
                required_resources=("operator_channel",),
                safety_class="human_gate",
                intent_ids=("human.approval.request",),
                affordances=("human.request_decision",),
                effects=("human.decision.requested",),
                handled_events=("ambiguity.detected",),
                positive_examples=("不確定就問我", "ask an operator for approval"),
                recovery_capabilities=("resume", "safe_stop"),
                arguments=(
                    ArgumentSpec(
                        "approval_id",
                        "string",
                        description="Stable correlation ID for the approval decision.",
                    ),
                    ArgumentSpec(
                        "prompt_key",
                        "string",
                        description=(
                            "Non-sensitive UI message key resolved outside the robot result."
                        ),
                    ),
                ),
                safety_notes=(
                    "Always emits zero velocity. An LLM cannot satisfy this capability; "
                    "an identified external actor must submit the decision."
                ),
            ),
            CapabilityDefinition(
                name="resume",
                description=(
                    "Verify a previously approved human gate before later motion continues."
                ),
                canonical_id="robotics.human.resume@1",
                tags=("approval", "continue", "resume", "核准", "繼續", "恢復"),
                aliases=("resume", "continue after approval", "核准後繼續", "恢復任務"),
                control_class="human_gate",
                required_observations=("human_decision",),
                required_resources=("operator_channel",),
                safety_class="human_gate",
                intent_ids=("human.approval.resume",),
                affordances=("human.resume_after_approval",),
                effects=("workflow.resumed",),
                handled_events=("human.approved",),
                positive_examples=("取得核准後繼續", "resume after approval"),
                arguments=(
                    ArgumentSpec(
                        "approval_id",
                        "string",
                        description="Approval correlation ID previously accepted by ask_human.",
                    ),
                ),
                safety_notes=(
                    "Does not move the robot. It fails closed unless the matching approval "
                    "exists and was granted."
                ),
            ),
            CapabilityDefinition(
                name="safe_stop",
                description="Stop motion and optionally hold the stopped state.",
                canonical_id="robotics.safety.safe_stop@1",
                tags=("stop", "safety", "emergency", "停止", "安全", "煞停"),
                aliases=(
                    "safe stop",
                    "stop",
                    "emergency stop",
                    "安全停止",
                    "立即停止",
                    "停下來",
                    "煞停",
                ),
                control_class="safety",
                required_observations=(),
                safety_class="safety_stop",
                intent_ids=("safety.stop",),
                affordances=("safety.stop.motion",),
                effects=("robot.motion.stopped",),
                handled_events=(
                    "emergency.requested",
                    "human.detected",
                    "obstacle.detected",
                ),
                positive_examples=("最後安全停止", "stop immediately"),
                arguments=(
                    ArgumentSpec(
                        "seconds",
                        "number",
                        required=False,
                        minimum=0.0,
                        maximum=300.0,
                        default=0.0,
                    ),
                ),
                safety_notes="Always emits zero velocity and cannot be bypassed by the planner.",
            ),
        )
    )
