"""Atomic multi-axis dependency policy and deterministic action derivation."""

from __future__ import annotations

import re
from collections.abc import Mapping
from dataclasses import dataclass

SAFETY_IMPACTS = ("none", "pause", "stop", "abort")
TASK_IMPACTS = ("none", "degrade", "block", "invalidate")
EVIDENCE_REQUIREMENTS = ("none", "record", "required")
SUBSTITUTION_MODES = ("any_healthy", "equivalent", "validated", "none")
SCOPE_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@*-]{0,191}$")


class DependencyPolicyError(ValueError):
    """Raised when a dependency contract or observation is invalid."""


def _choice(value: object, field_name: str, choices: tuple[str, ...]) -> str:
    if not isinstance(value, str) or value not in choices:
        raise DependencyPolicyError(
            f"{field_name} must be one of: {', '.join(choices)}"
        )
    return value


def _bounded_float(
    value: object,
    field_name: str,
    *,
    minimum: float,
    maximum: float,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise DependencyPolicyError(f"{field_name} must be a number")
    number = float(value)
    if not minimum <= number <= maximum:
        raise DependencyPolicyError(
            f"{field_name} must be between {minimum} and {maximum}"
        )
    return number


def _scope_identifier(value: object, field_name: str) -> str:
    if value == "*":
        return "*"
    if not isinstance(value, str) or not SCOPE_IDENTIFIER.fullmatch(value):
        raise DependencyPolicyError(f"{field_name} must be a safe identifier")
    return value


@dataclass(frozen=True)
class DependencyContract:
    """Independent dependency axes; UI labels are derived rather than stored."""

    safety_impact: str
    task_impact: str
    evidence_requirement: str
    substitution_mode: str
    minimum_confidence: float
    max_age_seconds: float
    recovery_timeout_seconds: float
    retry_limit: int
    active_phases: tuple[str, ...]

    @classmethod
    def from_mapping(
        cls,
        value: object = None,
        *,
        default: DependencyContract | None = None,
    ) -> DependencyContract:
        base = default or cls.observer_default()
        if value is None:
            return base
        if not isinstance(value, Mapping):
            raise DependencyPolicyError("dependency contract must be an object")
        raw = value
        allowed = {
            "safety_impact",
            "task_impact",
            "evidence_requirement",
            "substitution_mode",
            "minimum_confidence",
            "max_age_seconds",
            "recovery_timeout_seconds",
            "retry_limit",
            "active_phases",
        }
        unknown = sorted(set(raw) - allowed)
        if unknown:
            raise DependencyPolicyError(
                "dependency contract contains unsupported fields: "
                + ", ".join(unknown)
            )
        minimum_confidence = _bounded_float(
            raw.get("minimum_confidence", base.minimum_confidence),
            "dependency.minimum_confidence",
            minimum=0.0,
            maximum=1.0,
        )
        max_age_seconds = _bounded_float(
            raw.get("max_age_seconds", base.max_age_seconds),
            "dependency.max_age_seconds",
            minimum=0.05,
            maximum=3_600.0,
        )
        recovery_timeout_seconds = _bounded_float(
            raw.get(
                "recovery_timeout_seconds",
                base.recovery_timeout_seconds,
            ),
            "dependency.recovery_timeout_seconds",
            minimum=0.0,
            maximum=3_600.0,
        )
        retry_limit = raw.get("retry_limit", base.retry_limit)
        if (
            isinstance(retry_limit, bool)
            or not isinstance(retry_limit, int)
            or not 0 <= retry_limit <= 10
        ):
            raise DependencyPolicyError(
                "dependency.retry_limit must be an integer between 0 and 10"
            )
        raw_phases = raw.get("active_phases", list(base.active_phases))
        if not isinstance(raw_phases, list) or not 1 <= len(raw_phases) <= 32:
            raise DependencyPolicyError(
                "dependency.active_phases must contain 1 to 32 phase identifiers"
            )
        return cls(
            safety_impact=_choice(
                raw.get("safety_impact", base.safety_impact),
                "dependency.safety_impact",
                SAFETY_IMPACTS,
            ),
            task_impact=_choice(
                raw.get("task_impact", base.task_impact),
                "dependency.task_impact",
                TASK_IMPACTS,
            ),
            evidence_requirement=_choice(
                raw.get("evidence_requirement", base.evidence_requirement),
                "dependency.evidence_requirement",
                EVIDENCE_REQUIREMENTS,
            ),
            substitution_mode=_choice(
                raw.get("substitution_mode", base.substitution_mode),
                "dependency.substitution_mode",
                SUBSTITUTION_MODES,
            ),
            minimum_confidence=minimum_confidence,
            max_age_seconds=max_age_seconds,
            recovery_timeout_seconds=recovery_timeout_seconds,
            retry_limit=retry_limit,
            active_phases=tuple(
                _scope_identifier(item, "dependency.active_phases")
                for item in raw_phases
            ),
        )

    @classmethod
    def observer_default(cls) -> DependencyContract:
        """Default for non-controlling observation and presentation devices."""
        return cls(
            safety_impact="none",
            task_impact="degrade",
            evidence_requirement="record",
            substitution_mode="any_healthy",
            minimum_confidence=0.20,
            max_age_seconds=30.0,
            recovery_timeout_seconds=0.0,
            retry_limit=0,
            active_phases=("*",),
        )

    def to_mapping(self) -> dict[str, object]:
        return {
            "safety_impact": self.safety_impact,
            "task_impact": self.task_impact,
            "evidence_requirement": self.evidence_requirement,
            "substitution_mode": self.substitution_mode,
            "minimum_confidence": self.minimum_confidence,
            "max_age_seconds": self.max_age_seconds,
            "recovery_timeout_seconds": self.recovery_timeout_seconds,
            "retry_limit": self.retry_limit,
            "active_phases": list(self.active_phases),
        }

    def derived_band(self) -> str:
        """Return a compact UI hint without collapsing the runtime contract."""
        if self.safety_impact == "abort" or self.task_impact == "invalidate":
            return "mission_critical"
        if self.safety_impact == "stop":
            return "safety_critical"
        if (
            self.safety_impact == "pause"
            or self.task_impact == "block"
            or self.evidence_requirement == "required"
        ):
            return "required"
        if (
            self.task_impact == "degrade"
            or self.evidence_requirement == "record"
        ):
            return "assistive"
        return "optional"


@dataclass(frozen=True)
class DependencyAssessment:
    """Deterministic response to the loss or degradation of one binding."""

    derived_band: str
    state: str
    action: str
    must_stop: bool
    requires_human: bool
    reason: str


def assess_device_dependency(
    contract: DependencyContract,
    *,
    healthy: bool,
    confidence: float,
    observation_age_seconds: float,
    fallback_available: bool,
    fallback_equivalent: bool = False,
    fallback_validated: bool = False,
    evidence_present: bool = True,
    phase: str = "*",
) -> DependencyAssessment:
    """Derive an action from dependency facts and independent observed quality."""
    flags = (
        healthy,
        fallback_available,
        fallback_equivalent,
        fallback_validated,
        evidence_present,
    )
    if not all(isinstance(flag, bool) for flag in flags):
        raise DependencyPolicyError("dependency health flags must be booleans")
    active_phase = _scope_identifier(phase, "phase")
    observed_confidence = _bounded_float(
        confidence,
        "confidence",
        minimum=0.0,
        maximum=1.0,
    )
    observation_age = _bounded_float(
        observation_age_seconds,
        "observation_age_seconds",
        minimum=0.0,
        maximum=86_400.0,
    )
    if "*" not in contract.active_phases and active_phase not in contract.active_phases:
        return DependencyAssessment(
            derived_band=contract.derived_band(),
            state="not_applicable",
            action="ignore_outside_scope",
            must_stop=False,
            requires_human=False,
            reason="phase_outside_scope",
        )
    evidence_ready = (
        contract.evidence_requirement != "required" or evidence_present
    )
    if (
        healthy
        and observed_confidence >= contract.minimum_confidence
        and observation_age <= contract.max_age_seconds
        and evidence_ready
    ):
        return DependencyAssessment(
            derived_band=contract.derived_band(),
            state="ready",
            action="use_resource",
            must_stop=False,
            requires_human=False,
            reason="quality_within_requirement",
        )
    if not evidence_ready:
        reason = "required_evidence_missing"
    elif not healthy:
        reason = "unhealthy"
    elif observed_confidence < contract.minimum_confidence:
        reason = "confidence_below_requirement"
    else:
        reason = "observation_stale"
    substitute_allowed = (
        fallback_available
        and contract.substitution_mode != "none"
        and (
            contract.substitution_mode == "any_healthy"
            or (
                contract.substitution_mode == "equivalent"
                and fallback_equivalent
            )
            or (
                contract.substitution_mode == "validated"
                and fallback_validated
            )
        )
    )
    must_stop = (
        contract.safety_impact in {"stop", "abort"}
        or contract.task_impact == "invalidate"
    )
    requires_human = (
        contract.safety_impact != "none"
        or contract.task_impact in {"block", "invalidate"}
    )
    if substitute_allowed:
        if must_stop:
            action = "safe_stop_then_switch_substitute"
        elif contract.safety_impact == "pause":
            action = "pause_then_switch_substitute"
        else:
            action = "switch_substitute"
    elif contract.safety_impact == "abort":
        action = "safe_stop_and_abort"
    elif contract.safety_impact == "stop":
        action = "safe_stop_and_escalate"
    elif contract.safety_impact == "pause":
        action = "pause_and_escalate"
    elif contract.task_impact == "invalidate":
        action = "abort_and_invalidate"
    elif contract.task_impact == "block":
        action = "pause_and_escalate"
    elif contract.task_impact == "degrade":
        action = "continue_degraded"
    else:
        action = "continue_without_resource"
    return DependencyAssessment(
        derived_band=contract.derived_band(),
        state="unavailable",
        action=action,
        must_stop=must_stop,
        requires_human=requires_human,
        reason=reason,
    )
