"""Authenticated, replay-resistant human decisions for robot approval gates."""

from __future__ import annotations

import hashlib
import hmac
import json
import re
import time
import uuid
from dataclasses import dataclass
from typing import Any

from .capabilities import SAFE_TEXT

HUMAN_DECISION_CONTRACT_VERSION = "flyto.robotics.human-decision.v1"
MAX_DECISION_BYTES = 16 * 1024
MAX_DECISION_TTL_SECONDS = 300
MAX_CLOCK_SKEW_SECONDS = 30
SIGNATURE_PATTERN = re.compile(r"^[0-9a-f]{64}$")
DECISION_FIELDS = frozenset(
    {
        "contract_version",
        "job_id",
        "robot_id",
        "approval_id",
        "approved",
        "actor_id",
        "issued_at_epoch_seconds",
        "expires_at_epoch_seconds",
        "nonce",
        "signature",
    }
)


class HumanDecisionValidationError(ValueError):
    """Raised when an external approval is malformed, forged, stale, or replayed."""


@dataclass(frozen=True)
class VerifiedHumanDecision:
    """A decision whose signature, scope, freshness, and nonce have been verified."""

    job_id: str
    robot_id: str
    approval_id: str
    approved: bool
    actor_id: str
    issued_at_epoch_seconds: int
    expires_at_epoch_seconds: int
    nonce: str


def _secret_bytes(secret: str | bytes) -> bytes:
    encoded = secret.encode("utf-8") if isinstance(secret, str) else secret
    if not isinstance(encoded, bytes) or len(encoded) < 32:
        raise HumanDecisionValidationError(
            "approval secret must contain at least 32 bytes"
        )
    return encoded


def _identifier(value: object, field_name: str) -> str:
    if not isinstance(value, str) or not SAFE_TEXT.fullmatch(value):
        raise HumanDecisionValidationError(f"{field_name} must be a safe identifier")
    return value


def _integer(value: object, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise HumanDecisionValidationError(f"{field_name} must be an integer")
    return value


def _canonical_unsigned(data: dict[str, Any]) -> bytes:
    unsigned = {key: data[key] for key in sorted(data) if key != "signature"}
    return json.dumps(
        unsigned,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _signature(data: dict[str, Any], secret: str | bytes) -> str:
    return hmac.new(
        _secret_bytes(secret),
        _canonical_unsigned(data),
        hashlib.sha256,
    ).hexdigest()


def build_signed_human_decision(
    *,
    job_id: str,
    robot_id: str,
    approval_id: str,
    approved: bool,
    actor_id: str,
    secret: str | bytes,
    issued_at_epoch_seconds: int | None = None,
    ttl_seconds: int = 60,
    nonce: str | None = None,
) -> dict[str, Any]:
    """Build one short-lived decision envelope for a Flyto or ROS adapter."""
    issued_at = int(time.time()) if issued_at_epoch_seconds is None else _integer(
        issued_at_epoch_seconds,
        "issued_at_epoch_seconds",
    )
    if isinstance(ttl_seconds, bool) or not isinstance(ttl_seconds, int):
        raise HumanDecisionValidationError("ttl_seconds must be an integer")
    if not 1 <= ttl_seconds <= MAX_DECISION_TTL_SECONDS:
        raise HumanDecisionValidationError(
            f"ttl_seconds must be between 1 and {MAX_DECISION_TTL_SECONDS}"
        )
    if not isinstance(approved, bool):
        raise HumanDecisionValidationError("approved must be a boolean")
    data: dict[str, Any] = {
        "contract_version": HUMAN_DECISION_CONTRACT_VERSION,
        "job_id": _identifier(job_id, "job_id"),
        "robot_id": _identifier(robot_id, "robot_id"),
        "approval_id": _identifier(approval_id, "approval_id"),
        "approved": approved,
        "actor_id": _identifier(actor_id, "actor_id"),
        "issued_at_epoch_seconds": issued_at,
        "expires_at_epoch_seconds": issued_at + ttl_seconds,
        "nonce": _identifier(nonce or uuid.uuid4().hex, "nonce"),
    }
    data["signature"] = _signature(data, secret)
    return data


class HumanDecisionAuthenticator:
    """Verify signed decisions and reject a nonce after its first valid use."""

    def __init__(self, secret: str | bytes) -> None:
        self._secret = _secret_bytes(secret)
        self._used_nonces: set[str] = set()

    def verify(
        self,
        value: object,
        *,
        expected_job_id: str,
        expected_robot_id: str,
        now_epoch_seconds: int | None = None,
    ) -> VerifiedHumanDecision:
        decoded = value
        if isinstance(value, str):
            if len(value.encode("utf-8")) > MAX_DECISION_BYTES:
                raise HumanDecisionValidationError("human decision is too large")
            try:
                decoded = json.loads(value)
            except json.JSONDecodeError as exc:
                raise HumanDecisionValidationError(
                    "human decision must be JSON"
                ) from exc
        if not isinstance(decoded, dict):
            raise HumanDecisionValidationError("human decision must be an object")
        if not all(isinstance(key, str) for key in decoded):
            raise HumanDecisionValidationError(
                "human decision field names must be strings"
            )
        try:
            canonical_size = len(_canonical_unsigned(decoded))
        except (TypeError, ValueError) as exc:
            raise HumanDecisionValidationError(
                "human decision must contain JSON-compatible values"
            ) from exc
        if canonical_size > MAX_DECISION_BYTES:
            raise HumanDecisionValidationError("human decision is too large")
        unknown = sorted(set(decoded) - DECISION_FIELDS)
        missing = sorted(DECISION_FIELDS - set(decoded))
        if unknown:
            raise HumanDecisionValidationError(
                f"human decision contains unsupported fields: {', '.join(unknown)}"
            )
        if missing:
            raise HumanDecisionValidationError(
                f"human decision is missing: {', '.join(missing)}"
            )
        signature = decoded["signature"]
        if not isinstance(signature, str) or not SIGNATURE_PATTERN.fullmatch(signature):
            raise HumanDecisionValidationError(
                "human decision signature must be lowercase SHA-256 hex"
            )
        expected_signature = _signature(decoded, self._secret)
        if not hmac.compare_digest(signature, expected_signature):
            raise HumanDecisionValidationError("human decision signature is invalid")
        if decoded["contract_version"] != HUMAN_DECISION_CONTRACT_VERSION:
            raise HumanDecisionValidationError(
                f"contract_version must be {HUMAN_DECISION_CONTRACT_VERSION}"
            )

        job_id = _identifier(decoded["job_id"], "job_id")
        robot_id = _identifier(decoded["robot_id"], "robot_id")
        approval_id = _identifier(decoded["approval_id"], "approval_id")
        actor_id = _identifier(decoded["actor_id"], "actor_id")
        nonce = _identifier(decoded["nonce"], "nonce")
        approved = decoded["approved"]
        if not isinstance(approved, bool):
            raise HumanDecisionValidationError("approved must be a boolean")
        issued_at = _integer(
            decoded["issued_at_epoch_seconds"],
            "issued_at_epoch_seconds",
        )
        expires_at = _integer(
            decoded["expires_at_epoch_seconds"],
            "expires_at_epoch_seconds",
        )
        now = int(time.time()) if now_epoch_seconds is None else _integer(
            now_epoch_seconds,
            "now_epoch_seconds",
        )
        if expires_at <= issued_at:
            raise HumanDecisionValidationError("human decision expiry must follow issue time")
        if expires_at - issued_at > MAX_DECISION_TTL_SECONDS:
            raise HumanDecisionValidationError("human decision lifetime is too long")
        if issued_at > now + MAX_CLOCK_SKEW_SECONDS:
            raise HumanDecisionValidationError("human decision issue time is in the future")
        if expires_at < now:
            raise HumanDecisionValidationError("human decision has expired")
        if job_id != expected_job_id:
            raise HumanDecisionValidationError("human decision job_id does not match")
        if robot_id != expected_robot_id:
            raise HumanDecisionValidationError("human decision robot_id does not match")
        if nonce in self._used_nonces:
            raise HumanDecisionValidationError("human decision nonce was already used")
        self._used_nonces.add(nonce)
        return VerifiedHumanDecision(
            job_id=job_id,
            robot_id=robot_id,
            approval_id=approval_id,
            approved=approved,
            actor_id=actor_id,
            issued_at_epoch_seconds=issued_at,
            expires_at_epoch_seconds=expires_at,
            nonce=nonce,
        )


def decision_to_json(decision: dict[str, Any]) -> str:
    """Serialize a signed decision in the canonical one-line transport form."""
    return json.dumps(
        decision,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    )
