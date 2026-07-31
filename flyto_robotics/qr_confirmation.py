"""Signed, expiring, single-use QR confirmations for delivery handoff."""

from __future__ import annotations

import base64
import binascii
import hashlib
import hmac
import json
import re
import time
import uuid
from dataclasses import dataclass
from typing import Any

from .human_approval import build_signed_human_decision

QR_CONFIRMATION_CONTRACT_VERSION = "flyto.robotics.qr-confirmation.v1"
QR_CONFIRMATION_PURPOSE = "delivery_received"
QR_SIGNATURE_ALGORITHM = "hmac-sha256"
QR_TOKEN_PREFIX = "F2QR1."
MAX_QR_TOKEN_BYTES = 16 * 1024
MAX_QR_CONFIRMATION_TTL_SECONDS = 300
MAX_QR_CLOCK_SKEW_SECONDS = 30
IDENTIFIER_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")
RECIPIENT_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,119}$")
SIGNATURE_PATTERN = re.compile(r"^[0-9a-f]{64}$")
QR_CONFIRMATION_FIELDS = frozenset(
    {
        "contract_version",
        "confirmation_id",
        "purpose",
        "job_id",
        "robot_id",
        "approval_id",
        "recipient_ref",
        "issued_at_epoch_seconds",
        "expires_at_epoch_seconds",
        "nonce",
        "signature_algorithm",
        "signature",
    }
)


class QRConfirmationValidationError(ValueError):
    """Raised when a scanned delivery confirmation is not trustworthy."""


@dataclass(frozen=True)
class VerifiedQRConfirmation:
    """A QR confirmation whose signature, scope, time, and nonce were verified."""

    confirmation_id: str
    job_id: str
    robot_id: str
    approval_id: str
    recipient_ref: str
    issued_at_epoch_seconds: int
    expires_at_epoch_seconds: int
    nonce: str


def _secret_bytes(secret: str | bytes) -> bytes:
    encoded = secret.encode("utf-8") if isinstance(secret, str) else secret
    if not isinstance(encoded, bytes) or len(encoded) < 32:
        raise QRConfirmationValidationError(
            "QR confirmation secret must contain at least 32 bytes"
        )
    return encoded


def _identifier(
    value: object,
    field_name: str,
    *,
    recipient: bool = False,
) -> str:
    pattern = RECIPIENT_PATTERN if recipient else IDENTIFIER_PATTERN
    if not isinstance(value, str) or not pattern.fullmatch(value):
        raise QRConfirmationValidationError(
            f"{field_name} must be a safe identifier"
        )
    return value


def _integer(value: object, field_name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise QRConfirmationValidationError(f"{field_name} must be an integer")
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


def _encode_payload(data: dict[str, Any]) -> str:
    raw = json.dumps(
        data,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    encoded = base64.urlsafe_b64encode(raw).decode("ascii").rstrip("=")
    return f"{QR_TOKEN_PREFIX}{encoded}"


def _decode_payload(token: str) -> dict[str, Any]:
    if not isinstance(token, str):
        raise QRConfirmationValidationError("QR confirmation must be text")
    if len(token.encode("utf-8")) > MAX_QR_TOKEN_BYTES:
        raise QRConfirmationValidationError("QR confirmation is too large")
    if not token.startswith(QR_TOKEN_PREFIX):
        raise QRConfirmationValidationError("QR confirmation prefix is invalid")
    encoded = token[len(QR_TOKEN_PREFIX) :]
    if not encoded:
        raise QRConfirmationValidationError("QR confirmation payload is empty")
    try:
        padding = "=" * (-len(encoded) % 4)
        raw = base64.b64decode(
            encoded + padding,
            altchars=b"-_",
            validate=True,
        )
        decoded = json.loads(raw)
    except (binascii.Error, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise QRConfirmationValidationError(
            "QR confirmation payload is invalid"
        ) from exc
    if not isinstance(decoded, dict) or not all(
        isinstance(key, str) for key in decoded
    ):
        raise QRConfirmationValidationError(
            "QR confirmation payload must be an object"
        )
    return decoded


def build_signed_qr_confirmation(
    *,
    job_id: str,
    robot_id: str,
    approval_id: str,
    recipient_ref: str,
    secret: str | bytes,
    confirmation_id: str | None = None,
    issued_at_epoch_seconds: int | None = None,
    ttl_seconds: int = 120,
    nonce: str | None = None,
) -> str:
    """Build one compact QR token without embedding personal information."""
    issued_at = int(time.time()) if issued_at_epoch_seconds is None else _integer(
        issued_at_epoch_seconds,
        "issued_at_epoch_seconds",
    )
    if isinstance(ttl_seconds, bool) or not isinstance(ttl_seconds, int):
        raise QRConfirmationValidationError("ttl_seconds must be an integer")
    if not 1 <= ttl_seconds <= MAX_QR_CONFIRMATION_TTL_SECONDS:
        raise QRConfirmationValidationError(
            "ttl_seconds must be between 1 and "
            f"{MAX_QR_CONFIRMATION_TTL_SECONDS}"
        )
    data: dict[str, Any] = {
        "contract_version": QR_CONFIRMATION_CONTRACT_VERSION,
        "confirmation_id": _identifier(
            confirmation_id or f"confirm.{uuid.uuid4().hex}",
            "confirmation_id",
        ),
        "purpose": QR_CONFIRMATION_PURPOSE,
        "job_id": _identifier(job_id, "job_id"),
        "robot_id": _identifier(robot_id, "robot_id"),
        "approval_id": _identifier(approval_id, "approval_id"),
        "recipient_ref": _identifier(
            recipient_ref,
            "recipient_ref",
            recipient=True,
        ),
        "issued_at_epoch_seconds": issued_at,
        "expires_at_epoch_seconds": issued_at + ttl_seconds,
        "nonce": _identifier(nonce or uuid.uuid4().hex, "nonce"),
        "signature_algorithm": QR_SIGNATURE_ALGORITHM,
    }
    data["signature"] = _signature(data, secret)
    return _encode_payload(data)


class QRConfirmationAuthenticator:
    """Verify a scoped QR token and consume its nonce exactly once."""

    def __init__(self, secret: str | bytes) -> None:
        self._secret = _secret_bytes(secret)
        self._used_nonces: set[str] = set()

    def verify(
        self,
        token: str,
        *,
        expected_job_id: str,
        expected_robot_id: str,
        expected_approval_id: str,
        expected_recipient_ref: str | None = None,
        now_epoch_seconds: int | None = None,
    ) -> VerifiedQRConfirmation:
        decoded = _decode_payload(token)
        unknown = sorted(set(decoded) - QR_CONFIRMATION_FIELDS)
        missing = sorted(QR_CONFIRMATION_FIELDS - set(decoded))
        if unknown:
            raise QRConfirmationValidationError(
                "QR confirmation contains unsupported fields: "
                + ", ".join(unknown)
            )
        if missing:
            raise QRConfirmationValidationError(
                "QR confirmation is missing: " + ", ".join(missing)
            )
        signature = decoded["signature"]
        if not isinstance(signature, str) or not SIGNATURE_PATTERN.fullmatch(
            signature
        ):
            raise QRConfirmationValidationError(
                "QR confirmation signature must be lowercase SHA-256 hex"
            )
        expected_signature = _signature(decoded, self._secret)
        if not hmac.compare_digest(signature, expected_signature):
            raise QRConfirmationValidationError(
                "QR confirmation signature is invalid"
            )
        if decoded["contract_version"] != QR_CONFIRMATION_CONTRACT_VERSION:
            raise QRConfirmationValidationError(
                f"contract_version must be {QR_CONFIRMATION_CONTRACT_VERSION}"
            )
        if decoded["purpose"] != QR_CONFIRMATION_PURPOSE:
            raise QRConfirmationValidationError(
                f"purpose must be {QR_CONFIRMATION_PURPOSE}"
            )
        if decoded["signature_algorithm"] != QR_SIGNATURE_ALGORITHM:
            raise QRConfirmationValidationError(
                f"signature_algorithm must be {QR_SIGNATURE_ALGORITHM}"
            )

        confirmation_id = _identifier(
            decoded["confirmation_id"],
            "confirmation_id",
        )
        job_id = _identifier(decoded["job_id"], "job_id")
        robot_id = _identifier(decoded["robot_id"], "robot_id")
        approval_id = _identifier(decoded["approval_id"], "approval_id")
        recipient_ref = _identifier(
            decoded["recipient_ref"],
            "recipient_ref",
            recipient=True,
        )
        nonce = _identifier(decoded["nonce"], "nonce")
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
            raise QRConfirmationValidationError(
                "QR confirmation expiry must follow issue time"
            )
        if expires_at - issued_at > MAX_QR_CONFIRMATION_TTL_SECONDS:
            raise QRConfirmationValidationError(
                "QR confirmation lifetime is too long"
            )
        if issued_at > now + MAX_QR_CLOCK_SKEW_SECONDS:
            raise QRConfirmationValidationError(
                "QR confirmation issue time is in the future"
            )
        if expires_at < now:
            raise QRConfirmationValidationError("QR confirmation has expired")
        if job_id != expected_job_id:
            raise QRConfirmationValidationError(
                "QR confirmation job_id does not match"
            )
        if robot_id != expected_robot_id:
            raise QRConfirmationValidationError(
                "QR confirmation robot_id does not match"
            )
        if approval_id != expected_approval_id:
            raise QRConfirmationValidationError(
                "QR confirmation approval_id does not match"
            )
        if (
            expected_recipient_ref is not None
            and recipient_ref != expected_recipient_ref
        ):
            raise QRConfirmationValidationError(
                "QR confirmation recipient_ref does not match"
            )
        if nonce in self._used_nonces:
            raise QRConfirmationValidationError(
                "QR confirmation nonce was already used"
            )
        self._used_nonces.add(nonce)
        return VerifiedQRConfirmation(
            confirmation_id=confirmation_id,
            job_id=job_id,
            robot_id=robot_id,
            approval_id=approval_id,
            recipient_ref=recipient_ref,
            issued_at_epoch_seconds=issued_at,
            expires_at_epoch_seconds=expires_at,
            nonce=nonce,
        )


def qr_confirmation_to_human_decision(
    confirmation: VerifiedQRConfirmation,
    *,
    approval_secret: str | bytes,
    issued_at_epoch_seconds: int | None = None,
) -> dict[str, Any]:
    """Convert verified QR evidence into the existing ROS approval envelope."""
    issued_at = (
        int(time.time())
        if issued_at_epoch_seconds is None
        else _integer(issued_at_epoch_seconds, "issued_at_epoch_seconds")
    )
    remaining_seconds = confirmation.expires_at_epoch_seconds - issued_at
    if remaining_seconds < 1:
        raise QRConfirmationValidationError(
            "QR confirmation expired before approval conversion"
        )
    return build_signed_human_decision(
        job_id=confirmation.job_id,
        robot_id=confirmation.robot_id,
        approval_id=confirmation.approval_id,
        approved=True,
        actor_id=f"qr.{confirmation.recipient_ref}",
        secret=approval_secret,
        issued_at_epoch_seconds=issued_at,
        ttl_seconds=min(60, remaining_seconds),
    )


def qr_token_sha256(token: str) -> str:
    """Return an evidence-safe fingerprint without persisting the QR token."""
    if not isinstance(token, str):
        raise QRConfirmationValidationError("QR confirmation must be text")
    return hashlib.sha256(token.encode("utf-8")).hexdigest()
