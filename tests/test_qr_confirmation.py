from __future__ import annotations

import base64
import json
from pathlib import Path

import pytest

from flyto_robotics.cli import main
from flyto_robotics.human_approval import HumanDecisionAuthenticator
from flyto_robotics.qr_confirmation import (
    QRConfirmationAuthenticator,
    QRConfirmationValidationError,
    build_signed_qr_confirmation,
    qr_confirmation_to_human_decision,
    qr_token_sha256,
)

QR_SECRET = "test-only-qr-secret-with-at-least-32-bytes"
APPROVAL_SECRET = "test-only-approval-secret-with-at-least-32-bytes"
ROOT = Path(__file__).resolve().parents[1]


def build_token(**overrides: object) -> str:
    values: dict[str, object] = {
        "job_id": "job.delivery.001",
        "robot_id": "robot.sim.001",
        "approval_id": "delivery.handoff",
        "recipient_ref": "ward-b.receiver",
        "secret": QR_SECRET,
        "confirmation_id": "confirm.test.001",
        "issued_at_epoch_seconds": 1_000,
        "ttl_seconds": 120,
        "nonce": "qr-nonce-001",
    }
    values.update(overrides)
    return build_signed_qr_confirmation(**values)


def tamper(token: str, field: str, value: object) -> str:
    prefix, encoded = token.split(".", 1)
    padding = "=" * (-len(encoded) % 4)
    payload = json.loads(
        base64.urlsafe_b64decode(encoded + padding).decode("utf-8")
    )
    payload[field] = value
    altered = base64.urlsafe_b64encode(
        json.dumps(
            payload,
            separators=(",", ":"),
            sort_keys=True,
        ).encode()
    ).decode("ascii").rstrip("=")
    return f"{prefix}.{altered}"


def verify(token: str, *, now: int = 1_030):
    return QRConfirmationAuthenticator(QR_SECRET).verify(
        token,
        expected_job_id="job.delivery.001",
        expected_robot_id="robot.sim.001",
        expected_approval_id="delivery.handoff",
        expected_recipient_ref="ward-b.receiver",
        now_epoch_seconds=now,
    )


def test_signed_qr_round_trip_and_evidence_hash() -> None:
    token = build_token()
    confirmation = verify(token)

    assert token.startswith("F2QR1.")
    assert confirmation.confirmation_id == "confirm.test.001"
    assert confirmation.recipient_ref == "ward-b.receiver"
    assert len(qr_token_sha256(token)) == 64
    assert token not in qr_token_sha256(token)


def test_tampered_qr_is_rejected() -> None:
    with pytest.raises(
        QRConfirmationValidationError,
        match="signature is invalid",
    ):
        verify(tamper(build_token(), "recipient_ref", "ward-c.receiver"))


@pytest.mark.parametrize(
    ("overrides", "now", "message"),
    [
        ({"job_id": "job.other"}, 1_030, "job_id does not match"),
        ({"robot_id": "robot.other"}, 1_030, "robot_id does not match"),
        ({"approval_id": "other.gate"}, 1_030, "approval_id does not match"),
        ({"ttl_seconds": 10}, 1_011, "has expired"),
    ],
)
def test_qr_scope_and_expiry_fail_closed(
    overrides: dict[str, object],
    now: int,
    message: str,
) -> None:
    with pytest.raises(QRConfirmationValidationError, match=message):
        verify(build_token(**overrides), now=now)


def test_qr_nonce_is_single_use() -> None:
    authenticator = QRConfirmationAuthenticator(QR_SECRET)
    token = build_token()
    kwargs = {
        "expected_job_id": "job.delivery.001",
        "expected_robot_id": "robot.sim.001",
        "expected_approval_id": "delivery.handoff",
        "now_epoch_seconds": 1_030,
    }

    authenticator.verify(token, **kwargs)
    with pytest.raises(
        QRConfirmationValidationError,
        match="nonce was already used",
    ):
        authenticator.verify(token, **kwargs)


def test_verified_qr_converts_to_existing_replay_resistant_human_gate() -> None:
    confirmation = verify(build_token())
    decision = qr_confirmation_to_human_decision(
        confirmation,
        approval_secret=APPROVAL_SECRET,
        issued_at_epoch_seconds=1_031,
    )
    authenticator = HumanDecisionAuthenticator(APPROVAL_SECRET)
    verified = authenticator.verify(
        decision,
        expected_job_id="job.delivery.001",
        expected_robot_id="robot.sim.001",
        now_epoch_seconds=1_032,
    )

    assert verified.approved is True
    assert verified.approval_id == "delivery.handoff"
    assert verified.actor_id == "qr.ward-b.receiver"
    with pytest.raises(ValueError, match="nonce was already used"):
        authenticator.verify(
            decision,
            expected_job_id="job.delivery.001",
            expected_robot_id="robot.sim.001",
            now_epoch_seconds=1_032,
        )


def test_cli_builds_and_verifies_delivery_qr_without_persisting_secrets(
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
    capsys: pytest.CaptureFixture[str],
) -> None:
    monkeypatch.setenv("FLYTO_ROBOTICS_QR_SECRET", QR_SECRET)
    monkeypatch.setenv("FLYTO_ROBOTICS_APPROVAL_SECRET", APPROVAL_SECRET)
    token_file = tmp_path / "delivery-qr.txt"
    result_file = tmp_path / "verified.json"
    job_file = ROOT / "examples/jobs/pharmacy-to-ward.json"

    assert (
        main(
            [
                "sign-delivery-qr",
                "--job",
                str(job_file),
                "--approval-id",
                "ward-b-receipt",
                "--recipient-ref",
                "ward-b.receiver",
                "--output",
                str(token_file),
            ]
        )
        == 0
    )
    capsys.readouterr()
    assert (
        main(
            [
                "verify-delivery-qr",
                "--job",
                str(job_file),
                "--approval-id",
                "ward-b-receipt",
                "--recipient-ref",
                "ward-b.receiver",
                "--token-file",
                str(token_file),
                "--output",
                str(result_file),
            ]
        )
        == 0
    )
    output = json.loads(result_file.read_text(encoding="utf-8"))

    assert output["ok"] is True
    assert len(output["confirmation"]["token_sha256"]) == 64
    assert output["human_decision"]["actor_id"] == "qr.ward-b.receiver"
    persisted = result_file.read_text(encoding="utf-8")
    assert QR_SECRET not in persisted
    assert APPROVAL_SECRET not in persisted
    assert token_file.read_text(encoding="utf-8").strip() not in persisted
