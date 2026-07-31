from __future__ import annotations

import json
from pathlib import Path

import pytest

from flyto_robotics.guarded_handoff import (
    EVIDENCE_CONTRACT,
    GuardedHandoffPolicy,
    GuardedHandoffScript,
    GuardedHandoffSession,
    GuardedHandoffValidationError,
    HandoffAction,
    HandoffState,
    load_policy,
    load_script,
    main,
    run_scripted_handoff,
)

ROOT = Path(__file__).resolve().parents[1]
POLICY_FILE = ROOT / "examples/guarded-handoff/medication-policy.json"
SCRIPT_FILE = ROOT / "examples/guarded-handoff/medication-script.json"


def policy(*, synthetic: bool = True) -> GuardedHandoffPolicy:
    return GuardedHandoffPolicy.from_mapping(
        {
            "contract_version": "flyto.robotics.guarded-handoff-policy.v1",
            "policy_id": "test.policy.v1",
            "container_id": "box-01",
            "expected_item_ref": "item-A",
            "expected_recipient_ref": "recipient-12",
            "required_preconditions": {"billing_status": "completed"},
            "synthetic": synthetic,
        }
    )


def action(action_name: str, **fields: str) -> HandoffAction:
    return HandoffAction.from_mapping({"action": action_name, **fields}, 0)


def ready_for_item(session: GuardedHandoffSession) -> None:
    session.apply(
        action(
            "check_precondition",
            name="billing_status",
            value="completed",
        )
    )


def test_example_script_blocks_wrong_item_and_recipient_before_unlock() -> None:
    evidence = run_scripted_handoff(load_policy(POLICY_FILE), load_script(SCRIPT_FILE))

    assert evidence["contract_version"] == EVIDENCE_CONTRACT
    assert evidence["state"] == HandoffState.COMPLETED.value
    assert evidence["container_locked"] is False
    kinds = [event["kind"] for event in evidence["events"]]
    assert kinds == [
        "handoff_started",
        "precondition_verified",
        "item_rejected",
        "item_verified",
        "checkpoint_resumed",
        "recipient_rejected",
        "recipient_verified",
        "container_unlocked",
        "handoff_completed",
    ]
    rejected_item = next(
        event for event in evidence["events"] if event["kind"] == "item_rejected"
    )
    assert rejected_item["expected"] == "A12"
    assert rejected_item["actual"] == "B13"
    assert rejected_item["container_locked"] is True
    rejected_recipient = next(
        event
        for event in evidence["events"]
        if event["kind"] == "recipient_rejected"
    )
    assert rejected_recipient["expected"] == "patient-12"
    assert rejected_recipient["actual"] == "patient-13"
    assert rejected_recipient["container_locked"] is True


def test_wrong_precondition_keeps_container_locked() -> None:
    session = GuardedHandoffSession(policy(), session_id="session-01")

    event = session.apply(
        action("check_precondition", name="billing_status", value="pending")
    )

    assert event["kind"] == "precondition_rejected"
    assert session.state is HandoffState.AWAITING_PRECONDITIONS
    assert session.container_locked is True


def test_checkpoint_cannot_resume_until_correct_item_is_verified() -> None:
    session = GuardedHandoffSession(policy(), session_id="session-01")
    ready_for_item(session)
    session.apply(action("scan_item", observed_ref="item-B"))

    with pytest.raises(
        GuardedHandoffValidationError,
        match="newly verified item",
    ):
        session.apply(
            action("resume_checkpoint", actor_id="operator.synthetic-01")
        )

    assert session.state is HandoffState.BLOCKED_ITEM
    assert session.container_locked is True


def test_container_cannot_unlock_before_recipient_verification() -> None:
    session = GuardedHandoffSession(policy(), session_id="session-01")
    ready_for_item(session)
    session.apply(action("scan_item", observed_ref="item-A"))

    with pytest.raises(
        GuardedHandoffValidationError,
        match="all declared gates",
    ):
        session.apply(
            action("unlock_container", actor_id="robot.box-controller")
        )

    assert session.container_locked is True


def test_wrong_recipient_can_be_retried_without_unlocking() -> None:
    session = GuardedHandoffSession(policy(), session_id="session-01")
    ready_for_item(session)
    session.apply(action("scan_item", observed_ref="item-A"))

    rejected = session.apply(
        action("scan_recipient", observed_ref="recipient-13")
    )
    verified = session.apply(
        action("scan_recipient", observed_ref="recipient-12")
    )

    assert rejected["kind"] == "recipient_rejected"
    assert rejected["container_locked"] is True
    assert verified["kind"] == "recipient_verified"
    assert session.state is HandoffState.READY_TO_UNLOCK
    assert session.container_locked is True


def test_non_synthetic_evidence_hashes_item_and_recipient_references() -> None:
    session = GuardedHandoffSession(
        policy(synthetic=False),
        session_id="session-01",
    )
    ready_for_item(session)
    item_event = session.apply(action("scan_item", observed_ref="item-A"))
    recipient_event = session.apply(
        action("scan_recipient", observed_ref="recipient-12")
    )

    assert str(item_event["actual"]).startswith("sha256:")
    assert str(recipient_event["actual"]).startswith("sha256:")
    serialized = json.dumps(session.evidence(), ensure_ascii=False)
    assert "item-A" not in serialized
    assert "recipient-12" not in serialized


def test_script_policy_mismatch_is_rejected() -> None:
    script = GuardedHandoffScript.from_mapping(
        {
            "contract_version": "flyto.robotics.guarded-handoff-script.v1",
            "script_id": "script-01",
            "policy_id": "another-policy",
            "actions": [{"action": "complete"}],
        }
    )

    with pytest.raises(GuardedHandoffValidationError, match="does not match"):
        run_scripted_handoff(policy(), script)


def test_cli_writes_replayable_evidence(tmp_path: Path) -> None:
    output = tmp_path / "handoff-evidence.json"

    assert (
        main(
            [
                "--policy",
                str(POLICY_FILE),
                "--script",
                str(SCRIPT_FILE),
                "--output",
                str(output),
            ]
        )
        == 0
    )
    evidence = json.loads(output.read_text(encoding="utf-8"))
    assert evidence["state"] == "completed"
    assert [event["sequence"] for event in evidence["events"]] == list(
        range(1, 10)
    )
