"""Deterministic, replayable gates for handing a payload to a recipient."""

from __future__ import annotations

import argparse
import hashlib
import json
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any

from .contracts import write_json_atomic

POLICY_CONTRACT = "flyto.robotics.guarded-handoff-policy.v1"
SCRIPT_CONTRACT = "flyto.robotics.guarded-handoff-script.v1"
EVIDENCE_CONTRACT = "flyto.robotics.guarded-handoff-evidence.v1"
MAX_DOCUMENT_BYTES = 256 * 1024
MAX_ACTIONS = 64


class GuardedHandoffValidationError(ValueError):
    """Raised when a handoff contract or state transition fails closed."""


class HandoffState(str, Enum):
    AWAITING_PRECONDITIONS = "awaiting_preconditions"
    AWAITING_ITEM = "awaiting_item"
    BLOCKED_ITEM = "blocked_item"
    CHECKPOINT_READY = "checkpoint_ready"
    AWAITING_RECIPIENT = "awaiting_recipient"
    READY_TO_UNLOCK = "ready_to_unlock"
    UNLOCKED = "unlocked"
    COMPLETED = "completed"


@dataclass(frozen=True)
class GuardedHandoffPolicy:
    policy_id: str
    container_id: str
    expected_item_ref: str
    expected_recipient_ref: str
    required_preconditions: tuple[tuple[str, str], ...]
    synthetic: bool

    @classmethod
    def from_mapping(cls, value: object) -> GuardedHandoffPolicy:
        mapping = _object(value, "policy")
        _exact_keys(
            mapping,
            {
                "contract_version",
                "policy_id",
                "container_id",
                "expected_item_ref",
                "expected_recipient_ref",
                "required_preconditions",
                "synthetic",
            },
            "policy",
        )
        if mapping.get("contract_version") != POLICY_CONTRACT:
            raise GuardedHandoffValidationError("unsupported policy contract")
        preconditions = _object(
            mapping.get("required_preconditions"), "required_preconditions"
        )
        if not preconditions:
            raise GuardedHandoffValidationError(
                "required_preconditions must not be empty"
            )
        normalized_preconditions = tuple(
            sorted(
                (
                    _identifier(name, "precondition name"),
                    _bounded_text(expected, f"precondition {name}"),
                )
                for name, expected in preconditions.items()
            )
        )
        synthetic = mapping.get("synthetic")
        if not isinstance(synthetic, bool):
            raise GuardedHandoffValidationError("synthetic must be a boolean")
        return cls(
            policy_id=_identifier(mapping.get("policy_id"), "policy_id"),
            container_id=_identifier(mapping.get("container_id"), "container_id"),
            expected_item_ref=_bounded_text(
                mapping.get("expected_item_ref"), "expected_item_ref"
            ),
            expected_recipient_ref=_bounded_text(
                mapping.get("expected_recipient_ref"), "expected_recipient_ref"
            ),
            required_preconditions=normalized_preconditions,
            synthetic=synthetic,
        )


@dataclass(frozen=True)
class HandoffAction:
    action: str
    name: str | None = None
    value: str | None = None
    observed_ref: str | None = None
    actor_id: str | None = None

    @classmethod
    def from_mapping(cls, value: object, index: int) -> HandoffAction:
        mapping = _object(value, f"actions[{index}]")
        allowed = {"action", "name", "value", "observed_ref", "actor_id"}
        if set(mapping) - allowed:
            raise GuardedHandoffValidationError(
                f"actions[{index}] contains unknown fields"
            )
        action = _identifier(mapping.get("action"), f"actions[{index}].action")
        if action not in {
            "check_precondition",
            "scan_item",
            "resume_checkpoint",
            "scan_recipient",
            "unlock_container",
            "complete",
        }:
            raise GuardedHandoffValidationError(
                f"actions[{index}] has unsupported action {action}"
            )
        fields = {
            key: (
                _bounded_text(mapping[key], f"actions[{index}].{key}")
                if key in mapping
                else None
            )
            for key in ("name", "value", "observed_ref", "actor_id")
        }
        required_by_action = {
            "check_precondition": ("name", "value"),
            "scan_item": ("observed_ref",),
            "resume_checkpoint": ("actor_id",),
            "scan_recipient": ("observed_ref",),
            "unlock_container": ("actor_id",),
            "complete": (),
        }
        for required in required_by_action[action]:
            if fields[required] is None:
                raise GuardedHandoffValidationError(
                    f"actions[{index}].{required} is required"
                )
        return cls(action=action, **fields)


@dataclass(frozen=True)
class GuardedHandoffScript:
    script_id: str
    policy_id: str
    actions: tuple[HandoffAction, ...]

    @classmethod
    def from_mapping(cls, value: object) -> GuardedHandoffScript:
        mapping = _object(value, "script")
        _exact_keys(
            mapping,
            {"contract_version", "script_id", "policy_id", "actions"},
            "script",
        )
        if mapping.get("contract_version") != SCRIPT_CONTRACT:
            raise GuardedHandoffValidationError("unsupported script contract")
        raw_actions = mapping.get("actions")
        if not isinstance(raw_actions, list) or not 1 <= len(raw_actions) <= MAX_ACTIONS:
            raise GuardedHandoffValidationError(
                f"actions must contain between 1 and {MAX_ACTIONS} entries"
            )
        return cls(
            script_id=_identifier(mapping.get("script_id"), "script_id"),
            policy_id=_identifier(mapping.get("policy_id"), "policy_id"),
            actions=tuple(
                HandoffAction.from_mapping(action, index)
                for index, action in enumerate(raw_actions)
            ),
        )


class GuardedHandoffSession:
    """Apply small deterministic gates while keeping the container fail-closed."""

    def __init__(
        self,
        policy: GuardedHandoffPolicy,
        *,
        session_id: str,
    ) -> None:
        self.policy = policy
        self.session_id = _identifier(session_id, "session_id")
        self.state = HandoffState.AWAITING_PRECONDITIONS
        self.container_locked = True
        self.checkpoint: str | None = None
        self.verified_preconditions: set[str] = set()
        self.item_verified = False
        self.recipient_verified = False
        self.events: list[dict[str, object]] = []
        self._record("handoff_started", "guarded handoff session started")

    @property
    def terminal(self) -> bool:
        return self.state is HandoffState.COMPLETED

    def apply(self, action: HandoffAction) -> dict[str, object]:
        if self.terminal:
            raise GuardedHandoffValidationError(
                "completed handoff rejects additional actions"
            )
        handlers = {
            "check_precondition": self._check_precondition,
            "scan_item": self._scan_item,
            "resume_checkpoint": self._resume_checkpoint,
            "scan_recipient": self._scan_recipient,
            "unlock_container": self._unlock_container,
            "complete": self._complete,
        }
        before = len(self.events)
        handlers[action.action](action)
        if len(self.events) != before + 1:
            raise RuntimeError("each guarded handoff atom must emit exactly one event")
        return self.events[-1]

    def evidence(self) -> dict[str, object]:
        return {
            "contract_version": EVIDENCE_CONTRACT,
            "session_id": self.session_id,
            "policy_id": self.policy.policy_id,
            "state": self.state.value,
            "container_id": self.policy.container_id,
            "container_locked": self.container_locked,
            "checkpoint": self.checkpoint,
            "preconditions_verified": sorted(self.verified_preconditions),
            "item_verified": self.item_verified,
            "recipient_verified": self.recipient_verified,
            "synthetic": self.policy.synthetic,
            "events": list(self.events),
        }

    def _check_precondition(self, action: HandoffAction) -> None:
        if self.state is not HandoffState.AWAITING_PRECONDITIONS:
            raise GuardedHandoffValidationError(
                "preconditions can only be checked before item verification"
            )
        expected = dict(self.policy.required_preconditions).get(str(action.name))
        if expected is None:
            raise GuardedHandoffValidationError("undeclared precondition rejected")
        if action.value != expected:
            self._record(
                "precondition_rejected",
                f"{action.name} did not satisfy the declared requirement",
                condition=str(action.name),
                expected=expected,
                actual=str(action.value),
            )
            return
        self.verified_preconditions.add(str(action.name))
        if self.verified_preconditions == {
            name for name, _expected in self.policy.required_preconditions
        }:
            self.state = HandoffState.AWAITING_ITEM
        self._record(
            "precondition_verified",
            f"{action.name} satisfied",
            condition=str(action.name),
            actual=str(action.value),
        )

    def _scan_item(self, action: HandoffAction) -> None:
        if self.state not in {
            HandoffState.AWAITING_ITEM,
            HandoffState.BLOCKED_ITEM,
        }:
            raise GuardedHandoffValidationError(
                "item scan is not allowed in the current state"
            )
        observed = str(action.observed_ref)
        if observed != self.policy.expected_item_ref:
            self.state = HandoffState.BLOCKED_ITEM
            self.item_verified = False
            self.checkpoint = "verify_item"
            self._record(
                "item_rejected",
                "payload identifier mismatch; container remains locked",
                expected=self._evidence_ref(self.policy.expected_item_ref),
                actual=self._evidence_ref(observed),
            )
            return
        self.item_verified = True
        self.state = (
            HandoffState.CHECKPOINT_READY
            if self.checkpoint == "verify_item"
            else HandoffState.AWAITING_RECIPIENT
        )
        self._record(
            "item_verified",
            "payload identifier matched the policy",
            actual=self._evidence_ref(observed),
        )

    def _resume_checkpoint(self, action: HandoffAction) -> None:
        if (
            self.state is not HandoffState.CHECKPOINT_READY
            or not self.item_verified
            or self.checkpoint != "verify_item"
        ):
            raise GuardedHandoffValidationError(
                "checkpoint resume requires a newly verified item"
            )
        self.state = HandoffState.AWAITING_RECIPIENT
        checkpoint = self.checkpoint
        self.checkpoint = None
        self._record(
            "checkpoint_resumed",
            "operator resumed from the verified item checkpoint",
            checkpoint=checkpoint,
            actor_id=str(action.actor_id),
        )

    def _scan_recipient(self, action: HandoffAction) -> None:
        if self.state not in {
            HandoffState.AWAITING_RECIPIENT,
            HandoffState.READY_TO_UNLOCK,
        }:
            raise GuardedHandoffValidationError(
                "recipient scan is not allowed before item verification"
            )
        observed = str(action.observed_ref)
        if observed != self.policy.expected_recipient_ref:
            self.recipient_verified = False
            self.state = HandoffState.AWAITING_RECIPIENT
            self._record(
                "recipient_rejected",
                "recipient identifier mismatch; container remains locked",
                expected=self._evidence_ref(self.policy.expected_recipient_ref),
                actual=self._evidence_ref(observed),
            )
            return
        self.recipient_verified = True
        self.state = HandoffState.READY_TO_UNLOCK
        self._record(
            "recipient_verified",
            "recipient identifier matched the task",
            actual=self._evidence_ref(observed),
        )

    def _unlock_container(self, action: HandoffAction) -> None:
        all_preconditions = {
            name for name, _expected in self.policy.required_preconditions
        }
        if (
            self.state is not HandoffState.READY_TO_UNLOCK
            or self.verified_preconditions != all_preconditions
            or not self.item_verified
            or not self.recipient_verified
        ):
            raise GuardedHandoffValidationError(
                "container unlock requires all declared gates"
            )
        self.container_locked = False
        self.state = HandoffState.UNLOCKED
        self._record(
            "container_unlocked",
            "all declared gates passed; unlock authorized",
            actor_id=str(action.actor_id),
        )

    def _complete(self, _action: HandoffAction) -> None:
        if self.state is not HandoffState.UNLOCKED or self.container_locked:
            raise GuardedHandoffValidationError(
                "handoff cannot complete while the container is locked"
            )
        self.state = HandoffState.COMPLETED
        self._record("handoff_completed", "guarded handoff completed")

    def _record(
        self,
        kind: str,
        detail: str,
        **fields: object,
    ) -> None:
        event: dict[str, object] = {
            "sequence": len(self.events) + 1,
            "kind": kind,
            "detail": detail,
            "state": self.state.value,
            "container_locked": self.container_locked,
        }
        event.update(fields)
        self.events.append(event)

    def _evidence_ref(self, value: str) -> str:
        if self.policy.synthetic:
            return value
        return f"sha256:{hashlib.sha256(value.encode('utf-8')).hexdigest()}"


def load_policy(path: str | Path) -> GuardedHandoffPolicy:
    return GuardedHandoffPolicy.from_mapping(_load_json(path))


def load_script(path: str | Path) -> GuardedHandoffScript:
    return GuardedHandoffScript.from_mapping(_load_json(path))


def run_scripted_handoff(
    policy: GuardedHandoffPolicy,
    script: GuardedHandoffScript,
) -> dict[str, object]:
    if script.policy_id != policy.policy_id:
        raise GuardedHandoffValidationError(
            "script policy_id does not match the selected policy"
        )
    session = GuardedHandoffSession(policy, session_id=script.script_id)
    for action in script.actions:
        session.apply(action)
    if not session.terminal:
        raise GuardedHandoffValidationError(
            "script ended before the guarded handoff completed"
        )
    return session.evidence()


def _load_json(path: str | Path) -> object:
    source = Path(path)
    if source.stat().st_size > MAX_DOCUMENT_BYTES:
        raise GuardedHandoffValidationError("handoff document is too large")
    try:
        return json.loads(source.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise GuardedHandoffValidationError(
            f"unable to read handoff document: {exc}"
        ) from exc


def _object(value: object, field: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise GuardedHandoffValidationError(f"{field} must be an object")
    return value


def _exact_keys(value: dict[str, Any], expected: set[str], field: str) -> None:
    if set(value) != expected:
        raise GuardedHandoffValidationError(
            f"{field} fields must exactly match {sorted(expected)}"
        )


def _identifier(value: object, field: str) -> str:
    text = _bounded_text(value, field)
    if not all(character.isalnum() or character in "._-" for character in text):
        raise GuardedHandoffValidationError(
            f"{field} may only contain letters, numbers, dot, dash, underscore"
        )
    return text


def _bounded_text(value: object, field: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 128:
        raise GuardedHandoffValidationError(
            f"{field} must be a string between 1 and 128 characters"
        )
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run one deterministic guarded handoff script"
    )
    parser.add_argument("--policy", required=True)
    parser.add_argument("--script", required=True)
    parser.add_argument("--output", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    evidence = run_scripted_handoff(
        load_policy(args.policy),
        load_script(args.script),
    )
    write_json_atomic(args.output, evidence)
    print(json.dumps(evidence, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
