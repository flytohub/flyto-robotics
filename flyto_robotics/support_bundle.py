"""Deterministic, privacy-redacted support bundle.

A support bundle is evidence someone will email. It therefore has two hard
properties, and they pull against each other:

* **Deterministic.** Two runs over the same device state produce byte-identical
  output. Nothing here reads the clock, the process table, or a random source;
  ``now`` is a parameter. Without this, "the bundle changed" carries no
  information and no test can assert what the bundle contains.
* **Redacted by construction.** Redaction is applied to every value on the way
  in, not to a hand-listed set of fields. A field added later is redacted by
  default -- the opposite arrangement leaks the first time someone adds a key
  and forgets, and by then the bundle is already in a mailbox.

The bundle carries reason and action codes, release history, unit *names* and
their validation defects, and file inventories. It never carries credential
bytes, raw journals, SSIDs, addresses, or a device secret.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from typing import Any

from .fsio import atomic_write
from .health_codes import action_for, describe, is_known
from .lifecycle import Layout, LifecycleError, activation_window_evidence, status
from .lifecycle_profiles import ProfileError
from .systemd_units import validate_unit

__all__ = [
    "NOTE_MAX_LENGTH",
    "NOTE_POLICY",
    "NOTE_REJECTED",
    "NoteRejected",
    "REDACTED",
    "SUPPORT_BUNDLE_VERSION",
    "build_support_bundle",
    "check_note",
    "is_sensitive_key",
    "redact",
    "sanitize_note",
    "write_support_bundle",
]

SUPPORT_BUNDLE_VERSION = "flyto.support-bundle.v1"
REDACTED = "[redacted]"

_SENSITIVE_KEY = re.compile(
    r"(secret|password|passwd|token|credential|cred|apikey|api_key|private|"
    r"passphrase|ssid|bssid|psk|wifi|wpa|session|cookie|auth|signature|nonce|"
    r"patient|mrn|dob|birth|username|user_name|login|home_dir|homedir)",
    re.IGNORECASE,
)

# Value-shaped redaction. These fire even under a harmless-looking key, because
# the leak that matters is the one nobody labelled.
_VALUE_PATTERNS: tuple[tuple[re.Pattern[str], str], ...] = (
    (re.compile(r"\b(?:\d{1,3}\.){3}\d{1,3}\b"), "[ipv4]"),
    (re.compile(r"\b(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}\b"), "[mac]"),
    (re.compile(r"\b[\w.+-]+@[\w-]+\.[\w.-]+\b"), "[email]"),
    (re.compile(r"(?<=//)[^/\s:@]+:[^/\s@]+(?=@)"), "[userinfo]"),
    (re.compile(r"\b[A-Za-z0-9_-]{40,}\b"), "[opaque]"),
    (re.compile(r"/home/[^/\s:]+"), "/home/[user]"),
    (re.compile(r"/Users/[^/\s:]+"), "/Users/[user]"),
)


#: A bundle promises "no patient data". Free text cannot be made to keep that
#: promise: no pattern removes a name, a ward, or a sentence about a person's
#: condition. So the note is not free text -- it is a reference to something
#: that lives in the ticketing system, where access is controlled.
NOTE_POLICY = (
    "reference only: the note must carry a ticket or reference identifier such as "
    "FLY-1234, followed at most by a short mechanical description. Anything else is "
    "dropped. Never describe a person, a patient, a ward, a bed, or a location."
)
NOTE_MAX_LENGTH = 120
_NOTE_ALLOWED = re.compile(r"^[A-Za-z0-9 ._:#/@=+-]*$")
NOTE_REJECTED = "[note dropped: outside note policy]"

#: The note has to *point* at the ticketing system rather than stand in for it.
#: A character allowlist alone does not do that: "patient Jane Doe in ward 3
#: collapsed near the robot" is spelled entirely in allowed characters, so the
#: only thing that separates a reference from a clinical narrative is the
#: presence of an identifier that resolves somewhere access-controlled.
#: ``FLY-1234``, ``OPS-77``, ``INC-1024`` all match; a bare sentence does not.
_NOTE_REFERENCE = re.compile(r"\b[A-Z][A-Z0-9]{1,15}-\d{1,9}\b")

#: Vocabulary that only shows up when someone is describing a person or the
#: place a person is in. Refused even next to a valid identifier: a ticket id
#: does not make free text about a patient shippable, and the bundle's whole
#: promise is that it carries none. Deliberately narrow -- these words have no
#: role in a mechanical description of a robot -- so it cannot collide with
#: product vocabulary an operator legitimately needs to quote.
_NOTE_NARRATIVE = re.compile(
    r"\b(patients?|wards?|beds?|bedside|nurses?|clinicians?|physicians?|"
    r"surgeons?|residents?|visitors?|relatives?|mr|mrs|ms|miss|dr|"
    r"admitted|discharged)\b",
    re.IGNORECASE,
)


class NoteRejected(ValueError):
    """The supplied note is outside the note policy."""

    reason = "note_rejected"


def sanitize_note(note: str) -> str:
    """Return ``note`` if it is a reference, or a marker saying it was dropped.

    Fails closed and *visibly*: a silently swallowed note would leave the person
    reading the bundle believing the operator said nothing, when in fact they
    said something that could not be shipped.
    """

    note = (note or "").strip()
    if not note:
        return ""
    if len(note) > NOTE_MAX_LENGTH or not _NOTE_ALLOWED.fullmatch(note):
        return NOTE_REJECTED
    if not _NOTE_REFERENCE.search(note) or _NOTE_NARRATIVE.search(note):
        return NOTE_REJECTED
    # A reference can still be shaped like a secret or an address; the ordinary
    # value redaction applies to it exactly as it does to everything else.
    return _redact_text(note)


def check_note(note: str) -> str:
    """Like :func:`sanitize_note`, but raises so a CLI can refuse loudly."""

    result = sanitize_note(note)
    if result == NOTE_REJECTED:
        raise NoteRejected(
            f"notes are limited to {NOTE_MAX_LENGTH} characters of "
            f"[A-Za-z0-9 ._:#/@=+-] and must carry a reference identifier such as "
            f"FLY-1234; {NOTE_POLICY}"
        )
    return result


def is_sensitive_key(name: str) -> bool:
    return bool(_SENSITIVE_KEY.search(name))


def _redact_text(value: str) -> str:
    for pattern, replacement in _VALUE_PATTERNS:
        value = pattern.sub(replacement, value)
    return value


def redact(value: Any, *, key: str = "") -> Any:
    """Recursively redact ``value``.

    A sensitive *key* removes the value outright rather than masking part of it:
    a partially masked secret still tells an attacker its length and alphabet.
    """

    if key and is_sensitive_key(key):
        return REDACTED
    if isinstance(value, dict):
        return {str(name): redact(item, key=str(name)) for name, item in sorted(value.items())}
    if isinstance(value, (list, tuple)):
        return [redact(item) for item in value]
    if isinstance(value, str):
        return _redact_text(value)
    if isinstance(value, (int, float, bool)) or value is None:
        return value
    return _redact_text(str(value))


def _inventory(root: Path, *, limit: int = 200) -> list[dict[str, Any]]:
    """Names and sizes, never contents.

    A credential directory's *existence and mode* are diagnostic; its bytes are
    the thing we are protecting. Listing is sorted so the bundle is stable.
    """

    if not root.is_dir():
        return []
    entries: list[dict[str, Any]] = []
    for path in sorted(root.rglob("*")):
        if len(entries) >= limit:
            entries.append({"truncated": True, "note": f"more than {limit} entries"})
            break
        try:
            stat = path.stat()
        except OSError:
            continue
        entries.append(
            {
                "path": path.relative_to(root).as_posix(),
                "kind": "dir" if path.is_dir() else "file",
                "mode": oct(stat.st_mode & 0o777),
                "bytes": 0 if path.is_dir() else stat.st_size,
            }
        )
    return entries


#: What a lifecycle refusal is allowed to say in a bundle. The bundle carries a
#: *code*, and the code is the whole message: it is what the runbook indexes and
#: what a responder acts on. The exception's own text is deliberately dropped --
#: it quotes the state file, and the state file is attacker-controlled on exactly
#: the devices this path exists for, so echoing it would let corrupted bytes
#: choose what a support bundle says and would carry whatever a tamperer put
#: there into a mailbox.
_LIFECYCLE_FAILURE_DETAIL = (
    "the lifecycle refused to report; see reason and action_code. The refusal text is "
    "deliberately not carried: it quotes on-device state that is untrusted here."
)


def _lifecycle_failure_reason(error: BaseException) -> str:
    """Map a refusal to a stable, already-published reason code.

    Codes are the contract, so nothing is minted here. Anything without one
    fails closed to ``unexpected_error``, whose action is "collect a bundle" --
    which the operator has, by definition, just done.
    """

    reason = getattr(error, "reason", "")
    if isinstance(reason, str) and reason:
        return reason
    if isinstance(error, ProfileError):
        return "profiles_invalid"
    if isinstance(error, OSError):
        return "io_failed"
    if isinstance(error, ValueError):
        return "config_unreadable"
    return "unexpected_error"


def _unhealthy_status_reason(lifecycle: dict[str, Any]) -> str:
    """The status report's own refusal, when it made one.

    Only a *published* code is promoted. A report that says ``ok`` is not
    promoted, and neither is a reason this build does not publish -- minting a
    top-level code out of an unknown string would put something in the field a
    runbook cannot index, which is the one thing these codes exist to prevent.
    """

    if lifecycle.get("ok", True):
        return ""
    reason = lifecycle.get("reason")
    if isinstance(reason, str) and reason and reason != "ok" and is_known(reason):
        return reason
    return ""


def _lifecycle_section(
    layout: Layout, *, systemd: Any, profiles: Any
) -> tuple[dict[str, Any], str | None]:
    """Ask the lifecycle for status, and survive it refusing.

    :func:`~flyto_robotics.lifecycle.status` fails closed: a state file that does
    not verify, or a committed activation whose immutable record is missing or
    altered, is a refusal rather than a confident half-answer. That is right for
    status and wrong for a bundle -- a device whose lifecycle cannot be read is
    the device someone is opening a ticket about, and a collection command that
    raised there would be unusable in the one situation it exists for.

    So the refusal is turned into content: a reason, its action, and a flag that
    says the rest of the section is absent because it could not be read rather
    than because there is nothing to read.
    """

    try:
        return status(layout, systemd=systemd, profiles=profiles), None
    except (LifecycleError, ProfileError, OSError, ValueError) as error:
        reason = _lifecycle_failure_reason(error)
        return (
            {
                "reason": reason,
                "action_code": action_for(reason),
                "detail": _LIFECYCLE_FAILURE_DETAIL,
            },
            reason,
        )


def build_support_bundle(
    layout: Layout,
    *,
    now: str,
    reason: str = "ok",
    note: str = "",
    systemd: Any = None,
    profiles: Any = None,
) -> dict[str, Any]:
    """Assemble the bundle. ``now`` is supplied so the result is reproducible."""

    lifecycle, lifecycle_failure = _lifecycle_section(layout, systemd=systemd, profiles=profiles)

    units: list[dict[str, Any]] = []
    if layout.unit_dir.is_dir():
        for path in sorted(layout.unit_dir.glob("flyto-*")):
            defects = validate_unit(path.read_text(encoding="utf-8"), name=path.name)
            units.append(
                {
                    "name": path.name,
                    "ok": not defects,
                    "defects": [defect.as_dict() for defect in defects],
                }
            )

    unit_health = lifecycle.get("unit_health", [])
    if lifecycle_failure is not None and systemd is not None:
        # The lifecycle could not say which units this device is *supposed* to
        # run, which is exactly when "which units are actually running" stops
        # being redundant. Enumerated from the unit directory instead, because a
        # bundle collected on a corrupted device that carries no runtime state at
        # all is a bundle whose first question the responder has to ask by phone.
        unit_health = systemd.health([unit["name"] for unit in units])

    # A bundle collected on a device whose lifecycle cannot be read is the
    # bundle that matters most, so the failure becomes content rather than an
    # exception. It is promoted to the top-level reason only when the caller did
    # not already name one: the caller's reason is why the bundle was collected,
    # and overwriting it would lose that.
    #
    # A lifecycle that *answered* and said something is wrong is promoted the
    # same way, and this half was missing. `status` reports `state_drift` --
    # units switched, state write never landed -- as a perfectly readable report
    # with ``ok: false``, so a bundle collected on exactly the device this whole
    # window design exists for led with ``reason: ok`` and ``action: none``. The
    # first two fields a responder reads told them there was nothing to do.
    if reason == "ok":
        promoted = lifecycle_failure or _unhealthy_status_reason(lifecycle)
        if promoted:
            reason = promoted

    bundle = {
        "schema": SUPPORT_BUNDLE_VERSION,
        "collected_at": now,
        "reason": reason,
        "reason_text": describe(reason),
        "action_code": action_for(reason),
        "note": sanitize_note(note),
        "note_policy": NOTE_POLICY,
        "privacy": {
            "policy": "redact-by-construction",
            "excluded": [
                "credential bytes",
                "device secrets",
                "raw journals",
                "wifi ssids and passphrases",
                "network addresses",
                "home directory names",
            ],
        },
        "lifecycle": {
            # False exactly when the lifecycle refused to answer. A responder
            # must be able to tell "this device has no history" from "this
            # device's history could not be read", and an empty list says the
            # first when it means the second.
            "readable": lifecycle_failure is None,
            "active_release": lifecycle.get("version"),
            "recorded_current": lifecycle.get("recorded_current"),
            "active_profile": lifecycle.get("active_profile"),
            "installed_releases": lifecycle.get("installed_releases", []),
            "history": lifecycle.get("history", []),
            "reason": lifecycle.get("reason"),
            "action_code": lifecycle.get("action_code"),
            "detail": lifecycle.get("detail", ""),
            "profile": lifecycle.get("profile"),
            "identity_present": lifecycle.get("identity_present"),
            "config_present": lifecycle.get("config_present"),
            "runbook_present": lifecycle.get("runbook_present"),
            # Whether an activation was in flight, which is the difference
            # between "this device is mid-install" and "this device lost its
            # state file". Three facts and a clamped duration -- deliberately
            # nothing the marker itself chose, since it is writable on exactly
            # the devices whose bundles get collected, and no clock read, so two
            # bundles over one device state stay byte-identical.
            "activation_window": activation_window_evidence(layout),
        },
        # Runtime unit state, read through the same injectable systemd boundary
        # the lifecycle uses. Without it a bundle can prove a unit is *correct*
        # and say nothing about whether it is *running*, which is the first
        # question anyone reading the ticket will ask.
        "unit_health": unit_health,
        "paths": layout.as_dict(),
        "units": units,
    }
    redacted = redact(bundle)
    # Inventories are attached *after* the key-driven pass, deliberately.
    # `redact` nukes any value under a key like "credentials", which is right
    # for a value and wrong for a listing: "the credential directory holds one
    # 0600 file called device.cred" is the single most useful line in a bundle
    # about a device that will not authenticate, and it discloses nothing. Each
    # entry still goes through value redaction, and no file is ever read.
    redacted["inventory"] = _inventories(layout)
    return redacted


def _inventories(layout: Layout) -> dict[str, Any]:
    return {
        "policy": "names, sizes, and modes only; no file is opened",
        "config": redact(_inventory(layout.config_dir)),
        "credentials": redact(_inventory(layout.credentials_dir)),
        "diagnostics": redact(_inventory(layout.diagnostics_dir)),
        "logs": redact(_log_inventory(layout.log_dir)),
    }


def _log_inventory(root: Path, *, limit: int = 50) -> list[dict[str, Any]]:
    """Log *names and sizes* only.

    Journals and application logs are the single richest source of patient
    identifiers, addresses, and credentials on a robot. Shipping their contents
    -- even a bounded tail, even redacted -- means one unanticipated log line is
    one disclosure. The diagnostic value of "the file exists and it is 4 MB" is
    most of what a first-line responder needs anyway.
    """

    entries = _inventory(root, limit=limit)
    for entry in entries:
        entry.pop("contents", None)
    return entries


def write_support_bundle(path: Path, bundle: dict[str, Any], *, mode: int = 0o600) -> Path:
    """Write canonical JSON atomically and restrictively.

    Canonical: sorted keys, fixed indent, trailing newline, so two runs over the
    same device state are byte-identical and a diff between two bundles means
    something.

    Atomic and restrictive: the file is created with its final mode, fsync-ed,
    and renamed into place. Writing world-readable bytes and chmod-ing after
    leaves a window during which a redaction bug and a shared ``/tmp`` are the
    same incident, and a half-written bundle is worse evidence than none.
    """

    path = Path(path)
    text = json.dumps(bundle, indent=2, sort_keys=True, ensure_ascii=False) + "\n"
    atomic_write(path, text, mode)
    return path
