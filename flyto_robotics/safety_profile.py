"""Site-level safety limits that a job may tighten but never loosen.

`SafetyLimits` already came from the job, bounded by hard clamps at parse time,
so the numbers were never hardcoded. What was wrong was who decides. A job
carried its own limits, which meant "this robot, in this building, may not
exceed X" had to be written into every job and could be forgotten in any one of
them. In an installation the site decides and the job asks.

So a profile is a set of ceilings and floors that the job is folded into:

    job says 0.40 m/s, profile says 0.25  ->  0.25, and it is recorded
    job says 0.20 m/s, profile says 0.25  ->  0.20, the job is stricter

The direction is not the same for every field, and that is the part that gets
mixed up. A lower speed is more conservative; a *higher* stop distance is more
conservative. Getting it backwards would silently let a job drive closer to
things than the site allows, which is the failure this module exists to
prevent — so the direction is data, declared once, and every rule is derived
from it rather than written out per field.

Nothing here imports ROS, so the arithmetic that decides how fast a robot may
move can be exercised on any machine.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any, Literal

from .fsio import atomic_write

#: How a profile value constrains the job's value.
#:
#: ``at_most``  the job may not exceed it   — speeds, where lower is safer
#: ``at_least`` the job may not fall below it — distances, where higher is safer
Direction = Literal["at_most", "at_least"]

#: Every field a site may constrain, and which way "safer" runs for it. A field
#: absent from this table cannot be set by a profile at all: adding one is a
#: deliberate act, not something a stray key in a JSON file can do.
CONSTRAINABLE: Mapping[str, Direction] = {
    "max_linear_speed": "at_most",
    "max_angular_speed": "at_most",
    "obstacle_stop_distance": "at_least",
    "lateral_stop_distance": "at_least",
    "emergency_stop_distance": "at_least",
}

PROFILE_CONTRACT_VERSION = "flyto.robotics.safety-profile.v1"


class SafetyProfileError(ValueError):
    """A profile that cannot be trusted to constrain anything."""


@dataclass(frozen=True)
class Adjustment:
    """One limit the site overrode, and by how much."""

    field: str
    requested: float
    applied: float
    direction: Direction

    def describe(self) -> str:
        relation = (
            "above the site ceiling"
            if self.direction == "at_most"
            else "inside the site floor"
        )
        return (
            f"{self.field}: job asked {self.requested:g}, which is {relation} "
            f"of {self.applied:g}; using {self.applied:g}"
        )


@dataclass(frozen=True)
class ProfileOutcome:
    """The limits to run under, and every place the site overrode the job."""

    values: Mapping[str, float]
    adjustments: tuple[Adjustment, ...]

    @property
    def constrained(self) -> bool:
        return bool(self.adjustments)


def _number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SafetyProfileError(f"{field} must be a number")
    number = float(value)
    if number != number or number in (float("inf"), float("-inf")):
        raise SafetyProfileError(f"{field} must be finite")
    if number <= 0:
        raise SafetyProfileError(f"{field} must be greater than zero")
    return number


def parse_profile(document: Any) -> dict[str, float]:
    """Validate a site profile document into the limits it constrains.

    Unknown keys are refused rather than ignored. A typo in a safety file that
    silently does nothing is worse than one that fails loudly: the site would
    believe a limit was in force.
    """
    if not isinstance(document, Mapping):
        raise SafetyProfileError("safety profile must be a JSON object")

    version = document.get("contract_version")
    if version != PROFILE_CONTRACT_VERSION:
        raise SafetyProfileError(
            f"safety profile contract_version must be {PROFILE_CONTRACT_VERSION}"
        )

    limits = document.get("limits", {})
    if not isinstance(limits, Mapping):
        raise SafetyProfileError("safety profile limits must be an object")

    unknown = sorted(set(limits) - set(CONSTRAINABLE))
    if unknown:
        raise SafetyProfileError(
            f"safety profile cannot constrain {', '.join(unknown)}; "
            f"settable limits are {', '.join(sorted(CONSTRAINABLE))}"
        )

    return {name: _number(value, f"limits.{name}") for name, value in limits.items()}


def apply_profile(
    profile: Mapping[str, float],
    requested: Mapping[str, float | None],
) -> ProfileOutcome:
    """Fold a job's requested limits into the site's, and say what changed.

    A job is never rejected for asking too much — it runs, constrained, and the
    constraint is reported. Refusing outright would make a site profile
    something operators route around by editing jobs; folding makes the site
    the answer without making it an obstacle.
    """
    values: dict[str, float] = {}
    adjustments: list[Adjustment] = []

    for field, asked in requested.items():
        if asked is None:
            continue
        ceiling_or_floor = profile.get(field)
        direction = CONSTRAINABLE.get(field)
        if ceiling_or_floor is None or direction is None:
            values[field] = asked
            continue

        if direction == "at_most":
            applied = min(asked, ceiling_or_floor)
        else:
            applied = max(asked, ceiling_or_floor)

        values[field] = applied
        if applied != asked:
            adjustments.append(
                Adjustment(
                    field=field,
                    requested=asked,
                    applied=applied,
                    direction=direction,
                )
            )

    return ProfileOutcome(values=values, adjustments=tuple(adjustments))


def is_more_conservative(field: str, candidate: float, than: float) -> bool:
    """Whether ``candidate`` is the safer of two values for ``field``.

    Used when deciding if a proposed profile change tightens or relaxes the
    site. Relaxing is not forbidden — a site may genuinely need it — but it is
    the case that must be recorded and approved rather than slipped through.
    """
    direction = CONSTRAINABLE.get(field)
    if direction is None:
        raise SafetyProfileError(f"{field} is not a constrainable limit")
    return candidate < than if direction == "at_most" else candidate > than


# -- where a site keeps its profile, and what it records -----------------

#: Read from here unless a caller says otherwise. Outside the repository on
#: purpose: a site limit that lived in the deployed tree would be replaced by
#: the next `git pull`, which is the opposite of what a site limit is for.
DEFAULT_PROFILE_PATH = "/etc/flyto/safety-profile.json"

#: Appended to on every change, never rewritten.
DEFAULT_AUDIT_PATH = "/etc/flyto/safety-profile.audit.jsonl"

MAX_PROFILE_BYTES = 16 * 1024


def load_profile(path: Any) -> dict[str, float]:
    """The site profile, or an empty one when the site has not set any.

    A missing file means no constraint, which is the correct default for a
    robot that has not been commissioned into a site. A file that exists but
    cannot be read or parsed is an error and is *not* treated as absent: a site
    that wrote a profile and had it silently ignored would believe a limit was
    in force that was not.
    """
    from pathlib import Path as _Path

    profile_path = _Path(path)
    if not profile_path.exists():
        return {}
    if profile_path.stat().st_size > MAX_PROFILE_BYTES:
        raise SafetyProfileError(f"{profile_path} exceeds {MAX_PROFILE_BYTES} bytes")
    try:
        import json as _json

        document = _json.loads(profile_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, ValueError) as exc:
        raise SafetyProfileError(f"{profile_path} is not readable JSON") from exc
    return parse_profile(document)


def change_record(
    *,
    changed_by: str,
    at: str,
    before: Mapping[str, float],
    after: Mapping[str, float],
    reason: str,
) -> dict[str, Any]:
    """One audit line: who, when, from what to what, and whether it relaxed.

    ``relaxed`` is computed rather than declared. Someone widening a limit and
    describing it as a tightening is exactly the entry an audit exists to
    catch, so the record does not take their word for it.
    """
    relaxed: list[str] = []
    for field in sorted(set(before) | set(after)):
        old, new = before.get(field), after.get(field)
        if old is None and new is not None:
            continue  # a new constraint only tightens
        if new is None and old is not None:
            relaxed.append(field)  # removing a limit is the widest relaxation
            continue
        if (
            old is not None
            and new is not None
            and old != new
            and not is_more_conservative(field, new, old)
        ):
            relaxed.append(field)

    return {
        "contract_version": PROFILE_CONTRACT_VERSION,
        "changed_by": changed_by,
        "at": at,
        "reason": reason,
        "before": dict(before),
        "after": dict(after),
        "relaxed_limits": relaxed,
        "relaxes_safety": bool(relaxed),
    }


def _atomic_write(path: Any, text: str, mode: int = 0o600) -> None:
    """Replace a file's contents or leave the old ones entirely alone."""
    atomic_write(path, text, mode)


def update_profile(
    profile_path: Any,
    audit_path: Any,
    *,
    limits: Mapping[str, float],
    changed_by: str,
    reason: str,
    at: str,
) -> dict[str, Any]:
    """Change the site profile, refusing if the change cannot be recorded.

    The audit is checked for writability *before* the profile is touched. A
    limit that can be changed without leaving a trace is not governed, and the
    moment it matters is exactly the moment someone would rather it left none.

    Returns the audit record. A change takes effect at the next job load: a
    mission already running resolved its limits when it started, so this cannot
    reach into one in flight — which is why there is no lock here.
    """
    import json as _json
    from pathlib import Path as _Path

    if not changed_by.strip():
        raise SafetyProfileError("changed_by is required; an unattributed change is not an audit")
    if not reason.strip():
        raise SafetyProfileError("reason is required")

    validated = parse_profile({"contract_version": PROFILE_CONTRACT_VERSION, "limits": limits})
    before = load_profile(profile_path)
    record = change_record(
        changed_by=changed_by, at=at, before=before, after=validated, reason=reason
    )

    audit = _Path(audit_path)
    audit.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    try:
        # Open before writing the profile: if the record cannot be kept, the
        # change does not happen.
        import os as _os

        flags = _os.O_APPEND | _os.O_CREAT | _os.O_WRONLY | getattr(_os, "O_NOFOLLOW", 0)
        descriptor = _os.open(audit, flags, 0o600)
        _os.fchmod(descriptor, 0o600)
        with _os.fdopen(descriptor, "a", encoding="utf-8") as handle:
            handle.write(_json.dumps(record, sort_keys=True) + "\n")
            handle.flush()
            _os.fsync(handle.fileno())
    except OSError as exc:
        raise SafetyProfileError(
            f"{audit} could not be written, so the change was not made"
        ) from exc

    _atomic_write(
        profile_path,
        _json.dumps(
            {"contract_version": PROFILE_CONTRACT_VERSION, "limits": validated},
            indent=2,
            sort_keys=True,
        )
        + "\n",
    )
    return record


def audit_tail(audit_path: Any, limit: int = 20) -> list[dict[str, Any]]:
    """The most recent changes, newest last. Empty when nothing was recorded."""
    import json as _json
    from pathlib import Path as _Path

    path = _Path(audit_path)
    if not path.exists():
        return []
    entries: list[dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            entries.append(_json.loads(line))
        except ValueError:
            # A corrupt line is reported as one rather than dropped: an audit
            # that quietly skips what it cannot read is not an audit.
            entries.append({"unreadable_entry": line[:200]})
    return entries[-limit:]
