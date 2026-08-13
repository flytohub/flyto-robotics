"""Generic versioned device-event contract, owner-only journal, bounded export.

A device that carries out work upstream cannot ask an operator to read its
logs.  It has to be able to say, in a shape something else can consume without
knowing what the device is, *what happened, to which resource, in which run,
why, and what to do about it*.  That is all this module is.

Three properties are deliberate:

* **Generic.** Nothing here knows about robots, ROS, TurtleBot, Gazebo or
  motion.  ``component`` and ``resource_id`` are opaque identifiers, and a
  printer, a kiosk, a scanner or a background service is described by exactly
  the same envelope.  A contract named after the first thing that used it
  becomes a second contract the day the second thing arrives.
* **Bounded and public by construction.** Every field has an exact type, an
  exact size, and a count bound.  Free text is one short message; structure is
  one small object.  There is no field into which a raw journal, a stack trace
  or a log tail can be poured, because a field like that is where credentials
  and patient data leave a device.
* **Fail closed.** A key that looks like a secret or like personal data is
  rejected rather than dropped, and so is a value that looks like a token, a
  private key or a JWT.  Rejecting is louder than redacting: a caller finds out
  at the point of the mistake instead of shipping a quietly emptied event.
  Because this contract refuses rather than redacts, ``redacted_key_count`` is
  required to be ``0``; a non-zero count would be a claim about scrubbing that
  no code here performs.

The journal is append-only, owner-only, bounded, and safe against concurrent
writers.  Export hands out records after an opaque cursor with a hard item and
byte limit, so an upstream reader can resume without ever being given the file
— and a cursor that has been outrun by retention is reported as a gap rather
than silently satisfied, because "you are up to date" and "the records you
missed are gone" must never look the same to a reader.

Standard library only, Python 3.9 compatible.  This module must stay importable
on a device with no ROS, no simulator and no third-party package installed.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import re
import stat
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:  # pragma: no cover - exercised implicitly; absent only on non-POSIX hosts
    import fcntl
except ImportError:  # pragma: no cover - Windows has no advisory locking here
    fcntl = None  # type: ignore[assignment]

#: Everything another module, a device, or the export CLI may depend on. A name
#: absent from this list is an internal detail: it can change without a contract
#: version, and a caller that reached for it has no promise it will still be
#: there.
__all__ = [
    "ACTION_CODE_LIMIT",
    "DEFAULT_EXPORT_BYTES",
    "DEFAULT_EXPORT_ITEMS",
    "DEFAULT_JOURNAL_FILENAME",
    "DEFAULT_MAX_BYTES",
    "DEFAULT_MAX_EVENTS",
    "DETAILS_BYTE_LIMIT",
    "DETAILS_DEPTH_LIMIT",
    "DETAILS_KEY_LIMIT",
    "DETAILS_LIST_LIMIT",
    "DETAILS_NODE_LIMIT",
    "DETAILS_STRING_LIMIT",
    "DEVICE_EVENT_CONTRACT",
    "DEVICE_EVENT_EXPORT_CONTRACT",
    "DEVICE_EVENT_JOURNAL_CONTRACT",
    "DEVICE_EVENT_JOURNAL_ENV",
    "EVENT_BYTE_LIMIT",
    "HARD_MAX_EXPORT_BYTES",
    "HARD_MAX_EXPORT_ITEMS",
    "HARD_MAX_JOURNAL_BYTES",
    "HARD_MAX_JOURNAL_EVENTS",
    "JOURNAL_DIR_MODE",
    "JOURNAL_FILE_MODE",
    "MESSAGE_LIMIT",
    "MIN_EXPORT_BYTES",
    "RECORD_FRAMING_BYTES",
    "REDACTION_POLICY",
    "SEQUENCE_LIMIT",
    "SEVERITIES",
    "STATUSES",
    "DeviceEventBoundError",
    "DeviceEventContractError",
    "DeviceEventCursorError",
    "DeviceEventError",
    "DeviceEventJournal",
    "DeviceEventJournalError",
    "build_device_event",
    "canonical_json",
    "decode_cursor",
    "derive_event_id",
    "encode_cursor",
    "event_sequence",
    "export_page_bytes",
    "is_sensitive_key",
    "now_observed_at",
    "parse_device_event",
    "record_byte_size",
]

DEVICE_EVENT_CONTRACT = "flyto.device-event.v1"
DEVICE_EVENT_EXPORT_CONTRACT = "flyto.device-event-export.v1"
DEVICE_EVENT_JOURNAL_CONTRACT = "flyto.device-event-journal.v1"

#: The environment variable that names the journal to read. It is shared here
#: rather than spelled out again in the CLI because a device writes the journal
#: and something else exports it: two spellings of the same variable is two
#: journals, one of which is silently never read.
DEVICE_EVENT_JOURNAL_ENV = "FLYTO_DEVICE_EVENT_JOURNAL"

#: The file name a journal takes inside a directory that names no file. Also
#: shared for the same reason: the writer and the exporter have to agree.
DEFAULT_JOURNAL_FILENAME = "device-events.ndjson"

#: Named so a reader can tell which redaction rules produced an event, without
#: having to infer them from the event contract version.
REDACTION_POLICY = "flyto.device-event.redaction.v1"

#: How severe this is for whoever is watching the fleet.
SEVERITIES = ("info", "notice", "warning", "error", "critical")

#: What state the thing the event is about ended up in. This is a small closed
#: set on purpose: an upstream reader must be able to branch on it without
#: knowing what kind of device sent it.
STATUSES = (
    "started",
    "running",
    "succeeded",
    "failed",
    "refused",
    "degraded",
    "unavailable",
    "unknown",
)

_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")
_CODE = re.compile(r"^[a-z][a-z0-9_]{0,63}$")
_CURSOR = re.compile(r"^dej1:([0-9a-f]{32}):(0|[1-9][0-9]{0,15}):([0-9a-f]{16})$")
_JOURNAL_ID = re.compile(r"^[0-9a-f]{32}$")

#: Canonical RFC 3339 UTC, and only that. An offset form, a space separator, a
#: lowercase ``z`` or a local time would all still be "a timestamp", and every
#: one of them makes two devices' events unorderable without a parser that
#: knows which dialect each speaks.
_TIMESTAMP = re.compile(r"^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(\.\d{1,6})?Z$")

#: The largest ``sequence`` this contract accepts, mirrored from the schema so
#: a caller can bound its own counter without reading the validator.
SEQUENCE_LIMIT = 2**53

MESSAGE_LIMIT = 200
ACTION_CODE_LIMIT = 16
DETAILS_DEPTH_LIMIT = 6
DETAILS_NODE_LIMIT = 128
DETAILS_KEY_LIMIT = 32
DETAILS_LIST_LIMIT = 32
DETAILS_STRING_LIMIT = 512
DETAILS_BYTE_LIMIT = 8 * 1024
EVENT_BYTE_LIMIT = 16 * 1024

DEFAULT_MAX_EVENTS = 512
DEFAULT_MAX_BYTES = 1024 * 1024
HARD_MAX_JOURNAL_EVENTS = 100_000
HARD_MAX_JOURNAL_BYTES = 8 * 1024 * 1024
HARD_MAX_EXPORT_ITEMS = 1000
HARD_MAX_EXPORT_BYTES = 1024 * 1024
DEFAULT_EXPORT_ITEMS = 256
DEFAULT_EXPORT_BYTES = 256 * 1024

#: What a stored record costs on top of the event inside it: the two keys that
#: wrap it (``{"event":`` … ``,"journal_sequence":`` … ``}``), the digits of the
#: largest position this contract can hold, and the newline it occupies in the
#: file.  Forty-seven bytes today, rounded up, because a bound that is exact is
#: wrong the first time a key is renamed.
RECORD_FRAMING_BYTES = 64

#: The smallest export byte bound that can actually be honoured, and the reason
#: it is not simply 1024.
#:
#: :meth:`DeviceEventJournal.export` returns the first record after the cursor
#: even when it alone fills the budget, and it has to: dropping it would lose a
#: record with nothing said about it, and returning an empty page would leave a
#: reader whose cursor can never advance — asking forever, told each time that
#: nothing is there.  That guarantee is only safe while one record always fits.
#: So a budget too small to hold one maximal record is refused up front rather
#: than honoured approximately.  At or above this floor the promise in this
#: module's docstring is a promise: no page exceeds the bound, no record is
#: skipped, and every call moves the cursor.
MIN_EXPORT_BYTES = EVENT_BYTE_LIMIT + RECORD_FRAMING_BYTES

JOURNAL_FILE_MODE = 0o600
JOURNAL_DIR_MODE = 0o700
_HEADER_ALLOWANCE = 4096
_EMPTY_TAG = "0" * 16


class DeviceEventError(ValueError):
    """Base class for every refusal this module raises."""


class DeviceEventContractError(DeviceEventError):
    """An event is malformed, unbounded, or carries something it must not."""


class DeviceEventJournalError(DeviceEventError):
    """The journal is unusable, unsafe, or corrupt, and will not be guessed at."""


class DeviceEventCursorError(DeviceEventJournalError):
    """A cursor is malformed, from another journal, or ahead of what exists."""


class DeviceEventBoundError(DeviceEventJournalError):
    """A byte or item bound cannot be honoured, so nothing is returned at all.

    Raised instead of the two silent alternatives.  Trimming the response to fit
    would drop a record the caller was never told about; exceeding the bound
    would break the promise the caller sized its buffer, pipe or response body
    from.  A distinct type rather than a message, because a caller that can
    react — by raising ``max_bytes`` — has to be able to tell this apart from a
    corrupt journal, which no retry will fix.
    """


# -- privacy ------------------------------------------------------------------
#
# Word-based, not substring-based. A substring rule rejects "wifi_has_address"
# and "unknown_service_ids" while still missing "apiKey", which is the wrong
# answer in both directions: the false positives get the check disabled, and the
# real one ships.

_SENSITIVE_WORDS = frozenset(
    {
        "credential",
        "credentials",
        "passphrase",
        "passwd",
        "password",
        "pwd",
        "secret",
        "secrets",
        "apikey",
        "cookie",
        "cookies",
        "authorization",
        "patient",
        "mrn",
        "ssn",
        "phi",
        "pii",
        "dob",
        "birthdate",
        "birthday",
        "firstname",
        "lastname",
        "surname",
        "email",
        "phone",
        "telephone",
    }
)

#: "token" is a real word in bounded-resource names ("token_budget"), so it is
#: only sensitive when nothing beside it says it is a count.
_BENIGN_TOKEN_NEIGHBOURS = frozenset(
    {
        "budget",
        "bucket",
        "count",
        "length",
        "limit",
        "max",
        "maximum",
        "minimum",
        "rate",
        "threshold",
        "window",
    }
)

_SENSITIVE_PAIRS = frozenset(
    {
        ("access", "key"),
        ("api", "key"),
        ("auth", "token"),
        ("bearer", "token"),
        ("date", "birth"),
        ("given", "name"),
        ("home", "address"),
        ("mail", "address"),
        ("medical", "record"),
        ("national", "id"),
        ("patient", "id"),
        ("postal", "code"),
        ("private", "key"),
        ("secret", "key"),
        ("session", "key"),
        ("signing", "key"),
        ("street", "address"),
        ("zip", "code"),
    }
)

_SECRET_VALUE_PATTERNS = (
    re.compile(r"-----BEGIN [A-Z ]{0,40}PRIVATE KEY-----"),
    re.compile(r"(?i)\b(?:bearer|basic)\s+[A-Za-z0-9._~+/=-]{16,}"),
    re.compile(r"\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{4,}"),
    re.compile(r"(?i)\b(?:password|passwd|secret|api[_-]?key|token|credential)\b\s*[:=]\s*\S"),
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
    re.compile(r"\b\d{3}-\d{2}-\d{4}\b"),
)


def _words(value: str) -> list[str]:
    separated = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", value)
    return [part for part in re.split(r"[^A-Za-z0-9]+", separated.lower()) if part]


def is_sensitive_key(value: str) -> bool:
    """True when a key name looks like a secret or like personal data."""
    parts = _words(value)
    if any(part in _SENSITIVE_WORDS for part in parts):
        return True
    pairs = set(zip(parts, parts[1:]))
    if pairs & _SENSITIVE_PAIRS:
        return True
    for index, part in enumerate(parts):
        if part != "token":
            continue
        neighbours = parts[max(0, index - 1) : index] + parts[index + 1 : index + 2]
        if not any(neighbour in _BENIGN_TOKEN_NEIGHBOURS for neighbour in neighbours):
            return True
    return False


def _refuse_secret_value(value: str, label: str) -> None:
    for pattern in _SECRET_VALUE_PATTERNS:
        if pattern.search(value):
            raise DeviceEventContractError(f"{label} looks like a secret or personal data")


# -- scalar validation --------------------------------------------------------


def _mapping(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise DeviceEventContractError(f"{label} must be an object")
    return dict(value)


def _sequence_of(value: Any, label: str, limit: int) -> list[Any]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise DeviceEventContractError(f"{label} must be a list")
    if len(value) > limit:
        raise DeviceEventContractError(f"{label} exceeds {limit} items")
    return list(value)


def _exact_fields(value: Mapping[str, Any], *, required: set[str], optional: set[str]) -> None:
    missing = required - set(value)
    extra = set(value) - required - optional
    if missing:
        raise DeviceEventContractError(f"missing device event fields: {', '.join(sorted(missing))}")
    if extra:
        raise DeviceEventContractError(f"unknown device event fields: {', '.join(sorted(extra))}")


def _identifier(value: Any, label: str, *, allow_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise DeviceEventContractError(f"{label} must be text")
    normalized = value.strip()
    if not normalized:
        if allow_empty:
            return ""
        raise DeviceEventContractError(f"{label} must not be empty")
    if not _IDENTIFIER.fullmatch(normalized):
        raise DeviceEventContractError(f"{label} must be a safe identifier")
    return normalized


def _code(value: Any, label: str) -> str:
    if not isinstance(value, str) or not _CODE.fullmatch(value.strip()):
        raise DeviceEventContractError(f"{label} must be a lowercase snake_case code")
    return value.strip()


def _integer(value: Any, label: str, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DeviceEventContractError(f"{label} must be an integer")
    if value < minimum or value > maximum:
        raise DeviceEventContractError(f"{label} is out of bounds")
    return value


def _boolean(value: Any, label: str) -> bool:
    if not isinstance(value, bool):
        raise DeviceEventContractError(f"{label} must be boolean")
    return value


def _choice(value: Any, label: str, choices: Sequence[str]) -> str:
    if not isinstance(value, str) or value not in choices:
        raise DeviceEventContractError(f"{label} is unsupported")
    return value


def _observed_at(value: Any) -> str:
    """Exactly one timestamp dialect: ``YYYY-MM-DDTHH:MM:SS[.ffffff]Z``.

    The shape is checked and then the calendar is: ``2026-02-30T00:00:00Z``
    matches every character class a pattern can express and is not a day, and a
    device that emits it would sort into the middle of February forever.
    """
    if not isinstance(value, str):
        raise DeviceEventContractError("observed_at must be text")
    normalized = value.strip()
    if len(normalized) > 64:
        raise DeviceEventContractError("observed_at is out of bounds")
    matched = _TIMESTAMP.fullmatch(normalized)
    if not matched:
        raise DeviceEventContractError(
            "observed_at must be canonical RFC 3339 UTC, as YYYY-MM-DDTHH:MM:SS[.ffffff]Z"
        )
    year, month, day, hour, minute, second = (int(part) for part in matched.groups()[:6])
    if second > 59:
        # A leap second is a real instant that datetime cannot hold; accepting
        # the text and failing to parse it later is worse than refusing now.
        raise DeviceEventContractError("observed_at must not carry a leap second")
    try:
        datetime(year, month, day, hour, minute, second)
    except ValueError as exc:
        raise DeviceEventContractError("observed_at is not a real UTC instant") from exc
    return normalized


def now_observed_at(moment: datetime | None = None) -> str:
    """The canonical timestamp string this contract accepts, for callers.

    An aware value is converted to UTC first.  Stamping ``18:00+08:00`` as
    ``18:00Z`` because both end in a timestamp-shaped string would move an event
    eight hours into the future, and every ordering built on it afterwards would
    be wrong in a way no later validation could detect.

    A naive value is taken to be UTC already, because that is what the callers
    in this repository produce and because the alternative — reading it as the
    host's local time — makes the same event mean different instants on two
    machines with the same clock.
    """
    value = moment if moment is not None else datetime.now(timezone.utc)
    if value.tzinfo is not None:
        value = value.astimezone(timezone.utc)
    return value.strftime("%Y-%m-%dT%H:%M:%S.%f") + "Z"


_EPOCH = datetime(1970, 1, 1, tzinfo=timezone.utc)
_last_sequence = 0


def event_sequence(moment: datetime | None = None) -> int:
    """A contract-valid ordering key: epoch microseconds, never decreasing.

    ``time.time_ns()`` is the obvious thing to reach for and it is wrong here.
    An epoch *nanosecond* count passed ``2**53`` in 1970, so every event stamped
    with one is refused by :func:`parse_device_event` — a device whose entire
    purpose is to be able to say what went wrong, unable to say anything, and
    the refusal surfaces at the journal rather than at the clock.  Microseconds
    stay inside the bound until the year 2255.

    Monotonic *enough*, not monotonic.  A wall clock can be stepped backwards by
    NTP or by an operator, and two events from one process that compare equal or
    out of order cannot be ordered by a reader at all.  Within this process the
    value is therefore forced to advance by at least one microsecond per call.
    Across processes and reboots the wall clock is all there is, which is why
    ``event_id`` is derived from content rather than from position.
    """
    global _last_sequence
    value = moment if moment is not None else datetime.now(timezone.utc)
    if value.tzinfo is None:
        value = value.replace(tzinfo=timezone.utc)
    delta = value.astimezone(timezone.utc) - _EPOCH
    micros = (delta.days * 86_400 + delta.seconds) * 1_000_000 + delta.microseconds
    micros = max(0, min(micros, SEQUENCE_LIMIT))
    if micros <= _last_sequence:
        micros = min(_last_sequence + 1, SEQUENCE_LIMIT)
    _last_sequence = micros
    return micros


def _public_message(value: Any) -> str:
    if value is None:
        return ""
    if not isinstance(value, str):
        raise DeviceEventContractError("message must be text")
    if any(character < " " or character == "\x7f" for character in value):
        raise DeviceEventContractError("message must not contain control characters")
    normalized = " ".join(value.split())
    if len(normalized) > MESSAGE_LIMIT:
        raise DeviceEventContractError(f"message exceeds {MESSAGE_LIMIT} characters")
    _refuse_secret_value(normalized, "message")
    return normalized


def _action_codes(value: Any) -> list[str]:
    items = _sequence_of(value, "action_codes", ACTION_CODE_LIMIT)
    codes = [_code(item, "action code") for item in items]
    # Order is preserved because it is the order an operator should try them in.
    return list(dict.fromkeys(codes))


def _details(value: Any) -> dict[str, Any]:
    details = _mapping(value, "details")
    _bounded_public_json(details, label="details")
    encoded = _canonical_json(details).encode("utf-8")
    if len(encoded) > DETAILS_BYTE_LIMIT:
        raise DeviceEventContractError(f"details exceed {DETAILS_BYTE_LIMIT} bytes")
    return details


def _bounded_public_json(
    value: Any, *, label: str, depth: int = 0, nodes: list[int] | None = None
) -> None:
    """Enforce shape, size and privacy on everything reachable from ``details``."""
    counter = nodes if nodes is not None else [0]
    counter[0] += 1
    if counter[0] > DETAILS_NODE_LIMIT:
        raise DeviceEventContractError(f"{label} exceeds {DETAILS_NODE_LIMIT} values")
    if depth > DETAILS_DEPTH_LIMIT:
        raise DeviceEventContractError(f"{label} is nested too deeply")
    if value is None or isinstance(value, bool):
        return
    if isinstance(value, int):
        # Bounded so a reader never has to hold a bignum it cannot serialise.
        if abs(value) > 2**53:
            raise DeviceEventContractError(f"{label} integer is out of bounds")
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise DeviceEventContractError(f"{label} numbers must be finite")
        return
    if isinstance(value, str):
        if len(value) > DETAILS_STRING_LIMIT:
            raise DeviceEventContractError(f"{label} string exceeds {DETAILS_STRING_LIMIT}")
        _refuse_secret_value(value, label)
        return
    if isinstance(value, list):
        if len(value) > DETAILS_LIST_LIMIT:
            raise DeviceEventContractError(f"{label} list exceeds {DETAILS_LIST_LIMIT} items")
        for item in value:
            _bounded_public_json(item, label=label, depth=depth + 1, nodes=counter)
        return
    if isinstance(value, Mapping):
        if len(value) > DETAILS_KEY_LIMIT:
            raise DeviceEventContractError(f"{label} object exceeds {DETAILS_KEY_LIMIT} keys")
        for key, item in value.items():
            if not isinstance(key, str) or not key or len(key) > 64:
                raise DeviceEventContractError(f"{label} keys must be bounded strings")
            if is_sensitive_key(key):
                raise DeviceEventContractError(
                    f"{label} key {key!r} names a credential or personal data"
                )
            _bounded_public_json(item, label=label, depth=depth + 1, nodes=counter)
        return
    raise DeviceEventContractError(f"{label} contains an unsupported value")


def _redaction(value: Any, *, has_message: bool) -> dict[str, Any]:
    """Explicit, checkable privacy metadata rather than an implied promise."""
    supplied = _mapping(value if value is not None else {}, "redaction")
    _exact_fields(
        supplied,
        required=set(),
        optional={
            "policy",
            "free_text",
            "raw_logs_included",
            "credentials_included",
            "personal_data_included",
            "redacted_key_count",
        },
    )
    policy = supplied.get("policy", REDACTION_POLICY)
    if policy != REDACTION_POLICY:
        raise DeviceEventContractError("unsupported redaction policy")
    for field in ("raw_logs_included", "credentials_included", "personal_data_included"):
        if _boolean(supplied.get(field, False), f"redaction {field}"):
            raise DeviceEventContractError(
                f"redaction.{field} must be false; this contract carries neither"
            )
    declared = supplied.get("free_text")
    free_text = has_message if declared is None else _boolean(declared, "redaction free_text")
    if free_text != has_message:
        raise DeviceEventContractError("redaction.free_text does not match the message")
    count = _integer(
        supplied.get("redacted_key_count", 0),
        "redaction redacted_key_count",
        minimum=0,
        maximum=0,
    )
    return {
        "policy": REDACTION_POLICY,
        "free_text": free_text,
        "raw_logs_included": False,
        "credentials_included": False,
        "personal_data_included": False,
        "redacted_key_count": count,
    }


def canonical_json(value: Any) -> str:
    """The one encoding of a value this contract recognises: sorted, compact.

    Public because a hash, a stored line and an exported document all have to be
    the same bytes for the same content.  A caller that re-encodes "the same"
    JSON with different separators or unsorted keys produces a different digest
    for an identical event, and every deduplication built on that digest breaks.
    """
    return json.dumps(value, ensure_ascii=False, separators=(",", ":"), sort_keys=True)


#: The internal spelling used throughout this module, kept so the public name
#: and the private one can never drift into two encoders.
_canonical_json = canonical_json


def record_byte_size(record: Mapping[str, Any]) -> int:
    """What one stored record costs against an export byte bound.

    One record on the wire is its canonical encoding plus the single newline it
    occupies in the file, and that "plus one" is the whole reason this is a
    function rather than an expression repeated at each site.  The selector
    budgets pages with it, and a reader that re-measures a page it was handed
    must measure it the same way: two arithmetics that differ by one byte per
    record turn a bound check into a check of something else, and it fails only
    on the page that was exactly at the limit — the one case the bound exists for.

    Malformed input is a typed refusal, not a ``TypeError`` from inside
    ``json``.  A caller measuring a document it did not build has to be able to
    tell "this is not an export page" apart from a bug in its own code, and
    :class:`DeviceEventContractError` is the answer this contract already gives
    to everything else that is not the shape it claims to be.
    """
    if not isinstance(record, Mapping):
        raise DeviceEventContractError("export record must be an object")
    try:
        encoded = _canonical_json(record)
    except (TypeError, ValueError) as exc:
        # A record holding something JSON cannot express has no size in the only
        # encoding this contract recognises, so there is no number to return.
        raise DeviceEventContractError("export record is not encodable as canonical JSON") from exc
    return len(encoded.encode("utf-8")) + 1


def export_page_bytes(document: Mapping[str, Any]) -> int:
    """What the records of an export document cost, counted as ``export`` budgets.

    Public because the accounting is shared, not because the number is
    interesting.  :meth:`DeviceEventJournal.export` selects a page under a byte
    bound, and whoever prints or forwards that page has to be able to check it
    against the same bound with the same arithmetic; the ``flyto-device-events``
    command does exactly that before anything reaches stdout.  Keeping a second
    copy of the sum in the exporter is how the writer and the reader end up
    disagreeing about what a page costs.

    Only the records are counted.  The envelope around them — contract, cursor,
    counts, gap flags — is small, fixed, and deliberately outside the bound the
    caller sized its buffer from, which is what ``max_bytes`` has always meant
    here.
    """
    if not isinstance(document, Mapping):
        raise DeviceEventContractError("export document must be an object")
    if "records" not in document:
        raise DeviceEventContractError("export document has no records")
    records = _sequence_of(document["records"], "export document records", HARD_MAX_EXPORT_ITEMS)
    return sum(record_byte_size(record) for record in records)


def _content_hash(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def derive_event_id(
    *,
    resource_id: str,
    component: str,
    sequence: int,
    observed_at: str,
    reason_code: str,
    status: str,
    run_id: str = "",
    correlation_id: str = "",
) -> str:
    """A stable ID for one observation, derived from what the observation is.

    Deriving rather than randomising means the same observation reported twice
    — a retried export, a journal replayed after a reboot — carries the same ID,
    so an upstream reader can deduplicate without a second agreement about how.
    """
    digest = _content_hash(
        {
            "component": component,
            "correlation_id": correlation_id,
            "observed_at": observed_at,
            "reason_code": reason_code,
            "resource_id": resource_id,
            "run_id": run_id,
            "sequence": sequence,
            "status": status,
        }
    )
    return f"evt-{digest[:32]}"


def build_device_event(
    *,
    resource_id: str,
    component: str,
    sequence: int,
    observed_at: str,
    severity: str,
    status: str,
    reason_code: str,
    action_codes: Sequence[str] = (),
    correlation_id: str = "",
    run_id: str = "",
    message: str = "",
    details: Mapping[str, Any] | None = None,
    event_id: str = "",
) -> dict[str, Any]:
    """Assemble and validate one event, deriving the ID when none is given."""
    resolved_id = event_id or derive_event_id(
        resource_id=resource_id,
        component=component,
        sequence=sequence,
        observed_at=observed_at,
        reason_code=reason_code,
        status=status,
        run_id=run_id,
        correlation_id=correlation_id,
    )
    return parse_device_event(
        {
            "contract": DEVICE_EVENT_CONTRACT,
            "event_id": resolved_id,
            "resource_id": resource_id,
            "component": component,
            "sequence": sequence,
            "observed_at": observed_at,
            "severity": severity,
            "status": status,
            "reason_code": reason_code,
            "action_codes": list(action_codes),
            "correlation_id": correlation_id,
            "run_id": run_id,
            "message": message,
            "details": dict(details or {}),
        }
    )


def parse_device_event(value: Mapping[str, Any]) -> dict[str, Any]:
    """Validate one event and bind its content hash.

    ``correlation_id`` and ``run_id`` may be empty, and empty means exactly one
    thing: *this event belongs to no run and to no wider correlation*.  A
    periodic health observation genuinely has neither, and inventing an ID for
    it would make unrelated snapshots look like one sequence.  Empty is never a
    substitute for an ID the caller has but did not pass.
    """
    event = _mapping(value, "device event")
    _exact_fields(
        event,
        required={
            "contract",
            "event_id",
            "resource_id",
            "component",
            "sequence",
            "observed_at",
            "severity",
            "status",
            "reason_code",
        },
        optional={
            "action_codes",
            "correlation_id",
            "run_id",
            "message",
            "details",
            "redaction",
            "event_hash",
        },
    )
    if event["contract"] != DEVICE_EVENT_CONTRACT:
        raise DeviceEventContractError("unsupported device event contract")

    message = _public_message(event.get("message", ""))
    normalized = {
        "contract": DEVICE_EVENT_CONTRACT,
        "event_id": _identifier(event["event_id"], "event_id"),
        "resource_id": _identifier(event["resource_id"], "resource_id"),
        "component": _code(event["component"], "component"),
        "sequence": _integer(event["sequence"], "sequence", minimum=0, maximum=2**53),
        "observed_at": _observed_at(event["observed_at"]),
        "severity": _choice(event["severity"], "severity", SEVERITIES),
        "status": _choice(event["status"], "status", STATUSES),
        "reason_code": _code(event["reason_code"], "reason_code"),
        "action_codes": _action_codes(event.get("action_codes", [])),
        "correlation_id": _identifier(
            event.get("correlation_id", ""), "correlation_id", allow_empty=True
        ),
        "run_id": _identifier(event.get("run_id", ""), "run_id", allow_empty=True),
        "message": message,
        "details": _details(event.get("details", {})),
        "redaction": _redaction(event.get("redaction"), has_message=bool(message)),
    }

    expected = _content_hash(
        {key: item for key, item in normalized.items() if key != "contract"}
    )
    supplied = event.get("event_hash")
    if supplied not in (None, "") and supplied != expected:
        raise DeviceEventContractError("device event hash does not match its content")
    normalized["event_hash"] = expected

    if len(_canonical_json(normalized).encode("utf-8")) > EVENT_BYTE_LIMIT:
        raise DeviceEventContractError(f"device event exceeds {EVENT_BYTE_LIMIT} bytes")
    return normalized


# -- cursors ------------------------------------------------------------------


def _cursor_tag(journal_id: str, position: int, event_hash: str) -> str:
    return hashlib.sha256(f"{journal_id}:{position}:{event_hash}".encode()).hexdigest()[:16]


def encode_cursor(journal_id: str, position: int, event_hash: str = "") -> str:
    """The opaque position token an upstream reader resumes from.

    It carries the journal's identity and a digest of the record it points at,
    so a cursor from another device, from a journal that was recreated, or from
    a record that no longer says what it said is refused instead of quietly
    resolving to some other position.
    """
    if not isinstance(journal_id, str) or not _JOURNAL_ID.fullmatch(journal_id):
        raise DeviceEventCursorError("journal identity is malformed")
    if isinstance(position, bool) or not isinstance(position, int) or position < 0:
        raise DeviceEventCursorError("cursor position must be a non-negative integer")
    if position == 0:
        return f"dej1:{journal_id}:0:{_EMPTY_TAG}"
    if not isinstance(event_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", event_hash):
        raise DeviceEventCursorError("cursor content digest is malformed")
    return f"dej1:{journal_id}:{position}:{_cursor_tag(journal_id, position, event_hash)}"


def decode_cursor(cursor: str) -> tuple[str, int, str]:
    """Validate an opaque cursor. An unreadable cursor is refused, not reset.

    Silently treating a bad cursor as "start from the beginning" would replay
    the whole journal to a reader that believed it was resuming, which reads
    upstream as a burst of duplicate events with no explanation.
    """
    if cursor in (None, ""):
        return "", 0, _EMPTY_TAG
    if not isinstance(cursor, str):
        raise DeviceEventCursorError("device event cursor must be text")
    matched = _CURSOR.fullmatch(cursor)
    if not matched:
        raise DeviceEventCursorError("device event cursor is malformed")
    return matched.group(1), int(matched.group(2)), matched.group(3)


# -- journal ------------------------------------------------------------------


def _refuse_symlink_ancestors(directory: Path) -> None:
    """Refuse a journal whose path crosses a symlink at any depth.

    Checking only the immediate parent leaves the interesting case open: an
    attacker who owns any ancestor can point the whole subtree somewhere else,
    and every write lands outside the root the caller thought it named.
    """
    resolved = directory if directory.is_absolute() else Path(os.getcwd()) / directory
    walked = Path(resolved.anchor or os.sep)
    for part in resolved.parts[1:] if resolved.anchor else resolved.parts:
        walked = walked / part
        if walked.is_symlink():
            raise DeviceEventJournalError(
                f"journal path crosses a symlink at {walked}; refusing to write through it"
            )


def _bound(value: Any, label: str, *, minimum: int, maximum: int) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise DeviceEventJournalError(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise DeviceEventJournalError(f"{label} must be between {minimum} and {maximum}")
    return value


def _write_all(descriptor: int, data: bytes) -> None:
    """Write every byte, or raise. A short write is not a successful write.

    ``os.write`` is allowed to write less than it was given — on a pipe, on a
    filesystem near its limit, or when a signal lands mid-call.  Treating its
    return as "done" leaves a truncated final line, and this journal refuses a
    file that ends mid-record, so one short write turns every later read and
    every later append into a hard failure.
    """
    view = memoryview(data)
    written = 0
    while written < len(view):
        try:
            count = os.write(descriptor, view[written:])
        except InterruptedError:  # pragma: no cover - retried by PEP 475 first
            continue
        if count <= 0:
            raise DeviceEventJournalError("journal write made no progress")
        written += count


class DeviceEventJournal:
    """An append-only, owner-only, bounded local record of device events.

    Bounded means both ways: at most ``max_events`` records and at most
    ``max_bytes`` on disk, whichever binds first.  Retention drops whole oldest
    records and never rewrites one, so a record that has been read is the record
    that was written.  A dropped record is not forgotten quietly: the next
    export whose cursor pointed before it reports a gap.

    Exclusion is an advisory lock taken on the *directory descriptor* the
    journal lives in, not on a lock file beside it.  A lock file is a name, and
    a name can be unlinked and recreated while a writer holds it — after which
    two writers each hold a valid lock on a different inode and a retention
    rewrite loses whatever the other one appended.  A directory that has been
    opened and verified cannot be swapped out from under the descriptor, and
    every read, write, rename and unlink here is performed relative to that same
    descriptor, so they all provably act on the directory that was checked.
    """

    #: Line 1 of the file. Never dropped by retention; it is what makes a
    #: cursor from this journal distinguishable from a cursor from any other.
    _HEADER_KEY = "journal"

    def __init__(
        self,
        path: str | os.PathLike[str],
        *,
        max_events: int = DEFAULT_MAX_EVENTS,
        max_bytes: int = DEFAULT_MAX_BYTES,
    ) -> None:
        self.path = Path(path)
        self.name = self.path.name
        if not self.name or self.name in (".", ".."):
            raise DeviceEventJournalError("journal path must name a file")
        # Bounds are validated the same way the contract validates an event:
        # exact type, no bool-as-int, hard ceiling. int(1.5) is 1 and int(True)
        # is 1, so a caller who passed either would get a one-record journal and
        # no indication that the number it asked for was not the number it got.
        self.max_events = _bound(
            max_events, "journal max_events", minimum=1, maximum=HARD_MAX_JOURNAL_EVENTS
        )
        self.max_bytes = _bound(
            max_bytes, "journal max_bytes", minimum=1024, maximum=HARD_MAX_JOURNAL_BYTES
        )
        self._partial_name = f".{self.name}.partial"

    # -- public ---------------------------------------------------------------

    def append(self, event: Mapping[str, Any]) -> dict[str, Any]:
        """Record one event and return the stored record.

        The event is validated before anything touches the filesystem: a journal
        must never be the first place an invalid event is discovered.
        """
        normalized = parse_device_event(event)
        with _JournalDirectory(self.path.parent, create=True) as directory:
            header, records, lines = self._load(directory)
            new_journal = header is None
            if new_journal:
                header = {
                    "contract": DEVICE_EVENT_JOURNAL_CONTRACT,
                    "journal_id": os.urandom(16).hex(),
                }
                lines = [_canonical_json({self._HEADER_KEY: header}).encode("utf-8") + b"\n"]
            allocated = (records[-1]["journal_sequence"] + 1) if records else 1
            record = {"journal_sequence": allocated, "event": normalized}
            line = _canonical_json(record).encode("utf-8") + b"\n"
            if len(line) + _HEADER_ALLOWANCE > self.max_bytes:
                raise DeviceEventJournalError("one event is larger than the whole journal bound")
            kept, dropped = self._retain(lines, line)
            # The directory is re-checked against its own pathname immediately
            # before anything is published, not only when it was opened.
            directory.reverify()
            if dropped or new_journal:
                # A first append has to publish the header and the record
                # together. Appending only the record would leave a file whose
                # first line is a record, which every reader here refuses — an
                # append that returned success and destroyed the journal.
                self._rewrite(directory, kept)
            else:
                self._append_line(directory, line)
            return record

    def journal_id(self) -> str:
        """This journal's identity, or ``""`` if nothing has been written yet."""
        with _JournalDirectory(self.path.parent, create=False) as directory:
            header, _records, _lines = self._load(directory)
        return header["journal_id"] if header else ""

    def read_all(self) -> list[dict[str, Any]]:
        """Every stored record, oldest first, fully re-validated."""
        with _JournalDirectory(self.path.parent, create=False) as directory:
            _header, records, _lines = self._load(directory)
        return records

    def export(
        self,
        *,
        cursor: str = "",
        limit: int = DEFAULT_EXPORT_ITEMS,
        max_bytes: int = DEFAULT_EXPORT_BYTES,
    ) -> dict[str, Any]:
        """Records after ``cursor``, under a hard item and byte limit.

        This is the only way anything outside the device is meant to read the
        journal.  Handing over the file instead would hand over whatever else
        ended up in it, and would make retention someone else's problem.

        ``max_bytes`` is a hard limit on the encoded records returned, and hard
        means it is never exceeded: a bound that is quietly overshot is worse
        than no bound, because the caller has already sized something from it.
        It must be at least :data:`MIN_EXPORT_BYTES` — one maximal record — and
        a smaller value raises :class:`DeviceEventBoundError` rather than being
        approximated.  In exchange, while any record is waiting this returns at
        least one, so a reader's cursor always advances.  ``complete`` is
        ``False`` exactly when records were left for the next call.

        ``gap`` is the field that matters.  A reader resuming from a cursor that
        retention has already outrun will be handed the records that survive —
        it cannot be handed the ones that did not — but it is told, in the same
        response, that records between its cursor and the first record here were
        dropped.  Without that, "nothing happened since you last looked" and
        "the journal rolled over while you were away" are the same reply.
        """
        wanted_journal, after, tag = decode_cursor(cursor)
        limit = self._export_limit(limit)
        max_bytes = self._export_bytes(max_bytes)

        with _JournalDirectory(self.path.parent, create=False) as directory:
            header, records, _lines = self._load(directory)

        journal_id = header["journal_id"] if header else ""
        if wanted_journal and wanted_journal != journal_id:
            raise DeviceEventCursorError(
                "cursor was issued by a different device event journal"
            )

        last_sequence = records[-1]["journal_sequence"] if records else 0
        if after > last_sequence:
            raise DeviceEventCursorError(
                "cursor is ahead of this journal; it was not issued by this file"
            )
        self._verify_cursor_tag(records, after=after, tag=tag, journal_id=journal_id)

        selected, remaining = self._select(records, after=after, limit=limit, max_bytes=max_bytes)
        gap_before = 0
        if selected and selected[0]["journal_sequence"] != after + 1:
            gap_before = selected[0]["journal_sequence"]

        if selected:
            last = selected[-1]
            next_cursor = encode_cursor(
                journal_id, last["journal_sequence"], last["event"]["event_hash"]
            )
        else:
            next_cursor = cursor if cursor else encode_cursor(journal_id or "0" * 32, 0)
        return {
            "contract": DEVICE_EVENT_EXPORT_CONTRACT,
            "journal_id": journal_id,
            "records": selected,
            "count": len(selected),
            "next_cursor": next_cursor,
            "complete": not remaining,
            "gap": gap_before != 0,
            "gap_before_sequence": gap_before,
        }

    # -- export helpers -------------------------------------------------------

    @staticmethod
    def _export_limit(limit: Any) -> int:
        if isinstance(limit, bool) or not isinstance(limit, int):
            raise DeviceEventJournalError("export limit must be an integer")
        if not 1 <= limit <= HARD_MAX_EXPORT_ITEMS:
            raise DeviceEventJournalError(
                f"export limit must be between 1 and {HARD_MAX_EXPORT_ITEMS}"
            )
        return limit

    @staticmethod
    def _export_bytes(max_bytes: Any) -> int:
        if isinstance(max_bytes, bool) or not isinstance(max_bytes, int):
            raise DeviceEventJournalError("export byte limit must be an integer")
        if not MIN_EXPORT_BYTES <= max_bytes <= HARD_MAX_EXPORT_BYTES:
            # The floor is not arbitrary: see MIN_EXPORT_BYTES. Below it this
            # method would have to either overshoot the bound or strand the
            # caller's cursor, and refusing is the only answer that is neither.
            raise DeviceEventBoundError(
                f"export byte limit must be between {MIN_EXPORT_BYTES} and "
                f"{HARD_MAX_EXPORT_BYTES}"
            )
        return max_bytes

    @staticmethod
    def _verify_cursor_tag(
        records: list[dict[str, Any]], *, after: int, tag: str, journal_id: str
    ) -> None:
        """If the record the cursor names is still here, it must still match."""
        if after == 0:
            return
        for record in records:
            if record["journal_sequence"] != after:
                continue
            expected = _cursor_tag(journal_id, after, record["event"]["event_hash"])
            if expected != tag:
                raise DeviceEventCursorError(
                    "cursor does not match the record it names; this journal was rewritten"
                )
            return
        # Retention already dropped it. That is reported as a gap by export, not
        # treated as a forged cursor.

    @staticmethod
    def _select(
        records: list[dict[str, Any]], *, after: int, limit: int, max_bytes: int
    ) -> tuple[list[dict[str, Any]], bool]:
        """Records after ``after`` that fit, and whether any were left behind.

        The byte bound is a bound in both directions.  A page never exceeds
        ``max_bytes``, and a page is never empty while a record is waiting: the
        first is what a caller sized its buffer from, the second is what lets a
        cursor advance.  Both hold together only because ``_export_bytes``
        refuses a budget below :data:`MIN_EXPORT_BYTES`, so one record always
        fits.  If a record somehow does not — a hand-edited position far past
        what this contract issues, a future event limit raised without raising
        the floor — that is refused outright rather than resolved by breaking
        whichever of the two promises is cheaper.
        """
        selected: list[dict[str, Any]] = []
        used = 0
        for record in records:
            if record["journal_sequence"] <= after:
                continue
            if len(selected) >= limit:
                return selected, True
            size = record_byte_size(record)
            if used + size > max_bytes:
                if not selected:
                    raise DeviceEventBoundError(
                        f"record {record['journal_sequence']} needs {size} bytes, over the "
                        f"{max_bytes}-byte export bound; raise max_bytes to read past it"
                    )
                return selected, True
            selected.append(record)
            used += size
        return selected, False

    # -- filesystem -----------------------------------------------------------

    def _load(
        self, directory: _JournalDirectory
    ) -> tuple[dict[str, Any] | None, list[dict[str, Any]], list[bytes]]:
        """Header, validated records, and the raw lines, or empties if absent.

        The lines are returned *with* their terminating newline still attached.
        They are not decoration: this file is one JSON object per line, and a
        rotation rewrite joins these same lines back together.  Returning them
        stripped and rejoining them would concatenate ``{...}{...}`` into a
        single unparseable line, so the first append that triggered retention
        would destroy the whole journal — and it would do it silently, because
        the write itself succeeds and only the *next* read fails.
        """
        raw = self._read_bytes(directory)
        if not raw:
            return None, [], []
        if not raw.endswith(b"\n"):
            raise DeviceEventJournalError(
                f"journal {self.path} ends mid-record; refusing to guess where it stops"
            )
        chunks = [chunk for chunk in raw.split(b"\n") if chunk]
        header = _decode_header(chunks[0])
        records = [_decode_record(chunk) for chunk in chunks[1:]]
        _refuse_non_contiguous(records)
        lines = [chunk + b"\n" for chunk in chunks]
        return header, records, lines

    def _read_bytes(self, directory: _JournalDirectory) -> bytes:
        descriptor = directory.open(self.name, os.O_RDONLY, missing_ok=True)
        if descriptor is None:
            return b""
        try:
            info = os.fstat(descriptor)
            _refuse_unsafe_file(info, self.path)
            # Bound the read *before* allocating: a journal that grew past its
            # own limit — by another writer, by a bug, or on purpose — must not
            # be pulled into memory just to find out it is too big.
            allowed = min(self.max_bytes + _HEADER_ALLOWANCE, HARD_MAX_JOURNAL_BYTES)
            if info.st_size > allowed:
                raise DeviceEventJournalError(
                    f"journal {self.path} is {info.st_size} bytes, over its {allowed} bound"
                )
            with os.fdopen(os.dup(descriptor), "rb") as handle:
                return handle.read(allowed + 1)
        finally:
            os.close(descriptor)

    def _append_line(self, directory: _JournalDirectory, line: bytes) -> None:
        descriptor = directory.open(self.name, os.O_WRONLY | os.O_CREAT | os.O_APPEND)
        try:
            _refuse_unsafe_file(os.fstat(descriptor), self.path)
            _write_all(descriptor, line)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _rewrite(self, directory: _JournalDirectory, lines: list[bytes]) -> None:
        directory.unlink(self._partial_name, missing_ok=True)
        descriptor = directory.open(self._partial_name, os.O_WRONLY | os.O_CREAT | os.O_EXCL)
        try:
            _write_all(descriptor, b"".join(lines))
            os.fsync(descriptor)
        except BaseException:
            os.close(descriptor)
            directory.unlink(self._partial_name, missing_ok=True)
            raise
        os.close(descriptor)
        try:
            directory.replace(self._partial_name, self.name)
        except BaseException:
            directory.unlink(self._partial_name, missing_ok=True)
            raise
        directory.sync()

    # -- retention ------------------------------------------------------------

    def _retain(self, lines: list[bytes], line: bytes) -> tuple[list[bytes], bool]:
        """Oldest-first drop until both bounds hold. Deterministic, no clock.

        The header at index 0 is structural, not a record, so it is never a
        retention candidate: dropping it would orphan every cursor ever issued.
        """
        kept = [*lines, line]
        total = sum(len(item) for item in kept)
        dropped = False
        while len(kept) - 1 > self.max_events or total > self.max_bytes:
            if len(kept) <= 2:
                break
            total -= len(kept[1])
            del kept[1]
            dropped = True
        return kept, dropped


def _refuse_unsafe_file(info: os.stat_result, path: Path) -> None:
    if not stat.S_ISREG(info.st_mode):
        raise DeviceEventJournalError(f"journal {path} is not a regular file")
    if os.name != "posix":
        return
    mode = stat.S_IMODE(info.st_mode)
    if mode & 0o077:
        raise DeviceEventJournalError(
            f"journal {path} is mode {mode:04o}; a device journal that group or "
            "others can read is already disclosed"
        )


def _decode_header(line: bytes) -> dict[str, Any]:
    parsed = _decode_json_line(line)
    if not isinstance(parsed, Mapping) or set(parsed) != {DeviceEventJournal._HEADER_KEY}:
        raise DeviceEventJournalError("journal does not begin with its header")
    header = parsed[DeviceEventJournal._HEADER_KEY]
    if not isinstance(header, Mapping) or set(header) != {"contract", "journal_id"}:
        raise DeviceEventJournalError("journal header has unexpected fields")
    if header["contract"] != DEVICE_EVENT_JOURNAL_CONTRACT:
        raise DeviceEventJournalError("unsupported device event journal contract")
    journal_id = header["journal_id"]
    if not isinstance(journal_id, str) or not _JOURNAL_ID.fullmatch(journal_id):
        raise DeviceEventJournalError("journal identity is malformed")
    return {"contract": DEVICE_EVENT_JOURNAL_CONTRACT, "journal_id": journal_id}


def _decode_json_line(line: bytes) -> Any:
    try:
        return json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise DeviceEventJournalError("journal contains a line that is not JSON") from exc


def _decode_record(line: bytes) -> dict[str, Any]:
    parsed = _decode_json_line(line)
    if not isinstance(parsed, Mapping):
        raise DeviceEventJournalError("journal record must be an object")
    if set(parsed) != {"journal_sequence", "event"}:
        raise DeviceEventJournalError("journal record has unexpected fields")
    position = parsed["journal_sequence"]
    if isinstance(position, bool) or not isinstance(position, int) or position < 1:
        raise DeviceEventJournalError("journal record position is invalid")
    try:
        event = parse_device_event(parsed["event"])
    except DeviceEventContractError as exc:
        raise DeviceEventJournalError(f"journal holds an invalid event: {exc}") from exc
    return {"journal_sequence": position, "event": event}


def _refuse_non_contiguous(records: list[dict[str, Any]]) -> None:
    """Positions must step by exactly one. A hole is corruption, not history.

    Retention only ever removes a prefix, so a surviving journal is always
    contiguous.  If it is not, something outside this module edited the file,
    and the difference between "records 4 and 5 were deleted" and "records 4
    and 5 never existed" is not recoverable from what is left.  A duplicate
    position is the same problem seen from the other side: two different events
    would answer to the same cursor.
    """
    previous: int | None = None
    for record in records:
        position = record["journal_sequence"]
        if previous is not None and position != previous + 1:
            raise DeviceEventJournalError(
                f"journal positions jump from {previous} to {position}; it has been edited"
            )
        previous = position


class _JournalDirectory:
    """A verified directory descriptor that also carries the exclusion lock.

    Every filesystem call made through this object is relative to the
    descriptor, so once the directory has been checked for symlink ancestors and
    for loose permissions, nothing can substitute a different directory for it
    part-way through an append or a rotation.
    """

    _dir_fd: int | None

    def __init__(self, directory: Path, *, create: bool) -> None:
        self._directory = directory
        self._create = create
        self._dir_fd = None
        self._locked = False
        self._use_dir_fd = os.open in os.supports_dir_fd and os.name == "posix"

    def __enter__(self) -> _JournalDirectory:
        _refuse_symlink_ancestors(self._directory)
        if self._create:
            self._directory.mkdir(parents=True, exist_ok=True, mode=JOURNAL_DIR_MODE)
            if os.name == "posix":
                # mkdir's mode applies only on creation and umask masks it even
                # then, so an already-loose directory would otherwise stay loose.
                os.chmod(self._directory, JOURNAL_DIR_MODE)
        elif not self._directory.is_dir():
            raise DeviceEventJournalError(f"journal directory {self._directory} does not exist")

        flags = os.O_RDONLY
        for name in ("O_DIRECTORY", "O_NOFOLLOW"):
            flags |= getattr(os, name, 0)
        try:
            self._dir_fd = os.open(self._directory, flags)
        except OSError as exc:
            raise DeviceEventJournalError(
                f"journal directory {self._directory} cannot be opened safely"
            ) from exc
        try:
            self._verify_directory()
            if fcntl is not None:
                fcntl.flock(self._dir_fd, fcntl.LOCK_EX)
                self._locked = True
            # Identity is re-checked *after* the lock is held. A lock taken on a
            # directory inode says nothing about the pathname: rename the
            # directory away and create a fresh one under the same name, and a
            # second writer opens the new inode, takes an uncontended lock on
            # it, and appends — two writers, two directories, one pathname, no
            # exclusion at all. Re-checking here means the displaced writer
            # finds out instead of quietly writing into a detached directory.
            self.reverify()
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *_exc: object) -> None:
        if self._dir_fd is None:
            return
        try:
            if self._locked and fcntl is not None:
                fcntl.flock(self._dir_fd, fcntl.LOCK_UN)
        finally:
            self._locked = False
            os.close(self._dir_fd)
            self._dir_fd = None

    def _verify_directory(self) -> None:
        assert self._dir_fd is not None
        info = os.fstat(self._dir_fd)
        if not stat.S_ISDIR(info.st_mode):
            raise DeviceEventJournalError("journal directory is not a directory")
        if os.name == "posix" and stat.S_IMODE(info.st_mode) & 0o077:
            raise DeviceEventJournalError(
                f"journal directory {self._directory} is mode "
                f"{stat.S_IMODE(info.st_mode):04o}; it must be owner-only"
            )
        self.reverify()

    def reverify(self) -> None:
        """Confirm the pathname still names the directory this descriptor holds.

        Called after the lock is acquired and again immediately before anything
        is published, so a directory that was renamed or replaced part-way
        through an append is refused rather than written into.
        """
        if self._dir_fd is None:
            raise DeviceEventJournalError("journal directory is not open")
        _refuse_symlink_ancestors(self._directory)
        held = os.fstat(self._dir_fd)
        try:
            named = os.stat(self._directory, follow_symlinks=False)
        except OSError as exc:
            raise DeviceEventJournalError(
                f"journal directory {self._directory} no longer exists at its own path"
            ) from exc
        if (named.st_dev, named.st_ino) != (held.st_dev, held.st_ino):
            raise DeviceEventJournalError(
                f"journal directory {self._directory} was replaced while it was in use; "
                "refusing to write into the directory that was displaced"
            )

    # -- descriptor-relative operations ---------------------------------------

    def _kwargs(self) -> dict[str, Any]:
        return {"dir_fd": self._dir_fd} if self._use_dir_fd else {}

    def _resolve(self, name: str) -> Any:
        return name if self._use_dir_fd else self._directory / name

    def open(self, name: str, flags: int, *, missing_ok: bool = False) -> int | None:
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            return os.open(self._resolve(name), flags, JOURNAL_FILE_MODE, **self._kwargs())
        except FileNotFoundError:
            if missing_ok:
                return None
            raise
        except OSError as exc:
            raise DeviceEventJournalError(
                f"journal file {name} cannot be opened safely"
            ) from exc

    def unlink(self, name: str, *, missing_ok: bool = False) -> None:
        try:
            os.unlink(self._resolve(name), **self._kwargs())
        except FileNotFoundError:
            if not missing_ok:
                raise

    def replace(self, source: str, target: str) -> None:
        if self._use_dir_fd and os.replace in os.supports_dir_fd:
            os.replace(source, target, src_dir_fd=self._dir_fd, dst_dir_fd=self._dir_fd)
            return
        os.replace(self._directory / source, self._directory / target)

    def sync(self) -> None:
        if self._dir_fd is not None:
            os.fsync(self._dir_fd)
