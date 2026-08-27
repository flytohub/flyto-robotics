"""Focused tests for the generic device-event contract and its journal.

The bug these were written around is worth stating, because it is the kind that
a happy-path test cannot see: retention read the journal back with the line
terminators stripped and then rejoined the survivors with nothing between them,
so the first rotation wrote ``{...}{...}`` as one line.  The append that did it
returned success.  Only the *next* read failed, on a device, with the records
already unrecoverable.  So the rotation tests below assert on the bytes on disk
and on a re-read through a fresh journal object, not just on the return value of
the call that wrote them.

Every temporary path is ``.resolve()``d.  On macOS ``tempfile`` hands back
``/var/folders/...`` and ``/var`` is a symlink to ``/private/var``; this module
refuses to write through a symlinked ancestor, correctly, so an unresolved
``tmp_path`` would fail every filesystem test here for a reason that has nothing
to do with what is being tested.
"""

from __future__ import annotations

import importlib.util
import json
import os
import re
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from flyto_robotics import device_event_cli, device_events
from flyto_robotics.cli import validate_assets
from flyto_robotics.device_events import (
    ACTION_CODE_LIMIT,
    DEFAULT_EXPORT_BYTES,
    DEFAULT_JOURNAL_FILENAME,
    DETAILS_KEY_LIMIT,
    DEVICE_EVENT_CONTRACT,
    DEVICE_EVENT_EXPORT_CONTRACT,
    DEVICE_EVENT_JOURNAL_CONTRACT,
    DEVICE_EVENT_JOURNAL_ENV,
    EVENT_BYTE_LIMIT,
    HARD_MAX_EXPORT_BYTES,
    HARD_MAX_EXPORT_ITEMS,
    HARD_MAX_JOURNAL_BYTES,
    HARD_MAX_JOURNAL_EVENTS,
    MESSAGE_LIMIT,
    MIN_EXPORT_BYTES,
    RECORD_FRAMING_BYTES,
    REDACTION_POLICY,
    SEQUENCE_LIMIT,
    SEVERITIES,
    STATUSES,
    DeviceEventBoundError,
    DeviceEventContractError,
    DeviceEventCursorError,
    DeviceEventJournal,
    DeviceEventJournalError,
    build_device_event,
    canonical_json,
    decode_cursor,
    encode_cursor,
    export_page_bytes,
    is_sensitive_key,
    now_observed_at,
    parse_device_event,
    record_byte_size,
)
from flyto_robotics.fsio import atomic_write

JOURNAL_BYTES = 64 * 1024

# Reached through the module rather than imported by name: these are internals,
# and a test that names them in its import list reads like they are API.
_JournalDirectory = device_events._JournalDirectory
_write_all = device_events._write_all


def event(sequence: int, **changes: object) -> dict:
    """One valid event. ``sequence`` also varies the timestamp, so every event
    derives a distinct ``event_id`` and a distinct content hash."""
    fields = {
        "resource_id": "flyto-rover-sim-001",
        "component": "diagnostics",
        "sequence": sequence,
        "observed_at": f"2026-08-10T00:{sequence % 60:02d}:00.000000Z",
        "severity": "warning",
        "status": "degraded",
        "reason_code": "network_unreachable",
        "action_codes": ["check_uplink"],
        "message": "This device cannot reach the cloud.",
    }
    fields.update(changes)
    return build_device_event(**fields)  # type: ignore[arg-type]


@pytest.fixture()
def journal_path(tmp_path: Path) -> Path:
    # A subdirectory the journal creates itself, so it owns the 0700 mode.
    return tmp_path.resolve() / "state" / "events.ndjson"


@pytest.mark.skipif(os.name != "posix", reason="file modes are POSIX-only here")
def test_atomic_write_is_private_from_creation_even_with_a_permissive_umask(tmp_path: Path):
    target = tmp_path / "state" / "result.json"
    previous = os.umask(0)
    try:
        atomic_write(target, "synthetic", 0o600)
    finally:
        os.umask(previous)
    assert target.stat().st_mode & 0o777 == 0o600


def test_atomic_write_does_not_reuse_an_attacker_planted_fixed_temporary(tmp_path: Path):
    target = tmp_path / "result.json"
    planted = tmp_path / ".result.json.tmp"
    planted.write_text("attacker-owned", encoding="utf-8")
    atomic_write(target, "trusted", 0o600)
    assert target.read_text(encoding="utf-8") == "trusted"
    assert planted.read_text(encoding="utf-8") == "attacker-owned"


def test_runner_never_logs_secret_bearing_completion_detail(monkeypatch, tmp_path, caplog):
    runner_path = Path(__file__).resolve().parents[1] / "deploy" / "flyto_job_runner.py"
    monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", str(tmp_path / "runner"))
    spec = importlib.util.spec_from_file_location("flyto_job_runner_log_test", runner_path)
    assert spec and spec.loader
    runner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(runner)
    monkeypatch.setattr(runner, "_post", lambda *_args, **_kwargs: {"ok": True})

    secret = "Bearer synthetic-secret-that-must-not-be-logged"
    with caplog.at_level("INFO", logger="flyto.job_runner"):
        runner._report_completion(
            {"device_id": "synthetic-device"},
            job_id="synthetic-job",
            headers={},
            body={
                "status": "success",
                "variables": {
                    "detail": secret,
                    "evidence": [{"kind": "mission_summary"}],
                },
            },
        )
    assert secret not in caplog.text
    assert "synthetic-job reported success" in caplog.text
    assert "mission_summary" in caplog.text


def rotating(path: Path, *, max_events: int = 2) -> DeviceEventJournal:
    return DeviceEventJournal(path, max_events=max_events, max_bytes=JOURNAL_BYTES)


def sequences(records: list[dict]) -> list[int]:
    return [record["journal_sequence"] for record in records]


def cursor_for(journal: DeviceEventJournal, record: dict) -> str:
    return encode_cursor(
        journal.journal_id(), record["journal_sequence"], record["event"]["event_hash"]
    )


# -- contract -----------------------------------------------------------------


def test_build_and_parse_round_trip_binds_a_stable_hash_and_id():
    built = event(1)
    assert built["contract"] == DEVICE_EVENT_CONTRACT
    assert built["event_id"].startswith("evt-")
    assert built["redaction"]["policy"] == REDACTION_POLICY
    assert built["redaction"]["free_text"] is True
    # Re-parsing a stored event must be a fixed point, or a journal read would
    # not be able to re-validate what a journal write produced.
    assert parse_device_event(built) == built
    assert event(1)["event_id"] == built["event_id"]
    assert event(2)["event_id"] != built["event_id"]


def test_supplied_event_hash_that_disagrees_with_the_content_is_refused():
    tampered = dict(event(1))
    tampered["event_hash"] = "0" * 64
    with pytest.raises(DeviceEventContractError, match="hash does not match"):
        parse_device_event(tampered)


# -- timestamps ---------------------------------------------------------------


def test_aware_timestamp_is_converted_to_utc_not_relabelled():
    # 18:00+08:00 is 10:00Z. Stamping it as 18:00Z because both are
    # timestamp-shaped moves the event eight hours into the future and every
    # ordering built on it afterwards is wrong.
    moment = datetime(2026, 8, 10, 18, 0, 0, tzinfo=timezone(timedelta(hours=8)))
    assert now_observed_at(moment) == "2026-08-10T10:00:00.000000Z"


def test_naive_timestamp_is_taken_as_utc_and_the_clock_default_is_acceptable():
    naive = datetime(2026, 8, 10, 18, 0, 0)
    assert now_observed_at(naive) == "2026-08-10T18:00:00.000000Z"
    # The no-argument form must produce something this contract accepts.
    event(1, observed_at=now_observed_at())


@pytest.mark.parametrize(
    "value",
    [
        "2026-08-10T00:00:00+00:00",  # offset dialect
        "2026-08-10 00:00:00Z",  # space separator
        "2026-08-10T00:00:00z",  # lowercase designator
        "2026-02-30T00:00:00Z",  # shaped like a date, is not a day
        "2026-08-10T00:00:60Z",  # leap second datetime cannot hold
    ],
)
def test_non_canonical_or_impossible_timestamps_are_refused(value: str):
    with pytest.raises(DeviceEventContractError, match="observed_at"):
        event(1, observed_at=value)


# -- privacy ------------------------------------------------------------------


@pytest.mark.parametrize(
    ("name", "sensitive"),
    [
        ("apiKey", True),
        ("password", True),
        ("session_key", True),
        ("auth_token", True),
        ("token", True),
        ("patient_id", True),
        ("home_address", True),
        # The false positives that get a substring-based check switched off.
        ("wifi_has_address", False),
        ("unknown_service_ids", False),
        ("token_budget", False),
        ("token_count", False),
        ("service_states", False),
    ],
)
def test_sensitive_key_detection_is_word_based(name: str, sensitive: bool):
    assert is_sensitive_key(name) is sensitive


def test_details_key_that_names_a_credential_is_refused_not_dropped():
    with pytest.raises(DeviceEventContractError, match="names a credential"):
        event(1, details={"uplink": {"api_key": "x"}})


@pytest.mark.parametrize(
    "value",
    [
        "eyJhbGciOiJIUzI1NiJ9.eyJzdWIiOiIxMjM0NTY3ODkwIn0.dBjftJeZ4CVPmB92K27u",
        "Bearer abcdefghijklmnopqrstuvwxyz012345",
        "-----BEGIN RSA PRIVATE KEY-----",
        "password = hunter2",
        "AKIAIOSFODNN7EXAMPLE",
        "123-45-6789",
    ],
)
def test_values_that_look_like_secrets_are_refused_in_message_and_details(value: str):
    with pytest.raises(DeviceEventContractError, match="looks like a secret"):
        event(1, message=value)
    with pytest.raises(DeviceEventContractError, match="looks like a secret"):
        event(1, details={"note": value})


def test_redaction_cannot_claim_to_carry_or_to_have_scrubbed_anything():
    base = dict(event(1))
    for field in ("raw_logs_included", "credentials_included", "personal_data_included"):
        claim = dict(base)
        claim["redaction"] = {**base["redaction"], field: True}
        with pytest.raises(DeviceEventContractError, match=field):
            parse_device_event(claim)
    # This contract refuses rather than redacts, so a non-zero count is a claim
    # about scrubbing that no code here performs.
    counted = dict(base)
    counted["redaction"] = {**base["redaction"], "redacted_key_count": 1}
    with pytest.raises(DeviceEventContractError, match="redacted_key_count"):
        parse_device_event(counted)


def test_redaction_free_text_must_agree_with_whether_there_is_a_message():
    base = dict(event(1, message=""))
    lying = dict(base)
    lying["redaction"] = {**base["redaction"], "free_text": True}
    with pytest.raises(DeviceEventContractError, match="free_text does not match"):
        parse_device_event(lying)


# -- journal: fresh round trip ------------------------------------------------


def test_fresh_journal_round_trips_records_and_exports_them_complete(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    written = [journal.append(event(index)) for index in (1, 2, 3)]
    assert sequences(written) == [1, 2, 3]

    # A *fresh* object, so nothing is being served from in-process state.
    reread = rotating(journal_path, max_events=8)
    assert sequences(reread.read_all()) == [1, 2, 3]
    assert reread.read_all() == written

    export = reread.export()
    assert export["contract"] == DEVICE_EVENT_EXPORT_CONTRACT
    assert export["journal_id"] == reread.journal_id()
    assert export["count"] == 3
    assert export["complete"] is True
    assert export["gap"] is False
    assert export["gap_before_sequence"] == 0

    # Resuming from the cursor it just handed out yields nothing further.
    resumed = reread.export(cursor=export["next_cursor"])
    assert resumed["count"] == 0
    assert resumed["complete"] is True
    assert resumed["gap"] is False


def test_first_append_publishes_the_header_and_the_record_together(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    lines = journal_path.read_bytes().splitlines()
    assert len(lines) == 2
    header = json.loads(lines[0])["journal"]
    assert header["contract"] == DEVICE_EVENT_JOURNAL_CONTRACT
    assert len(header["journal_id"]) == 32
    assert json.loads(lines[1])["journal_sequence"] == 1


def test_export_limit_paginates_and_reports_incompleteness(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    for index in (1, 2, 3):
        journal.append(event(index))
    first = journal.export(limit=2)
    assert sequences(first["records"]) == [1, 2]
    assert first["complete"] is False
    second = journal.export(cursor=first["next_cursor"], limit=2)
    assert sequences(second["records"]) == [3]
    assert second["complete"] is True
    assert second["gap"] is False


# -- journal: retention rotation ----------------------------------------------


def test_rotation_keeps_the_newest_records_and_the_file_stays_parseable(journal_path: Path):
    """The regression. Four appends at max_events=2 must leave [3, 4] readable.

    Asserted on the bytes and through a fresh object, because the delimiter loss
    this covers produced a file that the writing process never re-read.
    """
    journal = rotating(journal_path, max_events=2)
    seen = []
    for index in (1, 2, 3, 4):
        journal.append(event(index))
        seen.append(sequences(journal.read_all()))
    assert seen == [[1], [1, 2], [2, 3], [3, 4]]

    raw = journal_path.read_bytes()
    assert raw.endswith(b"\n")
    lines = raw.split(b"\n")[:-1]
    assert len(lines) == 3, "header plus exactly two surviving records"
    # Every line is independently parseable: this is what a concatenated
    # rewrite destroys, and what a length check alone would not notice.
    decoded = [json.loads(line) for line in lines]
    assert set(decoded[0]) == {"journal"}
    assert [item["journal_sequence"] for item in decoded[1:]] == [3, 4]

    reread = rotating(journal_path, max_events=2)
    assert sequences(reread.read_all()) == [3, 4]
    assert [record["event"]["sequence"] for record in reread.read_all()] == [3, 4]


def test_rotation_never_drops_the_header_so_journal_identity_survives(journal_path: Path):
    journal = rotating(journal_path, max_events=1)
    journal.append(event(1))
    identity = journal.journal_id()
    for index in (2, 3, 4, 5):
        journal.append(event(index))
    assert journal.journal_id() == identity
    assert sequences(journal.read_all()) == [5]


def test_appends_continue_cleanly_after_a_rotation(journal_path: Path):
    # A rotation republishes the whole file; the next append takes the plain
    # append path again and must not corrupt what the rewrite just wrote.
    journal = rotating(journal_path, max_events=2)
    for index in (1, 2, 3, 4, 5, 6):
        journal.append(event(index))
    assert sequences(journal.read_all()) == [5, 6]
    assert len(journal_path.read_bytes().split(b"\n")[:-1]) == 3


def test_load_returns_lines_with_their_terminators_attached(journal_path: Path):
    # The direct statement of the invariant the rewrite path depends on.
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    journal.append(event(2))
    with _JournalDirectory(journal_path.parent, create=False) as directory:
        _header, records, lines = journal._load(directory)
    assert len(lines) == 3
    assert all(line.endswith(b"\n") for line in lines)
    assert b"".join(lines) == journal_path.read_bytes()
    assert sequences(records) == [1, 2]


def test_an_event_larger_than_the_whole_bound_is_refused_not_rotated_away(journal_path: Path):
    journal = DeviceEventJournal(journal_path, max_events=8, max_bytes=4200)
    with pytest.raises(DeviceEventJournalError, match="larger than the whole journal bound"):
        journal.append(event(1))


# -- journal: cursors and gaps ------------------------------------------------


def test_cursor_outrun_by_retention_reports_a_gap_and_the_first_surviving_sequence(
    journal_path: Path,
):
    journal = rotating(journal_path, max_events=2)
    first = journal.append(event(1))
    stale = cursor_for(journal, first)
    for index in (2, 3, 4):
        journal.append(event(index))
    assert sequences(journal.read_all()) == [3, 4]

    export = journal.export(cursor=stale)
    assert sequences(export["records"]) == [3, 4]
    assert export["gap"] is True
    assert export["gap_before_sequence"] == 3, "record 2 is gone and the reader must be told"
    assert export["complete"] is True


def test_a_cursor_whose_successor_survived_reports_no_gap(journal_path: Path):
    # "You are up to date" and "the journal rolled over" must not look alike —
    # and equally, a survivable resume must not be reported as data loss.
    journal = rotating(journal_path, max_events=2)
    journal.append(event(1))
    second = journal.append(event(2))
    resumable = cursor_for(journal, second)
    for index in (3, 4):
        journal.append(event(index))
    export = journal.export(cursor=resumable)
    assert sequences(export["records"]) == [3, 4]
    assert export["gap"] is False
    assert export["gap_before_sequence"] == 0


def test_cursor_at_the_end_of_a_rotated_journal_returns_nothing_and_no_gap(journal_path: Path):
    journal = rotating(journal_path, max_events=2)
    for index in (1, 2, 3):
        journal.append(event(index))
    latest = journal.export()
    assert sequences(latest["records"]) == [2, 3]
    assert latest["gap"] is True and latest["gap_before_sequence"] == 2
    idle = journal.export(cursor=latest["next_cursor"])
    assert idle["count"] == 0
    assert idle["gap"] is False


def test_empty_cursor_decodes_to_the_beginning_but_a_bad_one_is_refused():
    assert decode_cursor("") == ("", 0, "0" * 16)
    for bad in ("nope", "dej1:zz:1:" + "0" * 16, "dej1:" + "a" * 32 + ":1:short", 5, object()):
        with pytest.raises(DeviceEventCursorError):
            decode_cursor(bad)  # type: ignore[arg-type]


def test_malformed_cursor_is_refused_rather_than_reset_to_the_beginning(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    with pytest.raises(DeviceEventCursorError, match="malformed"):
        journal.export(cursor="dej1:not-a-journal:1:0000000000000000")


def test_cursor_from_another_journal_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    record = journal.append(event(1))
    foreign = encode_cursor("f" * 32, 1, record["event"]["event_hash"])
    with pytest.raises(DeviceEventCursorError, match="different device event journal"):
        journal.export(cursor=foreign)


def test_cursor_ahead_of_the_journal_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    record = journal.append(event(1))
    ahead = encode_cursor(journal.journal_id(), 99, record["event"]["event_hash"])
    with pytest.raises(DeviceEventCursorError, match="ahead of this journal"):
        journal.export(cursor=ahead)


def test_cursor_whose_tag_does_not_match_the_record_it_names_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    record = journal.append(event(1))
    valid = cursor_for(journal, record)
    prefix, tag = valid[: -len("0" * 16)], valid[-16:]
    tampered = prefix + ("1" if tag[0] == "0" else "0") + tag[1:]
    assert tampered != valid
    with pytest.raises(DeviceEventCursorError, match="does not match the record it names"):
        journal.export(cursor=tampered)


def test_encode_cursor_refuses_a_malformed_identity_or_position():
    with pytest.raises(DeviceEventCursorError, match="journal identity"):
        encode_cursor("nope", 1, "a" * 64)
    with pytest.raises(DeviceEventCursorError, match="non-negative integer"):
        encode_cursor("a" * 32, -1, "a" * 64)
    with pytest.raises(DeviceEventCursorError, match="non-negative integer"):
        encode_cursor("a" * 32, True, "a" * 64)  # noqa: FBT003 - bool is not a position
    with pytest.raises(DeviceEventCursorError, match="content digest"):
        encode_cursor("a" * 32, 1, "short")


# -- journal: corruption ------------------------------------------------------


def rewrite_raw(path: Path, lines: list[bytes]) -> None:
    """Replace the journal with exactly these lines, owner-only."""
    path.write_bytes(b"".join(line + b"\n" for line in lines))
    path.chmod(0o600)


def journal_lines(path: Path) -> list[bytes]:
    return path.read_bytes().split(b"\n")[:-1]


def test_duplicate_positions_are_refused_as_corruption(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    header, record = journal_lines(journal_path)
    rewrite_raw(journal_path, [header, record, record])
    with pytest.raises(DeviceEventJournalError, match="positions jump from 1 to 1"):
        journal.read_all()


def test_a_hole_in_the_positions_is_refused_as_corruption(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    for index in (1, 2, 3):
        journal.append(event(index))
    header, first, _second, third = journal_lines(journal_path)
    rewrite_raw(journal_path, [header, first, third])
    with pytest.raises(DeviceEventJournalError, match="positions jump from 1 to 3"):
        journal.read_all()


def test_concatenated_records_on_one_line_are_refused(journal_path: Path):
    # Exactly the shape the delimiter-loss bug produced. It must never be read
    # as valid, so that if it is ever reintroduced a read fails loudly.
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    journal.append(event(2))
    header, first, second = journal_lines(journal_path)
    rewrite_raw(journal_path, [header, first + second])
    with pytest.raises(DeviceEventJournalError, match="not JSON"):
        journal.read_all()


def test_a_file_that_ends_mid_record_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    truncated = journal_path.read_bytes()[:-4]
    journal_path.write_bytes(truncated)
    journal_path.chmod(0o600)
    with pytest.raises(DeviceEventJournalError, match="ends mid-record"):
        journal.read_all()


def test_a_file_that_does_not_begin_with_its_header_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    _header, record = journal_lines(journal_path)
    rewrite_raw(journal_path, [record])
    with pytest.raises(DeviceEventJournalError, match="does not begin with its header"):
        journal.read_all()


def test_a_stored_event_that_no_longer_validates_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    header, record = journal_lines(journal_path)
    parsed = json.loads(record)
    parsed["event"]["severity"] = "catastrophic"
    rewrite_raw(journal_path, [header, json.dumps(parsed).encode("utf-8")])
    with pytest.raises(DeviceEventJournalError, match="invalid event"):
        journal.read_all()


# -- journal: bounds ----------------------------------------------------------


@pytest.mark.parametrize(
    ("kwargs", "match"),
    [
        ({"max_events": 0}, "max_events"),
        ({"max_events": -1}, "max_events"),
        ({"max_events": True}, "max_events"),
        ({"max_events": 1.5}, "max_events"),
        ({"max_events": "8"}, "max_events"),
        ({"max_events": HARD_MAX_JOURNAL_EVENTS + 1}, "max_events"),
        ({"max_bytes": 1023}, "max_bytes"),
        ({"max_bytes": False}, "max_bytes"),
        ({"max_bytes": 4096.0}, "max_bytes"),
        ({"max_bytes": HARD_MAX_JOURNAL_BYTES + 1}, "max_bytes"),
    ],
)
def test_constructor_bounds_are_strict_about_type_and_range(
    journal_path: Path, kwargs: dict, match: str
):
    # int(True) is 1 and int(1.5) is 1: a coercing constructor would hand back a
    # one-record journal and never say the number asked for was not the number
    # given.
    with pytest.raises(DeviceEventJournalError, match=match):
        DeviceEventJournal(journal_path, **kwargs)


@pytest.mark.parametrize("name", ["", "..", "."])
def test_a_path_that_does_not_name_a_file_is_refused(name: str):
    with pytest.raises(DeviceEventJournalError, match="must name a file"):
        DeviceEventJournal(name)


@pytest.mark.parametrize("limit", [0, -1, True, 2.0, 1001])
def test_export_limit_is_strict(journal_path: Path, limit: object):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    with pytest.raises(DeviceEventJournalError, match="export limit"):
        journal.export(limit=limit)  # type: ignore[arg-type]


@pytest.mark.parametrize(
    "max_bytes",
    [1023, 1024, MIN_EXPORT_BYTES - 1, True, 4096.0, 1024 * 1024 + 1],
)
def test_export_byte_limit_is_strict(journal_path: Path, max_bytes: object):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    with pytest.raises(DeviceEventJournalError, match="export byte limit"):
        journal.export(max_bytes=max_bytes)  # type: ignore[arg-type]


# -- journal: the export byte bound is a bound ---------------------------------
#
# The bug these were written around: export(max_bytes=1024) returned a single
# 2664-byte record. The module docstring and this class both promise a hard byte
# limit, and DeviceEventJournal is public API -- so the CLI refusing the same
# value was a patch on one caller, not on the promise. The floor now lives here,
# beside the code that has to honour it, and these test it through the public
# method rather than through the command.


def bulky(index: int, keys: int = 16) -> dict:
    """One event padded to near the contract's per-event ceiling."""
    padding = {f"pad{item:02d}": "x" * 500 for item in range(keys)}
    return event(index, details=padding)


def stocked_bulky(path: Path, count: int) -> DeviceEventJournal:
    journal = DeviceEventJournal(path, max_events=64, max_bytes=HARD_MAX_JOURNAL_BYTES)
    for index in range(1, count + 1):
        journal.append(bulky(index))
    return journal


def page_bytes(document: dict) -> int:
    """What a page costs, counted the way the selector budgets it."""
    return sum(len(canonical_json(record).encode("utf-8")) + 1 for record in document["records"])


def test_the_export_floor_is_one_maximal_record_and_the_defaults_clear_it():
    assert MIN_EXPORT_BYTES == EVENT_BYTE_LIMIT + RECORD_FRAMING_BYTES
    assert "MIN_EXPORT_BYTES" in device_events.__all__
    assert "RECORD_FRAMING_BYTES" in device_events.__all__
    # A default that its own validator would reject is a trap for every caller
    # that never passes the argument.
    assert MIN_EXPORT_BYTES <= DEFAULT_EXPORT_BYTES <= HARD_MAX_EXPORT_BYTES


def test_a_byte_bound_below_the_floor_is_a_typed_refusal_not_an_overshoot(
    journal_path: Path,
):
    """The exact reported case, at the public API: 1024 in, no 2664 out."""
    journal = stocked_bulky(journal_path, 2)
    with pytest.raises(DeviceEventBoundError, match="export byte limit"):
        journal.export(max_bytes=1024)
    # Typed, so a caller can tell "raise your bound" from "this journal is
    # broken" -- and still a journal error, so existing handlers keep working.
    assert issubclass(DeviceEventBoundError, DeviceEventJournalError)


def test_a_record_too_large_for_the_bound_is_refused_rather_than_overshot(
    journal_path: Path,
):
    """The selector's own guarantee, reached directly.

    ``_export_bytes`` makes this unreachable through ``export`` while the floor
    holds. It is pinned anyway: if a future event limit rises without the floor
    rising with it, this must still refuse rather than quietly hand back a page
    larger than the caller asked for.
    """
    journal = stocked_bulky(journal_path, 2)
    records = journal.read_all()
    with pytest.raises(DeviceEventBoundError, match="over the"):
        DeviceEventJournal._select(records, after=0, limit=10, max_bytes=1024)


def test_the_public_export_never_overshoots_never_stalls_and_never_drops(
    journal_path: Path,
):
    """Walk a journal of maximal records at the floor, through the public API.

    Three failure modes at once: a page over the bound (the reported bug), an
    empty page while records remain (a livelock -- the reader's cursor would
    never advance), and a record that appears on no page at all (silent loss).
    """
    total = 6
    journal = stocked_bulky(journal_path, total)

    seen: list[int] = []
    cursor = ""
    for _page in range(total + 2):
        document = journal.export(cursor=cursor, max_bytes=MIN_EXPORT_BYTES)
        assert page_bytes(document) <= MIN_EXPORT_BYTES, "a page exceeded the bound"
        assert document["gap"] is False
        if document["complete"] and not document["records"]:
            break
        assert document["records"], "records remained but the page was empty"
        seen.extend(sequences(document["records"]))
        assert document["next_cursor"] != cursor, "the cursor must move"
        cursor = document["next_cursor"]
    else:  # pragma: no cover - only reached if the walk never terminates
        pytest.fail("the export never reported itself complete")

    assert seen == list(range(1, total + 1)), "every record exactly once, in order"


def test_one_maximal_record_really_does_fit_the_floor(journal_path: Path):
    """The floor is only correct if the largest record this contract permits
    fits inside it. Asserted on a real record, not on the arithmetic."""
    journal = stocked_bulky(journal_path, 1)
    document = journal.export(max_bytes=MIN_EXPORT_BYTES)
    assert document["count"] == 1
    assert page_bytes(document) <= MIN_EXPORT_BYTES
    stored = canonical_json(journal.read_all()[0]).encode("utf-8")
    assert len(stored) + 1 <= MIN_EXPORT_BYTES


# -- byte accounting: one arithmetic, shared by every caller --------------------
#
# ``record_byte_size`` and ``export_page_bytes`` are the contract's own
# accounting: the canonical encoding of a record plus the one newline it occupies
# in the file. The selector budgets a page with it, the journal's public export
# stops on it, and the command re-measures a page with it before printing. Two
# callers that each kept their own sum would agree on every page except the one
# sitting exactly on the bound -- the only page a bound exists for. These pin the
# agreement at that boundary, and pin the refusals the helpers owe a caller that
# hands them something that is not an export page at all.


def test_every_caller_measures_a_page_with_the_same_arithmetic(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    for index in (1, 2, 3):
        journal.append(event(index))
    records = journal.read_all()
    sizes = [record_byte_size(record) for record in records]

    document = journal.export()
    assert sequences(document["records"]) == [1, 2, 3]
    # The stored record, the page total, the public helper and the command all
    # arrive at the same number for the same bytes.
    assert export_page_bytes(document) == sum(sizes)
    assert device_event_cli.record_bytes(document) == sum(sizes)
    assert page_bytes(document) == sum(sizes)


def test_the_selector_splits_a_page_exactly_where_that_arithmetic_says(journal_path: Path):
    """The boundary case: a budget of exactly two records must take two.

    One byte less must take one. An accounting that forgot the per-record
    newline, or counted the envelope, would put the split somewhere else and
    would still look right on every page that was not up against the bound.
    """
    journal = rotating(journal_path, max_events=8)
    for index in (1, 2, 3):
        journal.append(event(index))
    records = journal.read_all()
    sizes = [record_byte_size(record) for record in records]
    exact = sizes[0] + sizes[1]

    selected, truncated = DeviceEventJournal._select(records, after=0, limit=10, max_bytes=exact)
    assert sequences(selected) == [1, 2]
    assert truncated is True
    assert sum(record_byte_size(record) for record in selected) == exact

    tighter, truncated = DeviceEventJournal._select(
        records, after=0, limit=10, max_bytes=exact - 1
    )
    assert sequences(tighter) == [1]
    assert truncated is True


def test_the_public_export_fills_a_page_up_to_that_same_arithmetic(journal_path: Path):
    """A page must stop only because the next record genuinely does not fit.

    Checked on maximal records at the floor, where the page really is bounded by
    bytes rather than by the item limit or by running out of records.
    """
    journal = stocked_bulky(journal_path, 4)
    records = journal.read_all()
    page = journal.export(max_bytes=MIN_EXPORT_BYTES)
    taken = len(page["records"])
    assert taken >= 1, "the floor must always admit one record"
    used = export_page_bytes(page)
    assert used <= MIN_EXPORT_BYTES
    assert used == device_event_cli.record_bytes(page)
    if taken < len(records):
        following = record_byte_size(records[taken])
        assert used + following > MIN_EXPORT_BYTES, "the page stopped short of its own bound"


@pytest.mark.parametrize("record", ["a record", b"a record", 5, None, [("event", {})], 2.5])
def test_a_record_that_is_not_an_object_is_a_typed_refusal(record: object):
    # Typed, not a TypeError from inside json: a caller measuring a document it
    # did not build has to tell "this is not an export page" from its own bug.
    with pytest.raises(DeviceEventContractError, match="must be an object"):
        record_byte_size(record)  # type: ignore[arg-type]


@pytest.mark.parametrize("document", ["a document", b"a document", 5, None, [{"event": {}}], 2.5])
def test_a_document_that_is_not_an_object_is_a_typed_refusal(document: object):
    with pytest.raises(DeviceEventContractError, match="must be an object"):
        export_page_bytes(document)  # type: ignore[arg-type]
    with pytest.raises(DeviceEventContractError, match="must be an object"):
        device_event_cli.record_bytes(document)  # type: ignore[arg-type]


def test_a_document_without_records_is_refused_rather_than_measured_as_empty():
    # Zero would be a lie a caller cannot detect: an envelope missing its records
    # is a malformed page, not a page that happens to carry nothing.
    stunted = {"contract": DEVICE_EVENT_EXPORT_CONTRACT, "count": 0}
    with pytest.raises(DeviceEventContractError, match="no records"):
        export_page_bytes(stunted)
    with pytest.raises(DeviceEventContractError, match="no records"):
        device_event_cli.record_bytes(stunted)


@pytest.mark.parametrize("records", ["", "records", 5, None, {"1": {"event": {}}}, True])
def test_a_records_container_that_is_not_a_list_is_a_typed_refusal(records: object):
    with pytest.raises(DeviceEventContractError, match="must be a list"):
        export_page_bytes({"records": records})
    with pytest.raises(DeviceEventContractError, match="must be a list"):
        device_event_cli.record_bytes({"records": records})


class Unencodable:
    """A value canonical JSON has no encoding for."""


@pytest.mark.parametrize(
    "record",
    [
        {"journal_sequence": 1, "event": Unencodable()},
        {"journal_sequence": 1, "event": {1: "int key", "b": "str key"}},  # unsortable keys
    ],
)
def test_a_record_canonical_json_cannot_encode_is_a_typed_refusal(record: dict):
    """No size exists in the only encoding this contract recognises, so there is
    no number to return -- and the caller is told that, not handed a TypeError
    or a KeyError from inside the encoder."""
    with pytest.raises(DeviceEventContractError, match="canonical JSON"):
        record_byte_size(record)
    with pytest.raises(DeviceEventContractError, match="canonical JSON"):
        export_page_bytes({"records": [record]})
    with pytest.raises(DeviceEventContractError, match="canonical JSON"):
        device_event_cli.record_bytes({"records": [record]})


def test_a_caller_supplied_page_over_the_item_ceiling_is_refused():
    """The journal never issues such a page -- its own limit is checked first --
    but these helpers are public and measure whatever they are handed."""
    record = {"journal_sequence": 1, "event": {}}
    at_the_ceiling = {"records": [record] * HARD_MAX_EXPORT_ITEMS}
    assert export_page_bytes(at_the_ceiling) == HARD_MAX_EXPORT_ITEMS * record_byte_size(record)

    over = {"records": [record] * (HARD_MAX_EXPORT_ITEMS + 1)}
    with pytest.raises(DeviceEventContractError, match=str(HARD_MAX_EXPORT_ITEMS)):
        export_page_bytes(over)
    with pytest.raises(DeviceEventContractError, match=str(HARD_MAX_EXPORT_ITEMS)):
        device_event_cli.record_bytes(over)


def test_the_command_measures_a_page_by_delegating_not_by_summing_it_again(monkeypatch):
    """A second sum that happens to agree today is the drift this prevents."""
    document = {"records": [{"journal_sequence": 1, "event": {"note": "x"}}]}
    assert device_event_cli.record_bytes(document) == export_page_bytes(document)
    # Replace the shared helper and the command's answer changes with it: the
    # sum is the contract's, not a copy the command keeps.
    monkeypatch.setattr(device_event_cli, "export_page_bytes", lambda _document: 4242)
    assert device_event_cli.record_bytes(document) == 4242


def test_a_journal_grown_past_its_own_bound_is_refused_before_it_is_read(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    with journal_path.open("ab") as handle:
        handle.write(b"x" * (JOURNAL_BYTES + 4096))
    with pytest.raises(DeviceEventJournalError, match="over its"):
        journal.read_all()


# -- journal: permissions and path safety -------------------------------------


@pytest.mark.skipif(os.name != "posix", reason="POSIX permission semantics")
def test_a_group_or_world_readable_journal_is_refused(journal_path: Path):
    journal = rotating(journal_path, max_events=8)
    journal.append(event(1))
    assert journal_path.stat().st_mode & 0o777 == 0o600
    journal_path.chmod(0o644)
    with pytest.raises(DeviceEventJournalError, match="already disclosed"):
        journal.read_all()


@pytest.mark.skipif(os.name != "posix", reason="POSIX permission semantics")
def test_a_loose_directory_is_refused_on_read_and_tightened_on_append(tmp_path: Path):
    loose = tmp_path.resolve() / "loose"
    loose.mkdir()
    loose.chmod(0o755)
    journal = rotating(loose / "events.ndjson", max_events=8)
    with pytest.raises(DeviceEventJournalError, match="owner-only"):
        journal.read_all()
    # Append creates and owns the directory mode, so it repairs rather than
    # refuses; after that the read path is satisfied.
    journal.append(event(1))
    assert loose.stat().st_mode & 0o777 == 0o700
    assert sequences(journal.read_all()) == [1]


@pytest.mark.skipif(os.name != "posix", reason="symlinks")
def test_a_journal_path_crossing_a_symlink_is_refused_at_any_depth(tmp_path: Path):
    root = tmp_path.resolve()
    real = root / "real"
    (real / "deep").mkdir(parents=True)
    (root / "link").symlink_to(real, target_is_directory=True)

    with pytest.raises(DeviceEventJournalError, match="crosses a symlink"):
        rotating(root / "link" / "events.ndjson", max_events=8).append(event(1))
    # An ancestor several levels up is the interesting case: checking only the
    # immediate parent would let this through.
    with pytest.raises(DeviceEventJournalError, match="crosses a symlink"):
        rotating(root / "link" / "deep" / "events.ndjson", max_events=8).append(event(1))
    # The same directory reached without the symlink is fine.
    rotating(real / "deep" / "events.ndjson", max_events=8).append(event(1))


@pytest.mark.skipif(os.name != "posix", reason="POSIX directory identity")
def test_a_directory_displaced_while_it_is_held_is_refused_not_written_into(tmp_path: Path):
    root = tmp_path.resolve()
    live = root / "live"
    live.mkdir(mode=0o700)
    with _JournalDirectory(live, create=False) as directory:
        directory.reverify()  # unchanged: no complaint
        live.rename(root / "moved")
        (root / "live").mkdir(mode=0o700)
        # Same pathname, different inode. A writer that only locked the inode
        # would append into a directory nothing can find.
        with pytest.raises(DeviceEventJournalError, match="was replaced while it was in use"):
            directory.reverify()


@pytest.mark.skipif(os.name != "posix", reason="POSIX directory identity")
def test_a_directory_removed_while_it_is_held_is_refused(tmp_path: Path):
    root = tmp_path.resolve()
    live = root / "gone"
    live.mkdir(mode=0o700)
    with _JournalDirectory(live, create=False) as directory:
        live.rmdir()
        with pytest.raises(DeviceEventJournalError, match="no longer exists at its own path"):
            directory.reverify()


def test_reading_a_journal_whose_directory_does_not_exist_is_refused(tmp_path: Path):
    journal = rotating(tmp_path.resolve() / "absent" / "events.ndjson", max_events=8)
    with pytest.raises(DeviceEventJournalError, match="does not exist"):
        journal.read_all()


# -- short writes -------------------------------------------------------------


def test_write_all_keeps_going_until_every_byte_lands(tmp_path: Path, monkeypatch):
    """A short ``os.write`` is not a successful write.

    One truncated final line makes every later read and every later append a
    hard failure, so the loop is load-bearing rather than defensive.
    """
    target = tmp_path.resolve() / "chunked.bin"
    payload = b"".join(f'{{"line":{index}}}\n'.encode() for index in range(40))
    real_write = os.write
    chunks: list[int] = []

    def stingy(descriptor: int, data) -> int:
        written = real_write(descriptor, bytes(data)[:7])
        chunks.append(written)
        return written

    handle = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        monkeypatch.setattr(device_events.os, "write", stingy)
        _write_all(handle, payload)
    finally:
        monkeypatch.undo()
        os.close(handle)

    assert target.read_bytes() == payload
    assert len(chunks) > 1, "the fixture must actually have forced a short write"


def test_a_write_that_makes_no_progress_is_an_error_not_a_silent_truncation(
    tmp_path: Path, monkeypatch
):
    target = tmp_path.resolve() / "stalled.bin"
    handle = os.open(target, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        monkeypatch.setattr(device_events.os, "write", lambda _fd, _data: 0)
        with pytest.raises(DeviceEventJournalError, match="made no progress"):
            _write_all(handle, b"anything at all\n")
    finally:
        monkeypatch.undo()
        os.close(handle)


def test_a_rotation_survives_a_filesystem_that_only_takes_short_writes(
    journal_path: Path, monkeypatch
):
    journal = rotating(journal_path, max_events=2)
    real_write = os.write

    def stingy(descriptor: int, data) -> int:
        return real_write(descriptor, bytes(data)[:11])

    monkeypatch.setattr(device_events.os, "write", stingy)
    try:
        for index in (1, 2, 3, 4):
            journal.append(event(index))
    finally:
        monkeypatch.undo()

    assert sequences(rotating(journal_path, max_events=2).read_all()) == [3, 4]


# -- ROS-free -----------------------------------------------------------------


_ISOLATED_IMPORT = """
import importlib.util
import sys

BANNED = {
    "rclpy", "rosidl_runtime_py", "rosidl_parser", "ament_index_python",
    "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "std_srvs",
    "action_msgs", "builtin_interfaces", "tf2_ros", "launch", "launch_ros",
    "gz", "ignition", "flyto_robotics", "yaml", "numpy", "requests", "pytest",
}


class Blocker:
    def find_spec(self, name, path=None, target=None):
        if name.split(".")[0] in BANNED:
            raise AssertionError("device_events imported " + name)
        return None


sys.meta_path.insert(0, Blocker())
spec = importlib.util.spec_from_file_location("device_events_isolated", sys.argv[1])
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
assert module.DEVICE_EVENT_CONTRACT == "flyto.device-event.v1"
built = module.build_device_event(
    resource_id="dev-1",
    component="diagnostics",
    sequence=1,
    observed_at="2026-08-10T00:00:00.000000Z",
    severity="info",
    status="succeeded",
    reason_code="all_clear",
)
assert built["event_hash"]
print("ros-free")
"""


def test_the_module_loads_and_works_with_no_ros_and_no_third_party_package():
    """The contract has to be importable on a device with nothing installed.

    Loaded straight from its file with the package, ROS and every third-party
    dependency blocked at the import hook, so an accidental dependency added to
    this module fails here rather than on a bare device.
    """
    module_path = Path(device_events.__file__).resolve()
    completed = subprocess.run(
        [sys.executable, "-c", _ISOLATED_IMPORT, str(module_path)],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(module_path.parent.parent),
    )
    assert completed.returncode == 0, completed.stderr
    assert completed.stdout.strip() == "ros-free"


# -- schema and runtime parity -------------------------------------------------
#
# The schema is what a consumer in another language validates against; the module
# is what this device produces. If the two drift, every event this repository
# calls valid is rejected by the reader it was written for, and nothing in either
# file says so. These tests are the only thing holding them together.

PROJECT_ROOT = Path(device_events.__file__).resolve().parents[1]
SCHEMA_PATH = PROJECT_ROOT / "contracts/device-event-v1.schema.json"


def schema() -> dict:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def test_the_device_event_schema_is_registered_with_validate_assets():
    """An unregistered schema is a file nothing ever parses; ``make verify``
    would keep passing with a syntax error sitting in the contract."""
    assert "contracts/device-event-v1.schema.json" in validate_assets()


def test_schema_and_runtime_agree_on_exactly_which_fields_are_public():
    document = schema()
    produced = set(event(1))
    assert set(document["properties"]) == produced
    # Required, not merely present: an optional field in the schema that the
    # runtime always emits is a field a reader is entitled to find missing.
    assert set(document["required"]) == produced
    assert document["additionalProperties"] is False
    assert document["properties"]["contract"]["const"] == DEVICE_EVENT_CONTRACT


def test_schema_and_runtime_agree_on_the_closed_enumerations():
    document = schema()
    assert tuple(document["properties"]["severity"]["enum"]) == SEVERITIES
    assert tuple(document["properties"]["status"]["enum"]) == STATUSES


def test_schema_and_runtime_agree_on_every_bound():
    properties = schema()["properties"]
    assert properties["sequence"]["maximum"] == SEQUENCE_LIMIT
    assert properties["sequence"]["minimum"] == 0
    assert properties["message"]["maxLength"] == MESSAGE_LIMIT
    assert properties["action_codes"]["maxItems"] == ACTION_CODE_LIMIT
    assert properties["details"]["maxProperties"] == DETAILS_KEY_LIMIT


def test_schema_and_runtime_agree_on_the_redaction_block():
    redaction = schema()["properties"]["redaction"]
    produced = event(1)["redaction"]
    assert set(redaction["properties"]) == set(produced)
    assert set(redaction["required"]) == set(produced)
    assert redaction["properties"]["policy"]["const"] == REDACTION_POLICY
    for field in ("raw_logs_included", "credentials_included", "personal_data_included"):
        assert redaction["properties"][field]["const"] is False
        assert produced[field] is False
    # Zero in both places, because this contract refuses rather than scrubs.
    assert redaction["properties"]["redacted_key_count"]["const"] == 0
    assert produced["redacted_key_count"] == 0


def test_schema_and_runtime_share_the_same_identifier_and_code_patterns():
    definitions = schema()["$defs"]
    assert definitions["identifier"]["pattern"] == device_events._IDENTIFIER.pattern
    assert definitions["code"]["pattern"] == device_events._CODE.pattern
    # The optional form is the identifier plus empty, and nothing else.
    optional = re.compile(definitions["optionalIdentifier"]["pattern"])
    assert optional.fullmatch("")
    assert optional.fullmatch("flyto-rover-sim-001")
    assert not optional.fullmatch("has space")


def test_the_schema_timestamp_pattern_accepts_what_the_runtime_stamps():
    pattern = re.compile(schema()["properties"]["observed_at"]["pattern"])
    stamped = now_observed_at()
    assert pattern.fullmatch(stamped)
    assert len(stamped) <= schema()["properties"]["observed_at"]["maxLength"]
    assert not pattern.fullmatch("2026-08-10T00:00:00+00:00")


def test_every_exported_name_exists_and_the_shared_constants_are_public():
    for name in device_events.__all__:
        assert hasattr(device_events, name), name
    # The two the exporter depends on: if either stopped being public, the CLI
    # would grow its own copy and the writer and the reader would disagree.
    assert "DEVICE_EVENT_JOURNAL_ENV" in device_events.__all__
    assert "DEFAULT_JOURNAL_FILENAME" in device_events.__all__
    assert DEVICE_EVENT_JOURNAL_ENV == "FLYTO_DEVICE_EVENT_JOURNAL"
    assert DEFAULT_JOURNAL_FILENAME == "device-events.ndjson"


# -- export CLI ----------------------------------------------------------------


@pytest.fixture(autouse=True)
def _no_ambient_journal(monkeypatch):
    """No test may pass because the developer's shell happens to export one."""
    monkeypatch.delenv(DEVICE_EVENT_JOURNAL_ENV, raising=False)


def cli(capsys, *argv: str) -> tuple[int, str, str]:
    code = device_event_cli.main(list(argv))
    captured = capsys.readouterr()
    return code, captured.out, captured.err


def document_from(out: str) -> dict:
    """One canonical JSON line, and the encoding is asserted, not assumed."""
    assert out.endswith("\n")
    assert out.count("\n") == 1, "one document per invocation, on one line"
    decoded = json.loads(out)
    assert out.rstrip("\n") == canonical_json(decoded), "stdout must be canonical JSON"
    return decoded


def stocked(path: Path, count: int, *, max_events: int = 8) -> DeviceEventJournal:
    journal = rotating(path, max_events=max_events)
    for index in range(1, count + 1):
        journal.append(event(index))
    return journal


def test_export_writes_one_canonical_document_and_leaves_stderr_empty(
    capsys, journal_path: Path
):
    stocked(journal_path, 3)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert (code, err) == (device_event_cli.EXIT_OK, "")
    exported = document_from(out)
    assert exported["contract"] == DEVICE_EVENT_EXPORT_CONTRACT
    assert sequences(exported["records"]) == [1, 2, 3]
    assert exported["count"] == 3
    assert exported["complete"] is True
    assert exported["gap"] is False


def test_export_paginates_and_resumes_from_the_cursor_it_handed_out(
    capsys, journal_path: Path
):
    stocked(journal_path, 3)
    code, out, _err = cli(capsys, "export", "--journal", str(journal_path), "--limit", "2")
    first = document_from(out)
    assert (code, sequences(first["records"])) == (0, [1, 2])
    assert first["complete"] is False

    code, out, _err = cli(
        capsys, "export", "--journal", str(journal_path), "--cursor", first["next_cursor"]
    )
    second = document_from(out)
    assert (code, sequences(second["records"])) == (0, [3])
    assert second["complete"] is True
    assert second["gap"] is False

    # Resuming again returns nothing, and says so without claiming a gap.
    code, out, _err = cli(
        capsys, "export", "--journal", str(journal_path), "--cursor", second["next_cursor"]
    )
    third = document_from(out)
    assert (code, third["count"], third["gap"]) == (0, 0, False)


def test_export_reports_a_retention_gap_instead_of_looking_up_to_date(
    capsys, journal_path: Path
):
    """The distinction the whole cursor design exists for, seen through the CLI.

    A reader that is told ``count: 2, complete: true`` and nothing else would
    conclude it has the whole history. It does not: records 1 and 2 rolled out
    of retention while it was away, and only ``gap`` says so.
    """
    journal = stocked(journal_path, 2, max_events=2)
    code, out, _err = cli(capsys, "export", "--journal", str(journal_path), "--limit", "1")
    first = document_from(out)
    stale = first["next_cursor"]
    assert (code, sequences(first["records"])) == (0, [1])

    for index in (3, 4):
        journal.append(event(index))

    code, out, err = cli(capsys, "export", "--journal", str(journal_path), "--cursor", stale)
    resumed = document_from(out)
    assert (code, err) == (0, "")
    assert sequences(resumed["records"]) == [3, 4]
    assert resumed["gap"] is True
    assert resumed["gap_before_sequence"] == 3


def test_export_hands_out_records_and_never_the_file_behind_them(capsys, journal_path: Path):
    stocked(journal_path, 2)
    raw = journal_path.read_text(encoding="utf-8")
    header = raw.splitlines()[0]

    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert code == 0
    exported = document_from(out)
    # The header line, the file's own path, and the raw bytes are all absent:
    # a reader gets validated records, not a view of the device's filesystem.
    assert header not in out
    assert str(journal_path) not in out + err
    assert raw not in out
    assert set(exported) == {
        "contract",
        "journal_id",
        "records",
        "count",
        "next_cursor",
        "complete",
        "gap",
        "gap_before_sequence",
    }
    for record in exported["records"]:
        assert set(record) == {"journal_sequence", "event"}
        assert record["event"]["redaction"]["raw_logs_included"] is False


def test_export_takes_the_journal_from_the_environment_when_no_flag_is_given(
    capsys, journal_path: Path, monkeypatch
):
    stocked(journal_path, 1)
    monkeypatch.setenv(DEVICE_EVENT_JOURNAL_ENV, str(journal_path))
    code, out, err = cli(capsys, "export")
    assert (code, err) == (0, "")
    assert document_from(out)["count"] == 1


def test_a_directory_resolves_to_the_shared_default_journal_filename(
    capsys, tmp_path: Path
):
    inside = tmp_path.resolve() / "state" / DEFAULT_JOURNAL_FILENAME
    stocked(inside, 1)
    code, out, err = cli(capsys, "export", "--journal", str(inside.parent))
    assert (code, err) == (0, "")
    assert document_from(out)["count"] == 1


def test_with_no_journal_named_the_command_refuses_rather_than_guessing(capsys):
    code, out, err = cli(capsys, "export")
    assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")
    assert err.splitlines() == [f"flyto-device-events: {device_event_cli._NO_JOURNAL_NAMED}"]


def test_a_missing_journal_is_an_error_not_an_empty_export(capsys, tmp_path: Path):
    absent = tmp_path.resolve() / "state" / "events.ndjson"
    code, out, err = cli(capsys, "export", "--journal", str(absent))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")
    assert err.strip().endswith(device_event_cli._MISSING_JOURNAL)
    assert str(absent) not in err


@pytest.mark.parametrize(
    "cursor",
    ["not-a-cursor", "dej1:zzz:1:0000000000000000", "dej1:" + "a" * 32 + ":1:short"],
)
def test_a_malformed_cursor_is_refused_before_the_journal_is_opened(
    capsys, journal_path: Path, cursor: str
):
    stocked(journal_path, 1)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path), "--cursor", cursor)
    assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")
    assert err.strip().endswith(device_event_cli._BAD_CURSOR)


def test_a_cursor_from_another_journal_is_refused_as_a_bad_request(capsys, tmp_path: Path):
    mine = tmp_path.resolve() / "mine" / "events.ndjson"
    theirs = tmp_path.resolve() / "theirs" / "events.ndjson"
    stocked(mine, 2)
    other = stocked(theirs, 2)
    foreign = cursor_for(other, other.read_all()[0])

    code, out, err = cli(capsys, "export", "--journal", str(mine), "--cursor", foreign)
    assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")
    assert err.strip().endswith(device_event_cli._BAD_CURSOR)


@pytest.mark.parametrize(
    ("flag", "value"),
    [
        ("--limit", "0"),
        ("--limit", str(HARD_MAX_EXPORT_ITEMS + 1)),
        ("--max-bytes", "512"),
        ("--max-bytes", "1024"),
        ("--max-bytes", str(MIN_EXPORT_BYTES - 1)),
        ("--max-bytes", str(HARD_MAX_EXPORT_BYTES + 1)),
    ],
)
def test_bounds_outside_the_contract_are_refused_as_a_bad_request(
    capsys, journal_path: Path, flag: str, value: str
):
    stocked(journal_path, 1)
    code, out, _err = cli(capsys, "export", "--journal", str(journal_path), flag, value)
    assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")


# -- export CLI: the byte bound is the bound ------------------------------------
#
# The floor and the refusal live in device_events, above; these check that the
# command consumes that shared bound rather than keeping an opinion of its own,
# and that what reaches stdout obeys it.


def test_the_cli_consumes_the_shared_floor_instead_of_a_private_copy():
    assert device_event_cli.MIN_EXPORT_BYTES == MIN_EXPORT_BYTES
    # Imported, not re-owned: the CLI does not advertise the bound as its own.
    assert "MIN_EXPORT_BYTES" not in device_event_cli.__all__
    # And the message it prints quotes that same number, so an operator who
    # raises --max-bytes to what they were told lands on a value that works.
    assert str(MIN_EXPORT_BYTES) in device_event_cli._BAD_MAX_BYTES


def test_a_budget_too_small_for_one_maximal_record_is_refused_not_overshot(
    capsys, journal_path: Path
):
    """The regression. ``--max-bytes 1024`` used to return a 2664-byte record.

    A bound that is silently exceeded is worse than no bound: the caller sized a
    buffer, a pipe or an HTTP body from it.
    """
    stocked_bulky(journal_path, 2)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path), "--max-bytes", "1024")
    assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")
    assert err.strip().endswith(device_event_cli._BAD_MAX_BYTES)


def test_the_smallest_accepted_budget_still_fits_the_largest_possible_record(
    capsys, journal_path: Path
):
    floor = MIN_EXPORT_BYTES
    stocked_bulky(journal_path, 1)
    code, out, err = cli(
        capsys, "export", "--journal", str(journal_path), "--max-bytes", str(floor)
    )
    assert (code, err) == (0, "")
    exported = document_from(out)
    assert exported["count"] == 1, "the floor must always admit one record"
    assert device_event_cli.record_bytes(exported) <= floor
    # The command measures a page exactly as the journal budgets one; two
    # slightly different ideas of "size" would make the check meaningless.
    assert device_event_cli.record_bytes(exported) == page_bytes(exported)


def test_every_page_stays_inside_the_budget_and_the_cursor_always_advances(
    capsys, journal_path: Path
):
    """Walk a journal of maximal records at the minimum budget, to exhaustion.

    Three failure modes are covered at once: a page over the bound (the bug), a
    page that comes back empty while records remain (a livelock -- the reader
    would ask forever), and a record that never appears on any page (silent loss).
    """
    total = 6
    stocked_bulky(journal_path, total)
    budget = MIN_EXPORT_BYTES

    seen: list[int] = []
    cursor = ""
    for _page in range(total + 2):
        code, out, err = cli(
            capsys,
            "export",
            "--journal",
            str(journal_path),
            "--cursor",
            cursor,
            "--max-bytes",
            str(budget),
        )
        assert (code, err) == (0, "")
        page = document_from(out)
        assert device_event_cli.record_bytes(page) <= budget, "a page exceeded the bound asked for"
        assert page["gap"] is False
        if page["complete"] and not page["records"]:
            break
        assert page["records"], "an incomplete export returned nothing; the cursor cannot advance"
        seen.extend(sequences(page["records"]))
        assert page["next_cursor"] != cursor, "the cursor must move or the reader is stuck"
        cursor = page["next_cursor"]
    else:  # pragma: no cover - only reached if the walk never terminates
        pytest.fail("the export never reported itself complete")

    assert seen == list(range(1, total + 1)), "every record exactly once, in order"


# -- export CLI: the path is input, and input is never echoed -------------------


@pytest.mark.parametrize(
    ("label", "path"),
    [
        ("over-long", "/tmp/" + "z" * 5000),
        ("embedded-nul", "/tmp/state\x00/events.ndjson"),
        ("empty", ""),
        ("just-spaces", "   "),
    ],
)
def test_an_unusable_journal_path_is_refused_without_echoing_it(capsys, label: str, path: str):
    """The second regression. A 5000-character path raised ``OSError`` -- whose
    message is the whole path -- straight through argparse's caller.

    ``Path.is_dir`` swallows ENOENT and ENOTDIR but re-raises ENAMETOOLONG, so
    "the path is too long" was the one path failure that escaped as a traceback.
    """
    code, out, err = cli(capsys, "export", "--journal", path)
    assert code in (
        device_event_cli.EXIT_INVALID_REQUEST,
        device_event_cli.EXIT_UNUSABLE_JOURNAL,
    ), label
    assert out == ""
    # One line, from the closed set, carrying nothing the caller passed in.
    assert len(err.splitlines()) == 1
    assert err.startswith("flyto-device-events: ")
    assert err.strip().split(": ", 1)[1] in {
        device_event_cli._NO_JOURNAL_NAMED,
        device_event_cli._UNUSABLE_PATH,
        device_event_cli._MISSING_JOURNAL,
    }
    assert "Traceback" not in err
    assert "zzzz" not in err
    assert "\x00" not in err
    assert path.strip() not in err or not path.strip()


def test_a_path_the_filesystem_itself_rejects_is_a_fixed_refusal(capsys, monkeypatch):
    """Whatever the OS raises while probing the path, the output is the same.

    Parametrising real paths can only reach the errors this kernel happens to
    produce; this covers the class, including the ones another platform raises.
    """
    for failure in (
        OSError(36, "File name too long: '/very/long/secret-looking/path'"),
        ValueError("embedded null byte: '/state\\x00/events.ndjson'"),
        RuntimeError("Could not determine home directory for '~operator'"),
    ):

        def explode(_self, _exception=failure):
            raise _exception

        # Re-patched rather than undone each round: undo() would also roll back
        # the autouse fixture that clears the ambient journal variable, and a
        # test that passes only on a machine with that variable unset is not a
        # test. Teardown restores the real method.
        monkeypatch.setattr(Path, "is_dir", explode)
        code, out, err = cli(capsys, "export", "--journal", "/state/events.ndjson")

        assert (code, out) == (device_event_cli.EXIT_INVALID_REQUEST, "")
        assert err.strip() == f"flyto-device-events: {device_event_cli._UNUSABLE_PATH}"
        assert "secret-looking" not in err
        assert "operator" not in err


def test_an_unexpected_failure_still_refuses_instead_of_printing_a_traceback(
    capsys, journal_path: Path, monkeypatch
):
    """The last guard, which exists so that a bug is not also a disclosure."""
    stocked(journal_path, 1)

    def explode(*_args, **_kwargs):
        raise MemoryError("while reading /home/operator/state/events.ndjson")

    monkeypatch.setattr(device_event_cli.DeviceEventJournal, "export", explode)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")
    assert err.strip() == f"flyto-device-events: {device_event_cli._UNEXPECTED}"
    assert "operator" not in err


def test_a_corrupt_journal_is_refused_without_quoting_what_corrupted_it(
    capsys, journal_path: Path
):
    """The refusal must not become the leak the export was designed to prevent.

    Whatever ended up in the file — a half-written record, someone's shell
    history, a token pasted by mistake — the journal error that rejects it can
    quote it. That text never reaches stderr.
    """
    stocked(journal_path, 1)
    with journal_path.open("ab") as handle:
        handle.write(b'{"journal_sequence": 9, "event": {"note": "password: hunter2-XYZ"}}\n')

    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")
    assert err.strip().endswith(device_event_cli._UNUSABLE_JOURNAL)
    assert "hunter2" not in err
    assert "password" not in err


def test_a_journal_that_ends_mid_record_is_refused_by_the_cli(capsys, journal_path: Path):
    stocked(journal_path, 2)
    truncated = journal_path.read_bytes()[:-40]
    journal_path.write_bytes(truncated)
    os.chmod(journal_path, 0o600)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")
    assert err.strip().endswith(device_event_cli._UNUSABLE_JOURNAL)


@pytest.mark.skipif(os.name != "posix", reason="file modes are POSIX-only here")
def test_a_world_readable_journal_is_refused_by_the_cli(capsys, journal_path: Path):
    stocked(journal_path, 1)
    os.chmod(journal_path, 0o644)
    code, out, err = cli(capsys, "export", "--journal", str(journal_path))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")
    assert err.strip().endswith(device_event_cli._UNUSABLE_JOURNAL)


def test_a_journal_that_is_a_symlink_is_refused_rather_than_followed(
    capsys, tmp_path: Path
):
    real = tmp_path.resolve() / "state" / "events.ndjson"
    stocked(real, 1)
    link = tmp_path.resolve() / "state" / "link.ndjson"
    link.symlink_to(real)
    code, out, _err = cli(capsys, "export", "--journal", str(link))
    assert (code, out) == (device_event_cli.EXIT_UNUSABLE_JOURNAL, "")


def test_the_command_line_itself_being_wrong_is_a_usage_error(capsys):
    assert device_event_cli.main([]) == device_event_cli.EXIT_USAGE
    capsys.readouterr()
    assert device_event_cli.main(["publish"]) == device_event_cli.EXIT_USAGE
    capsys.readouterr()
    assert device_event_cli.main(["export", "--journal", "x", "--limit", "many"]) == (
        device_event_cli.EXIT_USAGE
    )
    capsys.readouterr()
    assert device_event_cli.main(["--help"]) == device_event_cli.EXIT_OK
    assert "export" in capsys.readouterr().out


_ISOLATED_CLI = """
import sys
import types

BANNED = {
    "rclpy", "rosidl_runtime_py", "rosidl_parser", "ament_index_python",
    "geometry_msgs", "nav_msgs", "sensor_msgs", "std_msgs", "std_srvs",
    "action_msgs", "builtin_interfaces", "tf2_ros", "launch", "launch_ros",
    "gz", "ignition", "yaml", "numpy", "requests", "pytest",
    "socket", "http", "ssl", "ftplib", "smtplib", "telnetlib", "asyncio",
    "urllib.request", "xmlrpc",
}


class Blocker:
    def find_spec(self, name, path=None, target=None):
        if name in BANNED or name.split(".")[0] in BANNED:
            raise AssertionError("device_event_cli imported " + name)
        return None


sys.meta_path.insert(0, Blocker())

# A stub package, so the rest of flyto_robotics is never executed: this asserts
# what the export path itself needs, not what the package around it happens to
# pull in today.
package = types.ModuleType("flyto_robotics")
package.__path__ = [sys.argv[1]]
sys.modules["flyto_robotics"] = package

from flyto_robotics import device_event_cli

raise SystemExit(device_event_cli.main(["export", "--journal", sys.argv[2]]))
"""


def test_the_export_cli_runs_with_no_ros_no_network_and_no_third_party_package(
    journal_path: Path,
):
    """The device that needs exporting is the device with nothing installed.

    ROS, every third-party package, and every networking module in the standard
    library are blocked at the import hook: an export path that reached for a
    socket would fail here rather than on a hospital network.
    """
    stocked(journal_path, 2)
    package_directory = Path(device_events.__file__).resolve().parent
    environment = {
        key: item for key, item in os.environ.items() if key != DEVICE_EVENT_JOURNAL_ENV
    }
    completed = subprocess.run(
        [sys.executable, "-c", _ISOLATED_CLI, str(package_directory), str(journal_path)],
        capture_output=True,
        text=True,
        check=False,
        cwd=str(PROJECT_ROOT),
        env=environment,
    )
    assert completed.returncode == 0, completed.stderr
    exported = json.loads(completed.stdout)
    assert exported["contract"] == DEVICE_EVENT_EXPORT_CONTRACT
    assert exported["count"] == 2
