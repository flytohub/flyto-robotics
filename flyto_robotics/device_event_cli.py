"""``flyto-device-events``: hand out device events without handing over the file.

An upstream reader — a fleet console, a support engineer, a sync agent — needs
to know what a device has been reporting.  The obvious way to give it that is to
let it read the journal file, and that is the wrong way twice over: it exposes
whatever else ended up in the file (partial lines, a record being rewritten, the
file's own path and permissions), and it makes the device's retention policy
somebody else's problem.  This command is the narrow alternative.  It reads the
journal through the same validating code the device writes it with, and prints
one bounded, canonical JSON export document to stdout.

Four properties are deliberate:

* **Standard library only.** No ROS, no simulator, no third-party package, and
  no network of any kind.  This has to run on a device where the only thing
  installed is Python, which is exactly the device you need events from.
* **Explicit about which journal.** The path comes from ``--journal`` or from
  :data:`~flyto_robotics.device_events.DEVICE_EVENT_JOURNAL_ENV`, and from
  nowhere else.  There is no built-in search path, because a command that
  guesses where the journal lives will one day export the wrong device's.
* **Bounded, and the bound is the bound.** ``--limit`` and ``--max-bytes`` are
  held between the contract's floor and ceiling, so a caller cannot ask for the
  whole journal in one response no matter what it passes — and cannot ask for a
  response so small that the journal would have to break a promise to answer.
  Those limits belong to
  :mod:`~flyto_robotics.device_events`, not to this command: the floor is
  :data:`~flyto_robotics.device_events.MIN_EXPORT_BYTES`, the journal refuses
  anything under it for every caller, and this command imports it rather than
  keeping an opinion of its own.  What is added here is a nicer refusal in
  place of a traceback, and a re-measurement of each page against the bound
  before it is printed.
* **Fixed, content-free failures.** Every refusal prints one short line from a
  closed set and exits non-zero.  Exception text is deliberately not forwarded:
  the messages a corrupt journal produces can quote a key or a fragment of the
  file that caused them, and a diagnostic that leaks the contents of the file it
  refused to export defeats the point of not exporting the file.

Exit codes are part of the interface, because the caller is usually a script:

===== ==========================================================================
``0`` an export document was written to stdout
``2`` the command line itself was wrong (argparse usage)
``3`` the request was invalid: no journal named, unusable path, bad cursor,
      bounds out of range
``4`` the journal is missing, unreadable, unsafe, or corrupt
===== ==========================================================================
"""

from __future__ import annotations

import argparse
import os
import sys
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from flyto_robotics.device_events import (
    DEFAULT_EXPORT_BYTES,
    DEFAULT_EXPORT_ITEMS,
    DEFAULT_JOURNAL_FILENAME,
    DEVICE_EVENT_JOURNAL_ENV,
    HARD_MAX_EXPORT_BYTES,
    HARD_MAX_EXPORT_ITEMS,
    HARD_MAX_JOURNAL_BYTES,
    HARD_MAX_JOURNAL_EVENTS,
    MIN_EXPORT_BYTES,
    DeviceEventBoundError,
    DeviceEventCursorError,
    DeviceEventError,
    DeviceEventJournal,
    canonical_json,
    decode_cursor,
    export_page_bytes,
)

# MIN_EXPORT_BYTES is imported, never redefined. The floor below which an export
# byte bound cannot be honoured is a property of the contract, not of this
# command: the journal enforces it for every caller, and a private copy here
# would be a second policy that disagrees the day the event size limit moves.

__all__ = [
    "EXIT_INVALID_REQUEST",
    "EXIT_OK",
    "EXIT_UNUSABLE_JOURNAL",
    "EXIT_USAGE",
    "MAX_JOURNAL_PATH_CHARS",
    "PROGRAM",
    "main",
    "record_bytes",
    "resolve_journal_path",
]

PROGRAM = "flyto-device-events"

EXIT_OK = 0
EXIT_USAGE = 2
EXIT_INVALID_REQUEST = 3
EXIT_UNUSABLE_JOURNAL = 4

#: A path longer than this is refused before it is touched. Not a security
#: boundary — the point is that the OS error for an over-long path quotes the
#: whole path back, and this command does not put its input on stderr.
MAX_JOURNAL_PATH_CHARS = 4096

#: The closed set of things this command will say when it refuses. Fixed
#: strings, no path, no exception text, no fragment of the journal.
_NO_JOURNAL_NAMED = (
    f"no journal path; pass --journal or set {DEVICE_EVENT_JOURNAL_ENV}"
)
_UNUSABLE_PATH = "journal path is not usable"
_BAD_CURSOR = "cursor is malformed or was not issued by this journal"
_BAD_LIMIT = f"--limit must be between 1 and {HARD_MAX_EXPORT_ITEMS}"
_BAD_MAX_BYTES = f"--max-bytes must be between {MIN_EXPORT_BYTES} and {HARD_MAX_EXPORT_BYTES}"
_MISSING_JOURNAL = "journal was not found at the path given"
_UNUSABLE_JOURNAL = "journal is unreadable, unsafe, or corrupt"
_OVERSIZED_PAGE = "journal holds a record larger than the byte bound allows"
_UNEXPECTED = "the export could not be completed"


class _Refusal(Exception):
    """One fixed message and one exit code. Never carries a cause's text."""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def resolve_journal_path(
    argument: str | None = None, environ: Mapping[str, str] | None = None
) -> Path:
    """The journal this invocation is about, from the argument or the variable.

    A directory is resolved to :data:`DEFAULT_JOURNAL_FILENAME` inside it, which
    is the one convenience here — an operator who types the state directory gets
    the journal, not a refusal about naming a file.  Nothing else is inferred:
    with neither the argument nor the variable set, this refuses instead of
    reaching for a default location, because exporting *some* journal when the
    caller meant a particular one is worse than exporting none.

    Every step that can touch the filesystem is guarded, and the guard is the
    point.  ``expanduser`` raises on an unresolvable ``~user``; a path with a NUL
    in it raises ``ValueError``; an over-long path raises ``OSError`` whose
    message is *the path itself* — and ``Path.is_dir`` does not swallow that one,
    it re-raises it.  Left uncaught, any of those turns a refusal into a
    traceback that prints the caller's input, which is exactly what a command
    that promises fixed, content-free failures must never do.
    """
    source = os.environ if environ is None else environ
    raw = (argument if argument is not None else source.get(DEVICE_EVENT_JOURNAL_ENV, "")).strip()
    if not raw:
        raise _Refusal(EXIT_INVALID_REQUEST, _NO_JOURNAL_NAMED)
    if len(raw) > MAX_JOURNAL_PATH_CHARS or "\x00" in raw:
        raise _Refusal(EXIT_INVALID_REQUEST, _UNUSABLE_PATH)
    try:
        path = Path(raw).expanduser()
        is_directory = path.is_dir()
    except (OSError, ValueError, RuntimeError) as exc:
        raise _Refusal(EXIT_INVALID_REQUEST, _UNUSABLE_PATH) from exc
    if is_directory:
        path = path / DEFAULT_JOURNAL_FILENAME
    try:
        present = path.exists()
    except (OSError, ValueError) as exc:
        raise _Refusal(EXIT_INVALID_REQUEST, _UNUSABLE_PATH) from exc
    if not present:
        # Checked here so "there is no journal" and "the journal is empty" are
        # different answers. An absent file is a deployment mistake; an empty
        # export is a device that has had nothing to report.
        raise _Refusal(EXIT_UNUSABLE_JOURNAL, _MISSING_JOURNAL)
    return path


def record_bytes(document: Mapping[str, Any]) -> int:
    """What the records in an export document cost, counted as the journal does.

    The same arithmetic ``DeviceEventJournal._select`` budgets with — canonical
    encoding plus the newline each record occupies in the file — so a page can be
    checked against the bound it was selected under rather than against a second,
    slightly different idea of size.

    The sum itself belongs to the contract, not to this command: it delegates to
    :func:`~flyto_robotics.device_events.export_page_bytes` so the journal's
    selection, the public re-measurement and this command can never drift into
    three subtly different ideas of what a page costs.
    """
    return export_page_bytes(document)


def _bounded(value: int, *, minimum: int, maximum: int, message: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):  # pragma: no cover - argparse types
        raise _Refusal(EXIT_INVALID_REQUEST, message)
    if not minimum <= value <= maximum:
        raise _Refusal(EXIT_INVALID_REQUEST, message)
    return value


def _checked_cursor(cursor: str) -> str:
    """Refuse a malformed cursor before the filesystem is touched at all."""
    try:
        decode_cursor(cursor)
    except DeviceEventError as exc:
        raise _Refusal(EXIT_INVALID_REQUEST, _BAD_CURSOR) from exc
    return cursor


def _export(args: argparse.Namespace, stdout, stderr) -> int:
    try:
        path = resolve_journal_path(args.journal)
        cursor = _checked_cursor(args.cursor)
        limit = _bounded(args.limit, minimum=1, maximum=HARD_MAX_EXPORT_ITEMS, message=_BAD_LIMIT)
        max_bytes = _bounded(
            args.max_bytes,
            minimum=MIN_EXPORT_BYTES,
            maximum=HARD_MAX_EXPORT_BYTES,
            message=_BAD_MAX_BYTES,
        )
    except _Refusal as refusal:
        return _refuse(refusal, stderr)

    # Read at the contract's hard ceilings rather than the journal defaults: a
    # reader must not decide it cannot read a journal that the writer was
    # allowed to write. The export bounds above are what actually limit the
    # response.
    try:
        journal = DeviceEventJournal(
            path, max_events=HARD_MAX_JOURNAL_EVENTS, max_bytes=HARD_MAX_JOURNAL_BYTES
        )
        document = journal.export(cursor=cursor, limit=limit, max_bytes=max_bytes)
    except DeviceEventCursorError as exc:
        return _refuse(_Refusal(EXIT_INVALID_REQUEST, _BAD_CURSOR), stderr, cause=exc)
    except DeviceEventBoundError as exc:
        # Unreachable while the floor holds, because --max-bytes was already
        # checked against the same shared constant the journal enforces. Mapped
        # anyway, and to its own message: if the two ever drift, the caller is
        # told which of the two it is rather than being handed "corrupt".
        return _refuse(_Refusal(EXIT_UNUSABLE_JOURNAL, _OVERSIZED_PAGE), stderr, cause=exc)
    except (DeviceEventError, OSError, ValueError) as exc:
        return _refuse(_Refusal(EXIT_UNUSABLE_JOURNAL, _UNUSABLE_JOURNAL), stderr, cause=exc)

    # The bound is checked against what was actually produced, not assumed from
    # what was asked for. MIN_EXPORT_BYTES makes an over-budget page impossible;
    # this is what turns "impossible" into "refused rather than printed" if the
    # event size limit and this floor ever drift apart.
    if record_bytes(document) > max_bytes:
        return _refuse(_Refusal(EXIT_UNUSABLE_JOURNAL, _OVERSIZED_PAGE), stderr)

    stdout.write(canonical_json(document) + "\n")
    return EXIT_OK


def _refuse(refusal: _Refusal, stderr, cause: BaseException | None = None) -> int:
    """One line, from the closed set above. ``cause`` is accepted and dropped.

    It is taken as an argument so the call sites read as though the cause
    matters, and discarded because it does not belong on this stream: the text
    of a journal error can quote the record that produced it.
    """
    del cause
    stderr.write(f"{PROGRAM}: {refusal.message}\n")
    return refusal.code


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=PROGRAM,
        description=(
            "Export bounded, public device events from a local journal. "
            "The journal file itself is never written to stdout."
        ),
    )
    subcommands = parser.add_subparsers(dest="command", required=True)
    export = subcommands.add_parser(
        "export",
        help="print one bounded export document as canonical JSON",
        description=(
            "Print the records after --cursor as one canonical JSON document. "
            "Resume by passing the document's next_cursor back in; a non-zero "
            "gap_before_sequence means retention dropped records you never saw."
        ),
    )
    export.add_argument(
        "--journal",
        default=None,
        metavar="PATH",
        help=(
            "path to the journal file, or to a directory holding "
            f"{DEFAULT_JOURNAL_FILENAME}; defaults to ${DEVICE_EVENT_JOURNAL_ENV}"
        ),
    )
    export.add_argument(
        "--cursor",
        default="",
        metavar="CURSOR",
        help="opaque cursor from a previous export's next_cursor; omit to start at the beginning",
    )
    export.add_argument(
        "--limit",
        type=int,
        default=DEFAULT_EXPORT_ITEMS,
        metavar="N",
        help=(
            f"maximum records to return "
            f"(1-{HARD_MAX_EXPORT_ITEMS}, default {DEFAULT_EXPORT_ITEMS})"
        ),
    )
    export.add_argument(
        "--max-bytes",
        type=int,
        default=DEFAULT_EXPORT_BYTES,
        metavar="N",
        help=(
            f"maximum encoded record bytes to return, excluding the small fixed "
            f"envelope ({MIN_EXPORT_BYTES}-{HARD_MAX_EXPORT_BYTES}, default "
            f"{DEFAULT_EXPORT_BYTES}); the floor is one maximal record, below "
            f"which the bound could not be honoured"
        ),
    )
    export.set_defaults(handler=_export)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """Entry point. Returns the exit code rather than raising ``SystemExit``."""
    parser = _build_parser()
    try:
        args = parser.parse_args(list(argv) if argv is not None else None)
    except SystemExit as exc:
        # argparse has already written its own usage text. Its code is 0 for
        # --help and 2 for a usage error; both are returned rather than raised
        # so an in-process caller sees the same result as the shell does.
        return int(exc.code) if isinstance(exc.code, int) else EXIT_USAGE
    try:
        return int(args.handler(args, sys.stdout, sys.stderr))
    except _Refusal as refusal:  # pragma: no cover - handlers refuse in place
        return _refuse(refusal, sys.stderr)
    except Exception as exc:  # deliberately broad; see below
        # The last guard. Anything that reaches here is a bug, and a bug in this
        # command would otherwise print a traceback: the caller's path, the
        # journal's location, and often a fragment of the file that triggered
        # it. A fixed line and a non-zero exit is a worse debugging experience
        # and a much better privacy one, and this command was written for the
        # second of those.
        return _refuse(_Refusal(EXIT_UNUSABLE_JOURNAL, _UNEXPECTED), sys.stderr, cause=exc)


if __name__ == "__main__":  # pragma: no cover - exercised as a subprocess
    raise SystemExit(main())
