"""``flyto-robot`` -- the command a customer actually runs.

Five verbs, one JSON object each, an exit status that means something:

    flyto-robot status
    flyto-robot install --from-package --version 1.4.0
    flyto-robot update  --from-package --version 1.5.0
    flyto-robot rollback
    flyto-robot support-bundle --output /tmp/bundle.json

Nothing here needs a source checkout, a login user, or a working directory. The
release payload defaults to the installed package itself (``--from-package``),
so the artifact that ships is the artifact that performs the lifecycle.

Exit status is part of the contract, because the thing driving these commands is
usually a script:

    0  the operation succeeded
    1  the operation failed; read `reason` and `action_code`
    2  the command line was wrong (argparse)

The systemd boundary is chosen here, not in the library. Operating on the real
root gets a real ``systemctl``; a rehearsal against ``--root /tmp/...`` gets a
recording controller that mutates no host. That rule is why the whole test suite
can drive install, update, rollback, and failure recovery on a laptop.
"""

from __future__ import annotations

import argparse
import base64
import csv
import hashlib
import importlib.metadata
import io
import json
import os
import stat
import sys
import tempfile
from collections.abc import Sequence
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path

from . import lifecycle
from .legacy_takeover import REVALIDATION_SCHEMA as LEGACY_REVALIDATION_SCHEMA
from .legacy_takeover import SCHEMA as LEGACY_TAKEOVER_SCHEMA
from .legacy_takeover import (
    TakeoverError,
    plan_takeover,
    read_takeover_receipt,
    revalidate_takeover_receipt,
)
from .lifecycle import LIFECYCLE_PROFILES_DEFAULT, Layout, LifecycleError
from .lifecycle_profiles import ProfileError
from .support_bundle import NoteRejected
from .systemd_control import RecordingRunner, SystemdController, SystemdError, subprocess_runner

__all__ = ["PACKAGE_PAYLOAD_EXCLUDE", "build_package_payload", "main"]

#: Never shipped into a release. Byte-code is interpreter-specific, so copying it
#: would make the same source produce different digests on different devices.
PACKAGE_PAYLOAD_EXCLUDE = ("__pycache__", "*.pyc", "*.pyo", "*.egg-info")

_DISTRIBUTION_NAME = "flyto_robotics"
_CANONICAL_DEPLOY_FILES = (
    "deploy/__init__.py",
    "deploy/flyto_job_runner.py",
    "deploy/device_executor_contract.py",
    "deploy/device_executor_registry.py",
)
_PACKAGE_PAYLOAD_MAX_FILES = 4096
_PACKAGE_PAYLOAD_MAX_FILE_BYTES = 4 * 1024 * 1024
_PACKAGE_PAYLOAD_MAX_BYTES = 32 * 1024 * 1024


def _payload_error(detail: str) -> LifecycleError:
    return LifecycleError("release_payload_invalid", detail)


def _read_once(source: Path, relative: str, *, limit: int) -> tuple[bytes, int]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(source, flags)
    except OSError as error:
        raise _payload_error(f"installed payload source cannot be opened: {relative}") from error
    try:
        before = os.fstat(descriptor)
        if not stat.S_ISREG(before.st_mode) or before.st_size > limit:
            raise _payload_error(f"installed payload source is invalid or oversized: {relative}")
        chunks: list[bytes] = []
        remaining = limit + 1
        while remaining:
            chunk = os.read(descriptor, min(64 * 1024, remaining))
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
        after = os.fstat(descriptor)
        if len(data) > limit or len(data) != before.st_size:
            raise _payload_error(f"installed payload source changed while reading: {relative}")
        if (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) != (
            after.st_dev,
            after.st_ino,
            after.st_size,
            after.st_mtime_ns,
        ):
            raise _payload_error(f"installed payload source changed while reading: {relative}")
        return data, before.st_mode
    finally:
        os.close(descriptor)


def _confined_source(source: Path, root: Path, label: str) -> None:
    try:
        source.relative_to(root)
    except ValueError as error:
        raise _payload_error(f"installed {label} is outside the distribution") from error
    for parent in source.parents:
        if parent == root:
            return
        if parent.is_symlink():
            raise _payload_error(f"installed {label} has a symlink parent")
    raise _payload_error(f"installed {label} is outside the distribution")


def _record_entries(
    distribution, record_name: str, distribution_root: Path
) -> dict[str, tuple[str, int] | None]:
    record_source = Path(distribution.locate_file(record_name))
    _confined_source(record_source, distribution_root, "distribution RECORD")
    raw, _ = _read_once(record_source, record_name, limit=4 * 1024 * 1024)
    try:
        rows = csv.reader(io.StringIO(raw.decode("utf-8", errors="strict"), newline=""))
        entries: dict[str, tuple[str, int] | None] = {}
        for row in rows:
            if len(row) != 3:
                raise ValueError("malformed RECORD row")
            name, digest, size_text = row
            if name in entries:
                raise ValueError("duplicate RECORD row")
            if name == record_name:
                continue
            algorithm, separator, encoded = digest.partition("=")
            if not digest and not size_text:
                entries[name] = None
                continue
            if algorithm != "sha256" or not separator or not encoded:
                entries[name] = None
                continue
            if not size_text.isascii() or not size_text.isdecimal():
                raise ValueError("invalid RECORD size")
            size = int(size_text)
            if size < 0:
                raise ValueError("negative RECORD size")
            entries[name] = (encoded, size)
        return entries
    except (UnicodeError, csv.Error, ValueError) as error:
        raise _payload_error("installed distribution RECORD is invalid") from error


def _installed_payload_sources() -> list[tuple[bytes, Path, int]]:
    """Resolve a bounded payload exclusively from one installed distribution."""

    try:
        distribution = importlib.metadata.distribution(_DISTRIBUTION_NAME)
    except importlib.metadata.PackageNotFoundError as error:
        raise _payload_error("installed flyto_robotics distribution is unavailable") from error

    recorded = distribution.files
    if recorded is None:
        raise _payload_error("installed distribution has no file inventory")
    recorded_names = [Path(str(item)).as_posix() for item in recorded]
    distribution_root = Path(distribution.locate_file("")).resolve()
    record_names = [name for name in recorded_names if name.endswith(".dist-info/RECORD")]
    if len(record_names) != 1:
        raise _payload_error("installed distribution is not a wheel installation")
    records = _record_entries(distribution, record_names[0], distribution_root)

    package_init = Path(distribution.locate_file("flyto_robotics/__init__.py"))
    running_init = Path(__file__).parent / "__init__.py"
    try:
        if package_init.resolve(strict=True) != running_init.resolve(strict=True):
            raise _payload_error("running package is outside the installed distribution")
    except OSError as error:
        raise _payload_error("installed package source is unavailable") from error

    if len(recorded_names) != len(set(recorded_names)):
        raise _payload_error("installed distribution has duplicate file inventory entries")
    inventory = {Path(str(item)).as_posix(): item for item in recorded}
    selected = sorted(
        path
        for path in inventory
        if path.startswith("flyto_robotics/")
        and not any(
            part == "__pycache__" or part.endswith((".pyc", ".pyo", ".egg-info"))
            for part in Path(path).parts
        )
    )
    for required in _CANONICAL_DEPLOY_FILES:
        if required not in inventory:
            raise _payload_error(f"installed distribution is missing {required}")
        selected.append(required)
    selected = sorted(selected)
    if not selected or len(selected) > _PACKAGE_PAYLOAD_MAX_FILES:
        raise _payload_error("installed distribution exceeds the payload file bound")

    sources: list[tuple[bytes, Path, int]] = []
    total = 0
    for relative_text in selected:
        relative = Path(relative_text)
        if (
            relative.is_absolute()
            or ".." in relative.parts
            or relative.parts[0] not in {"flyto_robotics", "deploy"}
            or (relative.parts[0] == "deploy" and relative_text not in _CANONICAL_DEPLOY_FILES)
        ):
            raise _payload_error(f"unexpected installed payload name: {relative_text}")
        source = Path(distribution.locate_file(inventory[relative_text]))
        _confined_source(source, distribution_root, f"payload source {relative_text}")
        record = records.get(relative_text)
        if record is None:
            raise _payload_error(
                f"installed distribution RECORD is missing or incomplete: {relative_text}"
            )
        encoded_digest, recorded_size = record
        data, source_mode = _read_once(source, relative_text, limit=_PACKAGE_PAYLOAD_MAX_FILE_BYTES)
        try:
            if len(encoded_digest) != 43 or "=" in encoded_digest:
                raise ValueError("noncanonical unpadded sha256")
            padded = encoded_digest + "="
            expected = base64.b64decode(padded, altchars=b"-_", validate=True)
            canonical = base64.urlsafe_b64encode(expected).rstrip(b"=").decode("ascii")
            if canonical != encoded_digest:
                raise ValueError("noncanonical url-safe base64")
        except (ValueError, base64.binascii.Error) as error:
            raise _payload_error(
                f"installed distribution RECORD hash is invalid: {relative_text}"
            ) from error
        if len(expected) != hashlib.sha256().digest_size:
            raise _payload_error(f"installed distribution RECORD hash is invalid: {relative_text}")
        if recorded_size != len(data) or not hashlib.sha256(data).digest() == expected:
            raise _payload_error(f"installed payload source does not match RECORD: {relative_text}")
        total += len(data)
        if total > _PACKAGE_PAYLOAD_MAX_BYTES:
            raise _payload_error("installed distribution exceeds the payload byte bound")
        safe_mode = 0o755 if source_mode & 0o111 else 0o644
        sources.append((data, relative, safe_mode))
    return sources


def build_package_payload(destination: Path) -> Path:
    """Copy the *installed* package into ``destination`` as a release payload.

    This is what makes ``--from-package`` honest: the bytes that get staged are
    the bytes that are running, taken from wherever pip or the distro package
    put them. A device with no git, no network, and no checkout can still
    reinstall or pin a version of itself.
    """

    destination = Path(destination)
    sources = _installed_payload_sources()
    if destination.exists():
        if destination.is_symlink() or not destination.is_dir() or any(destination.iterdir()):
            raise _payload_error("payload destination must be a new or empty directory")
    else:
        destination.mkdir(parents=True, mode=0o755)
    for data, relative, mode in sources:
        target = destination / relative
        target.parent.mkdir(parents=True, exist_ok=True, mode=0o755)
        if target.parent.is_symlink() or not target.parent.resolve().is_relative_to(
            destination.resolve()
        ):
            raise _payload_error(f"unsafe payload target: {relative.as_posix()}")
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            descriptor = os.open(target, flags, mode)
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(data)
            os.chmod(target, mode, follow_symlinks=False)
        except OSError as error:
            raise _payload_error(f"cannot create payload target: {relative.as_posix()}") from error
    return destination


@contextmanager
def _payload_from(args: argparse.Namespace):
    if args.from_package:
        with tempfile.TemporaryDirectory(prefix="flyto-payload-") as scratch:
            yield build_package_payload(Path(scratch))
        return
    yield Path(args.payload)


def _systemd_for(root: Path, *, no_systemd: bool, dry_run: bool, command: str) -> SystemdController:
    """Real systemctl only on the real root; a recorder everywhere else.

    A rehearsal that reloaded the developer's own systemd would be a footgun
    shipped as a feature, and a test suite that could do it would be worse.

    ``--no-systemd`` is a rehearsal switch, so it is refused for a *landing*
    operation on the real root. Allowing it there would let ``install`` print
    ``"ok": true`` on a customer's machine having never reloaded, enabled,
    started, or verified a single service -- a success report for a device that
    will not come back after a reboot. Rehearsal is fine; rehearsal that calls
    itself an install is not.
    """

    lands = command in {"install", "update", "rollback"} and not dry_run
    if root == Path("/") and no_systemd and lands:
        raise LifecycleError(
            "systemd_required",
            "--no-systemd cannot be combined with a real install on / ; "
            "add --dry-run to rehearse, or use --root to rehearse elsewhere",
        )
    if no_systemd or root != Path("/"):
        return SystemdController(runner=RecordingRunner(), dry_run=dry_run, mode="recording")
    return SystemdController(runner=subprocess_runner, dry_run=dry_run, mode="systemctl")


def _emit(report: dict, stream=None) -> int:
    handle = stream or sys.stdout
    json.dump(report, handle, indent=2, sort_keys=True)
    handle.write("\n")
    handle.flush()
    return 0 if report.get("ok") else 1


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="flyto-robot",
        description="Install, update, roll back, inspect, and support a Flyto machine.",
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=Path("/"),
        help="filesystem root; point at a temporary directory to rehearse without privileges",
    )
    parser.add_argument(
        "--profiles",
        type=Path,
        default=None,
        help="unit profile registry (defaults to the one shipped in the package)",
    )
    parser.add_argument(
        "--no-systemd",
        action="store_true",
        help="record the systemctl commands instead of running them",
    )
    parser.add_argument("--json", action="store_true", help="accepted for symmetry; always JSON")
    sub = parser.add_subparsers(dest="command", required=True)

    for name in ("install", "update"):
        action = sub.add_parser(name, help=f"{name} a release")
        source = action.add_mutually_exclusive_group(required=True)
        source.add_argument("--payload", type=Path, help="directory holding the release tree")
        source.add_argument(
            "--from-package",
            action="store_true",
            help="use the installed flyto_robotics package as the payload",
        )
        source.add_argument(
            "--manifest",
            type=Path,
            help="closed release manifest; requires --wheel-dir and publishes by inventory SHA",
        )
        action.add_argument("--wheel-dir", type=Path)
        action.add_argument("--version")
        # Deliberately not `choices=`. argparse would freeze the answer to the
        # registry that happened to be importable at parse time, which makes
        # `--profiles my-site.json --profile my-transport` impossible: the whole
        # point of a site registry is to name a profile this build never heard
        # of. The name is validated against the *selected* registry instead, and
        # an unknown one comes back as a reason code like every other refusal.
        action.add_argument(
            "--profile",
            default=LIFECYCLE_PROFILES_DEFAULT,
            help=f"unit profile; bundled: {', '.join(lifecycle.PROFILES)}",
        )
        action.add_argument("--python", default="/usr/bin/python3")
        action.add_argument("--dry-run", action="store_true")

    back = sub.add_parser("rollback", help="return to the previous activated release")
    back.add_argument("--to-version", default=None)
    # Deliberately no --python. The interpreter a release was activated with is
    # recorded in the state file; asking an operator to remember it years later
    # is how a rollback silently activates a unit set nobody ever ran.
    back.add_argument("--dry-run", action="store_true")

    sub.add_parser("status", help="report installed and active releases")

    takeover = sub.add_parser(
        "plan-takeover", help="inspect colliding legacy units without changing the machine"
    )
    takeover.add_argument("--profile", required=True, help="bounded lifecycle profile to adopt")
    takeover.add_argument(
        "--acknowledge-legacy-takeover",
        action="store_true",
        help="explicitly acknowledge inspection of an existing legacy installation",
    )
    revalidate = sub.add_parser(
        "revalidate-takeover", help="revalidate a takeover receipt without changing the machine"
    )
    revalidate.add_argument("--profile", required=True)
    revalidate.add_argument("--receipt", required=True, type=Path)

    bundle = sub.add_parser("support-bundle", help="write a redacted diagnostic bundle")
    bundle.add_argument("--output", type=Path, default=None, help="write here instead of stdout")
    bundle.add_argument(
        "--now",
        default=None,
        help="timestamp to stamp the bundle with; supply it for byte-identical output",
    )
    bundle.add_argument("--note", default="", help="free-text note for the ticket")
    return parser


#: Every failure this command can meet, mapped to a stable reason code. The
#: contract is "exactly one JSON object", so a traceback is a contract breach --
#: a fleet tool parsing stdout sees malformed output and cannot even report what
#: went wrong. Anything genuinely unforeseen still lands as `unexpected_error`,
#: whose action code is "collect a support bundle".
_FAILURE_REASONS: tuple[tuple[type[BaseException], str], ...] = (
    (LifecycleError, ""),  # carries its own reason
    (ProfileError, "profiles_invalid"),
    (NoteRejected, "note_rejected"),
    (SystemdError, ""),  # carries its own reason
    (PermissionError, "prefix_not_writable"),
    (FileNotFoundError, "release_payload_invalid"),
    (OSError, "io_failed"),
    (ValueError, "unexpected_error"),
)


def _reason_for(error: BaseException) -> tuple[str, str]:
    reason = getattr(error, "reason", "")
    if isinstance(reason, str) and reason:
        return reason, getattr(error, "detail", "") or str(error)
    for kind, mapped in _FAILURE_REASONS:
        if isinstance(error, kind) and mapped:
            return mapped, f"{type(error).__name__}: {error}"
    return "unexpected_error", f"{type(error).__name__}: {error}"


def main(argv: Sequence[str] | None = None, *, stream=None, systemd=None) -> int:
    args = _build_parser().parse_args(argv)
    layout = Layout(root=args.root.resolve())
    dry_run = bool(getattr(args, "dry_run", False))

    try:
        controller = systemd or _systemd_for(
            layout.root,
            no_systemd=args.no_systemd,
            dry_run=dry_run,
            command=args.command,
        )
        if args.command == "plan-takeover":
            report = plan_takeover(
                layout=layout,
                profile=args.profile,
                profiles=args.profiles,
                systemd=controller,
                acknowledged=args.acknowledge_legacy_takeover,
            )
        elif args.command == "revalidate-takeover":
            report = revalidate_takeover_receipt(
                receipt=read_takeover_receipt(args.receipt),
                layout=layout,
                profile=args.profile,
                profiles=args.profiles,
                systemd=controller,
            )
        elif args.command in {"install", "update"}:
            operation = lifecycle.install if args.command == "install" else lifecycle.update
            if bool(args.manifest) != bool(args.wheel_dir):
                raise LifecycleError(
                    "release_payload_invalid",
                    "--manifest and --wheel-dir must be supplied together",
                )
            if args.manifest is not None:
                report = operation(
                    layout=layout,
                    profile=args.profile,
                    python=args.python,
                    dry_run=dry_run,
                    systemd=controller,
                    profiles=args.profiles,
                    manifest=args.manifest,
                    wheel_dir=args.wheel_dir,
                )
            else:
                if args.version is None:
                    raise LifecycleError(
                        "release_payload_invalid",
                        "--version is required for legacy release sources",
                    )
                with _payload_from(args) as payload:
                    report = operation(
                        payload=payload,
                        version=args.version,
                        layout=layout,
                        profile=args.profile,
                        python=args.python,
                        dry_run=dry_run,
                        systemd=controller,
                        profiles=args.profiles,
                    )
        elif args.command == "rollback":
            report = lifecycle.rollback(
                layout=layout,
                to_version=args.to_version,
                dry_run=dry_run,
                systemd=controller,
                profiles=args.profiles,
            )
        elif args.command == "support-bundle":
            return _support_bundle(args, layout, controller, stream)
        else:
            report = lifecycle.status(layout, systemd=controller, profiles=args.profiles)
    except TakeoverError as error:
        report = {
            "schema": (
                LEGACY_REVALIDATION_SCHEMA
                if args.command == "revalidate-takeover"
                else LEGACY_TAKEOVER_SCHEMA
            ),
            "ok": False,
            "reason": error.reason,
        }
    except Exception as error:  # noqa: BLE001 - deliberate: one JSON object, always
        reason, detail = _reason_for(error)
        report = lifecycle.report(
            args.command,
            layout,
            ok=False,
            reason=reason,
            dry_run=dry_run,
            detail=detail,
        )
    return _emit(report, stream)


def _support_bundle(
    args: argparse.Namespace,
    layout: Layout,
    controller: SystemdController,
    stream,
) -> int:
    from .support_bundle import build_support_bundle, check_note, write_support_bundle

    now = args.now or datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    bundle = build_support_bundle(
        layout,
        now=now,
        note=check_note(args.note),
        systemd=controller,
        profiles=args.profiles,
    )
    if args.output is not None:
        written = write_support_bundle(args.output, bundle)
        return _emit(
            {
                "schema": lifecycle.LIFECYCLE_REPORT_VERSION,
                "action": "support-bundle",
                "ok": True,
                "dry_run": False,
                "reason": "ok",
                "action_code": "none",
                "detail": f"redacted bundle written to {written}",
                "paths": layout.as_dict(),
                "output": str(written),
                "bundle_reason": bundle.get("reason"),
            },
            stream,
        )
    # Building a bundle always succeeds if it returned at all; the device's own
    # health lives in `lifecycle.reason` inside the document, not in this exit
    # status. A ticket-collection command that exited non-zero on an unhealthy
    # device would be unusable in exactly the situation it exists for.
    return _emit({**bundle, "ok": True}, stream)


if __name__ == "__main__":
    raise SystemExit(main())
