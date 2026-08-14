"""Customer-grade install / update / rollback lifecycle for a Flyto machine.

The recovery installer in :mod:`flyto_robotics.recovery_install` is a Raspberry
Pi repair tool: it edits boot files and enables an out-of-band channel. It is
not a product installer, and nothing about it is versioned, reversible, or safe
to run against a fleet.

This module is the product half, and it is deliberately generic. Nothing here
imports ROS, requires a TurtleBot3, names a middleware, or knows a source
checkout exists. Unit sets come from :mod:`flyto_robotics.lifecycle_profiles`,
which reads them out of a shipped JSON file; ROS 2 is one profile in that file.
A customer on a different base installs, updates, and rolls back with the
identical commands.

Shape
-----

``releases/<version>`` is immutable. Publishing the same version twice with
different bytes is an error, not an overwrite, because "current" must name one
knowable set of files forever. The digest of every version ever activated is
recorded in a write-once provenance file that outlives the release directory, so
deleting a release does not re-open its name for different bytes. Activation is
a symlink replaced with ``os.replace``, which is atomic on POSIX: there is no
instant at which ``current`` is missing or half-written.

``config``, ``identity``, ``credentials``, ``diagnostics``, and logs live outside
``releases/`` and are never created, moved, or deleted by a release operation. An
update that fails validation therefore cannot cost a customer their device
credential, and a rollback cannot resurrect an old configuration.

Crash consistency
-----------------

Two writers cannot interleave: every mutating operation holds an advisory
``flock`` for its whole duration and a second caller is refused with
``operation_in_progress`` rather than being allowed to race. Durability is not
claimed loosely -- the state file, the provenance file, and the ``current``
symlink are written to a temporary name, ``fsync``-ed, renamed, and their parent
directory ``fsync``-ed, so a power cut leaves either the old object or the new
one. A crash mid-stage leaves a ``.<version>.staging`` directory, which the next
run removes; a crash between activation and the state write is detected by
:func:`status` as ``state_drift`` and repaired by re-running install.

Transactionality
----------------

Activation is a transaction over three things at once: the unit files, the
``current`` symlink, and the recorded state. Everything that can fail without a
device changing (payload scan, immutability, unit validation) is done first.
After that the operation either completes -- units written, systemd reloaded,
services enabled, restarted, and *verified running*, health check passed, state
recorded -- or every one of those three things is put back the way it was. A
first install that fails is undone to "nothing installed", which is recoverable
by re-running install. An update that fails is undone to the previous release,
which is still running when the command returns.

The pre-commit window
---------------------

Because units are restarted and *verified* before the state write that commits
them, there is a real interval during which a running service can find no
committed state. That interval is the only thing entitled to be reported as
"pending", so it is written down rather than inferred: the transaction opens a
durable :func:`activation_window` marker before it touches systemd and removes it
once the state is committed or the undo has finished. Absence of state without a
live window is therefore never "a fresh install still settling" -- it is a device
that was interrupted between activation and its state write, and it is refused
with ``state_drift`` (or ``not_installed`` when nothing is active at all). The
window is bounded by its own recorded duration and clamped to
:data:`ACTIVATION_WINDOW_SECONDS`, so a marker left behind by a killed installer
stops excusing anything within seconds rather than forever.

Every operation is idempotent, non-interactive (there is no prompt to answer, so
there is no ``--yes``), optionally a dry run that writes nothing at all, and
reports a single JSON object with stable reason/action codes.
"""

from __future__ import annotations

import copy
import errno
import hashlib
import json
import os
import re
import shutil
import stat
import tempfile
import time
from collections.abc import Callable, Iterable, Sequence
from contextlib import contextmanager
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .activation_snapshot import Snapshot, SnapshotError
from .activation_snapshot import build as build_activation_snapshot
from .activation_snapshot import load_document as load_activation_document
from .bootstrap import BootstrapError, bootstrap_release
from .fsio import atomic_write, fsync_path
from .health_codes import action_for
from .lifecycle_profiles import Profile, ProfileError, load_profiles, runbook_text
from .readiness import PROVISIONING_PENDING, READY, Readiness, evaluate
from .systemd_control import RecordingRunner, SystemdController, SystemdError
from .systemd_units import UnitDefect, validate_unit

__all__ = [
    "ACTIVATION_WINDOW_SECONDS",
    "LIFECYCLE_PROFILES_DEFAULT",
    "LIFECYCLE_PROVENANCE_VERSION",
    "LIFECYCLE_REPORT_VERSION",
    "LIFECYCLE_STATE_VERSION",
    "LIFECYCLE_WINDOW_VERSION",
    "Layout",
    "LifecycleError",
    "PROFILES",
    "activation_window",
    "activation_window_evidence",
    "close_activation_window",
    "committed_activation",
    "current_activation_snapshot",
    "install",
    "open_activation_window",
    "main",
    "profile_for",
    "read_activation_record",
    "read_activation_snapshot",
    "resolve_activation",
    "rehearsal_systemd",
    "release_digest",
    "render_units",
    "report",
    "rollback",
    "runtime_activation",
    "status",
    "update",
]

LIFECYCLE_REPORT_VERSION = "flyto.lifecycle-report.v1"
#: v2 adds activation identity: a ``current_activation`` id and an
#: ``activation_id`` on every history entry. The bump is not ceremony -- a v1
#: document uses ``history`` as a list keyed by version, and this build reads it
#: as an ordered log in which one version may appear more than once. Reading the
#: older document under the newer contract is exactly the "same key names, new
#: meaning" mistake that pinning a schema exists to prevent, and here it would
#: mean picking the wrong rollback target.
LIFECYCLE_STATE_VERSION = "flyto.lifecycle-state.v2"
#: The schema the immediately preceding build wrote. Still readable, because a
#: device installed by that build has to remain updatable -- refusing it would
#: mean the first release carrying activation identity is also the release that
#: strands every machine already in the field.
LIFECYCLE_STATE_VERSION_V1 = "flyto.lifecycle-state.v1"
_READABLE_STATE_SCHEMAS = (LIFECYCLE_STATE_VERSION, LIFECYCLE_STATE_VERSION_V1)
LIFECYCLE_PROVENANCE_VERSION = "flyto.release-provenance.v1"
#: The durable evidence that an activation is *live* and has not committed yet.
LIFECYCLE_WINDOW_VERSION = "flyto.activation-window.v1"
LIFECYCLE_PROFILES_DEFAULT = "generic"

#: The ceiling on how long a missing state file may be excused as "still
#: committing". A real transaction is milliseconds of state write behind a
#: systemd restart it has already verified, so this is enormous headroom; what it
#: buys is that a marker left by a killed installer, a full disk, or a power cut
#: stops excusing anything at a knowable moment instead of forever. It is a
#: clamp, not merely a default: a marker that asks for longer gets this.
ACTIVATION_WINDOW_SECONDS = 120.0

#: One fixed sentence for every way the marker can fail to verify. The marker is
#: attacker-writable on the devices this path exists for, and its refusal detail
#: is published verbatim into the agent's status document, the doctor snapshot,
#: and the journal -- so the refusal says *that* it did not verify and never
#: *what* it said. Which check tripped is not diagnostic anyway: the action is
#: identical, and the file is one an operator restores rather than repairs.
_WINDOW_REFUSAL = (
    "the activation window marker did not verify; its contents are untrusted and are "
    "deliberately not quoted. Remove it and re-run the install to reconcile this device"
)

_VERSION = re.compile(r"^[0-9A-Za-z][0-9A-Za-z._+-]{0,63}$")

#: Bound on a payload scan, so a pointed-at ``/`` cannot hang the installer.
_MAX_PAYLOAD_ENTRIES = 20000

#: History is bounded: a device that has updated hundreds of times does not need
#: an unbounded state file, and only recent releases are still on disk anyway.
_MAX_HISTORY = 20

#: Declared once. ``/var/lib/flyto-robot`` holds the credential directory, so
#: the mode is a security property and not a preference -- and it is enforced in
#: two places (the advisory lock creates the directory before
#: :func:`_ensure_persistent` ever runs), which is exactly the sort of pair that
#: drifts when the number is written twice.
_STATE_DIR_MODE = 0o750

#: A release digest is a sha256 hexdigest and nothing else. Anything that is not
#: this shape in a persisted file is a corrupted record, not a digest to compare.
_DIGEST = re.compile(r"^[0-9a-f]{64}$")

#: ``is-active`` answers that mean there is nothing running to stop -- including
#: the case where systemd has never loaded the unit at all and is answering
#: about a name rather than a service. Anything else (``active``, ``activating``,
#: ``failed``) is a unit an undo has to quiesce before deleting its file.
_INERT_ACTIVE_STATES = frozenset({"inactive", "unknown"})


def _profile_names() -> tuple[str, ...]:
    try:
        return tuple(sorted(load_profiles()))
    except ProfileError:  # pragma: no cover - a broken shipped asset
        return (LIFECYCLE_PROFILES_DEFAULT,)


#: Declared by data, not by this module. Kept as a tuple for argparse choices.
PROFILES = _profile_names()


class LifecycleError(RuntimeError):
    """A lifecycle operation refused to proceed. Carries a reason code."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


# ---------------------------------------------------------------------------
# Layout
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Layout:
    """Absolute product paths, rooted at ``root``.

    ``root`` is ``/`` in production and a temporary directory in tests. It is
    the only thing that moves: every path below it is fixed, so a unit file, a
    runbook, and a support bundle can all name the same string. No path contains
    a login user name, and none points into a source checkout.
    """

    root: Path = Path("/")

    @property
    def prefix(self) -> Path:
        return self.root / "opt/flyto-robot"

    @property
    def releases(self) -> Path:
        return self.prefix / "releases"

    @property
    def current(self) -> Path:
        return self.prefix / "current"

    @property
    def config_dir(self) -> Path:
        return self.root / "etc/flyto-robot"

    @property
    def state_dir(self) -> Path:
        return self.root / "var/lib/flyto-robot"

    @property
    def log_dir(self) -> Path:
        return self.root / "var/log/flyto-robot"

    @property
    def unit_dir(self) -> Path:
        return self.root / "etc/systemd/system"

    @property
    def identity_file(self) -> Path:
        return self.config_dir / "identity.json"

    @property
    def config_file(self) -> Path:
        return self.config_dir / "robot.env"

    @property
    def runbook_file(self) -> Path:
        return self.config_dir / "README-runbook.txt"

    @property
    def credentials_dir(self) -> Path:
        return self.state_dir / "credentials"

    @property
    def diagnostics_dir(self) -> Path:
        return self.state_dir / "diagnostics"

    @property
    def state_file(self) -> Path:
        return self.state_dir / "lifecycle-state.json"

    @property
    def lock_file(self) -> Path:
        return self.state_dir / "lifecycle.lock"

    @property
    def activation_window_file(self) -> Path:
        """Evidence that an activation is in flight and has not committed.

        Beside the state file it is about, and deliberately not inside
        ``releases/``: it describes the transaction, not a release, and it has to
        be readable by a service that knows only ``--state-dir``.
        """

        return self.state_dir / "activation-window.json"

    @property
    def provenance_dir(self) -> Path:
        """Write-once digests, kept outside ``releases/``.

        A release directory can be deleted to reclaim space. Its provenance may
        not, or the version name would become re-usable for different bytes and
        two devices could report the same version while running different code.
        """

        return self.state_dir / "releases"

    def provenance_file(self, version: str) -> Path:
        return self.provenance_dir / f"{version}.json"

    @property
    def activation_dir(self) -> Path:
        """Activation snapshots.

        Beside provenance and for the same reason: what a version *was
        activated as* has to outlive the registry that described it, or a
        rollback depends on a file a site is free to edit or delete.
        """

        return self.state_dir / "activations"

    def activation_file(self, version: str) -> Path:
        """Where a *version* currently resolves to: its newest activation.

        This is the file the running services and :func:`status` read to learn
        which readiness contract the machine is actually under, so it names the
        version rather than the activation -- a unit cannot know an activation
        id, and asking it to would put the id in the unit text, which changes
        the unit text, which changes the id.

        It is therefore a view, not the record: re-activating the same version
        under a different profile legitimately repoints it. The record it points
        at is immutable and lives in :meth:`activation_record_file`.
        """

        return self.activation_dir / f"{version}.json"

    @property
    def activation_record_dir(self) -> Path:
        """The immutable, content-addressed activation records.

        A subdirectory rather than a suffix in the same namespace, because a
        version name is allowed to be 64 hex characters and an activation id
        always is. Sharing one directory would let a release published as
        ``aaaa...`` collide with a real activation record -- and the file that
        loses that collision is the one a rollback replays.
        """

        return self.activation_dir / "by-id"

    def activation_record_file(self, activation_id: str) -> Path:
        if not _DIGEST.fullmatch(activation_id):
            raise LifecycleError(
                "activation_snapshot_invalid", f"unsafe activation id {activation_id!r}"
            )
        return self.activation_record_dir / f"{activation_id}.json"

    def persistent_paths(self) -> tuple[Path, ...]:
        """Surfaces a release operation must never create, move, or delete."""

        return (
            self.config_dir,
            self.state_dir,
            self.log_dir,
            self.credentials_dir,
            self.diagnostics_dir,
        )

    def release(self, version: str) -> Path:
        if not _VERSION.fullmatch(version):
            raise LifecycleError("release_payload_invalid", f"unsafe version {version!r}")
        return self.releases / version

    @classmethod
    def for_state_dir(cls, state_dir: Path | str) -> Layout:
        """Recover the layout a *running unit* belongs to from its ``--state-dir``.

        The installed units are rendered with absolute paths and are not given a
        ``--root``: they name the directories they use directly, because a unit
        that carried a root would have to be re-rendered to move and would then
        no longer be byte-identical across the profiles that inherit it. A
        running service still has to ask the lifecycle which activation it is
        under, and that question is asked of a :class:`Layout`.

        So the one fixed relationship is inverted here rather than guessed at in
        the caller: ``state_dir`` is always ``<root>/var/lib/flyto-robot``, and a
        path that is not shaped that way is refused instead of being resolved to
        a root that would silently point every later read somewhere else.
        """

        parts = Path(state_dir).parts
        suffix = Path("var/lib/flyto-robot").parts
        if parts[-len(suffix):] != suffix:
            raise LifecycleError(
                "config_unreadable",
                f"{state_dir} is not a flyto-robot state directory; expected a path ending "
                f"in {'/'.join(suffix)}",
            )
        root = Path(*parts[: -len(suffix)]) if len(parts) > len(suffix) else Path(".")
        return cls(root=root)

    def as_dict(self) -> dict[str, str]:
        return {
            "prefix": str(self.prefix),
            "releases": str(self.releases),
            "current": str(self.current),
            "config_dir": str(self.config_dir),
            "state_dir": str(self.state_dir),
            "log_dir": str(self.log_dir),
            "unit_dir": str(self.unit_dir),
        }


# ---------------------------------------------------------------------------
# Durability helpers
# ---------------------------------------------------------------------------


#: Durability primitives live in :mod:`flyto_robotics.fsio` so the support
#: bundle can use the identical writer without importing a private name.
_atomic_write = atomic_write


def _fsync_path(path: Path, *, directory: bool = False) -> None:
    fsync_path(path, directory=directory)


@contextmanager
def _advisory_lock(layout: Layout, *, enabled: bool = True):
    """Hold an exclusive advisory lock for the whole operation.

    Two installers running at once is not a theoretical race: a fleet tool
    retrying a slow command is the ordinary way it happens, and the loser would
    otherwise interleave a symlink swap with someone else's unit write. The lock
    is non-blocking and refuses rather than queues -- an operator who is told
    ``operation_in_progress`` can retry, whereas one whose command silently
    blocked for ten minutes files a bug about a hang.
    """

    if not enabled:
        yield None
        return
    try:
        import fcntl
    except ImportError:  # pragma: no cover - POSIX only in production
        yield None
        return

    # The lock lives in the state directory, so the *first* operation a device
    # ever runs creates that directory here -- before `_ensure_persistent` has
    # had a chance to declare its mode. `mkdir` applies the process umask, which
    # on a default system leaves /var/lib/flyto-robot group- and world-readable
    # for the whole of that first install, and `credentials/` is created
    # underneath it. Declare the mode at creation *and* enforce it, because
    # `mode=` is masked by the umask and says nothing about a directory some
    # earlier tool already left loose.
    layout.state_dir.mkdir(parents=True, exist_ok=True, mode=_STATE_DIR_MODE)
    if (layout.state_dir.stat().st_mode & 0o777) != _STATE_DIR_MODE:
        layout.state_dir.chmod(_STATE_DIR_MODE)
    descriptor = os.open(layout.lock_file, os.O_RDWR | os.O_CREAT, 0o640)
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as error:
            if error.errno in {errno.EACCES, errno.EAGAIN, errno.EWOULDBLOCK}:
                raise LifecycleError("operation_in_progress", str(layout.lock_file)) from error
            raise
        try:
            yield descriptor
        finally:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
    finally:
        os.close(descriptor)


# ---------------------------------------------------------------------------
# Mutation journal: idempotency and dry-run share one implementation
# ---------------------------------------------------------------------------


@dataclass
class _Fs:
    """Records intended changes; performs them only when not a dry run.

    Every method compares before it writes, so "idempotent" and "dry-runnable"
    are the same code path rather than two behaviours that can drift. A second
    identical run produces an empty ``changed`` list; a dry run produces the
    list a real run would produce and touches nothing.
    """

    dry_run: bool = False
    changed: list[str] = field(default_factory=list)

    def _note(self, verb: str, path: Path) -> None:
        self.changed.append(f"{verb} {path}")

    def mkdir(self, path: Path, mode: int = 0o755) -> None:
        if path.is_dir():
            if (path.stat().st_mode & 0o777) != mode:
                # Modes are enforced, not merely set on creation. A credentials
                # directory that some earlier tool left at 0755 is a leak that
                # no amount of correct new code fixes by itself.
                self._note("chmod", path)
                if not self.dry_run:
                    path.chmod(mode)
            return
        self._note("create-dir", path)
        if not self.dry_run:
            path.mkdir(parents=True, exist_ok=True)
            path.chmod(mode)

    def write(self, path: Path, text: str, mode: int = 0o644) -> None:
        if path.is_file() and path.read_text(encoding="utf-8") == text:
            if (path.stat().st_mode & 0o777) == mode:
                return
            self._note("chmod", path)
            if not self.dry_run:
                path.chmod(mode)
            return
        self._note("write", path)
        if self.dry_run:
            return
        _atomic_write(path, text, mode)

    def remove_file(self, path: Path) -> None:
        if not path.is_file() and not path.is_symlink():
            return
        self._note("remove", path)
        if self.dry_run:
            return
        path.unlink()
        _fsync_path(path.parent, directory=True)

    def copy_tree(self, source: Path, destination: Path) -> None:
        self._note("stage-release", destination)
        if self.dry_run:
            return
        if destination.exists():
            self.thaw(destination)
            shutil.rmtree(destination)
        # ``symlinks=True`` copies a link as a link rather than dereferencing it.
        # The payload scan has already refused any symlink, so this is belt and
        # braces: it means a scan bug cannot turn into "copied /etc/shadow in".
        shutil.copytree(source, destination, symlinks=True)

    def freeze(self, root: Path) -> None:
        """Drop the write bit across a staged release.

        A release that can be edited in place is not a release: the digest
        recorded at activation would stop describing what is running, and a
        rollback would return to something that had since been changed.
        """

        self._note("freeze-release", root)
        if self.dry_run:
            return
        for path in sorted(root.rglob("*"), reverse=True):
            path.chmod(path.stat().st_mode & ~0o222)
        root.chmod(root.stat().st_mode & ~0o222)
        _fsync_path(root, directory=True)

    def thaw(self, root: Path) -> None:
        if self.dry_run or not root.exists():
            return
        root.chmod(root.stat().st_mode | 0o200)
        for path in root.rglob("*"):
            path.chmod(path.stat().st_mode | 0o200)

    def remove_tree(self, root: Path) -> None:
        if not root.exists():
            return
        self._note("remove", root)
        if self.dry_run:
            return
        self.thaw(root)
        shutil.rmtree(root)

    def point_current(self, link: Path, target: Path) -> None:
        """Atomically repoint ``link`` at ``target``.

        ``os.replace`` on a symlink is atomic, so a reader either sees the old
        release or the new one. A non-atomic unlink-then-symlink would leave a
        window in which ``current`` does not resolve, and a service that
        restarted inside that window would fail for a reason unrelated to the
        release being installed.
        """

        if link.is_symlink() and Path(os.readlink(link)) == target:
            return
        if link.exists() and not link.is_symlink():
            raise LifecycleError("current_symlink_foreign", str(link))
        self._note("activate", target)
        if self.dry_run:
            return
        link.parent.mkdir(parents=True, exist_ok=True)
        staging = link.with_name(f".{link.name}.staged")
        if staging.is_symlink() or staging.exists():
            staging.unlink()
        staging.symlink_to(target)
        os.replace(staging, link)
        _fsync_path(link.parent, directory=True)

    def clear_current(self, link: Path) -> None:
        if not link.is_symlink():
            return
        self._note("deactivate", link)
        if self.dry_run:
            return
        link.unlink()
        _fsync_path(link.parent, directory=True)


# ---------------------------------------------------------------------------
# Release content
# ---------------------------------------------------------------------------


def release_digest(root: Path) -> str:
    """Content digest of a release tree: path, executable bit, and bytes.

    Mode is folded in because an ``ExecStart=`` target that lost its executable
    bit is a different release even though every byte matches.
    """

    hasher = hashlib.sha256()
    for path in sorted(p for p in root.rglob("*") if p.is_file() and not p.is_symlink()):
        relative = path.relative_to(root).as_posix()
        executable = "1" if path.stat().st_mode & 0o111 else "0"
        hasher.update(f"{relative}\0{executable}\0".encode())
        hasher.update(hashlib.sha256(path.read_bytes()).digest())
    return hasher.hexdigest()


def _scan_payload(payload: Path) -> int:
    """Refuse a payload that could escape its own directory.

    A symlink inside a payload is copied into an immutable release and then
    resolved by a service running as root. ``../../etc/shadow`` and
    ``/dev/mem`` are both one careless tarball away, and a digest over the
    *link targets* would still match on the next device. Sockets, devices, and
    FIFOs are refused for the same reason: they are not release content.
    """

    if not payload.is_dir() or payload.is_symlink():
        raise LifecycleError("release_payload_invalid", f"{payload} is not a directory")
    seen = 0
    for path in payload.rglob("*"):
        seen += 1
        if seen > _MAX_PAYLOAD_ENTRIES:
            raise LifecycleError(
                "release_payload_invalid",
                f"payload exceeds {_MAX_PAYLOAD_ENTRIES} entries; this is not a release",
            )
        if path.is_symlink():
            raise LifecycleError(
                "release_payload_invalid",
                f"payload contains a symlink: {path.relative_to(payload).as_posix()}",
            )
        mode = path.lstat().st_mode
        if not (stat.S_ISREG(mode) or stat.S_ISDIR(mode)):
            raise LifecycleError(
                "release_payload_invalid",
                f"payload contains a non-regular file: {path.relative_to(payload).as_posix()}",
            )
    if seen == 0:
        raise LifecycleError("release_payload_invalid", f"{payload} is empty")
    return seen


# ---------------------------------------------------------------------------
# Unit rendering (data-driven; see lifecycle_profiles)
# ---------------------------------------------------------------------------


def profile_for(name: str, *, profiles: Path | str | None = None) -> Profile:
    try:
        registry = load_profiles(profiles)
    except ProfileError as error:
        raise LifecycleError("profiles_invalid", str(error)) from error
    if name not in registry:
        raise LifecycleError(
            "profiles_invalid", f"unknown profile {name!r}; known: {sorted(registry)}"
        )
    return registry[name]


def render_units(
    layout: Layout,
    *,
    profile: str = LIFECYCLE_PROFILES_DEFAULT,
    python: str = "/usr/bin/python3",
    profiles: Path | str | None = None,
) -> dict[str, str]:
    """Render the unit set for ``profile``.

    Profiles are additive by construction: the registry refuses a profile that
    redefines an inherited unit, so a customer can adopt or drop an adapter
    profile without their base install changing underneath them.
    """

    return profile_for(profile, profiles=profiles).render(render_fields(layout, python))


def render_fields(layout: Layout, python: str) -> dict[str, str]:
    """The substitution mapping a unit template and a readiness check share.

    One mapping, one source of truth for where things live: a readiness check
    that computed its own path could pass while the unit it is supposed to be
    guarding reads a different file.
    """

    return {
        "current": str(layout.current),
        "config_dir": str(layout.config_dir),
        "config_file": str(layout.config_file),
        "identity_file": str(layout.identity_file),
        "state_dir": str(layout.state_dir),
        "log_dir": str(layout.log_dir),
        "python": python,
    }


def _validate_units(units: dict[str, str]) -> list[UnitDefect]:
    defects: list[UnitDefect] = []
    for name in sorted(units):
        defects.extend(validate_unit(units[name], name=name))
    return defects


def _activation_condition_met(unit, fields: dict[str, str]) -> bool:
    """Evaluate one declarative activation condition without following links.

    A credential-shaped name is not enough authority to start the runner.  A
    missing path, a failed metadata lookup, a symlink, or any non-regular
    object all mean "not provisioned yet".  Conditions only select lifecycle
    verbs; unit files are still installed and enabled so a path unit can start
    the service when pairing publishes the credential atomically.
    """

    if unit.condition is None:
        return True
    try:
        path = unit.condition.render(fields)
        metadata = os.stat(path, follow_symlinks=False)
    except (OSError, ProfileError):
        return False
    return stat.S_ISREG(metadata.st_mode)


def _activation_conditions(spec: Profile, fields: dict[str, str]) -> frozenset[str]:
    """Take one immutable condition snapshot for a transition."""

    return frozenset(
        unit.name for unit in spec.units if _activation_condition_met(unit, fields)
    )


def _selected_unit_names(
    spec: Profile, conditions: frozenset[str], policy: str
) -> tuple[str, ...]:
    return tuple(
        unit.name
        for unit in spec.units
        if bool(getattr(unit, policy)) and unit.name in conditions
    )


def _inactive_conditional_units(
    spec: Profile, conditions: frozenset[str]
) -> tuple[str, ...]:
    return tuple(
        unit.name
        for unit in spec.units
        if unit.condition is not None and unit.name not in conditions
    )


def rehearsal_systemd(*, dry_run: bool = False) -> SystemdController:
    """A controller that records what it would do and touches no host."""

    return SystemdController(runner=RecordingRunner(), dry_run=dry_run, mode="recording")


# ---------------------------------------------------------------------------
# Persisted state and provenance
# ---------------------------------------------------------------------------


def _empty_state() -> dict:
    return {
        "schema": LIFECYCLE_STATE_VERSION,
        "current": None,
        "current_activation": None,
        "profile": None,
        "history": [],
    }


def _corrupt(path: Path, what: str) -> LifecycleError:
    return LifecycleError("config_unreadable", f"{path}: {what}")


def _upgrade_v1_state(data: dict, layout: Layout) -> dict:
    """Derive v2 activation identity for a device installed by the v1 build.

    v1 recorded history keyed by version and wrote exactly one snapshot per
    version, at the path v2 still keeps as the version *view*. So every v1
    history entry has a snapshot on disk, and that snapshot's digest is, by
    construction, the activation id v2 would have given it. The upgrade
    therefore *reads* identity rather than inventing it.

    Nothing is inferred. Every entry must have a snapshot that verifies against
    its own digest and agrees with the history entry on release digest, profile,
    interpreter, and every unit digest. A single disagreement refuses the whole
    document, because a half-believed history is one that names a rollback
    target nobody can reproduce -- and quietly minting an id for an entry whose
    snapshot is gone would hand exactly that back as if it were trustworthy.

    Purely in memory: a read does not write. The upgraded document is persisted
    by the next mutating operation's atomic state write, and the by-id records
    are materialised under the same lock by :func:`_materialise_activation_records`.
    """

    path = layout.state_file
    upgraded = copy.deepcopy(data)
    upgraded["schema"] = LIFECYCLE_STATE_VERSION

    history = upgraded.get("history", [])
    if not isinstance(history, list):
        raise _corrupt(path, "history is not a list")

    for entry in history:
        if not isinstance(entry, dict):
            raise _corrupt(path, f"history entry is not an object: {entry!r}")
        version = entry.get("version")
        if not (isinstance(version, str) and _VERSION.fullmatch(version)):
            raise _corrupt(path, f"history entry has no usable version: {entry!r}")

        view = layout.activation_file(version)
        if not view.is_file():
            raise _corrupt(
                path,
                f"{version} is in the v1 history but has no activation snapshot at {view}, "
                "so its activation cannot be identified; restore the file or remove the entry",
            )
        snapshot = _load_snapshot_file(view, version=version)

        for field_name, recorded, observed in (
            ("digest", entry.get("digest"), snapshot.release_digest),
            ("profile", entry.get("profile"), snapshot.profile),
            ("python", entry.get("python"), snapshot.python),
            ("units", entry.get("units"), unit_digests(snapshot.units)),
        ):
            if recorded != observed:
                raise _corrupt(
                    path,
                    f"the v1 history entry for {version} disagrees with its own activation "
                    f"snapshot on {field_name}; the two records cannot both be true",
                )
        entry["activation_id"] = snapshot.activation_id

    current = upgraded.get("current")
    if current is None:
        upgraded["current_activation"] = None
        return upgraded

    if not history:
        raise _corrupt(path, f"current is {current!r} but the v1 history is empty")
    # v1 appended the activation it was recording and set `current` in the same
    # breath, so the newest entry is the live one. Anything else is a document
    # v1 never wrote.
    newest = history[-1]
    if newest.get("version") != current:
        raise _corrupt(
            path,
            f"current is {current!r} but the newest v1 history entry is "
            f"{newest.get('version')!r}",
        )
    upgraded["current_activation"] = newest["activation_id"]
    return upgraded


def _materialise_activation_records(fs: _Fs, layout: Layout, state: dict) -> None:
    """Give every history entry a by-id record, from the version view if needed.

    The one write the v1 upgrade needs, and it is content addressed: a record is
    only ever written under the id its own bytes hash to, so this cannot
    fabricate an activation. A view that does not hash to the id history claims
    refuses instead of being adopted, which is the same fail-closed rule a
    tampered record gets.

    A no-op on any device this build has already written, so it costs one
    ``is_file`` per history entry and nothing else.
    """

    for entry in state.get("history", []):
        activation_id = entry["activation_id"]
        if layout.activation_record_file(activation_id).is_file():
            continue
        view = layout.activation_file(entry["version"])
        if not view.is_file():
            raise LifecycleError(
                "activation_not_recorded",
                f"{entry['version']} is in the recorded history but neither its activation "
                f"record nor its snapshot is on this device",
            )
        # Strict on both axes: it must be the version it claims and the exact
        # activation history references.
        snapshot = _load_snapshot_file(view, version=entry["version"], activation_id=activation_id)
        fs.mkdir(layout.activation_dir, 0o750)
        fs.mkdir(layout.activation_record_dir, 0o750)
        fs.write(
            layout.activation_record_file(activation_id),
            json.dumps(snapshot.document(), indent=2, sort_keys=True) + "\n",
            0o640,
        )


def _load_state_text(layout: Layout) -> str | None:
    """The state file's bytes, read **once**, or ``None`` if it is not there.

    One read, because two reads of one path are two different files whenever a
    writer is running: the state file is replaced atomically, so a reader that
    consults it twice can classify the schema of one document and validate the
    fields of another. That is not a hypothetical -- ``os.replace`` between the
    two reads is exactly what every install does -- and the consequence was
    authority-shaped rather than cosmetic: a v1 classification granted the
    version-view fallback to a v2 state, which is the one path that lets a
    mutable file stand in for the immutable record.

    ``FileNotFoundError`` is the only absence. Every other ``OSError`` is a
    refusal with a code, because "the state file is unreadable" and "this device
    has never been installed" must not become the same answer.
    """

    try:
        return layout.state_file.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as error:
        raise LifecycleError("config_unreadable", str(error)) from error


def _decode_state(layout: Layout, text: str) -> tuple[dict, str]:
    """Validate one already-read state document, or refuse it.

    Every later operation treats this document as fact: rollback picks a target
    out of ``history``, compares the recorded digest against the bytes on disk,
    and decides which profile's units to restart from it. A record that is
    merely *present* is not enough -- a history entry whose ``digest`` is null
    silently disables the "the rollback target is still the release that was
    activated" check, and a ``profile`` that is a number crashes inside
    ``profile_for`` with a traceback rather than a reason code. So the shape is
    checked once, here, and anything that does not match is ``config_unreadable``
    with an action that tells the operator to restore the file.

    Returns the validated document *and* the schema it declared, so a caller that
    needs to know whether this arrived as a raw v1 document gets that fact from
    the very bytes that were validated rather than from a second look at a path.
    """

    try:
        data = json.loads(text)
    except ValueError as error:
        raise LifecycleError("config_unreadable", str(error)) from error
    if not isinstance(data, dict):
        raise _corrupt(layout.state_file, "lifecycle state is not an object")

    path = layout.state_file
    # Required, not defaulted. Reading a missing schema as "must be the current
    # one" infers authority from absence: an older or hand-written document
    # gets interpreted under a contract it never agreed to, which is precisely
    # the mistake pinning a schema exists to prevent. Every state file this
    # build has ever written carries the field, so absence means the file did
    # not come from this product -- and there is no legacy schema-less format to
    # migrate, because the field predates the first release.
    if "schema" not in data:
        raise _corrupt(path, "state file declares no schema")
    schema = data["schema"]
    declared = schema if isinstance(schema, str) else ""
    if schema not in _READABLE_STATE_SCHEMAS:
        # Not "upgrade it": a document written under a schema this build has
        # never seen may mean something different by the same key names, and
        # guessing is how a device rolls back onto the wrong release. The one
        # exception is the schema this build's own predecessor wrote, whose
        # meaning is known exactly -- and even that is derived from records on
        # disk rather than assumed.
        raise _corrupt(path, f"unknown state schema {schema!r}")
    if schema == LIFECYCLE_STATE_VERSION_V1:
        data = _upgrade_v1_state(data, layout)

    current = data.get("current")
    if current is not None and not (isinstance(current, str) and _VERSION.fullmatch(current)):
        raise _corrupt(path, f"current is not a version: {current!r}")

    # Which *activation* is live, not merely which version. The two are checked
    # as a pair: a device that names a current version without naming the
    # activation behind it cannot say which unit set it is running when the same
    # version has been activated more than once, and recovery would have to
    # guess. Refusing is the only answer that cannot restart the wrong services.
    current_activation = data.get("current_activation")
    if current is None:
        if current_activation is not None:
            raise _corrupt(path, "current_activation is recorded with no current version")
    elif not (isinstance(current_activation, str) and _DIGEST.fullmatch(current_activation)):
        raise _corrupt(path, f"current_activation is not an activation id: {current_activation!r}")

    profile = data.get("profile")
    if profile is not None and not (isinstance(profile, str) and profile):
        raise _corrupt(path, f"profile is not a name: {profile!r}")

    history = data.get("history", [])
    if not isinstance(history, list):
        raise _corrupt(path, "history is not a list")
    for entry in history:
        if not isinstance(entry, dict):
            raise _corrupt(path, f"history entry is not an object: {entry!r}")
        version = entry.get("version")
        if not (isinstance(version, str) and _VERSION.fullmatch(version)):
            raise _corrupt(path, f"history entry has no usable version: {entry!r}")
        digest = entry.get("digest")
        if not (isinstance(digest, str) and _DIGEST.fullmatch(digest)):
            raise _corrupt(path, f"history entry for {version} has no usable digest")
        entry_profile = entry.get("profile")
        if not (isinstance(entry_profile, str) and entry_profile):
            raise _corrupt(path, f"history entry for {version} has no usable profile")
        entry_python = entry.get("python")
        if not (isinstance(entry_python, str) and entry_python):
            raise _corrupt(path, f"history entry for {version} records no interpreter")
        entry_units = entry.get("units")
        if not isinstance(entry_units, dict) or not entry_units:
            raise _corrupt(path, f"history entry for {version} records no unit digests")
        for unit_name, unit_digest in entry_units.items():
            if not (isinstance(unit_digest, str) and _DIGEST.fullmatch(unit_digest)):
                raise _corrupt(path, f"{version}/{unit_name} has no usable unit digest")
        # The rollback target is an activation, so every entry has to name one.
        # Without it the history is only a list of version names, and two
        # activations of one version become indistinguishable -- which is how a
        # rollback lands on the profile the operator was trying to leave.
        entry_activation = entry.get("activation_id")
        if not (isinstance(entry_activation, str) and _DIGEST.fullmatch(entry_activation)):
            raise _corrupt(path, f"history entry for {version} has no usable activation id")

    # Shapes are not enough. Every field above can be individually well formed
    # while the document as a whole describes a device that never existed, and
    # the fields that decide which unit set recovery restores are exactly the
    # ones worth editing: point `current_activation` at an older entry and the
    # next failed update "recovers" a machine onto a unit set it was not
    # running. So the live reference is checked against the log it indexes.
    if current is not None:
        if not history:
            raise _corrupt(path, f"current is {current!r} but history is empty")
        newest = history[-1]
        if newest["activation_id"] != current_activation:
            raise _corrupt(
                path,
                "current_activation does not name the newest history entry; the recorded "
                "history and the recorded current activation disagree about what is running",
            )
        if newest["version"] != current:
            raise _corrupt(
                path,
                f"current is {current!r} but the current activation records "
                f"{newest['version']!r}",
            )
        if profile is None or newest["profile"] != profile:
            raise _corrupt(
                path,
                f"profile is {profile!r} but the current activation was made under "
                f"{newest['profile']!r}",
            )

    data["history"] = history
    data["current"] = current
    data["current_activation"] = current_activation
    data["profile"] = profile
    return data, declared


def _read_state(layout: Layout) -> dict:
    """The committed state, or the empty document when nothing is installed."""

    text = _load_state_text(layout)
    if text is None:
        return _empty_state()
    return _decode_state(layout, text)[0]


def _write_state(fs: _Fs, layout: Layout, state: dict) -> None:
    state["schema"] = LIFECYCLE_STATE_VERSION
    fs.write(layout.state_file, json.dumps(state, indent=2, sort_keys=True) + "\n", 0o640)


def unit_digests(units: dict[str, str]) -> dict[str, str]:
    """Content digest of every rendered unit, by name."""

    return {
        name: hashlib.sha256(text.encode("utf-8")).hexdigest()
        for name, text in sorted(units.items())
    }


def _record_activation(
    state: dict,
    version: str,
    digest: str,
    profile: str,
    python: str,
    units: dict[str, str],
    activation_id: str,
) -> None:
    """Record enough to *reproduce* this activation, not merely to name it.

    Version, digest, and profile name what ran. They do not reproduce it. A unit
    set is rendered from three inputs -- the registry template, the layout, and
    the interpreter path -- and two of those can change underneath a device
    without the version changing: a site edits a template under the same profile
    name, or the original install was run with a non-default ``--python``. A
    rollback that re-renders from today's inputs would then activate a unit set
    that was never tested and never ran here, under the name of one that was.

    So the interpreter is persisted and every rendered unit's digest with it,
    under the id of the activation that produced them.

    History is an ordered log of *activations*, not a table keyed by version.
    Superseding by version was the bug: activating ``1.0.0`` under ``ros2``
    deleted the record of ``1.0.0`` under ``generic``, so the operator undoing
    the profile switch had no prior entry to return to and rollback skipped
    straight past to whatever came before -- a different release entirely.
    Entries are superseded by activation id instead, which keeps a re-run of the
    identical activation from growing the log while keeping two genuinely
    different activations of one version as the two rollback targets they are.

    Only non-secret facts are stored: paths and hashes, never file contents.
    """

    history = [
        entry for entry in state["history"] if entry.get("activation_id") != activation_id
    ]
    history.append(
        {
            "version": version,
            "digest": digest,
            "profile": profile,
            "python": python,
            "units": unit_digests(units),
            "activation_id": activation_id,
        }
    )
    state["history"] = history[-_MAX_HISTORY:]
    state["current"] = version
    state["current_activation"] = activation_id
    state["profile"] = profile


def _read_provenance(layout: Layout, version: str) -> dict | None:
    path = layout.provenance_file(version)
    if not path.is_file():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise LifecycleError("config_unreadable", f"{path}: {error}") from error
    if not isinstance(data, dict):
        raise _corrupt(path, "provenance is not an object")
    # Exact shape, not "has a digest key". Provenance is the record that makes a
    # version name mean one set of bytes forever, so a record that carries a
    # different schema, names a different version, or has been extended with
    # fields this build does not understand is not a weaker record -- it is a
    # record we cannot honestly compare against, and comparing anyway is how a
    # version silently gets republished with different content.
    if set(data) != {"schema", "version", "digest"}:
        raise _corrupt(path, f"provenance fields are {sorted(data)}")
    if data["schema"] != LIFECYCLE_PROVENANCE_VERSION:
        raise _corrupt(path, f"unknown provenance schema {data['schema']!r}")
    if data["version"] != version:
        raise _corrupt(path, f"provenance records version {data['version']!r}")
    if not (isinstance(data["digest"], str) and _DIGEST.fullmatch(data["digest"])):
        raise _corrupt(path, "provenance digest is not a sha256 hexdigest")
    return data


def _provenance_record(version: str, digest: str) -> dict:
    """What a version name means, forever.

    Deliberately only content facts. Which *profile* a version was installed
    under is a property of the device and can legitimately change (a site adopts
    the ROS 2 adapter without republishing the release), so it lives in the
    mutable state file. Putting it here would either forbid a legal profile
    switch or force provenance to be rewritten -- and provenance that can be
    rewritten proves nothing.
    """

    return {"schema": LIFECYCLE_PROVENANCE_VERSION, "version": version, "digest": digest}


def _write_provenance(fs: _Fs, layout: Layout, version: str, digest: str) -> None:
    """Write once, then never again.

    An existing record is not overwritten and not merged: if it differs in any
    field the operation is refused. Silently rewriting it would let a version
    name mean two different things over a device's life, which is the one thing
    the whole immutability argument rests on.
    """

    fs.mkdir(layout.provenance_dir, 0o750)
    record = _provenance_record(version, digest)
    path = layout.provenance_file(version)
    existing = _read_provenance(layout, version)
    if existing is not None:
        if existing != record:
            raise LifecycleError(
                "release_exists_with_different_content",
                f"{path} already records {json.dumps(existing, sort_keys=True)}",
            )
        return
    fs.write(path, json.dumps(record, indent=2, sort_keys=True) + "\n", 0o640)


def _load_snapshot_file(
    path: Path, *, version: str | None = None, activation_id: str | None = None
) -> Snapshot:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise LifecycleError("activation_snapshot_invalid", f"{path}: {error}") from error
    try:
        return load_activation_document(
            document, path=path, version=version, activation_id=activation_id
        )
    except SnapshotError as error:
        raise LifecycleError("activation_snapshot_invalid", str(error)) from error


def read_activation_snapshot(layout: Layout, version: str) -> Snapshot | None:
    """Load what ``version`` currently resolves to, or ``None`` if nothing does.

    This is the *view* -- the newest activation of that version -- and it is
    what a running service and :func:`status` ask for, because those callers
    know a version and cannot know an activation id.

    Malformed or tampered snapshots are *not* ``None``. Returning "no snapshot"
    for a file that exists and does not verify would send the caller down the
    "this predates snapshots" path and silently reproduce an activation from
    whatever the registry happens to say today -- the exact substitution the
    snapshot exists to prevent.
    """

    path = layout.activation_file(version)
    if not path.is_file():
        return None
    return _load_snapshot_file(path, version=version)


def read_activation_record(layout: Layout, activation_id: str) -> Snapshot | None:
    """Load one immutable activation by id, or ``None`` if it is not on disk.

    The record, not the view. Rollback and recovery resolve through this,
    because "the activation this device was running" is not answerable by
    version once a version has been activated more than once.
    """

    path = layout.activation_record_file(activation_id)
    if not path.is_file():
        return None
    return _load_snapshot_file(path, activation_id=activation_id)


def resolve_activation(layout: Layout, activation_id: str, version: str) -> Snapshot | None:
    """Find one activation by id, preferring its record and accepting its view.

    The fallback is not a weakening. The view is loaded with *both* the version
    and the activation id as expectations, so it is adopted only when its own
    bytes hash to exactly the activation being asked for -- the same test the
    record has to pass. What it buys is a device installed by the previous build
    behaving identically before and after its records are materialised, and a
    dry run (which writes nothing, so materialises nothing) predicting what the
    real run will do instead of inventing a failure the real run would not have.
    """

    snapshot = read_activation_record(layout, activation_id)
    if snapshot is not None:
        return snapshot
    view = layout.activation_file(version)
    if not view.is_file():
        return None
    return _load_snapshot_file(view, version=version, activation_id=activation_id)


def current_activation_snapshot(layout: Layout, state: dict) -> Snapshot | None:
    """The activation this device is running, from its own record.

    ``None`` only when nothing is activated. Every other answer comes off disk
    and is digest checked, so what a transaction treats as "the outgoing unit
    set" is what actually ran rather than what today's registry would render.
    """

    activation_id = state.get("current_activation")
    if not activation_id:
        return None
    entry = state["history"][-1]
    snapshot = resolve_activation(layout, activation_id, entry["version"])
    if snapshot is None:
        raise LifecycleError(
            "activation_not_recorded",
            f"the recorded current activation {activation_id[:16]}... has no record on this "
            "device; its unit set cannot be reproduced",
        )

    _corroborate(state, snapshot, activation_id)
    return snapshot


def _corroborate(state: dict, snapshot: Snapshot, activation_id: str) -> None:
    """Make the state file and the activation record agree, or refuse both.

    The id proves the record is internally consistent and is the one that was
    asked for. It does not prove the *state* agrees with it: a state file edited
    to point at a different -- perfectly valid -- activation would sail through
    every check so far and hand a transaction the wrong outgoing unit set to
    retire and the wrong services to restart on recovery.
    """

    entry = state["history"][-1]
    for field_name, recorded, observed in (
        ("version", state.get("current"), snapshot.version),
        ("profile", state.get("profile"), snapshot.profile),
        ("interpreter", entry.get("python"), snapshot.python),
        ("release digest", entry.get("digest"), snapshot.release_digest),
        ("unit digests", entry.get("units"), unit_digests(snapshot.units)),
    ):
        if recorded != observed:
            raise LifecycleError(
                "activation_snapshot_invalid",
                f"the recorded state and activation {activation_id[:16]}... disagree on "
                f"{field_name}; refusing to act on either",
            )


def committed_activation(
    layout: Layout, state: dict, *, v1_compatible: bool = False
) -> Snapshot | None:
    """The committed activation, from its **immutable by-id record**.

    The difference from :func:`current_activation_snapshot` is the whole point
    of this function: no version-view fallback. The per-version file is mutable
    by design -- re-activating one version under a different profile repoints it
    -- so a reader that accepts it whenever a record happens to be absent lets
    "delete one file" quietly promote the view back to being the authority. That
    is the exact regression this boundary was rebuilt to remove, and it is
    invisible: every digest still checks out, because the view is a valid
    snapshot of a real activation. It is simply not *this* one on any device
    where the two have diverged.

    ``v1_compatible`` is the one narrow exception and it is not a weakening. A
    device whose state file is still raw v1 has never had by-id records written
    -- v1 did not have them -- so its identity is *derived* from the view at read
    time, and demanding a record would strand every machine the previous build
    installed. Even then the view is adopted only when its own bytes hash to the
    exact activation being asked for, so it can stand in for a record without
    ever standing in for a policy. The records are materialised under the lock by
    the next mutating operation, after which this path is never taken again.
    """

    activation_id = state.get("current_activation")
    if not activation_id:
        return None
    snapshot = read_activation_record(layout, activation_id)
    if snapshot is None:
        if not v1_compatible:
            raise LifecycleError(
                "activation_not_recorded",
                f"the committed activation {activation_id[:16]}... has no immutable record at "
                f"{layout.activation_record_dir}; the per-version view is not authority for it",
            )
        snapshot = resolve_activation(layout, activation_id, state["history"][-1]["version"])
        if snapshot is None:
            raise LifecycleError(
                "activation_not_recorded",
                f"the committed activation {activation_id[:16]}... has no record on this "
                "device; its unit set cannot be reproduced",
            )
    _corroborate(state, snapshot, activation_id)
    return snapshot


def _read_runtime_state(layout: Layout) -> tuple[dict, bool]:
    """The committed state, plus whether it arrived as a raw v1 document.

    ``_decode_state`` upgrades v1 in memory and stamps the current schema on what
    it returns, which is right for every caller that only wants the meaning --
    and loses the one fact a *reader* needs to decide whether a missing by-id
    record is a v1 device that never had one or a v2 device that lost one.

    Both facts come out of a **single read**. Reading the file once for the
    schema and again for the fields was a time-of-check/time-of-use bug with
    authority consequences: a state file replaced atomically between the two
    reads could be classified from the v1 document and validated from the v2 one,
    and the v1 classification is precisely what unlocks the mutable version view
    as a stand-in for the immutable record.
    """

    text = _load_state_text(layout)
    if text is None:
        return _empty_state(), False
    return _read_runtime_state_from(layout, text)


def _read_runtime_state_from(layout: Layout, text: str) -> tuple[dict, bool]:
    """The same two facts, from bytes a caller has already read exactly once."""

    state, declared = _decode_state(layout, text)
    return state, declared == LIFECYCLE_STATE_VERSION_V1


def _window_document(action: str, version: str, seconds: float) -> dict:
    return {
        "schema": LIFECYCLE_WINDOW_VERSION,
        "action": action,
        "version": version,
        "opened_at": time.time(),
        "window_seconds": float(seconds),
    }


def open_activation_window(
    layout: Layout, *, action: str, version: str, seconds: float = ACTIVATION_WINDOW_SECONDS
) -> Path:
    """Declare that an activation is live and has not committed yet.

    Written before systemd is touched, because the service this transaction is
    about to restart reads it on its very first cycle -- and durably, because the
    thing it has to survive is the crash that leaves no state file at all.

    Deliberately outside the :class:`_Fs` change journal. This marker is not part
    of the device's installed state: it exists only between two moments of one
    operation, and counting it as a change would make every idempotent re-install
    report ``ok`` where it correctly reports ``no_change``.
    """

    # Validated on the way in as well as on the way out. A caller that passes a
    # NaN, an infinity, or a non-positive duration would write a marker no reader
    # can bound, and the failure would surface as an unbounded pending device
    # rather than as the programming error it is.
    requested = _finite(seconds)
    if requested is None or requested <= 0.0:
        raise LifecycleError(
            "unexpected_error",
            "an activation window needs a finite, positive duration in seconds",
        )
    layout.state_dir.mkdir(parents=True, exist_ok=True)
    document = _window_document(action, version, min(requested, ACTIVATION_WINDOW_SECONDS))
    atomic_write(
        layout.activation_window_file,
        json.dumps(document, indent=2, sort_keys=True) + "\n",
        0o640,
    )
    return layout.activation_window_file


def close_activation_window(layout: Layout) -> None:
    """Retract the excuse, whether the transaction committed or undid itself.

    Idempotent and never fatal: a window that cannot be removed still expires,
    which is the property the bound exists to guarantee.
    """

    path = layout.activation_window_file
    try:
        path.unlink()
    except FileNotFoundError:
        return
    except OSError:  # pragma: no cover - a read-only /var is its own incident
        return
    fsync_path(path.parent, directory=True)


def activation_window(layout: Layout) -> dict | None:
    """The live pre-commit window, or ``None`` when there is not one.

    ``None`` covers both "no marker" and "a marker whose bound has passed": an
    expired window is not a window, and the caller must not be able to tell the
    difference, or "leave a stale file behind" would become a way to keep a
    device pending forever.

    A marker that is present and does not parse is refused rather than ignored.
    Ignoring it would fail *open* in the one direction that matters -- the file
    is on a writable path, and its whole job is to say when a missing state file
    is excusable.

    The bound is checked in both directions from the marker's own ``opened_at``.
    A wall clock that has jumped forward closes the window early, which is safe;
    one that has jumped backwards makes the elapsed time negative, which is
    refused rather than treated as "plenty of time left".

    Numbers must be *finite*. ``json.loads`` accepts ``NaN`` and ``Infinity``, and
    every comparison against ``NaN`` is false -- so ``min(NaN, ceiling)`` is
    ``NaN`` and ``elapsed >= bound`` never fires. A forged marker carrying one
    token would have re-created exactly the unbounded ``activation_pending`` this
    window exists to abolish, which is why the boundedness invariant is enforced
    on the values rather than assumed from the clamp.

    Nothing the marker contains is echoed. Every refusal below is a fixed string
    plus this device's own path: the file is attacker-writable on precisely the
    devices this path exists for, and the detail is published verbatim in the
    agent's JSON document and the journal, so interpolating a tampered field
    would let it choose bytes that leave the machine.
    """

    document = _read_window_document(layout)
    if document is None:
        return None
    opened_at, bound = document

    elapsed = time.time() - opened_at
    if elapsed < 0.0 or elapsed >= bound:
        return None
    return {
        "opened_at": opened_at,
        "window_seconds": bound,
        "remaining_seconds": bound - elapsed,
    }


def _finite(value: object) -> float | None:
    """``value`` as a float when it is a real, finite number; ``None`` otherwise.

    ``bool`` is excluded deliberately -- ``True`` is an ``int`` in Python, and a
    window that opened at "true o'clock" is a corrupted record, not a timestamp.
    """

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    number = float(value)
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return number


def _read_window_document(layout: Layout) -> tuple[float, float] | None:
    """The marker's ``(opened_at, bound)``, or ``None`` when there is no marker.

    Refuses anything present that does not verify, and never quotes it.
    """

    path = layout.activation_window_file
    try:
        raw = path.read_text(encoding="utf-8")
    except FileNotFoundError:
        return None
    except OSError as error:
        raise LifecycleError("config_unreadable", f"{path}: unreadable") from error

    try:
        document = json.loads(raw)
    except ValueError as error:
        raise _corrupt(path, _WINDOW_REFUSAL) from error
    if not isinstance(document, dict) or document.get("schema") != LIFECYCLE_WINDOW_VERSION:
        raise _corrupt(path, _WINDOW_REFUSAL)

    opened_at = _finite(document.get("opened_at"))
    requested = _finite(document.get("window_seconds"))
    if opened_at is None or requested is None or requested <= 0.0:
        raise _corrupt(path, _WINDOW_REFUSAL)

    # Clamped, not trusted. A marker is a file on a writable path, so the
    # duration it asks for can only ever shorten the ceiling this build enforces.
    return opened_at, min(requested, ACTIVATION_WINDOW_SECONDS)


def activation_window_evidence(layout: Layout) -> dict:
    """Clock-free, content-free facts about the window, for a support bundle.

    A bundle is byte-identical across two runs over the same device state, so it
    may not ask whether the window has *expired* -- that answer moves on its own.

    It carries no field the marker chose, either. ``action`` and ``version`` are
    strings from an attacker-writable file, and a bundle is a document whose one
    promise is that it ships nothing of the sort; a responder does not need them,
    because "there is a marker and it verifies" is the whole diagnostic. So the
    evidence is three booleans and a duration this build has already clamped.
    """

    try:
        document = _read_window_document(layout)
    except LifecycleError:
        # Present and refused. Named as such rather than raised: a bundle is
        # collected *because* the device is in a state like this one.
        return {"present": True, "verified": False, "window_seconds": None}
    if document is None:
        return {"present": False, "verified": False, "window_seconds": None}
    return {"present": True, "verified": True, "window_seconds": document[1]}


def _uncommitted_reason(layout: Layout) -> str:
    """Why there is no committed state, when no window excuses it.

    The same two codes :func:`status` already publishes for the same two
    situations, so an operator reading a service document and an operator reading
    a status report are told the same thing and sent to the same runbook entry.
    """

    if layout.current.is_symlink() or layout.current.exists():
        # Units and `current` were switched; the state write never landed. That
        # is the one inconsistency this design can produce, and re-running the
        # install for the release `current` names reconciles it.
        return "state_drift"
    return "not_installed"


def runtime_activation(layout: Layout, *, allow_pending: bool = True) -> Snapshot | None:
    """What a *running service* is entitled to believe it is running.

    The single authority for every process that starts after an activation --
    the supervise loop, the one-shot doctor, and :func:`status` -- and it is the
    same authority a transaction uses, so a device cannot describe itself one
    way to an operator and another way to its own installer.

    ``None`` means exactly one thing: **there is no committed lifecycle state and
    a live transaction says so**. Units are restarted and verified *before* the
    state write that commits them, so that interval is real -- but it is now
    *proved* rather than assumed, from the durable marker the transaction writes
    before it touches systemd and removes when it commits or finishes undoing.

    Without that marker, or once its bound has passed, a missing state file is
    not a fresh install still settling. It is a device interrupted between its
    activation and its state write, and it is refused with ``state_drift`` --
    ``not_installed`` when nothing is active at all -- rather than being reported
    as pending forever, which is a broken machine describing itself as a busy one.

    ``allow_pending=False`` refuses even inside a live window. A one-shot caller
    cannot wait a window out: it publishes one snapshot and exits, so "not
    committed yet" would be indistinguishable, tick after tick, from "not
    committed at all".

    Everything else is either the committed activation or a refusal:

    * a state file that exists but names no activation is not the pre-commit
      window -- nothing this product writes looks like that, and reading it as
      "not committed yet" would let a truncated or hand-edited state file
      silently demote a running device to a pending one, forever;
    * a state file that does not verify, or that disagrees with the activation
      record it names -- on version, profile, interpreter, release digest, or a
      single unit digest -- is refused by the readers below rather than
      half-believed;
    * a committed activation whose immutable by-id record cannot be produced is
      ``activation_not_recorded``. The per-version file is a compatibility view,
      accepted only when its own bytes hash to the exact activation being asked
      for, which is why it can stand in for a record without ever standing in
      for a *policy*.
    """

    # One read, and it decides both questions: whether there is a committed
    # document at all, and what it says. Asking `exists()` first and reading
    # afterwards is the same time-of-check/time-of-use shape that the schema
    # classification had.
    text = _load_state_text(layout)
    if text is None:
        window = activation_window(layout)
        if allow_pending and window is not None:
            return None
        reason = _uncommitted_reason(layout)
        raise LifecycleError(
            reason,
            f"{layout.state_file} does not exist and no live activation window accounts for it; "
            "this device is not in a pre-commit window, it has no committed activation",
        )
    state, from_v1 = _read_runtime_state_from(layout, text)
    snapshot = committed_activation(layout, state, v1_compatible=from_v1)
    if snapshot is None:
        raise LifecycleError(
            "activation_not_recorded",
            f"{layout.state_file} exists but names no current activation; a committed state "
            "that describes nothing cannot say which unit set this device is running",
        )
    return snapshot


def _write_activation_snapshot(fs: _Fs, layout: Layout, snapshot: Snapshot) -> str:
    """Record the activation immutably, then repoint the version's view at it.

    The record is content addressed, so "write once" and "write the same thing
    twice" are the same operation and re-activating an identical activation is
    free. A record that exists under an id and does not verify against it has
    been tampered with -- refused, never overwritten.

    The per-version view *is* rewritten, and has to be: it answers "what is this
    machine running now", which legitimately changes when the same version is
    re-activated under a different profile or interpreter. Nothing is lost by
    that write, because the activation it previously named is still on disk
    under its own id.
    """

    fs.mkdir(layout.activation_dir, 0o750)
    fs.mkdir(layout.activation_record_dir, 0o750)
    document = snapshot.document()
    encoded = json.dumps(document, indent=2, sort_keys=True) + "\n"

    activation_id = snapshot.activation_id
    record = layout.activation_record_file(activation_id)
    if record.is_file():
        # Loading it verifies the digest *and* that it is this activation; a
        # record that fails either check raises rather than being replaced.
        _load_snapshot_file(record, activation_id=activation_id)
    else:
        fs.write(record, encoded, 0o640)

    fs.write(layout.activation_file(snapshot.version), encoded, 0o640)
    return activation_id


def _prune_activation_snapshots(fs: _Fs, layout: Layout, state: dict) -> None:
    """Snapshots live exactly as long as the state that can reach them.

    Rollback can only target an activation still in ``history``, which is
    already bounded, so a record outside it -- and outside the live
    ``current_activation`` -- is unreachable by any command, while an unbounded
    pile of them in ``/var/lib`` is a disk-full incident waiting for a device
    that updates nightly.

    Reachability is computed from the committed state, never from "everything
    except the newest". Two activations of one version are both reachable and
    keeping only one of them would delete a rollback target that history still
    names.
    """

    if not layout.activation_dir.is_dir():
        return

    reachable_versions = {entry["version"] for entry in state["history"]}
    reachable_ids = {entry["activation_id"] for entry in state["history"]}
    current_activation = state.get("current_activation")
    if current_activation:
        reachable_ids.add(current_activation)

    for path in sorted(layout.activation_dir.glob("*.json")):
        if path.stem not in reachable_versions:
            fs.remove_file(path)
    if layout.activation_record_dir.is_dir():
        for path in sorted(layout.activation_record_dir.glob("*.json")):
            if path.stem not in reachable_ids:
                fs.remove_file(path)


def _clean_staging(fs: _Fs, layout: Layout) -> None:
    """Remove debris a previous interrupted run left behind.

    A staging directory is never a rollback target and never a release: it is
    the half-copied tree of a run that was killed. Leaving it would eventually
    fill a device's disk with the wreckage of every retried update.
    """

    if not layout.releases.is_dir():
        return
    for path in sorted(layout.releases.glob(".*.staging")):
        if path.is_dir():
            fs.remove_tree(path)


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------


def report(
    action: str,
    layout: Layout,
    *,
    ok: bool,
    reason: str,
    dry_run: bool,
    version: str | None = None,
    previous_version: str | None = None,
    profile: str = LIFECYCLE_PROFILES_DEFAULT,
    changed: Iterable[str] = (),
    defects: Iterable[UnitDefect] = (),
    detail: str = "",
    recovery: dict | None = None,
    systemd: dict | None = None,
    readiness: dict | None = None,
) -> dict:
    return {
        "readiness": readiness,
        "schema": LIFECYCLE_REPORT_VERSION,
        "action": action,
        "ok": ok,
        "dry_run": dry_run,
        "reason": reason,
        "action_code": action_for(reason),
        "detail": detail,
        "profile": profile,
        "version": version,
        "previous_version": previous_version,
        "paths": layout.as_dict(),
        "changed": sorted(changed),
        "defects": [defect.as_dict() for defect in defects],
        "recovery": recovery,
        "systemd": systemd,
    }


#: Internal alias. ``report`` is the public name, so a CLI can build a refusal
#: in the same shape as a success without reaching for a private function.
_report = report


# ---------------------------------------------------------------------------
# The activation transaction
# ---------------------------------------------------------------------------


@dataclass
class _Activation:
    """Undo record for the three things activation changes at once.

    Captured *before* anything moves. Rolling back means putting the unit files,
    the ``current`` symlink, and the recorded state back to exactly these
    values, then telling systemd about it -- in that order, because a service
    restarted before ``current`` is restored would come up on the bad release.
    """

    layout: Layout
    units_before: dict[str, str | None]
    current_before: Path | None
    state_before: dict
    state_file_existed: bool
    #: The version view this activation is about to repoint, and what it said
    #: beforehand (``None`` if it did not exist). It belongs in the undo record
    #: for the same reason the state file does: it is read by the running
    #: services and by `status` to answer "what contract is this machine
    #: under", so a transaction that puts the units and `current` back while
    #: leaving the view pointing at the activation it failed to make would
    #: leave the device describing itself as something it is not. That is not
    #: hypothetical for a same-version profile switch, where the view is the
    #: *only* file whose content changes between the two activations.
    view_path: Path | None = None
    view_before: str | None = None
    #: Which managed units systemd would start at boot, before any of this ran.
    #: Enablement is a fourth mutable surface, and it was the one the undo
    #: forgot. Activation both ``enable``s the incoming set and ``disable``s the
    #: outgoing one, so a failed profile switch that restored files, ``current``
    #: and state still left the previous release's units disabled: the machine
    #: looked healthy because recovery had restarted them by hand, and came up
    #: dead at the next reboot with nothing in the report to say why.
    enabled_before: frozenset[str] = frozenset()
    #: The *previous profile's* own restart/verify policy, intersected with the
    #: units that actually existed. Restoring by "everything that was on disk"
    #: is wrong twice over: it would restart a ``Type=oneshot`` unit whose whole
    #: job is to exit, and then demand ``is-active`` from it and declare the
    #: recovery failed when it correctly reported ``inactive``.
    previous_restart: tuple[str, ...] = ()
    previous_verify: tuple[str, ...] = ()
    previous_spec: Profile | None = None
    previous_python: str = "/usr/bin/python3"
    previous_profile: str | None = None
    #: False when the outgoing activation has no record behind it, so the undo
    #: policy above is not "nothing to restart" but "unknown".
    previous_policy_known: bool = True

    def created_units(self) -> tuple[str, ...]:
        """Units this operation introduced, so recovery removes only those."""

        return tuple(sorted(name for name, text in self.units_before.items() if text is None))

    def restore(self, fs: _Fs, systemd: SystemdController) -> dict:
        steps: list[str] = []
        try:
            # Quiesce *before* the unit files go away. systemd can only act on a
            # unit it still has a definition for: once the file is deleted and
            # the daemon reloaded, `disable` fails outright and a service this
            # operation started keeps running with nothing on disk to explain
            # it. Ordering this after the file removal made the fake pass (it
            # accepts any unit name) and the real machine orphan a running
            # service. Only units this operation *created* are touched --
            # disabling one that pre-existed would be damage, not recovery.
            #
            # This is not conditional on being a first install. An update that
            # adds a unit (a profile switch to one with an adapter) creates and
            # enables it exactly as a first install does, and its file is about
            # to be removed below; leaving it running and enabled would orphan
            # the incoming release's adapter on a device that has been put back
            # on the outgoing release.
            #
            # What it *is* conditional on is what systemd actually has. A
            # transaction can fail at the reload itself -- the very step that
            # tells systemd these files exist -- and then the unit is on disk,
            # unknown to the daemon, neither started nor enabled. Issuing
            # `disable` for it asks systemd about a unit it has no definition
            # for, which fails, which fails the undo, which reports
            # `rollback_failed` and abandons the files it was in the middle of
            # removing. A first install that hit a momentary reload race would
            # be escalated to support instead of simply cleaning up after
            # itself. So each verb is issued only for the units whose observed
            # state calls for it, and the stop-then-disable-then-delete order is
            # preserved for the ones that were genuinely started or enabled.
            created = self.created_units()
            if created:
                observed = {entry["unit"]: entry for entry in systemd.health(created)}
                to_stop = tuple(
                    name
                    for name in created
                    if observed.get(name, {}).get("active", "unknown")
                    not in _INERT_ACTIVE_STATES
                )
                to_disable = tuple(
                    name for name in created if observed.get(name, {}).get("enabled") == "enabled"
                )
                if to_stop:
                    systemd.stop(to_stop)
                if to_disable:
                    systemd.disable(to_disable)
                if to_stop or to_disable:
                    steps.append("stop-and-disable")

            for name, text in sorted(self.units_before.items()):
                path = self.layout.unit_dir / name
                if text is None:
                    fs.remove_file(path)
                    steps.append(f"remove-unit {name}")
                elif not path.is_file() or path.read_text(encoding="utf-8") != text:
                    _atomic_write(path, text, 0o644)
                    steps.append(f"restore-unit {name}")

            if self.current_before is None:
                fs.clear_current(self.layout.current)
                steps.append("clear-current")
            else:
                fs.point_current(self.layout.current, self.current_before)
                steps.append(f"restore-current {self.current_before.name}")

            # Before the state file, so that at no point does a reader see a
            # restored state pointing at a view that still describes the failed
            # activation. The by-id record this operation may have written is
            # deliberately *not* removed: it is content addressed and now
            # unreferenced, so the next successful operation's prune reclaims
            # it, whereas unlinking it here would repeat the exact mistake the
            # post-commit prune rule exists to prevent -- destroying a record
            # while an undo is still in flight.
            if self.view_path is not None:
                if self.view_before is None:
                    if self.view_path.is_file():
                        self.view_path.unlink()
                        _fsync_path(self.view_path.parent, directory=True)
                        steps.append(f"clear-activation-view {self.view_path.name}")
                elif (
                    not self.view_path.is_file()
                    or self.view_path.read_text(encoding="utf-8") != self.view_before
                ):
                    _atomic_write(self.view_path, self.view_before, 0o640)
                    steps.append(f"restore-activation-view {self.view_path.name}")

            if self.state_file_existed:
                _atomic_write(
                    self.layout.state_file,
                    json.dumps(self.state_before, indent=2, sort_keys=True) + "\n",
                    0o640,
                )
                steps.append("restore-state")
            elif self.layout.state_file.is_file():
                self.layout.state_file.unlink()
                steps.append("clear-state")

            systemd.daemon_reload()
            steps.append("daemon-reload")

            # Enablement is restored *after* the reload, because `enable` needs
            # a definition and the file it names may have just been written
            # back. Only units that are on disk once the undo has finished are
            # touched: a created unit has been removed by now and was already
            # stopped and disabled above, so it is correctly absent from both
            # lists rather than being re-enabled here.
            present = [
                name
                for name in sorted(self.units_before)
                if (self.layout.unit_dir / name).is_file()
            ]
            # Disable first, so that a unit which is meant to end up disabled is
            # never briefly enabled, and enable second, so the last word on any
            # unit is the state it actually had.
            to_disable = tuple(name for name in present if name not in self.enabled_before)
            to_enable = tuple(name for name in present if name in self.enabled_before)
            if to_disable:
                systemd.disable(to_disable)
            if to_enable:
                systemd.enable(to_enable)
            if to_disable or to_enable:
                steps.append("restore-enablement")

            # Nothing was ever healthy on a first install, so the device is left
            # quiet and obviously un-installed rather than half-running a
            # release we just refused: `flyto-robot install` is then the whole
            # recovery, and the quiescing already happened above.
            if self.current_before is not None:
                if not self.previous_policy_known:
                    # Files and `current` are back, but the *policy* that says
                    # which services the previous release wanted running is
                    # gone, so there is no such thing as a successful recovery
                    # here -- only an unverified one. Claiming `ok` would tell
                    # an operator the old release is serving traffic when
                    # nothing has been restarted or checked.
                    raise LifecycleError(
                        "rollback_failed",
                        f"the outgoing release (profile {self.previous_profile!r}) has no "
                        "recorded activation, so recovery cannot know which services to "
                        "restart or verify; files and `current` were restored but no "
                        "service was started",
                    )
                previous_restart = self.previous_restart
                previous_verify = self.previous_verify
                if self.previous_spec is not None:
                    fields = render_fields(self.layout, self.previous_python)
                    conditions = _activation_conditions(self.previous_spec, fields)
                    inactive = _inactive_conditional_units(self.previous_spec, conditions)
                    if inactive:
                        systemd.stop(inactive)
                    selected_restart = set(
                        _selected_unit_names(self.previous_spec, conditions, "restart")
                    )
                    selected_verify = set(
                        _selected_unit_names(self.previous_spec, conditions, "verify")
                    )
                    previous_restart = tuple(
                        name for name in previous_restart if name in selected_restart
                    )
                    previous_verify = tuple(
                        name for name in previous_verify if name in selected_verify
                    )
                systemd.restart(previous_restart)
                systemd.verify_active(previous_verify)
                steps.append(f"restart-previous:{self.previous_profile}")
        except (LifecycleError, SystemdError, OSError) as error:
            return {
                "attempted": True,
                "ok": False,
                "restored_version": None,
                "steps": steps,
                "error": f"{type(error).__name__}: {error}",
            }
        return {
            "attempted": True,
            "ok": True,
            "restored_version": (
                self.current_before.name if self.current_before is not None else None
            ),
            "steps": steps,
            "error": "",
        }


def _capture(
    layout: Layout,
    managed: Iterable[str],
    state: dict,
    *,
    previous: Snapshot | None,
    view_version: str,
    systemd: SystemdController | None = None,
) -> _Activation:
    """Snapshot everything the transaction may change, plus the undo *policy*.

    The policy comes from the outgoing activation's own immutable record, not
    from a registry lookup of its profile *name*. That distinction is the whole
    point. Resolving the name meant recovery could only restart what a registry
    still declared, so the case a rollback is most needed for -- a site drops
    the profile its fleet was installed under, then an update fails -- was
    precisely the case in which recovery gave up, restored the files, and
    reported that it could not know which services to start. The record carries
    the per-unit ``restart``/``verify`` policy that actually governed those
    units, so it answers without consulting anything a site can edit.

    It is still intersected with the units that were really on disk: restoring
    a unit that was not there is not recovery, and demanding ``is-active`` from
    a ``Type=oneshot`` unit that has correctly exited turns a healthy undo into
    a false ``rollback_failed``.
    """

    units_before: dict[str, str | None] = {}
    for name in sorted(set(managed)):
        path = layout.unit_dir / name
        units_before[name] = path.read_text(encoding="utf-8") if path.is_file() else None
    current_before = Path(os.readlink(layout.current)) if layout.current.is_symlink() else None

    view_path = layout.activation_file(view_version)
    view_before = view_path.read_text(encoding="utf-8") if view_path.is_file() else None

    # Read straight from systemd rather than inferring from the profile, for the
    # same reason the unit *text* is read from disk: what a device will do at
    # the next boot is a fact about that device, not about the policy something
    # once rendered. `health` is the read-only accessor status and the support
    # bundle already use, so this asks no new question of the boundary. Only
    # units whose files exist are asked about -- `is-enabled` on a name with no
    # definition answers about nothing.
    enabled_before: set[str] = set()
    if systemd is not None:
        on_disk = [name for name, text in units_before.items() if text is not None]
        enabled_before = {
            entry["unit"] for entry in systemd.health(on_disk) if entry["enabled"] == "enabled"
        }

    previous_profile = None
    previous_restart: tuple[str, ...] = ()
    previous_verify: tuple[str, ...] = ()
    previous_spec: Profile | None = None
    previous_python = "/usr/bin/python3"
    # A first install has no previous release, so there is no previous policy to
    # be missing; "known" is vacuously true and recovery quiesces instead.
    previous_policy_known = True
    if current_before is not None:
        if previous is None:
            # `current` exists but the state file names no activation behind it,
            # so there is no record of which services it wanted running.
            # Recovery cannot restart a set it cannot name.
            previous_profile = state.get("profile")
            previous_policy_known = False
        else:
            previous_profile = previous.profile
            previous_spec = previous.spec()
            previous_python = previous.python
            present = {name for name, text in units_before.items() if text is not None}
            previous_restart = tuple(
                name
                for name in sorted(previous.units)
                if previous.policy[name]["restart"] and name in present
            )
            previous_verify = tuple(
                name
                for name in sorted(previous.units)
                if previous.policy[name]["verify"] and name in present
            )

    return _Activation(
        layout=layout,
        units_before=units_before,
        current_before=current_before,
        state_before=copy.deepcopy(state),
        state_file_existed=layout.state_file.is_file(),
        view_path=view_path,
        view_before=view_before,
        enabled_before=frozenset(enabled_before),
        previous_restart=previous_restart,
        previous_verify=previous_verify,
        previous_spec=previous_spec,
        previous_python=previous_python,
        previous_profile=previous_profile,
        previous_policy_known=previous_policy_known,
    )


# ---------------------------------------------------------------------------
# Operations
# ---------------------------------------------------------------------------


def _ensure_persistent(fs: _Fs, layout: Layout) -> None:
    """Create the surfaces that outlive every release, exactly once.

    These are created before any release work so that a first install which
    later fails validation still leaves a device with somewhere to store its
    identity -- and so that the credential directory's mode is set by this
    function alone rather than by whichever release happened to run first.
    """

    fs.mkdir(layout.config_dir, 0o755)
    fs.mkdir(layout.state_dir, _STATE_DIR_MODE)
    fs.mkdir(layout.log_dir, 0o750)
    fs.mkdir(layout.credentials_dir, 0o700)
    fs.mkdir(layout.diagnostics_dir, 0o750)
    fs.mkdir(layout.provenance_dir, 0o750)
    fs.mkdir(layout.activation_dir, 0o750)
    fs.mkdir(layout.activation_record_dir, 0o750)
    fs.mkdir(layout.releases, 0o755)


def _seed_site_files(fs: _Fs, layout: Layout, *, profiles: Path | str | None) -> None:
    """Write the files a site owns -- once, and never again.

    A release that rewrote configuration would silently revert whatever the site
    had set on every update, which is how a fleet loses its cloud URL. The
    runbook is treated the same way: an operator who annotated it keeps their
    annotations.
    """

    if not layout.config_file.exists():
        fs.write(
            layout.config_file,
            "# Flyto robot configuration. Survives every release, update, and rollback.\n"
            "FLYTO_ROBOT_RESOURCE_ID=\n"
            "FLYTO_CLOUD_URL=\n",
            0o640,
        )
    if not layout.runbook_file.exists():
        # The agent unit's Documentation= points here. A unit that documents a
        # file nobody writes is worse than no Documentation= at all.
        fs.write(layout.runbook_file, runbook_text(profiles), 0o644)


def _activate_unit_set(
    fs: _Fs,
    layout: Layout,
    controller: SystemdController,
    *,
    spec: Profile,
    units: dict[str, str],
    activation: _Activation,
    target: Path,
    python: str,
    dry_run: bool,
) -> Readiness:
    """The transition every activation shares, in the one order that is safe.

    Install, update, and rollback are the same move: put exactly one profile's
    unit set on disk, retire whatever the outgoing profile owned and the
    incoming one does not, tell systemd, repoint ``current``, then prove the
    services are actually up.

    Rollback used to do only the middle of that -- repoint ``current``, reload,
    restart the target profile's unit *names*. So returning to a release that
    had been activated under a different profile restored the release and not
    the configuration that made it work: rolling back to a ROS 2 activation
    after switching to ``generic`` restarted a unit whose file had been deleted,
    and rolling the other way left the adapter enabled and restarting forever
    against a release it no longer matched. Sharing this function is what makes
    "a rollback is an activation" true rather than aspirational.

    The order is load-bearing. Units are written before ``daemon-reload`` so
    systemd sees them; outgoing units are stopped and disabled *before* their
    files are removed, because systemd cannot act on a definition that is gone;
    ``current`` moves only after systemd has been told, so a unit that restarts
    on its own cannot come up against a half-applied configuration.
    """

    fs.mkdir(layout.unit_dir, 0o755)
    for name in sorted(units):
        fs.write(layout.unit_dir / name, units[name], 0o644)

    # Units the *outgoing* profile owned and the incoming one does not. Leaving
    # these behind is how "switch back to generic" leaves the ROS 2 adapter
    # enabled, restarting forever against a release it no longer matches. They
    # are retired inside the same transaction, so a later failure puts them back
    # exactly as they were.
    outgoing = tuple(
        name
        for name, text in sorted(activation.units_before.items())
        if text is not None and name not in units
    )
    if outgoing:
        controller.stop(outgoing)
        controller.disable(outgoing)
        for name in outgoing:
            fs.remove_file(layout.unit_dir / name)

    controller.daemon_reload()
    fs.point_current(layout.current, target)
    fields = render_fields(layout, python)
    conditions = _activation_conditions(spec, fields)
    inactive = _inactive_conditional_units(spec, conditions)
    if inactive:
        controller.stop(inactive)
    controller.disable([unit.name for unit in spec.units if not unit.enable])
    controller.enable([unit.name for unit in spec.units if unit.enable])
    controller.restart(_selected_unit_names(spec, conditions, "restart"))
    controller.verify_active(_selected_unit_names(spec, conditions, "verify"))

    # `is-active` says the process has not exited. It does not say the release
    # can be used. Every activation -- install, update, and rollback alike, and
    # therefore every shipped CLI verb -- ends here, so a device can never be
    # handed back as `ok` on the strength of a process that started and does
    # nothing. A pairing that has not happened yet is reported, not punished; a
    # release that does not work on this device is undone by the caller.
    # Gated on the *operation* being a rehearsal, not on the controller being a
    # recorder: a library caller that passes no systemd controller still made a
    # real change to a real tree, and readiness is a filesystem question that
    # has nothing to do with whether systemctl was spoken to.
    if dry_run:
        return Readiness(state=READY, checks=())
    verdict = evaluate(spec, render_fields(layout, python), config_file=layout.config_file)
    conditional_checks = tuple(
        {
            "id": f"activation_condition:{unit.name}",
            "kind": unit.condition.kind,
            "target": unit.condition.path,
            "description": f"{unit.name} has its activation prerequisite",
            "provisioning": True,
            "passed": unit.name in conditions,
        }
        for unit in spec.units
        if unit.condition is not None
    )
    if conditional_checks:
        condition_state = (
            verdict.state
            if all(check["passed"] for check in conditional_checks) or verdict.state != READY
            else PROVISIONING_PENDING
        )
        verdict = Readiness(
            state=condition_state,
            checks=(*verdict.checks, *conditional_checks),
        )
    if not verdict.ok:
        raise LifecycleError(
            "post_switch_readiness_failed",
            f"{target.name} is running but not usable: "
            f"failed {', '.join(verdict.failures())}",
        )
    return verdict


def _unit_authority_collisions(
    layout: Layout, spec: Profile, previous: Snapshot | None
) -> tuple[str, ...]:
    """Existing incoming names not byte-identical to committed owned units.

    Ownership is per unit, not per device.  A generic activation owns its base
    units but says nothing about a same-name camera or ROS adapter that appeared
    later.  Treating the existence of *any* previous snapshot as authority to
    overwrite *every* incoming name would silently take over that foreign unit.
    """

    owned = previous.units if previous is not None else {}
    collisions = []
    for unit in spec.units:
        path = layout.unit_dir / unit.name
        if not path.exists() and not path.is_symlink():
            # A missing unit the snapshot owns can be restored.  Absence is not
            # evidence that some other owner has claimed its name.
            continue
        expected = owned.get(unit.name)
        try:
            matches = path.is_file() and not path.is_symlink() and path.read_text(
                encoding="utf-8"
            ) == expected
        except (OSError, UnicodeError):
            matches = False
        if expected is None or not matches:
            collisions.append(unit.name)
    return tuple(sorted(collisions))


def _recover_incomplete_takeover(layout: Layout, controller: SystemdController) -> None:
    """Resolve a sealed private takeover before stacking another mutation."""

    # Local import avoids making the B1 observation module part of lifecycle's
    # import graph (it already imports Layout from this module).
    from .legacy_takeover import TakeoverError, incomplete_takeover, recover_takeover

    try:
        pending = incomplete_takeover(layout)
    except TakeoverError as error:
        raise LifecycleError("takeover_recovery_required") from error
    if pending is None:
        return
    result = recover_takeover(layout, controller)
    if not result["ok"]:
        raise LifecycleError("takeover_rollback_failed")


def _recover_takeover_before_collision(
    layout: Layout, controller: SystemdController, *, dry_run: bool
) -> None:
    """Make crash recovery reachable without mutating ordinary collisions."""

    if dry_run:
        return
    from .legacy_takeover import takeover_window_path

    marker = takeover_window_path(layout)
    try:
        marker.lstat()
    except FileNotFoundError:
        return
    except OSError as error:
        raise LifecycleError("takeover_recovery_required") from error
    with _advisory_lock(layout):
        _recover_incomplete_takeover(layout, controller)


def install(
    *,
    payload: Path | None = None,
    version: str | None = None,
    layout: Layout,
    profile: str = LIFECYCLE_PROFILES_DEFAULT,
    python: str = "/usr/bin/python3",
    dry_run: bool = False,
    health_check: Callable[[Path], bool] | None = None,
    systemd: SystemdController | None = None,
    profiles: Path | str | None = None,
    manifest: Path | str | None = None,
    wheel_dir: Path | None = None,
    takeover_receipt: dict[str, Any] | None = None,
    _action: str = "install",
) -> dict:
    """Stage ``payload`` as ``version``, activate it, and prove it came up.

    Ordering is the whole design. Everything that can fail without touching the
    device happens first: version syntax, payload safety, immutability against
    recorded provenance, unit rendering and semantic validation. Only then does
    the transaction begin, and from that point either every step succeeds or all
    three mutable surfaces -- units, ``current``, recorded state -- are put back.

    ``systemd`` is injected. Passing ``None`` means "do not talk to systemd at
    all", which is what a library caller in a test wants; the CLI supplies a
    real controller on the real root and a recording one everywhere else. There
    is no code path that reports success after a failed ``systemctl``.
    """

    controller = systemd or SystemdController(
        runner=RecordingRunner(), dry_run=True, mode="none"
    )
    if dry_run:
        controller = SystemdController(
            runner=controller.runner,
            dry_run=True,
            binary=controller.binary,
            mode=controller.mode,
        )
    if takeover_receipt is not None and (dry_run or _action != "install"):
        raise LifecycleError("takeover_not_permitted")
    _recover_takeover_before_collision(layout, controller, dry_run=dry_run)

    manifest_source = manifest is not None or wheel_dir is not None
    if manifest_source:
        if payload is not None or manifest is None or wheel_dir is None:
            raise LifecycleError(
                "release_payload_invalid",
                "manifest and wheel_dir are one release source and are exclusive with payload",
            )
        lifecycle_profile = {"generic": "generic", "ros2": "ros2", "camera-host": "camera"}
        bootstrap_profile = profile
        if bootstrap_profile not in lifecycle_profile:
            raise LifecycleError("profiles_invalid", f"no bootstrap mapping for {profile!r}")

        # Refuse a legacy same-name unit before bootstrap publishes even one
        # release byte.  Ownership exists only when a committed activation
        # record names the unit; a filename alone is never takeover authority.
        state = _read_state(layout)
        previous_snapshot = current_activation_snapshot(layout, state)
        incoming_spec = profile_for(lifecycle_profile[bootstrap_profile], profiles=profiles)
        collisions = _unit_authority_collisions(layout, incoming_spec, previous_snapshot)
        if collisions and takeover_receipt is None:
            raise LifecycleError("unit_name_collision", ", ".join(collisions))

        def activate(releases: Path) -> dict:
            try:
                release = bootstrap_release(
                    manifest, wheel_dir, releases, python, bootstrap_profile
                )
            except BootstrapError as error:
                raise LifecycleError(error.code, error.code) from error
            release_python = release / (
                "venv/Scripts/python.exe" if os.name == "nt" else "venv/bin/python"
            )
            return install(
                payload=release,
                version=release.name,
                layout=layout,
                profile=lifecycle_profile[bootstrap_profile],
                python=str(release_python),
                dry_run=dry_run,
                health_check=health_check,
                systemd=systemd,
                profiles=profiles,
                takeover_receipt=takeover_receipt,
                _action=_action,
            )

        if dry_run:
            with tempfile.TemporaryDirectory(prefix="flyto-release-dry-run-") as scratch:
                return activate(Path(scratch))
        return activate(layout.releases)

    if payload is None or version is None:
        raise LifecycleError("release_payload_invalid", "payload and version are required")
    if not _VERSION.fullmatch(version):
        raise LifecycleError("release_payload_invalid", f"unsafe version {version!r}")
    payload = Path(payload)
    entries = _scan_payload(payload)

    spec = profile_for(profile, profiles=profiles)
    fs = _Fs(dry_run=dry_run)

    # A first activation must never adopt a same-name unit merely because it is
    # present.  Check before the advisory lock (which creates state_dir) to keep
    # this refusal genuinely zero-mutation, then check again under the lock to
    # close the race with another installer.
    state_before_lock = _read_state(layout)
    snapshot_before_lock = current_activation_snapshot(layout, state_before_lock)
    collisions = _unit_authority_collisions(layout, spec, snapshot_before_lock)
    if collisions and takeover_receipt is None:
        raise LifecycleError("unit_name_collision", ", ".join(collisions))

    with _advisory_lock(layout, enabled=not dry_run):
        if not dry_run:
            _recover_incomplete_takeover(layout, controller)
        state = _read_state(layout)
        previous = state.get("current")
        target = layout.release(version)

        snapshot_under_lock = current_activation_snapshot(layout, state)
        collisions = _unit_authority_collisions(layout, spec, snapshot_under_lock)
        if collisions and takeover_receipt is None:
            raise LifecycleError("unit_name_collision", ", ".join(collisions))

        if takeover_receipt is not None and state != _empty_state():
            raise LifecycleError("takeover_not_first_install")
        if takeover_receipt is not None:
            from .legacy_takeover import TakeoverError, revalidate_takeover_receipt

            try:
                takeover_check = revalidate_takeover_receipt(
                    receipt=takeover_receipt,
                    layout=layout,
                    profile=profile,
                    profiles=Path(profiles) if profiles is not None else None,
                    systemd=controller,
                )
            except TakeoverError as error:
                raise LifecycleError(error.reason) from error
            if not takeover_check["ok"]:
                raise LifecycleError(takeover_check["reason"])

        _ensure_persistent(fs, layout)
        _materialise_activation_records(fs, layout, state)
        _clean_staging(fs, layout)

        incoming_digest = release_digest(payload)

        # Immutability is enforced against provenance first, because provenance
        # outlives the release directory: deleting releases/1.0.0 to reclaim disk
        # must not re-open the name "1.0.0" for different bytes.
        recorded = _read_provenance(layout, version)
        if recorded is not None and recorded["digest"] != incoming_digest:
            raise LifecycleError(
                "release_exists_with_different_content",
                f"{version} was published with digest {recorded['digest'][:16]}...",
            )
        if target.exists():
            if release_digest(target) != incoming_digest:
                raise LifecycleError("release_exists_with_different_content", str(target))
        else:
            staging = layout.releases / f".{version}.staging"
            if not dry_run:
                if staging.exists():
                    fs.thaw(staging)
                    shutil.rmtree(staging)
                layout.releases.mkdir(parents=True, exist_ok=True)
            fs.copy_tree(payload, staging)
            if not dry_run:
                # A release becomes visible under its real name only once it is
                # completely copied. A crash before this leaves debris that the
                # next run removes; it never leaves a truncated "release".
                staging.replace(target)
                _fsync_path(layout.releases, directory=True)
            fs.freeze(target)

        _write_provenance(fs, layout, version, incoming_digest)

        units = render_units(layout, profile=profile, python=python, profiles=profiles)
        defects = _validate_units(units)
        if defects:
            # The staged release stays on disk for inspection; what does not
            # happen is the switch. A defective unit set never becomes running.
            return _report(
                _action,
                layout,
                ok=False,
                reason="unit_validation_failed",
                dry_run=dry_run,
                version=version,
                previous_version=previous,
                profile=profile,
                changed=fs.changed,
                defects=defects,
                detail=f"{len(defects)} defect(s); nothing was activated",
            )

        # ---- transaction ------------------------------------------------
        # What this operation may touch: the units it is about to install, and
        # the units the *outgoing activation* installed -- read from that
        # activation's own record.
        #
        # This used to union in "every unit any readable profile declares".
        # That made the outgoing set a property of the registry the caller
        # happened to pass rather than of the device, so switching a machine
        # from a site profile back to `generic` with the site's registry no
        # longer available left the site's units on disk, enabled, and
        # restarting forever against a release that no longer matched them --
        # with nothing in the report to say so. The record knows what was
        # installed because it is what installed it.
        previous_snapshot = current_activation_snapshot(layout, state)
        managed = set(units)
        if previous_snapshot is not None:
            managed |= set(previous_snapshot.units)
        activation = _capture(
            layout,
            managed,
            state,
            previous=previous_snapshot,
            view_version=version,
            systemd=controller,
        )
        takeover_digest = None
        if takeover_receipt is not None:
            from .legacy_takeover import TakeoverError, seal_takeover_window

            try:
                takeover_digest = seal_takeover_window(
                    receipt=takeover_receipt,
                    layout=layout,
                    profile=profile,
                    version=version,
                    profiles=Path(profiles) if profiles is not None else None,
                    systemd=controller,
                )
            except TakeoverError as error:
                raise LifecycleError(error.reason) from error

        # From here until the state write commits -- or the undo finishes -- this
        # device legitimately has no committed state to show the services this
        # transaction is about to restart. That interval is written down rather
        # than left to be guessed at from an absent file, and it is removed in the
        # `finally` below on every exit from the transaction, including the
        # failure that returns a report.
        verdict = Readiness(state=READY, checks=())
        try:
            if not dry_run:
                open_activation_window(layout, action=_action, version=version)
            _seed_site_files(fs, layout, profiles=profiles)
            verdict = _activate_unit_set(
                fs,
                layout,
                controller,
                spec=spec,
                units=units,
                activation=activation,
                target=target,
                python=python,
                dry_run=dry_run,
            )

            if health_check is not None and not dry_run and not health_check(layout.current):
                raise LifecycleError(
                    "post_switch_health_failed",
                    f"{version} failed its post-activation health check",
                )

            if not dry_run:
                try:
                    snapshot = build_activation_snapshot(
                        version=version,
                        profile=spec,
                        python=python,
                        release_digest=incoming_digest,
                        units=units,
                    )
                except SnapshotError as error:
                    raise LifecycleError("activation_snapshot_invalid", str(error)) from error
                # The record is written first and the state names it second, so
                # a crash between the two leaves an unreferenced record (which
                # the next prune reclaims) rather than a state file pointing at
                # an activation that was never written down.
                activation_id = _write_activation_snapshot(fs, layout, snapshot)
                _record_activation(
                    state, version, incoming_digest, profile, python, units, activation_id
                )
                if takeover_digest is not None:
                    from .legacy_takeover import (
                        TakeoverError,
                        publish_takeover_commit_intent,
                    )

                    try:
                        publish_takeover_commit_intent(
                            layout, takeover_digest, activation_id, state
                        )
                    except TakeoverError as error:
                        raise LifecycleError(error.reason) from error
                _write_state(fs, layout, state)
                if takeover_digest is not None:
                    from .legacy_takeover import TakeoverError, clear_takeover_window

                    try:
                        clear_takeover_window(layout, takeover_digest)
                    except TakeoverError as error:
                        raise LifecycleError(error.reason) from error
        except (LifecycleError, SystemdError, OSError) as error:
            reason = getattr(error, "reason", "install_failed")
            detail = getattr(error, "detail", "") or str(error)
            if dry_run:
                raise
            recovery = activation.restore(fs, controller)
            if takeover_digest is not None:
                from .legacy_takeover import recover_takeover

                takeover_recovery = recover_takeover(layout, controller)
                recovery = {
                    **recovery,
                    "ok": recovery["ok"] and takeover_recovery["ok"],
                    "takeover": takeover_recovery,
                }
            return _report(
                _action,
                layout,
                ok=False,
                reason="rollback_failed" if not recovery["ok"] else reason,
                dry_run=dry_run,
                version=version,
                previous_version=previous,
                profile=profile,
                changed=fs.changed,
                detail=detail,
                recovery=recovery,
                systemd={"mode": controller.mode, "commands": controller.issued()},
            )
        finally:
            if not dry_run:
                close_activation_window(layout)

        # Pruning happens only once the state write has committed, and only
        # here, where nothing further can fail the transaction. Deleting a
        # snapshot before the commit made the undo record a lie: recovery can
        # put the old state file back, but it cannot put back a snapshot it
        # already unlinked, so a state-write failure at the history bound left a
        # previously reachable rollback target permanently unreproducible.
        # Orphan removal is best effort by construction -- a snapshot outside
        # the committed history is unreachable, so failing to remove it costs
        # disk, while removing it too early costs a recovery path.
        if not dry_run:
            _prune_activation_snapshots(fs, layout, state)

    reason = "ok" if fs.changed else "no_change"
    if dry_run:
        reason = "dry_run"
    return _report(
        _action,
        layout,
        ok=True,
        reason=reason,
        dry_run=dry_run,
        version=version,
        previous_version=previous,
        profile=profile,
        changed=fs.changed,
        detail=f"{entries} payload entries, digest {incoming_digest[:16]}",
        systemd={"mode": controller.mode, "commands": controller.issued()},
        readiness=verdict.as_dict(),
    )


def _all_known_unit_names(profiles: Path | str | None = None) -> set[str]:
    """Every unit any profile could have installed.

    An update that switches a device from the ROS 2 profile to the generic one
    must be able to put back -- or take away -- the units the other profile
    owns. Capturing only the incoming profile's units would leave an orphan
    unit running against a release that no longer exists.
    """

    try:
        registry = load_profiles(profiles)
    except ProfileError:
        return set()
    return {unit.name for spec in registry.values() for unit in spec.units}


def update(**kwargs) -> dict:
    """Install a new version. Identical to :func:`install` by construction.

    Update is not a separate code path because a separate code path is how a
    first install and a fleet update drift apart, and the update is the one that
    runs unattended on a device someone depends on. The only difference is the
    word in the report and the fact that the undo record is non-empty.
    """

    kwargs.setdefault("_action", "update")
    return install(**kwargs)


def rollback(
    *,
    layout: Layout,
    to_version: str | None = None,
    dry_run: bool = False,
    systemd: SystemdController | None = None,
    profiles: Path | str | None = None,
) -> dict:
    """Return ``current`` -- and the unit set -- to a previously activated release.

    Rollback is an explicit operation with its own test, not a side effect of a
    failed update. A device can be rolled back long after a bad release was
    accepted, when the damage shows up in the field rather than at install time.

    It is a full activation, not a symlink swap. A release was activated under a
    particular *profile*, and the profile is what decides which units exist,
    which are enabled, and which have to be running. Returning the bytes without
    returning the unit set produces a device that is running the old release
    under the new release's configuration -- which is a state no operator asked
    for and no test of either release covers.
    """

    controller = systemd or SystemdController(runner=RecordingRunner(), dry_run=True, mode="none")
    if dry_run:
        controller = SystemdController(
            runner=controller.runner, dry_run=True, binary=controller.binary, mode=controller.mode
        )
    fs = _Fs(dry_run=dry_run)

    with _advisory_lock(layout, enabled=not dry_run):
        if not dry_run:
            _recover_incomplete_takeover(layout, controller)
        state = _read_state(layout)
        history = state.get("history", [])
        current_version = state.get("current")
        current_activation = state.get("current_activation")

        if not current_version:
            raise LifecycleError("not_installed", str(layout.current))

        # A device installed by the previous build reaches its first rollback
        # here, before anything is selected: the by-id records its history
        # references are derived from the snapshots already on disk, or the
        # operation refuses.
        _materialise_activation_records(fs, layout, state)

        # Everything before the activation that is live now. Selecting by
        # *version* was wrong in both directions: it skipped straight past a
        # same-version activation (so undoing a profile switch on 1.0.0 landed
        # on some earlier release instead of the 1.0.0 the operator had been
        # running an hour ago), and it could not express "the one before this
        # one" at all.
        prior = [entry for entry in history if entry.get("activation_id") != current_activation]

        if to_version is None:
            # No argument means "undo the last thing that happened", which is
            # the immediately preceding activation whatever version it wore.
            if not prior:
                raise LifecycleError("no_rollback_target", current_version)
            entry = prior[-1]
        else:
            # An explicit version means "the last time this device was running
            # that version", which is its most recent prior activation -- not
            # the oldest one that happens to carry the name.
            matching = [item for item in prior if item.get("version") == to_version]
            if not matching:
                raise LifecycleError(
                    "no_rollback_target",
                    f"{to_version} has no activation on this device before the current one",
                )
            entry = matching[-1]

        to_version = entry["version"]
        target_activation = entry["activation_id"]

        target = layout.release(to_version)
        if not target.is_dir():
            raise LifecycleError("release_missing", str(target))

        provenance = _read_provenance(layout, to_version)
        expected = entry.get("digest") or (provenance or {}).get("digest")
        if expected is not None and release_digest(target) != expected:
            # The rollback target is not the release that was activated under
            # that name. Returning to it would be a fresh, unverified change.
            raise LifecycleError(
                "release_payload_invalid", f"{target} no longer matches its recorded digest"
            )

        # Replay the activation; do not re-derive it. The snapshot carries the
        # rendered unit text, the per-unit policy, the readiness contract and
        # the interpreter as they were at activation time, so this path does not
        # consult a registry at all -- not the shipped one, not `--profiles`,
        # not the one a site has since edited or deleted. That is the difference
        # between "we can tell you this would no longer reproduce" and "here is
        # the device you had", which is the thing an operator actually needs at
        # 3am years after the release shipped.
        snapshot = resolve_activation(layout, target_activation, to_version)
        if snapshot is None:
            raise LifecycleError(
                "activation_not_recorded",
                f"{to_version} has no activation snapshot on this device, so the unit set "
                "it ran under cannot be reproduced; install that version explicitly",
            )
        if snapshot.version != to_version:  # pragma: no cover - the loader checks the id
            raise LifecycleError(
                "activation_snapshot_invalid",
                f"activation {target_activation[:16]}... records {snapshot.version}, "
                f"not {to_version}",
            )
        if snapshot.release_digest != release_digest(target):
            raise LifecycleError(
                "release_payload_invalid",
                f"{target} does not match the release recorded in its activation snapshot",
            )

        profile = snapshot.profile
        python = snapshot.python
        spec = snapshot.spec()
        units = dict(snapshot.units)

        # Validated again on the way out. A snapshot is immutable and digest
        # checked, but this build's understanding of what a safe unit is can
        # have grown since it was written, and installing a unit we now know
        # misbehaves would be a regression dressed up as a recovery.
        defects = _validate_units(units)
        if defects:
            return _report(
                "rollback",
                layout,
                ok=False,
                reason="unit_validation_failed",
                dry_run=dry_run,
                version=to_version,
                previous_version=current_version,
                profile=profile,
                changed=fs.changed,
                defects=defects,
                detail=f"{len(defects)} defect(s); nothing was activated",
            )

        # What this rollback may touch: the units it is restoring and the units
        # the *outgoing* activation installed. Both terms come from records, so
        # a cross-profile rollback works with no registry at all -- without the
        # second, an adapter unit that only the outgoing profile owned would be
        # left enabled and restarting against a release that no longer matches
        # it.
        outgoing_snapshot = current_activation_snapshot(layout, state)
        managed = set(units)
        if outgoing_snapshot is not None:
            managed |= set(outgoing_snapshot.units)
        activation = _capture(
            layout,
            managed,
            state,
            previous=outgoing_snapshot,
            view_version=to_version,
            systemd=controller,
        )

        # A rollback restarts the same services before its own state write, so it
        # passes through the same pre-commit interval an install does and has to
        # account for it the same way.
        if not dry_run:
            open_activation_window(layout, action="rollback", version=to_version)

        verdict = Readiness(state=READY, checks=())
        try:
            verdict = _activate_unit_set(
                fs,
                layout,
                controller,
                spec=spec,
                units=units,
                activation=activation,
                target=target,
                python=python,
                dry_run=dry_run,
            )
            if not dry_run:
                # Re-recording the replayed snapshot is a no-op for the record
                # (same content, same id) and repoints the version's view at the
                # activation that is now live, so a running service reads the
                # contract it was actually started under.
                activation_id = _write_activation_snapshot(fs, layout, snapshot)
                _record_activation(
                    state, to_version, release_digest(target), profile, python, units, activation_id
                )
                _write_state(fs, layout, state)
        except (LifecycleError, SystemdError, OSError) as error:
            reason = getattr(error, "reason", "rollback_failed")
            detail = getattr(error, "detail", "") or str(error)
            if dry_run:
                raise
            recovery = activation.restore(fs, controller)
            return _report(
                "rollback",
                layout,
                ok=False,
                reason="rollback_failed" if not recovery["ok"] else reason,
                dry_run=dry_run,
                version=to_version,
                previous_version=current_version,
                profile=profile,
                changed=fs.changed,
                detail=detail,
                recovery=recovery,
                systemd={"mode": controller.mode, "commands": controller.issued()},
            )
        finally:
            if not dry_run:
                close_activation_window(layout)

        # After the commit, for the same reason as install: a snapshot deleted
        # before the state write is a rollback target the undo cannot give back.
        if not dry_run:
            _prune_activation_snapshots(fs, layout, state)

    reason = "dry_run" if dry_run else ("ok" if fs.changed else "no_change")
    return _report(
        "rollback",
        layout,
        ok=True,
        reason=reason,
        dry_run=dry_run,
        version=to_version,
        previous_version=current_version,
        profile=profile,
        changed=fs.changed,
        systemd={"mode": controller.mode, "commands": controller.issued()},
        readiness=verdict.as_dict(),
    )


def status(
    layout: Layout,
    *,
    systemd: SystemdController | None = None,
    profiles: Path | str | None = None,
) -> dict:
    """Report what is installed without changing anything."""

    # Fail closed on a registry that cannot be read. Status enumerates the units
    # a device is supposed to have out of the profile registry, so a malformed
    # one means the unit list is not "empty", it is *unknown* -- and a device
    # that has never been installed is indistinguishable from one whose registry
    # was corrupted after the fact. Swallowing the error turns that into a
    # confident `not_installed`, which sends an operator to `run_install` on a
    # machine that is already running a release.
    try:
        load_profiles(profiles)
    except ProfileError as error:
        raise LifecycleError("profiles_invalid", str(error)) from error

    state, from_v1 = _read_runtime_state(layout)
    installed: list[str] = []
    if layout.releases.is_dir():
        installed = sorted(p.name for p in layout.releases.glob("*") if p.is_dir())
    active: str | None = None
    if layout.current.is_symlink():
        active = Path(os.readlink(layout.current)).name

    recorded_current = state.get("current")
    # A crash between the atomic activation and the state write leaves these
    # disagreeing. Saying "ok" here would hide the one inconsistency this design
    # can actually produce, so it is named and given an action instead.
    drift = active is not None and recorded_current is not None and active != recorded_current

    reason = "ok"
    if active is None and recorded_current is None:
        reason = "not_installed"
    elif drift or (active is None) != (recorded_current is None):
        reason = "state_drift"
    elif not layout.identity_file.exists():
        reason = "identity_missing"

    profile = state.get("profile") or LIFECYCLE_PROFILES_DEFAULT
    report = _report(
        "status",
        layout,
        ok=reason == "ok",
        reason=reason,
        dry_run=False,
        version=active,
        previous_version=None,
        profile=profile,
        detail="" if not drift else f"current={active} but state records {recorded_current}",
    )
    report["installed_releases"] = installed
    report["history"] = state.get("history", [])
    report["recorded_current"] = recorded_current
    report["persistent_paths"] = [str(p) for p in layout.persistent_paths()]
    report["identity_present"] = layout.identity_file.exists()
    report["config_present"] = layout.config_file.exists()
    report["runbook_present"] = layout.runbook_file.exists()

    # What this device is *supposed* to be running comes from the activation it
    # actually performed, not from whichever registry the caller happens to hold.
    # Enumerating from the registry meant a machine installed against a site file
    # that has since been deleted reported `ok` while omitting every unit it
    # really runs -- and the units it does run are exactly the ones a responder
    # is asking about.
    #
    # Resolved by activation id, not by version. The per-version file is the
    # newest activation of that name, which is a different thing from the one
    # that is committed the moment a rollback returns to an earlier activation of
    # the same version -- and enumerating the newer one would list units this
    # device does not run and name a profile it is not under. Corruption fails
    # closed for the same reason: a status nobody can trust has to refuse rather
    # than answer confidently out of the registry.
    snapshot = (
        committed_activation(layout, state, v1_compatible=from_v1) if recorded_current else None
    )
    if snapshot is not None:
        known = sorted(snapshot.units)
        report["active_profile"] = snapshot.profile
    else:
        known = sorted(
            name for name in _all_known_unit_names(profiles) if (layout.unit_dir / name).is_file()
        )
        report["active_profile"] = None
    report["installed_units"] = known
    report["units_present"] = sorted(
        name for name in known if (layout.unit_dir / name).is_file()
    )
    report["unit_health"] = systemd.health(known) if systemd is not None else []
    return report


# ---------------------------------------------------------------------------
# CLI (the customer entry point lives in flyto_robotics.robot_cli)
# ---------------------------------------------------------------------------


def main(argv: Sequence[str] | None = None, *, stream=None) -> int:
    """Kept so ``python -m flyto_robotics.lifecycle`` keeps working.

    One parser, one set of exit codes: this delegates rather than defining a
    second CLI that would drift from the shipped one.
    """

    from .robot_cli import main as _main

    return _main(argv, stream=stream)


if __name__ == "__main__":
    raise SystemExit(main())
