"""The systemd boundary: one injectable place where ``systemctl`` is run.

Nothing else in this package may shell out to ``systemctl``. That is the whole
point of the module. A lifecycle that calls ``subprocess.run(["systemctl", ...])``
inline can only be tested on a machine with systemd and root, which in practice
means it is tested by customers.

Three runners implement the same one-method contract:

* :func:`subprocess_runner` -- the real thing. Used only when the lifecycle is
  operating on the real root.
* :class:`RecordingRunner` -- records the commands a run *would* issue and
  reports success. This is what a rehearsal against a temporary root uses, so
  ``--root /tmp/...`` can never touch the host's units.
* :class:`FakeSystemctl` -- an in-memory systemd with enough state to answer
  ``is-active``/``is-enabled``, plus deliberate fault injection. Failure paths
  (``systemctl`` non-zero, a unit that starts and then dies) are the paths that
  matter, and they are unreachable without it.

Failures are raised as :class:`SystemdError` carrying a lifecycle reason code,
never as a boolean the caller can forget to check. "Never report success when
systemctl failed" is enforced by there being no success value to return.
"""

from __future__ import annotations

import shutil
import subprocess
from collections.abc import Callable, Iterable, Mapping, Sequence
from dataclasses import dataclass, field

__all__ = [
    "CommandResult",
    "FakeSystemctl",
    "RecordingRunner",
    "SystemdController",
    "SystemdError",
    "subprocess_runner",
]


@dataclass(frozen=True)
class CommandResult:
    argv: tuple[str, ...]
    returncode: int
    stdout: str = ""
    stderr: str = ""

    @property
    def ok(self) -> bool:
        return self.returncode == 0


CommandRunner = Callable[[Sequence[str]], CommandResult]


class SystemdError(RuntimeError):
    """A systemd step failed. Carries a lifecycle reason code."""

    def __init__(self, reason: str, detail: str = "") -> None:
        super().__init__(detail or reason)
        self.reason = reason
        self.detail = detail


def subprocess_runner(argv: Sequence[str]) -> CommandResult:
    """Run a real ``systemctl``. Reached only for operations on the real root."""

    binary = shutil.which(argv[0])
    if binary is None:
        return CommandResult(tuple(argv), 127, "", f"{argv[0]} not found on PATH")
    completed = subprocess.run(  # noqa: S603 - argv is built from validated unit names
        [binary, *argv[1:]],
        capture_output=True,
        text=True,
        timeout=120,
        check=False,
    )
    return CommandResult(tuple(argv), completed.returncode, completed.stdout, completed.stderr)


@dataclass
class RecordingRunner:
    """Records what would be run and reports success. Mutates nothing."""

    commands: list[tuple[str, ...]] = field(default_factory=list)
    #: ``is-active`` answers "active" so a rehearsal exercises the success path.
    active_reply: str = "active"

    def __call__(self, argv: Sequence[str]) -> CommandResult:
        self.commands.append(tuple(argv))
        if len(argv) > 1 and argv[1] in {"is-active", "is-enabled"}:
            return CommandResult(tuple(argv), 0, self.active_reply)
        return CommandResult(tuple(argv), 0)


@dataclass
class FakeSystemctl:
    """An in-memory systemd with fault injection, for tests.

    ``fail_on`` matches on the verb (``"restart"``) or on ``"verb unit"``
    (``"restart flyto-robot-agent.service"``), so a test can fail exactly the
    step it means to and prove the rollback that follows. It is *persistent*:
    the step fails every time it is attempted, which is how a genuinely broken
    machine behaves and is what proves the escalation to ``rollback_failed``.

    ``fail_once`` and ``fail_times`` are the bounded counterparts, and they are
    what makes an *automatic recovery* provable at all. Recovery re-issues the
    same verbs the failed operation issued -- a rollback restarts the previous
    release's units, an undone first install reloads the daemon again -- so a
    permanently failing verb necessarily fails the undo too, and every such test
    can only ever observe ``rollback_failed``. A transient fault ("the reload
    lost a race the first time, the retry worked") is the real shape of most
    field failures and the only shape under which "the operation failed *and*
    the device came back on its own" can be asserted. ``fail_once`` fails a key
    exactly once; ``fail_times`` fails it a given number of times; both then
    behave normally.

    ``dies_after_start`` names units that report ``failed`` once started, which
    is how a release that starts and immediately crashes actually looks: the
    ``restart`` succeeds and ``is-active`` is the only thing that notices.
    """

    fail_on: frozenset[str] = frozenset()
    fail_once: frozenset[str] = frozenset()
    fail_times: Mapping[str, int] = field(default_factory=dict)
    dies_after_start: frozenset[str] = frozenset()
    #: Where unit *definitions* come from, refreshed on every ``daemon-reload``
    #: exactly as systemd refreshes them. Supply
    #: ``lambda: {p.name for p in unit_dir.glob("*")}`` and the fake starts
    #: refusing verbs that need a definition it does not have. Left as ``None``
    #: the fake accepts any unit name -- which is convenient and is precisely
    #: what hid an ordering bug: recovery deleted the unit files, reloaded, and
    #: only then tried to ``disable`` them. Real systemd fails that and leaves a
    #: started service orphaned; the fake happily returned 0.
    unit_source: Callable[[], set[str]] | None = None
    #: What the fake currently has definitions for. Only consulted when
    #: ``unit_source`` is set.
    defined: set[str] = field(default_factory=set)
    commands: list[tuple[str, ...]] = field(default_factory=list)
    enabled: set[str] = field(default_factory=set)
    active: set[str] = field(default_factory=set)
    reloads: int = 0
    #: Remaining bounded failures per key. Derived, never passed in, so the
    #: declared fault injection stays readable at the call site.
    _budget: dict[str, int] = field(default_factory=dict, init=False, repr=False)

    #: Verbs systemd cannot carry out without a unit definition on disk.
    NEEDS_DEFINITION = ("enable", "disable", "start", "restart")

    def __post_init__(self) -> None:
        budget: dict[str, int] = dict.fromkeys(self.fail_once, 1)
        for key, count in dict(self.fail_times).items():
            budget[key] = int(count)
        self._budget = {key: count for key, count in budget.items() if count > 0}
        if self.unit_source is not None:
            self.defined = set(self.unit_source())

    @staticmethod
    def _keys(verb: str, unit: str) -> tuple[str, ...]:
        return (f"{verb} {unit}", verb) if unit else (verb,)

    def _fails(self, verb: str, unit: str = "") -> bool:
        keys = self._keys(verb, unit)
        if any(key in self.fail_on for key in keys):
            return True
        # Consume at most one unit of budget per command, and only when the
        # command is actually issued: a bounded fault that expired on a step
        # nobody ran would inject the failure somewhere the test never named.
        for key in keys:
            remaining = self._budget.get(key, 0)
            if remaining > 0:
                self._budget[key] = remaining - 1
                return True
        return False

    def __call__(self, argv: Sequence[str]) -> CommandResult:
        argv = tuple(argv)
        self.commands.append(argv)
        verb = argv[1] if len(argv) > 1 else ""
        units = [item for item in argv[2:] if not item.startswith("-")]
        unit = units[0] if units else ""

        if self._fails(verb, unit):
            return CommandResult(argv, 1, "", f"fake systemctl: refusing {verb} {unit}".strip())

        if self.unit_source is not None and verb in self.NEEDS_DEFINITION:
            missing = [name for name in units if name not in self.defined]
            if missing:
                return CommandResult(
                    argv, 1, "", f"Unit file {missing[0]} does not exist."
                )

        if verb == "daemon-reload":
            self.reloads += 1
            if self.unit_source is not None:
                self.defined = set(self.unit_source())
        elif verb == "enable":
            self.enabled.update(units)
        elif verb == "disable":
            self.enabled.difference_update(units)
            self.active.difference_update(units)
        elif verb in {"start", "restart"}:
            for name in units:
                if name in self.dies_after_start:
                    self.active.discard(name)
                else:
                    self.active.add(name)
        elif verb == "stop":
            self.active.difference_update(units)
        elif verb == "is-active":
            state = "active" if unit in self.active else "inactive"
            return CommandResult(argv, 0 if state == "active" else 3, state)
        elif verb == "is-enabled":
            state = "enabled" if unit in self.enabled else "disabled"
            return CommandResult(argv, 0 if state == "enabled" else 1, state)
        return CommandResult(argv, 0)


@dataclass
class SystemdController:
    """Every systemd interaction the lifecycle is allowed to make.

    ``dry_run`` short-circuits every mutating verb, so ``--dry-run`` is honest
    about systemd too rather than only about the filesystem.
    """

    runner: CommandRunner
    dry_run: bool = False
    binary: str = "systemctl"
    mode: str = "recording"

    def _run(self, *args: str) -> CommandResult:
        return self.runner((self.binary, *args))

    def _require(self, *args: str) -> CommandResult:
        result = self._run(*args)
        if not result.ok:
            raise SystemdError(
                "systemctl_failed",
                f"`{' '.join(result.argv)}` exited {result.returncode}: "
                f"{(result.stderr or result.stdout).strip()[:200]}",
            )
        return result

    def daemon_reload(self) -> None:
        if self.dry_run:
            return
        self._require("daemon-reload")

    def enable(self, units: Iterable[str]) -> None:
        units = [unit for unit in units]
        if self.dry_run or not units:
            return
        self._require("enable", *units)

    def disable(self, units: Iterable[str]) -> None:
        units = [unit for unit in units]
        if self.dry_run or not units:
            return
        self._require("disable", *units)

    def restart(self, units: Iterable[str]) -> None:
        if self.dry_run:
            return
        for unit in units:
            self._require("restart", unit)

    def stop(self, units: Iterable[str]) -> None:
        """Stop each unit, and refuse to continue when systemd said no.

        A ``stop`` whose exit status is discarded is the one systemd verb that
        can silently leave the device running code the operation has already
        decided to retire: the caller deletes the unit file, reports success,
        and the old service keeps running until the next reboot -- against a
        release that no longer exists. Retiring an outgoing profile's units is
        part of the activation transaction, so a refused ``stop`` has to raise
        like every other step and let the transaction be undone.
        """

        if self.dry_run:
            return
        for unit in units:
            self._require("stop", unit)

    def is_active(self, unit: str) -> str:
        return (self._run("is-active", unit).stdout or "").strip() or "unknown"

    def verify_active(self, units: Iterable[str]) -> None:
        """Raise unless every named unit is actually running.

        A ``restart`` that exits 0 only means systemd accepted the job. A unit
        whose ``ExecStart`` dies immediately still gives a clean ``restart``, so
        this is the step that decides whether an update is allowed to stand.
        """

        if self.dry_run:
            return
        for unit in units:
            state = self.is_active(unit)
            if state != "active":
                raise SystemdError("service_not_active", f"{unit} is {state}, not active")

    def health(self, units: Iterable[str]) -> list[dict[str, str]]:
        """Read-only per-unit state for status and support bundles."""

        report: list[dict[str, str]] = []
        for unit in sorted(units):
            report.append(
                {
                    "unit": unit,
                    "active": self.is_active(unit),
                    "enabled": (self._run("is-enabled", unit).stdout or "").strip() or "unknown",
                }
            )
        return report

    def issued(self) -> list[str]:
        commands = getattr(self.runner, "commands", [])
        return [" ".join(argv) for argv in commands]
