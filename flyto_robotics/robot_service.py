"""The processes the installed units actually run.

The units used to point at two developer entry points that a real device cannot
satisfy. ``flyto_robotics.resource_agent`` requires ``--cloud-url``,
``--device-id``, ``--device-secret-file`` and ``--manifest``; the unit supplied
none of them, so on a real machine ``ExecStart=`` exited 2 immediately.
``flyto_robotics.robot_doctor`` requires ``--resource-id`` and has never had a
``--state-dir``; the unit passed the flag that does not exist and omitted the
one that does. Both are argparse usage errors, which means the shipped
``Restart=on-failure`` turned the whole install into a rate-limited restart loop
that ``systemctl is-active`` reported as ``activating`` and the fake systemd
reported as running.

This module is the contract those units should have had:

* **Stable unpaired.** A device that has been installed but not yet commissioned
  has no identity, no cloud URL and no credential. That is normal, so the agent
  stays up and reports ``provisioning_pending`` rather than exiting. A service
  that dies until a human finishes provisioning cannot be the service that tells
  the human what is missing.
* **Stable paired.** Once the site files are in place the same process keeps
  running and keeps writing the same status document.
* **No checkout.** Every input is a persistent path passed on the command line:
  the configuration, the identity, the credential directory, the active release.
  Nothing is resolved relative to a source tree.
* **Structured, not chatty.** Each cycle writes one JSON document to the state
  directory and one line to stdout for the journal. That document is what a
  support bundle and an operator read; there is no free text to parse.

Nothing here imports a middleware or names a transport. Which facts make a
device ready is declared in the profile registry and evaluated by
:mod:`flyto_robotics.readiness`.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from collections.abc import Sequence
from pathlib import Path

from .activation_snapshot import SnapshotError
from .health_codes import action_for
from .lifecycle import Layout, LifecycleError, runtime_activation
from .lifecycle_profiles import Profile, ProfileError
from .readiness import READINESS_VERSION, UNHEALTHY, Readiness, evaluate, read_config

__all__ = [
    "ACTIVATION_PENDING",
    "AGENT_STATUS_FILE",
    "BOOTSTRAP_POLL_SECONDS",
    "BOOTSTRAP_WINDOW_SECONDS",
    "DOCTOR_STATUS_FILE",
    "cycle",
    "main",
]

AGENT_STATUS_FILE = "agent-status.json"
DOCTOR_STATUS_FILE = "doctor-status.json"

#: The activation that started this service has not committed its state yet, and
#: a live transaction window on disk says so. Distinct from
#: ``provisioning_pending`` (installed, awaiting a human) and from ``unhealthy``
#: (installed and broken): this one resolves by itself, within seconds, and must
#: never be reported as a failure.
#:
#: It is never inferred from a missing file. The lifecycle grants it only while
#: the durable, bounded window an activation opens before it touches systemd is
#: still live, so a device that lost its state file -- or one whose commit never
#: arrived -- moves to ``unhealthy`` at a knowable moment instead of describing
#: itself as busy forever.
ACTIVATION_PENDING = "activation_pending"

#: Bound the supervise loop from the environment so a test can run the unit's
#: *exact* ExecStart -- the whole point is to prove that command shape works, and
#: appending a flag to it would prove something else.
MAX_CYCLES_ENV = "FLYTO_ROBOT_MAX_CYCLES"

#: How often this service looks again while it is still catching up with the
#: activation that started it. The supervision interval is a *steady state*
#: number -- thirty seconds is the right cadence for noticing that a device
#: stopped being ready, and it is the wrong cadence for the first read after a
#: restart, because the activation commits its state milliseconds later. Sleeping
#: the full interval there left every fresh install and every profile switch
#: publishing ``activation_pending``, and the stale document a support bundle
#: collected in that half minute described a device that no longer existed.
BOOTSTRAP_POLL_SECONDS = 0.25

#: How long the fast look-again lasts. Bounded on purpose, and bounded by one
#: supervision interval so a restart can never cost more than the cadence it is
#: catching up with: the transaction that restarted this service commits within
#: milliseconds, so this is orders of magnitude of headroom, and a device where
#: it never commits settles back to the ordinary interval rather than spinning.
BOOTSTRAP_WINDOW_SECONDS = 15.0


def _paths(args: argparse.Namespace) -> dict[str, str]:
    return {
        "current": str(args.release),
        "config_dir": str(args.config_dir),
        "config_file": str(args.config_dir / "robot.env"),
        "identity_file": str(args.config_dir / "identity.json"),
        "state_dir": str(args.state_dir),
        "log_dir": str(args.log_dir),
        "python": sys.executable,
    }


def _committed_activation(args: argparse.Namespace, *, allow_pending: bool):
    """The activation this device has committed to, or ``None`` before it has.

    Resolved through the lifecycle's own reader, so a running service and the
    installer that started it cannot disagree about what is installed. The
    authority is ``state.current_activation`` and the immutable by-id record it
    names; the per-version file is a compatibility view that is accepted only
    when its own bytes hash to that exact activation, and never as a source of
    policy in its own right.

    Resolving by *version* was the defect. A device that has rolled back to an
    earlier activation of a version it has installed twice still has the newer
    activation sitting in the version view, so the service evaluated a readiness
    contract -- and reported a profile -- that belongs to an activation this
    device deliberately left. Resolving generically was the same defect with a
    worse blast radius: a paired site machine would report ``ready`` while
    nothing its own profile requires had been checked at all.
    """

    return runtime_activation(
        Layout.for_state_dir(args.state_dir), allow_pending=allow_pending
    )


def _document(
    args: argparse.Namespace,
    fields: dict[str, str],
    verdict: Readiness,
    *,
    profile: str,
    activation: str,
) -> dict:
    """One shape, whatever the verdict.

    An unhealthy device emits the same keys as a ready one so that whatever
    parses this -- a support bundle, a console, a responder's `jq` -- does not
    have to carry a second parser for the case it most needs to read.
    """

    config = read_config(Path(fields["config_file"]))
    return {
        "schema": READINESS_VERSION,
        "service": args.command,
        "profile": profile,
        # Which activation the answer above was computed from. A profile name is
        # not enough to identify one: the same version under the same profile can
        # be activated twice with different interpreters, and the id is the only
        # thing that distinguishes them. It is a digest over rendered unit text,
        # so it names content and discloses nothing.
        "activation": activation,
        "state": verdict.state,
        "ok": verdict.ok,
        "checks": [dict(check) for check in verdict.checks],
        # Identifiers, never secrets: the resource id is how a responder finds
        # the device in the console, and the credential is never read at all.
        "resource_id": config.get("FLYTO_ROBOT_RESOURCE_ID", ""),
        "cloud_url_configured": bool(config.get("FLYTO_CLOUD_URL", "").strip()),
        "identity_present": Path(fields["identity_file"]).exists(),
        "release": str(Path(fields["current"]).resolve()) if args.release.exists() else "",
    }


def cycle(args: argparse.Namespace, *, pending_ok: bool = True) -> tuple[Readiness, dict]:
    """Evaluate readiness once and build the status document.

    Raises nothing a caller has to catch. The two outcomes that are not "ready"
    are deliberately different states rather than different exception types:

    ``activation_pending``
        There is no committed lifecycle state *and* the lifecycle's durable,
        bounded transaction window says one is being written right now. An
        activation restarts this service and *then* commits, so the first start
        of a clean install runs in that window. It resolves by itself, ``ok`` is
        true, and treating it as a failure is what made the shipped ExecStart
        exit 1 on every real device and turned ``Restart=on-failure`` into a flap.

    ``unhealthy``
        Everything else that is not readiness: a committed state that cannot be
        believed -- missing, altered, disagreeing with the activation record it
        names, or naming a record this device cannot produce -- and now also a
        *missing* state that no live window accounts for. That last one used to
        be reported as ``activation_pending`` forever, which is a device that has
        lost its lifecycle state describing itself as one that is busy acquiring
        it. All of them are one structured document with a stable reason and
        action, never a quiet fall back to a profile the device was never
        installed under.

    ``pending_ok=False`` withdraws the pending answer entirely. A one-shot caller
    exits after this single document, so it cannot wait a window out; reporting
    "still committing" from a command that will not look again is how a device
    with no committed state gets treated as a fresh install on every tick of a
    timer.
    """

    fields = _paths(args)
    try:
        snapshot = _committed_activation(args, allow_pending=pending_ok)
    except (LifecycleError, ProfileError, SnapshotError, OSError, ValueError) as error:
        reason = getattr(error, "reason", "") or "config_unreadable"
        document = _document(
            args,
            fields,
            Readiness(state=UNHEALTHY, checks=()),
            profile="",
            activation="",
        )
        document["reason"] = reason
        document["action_code"] = action_for(reason)
        # The message, not the traceback: a responder needs a state to act on,
        # and a journal full of stack frames is what this contract replaced.
        document["detail"] = getattr(error, "detail", "") or f"{type(error).__name__}: {error}"
        return Readiness(state=UNHEALTHY, checks=()), document

    if snapshot is None:
        verdict = Readiness(state=ACTIVATION_PENDING, checks=())
        return verdict, _document(args, fields, verdict, profile="", activation="")

    profile: Profile = snapshot.spec()
    verdict = evaluate(profile, fields, config_file=Path(fields["config_file"]))
    return verdict, _document(
        args,
        fields,
        verdict,
        profile=profile.name,
        activation=snapshot.activation_id,
    )


def _write_status(directory: Path, name: str, document: dict) -> Path:
    from .fsio import atomic_write

    directory.mkdir(parents=True, exist_ok=True)
    path = directory / name
    atomic_write(path, json.dumps(document, indent=2, sort_keys=True) + "\n", 0o640)
    return path


def _emit(document: dict) -> None:
    sys.stdout.write(json.dumps(document, sort_keys=True) + "\n")
    sys.stdout.flush()


def _max_cycles(args: argparse.Namespace) -> int:
    raw = os.getenv(MAX_CYCLES_ENV, "")
    if raw.strip().isdigit():
        return int(raw.strip())
    return args.max_cycles


def _delay(interval_seconds: float, *, settled: bool) -> float:
    """How long to wait before looking again.

    Once settled this is exactly what the operator configured; nothing here
    second-guesses the supervision interval.

    While catching up it is a *floor* as well as a ceiling. Taking the smaller of
    the two numbers alone means a device configured with ``--interval-seconds 0``
    -- which is a legitimate way to ask for "as fast as you can" in a bounded
    run -- spins the CPU flat for the whole bootstrap window on a long-running
    supervisor. A poll that exists to notice a commit that lands in milliseconds
    has no need to be faster than a quarter of a second.
    """

    interval = max(interval_seconds, 0.0)
    if settled:
        return interval
    if interval <= 0.0:
        return BOOTSTRAP_POLL_SECONDS
    return min(interval, BOOTSTRAP_POLL_SECONDS)


def _run_agent(args: argparse.Namespace) -> int:
    """Supervise: report state on an interval and keep running either way.

    The interval is not one number. Systemd starts this service *inside* an
    activation -- the transaction restarts and verifies it, and only then writes
    the state that commits it -- so the first document this process publishes is
    always about the previous world: no activation at all on a clean install, the
    outgoing one on a profile switch. Sleeping the steady-state interval there
    left the published document wrong for half a minute, which is precisely the
    half minute an operator watching an install is reading it.

    So the process looks again quickly until what it observes *changes*, and then
    settles. It never exits to get a fresh look: exiting is what
    ``Restart=on-failure`` turns into a flap, and a supervisor that restarts to
    re-read a file is a supervisor that cannot be trusted to stay up.
    """

    limit = _max_cycles(args)
    completed = 0
    started = time.monotonic()
    baseline: str | None = None
    settled = False
    while True:
        verdict, document = cycle(args)
        document["status_file"] = str(
            _write_status(args.state_dir, AGENT_STATUS_FILE, document)
        )
        _emit(document)
        completed += 1
        if limit and completed >= limit:
            # An unready device is still a device this process is supposed to
            # be watching, so the exit status reports the release, not the
            # pairing: `unhealthy` is a failure, `provisioning_pending` is not.
            return 0 if verdict.ok else 1

        observed = str(document.get("activation", ""))
        if baseline is None:
            baseline = observed
        elif observed != baseline:
            # The activation that restarted this service has committed. There is
            # nothing further to catch up with, so stop looking often.
            settled = True
        if not settled and time.monotonic() - started >= BOOTSTRAP_WINDOW_SECONDS:
            settled = True
        if verdict.state == ACTIVATION_PENDING:
            # Keep looking often for as long as the *lifecycle* says a commit is
            # genuinely in flight. This is bounded by the transaction window
            # rather than by the number above: the window is durable, is clamped
            # on disk, and turns into `unhealthy` by itself when it lapses, so
            # the loop cannot idle here -- while settling early would publish a
            # pending document for a supervision interval after the machine had
            # already resolved one way or the other.
            settled = False
        time.sleep(_delay(args.interval_seconds, settled=settled))


def _run_doctor(args: argparse.Namespace) -> int:
    """One shot: write the snapshot the timer exists to produce, then exit.

    ``pending_ok=False`` because this command cannot wait. The supervisor can sit
    inside a live transaction window and watch it resolve; a one-shot publishes a
    single document and exits, so accepting "a commit is in flight" would let a
    device with no committed state be recorded as a fresh install on every tick
    of the timer, forever, in the exact file a support bundle reads.
    """

    verdict, document = cycle(args, pending_ok=False)
    document["status_file"] = str(_write_status(args.state_dir, DOCTOR_STATUS_FILE, document))
    _emit(document)
    # Type=oneshot. A snapshot that could not be taken is a failure; a device
    # that has simply not been paired yet is not, and must not mark the timer's
    # unit failed on every uncommissioned machine in the fleet.
    return 0 if verdict.ok else 1


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="flyto-robot-service",
        description="The long-running and one-shot processes the installed units execute.",
    )
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("agent", "doctor", "readiness"):
        action = sub.add_parser(name)
        action.add_argument("--config-dir", type=Path, required=True)
        action.add_argument("--state-dir", type=Path, required=True)
        action.add_argument("--log-dir", type=Path, default=Path("/var/log/flyto-robot"))
        action.add_argument("--release", type=Path, default=Path("/opt/flyto-robot/current"))
        # Deliberately no --profile / --profiles. The contract in force is the
        # one the installed activation snapshot records; a flag here would let a
        # unit evaluate a contract the device was never installed under.
        action.add_argument("--interval-seconds", type=float, default=30.0)
        action.add_argument(
            "--max-cycles",
            type=int,
            default=0,
            help="stop after this many cycles; 0 supervises forever",
        )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    try:
        if args.command == "agent":
            return _run_agent(args)
        if args.command == "doctor":
            return _run_doctor(args)
        # `readiness` is the other one-shot, and it refuses for the same reason
        # the doctor does: it answers once and exits.
        verdict, document = cycle(args, pending_ok=False)
        _emit(document)
        return 0 if verdict.ok else 1
    except (LifecycleError, ProfileError, SnapshotError, OSError, ValueError) as error:
        # The backstop, not the path. Everything the lifecycle can refuse is
        # already turned into a full status document by `cycle`, which is what
        # keeps the supervise loop from exiting on a device it is supposed to
        # keep reporting on. What reaches here is the unanticipated case -- and
        # it is still one JSON object, because a unit whose failures arrive as
        # tracebacks gives a support responder a journal to read instead of a
        # state to act on, and an argparse exit 2 gives them nothing at all.
        reason = getattr(error, "reason", "") or "unexpected_error"
        _emit(
            {
                "schema": READINESS_VERSION,
                "service": args.command,
                "state": "unhealthy",
                "ok": False,
                "reason": reason,
                "action_code": action_for(reason),
                "error": f"{type(error).__name__}: {error}",
            }
        )
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
