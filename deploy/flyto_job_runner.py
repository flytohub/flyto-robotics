#!/usr/bin/env python3
"""Claim Flyto2 jobs for this robot and run them through the local gateway.

Flyto2 dispatches an execution job to a device. Something on the robot has to
claim it, carry it out and report back. The cloud backend ships a runner that
does this — `connected_runner.py` — but its import closure reaches 379 modules
and twenty-one third-party packages including Stripe, Firebase Admin, boto3 and
flyto-core, because it is the whole backend minus the web server. On a robot
that is the wrong shape, and none of it is needed: a robot job is a plan, and
the thing that executes a plan is already running on loopback.

So this speaks the same device API and nothing else. Standard library only.

Outbound only. It opens no port and holds no inbound credential: a pairing code
is exchanged once for a device credential stored 0600, and every later request
carries only that.

What it deliberately does not do:

* **Decide anything about motion.** It forwards a plan and reports what came
  back. Bounds, the frozen capability set, the one-mission-at-a-time rule and
  the final safe stop all belong to the gateway, which owns the robot.
* **Retry a job.** A job that failed is reported failed. Re-running something
  that already moved a robot, on the assumption it did not, is how a retry
  becomes a collision.
* **Interpret a non-robot step.** A job whose steps this does not recognise is
  completed as failed with a stated reason, not skipped silently.
"""

from __future__ import annotations

import contextlib
import hashlib
import hmac
import importlib.util
import json
import logging
import math
import os
import re
import signal
import stat
import sys
import time
import urllib.error
import urllib.request
from collections.abc import Mapping, Sequence
from datetime import datetime, timezone
from pathlib import Path
from types import ModuleType
from typing import Any

logger = logging.getLogger("flyto.job_runner")

CLOUD_URL = os.getenv("FLYTO_CLOUD_URL", "https://api.flyto2.com").rstrip("/")
GATEWAY_URL = os.getenv("FLYTO_ROBOTICS_GATEWAY_URL", "http://127.0.0.1:8766").rstrip("/")
DATA_DIR = Path(os.getenv("FLYTO_RUNNER_DATA_DIR", "/home/ubuntu/.flyto"))
CREDENTIAL_FILE = DATA_DIR / "runner-credentials.json"

# Owner only, for both. What this protects against is every other account on
# the robot, a careless backup, and anything that walks the filesystem. What it
# cannot protect against is physical possession of the SD card: an unattended
# device must be able to read its own secret at boot with no operator present,
# so any key it holds is a key the card holds. Encrypting it against a key
# stored beside it would look stronger and be worth nothing. Where the hardware
# can do better — a TPM — see _systemd_credential.
CREDENTIAL_MODE = 0o600
DATA_DIR_MODE = 0o700

#: Name the unit passes via LoadCredentialEncrypted=, read from tmpfs.
SYSTEMD_CREDENTIAL_NAME = "flyto-device"

# Commissioning is deliberately much smaller than the long-running runner.
# These bounds apply before any server-controlled value is retained or emitted.
PAIR_RESPONSE_MAX_BYTES = 4096
PAIR_STORED_MAX_BYTES = 4096
PAIR_CREDENTIAL_MAX_CHARS = 512
PAIR_CODE_MAX_CHARS = 256
PAIR_NAME_MAX_CHARS = 128
PAIR_DEVICE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:-]{0,127}$")

POLL_WAIT_SECONDS = 25
HEARTBEAT_INTERVAL_SECONDS = 30.0
IDLE_DELAY_SECONDS = 1.0
ERROR_BASE_DELAY_SECONDS = 3.0
ERROR_MAX_DELAY_SECONDS = 60.0
# How long to keep watching a mission before reporting the outcome unknown.
#
# Derived from the gateway's own bound when it tells us one — the session
# payload carries mission_timeout_seconds — plus a margin for the final stop
# and the report. A constant here is a second opinion about a number the
# gateway already owns: the job deployed on this robot allows 600s, so a
# fixed 300s would call a legitimately running mission unknown at half time,
# and the cloud takes that "failed" at face value.
MISSION_WATCH_MARGIN_SECONDS = 20.0
DEFAULT_MISSION_WATCH_SECONDS = 300.0
GATEWAY_POLL_SECONDS = 1.0

# What the gateway actually calls a finished mission. These are
# ``MissionState`` values (flyto_robotics/workflow.py), not invented names:
# a mission that finished its workflow reports "completed". The first version
# of this runner waited for "succeeded", which no gateway has ever sent, so
# every real mission would have been reported as "outcome unknown" after the
# full five-minute watch. "succeeded" is kept only because the delivery
# adapter's own tests speak it.
TERMINAL_STATES = frozenset({"completed", "succeeded", "failed", "cancelled", "aborted"})
SUCCESS_STATES = frozenset({"completed", "succeeded"})
DELIVERY_SESSION_RECEIPT_CONTRACT = "flyto.robotics.delivery-session.v2"
EXECUTION_RECEIPT_CONTRACT = "flyto.robotics.execution-receipt.v1"
MAX_EXECUTION_RECEIPT_BYTES = 16384
DEVICE_JOB_HANDOFF_CONTRACT = "flyto.cloud.device-job-handoff.v1"
TASK_COMPLETION_AUTHORITY = "flyto.space.evidence.v1"
MAX_DEVICE_JOB_HANDOFF_BYTES = 4096
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_EXECUTION_RECEIPT_FIELDS = frozenset(
    {
        "contract_version",
        "request_id",
        "session_id",
        "job_id",
        "robot_id",
        "workflow_id",
        "status",
        "plan_sha256",
        "mission_result_sha256",
        "events_sha256",
        "event_count",
        "safety_stop_count",
        "final_pose",
        "minimum_range",
        "elapsed_seconds",
        "task_completion_eligible",
        "receipt_sha256",
    }
)

# The header the device API reads a claim's lease from — api/devices/
# routes_jobs.py's LEASE_HEADER. This runner sent "X-Flyto-Lease", which that
# route does not read, so every completion was refused with 409 "Job lease is
# missing or invalid" after the mission had already run. A guessed header name
# is indistinguishable from no header at all.
LEASE_HEADER = "x-flyto2-job-lease"

_stopping = False


class RunnerError(RuntimeError):
    """The runner cannot continue without an operator."""


class AuthoredPlanRefused(RuntimeError):
    """A recognised Canvas robot step failed closed before execution."""

    def __init__(self, reason_code: str) -> None:
        super().__init__(reason_code)
        self.reason_code = reason_code


class DeviceHandoffRefused(RuntimeError):
    """Cloud's versioned ownership transfer did not bind this exact job."""

    def __init__(self, reason_code: str) -> None:
        super().__init__(reason_code)
        self.reason_code = reason_code


# -- the shared device event contract ------------------------------------
#
# The journal, the envelope and the privacy rules live in exactly one place:
# flyto_robotics/device_events.py, which is standard library only and stays
# importable on a device with no ROS and no simulator. A second copy here would
# be free to drift, and a drift in what counts as a credential is the kind that
# is invisible until something leaves the device.
#
# It is loaded by path rather than imported by name for two reasons that both
# matter on the robot:
#
# * This file is executed as an absolute path from a systemd unit whose
#   WorkingDirectory is the repository root, so sys.path[0] is deploy/ and the
#   package is its sibling. Nothing may be assumed about PYTHONPATH; the unit
#   sets none, and a runner that depends on one silently stops recording the
#   day someone tidies the environment file.
# * `import flyto_robotics.device_events` would execute the package's
#   __init__.py, which imports the AI planner, the capability registry and the
#   ROS adapters. Dragging that onto a Pi is the exact thing this runner exists
#   to avoid.

DEVICE_EVENTS_PATH = (
    Path(__file__).resolve().parent.parent / "flyto_robotics" / "device_events.py"
)
#: A name of its own, so nothing here can be mistaken for the installed package
#: and no partially-initialised package is left in sys.modules.
DEVICE_EVENTS_MODULE = "flyto_runner_device_events"

_device_events: ModuleType | None = None


def device_events() -> ModuleType:
    """The shared event module, loaded once, by path, with no package import."""
    global _device_events
    if _device_events is not None:
        return _device_events
    cached = sys.modules.get(DEVICE_EVENTS_MODULE)
    if cached is not None:
        _device_events = cached
        return cached
    if not DEVICE_EVENTS_PATH.is_file():
        raise RunnerError(
            f"the shared device event contract is not beside this runner at "
            f"{DEVICE_EVENTS_PATH}; this deployment cannot record what it does"
        )
    spec = importlib.util.spec_from_file_location(DEVICE_EVENTS_MODULE, DEVICE_EVENTS_PATH)
    if spec is None or spec.loader is None:
        raise RunnerError(f"{DEVICE_EVENTS_PATH} cannot be loaded as a module")
    module = importlib.util.module_from_spec(spec)
    sys.modules[DEVICE_EVENTS_MODULE] = module
    try:
        spec.loader.exec_module(module)
    except BaseException:
        sys.modules.pop(DEVICE_EVENTS_MODULE, None)
        raise
    _device_events = module
    return module


#: The file name this service's journal takes inside its own data directory.
EVENT_JOURNAL_NAME = "device-events.jsonl"

#: This service's own journal, and never the doctor's. That one is written by
#: root; this one by the service user. A single shared file would have to be
#: readable by both accounts, and a device journal group or others can read is
#: already disclosed.
#
# An explicit FLYTO_DEVICE_EVENT_JOURNAL wins; otherwise it follows
# FLYTO_RUNNER_DATA_DIR, which is the one thing the enterprise drop-in moves
# (to /var/lib/flyto-runner). A hard-coded absolute default here would not move
# with it, and an offline site would keep writing its records into a home
# directory the service may not even have.
EVENT_JOURNAL = Path(os.getenv("FLYTO_DEVICE_EVENT_JOURNAL", "") or DATA_DIR / EVENT_JOURNAL_NAME)

#: Generic on purpose. Nothing upstream should have to know a robot produced it.
EVENT_COMPONENT = "device_job_runner"

#: The contract's own identifier shape. A job id or a resource id that does not
#: fit it is linked by a stable digest rather than dropped — losing the linkage
#: would make an event unattributable, which is worse than an opaque id.
_EVENT_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._:@/-]{0,127}$")

#: One short fixed sentence per reason code. Fixed, not formatted: a message
#: assembled from an exception, a URL or a gateway reply is how a credential, an
#: address or a plan payload ends up in a fleet-wide event stream.
EVENT_MESSAGES = {
    "job_execution_started": "This device began carrying out an assigned job.",
    "job_lease_missing": "The job was claimed without a lease, so nothing was started.",
    "job_plan_unsupported": "The job carries no plan this device can carry out.",
    "device_handoff_invalid": "The Cloud-to-device job handoff did not pass validation.",
    "device_handoff_target_mismatch": "The Cloud-to-device job handoff names another device.",
    "capability_catalog_unavailable": "The trusted robot capability catalog is unavailable.",
    "capability_catalog_refused": "The trusted robot capability catalog request was refused.",
    "capability_catalog_invalid": "The robot capability catalog did not pass validation.",
    "capability_catalog_incompatible": "The authored step is incompatible with this robot.",
    "trusted_plan_construction_failed": "The trusted robot plan could not be constructed safely.",
    "job_completed": "The job finished and its outcome was observed.",
    "mission_failed": "The job did not finish successfully.",
    "mission_outcome_unknown": "The job's outcome was not observed within its own time bound.",
    "execution_receipt_missing": "The execution gateway returned no required terminal receipt.",
    "execution_receipt_invalid": "The execution gateway returned an invalid terminal receipt.",
    "gateway_unreachable": "The local execution gateway could not be used.",
    "gateway_returned_no_session": "The local execution gateway accepted nothing to watch.",
    "completion_report_refused": "The outcome was recorded here but the report was refused.",
    # A refusal and an unreachable upstream are different faults with different
    # first moves: one is answered by whoever owns the job upstream, the other by
    # whoever owns the link. Folding them into one code would leave an offline
    # operator unable to tell "Cloud said no" from "Cloud was never reached".
    "completion_report_unreachable": (
        "The outcome was recorded here but the report could not be delivered."
    ),
    "device_executor_refused": "The installed device executor refused this job safely.",
    "device_executor_failed": "The installed device executor did not complete this job.",
    "device_executor_registry_error": "The device executor registry could not be used safely.",
    "device_executor_replay_refused": (
        "This device cannot prove that an earlier execution did not already occur."
    ),
    "device_executor_started": "This device began carrying out a registered device job.",
    "device_executor_succeeded": "The registered device job completed successfully.",
}

#: What an operator should try, in order. Codes, never shell commands.
EVENT_ACTIONS = {
    "job_execution_started": (),
    "job_lease_missing": ("retry_job_claim", "inspect_job_lease"),
    "job_plan_unsupported": ("inspect_job_steps",),
    "device_handoff_invalid": ("inspect_job_handoff", "inspect_cloud_dispatch"),
    "device_handoff_target_mismatch": ("inspect_job_assignment", "inspect_device_identity"),
    "capability_catalog_unavailable": ("inspect_gateway_service", "retry_job"),
    "capability_catalog_refused": ("inspect_gateway_authorization", "retry_job"),
    "capability_catalog_invalid": ("inspect_gateway_catalog", "inspect_module_version"),
    "capability_catalog_incompatible": ("inspect_authored_step", "inspect_gateway_catalog"),
    "trusted_plan_construction_failed": ("inspect_module_version", "inspect_authored_step"),
    "job_completed": (),
    "mission_failed": ("inspect_mission_outcome",),
    "mission_outcome_unknown": ("inspect_gateway_session", "inspect_mission_outcome"),
    "execution_receipt_missing": ("inspect_gateway_session", "inspect_gateway_version"),
    "execution_receipt_invalid": ("inspect_gateway_session", "inspect_gateway_version"),
    "gateway_unreachable": ("inspect_gateway_service", "retry_job"),
    "gateway_returned_no_session": ("inspect_gateway_service",),
    "completion_report_refused": ("retry_completion_report", "inspect_job_lease"),
    "completion_report_unreachable": ("retry_completion_report", "inspect_device_uplink"),
    "device_executor_refused": ("inspect_device_executor",),
    "device_executor_failed": ("inspect_device_executor",),
    "device_executor_registry_error": ("inspect_device_executor_registry",),
    "device_executor_replay_refused": ("inspect_device_event_journal", "reconcile_job"),
    "device_executor_started": (),
    "device_executor_succeeded": (),
}

DEVICE_EXECUTOR_MANIFEST_DIR = Path(
    os.getenv("FLYTO_DEVICE_EXECUTOR_MANIFEST_DIR", "/etc/flyto/device-executors")
)
DEVICE_EXECUTOR_PACKAGE = "flyto_runner_device_executors"
_device_executor_registry: Any = None


def _executor_registry() -> Any:
    """Load the sibling registry for both package imports and direct launch."""
    global _device_executor_registry
    if _device_executor_registry is not None:
        return _device_executor_registry
    if (
        not DEVICE_EXECUTOR_MANIFEST_DIR.is_absolute()
        or Path(os.path.normpath(str(DEVICE_EXECUTOR_MANIFEST_DIR)))
        != DEVICE_EXECUTOR_MANIFEST_DIR
    ):
        raise RunnerError("device_executor_registry_unavailable")
    package = sys.modules.get(DEVICE_EXECUTOR_PACKAGE)
    if package is None:
        package = ModuleType(DEVICE_EXECUTOR_PACKAGE)
        package.__path__ = [str(Path(__file__).resolve().parent)]
        package.__package__ = DEVICE_EXECUTOR_PACKAGE
        sys.modules[DEVICE_EXECUTOR_PACKAGE] = package
    try:
        registry_module = __import__(
            f"{DEVICE_EXECUTOR_PACKAGE}.device_executor_registry",
            fromlist=["DeviceExecutorRegistry"],
        )
        _device_executor_registry = registry_module.DeviceExecutorRegistry(
            DEVICE_EXECUTOR_MANIFEST_DIR
        )
    except Exception:
        raise RunnerError("device_executor_registry_unavailable") from None
    return _device_executor_registry


def _generic_step(job: Mapping[str, Any], registry: Any) -> tuple[str, Any] | None:
    """Return one unambiguous registry-owned step, without domain knowledge."""
    matches: list[tuple[str, Any]] = []
    steps = job.get("steps")
    if not isinstance(steps, Sequence) or isinstance(steps, (str, bytes)):
        return None
    metadata = registry.module_metadata
    for step in steps:
        if not isinstance(step, dict):
            continue
        selectors = {
            value.strip()
            for key in ("module", "module_id", "action", "type")
            if isinstance((value := step.get(key)), str) and value.strip()
        }
        if len(selectors) > 1:
            raise RunnerError("device_executor_selector_ambiguous")
        if not selectors:
            continue
        module_id = next(iter(selectors))
        if module_id in metadata:
            if module_id.startswith("robotics."):
                raise RunnerError("device_executor_ownership_collision")
            has_params = "params" in step
            has_arguments = "arguments" in step
            if has_params and has_arguments and step["params"] != step["arguments"]:
                raise RunnerError("device_executor_arguments_ambiguous")
            arguments = step.get("params") if has_params else step.get("arguments", {})
            matches.append((module_id, arguments))
    if len(matches) > 1:
        raise RunnerError("device_executor_selector_ambiguous")
    return matches[0] if matches else None


_GENERIC_REPLAY_CODES = frozenset(
    {
        "device_executor_started",
        "device_executor_succeeded",
        "device_executor_failed",
        "device_executor_refused",
        "device_executor_replay_refused",
    }
)


def _generic_replay_seen(job_id: str) -> bool:
    """Fail closed if this generic run may already have reached execution."""
    run_id = _event_identifier(job_id, "job")
    try:
        os.lstat(EVENT_JOURNAL)
    except FileNotFoundError:
        # A journal that has never been created is the canonical empty history.
        # Only absence is accepted here: every existing-but-invalid shape is
        # handed to the journal reader below and therefore fails closed.
        return False
    records = device_events().DeviceEventJournal(EVENT_JOURNAL).read_all()
    return any(
        record.get("event", {}).get("run_id") == run_id
        and record.get("event", {}).get("reason_code") in _GENERIC_REPLAY_CODES
        for record in records
    )


def _generic_failure(
    credentials: Mapping[str, str],
    *,
    job_id: str,
    headers: dict[str, str],
    reason_code: str,
) -> None:
    """Record and report one fixed generic failure without private content."""
    _append_event(
        credentials,
        status="refused" if reason_code.endswith("refused") else "failed",
        severity="warning" if reason_code.endswith("refused") else "error",
        reason_code=reason_code,
        job_id=job_id,
    )
    _report_completion(
        credentials,
        job_id=job_id,
        headers=headers,
        body=_completion(status="failed", detail=reason_code),
    )


def _event_identifier(value: str, prefix: str) -> str:
    if _EVENT_IDENTIFIER.fullmatch(value):
        return value
    return f"{prefix}-{hashlib.sha256(value.encode('utf-8')).hexdigest()[:32]}"


def _resource_id(credentials: Mapping[str, str]) -> str:
    """Which device these events are about. Configured, or the paired identity.

    There is no fallback and there must not be one. A placeholder such as
    "unidentified-device" is worse than no event at all: every robot in the
    fleet emits under the same resource_id, so the records interleave into one
    stream that names no machine, and the first time an operator needs to know
    *which* robot refused to start they cannot find out — from records that
    looked complete the whole time. A hostname is not an alternative either: it
    names the network the device sits on, which this contract deliberately omits.

    FLYTO_ROBOT_RESOURCE_ID lets a site keep its own naming; absent that, the
    device_id the pairing returned is already a stable per-device identity that
    upstream issued, so it needs no second agreement about naming.
    """
    configured = os.getenv("FLYTO_ROBOT_RESOURCE_ID", "").strip()
    if configured:
        return _event_identifier(configured, "device")
    paired = str(credentials.get("device_id") or "").strip()
    if paired:
        return _event_identifier(paired, "device")
    raise RunnerError(
        "no device identity: set FLYTO_ROBOT_RESOURCE_ID or pair this device. "
        "Events attributed to no machine cannot be acted on, so none are written."
    )


def _append_event(
    credentials: Mapping[str, str],
    *,
    status: str,
    severity: str,
    reason_code: str,
    job_id: str = "",
    details: dict[str, Any] | None = None,
    action_codes: Sequence[str] | None = None,
    correlation_id: str = "",
) -> dict[str, Any]:
    """Record one bounded, public observation. Raises if it cannot be recorded.

    Raising is the point at the start of a job: an audit record that may or may
    not have been written is not an audit record, and a robot that moved with no
    trace of having been told to is the state this exists to prevent.
    """
    events = device_events()
    moment = datetime.now(timezone.utc)
    event = events.build_device_event(
        resource_id=_resource_id(credentials),
        component=EVENT_COMPONENT,
        sequence=events.event_sequence(moment),
        observed_at=events.now_observed_at(moment),
        severity=severity,
        status=status,
        reason_code=reason_code,
        action_codes=list(
            action_codes if action_codes is not None else EVENT_ACTIONS.get(reason_code, ())
        ),
        correlation_id=(
            _event_identifier(correlation_id, "trace") if correlation_id else ""
        ),
        # The job this belongs to. Empty would mean "no run at all", which for a
        # job runner is never true and would break every upstream join.
        run_id=_event_identifier(job_id, "job") if job_id else "",
        message=EVENT_MESSAGES[reason_code],
        details=dict(details or {}),
    )
    events.DeviceEventJournal(EVENT_JOURNAL).append(event)
    return event


# -- transport -----------------------------------------------------------


def _call(
    base: str,
    path: str,
    *,
    payload: Any = None,
    headers: dict[str, str] | None = None,
    timeout: float = 35.0,
) -> Any:
    request = urllib.request.Request(
        f"{base}{path}",
        data=None if payload is None else json.dumps(payload).encode("utf-8"),
        headers={
            **(headers or {}),
            **({"Content-Type": "application/json"} if payload is not None else {}),
        },
        method="POST" if payload is not None else "GET",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:
        body = response.read()
        return json.loads(body) if body else {}


def _post(
    base: str, path: str, payload: Any, headers: dict[str, str], timeout: float = 35.0
) -> Any:
    return _call(
        base, path, payload=payload if payload is not None else {}, headers=headers, timeout=timeout
    )


# -- credentials ---------------------------------------------------------


def _validated(data: Any, source: Path) -> dict[str, str]:
    if not isinstance(data, dict) or not data.get("device_id") or not data.get("device_secret"):
        raise RunnerError(f"{source} is malformed; delete it and pair again")
    return data


def _systemd_credential() -> dict[str, str] | None:
    """A credential systemd placed in memory for us, if the unit supplies one.

    ``LoadCredentialEncrypted=`` decrypts into ``$CREDENTIALS_DIRECTORY``: a
    private tmpfs, readable only by this service, that never touches persistent
    storage. On a host with a TPM the ciphertext at rest is sealed to the
    hardware, so a stolen disk yields nothing.

    Where an operator can provision that, none of the on-disk path below runs
    and this process never writes a secret to a filesystem at all. The lab
    TurtleBot3 is a Raspberry Pi 4 with no TPM and self-service pairing, so it
    still uses the file — see the module notes on what that does and does not
    protect against.
    """
    directory = os.getenv("CREDENTIALS_DIRECTORY")
    if not directory:
        return None
    path = Path(directory) / SYSTEMD_CREDENTIAL_NAME
    if not path.is_file():
        return None
    return _validated(json.loads(path.read_text()), path)


def _refuse_loose_permissions(path: Path) -> None:
    """Stop if anyone but the owner can read the secret.

    A credential restored from a backup, copied with ``cp`` under a permissive
    umask, or chmod'd by hand comes back readable and nothing would have said
    so. The exposure is silent by nature: the file keeps working perfectly.
    """
    mode = path.stat().st_mode & 0o777
    if mode & 0o077:
        raise RunnerError(
            f"{path} is mode {mode:04o}. A device secret that group or others "
            "can read is already disclosed, so this will not use it. Either "
            "chmod 600 it, or — if you cannot account for the permissions — "
            "delete it and pair again, which revokes what leaked."
        )


def _read_credentials() -> dict[str, str] | None:
    from_systemd = _systemd_credential()
    if from_systemd is not None:
        return from_systemd
    if not CREDENTIAL_FILE.exists():
        return None
    _refuse_loose_permissions(CREDENTIAL_FILE)
    return _validated(json.loads(CREDENTIAL_FILE.read_text()), CREDENTIAL_FILE)


def _write_credentials(data: dict[str, str]) -> None:
    """Persist the credential without ever exposing it, even briefly.

    ``Path.write_text`` creates at ``0o666 & ~umask`` — 0644 under the usual
    umask — and only narrows afterwards, so the secret sat world-readable for
    the length of a write and a chmod. Opening with the final mode closes that
    window: the file has never existed at any other permission.

    The write is also atomic. A power cut mid-write used to leave truncated
    JSON, which the reader rejects as malformed, which costs the pairing — and
    a robot loses power for reasons that have nothing to do with software.
    """
    DATA_DIR.mkdir(parents=True, exist_ok=True, mode=DATA_DIR_MODE)
    # mkdir's mode applies only when it creates the directory, and umask masks
    # it even then. Set it outright so an existing loose directory is tightened.
    os.chmod(DATA_DIR, DATA_DIR_MODE)

    partial = DATA_DIR / f"{CREDENTIAL_FILE.name}.partial"
    # A leftover partial from an interrupted write holds a secret of its own.
    partial.unlink(missing_ok=True)
    descriptor = os.open(partial, os.O_CREAT | os.O_EXCL | os.O_WRONLY, CREDENTIAL_MODE)
    try:
        with os.fdopen(descriptor, "w") as handle:
            json.dump(data, handle)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(partial, CREDENTIAL_FILE)
        # Durability of the rename itself, for the same power-cut reason.
        directory = os.open(DATA_DIR, os.O_RDONLY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    except BaseException:
        partial.unlink(missing_ok=True)
        raise


def _pair() -> dict[str, str]:
    """Exchange a one-time pairing code for a device credential.

    The code is popped from the environment so it cannot be reused or read back
    out of this process, and it is never written anywhere.
    """
    code = os.environ.pop("FLYTO_PAIRING_CODE", "").strip()
    if not code:
        raise RunnerError(
            "No stored credential. Set FLYTO_PAIRING_CODE once, from "
            "Pair a device in the operations room."
        )
    body = _post(
        CLOUD_URL,
        "/api/devices/pair/claim",
        {
            "pairing_code": code,
            "name": os.getenv("FLYTO_RUNNER_NAME", os.uname().nodename),
            "platform": "robot",
        },
        headers={},
    )
    if not body.get("device_id") or not body.get("device_secret"):
        raise RunnerError("pairing did not return a device credential")
    credentials = {
        "device_id": body["device_id"],
        "device_secret": body["device_secret"],
    }
    _write_credentials(credentials)
    logger.info("paired as device %s", credentials["device_id"])
    return credentials


def _pair_response(data: Any) -> dict[str, str]:
    """Accept only the two-field credential contract issued by pairing."""
    if not isinstance(data, dict) or set(data) != {"device_id", "device_secret"}:
        raise RunnerError("pairing_response_invalid")
    device_id = data["device_id"]
    device_secret = data["device_secret"]
    if (
        not isinstance(device_id, str)
        or PAIR_DEVICE_ID.fullmatch(device_id) is None
        or not isinstance(device_secret, str)
        or not device_secret
        or len(device_secret) > PAIR_CREDENTIAL_MAX_CHARS
        or any(ord(character) < 0x21 or ord(character) > 0x7E for character in device_secret)
    ):
        raise RunnerError("pairing_response_invalid")
    return {"device_id": device_id, "device_secret": device_secret}


def _strict_pair_json(body: bytes) -> Any:
    """Decode bounded UTF-8 JSON without duplicate names or numeric constants."""

    def reject_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError("duplicate_json_key")
            result[key] = value
        return result

    def reject_constant(_value: str) -> Any:
        raise ValueError("invalid_json_constant")

    try:
        text = body.decode("utf-8", errors="strict")
        return json.loads(
            text,
            object_pairs_hook=reject_pairs,
            parse_constant=reject_constant,
        )
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError):
        raise RunnerError("pairing_response_invalid") from None


def _pair_read_file(path: Path) -> dict[str, str]:
    """Read one owner-only regular credential file without following links."""
    if not hasattr(os, "O_NOFOLLOW"):
        raise RunnerError("stored_credential_invalid")
    flags = os.O_RDONLY | os.O_NOFOLLOW
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode) or metadata.st_mode & 0o077:
            raise RunnerError("stored_credential_invalid")
        with os.fdopen(descriptor, "rb", closefd=False) as handle:
            body = handle.read(PAIR_STORED_MAX_BYTES + 1)
        if not body or len(body) > PAIR_STORED_MAX_BYTES:
            raise RunnerError("stored_credential_invalid")
    finally:
        os.close(descriptor)
    try:
        return _pair_response(_strict_pair_json(body))
    except RunnerError:
        raise RunnerError("stored_credential_invalid") from None


def _pair_existing_credentials() -> dict[str, str] | None:
    """Find an existing pair credential using only the bounded pair reader."""
    directory = os.getenv("CREDENTIALS_DIRECTORY")
    if directory:
        systemd_path = Path(directory) / SYSTEMD_CREDENTIAL_NAME
        try:
            return _pair_read_file(systemd_path)
        except FileNotFoundError:
            pass
    try:
        return _pair_read_file(CREDENTIAL_FILE)
    except FileNotFoundError:
        return None


def _pair_ascii_input(value: str, maximum: int, reason: str) -> str:
    """Validate one request input before constructing any network object."""
    if (
        not value
        or len(value) > maximum
        or any(ord(character) < 0x21 or ord(character) > 0x7E for character in value)
    ):
        raise RunnerError(reason)
    return value


def _pair_request(code: str, name: str) -> dict[str, str]:
    """Make the single commissioning request and bound its response body."""
    request = urllib.request.Request(
        f"{CLOUD_URL}/api/devices/pair/claim",
        data=json.dumps(
            {
                "pairing_code": code,
                "name": name,
                "platform": "robot",
            }
        ).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(request, timeout=35.0) as response:
        body = response.read(PAIR_RESPONSE_MAX_BYTES + 1)
    if not body or len(body) > PAIR_RESPONSE_MAX_BYTES:
        raise RunnerError("pairing_response_invalid")
    return _pair_response(_strict_pair_json(body))


PAIR_RESULTS = {
    "paired": {"ok": True, "status": "paired"},
    "already_paired": {"ok": True, "status": "already_paired"},
}
PAIR_ERRORS = {
    "missing_code": {
        "ok": False,
        "reason": "pairing_code_missing",
        "action_code": "set_pairing_code",
    },
    "invalid_code": {
        "ok": False,
        "reason": "pairing_code_invalid",
        "action_code": "request_new_pairing_code",
    },
    "invalid_name": {
        "ok": False,
        "reason": "runner_name_invalid",
        "action_code": "set_runner_name",
    },
    "existing_credential": {
        "ok": False,
        "reason": "stored_credential_invalid",
        "action_code": "repair_stored_credential",
    },
    "response": {
        "ok": False,
        "reason": "pairing_response_invalid",
        "action_code": "request_new_pairing_code",
    },
    "network": {
        "ok": False,
        "reason": "pairing_request_failed",
        "action_code": "retry_pairing",
    },
    "io": {
        "ok": False,
        "reason": "credential_storage_failed",
        "action_code": "inspect_credential_storage",
    },
}


def _pair_output(value: Mapping[str, Any]) -> None:
    """Emit one compact, content-free commissioning result."""
    print(json.dumps(value, sort_keys=True, separators=(",", ":")))


def pair_main() -> int:
    """Commission this installation without entering any job-running path."""
    code = os.environ.pop("FLYTO_PAIRING_CODE", "")
    try:
        existing = _pair_existing_credentials()
    except (RunnerError, ValueError, TypeError, OSError):
        _pair_output(PAIR_ERRORS["existing_credential"])
        return 2
    if existing is not None:
        _pair_output(PAIR_RESULTS["already_paired"])
        return 0
    if not code.strip():
        _pair_output(PAIR_ERRORS["missing_code"])
        return 2
    try:
        code = _pair_ascii_input(code, PAIR_CODE_MAX_CHARS, "pairing_code_invalid")
    except RunnerError:
        _pair_output(PAIR_ERRORS["invalid_code"])
        return 2
    try:
        name = _pair_ascii_input(
            os.getenv("FLYTO_RUNNER_NAME", "flyto-device"),
            PAIR_NAME_MAX_CHARS,
            "runner_name_invalid",
        )
    except RunnerError:
        _pair_output(PAIR_ERRORS["invalid_name"])
        return 2
    try:
        credentials = _pair_request(code, name)
    except RunnerError:
        _pair_output(PAIR_ERRORS["response"])
        return 3
    except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError, OSError):
        _pair_output(PAIR_ERRORS["network"])
        return 3
    finally:
        code = ""
    try:
        _write_credentials(credentials)
    except (OSError, ValueError, TypeError):
        _pair_output(PAIR_ERRORS["io"])
        return 4
    _pair_output(PAIR_RESULTS["paired"])
    return 0


def _headers(credentials: dict[str, str]) -> dict[str, str]:
    return {
        "Authorization": (
            f"Bearer device:{credentials['device_id']}.{credentials['device_secret']}"
        )
    }


# -- executing one job ---------------------------------------------------


def _plan_from(job: dict[str, Any]) -> dict[str, Any] | None:
    """The robot plan this job carries, or None if it is not a robot job.

    Three shapes are accepted, in the order a job is most likely to carry
    them: a step whose params hold a plan outright, a step naming a plan file
    already on this robot, and a step authored on the canvas as one of the
    ``robotics.*`` motion steps. Anything else is not something this runner
    can carry out, and saying so is better than guessing.
    """
    for step in job.get("steps") or []:
        params = step.get("params") or step.get("arguments") or {}
        plan = params.get("plan")
        if isinstance(plan, dict) and plan.get("contract_version"):
            return plan
        reference = params.get("plan_path")
        if isinstance(reference, str) and reference:
            candidate = Path(reference)
            # Confined to the plans directory: a job must not be able to name an
            # arbitrary path on this machine.
            root = Path(
                os.getenv("FLYTO_PLAN_ROOT", "/home/ubuntu/flyto-robotics/examples/plans")
            ).resolve()
            resolved = (root / candidate.name).resolve()
            if resolved.parent == root and resolved.is_file():
                return json.loads(resolved.read_text())
        authored = _authored_plan(step, params)
        if authored is not None:
            return authored
    return None


def _authored_plan(step: dict[str, Any], params: dict[str, Any]) -> dict[str, Any] | None:
    """The plan a canvas-authored motion step describes, if this is one.

    The mapping from a step to a plan is not written here. It lives in
    flyto-modules-robotics, beside the module identifiers it gives meaning
    to, and the workflow engine reads the same table on the other side. One
    table, two readers: a copy here would be free to drift, and the drift
    would be invisible until a robot moved differently from what the canvas
    said.

    That package is pure Python with no dependencies, so installing it on a
    Pi costs nothing and pulls in no engine. Absent, this returns None and the
    job is reported as one this device cannot run — which is true, and better
    than a plan assembled from guesses.
    """
    # Do not import the optional package for an unrelated step. This mirrors
    # the public step_module_id spellings closely enough to decide only whether
    # the installed robotics package is relevant; that package remains the
    # authority for the actual module id below.
    authored_module = next(
        (
            value.strip()
            for key in ("module", "module_id", "action", "type")
            if isinstance((value := step.get(key)), str) and value.strip()
        ),
        "",
    )
    if not authored_module.startswith("robotics."):
        return None

    try:
        import flyto_modules_robotics  # noqa: F401 - distinguish optional root absence
    except ModuleNotFoundError as exc:
        if exc.name == "flyto_modules_robotics":
            return None
        raise AuthoredPlanRefused("trusted_plan_construction_failed") from None
    except ImportError:
        raise AuthoredPlanRefused("trusted_plan_construction_failed") from None

    try:
        from flyto_modules_robotics.gateway import (
            CapabilityCatalogError,
            GatewayError,
            GatewayRefused,
            capability_catalog,
        )
        from flyto_modules_robotics.steps import (
            PlanBuildError,
            step_module_id,
            trusted_plan_for_step,
        )
    except (ImportError, AttributeError):
        # The root package exists, so a failed internal import or missing 0.1.1
        # public symbol is a broken/incompatible installation, not absence of
        # robotics support. Fail closed under fixed, content-free reporting.
        raise AuthoredPlanRefused("trusted_plan_construction_failed") from None

    module_id = step_module_id(step)
    if not module_id or not module_id.startswith("robotics."):
        return None

    robot_id = os.getenv("FLYTO_ROBOTICS_ROBOT_ID", "").strip()
    if not robot_id:
        # The gateway checks a plan's robot_id against the job it was started
        # with, so a guess here becomes a refusal there, one layer further from
        # the cause.
        logger.warning(
            "step %s is a robotics step, but FLYTO_ROBOTICS_ROBOT_ID is not set "
            "on this robot; cannot say which robot the plan is for",
            module_id,
        )
        return None

    try:
        catalog = capability_catalog()
    except GatewayRefused:
        raise AuthoredPlanRefused("capability_catalog_refused") from None
    except GatewayError:
        raise AuthoredPlanRefused("capability_catalog_unavailable") from None
    except CapabilityCatalogError:
        raise AuthoredPlanRefused("capability_catalog_invalid") from None

    try:
        plan = trusted_plan_for_step(module_id, params, robot_id=robot_id, catalog=catalog)
    except (PlanBuildError, CapabilityCatalogError):
        raise AuthoredPlanRefused("capability_catalog_incompatible") from None
    except Exception:  # noqa: BLE001 - fail closed without publishing library text
        raise AuthoredPlanRefused("trusted_plan_construction_failed") from None
    if plan is None:
        raise AuthoredPlanRefused("capability_catalog_incompatible")
    return plan


def _run_plan(plan: dict[str, Any], job_id: str) -> dict[str, Any]:
    """Hand the plan to the gateway and watch the session to an outcome."""
    token = os.getenv("FLYTO_ROBOTICS_DELIVERY_TOKEN", "").strip()
    if not token:
        raise RunnerError("FLYTO_ROBOTICS_DELIVERY_TOKEN is not set")
    gateway_headers = {"Authorization": f"Bearer {token}"}

    request_id = f"job-{job_id}"[:128]
    session = _post(
        GATEWAY_URL,
        "/v1/plans",
        {
            "contract_version": "flyto.cloud.plan-run-request.v1",
            "request_id": request_id,
            "plan": plan,
            "requested_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        },
        headers=gateway_headers,
        timeout=15.0,
    )
    session_id = str(session.get("session_id") or "")
    if not session_id:
        return {
            "status": "failed",
            "detail": "gateway returned no session",
            "reason_code": "gateway_returned_no_session",
        }

    watch_seconds = _watch_seconds(session)
    deadline = time.monotonic() + watch_seconds
    latest = session
    while time.monotonic() < deadline and not _stopping:
        # "status" is what the real session payload carries; "state" is
        # accepted because the delivery adapter's fixtures use it.
        state = str(latest.get("status") or latest.get("state") or "").lower()
        if state in TERMINAL_STATES:
            succeeded = state in SUCCESS_STATES
            receipt, receipt_error = _execution_receipt(
                latest,
                request_id=request_id,
                plan=plan,
                succeeded=succeeded,
            )
            if receipt_error:
                return {
                    "status": "failed",
                    "detail": receipt_error,
                    "reason_code": receipt_error,
                }
            return {
                "status": "succeeded" if succeeded else "failed",
                # The event carries a fixed reason code; this free text goes
                # only to the Cloud completion body, which is not the event.
                "reason_code": "job_completed" if succeeded else "mission_failed",
                "detail": str(latest.get("failure_reason") or latest.get("reason") or state)[:300],
                # The session payload spells this "pose"; "final_pose" is the
                # fixtures' name. Reading only the latter meant a real mission
                # produced no arrival evidence at all.
                "pose": latest.get("pose") or latest.get("final_pose"),
                # The closest thing the lidar saw during the mission — the same
                # reading the mission's own sensor gate trusts. This is what
                # answers "is that passage actually walkable", which a camera
                # frame cannot.
                "minimum_range": latest.get("minimum_range"),
                "execution_receipt": receipt,
            }
        time.sleep(GATEWAY_POLL_SECONDS)
        latest = _call(
            GATEWAY_URL, f"/v1/deliveries/{session_id}", headers=gateway_headers, timeout=15.0
        )

    # Not knowing an outcome is not the same as the mission having failed, and
    # the gateway still owns the robot. Report it as what it is.
    return {
        "status": "failed",
        "detail": f"outcome unknown after {watch_seconds:.0f}s",
        "reason_code": "mission_outcome_unknown",
    }


def _execution_receipt(
    session: Mapping[str, Any],
    *,
    request_id: str,
    plan: Mapping[str, Any],
    succeeded: bool,
) -> tuple[dict[str, Any] | None, str]:
    """Validate and detach a terminal gateway receipt before it reaches Cloud."""
    raw = session.get("execution_receipt")
    receipt_required = session.get("contract_version") == (
        DELIVERY_SESSION_RECEIPT_CONTRACT
    )
    if raw is None:
        return (None, "execution_receipt_missing") if receipt_required else (None, "")
    if not isinstance(raw, Mapping) or set(raw) != _EXECUTION_RECEIPT_FIELDS:
        return None, "execution_receipt_invalid"
    receipt = dict(raw)
    try:
        encoded = json.dumps(
            receipt,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError):
        return None, "execution_receipt_invalid"
    if len(encoded) > MAX_EXECUTION_RECEIPT_BYTES:
        return None, "execution_receipt_invalid"
    receipt = json.loads(encoded)
    if receipt.get("contract_version") != EXECUTION_RECEIPT_CONTRACT:
        return None, "execution_receipt_invalid"
    for field in ("request_id", "session_id", "job_id", "robot_id", "workflow_id"):
        value = receipt.get(field)
        if not isinstance(value, str) or not _EVENT_IDENTIFIER.fullmatch(value):
            return None, "execution_receipt_invalid"
    if receipt["request_id"] != request_id:
        return None, "execution_receipt_invalid"
    if receipt.get("status") not in {"succeeded", "failed", "cancelled", "aborted"}:
        return None, "execution_receipt_invalid"
    if (receipt["status"] == "succeeded") is not succeeded:
        return None, "execution_receipt_invalid"
    expected_plan = _canonical_sha256(plan)
    if receipt.get("plan_sha256") != expected_plan:
        return None, "execution_receipt_invalid"
    for field in (
        "mission_result_sha256",
        "events_sha256",
        "receipt_sha256",
    ):
        if not isinstance(receipt.get(field), str) or not _SHA256.fullmatch(receipt[field]):
            return None, "execution_receipt_invalid"
    for field in ("event_count", "safety_stop_count"):
        value = receipt.get(field)
        if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 100000:
            return None, "execution_receipt_invalid"
    if not _bounded_number(receipt.get("elapsed_seconds"), minimum=0.0, maximum=86400.0):
        return None, "execution_receipt_invalid"
    minimum_range = receipt.get("minimum_range")
    if minimum_range is not None and not _bounded_number(
        minimum_range, minimum=0.0, maximum=1000000.0
    ):
        return None, "execution_receipt_invalid"
    pose = receipt.get("final_pose")
    if pose is not None:
        if not isinstance(pose, Mapping) or set(pose) != {"x", "y", "yaw"}:
            return None, "execution_receipt_invalid"
        if not all(
            _bounded_number(pose.get(axis), minimum=-1000000.0, maximum=1000000.0)
            for axis in ("x", "y", "yaw")
        ):
            return None, "execution_receipt_invalid"
    if receipt.get("task_completion_eligible") is not False:
        return None, "execution_receipt_invalid"
    asserted_digest = receipt.pop("receipt_sha256")
    if not hmac.compare_digest(asserted_digest, _canonical_sha256(receipt)):
        return None, "execution_receipt_invalid"
    receipt["receipt_sha256"] = asserted_digest
    return receipt, ""


def _bounded_number(value: Any, *, minimum: float, maximum: float) -> bool:
    return (
        not isinstance(value, bool)
        and isinstance(value, (int, float))
        and math.isfinite(float(value))
        and minimum <= float(value) <= maximum
    )


def _canonical_sha256(value: object) -> str:
    raw = json.dumps(
        value,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _watch_seconds(session: dict[str, Any]) -> float:
    """How long this mission may run, according to the mission itself."""
    bound = session.get("mission_timeout_seconds")
    if isinstance(bound, (int, float)) and bound > 0:
        return float(bound) + MISSION_WATCH_MARGIN_SECONDS
    return DEFAULT_MISSION_WATCH_SECONDS


def _device_job_handoff(
    job: Mapping[str, Any],
    credentials: Mapping[str, str],
) -> str:
    """Validate the v1 handoff required by every trace-bearing Space job."""
    params = job.get("input_params")
    if not isinstance(params, Mapping):
        return ""
    if "_flyto_device_handoff" not in params:
        if "_flyto_trace_id" in params:
            raise DeviceHandoffRefused("device_handoff_invalid")
        return ""
    raw = params.get("_flyto_device_handoff")
    fields = {
        "contract_version",
        "device_id",
        "trace_id",
        "workflow_sha256",
        "task_completion_authority",
        "handoff_sha256",
    }
    if not isinstance(raw, Mapping) or set(raw) != fields:
        raise DeviceHandoffRefused("device_handoff_invalid")
    handoff = dict(raw)
    try:
        encoded = json.dumps(
            handoff,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8")
    except (TypeError, ValueError):
        raise DeviceHandoffRefused("device_handoff_invalid") from None
    if len(encoded) > MAX_DEVICE_JOB_HANDOFF_BYTES:
        raise DeviceHandoffRefused("device_handoff_invalid")
    if handoff.get("contract_version") != DEVICE_JOB_HANDOFF_CONTRACT:
        raise DeviceHandoffRefused("device_handoff_invalid")
    device_id = handoff.get("device_id")
    trace_id = handoff.get("trace_id")
    if (
        not isinstance(device_id, str)
        or not _EVENT_IDENTIFIER.fullmatch(device_id)
        or not isinstance(trace_id, str)
        or not _EVENT_IDENTIFIER.fullmatch(trace_id)
    ):
        raise DeviceHandoffRefused("device_handoff_invalid")
    if params.get("_flyto_trace_id") != trace_id:
        raise DeviceHandoffRefused("device_handoff_invalid")
    assigned = str(job.get("device_id") or "")
    paired = str(credentials.get("device_id") or "")
    if device_id != assigned or device_id != paired:
        raise DeviceHandoffRefused("device_handoff_target_mismatch")
    if handoff.get("workflow_sha256") != _canonical_sha256(job.get("steps")):
        raise DeviceHandoffRefused("device_handoff_invalid")
    if handoff.get("task_completion_authority") != TASK_COMPLETION_AUTHORITY:
        raise DeviceHandoffRefused("device_handoff_invalid")
    asserted = handoff.pop("handoff_sha256", None)
    if (
        not isinstance(asserted, str)
        or not _SHA256.fullmatch(asserted)
        or not hmac.compare_digest(asserted, _canonical_sha256(handoff))
    ):
        raise DeviceHandoffRefused("device_handoff_invalid")
    return trace_id


def _handle(job: dict[str, Any], credentials: dict[str, str]) -> None:
    job_id = str(job.get("job_id") or job.get("id") or "")
    if not job_id:
        return
    headers = _headers(credentials)

    claimed = _post(CLOUD_URL, f"/api/devices/jobs/{job_id}/claim", {}, headers)
    lease = str(claimed.get("lease_id") or "").strip()
    if not lease:
        # The completion endpoint refuses a report with no lease (409), so a
        # claim that returned none means this mission could never be reported.
        # Running it anyway moves a real robot in a hospital corridor and then
        # has no way to say that it did: the schedule still shows the step
        # pending, an operator re-dispatches, and the second run collides with
        # the consequences of the first. So this stops here, before the gateway
        # is touched at all — a job that cannot be reported is not started.
        logger.error("job %s was claimed without a lease; nothing was started", job_id)
        _append_event(
            credentials,
            status="refused",
            severity="warning",
            reason_code="job_lease_missing",
            job_id=job_id,
        )
        return
    headers = {**headers, LEASE_HEADER: lease}

    try:
        trace_id = _device_job_handoff(job, credentials)
    except DeviceHandoffRefused as exc:
        _append_event(
            credentials,
            status="refused",
            severity="warning",
            reason_code=exc.reason_code,
            job_id=job_id,
        )
        _report_completion(
            credentials,
            job_id=job_id,
            headers=headers,
            body=_completion(status="failed", detail=exc.reason_code),
        )
        return

    try:
        plan = _plan_from(job)
    except AuthoredPlanRefused as exc:
        logger.warning("job %s failed trusted Canvas plan preparation: %s", job_id, exc.reason_code)
        _append_event(
            credentials,
            status="refused",
            severity="warning",
            reason_code=exc.reason_code,
            job_id=job_id,
        )
        _report_completion(
            credentials,
            job_id=job_id,
            headers=headers,
            body=_completion(status="failed", detail="robot capability verification failed"),
        )
        return
    generic: tuple[Any, Any] | None = None
    registry = None
    try:
        registry = _executor_registry()
    except Exception:
        if plan is None:
            _generic_failure(
                credentials,
                job_id=job_id,
                headers=headers,
                reason_code="device_executor_registry_error",
            )
            return
    if registry is not None:
        try:
            selected = _generic_step(job, registry)
        except Exception:
            _generic_failure(
                credentials,
                job_id=job_id,
                headers=headers,
                reason_code="device_executor_registry_error",
            )
            return
        if selected is not None and plan is not None:
            _generic_failure(
                credentials,
                job_id=job_id,
                headers=headers,
                reason_code="device_executor_registry_error",
            )
            return
        if selected is not None:
            try:
                replay = _generic_replay_seen(job_id)
            except Exception:
                _generic_failure(
                    credentials,
                    job_id=job_id,
                    headers=headers,
                    reason_code="device_executor_replay_refused",
                )
                return
            if replay:
                _generic_failure(
                    credentials,
                    job_id=job_id,
                    headers=headers,
                    reason_code="device_executor_replay_refused",
                )
                return
            module_id, params = selected
            try:
                generic = (registry, registry.prepare(module_id, params))
            except Exception:
                _generic_failure(
                    credentials,
                    job_id=job_id,
                    headers=headers,
                    reason_code="device_executor_registry_error",
                )
                return

    if plan is None and generic is None:
        logger.warning("job %s carries no robot plan", job_id)
        # Recorded before the completion is sent, for the same reason the
        # terminal outcome is: if the report is refused, the refusal must still
        # be explainable from this device alone.
        _append_event(
            credentials,
            status="refused",
            severity="notice",
            reason_code="job_plan_unsupported",
            job_id=job_id,
        )
        _report_completion(
            credentials,
            job_id=job_id,
            headers=headers,
            body=_completion(status="failed", detail="this device runs robot plans only"),
        )
        return

    if generic is not None:
        registry, handle = generic
        if _stopping:
            with contextlib.suppress(Exception):
                registry.discard(handle)
            return
        try:
            _append_event(
                credentials,
                status="started",
                severity="info",
                reason_code="device_executor_started",
                job_id=job_id,
            )
        except BaseException:
            with contextlib.suppress(Exception):
                registry.discard(handle)
            raise
        if _stopping:
            with contextlib.suppress(Exception):
                registry.discard(handle)
            return
        try:
            result = registry.execute(handle)
        except Exception:
            result = {"status": "failed", "reason_code": "device_executor_execute_failed"}
        status = result.get("status")
        succeeded = status == "succeeded"
        reason = (
            "device_executor_succeeded"
            if succeeded
            else "device_executor_refused" if status == "refused" else "device_executor_failed"
        )
        evidence = result.get("evidence", []) if succeeded else []
        body: dict[str, Any] = {
            "status": "success" if succeeded else "failed",
            "variables": {"detail": reason},
        }
        if succeeded:
            body["variables"]["evidence"] = evidence
        else:
            body["error_message"] = reason
        _append_event(
            credentials,
            status="succeeded" if succeeded else "failed",
            severity="info" if succeeded else "error",
            reason_code=(
                "device_executor_succeeded"
                if succeeded
                else "device_executor_refused" if status == "refused" else "device_executor_failed"
            ),
            job_id=job_id,
            details={"job": {"reported_status": body["status"], "evidence_count": len(evidence)}},
        )
        _report_completion(credentials, job_id=job_id, headers=headers, body=body)
        return

    # The audit record that a robot was told to move, written before it is told.
    # If this raises, nothing below runs and the gateway is never called: an
    # unrecordable start is a refusal, not a warning. The poll loop treats the
    # exception as a failure and backs off, so the job stays claimable.
    _append_event(
        credentials,
        status="started",
        severity="info",
        reason_code="job_execution_started",
        job_id=job_id,
        correlation_id=trace_id,
    )

    logger.info("job %s -> plan %s", job_id, plan.get("plan_id"))
    try:
        outcome = _run_plan(plan, job_id)
    except Exception as exc:  # noqa: BLE001 - any failure is a failed job, reported
        logger.exception("job %s failed", job_id)
        # str(exc) reaches the Cloud completion body, which is a different
        # channel with a different contract. It must not reach the event: an
        # exception's text is whatever the failing library chose to put in it,
        # which has included URLs, tokens and file contents.
        outcome = {
            "status": "failed",
            "detail": str(exc)[:300],
            "reason_code": "gateway_unreachable",
        }

    body = _completion(
        status=outcome["status"],
        detail=outcome.get("detail"),
        pose=outcome.get("pose"),
        minimum_range=outcome.get("minimum_range"),
        execution_receipt=outcome.get("execution_receipt"),
    )
    succeeded = outcome["status"] == "succeeded"
    _append_event(
        credentials,
        status="succeeded" if succeeded else "failed",
        severity="info" if succeeded else "error",
        reason_code=str(
            outcome.get("reason_code") or ("job_completed" if succeeded else "mission_failed")
        ),
        job_id=job_id,
        correlation_id=trace_id,
        # Counts and a closed-vocabulary status. Never the pose, the plan, the
        # gateway's own text or the address it lives at.
        details={
            "job": {
                "reported_status": body["status"],
                "evidence_count": len(body.get("variables", {}).get("evidence", [])),
            }
        },
    )
    _report_completion(credentials, job_id=job_id, headers=headers, body=body)


def _report_completion(
    credentials: Mapping[str, str],
    *,
    job_id: str,
    headers: dict[str, str],
    body: dict[str, Any],
) -> None:
    """Send one completion, and record its failure to land. The only way out.

    Two different things can stop a completion, and they are recorded as two
    different reason codes because they have two different first moves:

    * **Refused** — the request reached Cloud and Cloud would not take it
      (``HTTPError``). Whoever owns the job upstream answers that.
    * **Unreachable** — the request never got an answer at all: DNS did not
      resolve, the connection was refused, the peer disconnected, or it timed
      out (``URLError``). Whoever owns the link answers that.

    An offline operator holding only this device's journal has to be able to
    tell those apart, so neither is allowed to surface as the other, and neither
    is allowed to surface as nothing.

    Every completion this runner sends goes through here — the unsupported-plan
    report as much as the one that follows a mission. A second call site that
    posted directly would be a path on which a 409, or a severed uplink,
    produced no event, and the whole point of the terminal record is that a
    report upstream did not take is still explainable from the device
    afterwards.
    """
    try:
        _post(CLOUD_URL, f"/api/devices/jobs/{job_id}/complete", body, headers)
    except urllib.error.HTTPError as exc:
        # A job that ran and could not be reported is the worst state to be
        # silent about: the robot moved, the schedule still thinks the step is
        # running, and nothing says why. Name it here rather than letting the
        # poll loop swallow it as one more backoff.
        #
        # The status line and nothing else. An error body is server text, and
        # server text has echoed back authorization headers and query strings
        # before now — into a log an operator then pastes into a ticket.
        logger.error(
            "job %s reported %s but the report was refused with HTTP %s",
            job_id,
            body.get("status"),
            exc.code,
        )
        try:
            _append_event(
                credentials,
                status="failed",
                severity="error",
                reason_code="completion_report_refused",
                job_id=job_id,
                details={"upstream": {"http_status": int(exc.code)}},
            )
        except Exception:  # noqa: BLE001 - the refusal itself must still surface
            logger.exception("job %s: the refusal could not be recorded either", job_id)
        raise
    except urllib.error.URLError:
        # Not a refusal: nothing upstream ever answered. HTTPError is a subclass
        # of URLError, so this must stay second — reordered, every 409 would be
        # recorded as an unreachable Cloud and the lease problem would never be
        # named.
        #
        # Unbound on purpose. There is nothing here this may read: URLError.reason
        # is an OS error or a TLS message carrying the host, the port and
        # sometimes the URL, which is the site's own Cloud address and network
        # identity — not something to write into a journal that leaves the robot,
        # or into a log an operator pastes into a ticket. The job id and the
        # status this device tried to report are the whole of what is said.
        logger.error(
            "job %s reported %s but the report could not be delivered",
            job_id,
            body.get("status"),
        )
        try:
            _append_event(
                credentials,
                status="failed",
                severity="error",
                reason_code="completion_report_unreachable",
                job_id=job_id,
                # No details at all. A refusal has a status code worth carrying;
                # an unreachable peer has only the exception's own text, and
                # there is no bounded, content-free field to distil it into.
            )
        except Exception as unrecordable:  # noqa: BLE001 - it must still surface
            # Surfaced the same way the refusal above is — named, bounded, and
            # not allowed to mask the raise below — but without a traceback. A
            # traceback prints the chained __context__, and here that context is
            # the URLError this branch deliberately never reads: its text would
            # arrive in the log by the back door. The class of what stopped the
            # write is a bounded name; its message is not, so only the name is
            # said.
            logger.error(
                "job %s: the undelivered report could not be recorded either (%s)",
                job_id,
                type(unrecordable).__name__,
            )
        # Re-raised unchanged. The poll loop treats it as a failure and backs
        # off; nothing re-runs the mission, which already moved a real robot.
        raise
    # Everything here comes from the body that was actually accepted. This line
    # used to read from the caller's `outcome`, which does not exist in this
    # function: every successfully reported job raised NameError *after* the
    # report had gone through, so the poll loop backed off and re-claimed a job
    # the cloud already considered done.
    variables = body.get("variables", {})
    logger.info(
        "job %s reported %s: %s | evidence: %s",
        job_id,
        body.get("status"),
        variables.get("detail"),
        [item["kind"] for item in variables.get("evidence", [])],
    )


def _completion(
    *,
    status: str,
    detail: Any = None,
    pose: Any = None,
    minimum_range: Any = None,
    execution_receipt: Mapping[str, Any] | None = None,
) -> dict[str, Any]:
    """The body /api/devices/jobs/{id}/complete actually accepts.

    The contract is ``status`` matching ``^(success|failed)$``, an optional
    ``error_message``, and ``variables`` — the dict a workflow's own output
    lands in. The first version of this runner sent ``status: "succeeded"``
    and a ``result`` field, both of which that model rejects or ignores; it
    had only ever been tested against a fake cloud that accepted anything.

    Evidence rides in ``variables["evidence"]`` because that is where the
    Space task sweep reads it from (``job_evidence`` in
    ``services/space_tasks/dispatch.py``). A finished mission reports only
    what it can honestly show: the pose odometry closed on
    (``arrival.pose``), and the nearest thing the lidar saw while it ran
    (``clearance.measurement``) — the reading that answers "is that passage
    walkable" where a camera frame cannot. Nothing is reported for a mission
    that did not finish: not knowing where the robot ended up must never be
    written down as an arrival.
    """
    succeeded = status == "succeeded"
    body: dict[str, Any] = {"status": "success" if succeeded else "failed"}
    variables: dict[str, Any] = {}
    if detail:
        variables["detail"] = str(detail)[:300]
        if not succeeded:
            body["error_message"] = str(detail)[:300]

    evidence: list[dict[str, Any]] = []
    if succeeded and pose is not None:
        evidence.append(
            {
                "kind": "arrival.pose",
                "usable": True,
                "detail": _short(pose),
            }
        )
    if succeeded and isinstance(minimum_range, (int, float)):
        evidence.append(
            {
                "kind": "clearance.measurement",
                "usable": True,
                "detail": f"nearest obstacle {float(minimum_range):.2f} m",
            }
        )
    if evidence:
        variables["evidence"] = evidence
    if execution_receipt is not None:
        # Separate from goal evidence by design. This receipt proves what the
        # actuator runtime observed; it may never make a Space task complete.
        variables["execution_receipt"] = dict(execution_receipt)
    if variables:
        body["variables"] = variables
    return body


def _short(value: Any) -> str:
    return value[:200] if isinstance(value, str) else json.dumps(value)[:200]


# -- the loop ------------------------------------------------------------


def _stop(signum, _frame) -> None:
    global _stopping
    _stopping = True
    logger.info("signal %s received; finishing the current job then stopping", signum)


def main() -> int:
    logging.basicConfig(
        level=os.getenv("FLYTO_LOG_LEVEL", "INFO"),
        format="%(asctime)s %(levelname)s %(message)s",
    )
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    try:
        credentials = _read_credentials() or _pair()
    except RunnerError as exc:
        logger.error("%s", exc)
        return 2

    headers = _headers(credentials)
    logger.info("device %s polling %s", credentials["device_id"], CLOUD_URL)

    failures = 0
    last_heartbeat = 0.0
    while not _stopping:
        try:
            now = time.monotonic()
            if now - last_heartbeat >= HEARTBEAT_INTERVAL_SECONDS:
                _post(
                    CLOUD_URL,
                    f"/api/devices/{credentials['device_id']}/heartbeat",
                    {},
                    headers,
                    timeout=15.0,
                )
                last_heartbeat = now

            body = _post(
                CLOUD_URL,
                f"/api/devices/jobs/poll?wait_seconds={POLL_WAIT_SECONDS}",
                {},
                headers,
                timeout=POLL_WAIT_SECONDS + 15,
            )
            failures = 0
            job = body.get("job") if isinstance(body, dict) else None
            if job:
                _handle(job, credentials)
            else:
                time.sleep(IDLE_DELAY_SECONDS)
        except urllib.error.HTTPError as exc:
            if exc.code in (401, 403):
                logger.error(
                    "credential rejected (%s); delete %s and pair again", exc.code, CREDENTIAL_FILE
                )
                return 3
            # Say which request failed and how. Backing off with only "retrying
            # in 6s" is how a job that finished on the robot but could not be
            # reported became an unreadable silence.
            #
            # The status line and the path, never the response body. A body is
            # server text: it has echoed back authorization headers, query
            # strings and stack traces, and this log is read by an operator who
            # will paste it into a ticket. The status code plus the path is what
            # actually identifies the failure.
            logger.warning("%s from %s", exc.code, _request_path(exc))
            failures += 1
            _back_off(failures)
        except Exception:  # noqa: BLE001 - the loop must survive a bad poll
            failures += 1
            logger.warning("poll failed", exc_info=True)
            _back_off(failures)

    with contextlib.suppress(Exception):
        _post(
            CLOUD_URL, f"/api/devices/{credentials['device_id']}/offline", {}, headers, timeout=10.0
        )
    logger.info("stopped")
    return 0


def _request_path(exc: urllib.error.HTTPError) -> str:
    """Which endpoint refused, with no host, no query string and no body.

    The path alone says what was being attempted. The host names the site's
    Cloud endpoint and a query string can carry a token, so neither is logged,
    and the body is never read at all — a body that is never read is a body that
    cannot be forwarded into a ticket by mistake.
    """
    url = str(getattr(exc, "url", "") or "")
    without_scheme = url.split("://", 1)[-1]
    path = without_scheme[without_scheme.find("/") :] if "/" in without_scheme else ""
    return path.split("?", 1)[0] or "an unnamed endpoint"


def _back_off(failures: int) -> None:
    delay = min(ERROR_MAX_DELAY_SECONDS, ERROR_BASE_DELAY_SECONDS * (2 ** min(failures, 5)))
    logger.info("retrying in %.0fs", delay)
    time.sleep(delay)


if __name__ == "__main__":
    sys.exit(main())
