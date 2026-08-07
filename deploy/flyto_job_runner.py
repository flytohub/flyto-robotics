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

import json
import logging
import os
import signal
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any

logger = logging.getLogger("flyto.job_runner")

CLOUD_URL = os.getenv("FLYTO_CLOUD_URL", "https://api.flyto2.com").rstrip("/")
GATEWAY_URL = os.getenv("FLYTO_ROBOTICS_GATEWAY_URL", "http://127.0.0.1:8766").rstrip("/")
DATA_DIR = Path(os.getenv("FLYTO_RUNNER_DATA_DIR", "/home/ubuntu/.flyto"))
CREDENTIAL_FILE = DATA_DIR / "runner-credentials.json"

POLL_WAIT_SECONDS = 25
HEARTBEAT_INTERVAL_SECONDS = 30.0
IDLE_DELAY_SECONDS = 1.0
ERROR_BASE_DELAY_SECONDS = 3.0
ERROR_MAX_DELAY_SECONDS = 60.0
# A mission is bounded by the gateway's own job timeout; this only decides how
# long to keep watching one before reporting that the outcome is unknown.
MISSION_WATCH_SECONDS = 300.0
GATEWAY_POLL_SECONDS = 1.0

TERMINAL_STATES = frozenset({"succeeded", "failed", "cancelled", "aborted"})

_stopping = False


class RunnerError(RuntimeError):
    """The runner cannot continue without an operator."""


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


def _post(base: str, path: str, payload: Any, headers: dict[str, str], timeout: float = 35.0) -> Any:
    return _call(base, path, payload=payload if payload is not None else {}, headers=headers, timeout=timeout)


# -- credentials ---------------------------------------------------------


def _read_credentials() -> dict[str, str] | None:
    if not CREDENTIAL_FILE.exists():
        return None
    data = json.loads(CREDENTIAL_FILE.read_text())
    if not data.get("device_id") or not data.get("device_secret"):
        raise RunnerError(f"{CREDENTIAL_FILE} is malformed; delete it and pair again")
    return data


def _write_credentials(data: dict[str, str]) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    CREDENTIAL_FILE.write_text(json.dumps(data))
    CREDENTIAL_FILE.chmod(0o600)


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


def _headers(credentials: dict[str, str]) -> dict[str, str]:
    return {
        "Authorization": (
            f"Bearer device:{credentials['device_id']}.{credentials['device_secret']}"
        )
    }


# -- executing one job ---------------------------------------------------


def _plan_from(job: dict[str, Any]) -> dict[str, Any] | None:
    """The robot plan this job carries, or None if it is not a robot job.

    Two shapes are accepted: a step whose params hold a plan outright, and a
    step naming a plan file already on this robot. Anything else is not
    something this runner can carry out, and saying so is better than guessing.
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
            root = Path(os.getenv("FLYTO_PLAN_ROOT", "/home/ubuntu/flyto-robotics/examples/plans")).resolve()
            resolved = (root / candidate.name).resolve()
            if resolved.parent == root and resolved.is_file():
                return json.loads(resolved.read_text())
    return None


def _run_plan(plan: dict[str, Any], job_id: str) -> dict[str, Any]:
    """Hand the plan to the gateway and watch the session to an outcome."""
    token = os.getenv("FLYTO_ROBOTICS_DELIVERY_TOKEN", "").strip()
    if not token:
        raise RunnerError("FLYTO_ROBOTICS_DELIVERY_TOKEN is not set")
    gateway_headers = {"Authorization": f"Bearer {token}"}

    session = _post(
        GATEWAY_URL,
        "/v1/plans",
        {
            "contract_version": "flyto.cloud.plan-run-request.v1",
            "request_id": f"job-{job_id}"[:128],
            "plan": plan,
            "requested_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        },
        headers=gateway_headers,
        timeout=15.0,
    )
    session_id = str(session.get("session_id") or "")
    if not session_id:
        return {"status": "failed", "detail": "gateway returned no session"}

    deadline = time.monotonic() + MISSION_WATCH_SECONDS
    latest = session
    while time.monotonic() < deadline and not _stopping:
        state = str(latest.get("state") or latest.get("status") or "").lower()
        if state in TERMINAL_STATES:
            return {
                "status": "succeeded" if state == "succeeded" else "failed",
                "detail": str(latest.get("reason") or state)[:300],
                "pose": latest.get("final_pose"),
            }
        time.sleep(GATEWAY_POLL_SECONDS)
        latest = _call(GATEWAY_URL, f"/v1/deliveries/{session_id}", headers=gateway_headers, timeout=15.0)

    # Not knowing an outcome is not the same as the mission having failed, and
    # the gateway still owns the robot. Report it as what it is.
    return {"status": "failed", "detail": f"outcome unknown after {MISSION_WATCH_SECONDS:.0f}s"}


def _handle(job: dict[str, Any], credentials: dict[str, str]) -> None:
    job_id = str(job.get("job_id") or job.get("id") or "")
    if not job_id:
        return
    headers = _headers(credentials)

    claimed = _post(CLOUD_URL, f"/api/devices/jobs/{job_id}/claim", {}, headers)
    lease = claimed.get("lease_id")
    if lease:
        headers = {**headers, "X-Flyto-Lease": str(lease)}

    plan = _plan_from(job)
    if plan is None:
        logger.warning("job %s carries no robot plan", job_id)
        _post(
            CLOUD_URL,
            f"/api/devices/jobs/{job_id}/complete",
            _completion(status="failed", detail="this device runs robot plans only"),
            headers,
        )
        return

    logger.info("job %s -> plan %s", job_id, plan.get("plan_id"))
    try:
        outcome = _run_plan(plan, job_id)
    except Exception as exc:  # noqa: BLE001 - any failure is a failed job, reported
        logger.exception("job %s failed", job_id)
        outcome = {"status": "failed", "detail": str(exc)[:300]}

    _post(
        CLOUD_URL,
        f"/api/devices/jobs/{job_id}/complete",
        _completion(
            status=outcome["status"],
            detail=outcome.get("detail"),
            pose=outcome.get("pose"),
        ),
        headers,
    )
    logger.info("job %s reported %s: %s", job_id, outcome["status"], outcome.get("detail"))


def _completion(*, status: str, detail: Any = None, pose: Any = None) -> dict[str, Any]:
    """The body /api/devices/jobs/{id}/complete actually accepts.

    The contract is ``status`` matching ``^(success|failed)$``, an optional
    ``error_message``, and ``variables`` — the dict a workflow's own output
    lands in. The first version of this runner sent ``status: "succeeded"``
    and a ``result`` field, both of which that model rejects or ignores; it
    had only ever been tested against a fake cloud that accepted anything.

    Evidence rides in ``variables["evidence"]`` because that is where the
    Space task sweep reads it from (``job_evidence`` in
    ``services/space_tasks/dispatch.py``). A completed motion plan reports
    ``arrival.pose`` — the one thing odometry-closed motion honestly proves —
    so the mission's evidence loop hears about it without any new plumbing.
    """
    succeeded = status == "succeeded"
    body: dict[str, Any] = {"status": "success" if succeeded else "failed"}
    variables: dict[str, Any] = {}
    if detail:
        variables["detail"] = str(detail)[:300]
        if not succeeded:
            body["error_message"] = str(detail)[:300]
    if succeeded and pose is not None:
        variables["evidence"] = [
            {
                "kind": "arrival.pose",
                "usable": True,
                "detail": json.dumps(pose)[:200] if not isinstance(pose, str) else pose[:200],
            }
        ]
    if variables:
        body["variables"] = variables
    return body


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
                _post(CLOUD_URL, f"/api/devices/{credentials['device_id']}/heartbeat", {}, headers, timeout=15.0)
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
                logger.error("credential rejected (%s); delete %s and pair again", exc.code, CREDENTIAL_FILE)
                return 3
            failures += 1
            _back_off(failures)
        except Exception:  # noqa: BLE001 - the loop must survive a bad poll
            failures += 1
            logger.warning("poll failed", exc_info=True)
            _back_off(failures)

    try:
        _post(CLOUD_URL, f"/api/devices/{credentials['device_id']}/offline", {}, headers, timeout=10.0)
    except Exception:  # noqa: BLE001 - going offline politely is best effort
        pass
    logger.info("stopped")
    return 0


def _back_off(failures: int) -> None:
    delay = min(ERROR_MAX_DELAY_SECONDS, ERROR_BASE_DELAY_SECONDS * (2 ** min(failures, 5)))
    logger.info("retrying in %.0fs", delay)
    time.sleep(delay)


if __name__ == "__main__":
    sys.exit(main())
