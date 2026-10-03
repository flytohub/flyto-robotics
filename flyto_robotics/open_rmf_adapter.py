#!/usr/bin/env python3
"""A conformance adapter over an Open-RMF deployment.

The fleet layer, and the one place in this system where Flyto2 deliberately
stops deciding.

## The boundary, stated once

Flyto2 is the executive: it knows what the organisation can do and composes a
Goal out of capabilities. Open-RMF is the dispatch office: given a task, it runs
a bid across the fleets, weighs battery, position, traffic and current load, and
picks the machine. Neither one should do the other's job.

So this adapter **never names a robot**. Open-RMF's dispatcher exists to choose,
and a request that pinned `robot: "tinyRobot1"` would turn a bidding system into
a remote control and silently discard the traffic coordination that is the whole
reason to run it. What comes back is which fleet and which robot RMF chose, and
that is recorded as *evidence of a decision made elsewhere* rather than as a
decision Flyto2 made.

The direction that matters:

    Flyto2   "something that can navigate and carry, to ward 3"
    Open-RMF "fleet tinyRobot, robot tinyRobot2, bid won at cost 41.2"

## Why this is a script and not a dependency

`flyto-robotics` is the single-machine path — primitives, sensing gates, safety,
the frozen capability contract. Putting an Open-RMF dependency inside it would
make every single-robot deployment carry a fleet stack it never runs. This lives
outside the core exactly like the other adapters do, so a venue with one robot
installs nothing and a venue with four fleets adds this file's dependencies and
nothing else changes above it.

## What it maps onto

Open-RMF's task API takes a *category* and a description, not a capability id.
The translation is declared rather than inferred, so a capability nobody mapped
reaches a refusal here instead of a task request the dispatcher rejects with a
message no operator can act on.

    motion.navigate    -> patrol    one waypoint, no payload
    motion.navigate_to_waypoint     the same, as a fleet resource declares it
    transport.load     -> delivery  pickup half of an RMF delivery
    transport.unload   -> delivery  dropoff half
    motion.dock        -> patrol    to the charger's waypoint

`motion.advance` and its siblings are deliberately absent. "Forward three
metres" is a single-machine primitive with no fleet meaning: RMF plans between
named waypoints on a shared map, and a relative nudge cannot be traffic-managed
against other robots. Those stay on the single-machine path, which is the same
reason radar stayed off `vision.stream`.

Run it with::

    FLYTO_RMF_API_URL=http://rmf.local:8000 \\
    python scripts/run_adapter_conformance.py \\
        scripts.adapters.open_rmf_adapter:build
"""

from __future__ import annotations

import json
import os
import time
import urllib.request
from collections.abc import Callable, Iterable, Mapping, Sequence
from datetime import datetime, timezone
from typing import Any

from . import adapter_contract as decl
from .adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)

API_URL_ENV = "FLYTO_RMF_API_URL"
DEFAULT_API_URL = "http://127.0.0.1:8000"
DEPLOYMENT_MODE_ENV = "FLYTO_RMF_DEPLOYMENT_MODE"

# The ``flyto2.external_adapters`` entry-point name of this adapter, and the
# ``flyto.modules`` entry-point name of the flyto-modules-robotics pack whose
# steps drive it. A host joins a discovered fleet to that pack by this name.
ADAPTER_ID = "open_rmf.fleet"
MODULE_PACK = "fleet"
# Manifest fields emitted only when a host names them (see adapter_provider).
MANIFEST_EXTENSIONS: tuple[str, ...] = ("module_pack",)
FLEET_PREFIX = "fleet:"
# How often a dispatched task's state is read while a call waits on it.
POLL_SECONDS = 1.0

MAX_RESPONSE_BYTES = 512 * 1024
MAX_FLEETS = 32

# Flyto2 capability -> Open-RMF task category. Declared, never inferred: a
# capability nobody mapped must reach a refusal here rather than becoming a
# request the dispatcher rejects with a message no operator can act on.
CAPABILITY_TO_CATEGORY: Mapping[str, str] = {
    "motion.navigate": "patrol",
    "motion.navigate_to_waypoint": "patrol",
    "motion.dock": "patrol",
    "transport.load": "delivery",
    "transport.unload": "delivery",
}

# A fleet resource travels to a named waypoint, which is not the contract of a
# single robot's ``motion.navigate`` (a map coordinate). Two contracts under
# one capability id make a contract host treat it as ambiguous and fail
# closed, so the fleet resource declares its own id; ``motion.navigate`` is
# still accepted, and still declared by the unbound conformance build.
WAYPOINT_NAVIGATE = "motion.navigate_to_waypoint"
_UNBOUND_ONLY = frozenset({"motion.navigate"})
_FLEET_ONLY = frozenset({WAYPOINT_NAVIGATE})

# Deliberately unmapped, and why. Kept as data so the refusal can say which of
# the two reasons applies instead of answering "unknown" to both.
NOT_FLEET_WORK: Mapping[str, str] = {
    "motion.advance": "a relative nudge has no shared-map meaning to plan or "
    "traffic-manage; it belongs on the single-machine path",
    "motion.retreat": "same as motion.advance",
    "motion.rotate": "same as motion.advance",
    "motion.hop": "same as motion.advance",
    "motion.ascend": "same as motion.advance",
    "motion.halt": "a stop must reach the machine directly; routing it through "
    "a dispatcher's queue would put a bid between an operator and a brake",
}

# What Open-RMF's task state calls a finished task, and what each means here.
RMF_TERMINAL_SUCCESS = frozenset({"completed"})
RMF_TERMINAL_FAILURE = frozenset({"failed", "canceled", "killed"})


class RmfUnavailable(RuntimeError):
    """The dispatcher could not be reached or answered something unusable."""


def api_url() -> str:
    return (os.environ.get(API_URL_ENV) or DEFAULT_API_URL).rstrip("/")


def _request(path: str, *, payload: Any = None, opener=urllib.request.urlopen) -> Any:
    """One bounded call to the RMF API server."""
    body = None if payload is None else json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        f"{api_url()}{path}",
        data=body,
        method="POST" if body is not None else "GET",
        headers={"Content-Type": "application/json"} if body is not None else {},
    )
    with opener(request, timeout=15) as response:
        raw = response.read(MAX_RESPONSE_BYTES + 1)
    if len(raw) > MAX_RESPONSE_BYTES:
        raise RmfUnavailable("RMF response exceeded the size ceiling")
    return json.loads(raw.decode("utf-8"))


class OpenRmfAdapter:
    """The conformance methods over an Open-RMF dispatcher."""

    def __init__(
        self,
        *,
        call=None,
        requester: str = "flyto2",
        fleet: str | None = None,
        wait_for_completion: bool = False,
        poll_seconds: float = POLL_SECONDS,
        sleep: Callable[[float], None] = time.sleep,
        clock: Callable[[], float] = time.monotonic,
    ):
        self._call = call or _request
        self._requester = requester
        self.unmapped: list[tuple[str, str]] = []
        self._dispatched: dict[str, dict[str, Any]] = {}
        # Set when the adapter commands one fleet (the ``fleet:<name>``
        # resource a host built it for). The request then names that fleet,
        # never a robot: the fleet's own dispatcher still picks the machine.
        self.fleet = fleet
        self.resource_id = f"{FLEET_PREFIX}{fleet}" if fleet else ""
        # A capability host reads a call as done when it returns, so the
        # entry-point adapter waits for Open-RMF to finish the task. The
        # conformance build returns once the task is accepted, as it always has.
        self._wait = bool(wait_for_completion)
        self._poll_seconds = max(0.05, float(poll_seconds))
        self._sleep = sleep
        self._clock = clock
        self._results: dict[str, CallResult] = {}

    @property
    def deployment_mode(self) -> str:
        """``simulation`` for an Open-RMF demo world; anything else is physical."""
        mode = (os.environ.get(DEPLOYMENT_MODE_ENV) or "physical").strip().lower()
        return "simulation" if mode == "simulation" else "physical"

    # -- describe ------------------------------------------------------------

    def describe(self) -> Sequence[decl.CapabilityDeclaration]:
        """What the fleets can do, as declarations nobody approved yet.

        Read from ``GET /fleets`` rather than assumed, so a deployment with no
        delivery-capable fleet does not advertise ``transport.load``.

        Every declaration carries ``executor_kind = open-rmf`` and no resource
        id. That absence is the point: the capability belongs to the *fleet*,
        and which robot provides it is not decided until the dispatcher runs a
        bid. A declaration naming a robot here would be Flyto2 claiming a choice
        it does not make.
        """
        payload = self._call("/fleets")
        fleets = list(payload if isinstance(payload, list) else payload.get("data") or ())
        out: list[decl.CapabilityDeclaration] = []
        seen: set[tuple[str, str]] = set()
        for fleet in fleets[:MAX_FLEETS]:
            if not isinstance(fleet, Mapping):
                continue
            name = str(fleet.get("name") or "").strip()
            if not name:
                self.unmapped.append(("", "a fleet with no name"))
                continue
            if self.fleet is not None and name != self.fleet:
                continue
            hidden = _UNBOUND_ONLY if self.fleet is not None else _FLEET_ONLY
            for category in self._categories(fleet):
                for capability_id, mapped in CAPABILITY_TO_CATEGORY.items():
                    if mapped != category or (name, capability_id) in seen:
                        continue
                    if capability_id in hidden:
                        continue
                    seen.add((name, capability_id))
                    try:
                        out.append(
                            decl.declare(
                                capability_id=capability_id,
                                # The fleet, not a robot. Open-RMF picks the
                                # machine at dispatch time and this adapter must
                                # not pre-empt that.
                                resource_id=f"fleet:{name}",
                                executor_kind=decl.EXECUTOR_OPEN_RMF,
                                source=decl.SOURCE_DEVICE,
                                runtime_name=f"open-rmf:{category}",
                            )
                        )
                    except decl.DeclarationRefused as error:
                        self.unmapped.append((capability_id, str(error)))
        return out

    @staticmethod
    def _categories(fleet: Mapping[str, Any]) -> tuple[str, ...]:
        """Which task categories a fleet says it accepts.

        RMF reports this under ``task_types`` on older builds and
        ``capabilities`` on newer ones. Both are read; neither is guessed at,
        and a fleet reporting neither contributes nothing rather than being
        assumed to do everything.
        """
        raw = fleet.get("task_types") or fleet.get("capabilities") or ()
        return tuple(str(item).strip().lower() for item in raw if str(item).strip())

    # -- invoke --------------------------------------------------------------

    def invoke(self, request: CallRequest) -> CallResult:
        """Hand one capability to the dispatcher and let it choose the machine.

        The request carries a category and a description. It does not carry a
        robot, and that omission is deliberate rather than unfinished: naming
        one turns a bidding system into a remote control and discards the
        traffic coordination this layer exists for.
        """
        capability_id = str(request.capability_id)
        if capability_id in NOT_FLEET_WORK:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"{capability_id} is not fleet work: {NOT_FLEET_WORK[capability_id]}",
            )
        category = CAPABILITY_TO_CATEGORY.get(capability_id)
        if category is None:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=(
                    f"{capability_id} has no declared Open-RMF category; add one "
                    "rather than letting the dispatcher refuse an unreadable request"
                ),
            )

        kept = self._results.get(request.call_id)
        if kept is not None:
            # The same call again: its result, never a second task.
            return kept
        if request.call_id in self._dispatched:
            # Dispatched before and still running (the last wait timed out):
            # wait on that task rather than dispatching another one.
            return self._settle(request, self._dispatched[request.call_id])

        description = self._description(category, request)
        if description is None:
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=(
                    f"{category} needs a named waypoint on the shared map and the "
                    "call supplied none; RMF plans between places, not distances"
                ),
            )

        task_request: dict[str, Any] = {
            "category": category,
            "description": description,
            "requester": self._requester,
            # Left to RMF. Stamping a start time here would make
            # this process's clock the fleet's schedule.
            "unix_millis_earliest_start_time": 0,
        }
        if self.fleet:
            # The commanded resource is the fleet. Its dispatcher still picks
            # the robot; this only keeps the task inside the fleet asked for.
            task_request["fleet_name"] = self.fleet
        try:
            answer = self._call(
                "/tasks/dispatch_task",
                payload={"type": "dispatch_task_request", "request": task_request},
            )
        except Exception as error:  # noqa: BLE001 - reported, never swallowed
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"the dispatcher could not be reached: {type(error).__name__}",
            )

        state = answer.get("state") if isinstance(answer, Mapping) else None
        if not isinstance(state, Mapping):
            errors = answer.get("errors") if isinstance(answer, Mapping) else None
            return CallResult(
                request.call_id,
                OUTCOME_REFUSED,
                detail=f"the dispatcher refused the request: {errors or 'no state returned'}",
            )

        booking = state.get("booking") or {}
        rmf_id = str(booking.get("id") or "")
        self._dispatched[request.call_id] = {"rmf_task_id": rmf_id}
        evidence = _task_evidence(rmf_id, state)
        if not self._wait:
            return CallResult(request.call_id, OUTCOME_COMPLETED, evidence=evidence)
        return self._settle(request, {"rmf_task_id": rmf_id}, evidence)

    def _settle(
        self,
        request: CallRequest,
        record: Mapping[str, Any],
        evidence: Mapping[str, Any] | None = None,
    ) -> CallResult:
        """Wait, up to the call's deadline, for Open-RMF to finish the task.

        ``completed`` is RMF's word for a finished task; ``failed``,
        ``canceled`` and ``killed`` end the call as failed. A task still
        running at the deadline is a timeout, and the host's cancel then
        withdraws it by the id RMF gave it.
        """
        rmf_id = str(record.get("rmf_task_id") or "")
        latest = dict(evidence or {"rmf_task_id": rmf_id, "chosen_by": "open-rmf.dispatcher"})
        deadline = self._clock() + max(0.0, float(request.deadline_seconds))
        while True:
            try:
                answer = self._call(f"/tasks/{rmf_id}/state")
            except Exception:  # noqa: BLE001 - a missed read is retried until the deadline
                answer = None
            if isinstance(answer, Mapping):
                latest = {**latest, **_task_evidence(rmf_id, answer, previous=latest)}
                status = latest["rmf_status"].lower()
                if status in RMF_TERMINAL_SUCCESS:
                    items = _arrival_items(request, latest)
                    if items:
                        latest["evidence_items"] = items
                    return self._keep(
                        CallResult(request.call_id, OUTCOME_COMPLETED, evidence=latest)
                    )
                if status in RMF_TERMINAL_FAILURE:
                    return self._keep(
                        CallResult(
                            request.call_id,
                            OUTCOME_FAILED,
                            evidence=latest,
                            detail=f"Open-RMF reports task {rmf_id} {status}",
                        )
                    )
            if self._clock() >= deadline:
                return CallResult(
                    request.call_id,
                    OUTCOME_TIMEOUT,
                    evidence=latest,
                    detail=(
                        f"Open-RMF task {rmf_id} was still "
                        f"{latest.get('rmf_status') or 'unreported'} at the deadline"
                    ),
                )
            self._sleep(self._poll_seconds)

    def _keep(self, result: CallResult) -> CallResult:
        self._results[result.call_id] = result
        while len(self._results) > 256:
            self._results.pop(next(iter(self._results)))
        return result

    @staticmethod
    def _description(category: str, request: CallRequest) -> dict[str, Any] | None:
        """The category-shaped body RMF expects, or ``None`` if it cannot be built.

        RMF plans between named waypoints on a shared map. A call that supplies
        a distance instead of a place cannot be turned into a request, and
        inventing a waypoint would dispatch a robot somewhere nobody asked for.
        """
        arguments = dict(getattr(request, "arguments", None) or {})
        waypoint = str(arguments.get("waypoint") or arguments.get("destination") or "")
        if not waypoint:
            return None
        if category == "patrol":
            return {"places": [waypoint], "rounds": 1}
        return {
            "pickup": {"place": waypoint, "handler": waypoint, "payload": []},
            "dropoff": {"place": waypoint, "handler": waypoint, "payload": []},
        }

    # -- cancel --------------------------------------------------------------

    def cancel(self, call_id: str) -> CallResult:
        """Withdraw a dispatched task, by the id RMF gave it.

        A call this adapter never dispatched is refused rather than reported
        cancelled: answering "cancelled" for something that was never running
        would let a caller believe a machine was stopped when nothing was ever
        told to move.
        """
        record = self._dispatched.get(call_id)
        if record is None:
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail="this adapter never dispatched that call",
            )
        try:
            self._call(
                "/tasks/cancel_task",
                payload={"type": "cancel_task_request", "task_id": record["rmf_task_id"]},
            )
        except Exception as error:  # noqa: BLE001
            return CallResult(
                call_id,
                OUTCOME_REFUSED,
                detail=f"the dispatcher could not be reached: {type(error).__name__}",
            )
        return CallResult(call_id, OUTCOME_COMPLETED, evidence=dict(record))

    # -- safe stop -----------------------------------------------------------

    def safe_stop(self) -> CallResult:
        """Refused, and this is the most important refusal in the file.

        Open-RMF has no fleet-wide emergency stop, and a stop routed through a
        dispatcher's task queue is a stop that waits behind a bid. Reporting
        ``completed`` here would tell an operator every machine was at rest when
        nothing had been asked to brake.

        A real stop reaches each machine on the single-machine path, which is
        why ``motion.halt`` is in ``NOT_FLEET_WORK``. The conformance kit failing
        this check is the correct finding about this deployment, not a defect in
        this adapter.
        """
        return CallResult(
            "safe-stop",
            OUTCOME_REFUSED,
            detail=(
                "Open-RMF has no fleet-wide stop; a stop queued behind a "
                "dispatch bid is not a stop. Halt each machine directly."
            ),
        )

    # -- state ---------------------------------------------------------------

    def task_state(self, call_id: str) -> str:
        """What RMF says about a dispatched task, in RMF's own words.

        Not translated into Flyto2's task states. The fleet layer's ``completed``
        means the fleet finished its job, which is exactly the claim the
        evidence layer exists to stop being read as "the mission is done".
        """
        record = self._dispatched.get(call_id)
        if record is None:
            return "unknown"
        answer = self._call(f"/tasks/{record['rmf_task_id']}/state")
        if not isinstance(answer, Mapping):
            return "unknown"
        return str(answer.get("status") or "unknown")


def _task_evidence(
    rmf_id: str, state: Mapping[str, Any], *, previous: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    """What RMF reported about a task, in RMF's words.

    Who the dispatcher picked is recorded as evidence of a decision made
    elsewhere: this is the answer to "why this robot", and the answer is
    "Open-RMF's bid said so", not "Flyto2 chose".
    """
    assigned = state.get("assigned_to") if isinstance(state.get("assigned_to"), Mapping) else {}
    earlier = previous or {}
    return {
        "rmf_task_id": rmf_id,
        "rmf_status": str(state.get("status") or earlier.get("rmf_status") or ""),
        "assigned_fleet": str(assigned.get("group") or earlier.get("assigned_fleet") or ""),
        "assigned_robot": str(assigned.get("name") or earlier.get("assigned_robot") or ""),
        "chosen_by": "open-rmf.dispatcher",
    }


# Capabilities whose finished task means a machine arrived somewhere.
_ARRIVALS = frozenset({"motion.navigate", WAYPOINT_NAVIGATE, "motion.dock"})


def _arrival_items(request: CallRequest, evidence: Mapping[str, Any]) -> list[dict[str, Any]]:
    """The arrival a finished fleet task proves, in the host's evidence shape.

    Usable because RMF reported the task completed: that is the fleet's own
    claim, recorded with who made it. A load or unload proves no arrival.
    """
    if request.capability_id not in _ARRIVALS:
        return []
    arguments = dict(getattr(request, "arguments", None) or {})
    waypoint = str(arguments.get("waypoint") or arguments.get("destination") or "")
    robot = evidence.get("assigned_robot") or "a robot"
    fleet = evidence.get("assigned_fleet") or "its fleet"
    return [
        {
            "kind": "robot.arrival",
            "usable": True,
            "detail": (
                f"Open-RMF reports task {evidence.get('rmf_task_id')} completed: "
                f"{robot} of {fleet} at {waypoint}"
            )[:200],
            "reason": {
                "code": "rmf_task_completed",
                "observed": {
                    "rmf_task_id": evidence.get("rmf_task_id"),
                    "rmf_status": evidence.get("rmf_status"),
                    "assigned_fleet": evidence.get("assigned_fleet"),
                    "assigned_robot": evidence.get("assigned_robot"),
                    "waypoint": waypoint,
                },
                "required": {"rmf_status": "completed"},
                "source": "open-rmf.task-state",
            },
        }
    ]


def build() -> OpenRmfAdapter:
    return OpenRmfAdapter()


def fleet_name(resource_id: str) -> str:
    """The fleet a ``fleet:<name>`` resource id names, or ValueError."""
    text = str(resource_id or "").strip()
    name = text[len(FLEET_PREFIX) :].strip() if text.startswith(FLEET_PREFIX) else ""
    if not name or len(name) > 120:
        raise ValueError(
            f"an Open-RMF resource is a fleet, named {FLEET_PREFIX}<fleet name>; got {text!r}"
        )
    return name


def build_adapter(resource_id: str) -> OpenRmfAdapter:
    """The ``flyto2.external_adapters`` factory: one fleet, calls that finish.

    A host builds it for the commanded resource, which is a fleet. Each call
    returns once Open-RMF reports the task finished, failed, or still running
    at the call's deadline.
    """
    return OpenRmfAdapter(fleet=fleet_name(resource_id), wait_for_completion=True)


def _discovery_enabled() -> bool:
    # Only an explicitly configured dispatcher is discovered: the default URL
    # is for the conformance kit, not a claim that a fleet exists here.
    return bool((os.environ.get(API_URL_ENV) or "").strip())


def discover_fleet_manifests(
    *,
    execution_host_id: str,
    manifest_extensions: Iterable[str] | None = None,
    adapter: OpenRmfAdapter | None = None,
) -> list[dict[str, Any]]:
    """One resource manifest per fleet the configured dispatcher reports.

    Discovery grants nothing: every capability arrives DISCOVERED, and the
    resource is the fleet, never a robot. Best effort, like the ROS 2
    discoverer: a dispatcher that cannot be reached is not a statement that
    no fleet exists.
    """
    _ = execution_host_id
    if adapter is None and not _discovery_enabled():
        return []
    # Bound to no fleet yet, but declaring what a fleet resource offers.
    reader = adapter or OpenRmfAdapter(fleet=None)
    reader_hides = _UNBOUND_ONLY
    try:
        declarations = tuple(reader.describe())
    except Exception:  # noqa: BLE001 - see docstring
        return []
    requested = (
        frozenset()
        if manifest_extensions is None or isinstance(manifest_extensions, (str, bytes))
        else frozenset(str(item) for item in manifest_extensions)
    ) & frozenset(MANIFEST_EXTENSIONS)
    by_fleet: dict[str, list[decl.CapabilityDeclaration]] = {}
    for item in declarations:
        if item.capability_id in reader_hides:
            # A fleet resource offers the waypoint navigation, not a robot's.
            item = decl.declare(
                capability_id=WAYPOINT_NAVIGATE,
                resource_id=item.resource_id,
                executor_kind=item.executor_kind,
                source=item.source,
                runtime_name=item.runtime_name,
            )
        by_fleet.setdefault(item.resource_id, []).append(item)
    manifests = []
    for resource_id, items in sorted(by_fleet.items()):
        contracts = []
        for item in sorted(items, key=lambda value: value.capability_id):
            metadata = decl.capability_metadata(item.capability_id)
            contracts.append(
                {
                    "capability_id": item.capability_id,
                    "display_name": str(metadata.get("display_name") or item.capability_id),
                    "description": str(metadata.get("description") or "")[:1000],
                    "input_schema": _WAYPOINT_SCHEMA,
                    "safety_class": item.safety_class,
                    "required_permissions": list(item.required_permissions),
                    # The fleet has no stop of its own (see safe_stop).
                    "requires_safe_stop": False,
                }
            )
        manifest: dict[str, Any] = {
            "contract": "flyto.resource-manifest.v1",
            "resource_id": resource_id,
            "resource_type": "fleet",
            "display_name": f"Open-RMF fleet {resource_id[len(FLEET_PREFIX):]}"[:200],
            "revision": 1,
            "adapter": {
                "adapter_id": ADAPTER_ID,
                "version": "1.0.0",
                "provider": "Flyto2 Robotics",
            },
            "deployment_mode": "simulation" if reader.deployment_mode == "simulation" else "real",
            "capability_ids": [item["capability_id"] for item in contracts],
            "capability_contracts": contracts,
            "settings": [],
            "telemetry_channels": [],
            "observed_at": datetime.now(timezone.utc).isoformat(),
            "contract_hash": "",
        }
        if "module_pack" in requested:
            manifest["module_pack"] = MODULE_PACK
        manifests.append(manifest)
    return manifests


discover_fleet_manifests.manifest_extensions = MANIFEST_EXTENSIONS  # type: ignore[attr-defined]

# RMF plans between named places: every fleet capability takes one waypoint.
_WAYPOINT_SCHEMA: Mapping[str, Any] = {
    "type": "object",
    "properties": {
        "waypoint": {
            "type": "string",
            "minLength": 1,
            "maxLength": 128,
            "description": "A named waypoint on the fleet's shared map",
        }
    },
    "required": ["waypoint"],
    "additionalProperties": False,
}


if __name__ == "__main__":
    adapter = build()
    print(f"dispatcher: {api_url()}")
    try:
        declarations = adapter.describe()
    except Exception as error:  # noqa: BLE001
        print(f"  could not read fleets: {error}")
        raise SystemExit(1) from error
    for item in declarations:
        print(f"  {item.resource_id:<24} {item.capability_id:<20} {item.approval_status}")
    for identifier, why in adapter.unmapped:
        print(f"  UNMAPPED {identifier or '(no id)'}: {why}")
    print(f"\n{len(declarations)} fleet capability declaration(s).")
    print("None are usable until somebody approves them, and no robot is named:")
    print("Open-RMF picks the machine when the task is dispatched.")
