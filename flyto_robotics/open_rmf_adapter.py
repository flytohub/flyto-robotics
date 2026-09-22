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
import urllib.request
from collections.abc import Mapping, Sequence
from typing import Any

from . import adapter_contract as decl
from .adapter_contract import (
    OUTCOME_COMPLETED,
    OUTCOME_REFUSED,
    CallRequest,
    CallResult,
)

API_URL_ENV = "FLYTO_RMF_API_URL"
DEFAULT_API_URL = "http://127.0.0.1:8000"

MAX_RESPONSE_BYTES = 512 * 1024
MAX_FLEETS = 32

# Flyto2 capability -> Open-RMF task category. Declared, never inferred: a
# capability nobody mapped must reach a refusal here rather than becoming a
# request the dispatcher rejects with a message no operator can act on.
CAPABILITY_TO_CATEGORY: Mapping[str, str] = {
    "motion.navigate": "patrol",
    "motion.dock": "patrol",
    "transport.load": "delivery",
    "transport.unload": "delivery",
}

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

    def __init__(self, *, call=None, requester: str = "flyto2"):
        self._call = call or _request
        self._requester = requester
        self.unmapped: list[tuple[str, str]] = []
        self._dispatched: dict[str, dict[str, Any]] = {}

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
            for category in self._categories(fleet):
                for capability_id, mapped in CAPABILITY_TO_CATEGORY.items():
                    if mapped != category or (name, capability_id) in seen:
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

        try:
            answer = self._call(
                "/tasks/dispatch_task",
                payload={
                    "type": "dispatch_task_request",
                    "request": {
                        "category": category,
                        "description": description,
                        "requester": self._requester,
                        # Left to RMF. Stamping a start time here would make
                        # this process's clock the fleet's schedule.
                        "unix_millis_earliest_start_time": 0,
                    },
                },
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
        assigned = state.get("assigned_to") or {}
        rmf_id = str(booking.get("id") or "")
        self._dispatched[request.call_id] = {"rmf_task_id": rmf_id}
        return CallResult(
            request.call_id,
            OUTCOME_COMPLETED,
            evidence={
                "rmf_task_id": rmf_id,
                "rmf_status": str(state.get("status") or ""),
                # Who the dispatcher picked. Recorded as evidence of a decision
                # made elsewhere — this is the answer to "why this robot", and
                # the answer is "Open-RMF's bid said so", not "Flyto2 chose".
                "assigned_fleet": str(assigned.get("group") or ""),
                "assigned_robot": str(assigned.get("name") or ""),
                "chosen_by": "open-rmf.dispatcher",
            },
        )

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


def build() -> OpenRmfAdapter:
    return OpenRmfAdapter()


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
