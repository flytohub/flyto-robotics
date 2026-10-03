"""The fleet layer, and the one place Flyto2 deliberately stops deciding.

Flyto2 is the executive: it composes a Goal out of capabilities. Open-RMF is the
dispatch office: it runs a bid across fleets and picks the machine. The tests
that matter here are the ones holding that line — most of a bad fleet adapter's
damage comes from quietly doing the dispatcher's job.
"""

from __future__ import annotations

import pytest

from flyto_robotics import adapter_contract as decl
from flyto_robotics.adapter_contract import (
    KNOWN_CAPABILITY_IDS,
    OUTCOME_COMPLETED,
    OUTCOME_FAILED,
    OUTCOME_TIMEOUT,
    CallRequest,
)
from flyto_robotics.open_rmf_adapter import (
    ADAPTER_ID,
    CAPABILITY_TO_CATEGORY,
    MODULE_PACK,
    NOT_FLEET_WORK,
    OpenRmfAdapter,
    build_adapter,
    discover_fleet_manifests,
    fleet_name,
)


def adapter(*, fleets=None, dispatch=None, sent=None, fail=None):
    fleets = fleets if fleets is not None else [
        {"name": "tinyRobot", "task_types": ["patrol", "delivery"]}
    ]
    dispatch = dispatch if dispatch is not None else {
        "state": {
            "booking": {"id": "rmf-task-1"},
            "status": "queued",
            "assigned_to": {"group": "tinyRobot", "name": "tinyRobot2"},
        }
    }

    def call(path, payload=None, **_):
        if sent is not None:
            sent.append((path, payload))
        if fail is not None:
            raise fail
        if path == "/fleets":
            return fleets
        if path == "/tasks/dispatch_task":
            return dispatch
        if path == "/tasks/cancel_task":
            return {"success": True}
        if path.endswith("/state"):
            return {"status": "completed"}
        raise AssertionError(f"unexpected path {path}")

    return OpenRmfAdapter(call=call)


def navigate(**arguments):
    return CallRequest(
        "call-1", "motion.navigate", arguments or {"waypoint": "ward_3"}
    )


# -- the boundary ------------------------------------------------------------


def test_no_request_ever_names_a_robot():
    """The load-bearing test. Pinning a robot turns a bidding system into a
    remote control and discards the traffic coordination this layer exists for."""
    sent: list = []
    adapter(sent=sent).invoke(navigate())
    _path, payload = sent[-1]
    body = str(payload)
    assert "tinyRobot2" not in body
    assert "robot" not in payload["request"]


def test_a_declaration_names_the_fleet_not_a_machine():
    """Which robot provides the capability is not decided until the dispatcher
    runs a bid, so a declaration naming one would claim a choice Flyto2 does not
    make."""
    declared = adapter().describe()
    assert declared
    assert all(item.resource_id.startswith("fleet:") for item in declared)


def test_the_executor_kind_is_open_rmf():
    assert adapter().describe()[0].executor_kind == decl.EXECUTOR_OPEN_RMF


def test_who_the_dispatcher_chose_is_recorded_as_evidence():
    """The answer to "why this robot" is "Open-RMF's bid said so", and the
    record has to say that rather than implying Flyto2 chose."""
    result = adapter().invoke(navigate())
    assert result.evidence["assigned_robot"] == "tinyRobot2"
    assert result.evidence["assigned_fleet"] == "tinyRobot"
    assert result.evidence["chosen_by"] == "open-rmf.dispatcher"


def test_fleet_state_is_reported_in_rmf_s_own_words():
    """The fleet's `completed` means the fleet finished its job, which is the
    claim the evidence layer exists to stop being read as mission completion."""
    device = adapter()
    device.invoke(navigate())
    assert device.task_state("call-1") == "completed"


# -- what is deliberately not fleet work -------------------------------------


def test_a_relative_nudge_is_refused_with_the_reason():
    """RMF plans between named waypoints on a shared map; a relative move
    cannot be traffic-managed against other robots."""
    result = adapter().invoke(CallRequest("c", "motion.advance", {"distance_m": 3}))
    assert result.outcome == "refused"
    assert "single-machine path" in result.detail


def test_a_stop_is_never_routed_through_the_dispatcher():
    """A stop queued behind a bid is not a stop."""
    result = adapter().invoke(CallRequest("c", "motion.halt", {}))
    assert result.outcome == "refused"
    assert "brake" in result.detail


def test_safe_stop_refuses_rather_than_claiming_every_machine_is_at_rest():
    """Reporting completed here would tell an operator the fleet was stopped
    when nothing had been asked to brake. The kit failing this check is the
    correct finding about the deployment."""
    result = adapter().safe_stop()
    assert result.outcome == "refused"
    assert "no fleet-wide stop" in result.detail


def test_every_not_fleet_work_entry_is_a_real_capability():
    """A refusal naming a capability the vocabulary does not have would be
    dead code pretending to be a policy."""
    assert set(NOT_FLEET_WORK) <= KNOWN_CAPABILITY_IDS


def test_every_mapped_capability_is_a_real_capability():
    assert set(CAPABILITY_TO_CATEGORY) <= KNOWN_CAPABILITY_IDS


def test_nothing_is_both_mapped_and_declared_not_fleet_work():
    assert not (set(CAPABILITY_TO_CATEGORY) & set(NOT_FLEET_WORK))


def test_an_unmapped_capability_is_refused_here_not_by_the_dispatcher():
    """So the operator gets a sentence they can act on rather than RMF's
    rejection of a request it could not read."""
    result = adapter().invoke(
        CallRequest("c", "vision.observe", {"waypoint": "ward_3"})
    )
    assert result.outcome == "refused"
    assert "no declared Open-RMF category" in result.detail


# -- reading the fleets ------------------------------------------------------


def test_a_fleet_that_cannot_deliver_does_not_advertise_transport():
    declared = adapter(fleets=[{"name": "patrolOnly", "task_types": ["patrol"]}]).describe()
    assert {item.capability_id for item in declared} == {"motion.navigate", "motion.dock"}


def test_the_newer_capabilities_key_is_read_too():
    declared = adapter(fleets=[{"name": "f", "capabilities": ["delivery"]}]).describe()
    assert {item.capability_id for item in declared} == {
        "transport.load",
        "transport.unload",
    }


def test_a_fleet_reporting_neither_key_contributes_nothing():
    """Rather than being assumed to do everything."""
    assert adapter(fleets=[{"name": "silent"}]).describe() == []


def test_a_fleet_with_no_name_is_recorded_rather_than_guessed():
    device = adapter(fleets=[{"task_types": ["patrol"]}])
    assert device.describe() == []
    assert device.unmapped == [("", "a fleet with no name")]


def test_two_fleets_offering_the_same_category_each_declare():
    declared = adapter(
        fleets=[
            {"name": "a", "task_types": ["patrol"]},
            {"name": "b", "task_types": ["patrol"]},
        ]
    ).describe()
    assert {item.resource_id for item in declared} == {"fleet:a", "fleet:b"}


# -- requests RMF can actually read ------------------------------------------


def test_a_call_with_no_waypoint_is_refused_rather_than_invented():
    """Inventing a waypoint would dispatch a robot somewhere nobody asked for."""
    result = adapter().invoke(navigate(distance_m=3))
    assert result.outcome == "refused"
    assert "named waypoint" in result.detail


def test_patrol_carries_the_place_it_was_given():
    sent: list = []
    adapter(sent=sent).invoke(navigate(waypoint="ward_3"))
    assert sent[-1][1]["request"]["description"]["places"] == ["ward_3"]


def test_the_start_time_is_left_to_the_fleet():
    """Stamping one here would make this process's clock the fleet's schedule."""
    sent: list = []
    adapter(sent=sent).invoke(navigate())
    assert sent[-1][1]["request"]["unix_millis_earliest_start_time"] == 0


# -- failure ------------------------------------------------------------------


def test_a_dispatcher_that_cannot_be_reached_is_refused_not_reported_done():
    result = adapter(fail=OSError("connection refused")).invoke(navigate())
    assert result.outcome == "refused"
    assert "could not be reached" in result.detail


def test_a_dispatcher_that_returns_no_state_is_refused():
    result = adapter(dispatch={"errors": ["no fleet bid"]}).invoke(navigate())
    assert result.outcome == "refused"
    assert "no fleet bid" in result.detail


def test_cancelling_something_never_dispatched_is_refused():
    """Answering "cancelled" for something never running would let a caller
    believe a machine was stopped when nothing was ever told to move."""
    result = adapter().cancel("never-sent")
    assert result.outcome == "refused"


def test_cancelling_a_dispatched_task_uses_the_id_rmf_gave_it():
    sent: list = []
    device = adapter(sent=sent)
    device.invoke(navigate())
    result = device.cancel("call-1")
    assert result.outcome == "completed"
    assert sent[-1][1]["task_id"] == "rmf-task-1"


# -- the fleet as a commanded resource (flyto2.external_adapters) -------------


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def fleet_adapter(states, *, sent=None, fleets=None):
    """A dispatcher that reports ``states`` in turn for the dispatched task."""
    clock = Clock()
    remaining = list(states)

    def call(path, payload=None, **_):
        if sent is not None:
            sent.append((path, payload))
        if path == "/fleets":
            return fleets if fleets is not None else [
                {"name": "tinyRobot", "task_types": ["patrol", "delivery"]},
                {"name": "deliveryRobot", "task_types": ["delivery"]},
            ]
        if path == "/tasks/dispatch_task":
            return {
                "state": {
                    "booking": {"id": "rmf-task-1"},
                    "status": "queued",
                    "assigned_to": {"group": "tinyRobot", "name": "tinyRobot2"},
                }
            }
        if path == "/tasks/rmf-task-1/state":
            status = remaining.pop(0) if len(remaining) > 1 else remaining[0]
            return {"status": status, "assigned_to": {"group": "tinyRobot", "name": "tinyRobot1"}}
        if path == "/tasks/cancel_task":
            return {"success": True}
        raise AssertionError(f"unexpected path {path}")

    device = OpenRmfAdapter(
        call=call,
        fleet="tinyRobot",
        wait_for_completion=True,
        poll_seconds=1.0,
        sleep=clock.sleep,
        clock=clock,
    )
    return device


def fleet_call(capability_id="motion.navigate", deadline=30.0, call_id="call-1"):
    return CallRequest(call_id, capability_id, {"waypoint": "ward_3"}, deadline)


def test_the_entry_point_builds_one_fleet_that_waits_for_the_task():
    device = build_adapter("fleet:tinyRobot")
    assert device.fleet == "tinyRobot"
    assert device.resource_id == "fleet:tinyRobot"
    assert device._wait is True
    assert ADAPTER_ID == "open_rmf.fleet" and MODULE_PACK == "fleet"


@pytest.mark.parametrize("resource_id", ["", "tinyRobot", "fleet:", "robot:tinyRobot2"])
def test_a_resource_that_is_not_a_fleet_is_refused(resource_id):
    with pytest.raises(ValueError):
        fleet_name(resource_id)


def test_the_request_keeps_the_task_in_the_fleet_and_still_names_no_robot():
    sent: list = []
    fleet_adapter(["completed"], sent=sent).invoke(fleet_call())
    _path, payload = next(item for item in sent if item[0] == "/tasks/dispatch_task")
    assert payload["request"]["fleet_name"] == "tinyRobot"
    assert "robot" not in payload["request"]
    assert "tinyRobot2" not in str(payload)


def test_a_finished_navigation_is_completed_with_the_fleet_s_arrival():
    device = fleet_adapter(["queued", "underway", "completed"])
    result = device.invoke(fleet_call())
    assert result.outcome == OUTCOME_COMPLETED
    assert result.evidence["rmf_status"] == "completed"
    # The robot RMF reported at the end, not the one bid at dispatch.
    assert result.evidence["assigned_robot"] == "tinyRobot1"
    [arrival] = result.evidence["evidence_items"]
    assert arrival["kind"] == "robot.arrival" and arrival["usable"] is True
    assert arrival["reason"]["observed"]["waypoint"] == "ward_3"
    assert arrival["reason"]["source"] == "open-rmf.task-state"


def test_a_load_proves_no_arrival():
    result = fleet_adapter(["completed"]).invoke(fleet_call("transport.load"))
    assert result.outcome == OUTCOME_COMPLETED
    assert "evidence_items" not in result.evidence


@pytest.mark.parametrize("status", ["failed", "canceled", "killed"])
def test_a_task_rmf_ended_badly_is_a_failed_call(status):
    result = fleet_adapter(["underway", status]).invoke(fleet_call())
    assert result.outcome == OUTCOME_FAILED
    assert status in result.detail


def test_a_task_still_running_at_the_deadline_times_out_and_is_never_dispatched_twice():
    sent: list = []
    device = fleet_adapter(["underway"], sent=sent)
    first = device.invoke(fleet_call(deadline=3.0))
    assert first.outcome == OUTCOME_TIMEOUT
    assert first.evidence["rmf_task_id"] == "rmf-task-1"
    device.invoke(fleet_call(deadline=3.0))
    assert [path for path, _ in sent].count("/tasks/dispatch_task") == 1
    # The host's cancel after a timeout withdraws the task RMF is running.
    assert device.cancel("call-1").outcome == OUTCOME_COMPLETED


def test_a_finished_call_answers_again_with_its_result():
    sent: list = []
    device = fleet_adapter(["completed"], sent=sent)
    first = device.invoke(fleet_call())
    assert device.invoke(fleet_call()) is first
    assert [path for path, _ in sent].count("/tasks/dispatch_task") == 1


def test_the_fleet_adapter_declares_only_its_own_fleet():
    declared = fleet_adapter(["completed"]).describe()
    assert {item.resource_id for item in declared} == {"fleet:tinyRobot"}
    # A fleet travels to waypoints: it declares that, not a robot's navigate.
    assert {item.capability_id for item in declared} == {
        "motion.dock",
        "motion.navigate_to_waypoint",
        "transport.load",
        "transport.unload",
    }


def test_waypoint_navigation_is_a_patrol_and_arrives():
    sent: list = []
    result = fleet_adapter(["completed"], sent=sent).invoke(
        fleet_call("motion.navigate_to_waypoint")
    )
    assert result.outcome == OUTCOME_COMPLETED
    _path, payload = next(item for item in sent if item[0] == "/tasks/dispatch_task")
    assert payload["request"]["category"] == "patrol"
    assert payload["request"]["description"] == {"places": ["ward_3"], "rounds": 1}
    assert result.evidence["evidence_items"][0]["kind"] == "robot.arrival"


def test_the_unbound_conformance_build_still_declares_motion_navigate():
    declared = adapter(fleets=[{"name": "p", "task_types": ["patrol"]}]).describe()
    assert {item.capability_id for item in declared} == {"motion.navigate", "motion.dock"}


def test_deployment_mode_is_physical_unless_said_otherwise(monkeypatch):
    device = fleet_adapter(["completed"])
    monkeypatch.delenv("FLYTO_RMF_DEPLOYMENT_MODE", raising=False)
    assert device.deployment_mode == "physical"
    monkeypatch.setenv("FLYTO_RMF_DEPLOYMENT_MODE", "simulation")
    assert device.deployment_mode == "simulation"


def test_discovery_publishes_each_fleet_with_its_pack_only_when_asked():
    reader = OpenRmfAdapter(call=fleet_adapter(["completed"])._call)
    plain = discover_fleet_manifests(execution_host_id="host", adapter=reader)
    asked = discover_fleet_manifests(
        execution_host_id="host", adapter=reader, manifest_extensions=["module_pack"]
    )
    assert [item["resource_id"] for item in plain] == ["fleet:deliveryRobot", "fleet:tinyRobot"]
    assert all("module_pack" not in item for item in plain)
    assert all(item["module_pack"] == "fleet" for item in asked)
    tiny = asked[1]
    assert tiny["adapter"]["adapter_id"] == "open_rmf.fleet"
    assert tiny["capability_ids"] == [
        "motion.dock",
        "motion.navigate_to_waypoint",
        "transport.load",
        "transport.unload",
    ]
    assert all(item["requires_safe_stop"] is False for item in tiny["capability_contracts"])
    assert asked[0]["capability_ids"] == ["transport.load", "transport.unload"]


def test_discovery_needs_a_configured_dispatcher(monkeypatch):
    monkeypatch.delenv("FLYTO_RMF_API_URL", raising=False)
    assert discover_fleet_manifests(execution_host_id="host") == []
