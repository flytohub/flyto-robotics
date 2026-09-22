"""The fleet layer, and the one place Flyto2 deliberately stops deciding.

Flyto2 is the executive: it composes a Goal out of capabilities. Open-RMF is the
dispatch office: it runs a bid across fleets and picks the machine. The tests
that matter here are the ones holding that line — most of a bad fleet adapter's
damage comes from quietly doing the dispatcher's job.
"""

from __future__ import annotations

from flyto_robotics import adapter_contract as decl
from flyto_robotics.adapter_contract import KNOWN_CAPABILITY_IDS, CallRequest
from flyto_robotics.open_rmf_adapter import (
    CAPABILITY_TO_CATEGORY,
    NOT_FLEET_WORK,
    OpenRmfAdapter,
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
