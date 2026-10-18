"""The arm's capability contract rows say what the adapter does.

The structural checks run everywhere. The checks against flyto-core's own
``validate_contract`` and ``judge`` run only where flyto-core is importable
(this repository does not depend on it): on a workstation,
``PYTHONPATH=<flyto-core>/src <flyto-core venv python> -m pytest
tests/test_arm_contracts.py``.
"""

from __future__ import annotations

import math

import pytest

from flyto_robotics.adapter_contract import (
    KNOWN_CAPABILITY_IDS,
    OUTCOME_COMPLETED,
    CallRequest,
    capability_metadata,
)
from flyto_robotics.arm_adapter import (
    CAPABILITY_GRASP,
    CAPABILITY_HALT,
    CAPABILITY_MOVE,
    CAPABILITY_RELEASE,
    CAPABILITY_STATE,
    SerialArmAdapter,
    load_profile,
    parse_profile,
)
from flyto_robotics.arm_bus import SimulatedArmBus
from flyto_robotics.arm_contracts import capability_rows
from tests.test_arm_adapter import EXAMPLE_PROFILE, JOINTS, FakeClock, raw_profile
from tests.test_arm_gripper import gripper_arm, gripper_profile


def rows_by_id(profile):
    return {row["provides_capability"]: row for row in capability_rows(profile)}


@pytest.fixture(params=["hold", "torque_off"])
def profile(request):
    return parse_profile(raw_profile(stop_mode=request.param))


def test_the_arm_serves_the_shared_vocabulary_ids(profile):
    """Cloud approves only vocabulary capabilities, so the arm speaks them, not ids of its own.

    A manipulator's state and joint moves are the ``manipulation`` family; its
    stop is the one shipped safe stop, whatever is moving.
    """
    rows = rows_by_id(profile)

    assert set(rows) == {"manipulation.state", "manipulation.move_joints", "motion.halt"}
    assert {"manipulation.grasp", "manipulation.release"} <= KNOWN_CAPABILITY_IDS
    for capability_id, row in rows.items():
        assert capability_id in KNOWN_CAPABILITY_IDS and row["module_id"] == capability_id
        metadata = capability_metadata(capability_id)
        assert row["contract"]["safety_class"] == metadata["safety_class"]
        assert row["contract"]["requires_safe_stop"] == metadata["requires_safe_stop"]
        assert row["contract"]["cancellable"] == metadata["cancellable"]


def test_the_three_m0_capabilities_are_declared(profile):
    rows = rows_by_id(profile)

    assert set(rows) == {CAPABILITY_STATE, CAPABILITY_MOVE, CAPABILITY_HALT}
    assert rows[CAPABILITY_STATE]["contract"]["safety_class"] == "read_only"
    assert rows[CAPABILITY_STATE]["contract"]["actuates"] is False
    assert rows[CAPABILITY_MOVE]["contract"]["safety_class"] == "movement"
    halt = rows[CAPABILITY_HALT]["contract"]
    assert halt["role"] == "safe_stop"
    assert (halt["cancellable"], halt["requires_safe_stop"], halt["idempotent"]) == (
        False, False, True,
    )


def test_every_move_number_has_the_profiles_finite_limits(profile):
    params = rows_by_id(profile)[CAPABILITY_MOVE]["params_schema"]

    assert set(params) == {f"{joint.name}_rad" for joint in profile.joints}
    for joint in profile.joints:
        schema = params[f"{joint.name}_rad"]
        assert schema["type"] == "number" and schema["required"] is True
        assert (schema["min"], schema["max"]) == (joint.min_rad, joint.max_rad)
        assert math.isfinite(schema["min"]) and math.isfinite(schema["max"])


def test_move_evidence_is_measured_from_read_back_joint_positions(profile):
    evidence = rows_by_id(profile)[CAPABILITY_MOVE]["contract"]["evidence"]

    assert [spec["measure"]["fields"] for spec in evidence] == [[j] for j in JOINTS]
    assert {spec["observe"] for spec in evidence} == {"joint_positions"}
    assert {spec["measure"]["op"] for spec in evidence} == {"distance_to"}


def test_the_example_profile_produces_rows_too():
    # Its gripper is declared, so it serves grasp and release as well.
    assert [row["module_id"] for row in capability_rows(load_profile(EXAMPLE_PROFILE))] == [
        CAPABILITY_STATE, CAPABILITY_MOVE, CAPABILITY_HALT, CAPABILITY_GRASP, CAPABILITY_RELEASE,
    ]


# -- grasp and release -----------------------------------------------------------


@pytest.fixture(params=["min", "max"])
def gripper(request):
    return parse_profile(gripper_profile(request.param))


def test_a_declared_gripper_serves_grasp_and_release_under_the_shared_vocabulary(gripper):
    rows = rows_by_id(gripper)

    assert set(rows) == {
        CAPABILITY_STATE, CAPABILITY_MOVE, CAPABILITY_HALT, CAPABILITY_GRASP, CAPABILITY_RELEASE,
    }
    for capability_id in (CAPABILITY_GRASP, CAPABILITY_RELEASE):
        contract = rows[capability_id]["contract"]
        metadata = capability_metadata(capability_id)
        assert capability_id in KNOWN_CAPABILITY_IDS
        for key in ("safety_class", "requires_safe_stop", "cancellable"):
            assert contract[key] == metadata[key], (capability_id, key)
        assert contract["cancellable"] is True
        assert contract["actuates"] is True and contract["idempotent"] is False


def test_the_release_target_is_bounded_by_the_profile_and_grasp_takes_none(gripper):
    rows = rows_by_id(gripper)
    schema = rows[CAPABILITY_RELEASE]["params_schema"]["open_to_rad"]

    assert rows[CAPABILITY_GRASP]["params_schema"] == {}
    assert schema["required"] is True
    assert (schema["min"], schema["max"]) == gripper.gripper.release_range


def test_grip_evidence_reads_the_jaw_position_and_whether_an_object_is_held(gripper):
    rows = rows_by_id(gripper)
    grasp = rows[CAPABILITY_GRASP]["contract"]["evidence"]
    release = rows[CAPABILITY_RELEASE]["contract"]["evidence"]

    assert {spec["observe"] for spec in grasp} == {"object_held"}
    assert [spec["measure"]["fields"] for spec in grasp] == [["held"], ["gripper"]]
    assert [spec["observe"] for spec in release] == ["gripper_position"]
    assert release[0]["measure"]["op"] == "distance_to"


def test_an_arm_without_a_gripper_declares_no_grip_rows(profile):
    assert not {CAPABILITY_GRASP, CAPABILITY_RELEASE} & set(rows_by_id(profile))


# -- flyto-core, when installed ------------------------------------------------


def _core():
    return pytest.importorskip("core.capability_contract")


def test_flyto_core_admits_every_row(profile):
    core = _core()
    for row in capability_rows(profile):
        normalized = core.validate_contract(row["contract"], row["params_schema"])
        assert normalized["schema"] == "flyto.capability-contract.v1"


def test_flyto_core_admits_the_example_profiles_rows():
    core = _core()
    for row in capability_rows(load_profile(EXAMPLE_PROFILE)):
        core.validate_contract(row["contract"], row["params_schema"])


def test_flyto_core_refuses_the_move_row_if_a_bound_is_dropped(profile):
    core = _core()
    row = rows_by_id(profile)[CAPABILITY_MOVE]
    del row["params_schema"]["gripper_rad"]["max"]

    with pytest.raises(Exception):  # noqa: B017 - core's own error type
        core.validate_contract(row["contract"], row["params_schema"])


def _adapter(stop_mode="hold"):
    profile = parse_profile(raw_profile(stop_mode=stop_mode))
    clock = FakeClock()
    bus = SimulatedArmBus(JOINTS, torque_enabled=True)
    return SerialArmAdapter(bus, profile, clock=clock, sleep=clock.sleep), bus, profile


def test_flyto_core_judges_a_completed_move_usable_and_a_short_one_not():
    core = _core()
    adapter, bus, profile = _adapter()
    arguments = {"shoulder_pan_rad": 0.3, "elbow_flex_rad": -0.2, "gripper_rad": 0.1}
    result = adapter.invoke(CallRequest("m", CAPABILITY_MOVE, arguments))
    assert result.outcome == OUTCOME_COMPLETED
    observed = result.evidence["observations"]["joint_positions"]
    specs = rows_by_id(profile)[CAPABILITY_MOVE]["contract"]["evidence"]

    assert all(core.judge(spec, arguments, observed)["usable"] for spec in specs)

    short = {phase: {**values, "elbow_flex": 0.0} for phase, values in observed.items()}
    verdicts = {spec["kind"]: core.judge(spec, arguments, short)["usable"] for spec in specs}
    assert verdicts["joint.elbow_flex.arrival"] is False


def test_flyto_core_judges_a_confirmed_hold_usable_and_a_sagging_one_not():
    core = _core()
    adapter, bus, profile = _adapter()
    specs = rows_by_id(profile)[CAPABILITY_HALT]["contract"]["evidence"]

    held = adapter.safe_stop()
    observed = held.evidence["observations"]["joint_positions"]
    assert all(core.judge(spec, {}, observed)["usable"] for spec in specs)

    bus.sag_rad = 0.01
    sagged = adapter.safe_stop()
    observed = sagged.evidence["observations"]["joint_positions"]
    assert not all(core.judge(spec, {}, observed)["usable"] for spec in specs)


def test_every_arm_call_but_the_stop_is_cancellable_as_cloud_reads_it():
    """Cloud's vocabulary cancels every dispatched call except the stop and a stream; a read
    is cancellable too (cancelling it stops waiting). The adapter must say the same."""
    from flyto_robotics.adapter_contract import _CAPABILITY_METADATA

    manipulation = {
        cap: row for cap, row in _CAPABILITY_METADATA.items() if cap.startswith("manipulation.")
    }
    assert manipulation and all(row["cancellable"] for row in manipulation.values())


def test_every_served_manipulation_row_is_cancellable_and_the_stop_is_not(gripper):
    """Grasp and release included: the rows the adapter serves say what Cloud reads."""
    rows = rows_by_id(gripper)

    manipulation = {cap for cap in rows if cap.startswith("manipulation.")}
    assert manipulation == {
        CAPABILITY_STATE, CAPABILITY_MOVE, CAPABILITY_GRASP, CAPABILITY_RELEASE,
    }
    assert all(rows[cap]["contract"]["cancellable"] is True for cap in manipulation)
    assert rows[CAPABILITY_HALT]["contract"]["cancellable"] is False


def test_flyto_core_admits_the_grip_rows(gripper):
    core = _core()
    rows = rows_by_id(gripper)
    for capability_id in (CAPABILITY_GRASP, CAPABILITY_RELEASE):
        row = rows[capability_id]
        assert core.validate_contract(row["contract"], row["params_schema"])["evidence"]


def test_flyto_core_refuses_the_release_row_if_its_bound_is_dropped(gripper):
    core = _core()
    row = rows_by_id(gripper)[CAPABILITY_RELEASE]
    del row["params_schema"]["open_to_rad"]["min"]

    with pytest.raises(Exception):  # noqa: B017 - core's own error type
        core.validate_contract(row["contract"], row["params_schema"])


def test_flyto_core_judges_a_held_grasp_usable_and_an_empty_or_slipping_one_not():
    core = _core()
    adapter, bus = gripper_arm()
    specs = rows_by_id(adapter.profile)[CAPABILITY_GRASP]["contract"]["evidence"]

    def verdicts(result):
        observed = result.evidence["observations"]
        return {spec["kind"]: core.judge(spec, {}, observed[spec["observe"]])["usable"]
                for spec in specs}

    bus.place_object("gripper", 0.2)
    held = adapter.invoke(CallRequest("g1", CAPABILITY_GRASP))
    assert held.outcome == OUTCOME_COMPLETED
    assert verdicts(held) == {"gripper.object_held": True, "gripper.hold_stable": True}

    adapter, bus = gripper_arm()
    empty = adapter.invoke(CallRequest("g2", CAPABILITY_GRASP))
    # Closed on nothing: no hold was observed, so neither spec can be proven.
    assert "object_held" not in empty.evidence["observations"]
    assert not any(
        core.judge(spec, {}, empty.evidence["observations"].get(spec["observe"], {}))["usable"]
        for spec in specs
    )

    adapter, bus = gripper_arm()
    bus.place_object("gripper", 0.2, slip_rad_per_read=0.01, slip_after_reads=11)
    slipped = adapter.invoke(CallRequest("g3", CAPABILITY_GRASP))
    assert verdicts(slipped) == {"gripper.object_held": False, "gripper.hold_stable": False}


def test_flyto_core_judges_an_open_release_usable_and_a_short_one_not():
    core = _core()
    adapter, bus = gripper_arm()
    bus.place_object("gripper", 0.2)
    assert adapter.invoke(CallRequest("g", CAPABILITY_GRASP)).outcome == OUTCOME_COMPLETED
    arguments = {"open_to_rad": 0.45}
    result = adapter.invoke(CallRequest("r", CAPABILITY_RELEASE, arguments))
    assert result.outcome == OUTCOME_COMPLETED
    (spec,) = rows_by_id(adapter.profile)[CAPABILITY_RELEASE]["contract"]["evidence"]
    observed = result.evidence["observations"]["gripper_position"]

    assert core.judge(spec, arguments, observed)["usable"]
    short = {phase: {**values, "gripper": 0.3} for phase, values in observed.items()}
    assert not core.judge(spec, arguments, short)["usable"]
