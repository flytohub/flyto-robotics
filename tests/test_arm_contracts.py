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

from flyto_robotics.adapter_contract import OUTCOME_COMPLETED, CallRequest
from flyto_robotics.arm_adapter import (
    CAPABILITY_HALT,
    CAPABILITY_MOVE,
    CAPABILITY_STATE,
    SerialArmAdapter,
    load_profile,
    parse_profile,
)
from flyto_robotics.arm_bus import SimulatedArmBus
from flyto_robotics.arm_contracts import capability_rows
from tests.test_arm_adapter import EXAMPLE_PROFILE, JOINTS, FakeClock, raw_profile


def rows_by_id(profile):
    return {row["provides_capability"]: row for row in capability_rows(profile)}


@pytest.fixture(params=["hold", "torque_off"])
def profile(request):
    return parse_profile(raw_profile(stop_mode=request.param))


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
    assert len(capability_rows(load_profile(EXAMPLE_PROFILE))) == 3


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
