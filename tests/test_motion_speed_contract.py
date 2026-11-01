"""A straight drive's speed comes from the declared contract, never a literal.

``motion.advance`` and ``motion.retreat`` declare ``speed_mps`` with bounds and
a default in their input schema. A call that names no speed is sent at the
declared default. The one speed ceiling above that is the site safety
profile's ``max_linear_speed`` (``safety_profile.CONSTRAINABLE``, ``at_most``),
the same limit every job is folded into: it can only tighten the declared
bound and the default, never widen them, and a profile that cannot be read
refuses motion. The braking plan still lowers the speed near obstacles and the
clearance floor is untouched.
"""

from __future__ import annotations

import dataclasses
import json

import pytest

from flyto_robotics import adapter_contract as decl
from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.adapter_contract import CallRequest
from flyto_robotics.generic_ros2_adapter import (
    GenericROS2Adapter,
    RosbridgeROS2Backend,
    declared_arguments,
    declared_default,
)
from flyto_robotics.safety_profile import PROFILE_CONTRACT_VERSION

RETIRED_ENV = "FLYTO_ROS2_MAX_LINEAR_SPEED_MPS"


def site_profile(tmp_path, monkeypatch, limits):
    """Point the adapter at a site safety profile with these limits."""
    path = tmp_path / "safety-profile.json"
    path.write_text(
        json.dumps({"contract_version": PROFILE_CONTRACT_VERSION, "limits": limits}),
        encoding="utf-8",
    )
    monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(path))
    return path


class QuietSocket:
    def send(self, _payload):
        pass

    def recv(self, timeout=None):
        raise TimeoutError

    def pong(self, _data=b""):
        pass

    def close(self):
        pass


def speed_argument(capability_id):
    return next(item for item in declared_arguments(capability_id) if item.name == "speed_mps")


def with_speed_default(monkeypatch, capability_id, default):
    """The contract with another declared default, so a literal in code would show."""
    arguments = tuple(
        dataclasses.replace(item, default=default) if item.name == "speed_mps" else item
        for item in adapter_module.ARGUMENTS[capability_id]
    )
    monkeypatch.setitem(adapter_module.ARGUMENTS, capability_id, arguments)


@pytest.fixture(autouse=True)
def no_site_profile(monkeypatch, tmp_path):
    monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(tmp_path / "absent.json"))
    monkeypatch.delenv(RETIRED_ENV, raising=False)


@pytest.mark.parametrize("capability_id", ["motion.advance", "motion.retreat"])
def test_the_speed_default_is_part_of_the_published_input_schema(capability_id):
    speed = speed_argument(capability_id)
    assert speed.default is not None
    assert speed.minimum <= speed.default <= speed.maximum
    schema = decl.arguments_to_json_schema(declared_arguments(capability_id))
    published = schema["properties"]["speed_mps"]
    assert published["default"] == speed.default
    assert published["maximum"] == speed.maximum
    assert published["minimum"] == speed.minimum


@pytest.mark.parametrize("capability_id", ["motion.advance", "motion.retreat"])
def test_a_drive_without_a_speed_is_sent_at_the_declared_default(monkeypatch, capability_id):
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: QuietSocket()
    )
    try:
        with_speed_default(monkeypatch, capability_id, 0.15)
        goal = backend._action_goal(capability_id, {"distance_m": 1.0})
        assert goal["speed"] == pytest.approx(0.15)
        assert declared_default(capability_id, "speed_mps") == pytest.approx(0.15)
        # A speed the caller named is sent as named.
        assert backend._action_goal(
            capability_id, {"distance_m": 1.0, "speed_mps": 0.05}
        )["speed"] == pytest.approx(0.05)
    finally:
        backend.disconnect()


def test_the_site_ceiling_tightens_the_declared_bound_and_default(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 0.22})
    advance = speed_argument("motion.advance")
    assert advance.maximum == pytest.approx(0.22)
    retreat = speed_argument("motion.retreat")
    # Already below the site's ceiling: the contract's own bound stands.
    assert retreat.maximum == adapter_module.ARGUMENTS["motion.retreat"][1].maximum

    with_speed_default(monkeypatch, "motion.advance", 0.25)
    assert declared_default("motion.advance", "speed_mps") == pytest.approx(0.22)


def test_a_site_ceiling_above_the_contract_never_widens_it(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 3.0})
    contract = next(
        item for item in adapter_module.ARGUMENTS["motion.advance"] if item.name == "speed_mps"
    )
    assert speed_argument("motion.advance").maximum == contract.maximum
    assert speed_argument("motion.advance").default == contract.default


def test_a_speed_above_the_site_ceiling_is_refused(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 0.22})
    with pytest.raises(ValueError, match="above the declared maximum"):
        adapter_module._numeric_arguments(
            "motion.advance", {"distance_m": 1.0, "speed_mps": 0.24}
        )
    assert adapter_module._numeric_arguments(
        "motion.advance", {"distance_m": 1.0, "speed_mps": 0.22}
    )["speed_mps"] == pytest.approx(0.22)


def test_the_retired_adapter_speed_variable_has_no_effect(monkeypatch):
    # One ceiling: the site profile. The adapter's own variable is gone.
    monkeypatch.setenv(RETIRED_ENV, "0.05")
    contract = adapter_module.ARGUMENTS["motion.advance"][1]
    assert speed_argument("motion.advance").maximum == contract.maximum
    assert not hasattr(adapter_module, "MAX_LINEAR_SPEED_ENV")


def preflight(monkeypatch):
    """The adapter's motion preflight for an advance, on a graph that never answers."""
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")  # no deployment cross-check
    monkeypatch.setenv("FLYTO_ROS2_OBSERVATION_WAIT_SECONDS", "0.1")
    backend = RosbridgeROS2Backend(
        url="ws://127.0.0.1:1", connection_factory=lambda _url: QuietSocket()
    )
    adapter = GenericROS2Adapter(backend=backend, resource_id="robot")
    adapter._declared = {"motion.advance"}
    try:
        return adapter.invoke(CallRequest("adv-1", "motion.advance", {"distance_m": 0.3}))
    finally:
        adapter.disconnect()


@pytest.mark.parametrize(
    "document",
    [
        "{not json",
        json.dumps({"contract_version": "other", "limits": {}}),
        json.dumps({"contract_version": PROFILE_CONTRACT_VERSION, "limits": {"top": 1}}),
    ],
)
def test_an_unreadable_site_profile_refuses_motion(monkeypatch, tmp_path, document):
    path = tmp_path / "safety-profile.json"
    path.write_text(document, encoding="utf-8")
    monkeypatch.setenv("FLYTO_SAFETY_PROFILE", str(path))
    result = preflight(monkeypatch)
    assert result.outcome == "refused"
    assert "safety profile" in result.detail


def test_a_site_ceiling_below_every_declared_speed_refuses_motion(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 0.01})
    # The declared bound never drops below the contract minimum, so a speed
    # the site forbids would otherwise stay allowed.
    result = preflight(monkeypatch)
    assert result.outcome == "refused"
    assert "max_linear_speed" in result.detail


def test_a_readable_site_ceiling_lets_the_preflight_go_on(monkeypatch, tmp_path):
    site_profile(tmp_path, monkeypatch, {"max_linear_speed": 0.22})
    result = preflight(monkeypatch)
    # Refused further on (the quiet graph has no odometry), not by the ceiling.
    assert "safety profile" not in result.detail and "max_linear_speed" not in result.detail


def test_no_speed_literal_is_left_in_the_adapter():
    assert not hasattr(adapter_module, "DEFAULT_STRAIGHT_SPEED_MPS")


def test_the_clearance_floor_is_untouched(monkeypatch):
    monkeypatch.delenv("FLYTO_ROS2_MIN_CLEARANCE_M", raising=False)
    assert adapter_module._clearance_floor() == 0.35
    monkeypatch.setenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.1")
    assert adapter_module._clearance_floor() == 0.35
