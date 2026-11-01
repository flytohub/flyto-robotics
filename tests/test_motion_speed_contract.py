"""A straight drive's speed comes from the declared contract, never a literal.

``motion.advance`` and ``motion.retreat`` declare ``speed_mps`` with bounds and
a default in their input schema. A call that names no speed is sent at the
declared default; a resource declares its own top speed
(``FLYTO_ROS2_MAX_LINEAR_SPEED_MPS``, 0.22 m/s on a TurtleBot3 Burger), which
can only tighten the declared bound and the default, never widen them. The
braking plan still lowers the speed near obstacles and the clearance floor is
untouched.
"""

from __future__ import annotations

import dataclasses

import pytest

from flyto_robotics import adapter_contract as decl
from flyto_robotics import generic_ros2_adapter as adapter_module
from flyto_robotics.generic_ros2_adapter import (
    RosbridgeROS2Backend,
    declared_arguments,
    declared_default,
)

LIMIT_ENV = "FLYTO_ROS2_MAX_LINEAR_SPEED_MPS"


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
def no_resource_limit(monkeypatch):
    monkeypatch.delenv(LIMIT_ENV, raising=False)


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


def test_the_resource_limit_tightens_the_declared_bound_and_default(monkeypatch):
    monkeypatch.setenv(LIMIT_ENV, "0.22")
    advance = speed_argument("motion.advance")
    assert advance.maximum == pytest.approx(0.22)
    retreat = speed_argument("motion.retreat")
    # Already below the robot's limit: the contract's own bound stands.
    assert retreat.maximum == adapter_module.ARGUMENTS["motion.retreat"][1].maximum

    with_speed_default(monkeypatch, "motion.advance", 0.25)
    assert declared_default("motion.advance", "speed_mps") == pytest.approx(0.22)


def test_a_resource_limit_above_the_contract_never_widens_it(monkeypatch):
    monkeypatch.setenv(LIMIT_ENV, "3.0")
    contract = next(
        item for item in adapter_module.ARGUMENTS["motion.advance"] if item.name == "speed_mps"
    )
    assert speed_argument("motion.advance").maximum == contract.maximum


def test_a_speed_above_the_resource_limit_is_refused(monkeypatch):
    monkeypatch.setenv(LIMIT_ENV, "0.22")
    with pytest.raises(ValueError, match="above the declared maximum"):
        adapter_module._numeric_arguments(
            "motion.advance", {"distance_m": 1.0, "speed_mps": 0.24}
        )
    assert adapter_module._numeric_arguments(
        "motion.advance", {"distance_m": 1.0, "speed_mps": 0.22}
    )["speed_mps"] == pytest.approx(0.22)


@pytest.mark.parametrize("raw", ["fast", "nan", "inf", "0", "-0.1", "0.01"])
def test_an_unusable_resource_limit_refuses_motion(monkeypatch, raw):
    monkeypatch.setenv(LIMIT_ENV, raw)
    problem = adapter_module._speed_limit_problem()
    assert problem is not None and LIMIT_ENV in problem


def test_no_speed_literal_is_left_in_the_adapter():
    assert not hasattr(adapter_module, "DEFAULT_STRAIGHT_SPEED_MPS")


def test_the_clearance_floor_is_untouched(monkeypatch):
    monkeypatch.delenv("FLYTO_ROS2_MIN_CLEARANCE_M", raising=False)
    assert adapter_module._clearance_floor() == 0.35
    monkeypatch.setenv("FLYTO_ROS2_MIN_CLEARANCE_M", "0.1")
    assert adapter_module._clearance_floor() == 0.35
