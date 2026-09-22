"""Generic external ROS 2 adapter contract.

The fake backend is a ROS graph, not a Flyto2 robot gateway. These tests pin the
architectural boundary: the adapter may know standard ROS 2 action/topic names,
but the TurtleBot3 side needs no Flyto2 process, credential, queue, or HTTP
endpoint.
"""

from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path
from typing import Any

from flyto_robotics import adapter_contract as decl
from flyto_robotics.adapter_contract import (
    OUTCOME_CANCELLED,
    OUTCOME_COMPLETED,
    OUTCOME_REFUSED,
    OUTCOME_TIMEOUT,
    CallRequest,
    CallResult,
)
from flyto_robotics.generic_ros2_adapter import (
    CMD_VEL_TYPES,
    DEFAULT_INTERFACES,
    GenericROS2Adapter,
    StandardInterface,
)

ROOT = Path(__file__).resolve().parent.parent


class FakeROS2Backend:
    def __init__(self, capabilities=None):
        selected = capabilities or set(DEFAULT_INTERFACES)
        self.interfaces = []
        for capability_id in selected:
            kind, name, interface_type = DEFAULT_INTERFACES[capability_id]
            if capability_id == "motion.halt":
                interface_type = "geometry_msgs/msg/TwistStamped"
            self.interfaces.append(StandardInterface(kind, name, interface_type))
        self.connected = True
        self.active = set()
        self.stop_calls = set()
        self.counts = {}
        self.calls = []
        self.observation_payload = {
            "pose": {"frame": "odom", "x": 0.25, "y": 0.0, "yaw": 0.0},
            "range": {"minimum_range_m": 0.8, "sample_count": 400},
            "camera": {
                "encoding": "yuv422_yuy2",
                "width": 640,
                "height": 480,
                "calibrated": False,
                "calibration_snapshot": None,
                "frame_snapshot": "b" * 64,
            },
            "map_tf_available": True,
        }

    def discover(self):
        return tuple(self.interfaces)

    def invoke(
        self,
        *,
        call_id: str,
        capability_id: str,
        arguments: Mapping[str, Any],
        deadline_seconds: float,
    ):
        if not self.connected:
            return CallResult(call_id, OUTCOME_REFUSED, detail="disconnected")
        previous = next((item for item in self.calls if item[0] == call_id), None)
        if previous is not None:
            if call_id in self.active:
                return CallResult(call_id, OUTCOME_TIMEOUT, detail="still running")
            return CallResult(
                call_id,
                OUTCOME_COMPLETED,
                evidence={"standard_interface": previous[1], "odom": {"x": 0.25}},
            )
        self.calls.append((call_id, capability_id, dict(arguments)))
        self.counts[call_id] = self.counts.get(call_id, 0) + 1
        if deadline_seconds < 0.01:
            self.active.add(call_id)
            return CallResult(call_id, OUTCOME_TIMEOUT, detail="still running")
        return CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={
                "standard_interface": capability_id,
                "odom": {"x": 0.25, "y": 0.0, "yaw": 0.0},
                "minimum_range_m": 0.8,
            },
        )

    def cancel(self, call_id: str):
        if call_id in self.stop_calls:
            return CallResult(call_id, OUTCOME_REFUSED, detail="stop is not cancellable")
        if call_id not in self.active:
            return CallResult(call_id, OUTCOME_REFUSED, detail="no active goal")
        self.active.remove(call_id)
        return CallResult(call_id, OUTCOME_CANCELLED, detail="cancelled")

    def safe_stop(self, call_id: str):
        self.stop_calls.add(call_id)
        self.counts[call_id] = self.counts.get(call_id, 0) + 1
        self.active.clear()
        return CallResult(
            call_id,
            OUTCOME_COMPLETED,
            evidence={"zero_velocity_published": True},
        )

    def execution_count(self, call_id: str):
        return self.counts.get(call_id, 0)

    def observation(self):
        return dict(self.observation_payload)

    def disconnect(self):
        self.connected = False

    def reconnect(self):
        self.connected = True


def adapter(capabilities=None):
    return GenericROS2Adapter(
        backend=FakeROS2Backend(capabilities),
        resource_id="standard-turtlebot3",
    )


def test_contract_contains_only_standard_ros2_interfaces():
    text = " ".join(
        f"{kind} {name} {interface_type}"
        for kind, name, interface_type in DEFAULT_INTERFACES.values()
    )
    assert "8766" not in text
    assert "flyto.robotics" not in text
    assert "job-runner" not in text
    assert "nav2_msgs/action/NavigateToPose" in text
    assert "nav2_msgs/action/Spin" in text
    assert "nav2_msgs/action/BackUp" in text
    assert {
        "geometry_msgs/msg/Twist",
        "geometry_msgs/msg/TwistStamped",
    } == CMD_VEL_TYPES


def test_discovery_declares_only_interfaces_present_on_the_ros_graph():
    device = adapter({"motion.rotate", "motion.halt"})
    declared = device.describe()

    assert {item.capability_id for item in declared} == {
        "motion.rotate",
        "motion.halt",
    }
    assert all(item.resource_id == "standard-turtlebot3" for item in declared)
    assert all(item.executor_kind == decl.EXECUTOR_EXTERNAL_API for item in declared)
    assert all(item.approval_status == decl.STATUS_DISCOVERED for item in declared)


def test_ros_graph_discovery_does_not_self_approve_motion():
    device = adapter({"motion.advance"})
    item = device.describe()[0]

    assert item.capability_id == "motion.advance"
    assert item.approval_status == decl.STATUS_DISCOVERED
    assert item.usable is False


def test_declared_motion_bounds_are_real_units_not_gateway_catalog_guesses():
    device = adapter({"motion.advance", "motion.rotate"})
    by_id = {item.capability_id: item for item in device.describe()}

    advance = {item.name: item for item in by_id["motion.advance"].arguments}
    rotate = {item.name: item for item in by_id["motion.rotate"].arguments}
    assert advance["distance_m"].unit == "m"
    assert advance["distance_m"].maximum == 2.0
    assert rotate["yaw_radians"].unit == "rad"


def test_out_of_bounds_is_refused_before_the_ros_backend_is_called():
    device = adapter({"motion.advance"})
    device.describe()

    result = device.invoke(
        CallRequest(
            "too-far",
            "motion.advance",
            arguments={"distance_m": 20.0},
        )
    )

    assert result.outcome == OUTCOME_REFUSED
    assert device.backend.calls == []


def test_motion_refuses_close_real_world_clearance_before_backend_call():
    device = adapter({"motion.advance"})
    device.backend.observation_payload["range"]["minimum_range_m"] = 0.20
    device.describe()

    result = device.invoke(
        CallRequest(
            "blocked",
            "motion.advance",
            arguments={"distance_m": 0.25},
        )
    )

    assert result.outcome == OUTCOME_REFUSED
    assert "below the 0.350m motion safety minimum" in result.detail
    assert device.backend.calls == []


def test_navigation_requires_fresh_map_transform_before_backend_call():
    device = adapter({"motion.navigate"})
    device.backend.observation_payload["map_tf_available"] = False
    device.describe()

    result = device.invoke(
        CallRequest(
            "no-map",
            "motion.navigate",
            arguments={"x": 1.0, "y": 0.0},
        )
    )

    assert result.outcome == OUTCOME_REFUSED
    assert result.detail == "fresh map-to-odom transform is required before navigation"
    assert device.backend.calls == []


def test_motion_requires_fresh_odometry_and_lidar():
    device = adapter({"motion.advance"})
    device.describe()
    device.backend.observation_payload["pose"] = None

    no_odom = device.invoke(CallRequest("no-odom", "motion.advance", {"distance_m": 0.25}))
    assert no_odom.outcome == OUTCOME_REFUSED
    assert no_odom.detail == "fresh odometry is required before motion"

    device.backend.observation_payload["pose"] = {
        "frame": "odom", "x": 0.0, "y": 0.0, "yaw": 0.0
    }
    device.backend.observation_payload["range"] = None
    no_lidar = device.invoke(CallRequest("no-lidar", "motion.advance", {"distance_m": 0.25}))
    assert no_lidar.outcome == OUTCOME_REFUSED
    assert no_lidar.detail == "fresh LiDAR is required before motion"
    assert device.backend.calls == []


def test_same_call_id_does_not_execute_twice():
    device = adapter({"motion.advance"})
    device.describe()
    request = CallRequest(
        "same",
        "motion.advance",
        arguments={"distance_m": 0.25},
    )

    first = device.invoke(request)
    second = device.invoke(request)

    assert first.outcome == OUTCOME_COMPLETED
    assert second.outcome == OUTCOME_COMPLETED
    assert device.execution_count("same") == 1


def test_cancel_withdraws_the_adapter_owned_nav2_goal():
    device = adapter({"motion.advance", "motion.halt"})
    device.describe()

    started = device.invoke(
        CallRequest(
            "moving",
            "motion.advance",
            arguments={"distance_m": 0.25},
            deadline_seconds=0.001,
        )
    )
    cancelled = device.cancel("moving")

    assert started.outcome == OUTCOME_TIMEOUT
    assert cancelled.outcome == OUTCOME_CANCELLED


def test_motion_halt_is_zero_velocity_and_cannot_be_cancelled():
    device = adapter({"motion.halt"})
    device.describe()

    stopped = device.invoke(CallRequest("stop", "motion.halt"))
    cancelled = device.cancel("stop")

    assert stopped.outcome == OUTCOME_COMPLETED
    assert stopped.evidence["zero_velocity_published"] is True
    assert cancelled.outcome == OUTCOME_REFUSED


def test_disconnected_external_adapter_refuses_instead_of_queueing_motion():
    device = adapter({"motion.advance"})
    device.describe()
    device.disconnect()
    try:
        result = device.invoke(CallRequest("offline", "motion.advance", {"distance_m": 0.25}))
    finally:
        device.reconnect()

    assert result.outcome == OUTCOME_REFUSED


def test_adapter_observation_bundle_has_same_shape_for_simulation_and_hardware():
    device = adapter({"motion.navigate", "motion.halt"})

    hardware = device.observe(
        phase="after",
        execution_id="exec-physical-1",
        deployment_mode="hardware",
    )
    simulation = device.observe(
        phase="after",
        execution_id="exec-sim-1",
        deployment_mode="simulation",
    )

    assert set(hardware) == set(simulation)
    assert hardware["contract_version"] == simulation["contract_version"]
    assert hardware["provider"] == "ros2"
    assert simulation["provider"] == "gazebo"
    assert hardware["pose"]["frame"] == "odom"
    assert hardware["range"]["sample_count"] == 400
    assert hardware["camera"]["calibrated"] is False
    assert hardware["map_tf_available"] is True


def test_observation_runtime_snapshot_is_stable_for_same_graph():
    device = adapter({"motion.rotate", "motion.halt"})

    first = device.observe()
    device.backend.interfaces.reverse()
    second = device.observe()

    assert first["runtime_snapshot"] == second["runtime_snapshot"]


def test_adapter_has_no_dependency_on_flyto_modules_robotics_gateway():
    source = (ROOT / "flyto_robotics/generic_ros2_adapter.py").read_text()
    assert "flyto_modules_robotics" not in source
    assert "FLYTO_ROBOTICS_GATEWAY_URL" not in source
    assert "/v1/plans" not in source
