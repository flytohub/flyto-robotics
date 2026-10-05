"""An adapter says which resource it serves; it is never built for another.

Live 2026-10-05: the adapter on a computer was reconfigured from the TurtleBot3
twin (simulation) to the physical robot ``burger-01`` (hardware). A Cloud job
for ``turtlebot3-twin`` was built an adapter "for the twin" that reached the
physical robot, ran, and was recorded as the twin. A simulated movement is
auto-run, so the same path could have moved the robot with nobody asked.

Two guards close it on this side. ``build_adapter`` refuses any id but the one
configured here (``FLYTO_ROS2_RESOURCE_ID``), and ``served_identity()`` answers
the Cloud host (flyto-cloud ``local/served_resource.py``) with the configured
resource and the deployment the live graph confirms (/clock iff simulation).
"""

from __future__ import annotations

import io
import json
import sys
from contextlib import redirect_stdout

import pytest

import flyto_robotics.adapter_provider as provider
from flyto_robotics.generic_ros2_adapter import (
    CLOCK_TYPE,
    DEFAULT_INTERFACES,
    GenericROS2Adapter,
    ServedIdentityError,
    StandardInterface,
    configured_resource_id,
)

ROBOT_GRAPH = tuple(
    StandardInterface(kind, name, interface_type)
    for kind, name, interface_type in DEFAULT_INTERFACES.values()
)
TWIN_GRAPH = ROBOT_GRAPH + (StandardInterface("topic", "/clock", CLOCK_TYPE),)


class Graph:
    """A ROS graph that may change between reads, as a swapped endpoint does."""

    def __init__(self, interfaces) -> None:
        self.interfaces = tuple(interfaces)
        self.invalidated = 0

    def discover(self):
        return self.interfaces

    def invalidate_discovery(self) -> None:
        self.invalidated += 1


@pytest.fixture
def burger(monkeypatch):
    """This computer as it was live: configured for the physical burger-01."""
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "burger-01")
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "hardware")
    monkeypatch.delenv("FLYTO_ROS2_SIM_MARKER_TOPIC", raising=False)


@pytest.fixture
def twin(monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_RESOURCE_ID", "turtlebot3-twin")
    monkeypatch.setenv("FLYTO_ROS2_DEPLOYMENT_MODE", "simulation")
    monkeypatch.delenv("FLYTO_ROS2_SIM_MARKER_TOPIC", raising=False)


# --- build_adapter: only the configured resource ------------------------------


def test_the_live_case_a_twin_job_is_refused_on_a_computer_serving_burger(
    burger, monkeypatch
):
    built: list[str] = []
    monkeypatch.setattr(provider, "build", lambda rid, **_: built.append(rid))

    with pytest.raises(provider.ResourceNotServed) as refused:
        provider.build_adapter("turtlebot3-twin")

    assert "'burger-01'" in str(refused.value)
    assert "'turtlebot3-twin'" in str(refused.value)
    # Refused before any transport was opened.
    assert built == []


def test_the_configured_resource_is_built(burger, monkeypatch):
    built: list[str] = []
    monkeypatch.setattr(provider, "build", lambda rid, **_: built.append(rid) or rid)

    assert provider.build_adapter(" burger-01 ") == "burger-01"
    assert built == ["burger-01"]


@pytest.mark.parametrize("requested", ["", "   ", None])
def test_a_request_without_a_resource_id_is_refused(burger, monkeypatch, requested):
    monkeypatch.setattr(provider, "build", lambda rid, **_: pytest.fail("built"))

    with pytest.raises(provider.ResourceNotServed):
        provider.build_adapter(requested)


def test_without_a_configured_id_only_the_derived_one_is_served(monkeypatch):
    monkeypatch.delenv("FLYTO_ROS2_RESOURCE_ID", raising=False)
    monkeypatch.setattr(provider, "build", lambda rid, **_: rid)
    derived = configured_resource_id()

    assert derived.startswith("ros2-")
    assert provider.build_adapter(derived) == derived
    assert provider._resource_identity()[0] == derived
    with pytest.raises(provider.ResourceNotServed):
        provider.build_adapter("turtlebot3-twin")


def test_the_process_protocol_refuses_another_resource(burger, monkeypatch):
    monkeypatch.setattr(provider, "build", lambda rid, **_: pytest.fail("built"))
    out = io.StringIO()

    with redirect_stdout(out):
        code = provider.main(["--resource-id", "turtlebot3-twin"])

    assert code == 2
    response = json.loads(out.getvalue())
    assert response["ok"] is False
    assert "'burger-01'" in response["error"]


def test_the_process_protocol_answers_served_identity(burger, monkeypatch):
    adapter = GenericROS2Adapter(backend=Graph(ROBOT_GRAPH), resource_id="burger-01")
    monkeypatch.setattr(provider, "build", lambda rid, **_: adapter)
    monkeypatch.setattr(
        sys, "stdin", io.StringIO('{"id": 1, "op": "served_identity"}\n{"id": 2, "op": "close"}\n')
    )
    out = io.StringIO()

    with redirect_stdout(out):
        assert provider.main(["--resource-id", "burger-01"]) == 0

    first = json.loads(out.getvalue().splitlines()[0])
    assert first["ok"] is True
    assert first["result"] == {"resource_id": "burger-01", "deployment_mode": "real"}


# --- served_identity: configured resource, deployment the graph confirms ------


def test_hardware_on_a_robot_graph_is_real(burger):
    graph = Graph(ROBOT_GRAPH)
    adapter = GenericROS2Adapter(backend=graph, resource_id="burger-01")

    assert adapter.served_identity() == {"resource_id": "burger-01", "deployment_mode": "real"}
    # Read from the graph as it is now, not from a cached discovery.
    assert graph.invalidated == 1


def test_simulation_on_a_graph_with_clock_is_simulation(twin):
    adapter = GenericROS2Adapter(backend=Graph(TWIN_GRAPH), resource_id="turtlebot3-twin")

    assert adapter.served_identity() == {
        "resource_id": "turtlebot3-twin",
        "deployment_mode": "simulation",
    }


def test_the_answer_is_the_configured_resource_not_the_id_it_was_built_with(burger):
    # Built directly for the twin (bypassing build_adapter), it still says
    # burger-01, so the host's comparison with the commanded id refuses.
    adapter = GenericROS2Adapter(backend=Graph(ROBOT_GRAPH), resource_id="turtlebot3-twin")

    assert adapter.served_identity()["resource_id"] == "burger-01"


def test_configured_simulation_on_a_graph_without_clock_is_refused(twin):
    adapter = GenericROS2Adapter(backend=Graph(ROBOT_GRAPH), resource_id="turtlebot3-twin")

    with pytest.raises(ServedIdentityError, match="physical hardware"):
        adapter.served_identity()


def test_configured_hardware_on_a_graph_with_clock_is_refused(burger):
    adapter = GenericROS2Adapter(backend=Graph(TWIN_GRAPH), resource_id="burger-01")

    with pytest.raises(ServedIdentityError, match="simulator"):
        adapter.served_identity()


def test_an_empty_graph_cannot_show_what_it_reaches(burger):
    adapter = GenericROS2Adapter(backend=Graph(()), resource_id="burger-01")

    with pytest.raises(ServedIdentityError, match="no interfaces"):
        adapter.served_identity()


def test_simulation_is_never_claimed_with_the_marker_check_disabled(twin, monkeypatch):
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    adapter = GenericROS2Adapter(backend=Graph(TWIN_GRAPH), resource_id="turtlebot3-twin")

    with pytest.raises(ServedIdentityError, match="disabled"):
        adapter.served_identity()


def test_hardware_with_the_marker_check_disabled_stays_real(burger, monkeypatch):
    # A robot that legitimately publishes /clock disables the check; real is
    # the stricter class, so it is reported as real.
    monkeypatch.setenv("FLYTO_ROS2_SIM_MARKER_TOPIC", "")
    adapter = GenericROS2Adapter(backend=Graph(TWIN_GRAPH), resource_id="burger-01")

    assert adapter.served_identity() == {"resource_id": "burger-01", "deployment_mode": "real"}


def test_a_swap_behind_the_endpoint_is_seen_on_the_next_answer(twin):
    graph = Graph(TWIN_GRAPH)
    adapter = GenericROS2Adapter(backend=graph, resource_id="turtlebot3-twin")
    assert adapter.served_identity()["deployment_mode"] == "simulation"

    # The simulator is replaced by the physical robot on the same endpoint.
    graph.interfaces = ROBOT_GRAPH

    with pytest.raises(ServedIdentityError):
        adapter.served_identity()
