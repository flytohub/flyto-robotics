from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

from flyto_robotics.route_graph import RouteGraph, RouteGraphError

ROOT = Path(__file__).resolve().parents[1]
SCENARIO_FILE = ROOT / "examples/routes/ai4all-branching-routes.json"
WORLD_FILE = ROOT / "worlds/ai4all-branching-route.sdf"


def scenario() -> dict[str, object]:
    value = json.loads(SCENARIO_FILE.read_text(encoding="utf-8"))
    assert isinstance(value, dict)
    return value


def test_branch_graph_starts_with_eight_real_route_candidates() -> None:
    payload = scenario()
    graph = RouteGraph.from_mapping(payload["graph"])
    evaluation = graph.evaluate(payload["initial_resource_observations"])

    candidates = evaluation["candidates"]
    assert len(candidates) == 8
    assert candidates[0]["route_id"] == "yellow-purple"
    assert {item["attributes"]["first_branch"] for item in candidates} == {
        "yellow",
        "orange",
    }
    assert {item["attributes"]["second_branch"] for item in candidates} == {
        "blue",
        "green",
        "purple",
        "red",
    }
    assert candidates[0]["dependencies"][0]["derived_band"] == "mission_critical"


def test_camera_b_failure_excludes_all_yellow_routes_and_keeps_four_orange() -> None:
    payload = scenario()
    graph = RouteGraph.from_mapping(payload["graph"])
    observations = dict(payload["initial_resource_observations"])
    change = payload["preflight_change"]
    observations[change["resource_id"]] = change["observation"]

    evaluation = graph.evaluate(observations)

    assert [item["route_id"] for item in evaluation["candidates"]] == [
        "orange-purple",
        "orange-green",
        "orange-blue",
        "orange-red",
    ]
    assert {item["attributes"]["first_branch"] for item in evaluation["excluded"]} == {
        "yellow"
    }
    yellow = evaluation["excluded"][0]
    assert yellow["dependencies"][0]["must_stop"] is True
    assert yellow["dependencies"][0]["action"] == "safe_stop_and_escalate"


def test_route_graph_fails_closed_when_every_branch_dependency_is_unavailable() -> None:
    payload = scenario()
    unavailable = {
        resource_id: {
            "healthy": False,
            "confidence": 0.0,
            "observation_age_seconds": 0.1,
            "fallback_available": False,
            "fallback_equivalent": False,
            "fallback_validated": False,
            "evidence_present": False,
        }
        for resource_id in (
            "camera.corridor.b",
            "camera.floor1.overhead",
        )
    }
    graph_payload = payload["graph"]
    for route in graph_payload["routes"]:
        if route["attributes"]["first_branch"] == "orange":
            route["dependencies"][0]["contract"]["task_impact"] = "block"

    blocking_graph = RouteGraph.from_mapping(graph_payload)
    with pytest.raises(RouteGraphError, match="no executable route remains"):
        blocking_graph.evaluate(unavailable)


def test_route_graph_rejects_unknown_configuration_instead_of_guessing() -> None:
    payload = scenario()["graph"]
    payload["routes"][0]["dependency_level"] = "strong"

    with pytest.raises(RouteGraphError, match="unsupported fields"):
        RouteGraph.from_mapping(payload)


def test_world_contains_two_stage_forks_and_all_three_real_camera_topics() -> None:
    root = ET.parse(WORLD_FILE).getroot()
    names = {
        model.attrib["name"]
        for model in root.findall(".//model")
        if "name" in model.attrib
    }
    assert {
        "first_fork_yellow_diagonal",
        "first_fork_orange_diagonal",
        "second_fork_blue_diagonal",
        "second_fork_green_diagonal",
        "second_fork_purple_diagonal",
        "second_fork_red_diagonal",
    }.issubset(names)
    topics = {topic.text for topic in root.findall(".//sensor/topic")}
    assert topics == {
        "/flyto/evidence/overhead",
        "/flyto/evidence/zone_a",
        "/flyto/evidence/zone_b",
    }
