from __future__ import annotations

from datetime import datetime, timezone
from types import SimpleNamespace

import pytest

from flyto_robotics import ros2_closed_loop_lab as lab


class FakeNode:
    def __init__(self, scenario: str) -> None:
        self.values = {
            "manifest_file": "/synthetic/manifest.json",
            "resource_plan_file": "/synthetic/resources.json",
            "semantic_map_file": "/synthetic/map.json",
            "semantic_map_id": "synthetic.map.v1",
            "scenario": scenario,
            "output_file": "/synthetic/evidence.json",
            "odometry_topic": "/flyto/odom",
            "lidar_topic": "/flyto/scan",
            "safety_state_topic": "/safety/emergency_stop_state",
            "emergency_stop_node": "/safety/emergency_supervisor",
            "emergency_stop_service": "/safety/emergency_stop",
            "goal_frame": "map",
            "cancel_after_displacement_m": 0.25,
            "discovery_timeout_seconds": 60.0,
            "sensor_timeout_seconds": 0.55,
        }

    def declare_parameter(self, name: str, default: object) -> None:
        self.values.setdefault(name, default)

    def get_parameter(self, name: str) -> SimpleNamespace:
        return SimpleNamespace(value=self.values[name])


@pytest.mark.parametrize(
    ("scenario", "expected_location"),
    [
        ("success", "hospital.route.blue_end"),
        ("cancel", "hospital.route.yellow_end"),
        ("emergency_stop", "hospital.route.yellow_end"),
        ("lidar_dropout", "hospital.route.yellow_end"),
        ("odometry_freeze", "hospital.route.yellow_end"),
        ("nav2_lifecycle_failure", "hospital.route.yellow_end"),
    ],
)
def test_lab_closes_probe_authority_action_evidence_and_verdict(
    monkeypatch: pytest.MonkeyPatch,
    scenario: str,
    expected_location: str,
) -> None:
    calls: dict[str, object] = {}
    sequence: list[str] = []
    manifest = {"robot_id": "flyto-rover-sim-001"}
    runtime = {"runtime": "ready"}
    grant = {"grant": "authorized"}
    prepared = SimpleNamespace(location_id=expected_location)
    outcome = SimpleNamespace(status="terminal")
    evidence = {"snapshot": "a" * 64}

    monkeypatch.setattr(lab, "load_ros2_adapter_manifest", lambda _path: manifest)
    monkeypatch.setattr(lab, "load_resource_plan", lambda _path: {"plan": True})
    monkeypatch.setattr(
        lab,
        "_load_json",
        lambda _path: {"map_id": "synthetic.map.v1"},
    )
    graph_probe = SimpleNamespace(close=lambda: calls.update(graph_probe_closed=True))
    monkeypatch.setattr(lab, "RclpyGraphProbe", lambda _node: graph_probe)
    observed_at = datetime(2026, 8, 2, tzinfo=timezone.utc)

    monkeypatch.setattr(
        lab,
        "_start_navigation_lifecycle",
        lambda _node, _client, *, deadline: (
            sequence.append("lifecycle_started") if deadline > 0 else None
        ),
    )

    def collect_ready(*_args: object, **kwargs: object) -> object:
        calls["runtime_timeout_seconds"] = kwargs["discovery_timeout_seconds"]
        return runtime, observed_at

    monkeypatch.setattr(lab, "_collect_ready_runtime_snapshot", collect_ready)
    monkeypatch.setattr(
        lab,
        "authorize_ros2_execution",
        lambda **_kwargs: grant,
    )

    def prepare(**kwargs: object) -> object:
        calls["location_id"] = kwargs["location_id"]
        return prepared

    monkeypatch.setattr(lab, "prepare_authorized_navigation", prepare)

    def execute(*_args: object, **kwargs: object) -> object:
        calls["execution_lidar_topic"] = kwargs["lidar_topic"]
        calls["execution_sensor_timeout_seconds"] = kwargs["sensor_timeout_seconds"]
        return outcome

    monkeypatch.setattr(lab, "execute_rclpy_navigation", execute)
    monkeypatch.setattr(
        lab,
        "build_ros2_execution_evidence",
        lambda *_args, **_kwargs: evidence,
    )
    monkeypatch.setattr(
        lab,
        "evaluate_closed_loop_evidence",
        lambda *_args, **_kwargs: {"passed": True, "scenario": scenario},
    )
    monkeypatch.setattr(
        lab,
        "write_json_atomic",
        lambda path, value: (
            sequence.append("evidence_written"),
            calls.update(output=(path, value)),
        ),
    )
    monkeypatch.setattr(
        lab,
        "_prepare_lifecycle_shutdown_clients",
        lambda _clients: sequence.append("shutdown_ready"),
    )
    monkeypatch.setattr(
        lab,
        "_shutdown_lifecycle_managers",
        lambda _node, _clients: sequence.append("lifecycle_shutdown"),
    )

    manager_clients = [(lab.NAVIGATION_MANAGER_SERVICE, object())]
    report = lab.run_lab(FakeNode(scenario), shutdown_clients=manager_clients)

    assert calls["location_id"] == expected_location
    assert float(calls["runtime_timeout_seconds"]) <= 60.0
    assert float(calls["runtime_timeout_seconds"]) == pytest.approx(60.0, abs=0.01)
    assert calls["execution_lidar_topic"] == "/flyto/scan"
    assert calls["execution_sensor_timeout_seconds"] == 0.55
    assert calls["graph_probe_closed"] is True
    assert calls["output"] == ("/synthetic/evidence.json", evidence)
    assert sequence == [
        "lifecycle_started",
        "shutdown_ready",
        "evidence_written",
        "lifecycle_shutdown",
    ]
    assert report["verdict"]["passed"] is True
    assert report["task_completion_eligible"] is False


def test_discovery_retries_transient_unready_snapshot_within_budget(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    attempts: list[float] = []
    runtimes = [{"ready": False}, {"ready": True}]
    monkeypatch.setattr(
        lab,
        "collect_ros2_runtime_snapshot",
        lambda *_args, **kwargs: attempts.append(kwargs["timeout_seconds"]) or runtimes.pop(0),
    )
    monkeypatch.setattr(
        lab,
        "verify_ros2_pairing",
        lambda _manifest, runtime, **_kwargs: {"passed": runtime["ready"]},
    )
    spins: list[float] = []
    monkeypatch.setattr(
        lab,
        "_spin_discovery_once",
        lambda _node, timeout: spins.append(timeout),
    )

    runtime, _observed_at = lab._collect_ready_runtime_snapshot(
        object(),
        {"robot_id": "flyto-rover-sim-001"},
        object(),  # type: ignore[arg-type]
        emergency_stop_node="/safety/emergency_supervisor",
        emergency_stop_service="/safety/emergency_stop",
        discovery_timeout_seconds=30.0,
    )

    assert runtime == {"ready": True}
    assert attempts == [2.0, 2.0]
    assert spins == [0.1]


@pytest.mark.parametrize("timeout", [4.9, 60.1])
def test_lab_rejects_unsafe_discovery_timeout(
    monkeypatch: pytest.MonkeyPatch,
    timeout: float,
) -> None:
    node = FakeNode("success")
    node.values["discovery_timeout_seconds"] = timeout
    monkeypatch.setattr(
        lab,
        "load_ros2_adapter_manifest",
        lambda _path: {"robot_id": "flyto-rover-sim-001"},
    )
    monkeypatch.setattr(lab, "load_resource_plan", lambda _path: {"plan": True})
    monkeypatch.setattr(
        lab,
        "_load_json",
        lambda _path: {"map_id": "synthetic.map.v1"},
    )
    with pytest.raises(ValueError, match="discovery_timeout_seconds"):
        lab.run_lab(node, shutdown_clients=[])
