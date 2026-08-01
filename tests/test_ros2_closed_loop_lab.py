from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

import flyto_robotics.ros2_closed_loop_lab as lab


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
            "safety_state_topic": "/safety/emergency_stop_state",
            "emergency_stop_node": "/safety/emergency_supervisor",
            "emergency_stop_service": "/safety/emergency_stop",
            "goal_frame": "map",
            "cancel_after_displacement_m": 0.25,
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
    ],
)
def test_lab_closes_probe_authority_action_evidence_and_verdict(
    monkeypatch: pytest.MonkeyPatch,
    scenario: str,
    expected_location: str,
) -> None:
    calls: dict[str, object] = {}
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
    monkeypatch.setattr(
        lab,
        "RclpyGraphProbe",
        lambda _node: SimpleNamespace(
            external_emergency_stop_ready=lambda **_kwargs: True
        ),
    )
    monkeypatch.setattr(
        lab,
        "collect_ros2_runtime_snapshot",
        lambda *_args, **_kwargs: runtime,
    )
    monkeypatch.setattr(
        lab,
        "authorize_ros2_execution",
        lambda **_kwargs: grant,
    )

    def prepare(**kwargs: object) -> object:
        calls["location_id"] = kwargs["location_id"]
        return prepared

    monkeypatch.setattr(lab, "prepare_authorized_navigation", prepare)
    monkeypatch.setattr(
        lab,
        "execute_rclpy_navigation",
        lambda *_args, **_kwargs: outcome,
    )
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
        lambda path, value: calls.update(output=(path, value)),
    )

    report = lab.run_lab(FakeNode(scenario))

    assert calls["location_id"] == expected_location
    assert calls["output"] == ("/synthetic/evidence.json", evidence)
    assert report["verdict"]["passed"] is True


def test_load_json_rejects_non_object(tmp_path: Path) -> None:
    path = tmp_path / "value.json"
    path.write_text("[]", encoding="utf-8")
    with pytest.raises(ValueError, match="JSON object"):
        lab._load_json(str(path))
