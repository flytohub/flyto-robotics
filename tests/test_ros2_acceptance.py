from __future__ import annotations

import copy
import hashlib
import json
import shutil
from pathlib import Path

import pytest

from flyto_robotics.ros2_acceptance import (
    ORACLE_SOURCE_PATHS,
    ROS2_ACCEPTANCE_PROFILES,
    Ros2AcceptanceError,
    build_ros2_acceptance_report,
    classify_process_deaths,
    parse_ros2_acceptance_report,
    verify_ros2_acceptance_report,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
IMAGE_ID = "sha256:" + "a" * 64


def _snapshot(value: dict[str, object]) -> str:
    return hashlib.sha256(
        json.dumps(value, separators=(",", ":"), sort_keys=True).encode()
    ).hexdigest()


def _evidence(scenario: str) -> dict[str, object]:
    fault = scenario == "lidar_dropout"
    evidence: dict[str, object] = {
        "adapter_id": "ros2.nav2.navigate_to_pose.v1",
        "cancel_reason": "lidar_stale" if fault else None,
        "cancel_requested": fault,
        "capability_id": "robotics.motion.navigate@1",
        "contract_version": "flyto.robotics.ros2-execution-evidence.v1",
        "displacement_m": 0.2,
        "duration_seconds": 5.0,
        "event_codes": [
            "authority_validated",
            "goal_accepted",
            "feedback_observed",
            "post_stop_observed",
        ],
        "execution_id": "ros2-exec-synthetic-001",
        "fault_injection_observed": fault,
        "feedback_count": 4,
        "final_pose": {"x": 0.2, "y": 0.0, "yaw": 0.0},
        "finished_at": "2026-08-03T00:00:05Z",
        "goal_accepted": True,
        "goal_error_m": 0.01 if not fault else 2.8,
        "goal_frame": "map",
        "grant_snapshot": "1" * 64,
        "initial_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
        "post_stop_drift_m": 0.01,
        "profile_snapshot": "2" * 64,
        "resource_id": "flyto-rover-sim-001",
        "resource_plan_snapshot": "3" * 64,
        "result_code": "canceled" if fault else "succeeded",
        "robot_id": "flyto-rover-sim-001",
        "runtime_snapshot": "4" * 64,
        "safety_stop_latency_ms": 500.0 if fault else None,
        "safety_stop_observed": fault,
        "safety_stop_reason": "lidar_stale" if fault else None,
        "scenario": scenario,
        "semantic_location_id": "hospital.route.synthetic",
        "semantic_map_id": "gazebo.synthetic.v1",
        "started_at": "2026-08-03T00:00:00Z",
        "status": "safety_stopped" if fault else "succeeded",
        "target_space_id": "gazebo-nav2-lab",
        "workflow_id": "hospital_delivery.v1",
    }
    evidence["snapshot"] = _snapshot(evidence)
    return evidence


def _copy_oracle(root: Path) -> None:
    for relative in ORACLE_SOURCE_PATHS:
        source = PROJECT_ROOT / relative
        target = root / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def _write_run(output: Path, run_id: str, scenario: str) -> None:
    output.mkdir(parents=True, exist_ok=True)
    (output / f"{run_id}.json").write_text(
        json.dumps(_evidence(scenario), indent=2) + "\n",
        encoding="utf-8",
    )
    (output / f"{run_id}.log").write_text(
        "\n".join(
            (
                "[lifecycle_manager-14] [INFO] [lifecycle_manager_navigation]: "
                "Managed nodes have been shut down",
                "[lifecycle_manager-13] [INFO] [map_lifecycle_manager]: "
                "Managed nodes have been shut down",
                "[ERROR] [gazebo-1]: process has died [pid 37, exit code -15, cmd 'gz'].",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    (output / f"{run_id}.pressure").write_text(
        "memory_peak=100\n"
        "usage_usec=200\n"
        "throttled_usec=0\n"
        "oom_kill=0\n"
        "scenario_exit_code=0\n",
        encoding="utf-8",
    )


def _repository(tmp_path: Path) -> Path:
    root = tmp_path / "repo"
    _copy_oracle(root)
    return root


def _complete_profile(root: Path, profile_id: str) -> Path:
    profile = ROS2_ACCEPTANCE_PROFILES[profile_id]
    output = root / "results" / "ros2-resilience" / "control"
    prefix = "success" if profile["scenario"] == "success" else "lidar-dropout"
    for index in range(1, int(profile["expected_runs"]) + 1):
        _write_run(output, f"{prefix}-{index:03d}", str(profile["scenario"]))
    return output


def _report(root: Path, output: Path, profile_id: str) -> dict[str, object]:
    return build_ros2_acceptance_report(
        root=root,
        output_dir=output,
        profile_id=profile_id,
        control_id="synthetic-control",
        container_image_id=IMAGE_ID,
    )


def test_acceptance_profiles_own_their_run_volume_and_limits() -> None:
    assert ROS2_ACCEPTANCE_PROFILES["shutdown-smoke"]["expected_runs"] == 3
    assert ROS2_ACCEPTANCE_PROFILES["shutdown-soak"]["expected_runs"] == 50
    assert ROS2_ACCEPTANCE_PROFILES["lidar-fault-proof"] == {
        "scenario": "lidar_dropout",
        "expected_runs": 1,
        "max_post_stop_drift_m": 0.04,
        "max_safety_stop_latency_ms": 600.0,
        "min_displacement_m": 0.05,
        "min_feedback_count": 1,
        "expected_status": "safety_stopped",
        "expected_safety_reason": "lidar_stale",
        "fault_injection_required": True,
    }


def test_complete_shutdown_smoke_rebuilds_and_verifies(tmp_path: Path) -> None:
    root = _repository(tmp_path)
    output = _complete_profile(root, "shutdown-smoke")

    report = _report(root, output, "shutdown-smoke")

    assert report["passed"] is True
    assert verify_ros2_acceptance_report(root, report) == report
    assert all(run["passed"] for run in report["run_summaries"])
    receipt = report["run_summaries"][0]["execution_receipt"]
    assert receipt["task_completion_eligible"] is False
    assert receipt["evidence_snapshot"] == report["run_summaries"][0]["evidence_snapshot"]


def test_missing_run_is_preserved_as_fail_not_redefined_volume(tmp_path: Path) -> None:
    root = _repository(tmp_path)
    output = root / "results" / "ros2-resilience" / "partial"
    _write_run(output, "success-001", "success")
    _write_run(output, "success-002", "success")

    report = _report(root, output, "shutdown-smoke")

    assert report["expected_runs"] == 3
    assert report["passed"] is False
    assert report["run_summaries"][2]["passed"] is False
    assert parse_ros2_acceptance_report(report) == report


def test_threshold_and_verdict_tampering_fail_even_when_resealed(tmp_path: Path) -> None:
    root = _repository(tmp_path)
    report = _report(root, _complete_profile(root, "shutdown-smoke"), "shutdown-smoke")
    tampered = copy.deepcopy(report)
    tampered["thresholds"]["max_post_stop_drift_m"] = 1.0
    tampered["snapshot"] = _snapshot(
        {key: value for key, value in tampered.items() if key != "snapshot"}
    )

    with pytest.raises(Ros2AcceptanceError, match="thresholds"):
        parse_ros2_acceptance_report(tampered)

    tampered = copy.deepcopy(report)
    tampered["passed"] = False
    tampered["snapshot"] = _snapshot(
        {key: value for key, value in tampered.items() if key != "snapshot"}
    )
    with pytest.raises(Ros2AcceptanceError, match="verdict"):
        parse_ros2_acceptance_report(tampered)

    tampered = copy.deepcopy(report)
    tampered["run_summaries"][0]["execution_receipt"][
        "task_completion_eligible"
    ] = True
    tampered["snapshot"] = _snapshot(
        {key: value for key, value in tampered.items() if key != "snapshot"}
    )
    with pytest.raises(Ros2AcceptanceError, match="checks"):
        parse_ros2_acceptance_report(tampered)


def test_raw_artifact_or_oracle_change_invalidates_old_report(tmp_path: Path) -> None:
    root = _repository(tmp_path)
    output = _complete_profile(root, "shutdown-smoke")
    report = _report(root, output, "shutdown-smoke")

    with (output / "success-001.log").open("a", encoding="utf-8") as stream:
        stream.write("tampered\n")
    with pytest.raises(Ros2AcceptanceError, match="does not match"):
        verify_ros2_acceptance_report(root, report)

    report = _report(root, output, "shutdown-smoke")
    oracle = root / "scripts" / "run_ros2_fault_proof.py"
    with oracle.open("a", encoding="utf-8") as stream:
        stream.write("\n# tampered\n")
    with pytest.raises(Ros2AcceptanceError, match="does not match"):
        verify_ros2_acceptance_report(root, report)


def test_process_death_classifier_only_allows_bounded_gazebo_teardown(
    tmp_path: Path,
) -> None:
    text = (
        "[ERROR] [gazebo-1]: process has died [pid 1, exit code -15, cmd 'gz'].\n"
        "[ERROR] [controller_server-7]: process has died "
        "[pid 2, exit code -9, cmd 'controller'].\n"
    )
    deaths = classify_process_deaths(text)
    assert deaths["expected"] == [{"process": "gazebo-1", "exit_code": -15}]
    assert deaths["unexpected"] == [
        {"process": "controller_server-7", "exit_code": -9}
    ]

    root = _repository(tmp_path)
    output = _complete_profile(root, "shutdown-smoke")
    with (output / "success-001.log").open("a", encoding="utf-8") as stream:
        stream.write(text.splitlines()[1] + "\n")
    report = _report(root, output, "shutdown-smoke")
    assert report["passed"] is False
    assert report["run_summaries"][0]["unexpected_process_deaths"]


def test_lidar_fault_proof_requires_latency_reason_and_clean_shutdown(
    tmp_path: Path,
) -> None:
    root = _repository(tmp_path)
    output = _complete_profile(root, "lidar-fault-proof")
    report = _report(root, output, "lidar-fault-proof")
    assert report["passed"] is True

    evidence_path = output / "lidar-dropout-001.json"
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    evidence["safety_stop_latency_ms"] = 600.1
    evidence["snapshot"] = _snapshot(
        {key: value for key, value in evidence.items() if key != "snapshot"}
    )
    evidence_path.write_text(json.dumps(evidence) + "\n", encoding="utf-8")
    failed = _report(root, output, "lidar-fault-proof")
    assert failed["passed"] is False
    assert next(
        check
        for check in failed["run_summaries"][0]["checks"]
        if check["code"] == "safety_stop_latency"
    )["passed"] is False

