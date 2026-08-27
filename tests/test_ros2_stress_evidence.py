from __future__ import annotations

import copy
import hashlib
import json
import os
import subprocess
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest

from flyto_robotics.contracts import StationPose
from flyto_robotics.resource_binding import load_resource_plan
from flyto_robotics.ros2_action_executor import (
    NavigationExecutionMonitor,
    prepare_authorized_navigation,
)
from flyto_robotics.ros2_execution import authorize_ros2_execution
from flyto_robotics.ros2_execution_evidence import build_ros2_execution_evidence
from flyto_robotics.ros2_pairing import (
    load_ros2_adapter_manifest,
    load_ros2_runtime_snapshot,
)
from flyto_robotics.ros2_stress_evidence import (
    ROS2_RESILIENCE_PROFILES,
    Ros2StressEvidenceError,
    build_ros2_pressure_report,
    build_ros2_resilience_report,
    build_ros2_resilience_series,
    build_ros2_stress_campaign,
    build_ros2_stress_report,
    parse_ros2_pressure_report,
    parse_ros2_resilience_report,
    parse_ros2_resilience_series,
    parse_ros2_stress_campaign,
    parse_ros2_stress_report,
    prove_expired_grant_rejected,
)

ROOT = Path(__file__).resolve().parents[1]
AT = datetime(2026, 8, 1, 10, 0, tzinfo=timezone.utc)
FAULT_REASONS = {
    "lidar_dropout": "lidar_stale",
    "odometry_freeze": "odometry_stale",
    "nav2_lifecycle_failure": "command_stale",
}
BUILD_PROVENANCE = {
    "source_snapshot": "1" * 64,
    "container_image_id": "sha256:" + "2" * 64,
    "ros_distro": "jazzy",
    "simulator": "gazebo-harmonic",
    "execution_mode": "simulation",
}


def _reseal(value: dict) -> None:
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    value["snapshot"] = hashlib.sha256(
        json.dumps(
            unsigned,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode()
    ).hexdigest()


def _runtime_hygiene(scenario_logs: int) -> dict:
    return {
        "scenario_log_count": scenario_logs,
        "expected_forced_terminations": scenario_logs,
        "unexpected_process_deaths": 0,
        "unexpected_exit_codes": [],
    }


def _authority():
    manifest = load_ros2_adapter_manifest(ROOT / "examples/ros2-adapters/flyto2-standard.json")
    runtime = load_ros2_runtime_snapshot(ROOT / "examples/ros2-runtime/ready-sim.json")
    grant = authorize_ros2_execution(
        resource_plan=load_resource_plan(
            ROOT / "examples/resource-plans/nav2-hospital-delivery.json"
        ),
        manifest=manifest,
        runtime=runtime,
        workflow_id="hospital_delivery.v1",
        resource_id="flyto-rover-sim-001",
        capability_id="robotics.motion.navigate@1",
        target_space_id="gazebo-nav2-lab",
        observed_at=AT,
    )
    semantic_map = json.loads((ROOT / "examples/maps/atomic-color-route.json").read_text())
    return manifest, runtime, grant, semantic_map


def _execution(scenario: str, index: int) -> dict:
    manifest, runtime, grant, semantic_map = _authority()
    location = "hospital.route.blue_end" if scenario == "success" else "hospital.route.yellow_end"
    prepared = prepare_authorized_navigation(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id=location,
        frame_id="map",
        observed_at=AT,
    )
    started = AT + timedelta(seconds=index)
    monitor = NavigationExecutionMonitor(
        prepared,
        StationPose("robot.current", 0.0, 0.0, 0.0),
        started_at=started,
    )
    monitor.accept_goal()
    monitor.feedback(1.0)
    if scenario == "success":
        result_code = "succeeded"
        terminal = StationPose("robot.current", 1.44, 0.0, 0.0)
    else:
        monitor.request_cancel(
            FAULT_REASONS[scenario],
            fault_injection_observed=True,
            safety_stop_latency_ms=350.0 + index,
        )
        result_code = "canceled"
        terminal = StationPose("robot.current", 0.2, 0.0, 0.0)
    finished = started + timedelta(seconds=2)
    outcome = monitor.finish(result_code, terminal, finished_at=finished)
    return build_ros2_execution_evidence(
        grant,
        prepared,
        outcome,
        scenario=scenario,
        finished_at=finished,
    )


def test_stress_report_closes_soak_faults_and_expired_grant() -> None:
    manifest, runtime, grant, semantic_map = _authority()
    probe = prove_expired_grant_rejected(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
    )
    executions = [
        _execution("success", 1),
        _execution("success", 2),
        _execution("lidar_dropout", 3),
        _execution("odometry_freeze", 4),
        _execution("nav2_lifecycle_failure", 5),
    ]

    report = build_ros2_stress_report(executions, probe, requested_soak_runs=2)

    assert report["passed"] is True
    assert report["completed_soak_runs"] == 2
    assert report["grant_expiry_rejected"] is True
    assert report["max_safety_stop_latency_ms"] == 355.0
    assert len(set(report["evidence_snapshots"])) == 5


def test_stress_report_tampering_and_missing_fault_fail_closed() -> None:
    manifest, runtime, grant, semantic_map = _authority()
    probe = prove_expired_grant_rejected(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
    )
    executions = [
        _execution("success", 1),
        _execution("lidar_dropout", 2),
        _execution("odometry_freeze", 3),
    ]
    report = build_ros2_stress_report(executions, probe, requested_soak_runs=1)
    assert report["passed"] is False
    assert (
        next(check for check in report["checks"] if check["code"] == "fault_matrix_complete")[
            "passed"
        ]
        is False
    )

    tampered = copy.deepcopy(report)
    tampered["passed"] = True
    with pytest.raises(Ros2StressEvidenceError, match="verdict"):
        parse_ros2_stress_report(tampered)


def test_stress_report_rejects_unknown_missing_and_duplicate_fields() -> None:
    report = _stress_round(success_runs=1, offset=0)

    unknown = copy.deepcopy(report)
    unknown["untrusted_extension"] = True
    with pytest.raises(Ros2StressEvidenceError, match="fields"):
        parse_ros2_stress_report(unknown)

    missing = copy.deepcopy(report)
    del missing["checks"]
    with pytest.raises(Ros2StressEvidenceError, match="fields"):
        parse_ros2_stress_report(missing)

    duplicate = copy.deepcopy(report)
    duplicate["checks"].append(copy.deepcopy(duplicate["checks"][0]))
    _reseal(duplicate)
    with pytest.raises(Ros2StressEvidenceError, match="codes must be unique"):
        parse_ros2_stress_report(duplicate)


def _stress_round(*, success_runs: int, offset: int) -> dict:
    manifest, runtime, grant, semantic_map = _authority()
    probe = prove_expired_grant_rejected(
        grant=grant,
        manifest=manifest,
        runtime=runtime,
        semantic_map=semantic_map,
        location_id="hospital.route.blue_end",
    )
    executions = [_execution("success", offset + index) for index in range(1, success_runs + 1)]
    executions.extend(
        _execution(scenario, offset + success_runs + index)
        for index, scenario in enumerate(FAULT_REASONS, start=1)
    )
    return build_ros2_stress_report(
        executions,
        probe,
        requested_soak_runs=success_runs,
    )


def test_load_l2_campaign_proves_multi_round_volume_and_thresholds() -> None:
    reports = [
        _stress_round(success_runs=10, offset=0),
        _stress_round(success_runs=10, offset=100),
    ]

    campaign = build_ros2_stress_campaign(
        reports,
        profile_id="load-l2",
        build_provenance=BUILD_PROVENANCE,
        runtime_hygiene=_runtime_hygiene(26),
    )

    assert campaign["passed"] is True
    assert campaign["pressure_level"] == 2
    assert campaign["test_type"] == "mission-load"
    assert campaign["round_count"] == 2
    assert campaign["total_success_runs"] == 20
    assert campaign["total_fault_runs"] == 6
    assert campaign["total_execution_runs"] == 26
    assert campaign["max_safety_stop_latency_ms"] == 463.0
    assert (
        next(
            check
            for check in campaign["checks"]
            if check["code"] == "execution_snapshots_unique_across_rounds"
        )["passed"]
        is True
    )


def test_campaign_fails_closed_on_weak_volume_and_tampering() -> None:
    campaign = build_ros2_stress_campaign(
        [_stress_round(success_runs=10, offset=0)],
        profile_id="load-l2",
        build_provenance=BUILD_PROVENANCE,
        runtime_hygiene=_runtime_hygiene(13),
    )
    assert campaign["passed"] is False
    assert (
        next(check for check in campaign["checks"] if check["code"] == "round_volume_met")["passed"]
        is False
    )

    tampered = copy.deepcopy(campaign)
    tampered["thresholds"]["required_success_runs"] = 1
    with pytest.raises(Ros2StressEvidenceError, match="thresholds"):
        parse_ros2_stress_campaign(tampered)

    tampered = copy.deepcopy(campaign)
    tampered["pass_rate"] = 0.5
    tampered["snapshot"] = "0" * 64
    with pytest.raises(Ros2StressEvidenceError, match="pass rate"):
        parse_ros2_stress_campaign(tampered)

    tampered = copy.deepcopy(campaign)
    tampered["build_provenance"]["execution_mode"] = "physical"
    with pytest.raises(Ros2StressEvidenceError, match="execution mode"):
        parse_ros2_stress_campaign(tampered)


def test_campaign_rejects_duplicate_round_snapshots() -> None:
    report = _stress_round(success_runs=5, offset=0)
    campaign = build_ros2_stress_campaign(
        [report, report],
        profile_id="load-l2",
        build_provenance=BUILD_PROVENANCE,
        runtime_hygiene=_runtime_hygiene(16),
    )

    assert campaign["passed"] is False
    assert next(
        check for check in campaign["checks"] if check["code"] == "report_snapshots_unique"
    )["passed"] is False


def test_campaign_fails_when_runtime_crashes_after_mission_evidence() -> None:
    reports = [_stress_round(success_runs=5, offset=index * 100) for index in range(3)]
    hygiene = _runtime_hygiene(24)
    hygiene["unexpected_process_deaths"] = 1
    hygiene["unexpected_exit_codes"] = [-11]

    campaign = build_ros2_stress_campaign(
        reports,
        profile_id="fault-l3",
        build_provenance=BUILD_PROVENANCE,
        runtime_hygiene=hygiene,
    )

    assert campaign["passed"] is False
    assert (
        next(check for check in campaign["checks"] if check["code"] == "runtime_hygiene_clean")[
            "passed"
        ]
        is False
    )


def _fault_l3_campaign() -> dict:
    reports = [_stress_round(success_runs=5, offset=index * 100) for index in range(3)]
    return build_ros2_stress_campaign(
        reports,
        profile_id="fault-l3",
        build_provenance=BUILD_PROVENANCE,
        runtime_hygiene=_runtime_hygiene(24),
    )


def _pressure_observations(**overrides) -> dict:
    observations = {
        "campaign_passed": True,
        "campaign_execution_runs": 24,
        "scenario_log_count": 24,
        "completed_scenarios": 24,
        "cpu_limit_verified": True,
        "memory_limit_verified": True,
        "max_memory_bytes": 900 * 1024 * 1024,
        "cpu_usage_usec": 5_000_000,
        "cpu_throttled_usec": 100_000,
        "oom_kill_count": 0,
        "unexpected_process_deaths": 0,
        "network_injection_verified": False,
        "recovery_verified": True,
        "elapsed_seconds": 120.0,
    }
    observations.update(overrides)
    return observations


def test_resource_pressure_report_binds_limits_and_campaign() -> None:
    report = build_ros2_pressure_report(
        _fault_l3_campaign(),
        pressure_profile_id="resource-r1",
        observations=_pressure_observations(),
    )

    assert report["passed"] is True
    assert report["mode"] == "resource"
    assert report["limits"]["cpu_limit_millicores"] == 1500
    assert report["limits"]["memory_limit_mib"] == 2048


def test_pressure_report_fails_closed_on_oom_or_missing_network_injection() -> None:
    resource = build_ros2_pressure_report(
        _fault_l3_campaign(),
        pressure_profile_id="resource-r1",
        observations=_pressure_observations(oom_kill_count=1),
    )
    assert resource["passed"] is False
    assert (
        next(check for check in resource["checks"] if check["code"] == "no_oom_kills")["passed"]
        is False
    )

    network = build_ros2_pressure_report(
        _fault_l3_campaign(),
        pressure_profile_id="network-n1",
        observations=_pressure_observations(network_injection_verified=False),
    )
    assert network["passed"] is False
    assert (
        next(
            check
            for check in network["checks"]
            if check["code"] == "network_injection_state_matches"
        )["passed"]
        is False
    )


def test_pressure_report_rejects_resealed_limit_tampering() -> None:
    report = build_ros2_pressure_report(
        _fault_l3_campaign(),
        pressure_profile_id="resource-r1",
        observations=_pressure_observations(),
    )
    tampered = copy.deepcopy(report)
    tampered["limits"]["memory_limit_mib"] = 8192
    unsigned = {key: item for key, item in tampered.items() if key != "snapshot"}
    tampered["snapshot"] = (
        __import__("hashlib")
        .sha256(
            json.dumps(
                unsigned,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ).encode()
        )
        .hexdigest()
    )

    with pytest.raises(Ros2StressEvidenceError, match="limits"):
        parse_ros2_pressure_report(tampered)


def test_pressure_report_recomputes_verdict_from_raw_observations() -> None:
    report = build_ros2_pressure_report(
        _fault_l3_campaign(),
        pressure_profile_id="resource-r1",
        observations=_pressure_observations(),
    )
    tampered = copy.deepcopy(report)
    tampered["observations"]["oom_kill_count"] = 1
    _reseal(tampered)

    with pytest.raises(Ros2StressEvidenceError, match="checks"):
        parse_ros2_pressure_report(tampered)


def _resilience_report(
    test_id: str,
    *,
    metric_overrides: dict[str, float] | None = None,
    omit_event: str | None = None,
) -> dict:
    profile = ROS2_RESILIENCE_PROFILES[test_id]
    run_count = int(profile["minimum_runs"])
    metrics = {
        name: float(threshold) for name, (_comparator, threshold) in profile["metrics"].items()
    }
    if metric_overrides:
        metrics.update(metric_overrides)
    events = [event for event in profile["required_events"] if event != omit_event]
    return build_ros2_resilience_report(
        test_id=test_id,
        source_snapshot="3" * 64,
        container_image_id="sha256:" + "4" * 64,
        started_at="2026-08-02T00:00:00Z",
        finished_at="2026-08-02T06:00:00Z",
        run_summaries=[
            {
                "run": index,
                "run_id": f"{test_id}-{index:03d}",
                "passed": True,
                "snapshot": __import__("hashlib").sha256(f"{test_id}:{index}".encode()).hexdigest(),
                "duration_seconds": 30.0,
            }
            for index in range(1, run_count + 1)
        ],
        timeline=[
            {
                "sequence": index,
                "event": event,
                "at": (datetime(2026, 8, 2, tzinfo=timezone.utc) + timedelta(minutes=index))
                .isoformat()
                .replace("+00:00", "Z"),
            }
            for index, event in enumerate(events, start=1)
        ],
        metrics=metrics,
        artifacts=[
            {
                "kind": kind,
                "path": f"results/{test_id}/{kind}.json",
                "sha256": __import__("hashlib").sha256(f"{test_id}:{kind}".encode()).hexdigest(),
                "bytes": 128,
            }
            for kind in sorted(profile["artifact_kinds"])
        ],
    )


def test_resilience_profiles_close_008_through_012_and_series() -> None:
    reports = [_resilience_report(test_id) for test_id in ROS2_RESILIENCE_PROFILES]

    assert all(report["passed"] is True for report in reports)
    assert [report["episode"] for report in reports] == [
        "#008",
        "#009",
        "#010",
        "#011",
        "#012",
    ]
    series = build_ros2_resilience_series(reports)
    assert series["complete"] is True
    assert series["all_passed"] is True
    assert series["failed_reports"] == []
    assert parse_ros2_resilience_series(series) == series


def test_resilience_report_preserves_fail_and_rejects_resealed_threshold() -> None:
    report = _resilience_report(
        "runtime-network-r2",
        metric_overrides={"safety_stop_latency_ms": 651.0},
    )
    assert report["passed"] is False
    assert (
        next(check for check in report["checks"] if check["code"] == "metric_thresholds_met")[
            "passed"
        ]
        is False
    )

    reports = [
        report if test_id == "runtime-network-r2" else _resilience_report(test_id)
        for test_id in ROS2_RESILIENCE_PROFILES
    ]
    series = build_ros2_resilience_series(reports)
    assert series["complete"] is True
    assert series["all_passed"] is False
    assert series["failed_reports"] == ["runtime-network-r2"]

    tampered = copy.deepcopy(report)
    tampered["thresholds"]["safety_stop_latency_ms"]["value"] = 9999.0
    unsigned = {key: item for key, item in tampered.items() if key != "snapshot"}
    tampered["snapshot"] = (
        __import__("hashlib")
        .sha256(
            json.dumps(
                unsigned,
                ensure_ascii=False,
                separators=(",", ":"),
                sort_keys=True,
            ).encode()
        )
        .hexdigest()
    )
    with pytest.raises(Ros2StressEvidenceError, match="thresholds"):
        parse_ros2_resilience_report(tampered)


def test_resilience_report_fails_closed_on_missing_event_and_unsafe_artifact() -> None:
    report = _resilience_report(
        "compound-chaos-c1",
        omit_event="safety_stop_observed",
    )
    assert report["passed"] is False
    assert (
        next(check for check in report["checks"] if check["code"] == "required_events_observed")[
            "passed"
        ]
        is False
    )

    profile = ROS2_RESILIENCE_PROFILES["runtime-network-r2"]
    with pytest.raises(Ros2StressEvidenceError, match="artifact path"):
        build_ros2_resilience_report(
            test_id="runtime-network-r2",
            source_snapshot="3" * 64,
            container_image_id="sha256:" + "4" * 64,
            started_at="2026-08-02T00:00:00Z",
            finished_at="2026-08-02T01:00:00Z",
            run_summaries=_resilience_report("runtime-network-r2")["run_summaries"],
            timeline=_resilience_report("runtime-network-r2")["timeline"],
            metrics={
                name: float(threshold)
                for name, (_comparator, threshold) in profile["metrics"].items()
            },
            artifacts=[
                {
                    "kind": "raw_log",
                    "path": "../secret.log",
                    "sha256": "5" * 64,
                    "bytes": 1,
                }
            ],
        )


@pytest.mark.parametrize(
    ("environment", "message"),
    [
        ({"FLYTO_ROBOTICS_STRESS_RUN_ID": "../escape"}, "unsafe characters"),
        ({"FLYTO_ROBOTICS_STRESS_PROFILE": "unknown"}, "Unsupported"),
        ({"FLYTO_ROBOTICS_PRESSURE_PROFILE": "unknown"}, "Unsupported"),
        ({"FLYTO_ROBOTICS_STRESS_SOAK_RUNS": "101"}, "between 1 and 100"),
        (
            {
                "FLYTO_ROBOTICS_STRESS_PROFILE": "baseline-l1",
                "FLYTO_ROBOTICS_STRESS_SOAK_RUNS": "5",
            },
            "owns its soak count",
        ),
        (
            {
                "FLYTO_ROBOTICS_STRESS_SOAK_RUNS": "5",
                "FLYTO_ROBOTICS_ROS_DOMAIN_ID": "230",
            },
            "range cannot fit",
        ),
    ],
)
def test_stress_runner_rejects_unbounded_inputs_before_docker(
    environment: dict[str, str], message: str
) -> None:
    completed = subprocess.run(
        ["bash", str(ROOT / "scripts/run_nav2_stress.sh")],
        cwd=ROOT,
        env={**os.environ, **environment},
        text=True,
        capture_output=True,
        check=False,
    )

    assert completed.returncode == 2
    assert message in completed.stderr
