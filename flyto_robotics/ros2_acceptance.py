"""Repository-owned adjudicator for targeted ROS 2 acceptance controls.

Generated reports are not trusted inputs.  A report is accepted only when this
module can rebuild it from the fixed-profile raw evidence, log, and cgroup
pressure files under ``results/ros2-resilience``.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from .ros2_execution_evidence import parse_ros2_execution_evidence

ROS2_ACCEPTANCE_REPORT_VERSION = "flyto.robotics.ros2-acceptance-report.v1"
ROS2_ACCEPTANCE_PROFILES: dict[str, dict[str, Any]] = {
    "shutdown-smoke": {
        "scenario": "success",
        "expected_runs": 3,
        "max_post_stop_drift_m": 0.03,
        "max_safety_stop_latency_ms": None,
        "min_displacement_m": 0.05,
        "min_feedback_count": 1,
        "expected_status": "succeeded",
        "expected_safety_reason": None,
        "fault_injection_required": False,
    },
    "shutdown-soak": {
        "scenario": "success",
        "expected_runs": 50,
        "max_post_stop_drift_m": 0.03,
        "max_safety_stop_latency_ms": None,
        "min_displacement_m": 0.05,
        "min_feedback_count": 1,
        "expected_status": "succeeded",
        "expected_safety_reason": None,
        "fault_injection_required": False,
    },
    "lidar-fault-proof": {
        "scenario": "lidar_dropout",
        "expected_runs": 1,
        "max_post_stop_drift_m": 0.04,
        "max_safety_stop_latency_ms": 600.0,
        "min_displacement_m": 0.05,
        "min_feedback_count": 1,
        "expected_status": "safety_stopped",
        "expected_safety_reason": "lidar_stale",
        "fault_injection_required": True,
    },
}

ORACLE_SOURCE_PATHS = (
    "flyto_robotics/ros2_acceptance.py",
    "flyto_robotics/ros2_closed_loop_lab.py",
    "flyto_robotics/ros2_execution_evidence.py",
    "scripts/ros2_execution_observer.py",
    "scripts/run_ros2_acceptance_mutations.py",
    "scripts/run_ros2_fault_proof.py",
    "tests/test_ros2_closed_loop_lab.py",
)

_SOURCE_EXCLUDED_PARTS = {
    ".flyto-index",
    ".git",
    ".pytest_cache",
    ".ruff_cache",
    "__pycache__",
    "build",
    "install",
    "log",
    "output",
    "results",
    "tmp",
}
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_IMAGE_ID = re.compile(r"^sha256:[0-9a-f]{64}$")
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")
_DEATH_PATTERN = re.compile(
    r"\[ERROR\] \[(?P<process>[^]]+)\]: process has died .*?"
    r"exit code (?P<code>-?\d+)"
)
_ANSI = re.compile(r"\x1b\[[0-9;]*m")
_PRESSURE_FIELDS = {
    "memory_peak",
    "usage_usec",
    "throttled_usec",
    "oom_kill",
    "scenario_exit_code",
}
_ARTIFACT_FIELDS = {"path", "sha256", "bytes"}
_RUN_FIELDS = {
    "index",
    "run_id",
    "scenario",
    "artifacts",
    "evidence_valid",
    "evidence_snapshot",
    "status",
    "safety_stop_reason",
    "fault_injection_observed",
    "safety_stop_latency_ms",
    "displacement_m",
    "post_stop_drift_m",
    "goal_accepted",
    "feedback_count",
    "lifecycle_managers_shutdown",
    "expected_process_deaths",
    "unexpected_process_deaths",
    "oom_kill_count",
    "scenario_exit_code",
    "execution_receipt",
    "checks",
    "passed",
}
_RECEIPT_FIELDS = {
    "contract_version",
    "run_id",
    "scenario",
    "status",
    "evidence_snapshot",
    "artifact_sha256",
    "task_completion_eligible",
    "receipt_sha256",
}
_REPORT_FIELDS = {
    "contract_version",
    "report_id",
    "control_id",
    "profile_id",
    "execution_mode",
    "source_snapshot",
    "oracle_snapshot",
    "container_image_id",
    "artifact_root",
    "observed_artifact_names",
    "expected_runs",
    "thresholds",
    "run_summaries",
    "checks",
    "passed",
    "snapshot",
}


class Ros2AcceptanceError(ValueError):
    """Raised when a targeted acceptance report cannot be trusted."""


def _canonical_digest(value: Any) -> str:
    encoded = json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode()
    return hashlib.sha256(encoded).hexdigest()


def _digest_files(root: Path, relative_paths: list[str] | tuple[str, ...]) -> str:
    digest = hashlib.sha256()
    for relative in sorted(relative_paths):
        path = root / relative
        if not path.is_file() or path.is_symlink():
            raise Ros2AcceptanceError(f"source file is missing or unsafe: {relative}")
        digest.update(relative.encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def repository_source_snapshot(root: Path) -> str:
    """Hash the source tree while excluding generated and crash output."""

    root = root.resolve()
    relative_paths: list[str] = []
    for path in root.rglob("*"):
        if not path.is_file() or path.is_symlink():
            continue
        relative = path.relative_to(root)
        if any(part in _SOURCE_EXCLUDED_PARTS for part in relative.parts):
            continue
        if relative.name == "core" or relative.name.startswith("core."):
            continue
        relative_paths.append(relative.as_posix())
    return _digest_files(root, relative_paths)


def acceptance_oracle_snapshot(root: Path) -> str:
    """Hash every tracked source file that can issue or test this verdict."""

    return _digest_files(root.resolve(), ORACLE_SOURCE_PATHS)


def classify_process_deaths(log_text: str) -> dict[str, list[dict[str, int | str]]]:
    """Separate the one expected bounded-teardown death from real failures."""

    expected: list[dict[str, int | str]] = []
    unexpected: list[dict[str, int | str]] = []
    for match in _DEATH_PATTERN.finditer(_ANSI.sub("", log_text)):
        death: dict[str, int | str] = {
            "process": match.group("process"),
            "exit_code": int(match.group("code")),
        }
        if str(death["process"]).startswith("gazebo-") and death["exit_code"] == -15:
            expected.append(death)
        else:
            unexpected.append(death)
    return {"expected": expected, "unexpected": unexpected}


def _lifecycle_shutdowns(log_text: str) -> list[str]:
    clean = _ANSI.sub("", log_text)
    required = ("lifecycle_manager_navigation", "map_lifecycle_manager")
    return [
        manager
        for manager in required
        if any(
            f"[{manager}]" in line and "Managed nodes have been shut down" in line
            for line in clean.splitlines()
        )
    ]


def _safe_artifact_root(root: Path, output_dir: Path) -> tuple[Path, str]:
    root = root.resolve()
    output_dir = output_dir.resolve()
    allowed = (root / "results" / "ros2-resilience").resolve()
    if output_dir == allowed or allowed not in output_dir.parents:
        raise Ros2AcceptanceError(
            "acceptance artifacts must use a child of results/ros2-resilience"
        )
    return output_dir, output_dir.relative_to(root).as_posix()


def _artifact_record(root: Path, path: Path) -> dict[str, Any] | None:
    if not path.is_file() or path.is_symlink():
        return None
    resolved = path.resolve()
    if root.resolve() not in resolved.parents:
        raise Ros2AcceptanceError("artifact escapes the repository")
    return {
        "path": resolved.relative_to(root.resolve()).as_posix(),
        "sha256": hashlib.sha256(resolved.read_bytes()).hexdigest(),
        "bytes": resolved.stat().st_size,
    }


def _read_pressure(path: Path) -> dict[str, int] | None:
    if not path.is_file() or path.is_symlink():
        return None
    values: dict[str, int] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if not separator or key in values or key not in _PRESSURE_FIELDS:
            return None
        if re.fullmatch(r"-?\d+", value) is None:
            return None
        values[key] = int(value)
    if set(values) != _PRESSURE_FIELDS:
        return None
    if any(values[key] < 0 for key in _PRESSURE_FIELDS - {"scenario_exit_code"}):
        return None
    return values


def _read_evidence(path: Path) -> dict[str, Any] | None:
    if not path.is_file() or path.is_symlink():
        return None
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return parse_ros2_execution_evidence(value)
    except (OSError, ValueError, json.JSONDecodeError):
        return None


def _run_id(profile: Mapping[str, Any], index: int) -> str:
    prefix = "success" if profile["scenario"] == "success" else "lidar-dropout"
    return f"{prefix}-{index:03d}"


def _expected_artifact_names(profile: Mapping[str, Any]) -> set[str]:
    names: set[str] = set()
    for index in range(1, int(profile["expected_runs"]) + 1):
        run_id = _run_id(profile, index)
        names.update({f"{run_id}.json", f"{run_id}.log", f"{run_id}.pressure"})
    return names


def _run_checks(summary: Mapping[str, Any], profile: Mapping[str, Any]) -> dict[str, bool]:
    latency_limit = profile["max_safety_stop_latency_ms"]
    expected_reason = profile["expected_safety_reason"]
    return {
        "raw_artifacts_complete": all(
            summary["artifacts"].get(kind) is not None for kind in ("evidence", "log", "pressure")
        ),
        "strict_evidence_valid": summary["evidence_valid"] is True,
        "terminal_status": summary["status"] == profile["expected_status"],
        "safety_reason": summary["safety_stop_reason"] == expected_reason,
        "fault_observation": (
            summary["fault_injection_observed"] is profile["fault_injection_required"]
        ),
        "safety_stop_latency": (
            summary["safety_stop_latency_ms"] is None
            if latency_limit is None
            else isinstance(summary["safety_stop_latency_ms"], (int, float))
            and not isinstance(summary["safety_stop_latency_ms"], bool)
            and 0.0 <= float(summary["safety_stop_latency_ms"]) <= float(latency_limit)
        ),
        "odometry_displacement": (
            isinstance(summary["displacement_m"], (int, float))
            and not isinstance(summary["displacement_m"], bool)
            and float(summary["displacement_m"]) >= float(profile["min_displacement_m"])
        ),
        "post_stop_stability": (
            isinstance(summary["post_stop_drift_m"], (int, float))
            and not isinstance(summary["post_stop_drift_m"], bool)
            and 0.0
            <= float(summary["post_stop_drift_m"])
            <= float(profile["max_post_stop_drift_m"])
        ),
        "goal_accepted": summary["goal_accepted"] is True,
        "feedback_observed": (
            isinstance(summary["feedback_count"], int)
            and not isinstance(summary["feedback_count"], bool)
            and summary["feedback_count"] >= int(profile["min_feedback_count"])
        ),
        "lifecycle_shutdown": summary["lifecycle_managers_shutdown"]
        == ["lifecycle_manager_navigation", "map_lifecycle_manager"],
        "no_unexpected_process_deaths": summary["unexpected_process_deaths"] == [],
        "no_oom_kill": summary["oom_kill_count"] == 0,
        "zero_scenario_exit": summary["scenario_exit_code"] == 0,
        "receipt_bound": _receipt_matches_summary(summary),
    }


def _build_execution_receipt(
    run_id: str,
    scenario: str,
    evidence: Mapping[str, Any] | None,
    artifacts: Mapping[str, Any],
) -> dict[str, Any] | None:
    """Bind observed execution artifacts without granting mission completion."""

    if evidence is None or any(
        artifacts.get(kind) is None for kind in ("evidence", "log", "pressure")
    ):
        return None
    receipt: dict[str, Any] = {
        "contract_version": "flyto.robotics.ros2-acceptance-receipt.v1",
        "run_id": run_id,
        "scenario": scenario,
        "status": evidence["status"],
        "evidence_snapshot": evidence["snapshot"],
        "artifact_sha256": {
            kind: artifacts[kind]["sha256"] for kind in ("evidence", "log", "pressure")
        },
        "task_completion_eligible": False,
    }
    receipt["receipt_sha256"] = _canonical_digest(receipt)
    return receipt


def _receipt_matches_summary(summary: Mapping[str, Any]) -> bool:
    receipt = summary.get("execution_receipt")
    if not isinstance(receipt, Mapping) or set(receipt) != _RECEIPT_FIELDS:
        return False
    artifacts = summary.get("artifacts")
    if not isinstance(artifacts, Mapping):
        return False
    expected_artifacts = {}
    for kind in ("evidence", "log", "pressure"):
        artifact = artifacts.get(kind)
        if not isinstance(artifact, Mapping):
            return False
        expected_artifacts[kind] = artifact.get("sha256")
    unsigned = {key: item for key, item in receipt.items() if key != "receipt_sha256"}
    return (
        receipt["contract_version"] == "flyto.robotics.ros2-acceptance-receipt.v1"
        and receipt["run_id"] == summary.get("run_id")
        and receipt["scenario"] == summary.get("scenario")
        and receipt["status"] == summary.get("status")
        and isinstance(receipt["evidence_snapshot"], str)
        and _DIGEST.fullmatch(receipt["evidence_snapshot"]) is not None
        and receipt["evidence_snapshot"] == summary.get("evidence_snapshot")
        and receipt["artifact_sha256"] == expected_artifacts
        and receipt["task_completion_eligible"] is False
        and receipt["receipt_sha256"] == _canonical_digest(unsigned)
    )


def _checks_array(checks: Mapping[str, bool]) -> list[dict[str, Any]]:
    return [{"code": code, "passed": passed} for code, passed in checks.items()]


def _build_run_summary(
    *,
    root: Path,
    output_dir: Path,
    profile: Mapping[str, Any],
    index: int,
) -> dict[str, Any]:
    run_id = _run_id(profile, index)
    evidence_path = output_dir / f"{run_id}.json"
    log_path = output_dir / f"{run_id}.log"
    pressure_path = output_dir / f"{run_id}.pressure"
    evidence = _read_evidence(evidence_path)
    log_text = log_path.read_text(encoding="utf-8") if log_path.is_file() else ""
    pressure = _read_pressure(pressure_path)
    deaths = classify_process_deaths(log_text)
    artifacts = {
        "evidence": _artifact_record(root, evidence_path),
        "log": _artifact_record(root, log_path),
        "pressure": _artifact_record(root, pressure_path),
    }
    summary: dict[str, Any] = {
        "index": index,
        "run_id": run_id,
        "scenario": profile["scenario"],
        "artifacts": artifacts,
        "evidence_valid": evidence is not None,
        "evidence_snapshot": evidence.get("snapshot") if evidence else None,
        "status": evidence.get("status") if evidence else None,
        "safety_stop_reason": evidence.get("safety_stop_reason") if evidence else None,
        "fault_injection_observed": (
            evidence.get("fault_injection_observed") if evidence else None
        ),
        "safety_stop_latency_ms": (evidence.get("safety_stop_latency_ms") if evidence else None),
        "displacement_m": evidence.get("displacement_m") if evidence else None,
        "post_stop_drift_m": evidence.get("post_stop_drift_m") if evidence else None,
        "goal_accepted": evidence.get("goal_accepted") if evidence else None,
        "feedback_count": evidence.get("feedback_count") if evidence else None,
        "lifecycle_managers_shutdown": _lifecycle_shutdowns(log_text),
        "expected_process_deaths": deaths["expected"],
        "unexpected_process_deaths": deaths["unexpected"],
        "oom_kill_count": pressure.get("oom_kill") if pressure else None,
        "scenario_exit_code": pressure.get("scenario_exit_code") if pressure else None,
        "execution_receipt": _build_execution_receipt(
            run_id, str(profile["scenario"]), evidence, artifacts
        ),
    }
    checks = _run_checks(summary, profile)
    summary["checks"] = _checks_array(checks)
    summary["passed"] = all(checks.values())
    return summary


def _validate_id(value: Any, label: str) -> str:
    if not isinstance(value, str) or _SAFE_ID.fullmatch(value) is None:
        raise Ros2AcceptanceError(f"{label} is invalid")
    return value


def build_ros2_acceptance_report(
    *,
    root: Path,
    output_dir: Path,
    profile_id: str,
    control_id: str,
    container_image_id: str,
) -> dict[str, Any]:
    """Rebuild one fixed-profile verdict directly from its raw artifacts."""

    root = root.resolve()
    profile = ROS2_ACCEPTANCE_PROFILES.get(profile_id)
    if profile is None:
        raise Ros2AcceptanceError("acceptance profile is unsupported")
    _validate_id(control_id, "control_id")
    if not isinstance(container_image_id, str) or _IMAGE_ID.fullmatch(container_image_id) is None:
        raise Ros2AcceptanceError("container image id must be a sha256 image digest")
    output_dir, artifact_root = _safe_artifact_root(root, output_dir)
    runs = [
        _build_run_summary(root=root, output_dir=output_dir, profile=profile, index=index)
        for index in range(1, int(profile["expected_runs"]) + 1)
    ]
    present_names = (
        {
            path.name
            for path in output_dir.iterdir()
            if path.is_file()
            and re.fullmatch(r"(?:success|lidar-dropout)-\d{3}\.(?:json|log|pressure)", path.name)
        }
        if output_dir.is_dir()
        else set()
    )
    expected_names = _expected_artifact_names(profile)
    aggregate_checks = {
        "fixed_run_volume": len(runs) == int(profile["expected_runs"]),
        "exact_artifact_set": present_names == expected_names,
        "sequential_run_identity": [item["index"] for item in runs]
        == list(range(1, int(profile["expected_runs"]) + 1)),
        "unique_artifact_paths": len(
            {
                artifact["path"]
                for run in runs
                for artifact in run["artifacts"].values()
                if artifact is not None
            }
        )
        == sum(artifact is not None for run in runs for artifact in run["artifacts"].values()),
        "all_runs_passed": all(run["passed"] for run in runs),
    }
    report: dict[str, Any] = {
        "contract_version": ROS2_ACCEPTANCE_REPORT_VERSION,
        "report_id": "",
        "control_id": control_id,
        "profile_id": profile_id,
        "execution_mode": "simulation",
        "source_snapshot": repository_source_snapshot(root),
        "oracle_snapshot": acceptance_oracle_snapshot(root),
        "container_image_id": container_image_id,
        "artifact_root": artifact_root,
        "observed_artifact_names": sorted(present_names),
        "expected_runs": int(profile["expected_runs"]),
        "thresholds": {
            key: profile[key]
            for key in (
                "max_post_stop_drift_m",
                "max_safety_stop_latency_ms",
                "min_displacement_m",
                "min_feedback_count",
            )
        },
        "run_summaries": runs,
        "checks": _checks_array(aggregate_checks),
        "passed": all(aggregate_checks.values()),
    }
    report["report_id"] = (
        "ros2-acceptance-"
        + _canonical_digest(
            {
                "control_id": control_id,
                "profile_id": profile_id,
                "source_snapshot": report["source_snapshot"],
                "oracle_snapshot": report["oracle_snapshot"],
                "container_image_id": container_image_id,
                "run_summaries": runs,
            }
        )[:24]
    )
    report["snapshot"] = _canonical_digest(report)
    return parse_ros2_acceptance_report(report)


def _parse_artifact(value: Any) -> None:
    if value is None:
        return
    if not isinstance(value, Mapping) or set(value) != _ARTIFACT_FIELDS:
        raise Ros2AcceptanceError("artifact record fields do not match")
    if not isinstance(value["path"], str):
        raise Ros2AcceptanceError("artifact path is invalid")
    path = Path(value["path"])
    if path.is_absolute() or ".." in path.parts or "results" not in path.parts:
        raise Ros2AcceptanceError("artifact path is unsafe")
    if not isinstance(value["sha256"], str) or _DIGEST.fullmatch(value["sha256"]) is None:
        raise Ros2AcceptanceError("artifact digest is invalid")
    if (
        isinstance(value["bytes"], bool)
        or not isinstance(value["bytes"], int)
        or value["bytes"] < 0
    ):
        raise Ros2AcceptanceError("artifact size is invalid")


def _parse_checks(value: Any, expected: Mapping[str, bool], label: str) -> None:
    if not isinstance(value, list) or len(value) != len(expected):
        raise Ros2AcceptanceError(f"{label} checks are invalid")
    parsed: dict[str, bool] = {}
    for item in value:
        if not isinstance(item, Mapping) or set(item) != {"code", "passed"}:
            raise Ros2AcceptanceError(f"{label} check fields do not match")
        if not isinstance(item["code"], str) or type(item["passed"]) is not bool:
            raise Ros2AcceptanceError(f"{label} check is invalid")
        if item["code"] in parsed:
            raise Ros2AcceptanceError(f"{label} check codes are duplicated")
        parsed[item["code"]] = item["passed"]
    if parsed != expected:
        raise Ros2AcceptanceError(f"{label} checks are inconsistent")


def parse_ros2_acceptance_report(value: Any) -> dict[str, Any]:
    """Strictly parse and recompute a targeted acceptance verdict."""

    if not isinstance(value, Mapping) or set(value) != _REPORT_FIELDS:
        raise Ros2AcceptanceError("acceptance report fields do not match")
    if value["contract_version"] != ROS2_ACCEPTANCE_REPORT_VERSION:
        raise Ros2AcceptanceError("acceptance report version is unsupported")
    profile = ROS2_ACCEPTANCE_PROFILES.get(value["profile_id"])
    if profile is None:
        raise Ros2AcceptanceError("acceptance profile is unsupported")
    _validate_id(value["control_id"], "control_id")
    if value["execution_mode"] != "simulation":
        raise Ros2AcceptanceError("acceptance execution mode is invalid")
    for field in ("source_snapshot", "oracle_snapshot", "snapshot"):
        if not isinstance(value[field], str) or _DIGEST.fullmatch(value[field]) is None:
            raise Ros2AcceptanceError(f"{field} is invalid")
    if (
        not isinstance(value["container_image_id"], str)
        or _IMAGE_ID.fullmatch(value["container_image_id"]) is None
    ):
        raise Ros2AcceptanceError("container image id is invalid")
    artifact_root = value["artifact_root"]
    if not isinstance(artifact_root, str):
        raise Ros2AcceptanceError("artifact root is invalid")
    artifact_path = Path(artifact_root)
    if (
        artifact_path.is_absolute()
        or ".." in artifact_path.parts
        or artifact_path.parts[:2] != ("results", "ros2-resilience")
        or len(artifact_path.parts) < 3
    ):
        raise Ros2AcceptanceError("artifact root is unsafe")
    if value["expected_runs"] != profile["expected_runs"]:
        raise Ros2AcceptanceError("expected run count is inconsistent")
    expected_thresholds = {
        key: profile[key]
        for key in (
            "max_post_stop_drift_m",
            "max_safety_stop_latency_ms",
            "min_displacement_m",
            "min_feedback_count",
        )
    }
    if value["thresholds"] != expected_thresholds:
        raise Ros2AcceptanceError("acceptance thresholds are inconsistent")
    runs = value["run_summaries"]
    if not isinstance(runs, list) or len(runs) != int(profile["expected_runs"]):
        raise Ros2AcceptanceError("acceptance run volume is inconsistent")
    observed_names = value["observed_artifact_names"]
    if (
        not isinstance(observed_names, list)
        or any(not isinstance(item, str) for item in observed_names)
        or observed_names != sorted(set(observed_names))
    ):
        raise Ros2AcceptanceError("observed artifact names are invalid")
    artifact_paths: list[str] = []
    for index, run in enumerate(runs, start=1):
        if not isinstance(run, Mapping) or set(run) != _RUN_FIELDS:
            raise Ros2AcceptanceError("acceptance run fields do not match")
        if run["index"] != index or run["run_id"] != _run_id(profile, index):
            raise Ros2AcceptanceError("acceptance run identity is inconsistent")
        if run["scenario"] != profile["scenario"]:
            raise Ros2AcceptanceError("acceptance run scenario is inconsistent")
        artifacts = run["artifacts"]
        if not isinstance(artifacts, Mapping) or set(artifacts) != {
            "evidence",
            "log",
            "pressure",
        }:
            raise Ros2AcceptanceError("acceptance run artifacts are invalid")
        for kind, artifact in artifacts.items():
            _parse_artifact(artifact)
            if artifact is not None:
                extension = "json" if kind == "evidence" else kind
                expected_path = f"{artifact_root}/{run['run_id']}.{extension}"
                if artifact["path"] != expected_path:
                    raise Ros2AcceptanceError("artifact path does not match its run")
                artifact_paths.append(artifact["path"])
        if type(run["evidence_valid"]) is not bool:
            raise Ros2AcceptanceError("evidence validity is invalid")
        for field in (
            "status",
            "safety_stop_reason",
            "fault_injection_observed",
            "safety_stop_latency_ms",
            "displacement_m",
            "post_stop_drift_m",
            "goal_accepted",
            "feedback_count",
            "oom_kill_count",
            "scenario_exit_code",
            "evidence_snapshot",
        ):
            if run[field] is not None and isinstance(run[field], (dict, list)):
                raise Ros2AcceptanceError(f"run {field} is invalid")
        if not isinstance(run["lifecycle_managers_shutdown"], list):
            raise Ros2AcceptanceError("lifecycle shutdown record is invalid")
        for field in ("expected_process_deaths", "unexpected_process_deaths"):
            if not isinstance(run[field], list):
                raise Ros2AcceptanceError("process death record is invalid")
            for death in run[field]:
                if not isinstance(death, Mapping) or set(death) != {"process", "exit_code"}:
                    raise Ros2AcceptanceError("process death fields do not match")
                if (
                    not isinstance(death["process"], str)
                    or isinstance(death["exit_code"], bool)
                    or not isinstance(death["exit_code"], int)
                ):
                    raise Ros2AcceptanceError("process death record is invalid")
        expected_run_checks = _run_checks(run, profile)
        _parse_checks(run["checks"], expected_run_checks, "run")
        if type(run["passed"]) is not bool or run["passed"] is not all(
            expected_run_checks.values()
        ):
            raise Ros2AcceptanceError("run verdict is inconsistent")
    expected_aggregate_checks = {
        "fixed_run_volume": len(runs) == int(profile["expected_runs"]),
        "exact_artifact_set": set(observed_names) == _expected_artifact_names(profile),
        "sequential_run_identity": [run["index"] for run in runs]
        == list(range(1, int(profile["expected_runs"]) + 1)),
        "unique_artifact_paths": len(artifact_paths) == len(set(artifact_paths)),
        "all_runs_passed": all(run["passed"] for run in runs),
    }
    _parse_checks(value["checks"], expected_aggregate_checks, "aggregate")
    if type(value["passed"]) is not bool or value["passed"] is not all(
        expected_aggregate_checks.values()
    ):
        raise Ros2AcceptanceError("acceptance verdict is inconsistent")
    expected_report_id = (
        "ros2-acceptance-"
        + _canonical_digest(
            {
                "control_id": value["control_id"],
                "profile_id": value["profile_id"],
                "source_snapshot": value["source_snapshot"],
                "oracle_snapshot": value["oracle_snapshot"],
                "container_image_id": value["container_image_id"],
                "run_summaries": runs,
            }
        )[:24]
    )
    if value["report_id"] != expected_report_id:
        raise Ros2AcceptanceError("acceptance report id is inconsistent")
    unsigned = {key: item for key, item in value.items() if key != "snapshot"}
    if value["snapshot"] != _canonical_digest(unsigned):
        raise Ros2AcceptanceError("acceptance report snapshot does not match")
    return dict(value)


def verify_ros2_acceptance_report(root: Path, report: Any) -> dict[str, Any]:
    """Rebuild a report from disk and require byte-for-byte semantic equality."""

    parsed = parse_ros2_acceptance_report(report)
    rebuilt = build_ros2_acceptance_report(
        root=root,
        output_dir=root / parsed["artifact_root"],
        profile_id=parsed["profile_id"],
        control_id=parsed["control_id"],
        container_image_id=parsed["container_image_id"],
    )
    if rebuilt != parsed:
        raise Ros2AcceptanceError("report does not match current source, oracle, or raw artifacts")
    return parsed


def write_report_atomic(path: Path, report: Mapping[str, Any]) -> None:
    """Write a validated report without exposing a partially written verdict."""

    parsed = parse_ros2_acceptance_report(report)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(parsed, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Verify a ROS 2 acceptance report")
    parser.add_argument("report", type=Path)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args(argv)
    payload = json.loads(args.report.read_text(encoding="utf-8"))
    verified = verify_ros2_acceptance_report(args.root.resolve(), payload)
    print(
        json.dumps(
            {
                "report_id": verified["report_id"],
                "profile_id": verified["profile_id"],
                "passed": verified["passed"],
                "snapshot": verified["snapshot"],
            },
            indent=2,
        )
    )
    return 0 if verified["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
