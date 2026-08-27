#!/usr/bin/env python3
"""Run and seal the fixed ROS 2 simulation resilience episodes #008-#012."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
SCRIPTS_ROOT = Path(__file__).resolve().parent
if str(SCRIPTS_ROOT) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_ROOT))

from build_ros2_resilience_report import build_from_manifest  # noqa: E402

from flyto_robotics.ros2_execution_evidence import (  # noqa: E402
    parse_ros2_execution_evidence,
)
from flyto_robotics.ros2_stress_evidence import (  # noqa: E402
    ROS2_RESILIENCE_PROFILES,
    build_ros2_resilience_series,
    parse_ros2_pressure_report,
    parse_ros2_stress_campaign,
)

RESULTS_ROOT = ROOT / "results" / "ros2-resilience"
DEFAULT_IMAGE = "flyto-robotics:jazzy-harmonic"
NETEM_IMAGE = "flyto-robotics:jazzy-harmonic-netem"
_SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,79}$")
ROS_DOMAIN_MAX = 232
PROFILE_DOMAIN_SPANS = {
    "runtime-network-r2": 2,
    "resource-cliff-r2": 12,
    "compound-chaos-c1": 2,
    "gazebo-endurance-l4": 115,
    "cold-repro-b3": 3,
}
_SCENARIO_TERMINAL_EXPECTATIONS = {
    "success": ("succeeded", None),
    "cancel": ("canceled", None),
    "emergency_stop": ("safety_stopped", "emergency_stop"),
    "lidar_dropout": ("safety_stopped", "lidar_stale"),
    "odometry_freeze": ("safety_stopped", "odometry_stale"),
    "nav2_lifecycle_failure": ("safety_stopped", "command_stale"),
}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _run(
    args: list[str],
    *,
    check: bool = True,
    timeout: float | None = None,
    input_text: str | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        args,
        cwd=ROOT,
        check=check,
        text=True,
        input=input_text,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout,
    )


def _source_snapshot(root: Path = ROOT) -> str:
    excluded = {
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
    resolved_root = root.resolve()
    digest = hashlib.sha256()
    for path in sorted(item for item in root.rglob("*") if item.is_file()):
        try:
            path.resolve().relative_to(resolved_root)
        except ValueError:
            continue
        relative = path.relative_to(root)
        if relative == Path("core"):
            continue
        if any(part in excluded for part in relative.parts):
            continue
        digest.update(relative.as_posix().encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _ensure_image(image: str) -> str:
    _run(["docker", "image", "inspect", image], timeout=20)
    if image == NETEM_IMAGE:
        return _run(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image],
            timeout=20,
        ).stdout.strip()
    dockerfile = """ARG BASE_IMAGE
FROM ${BASE_IMAGE}
RUN apt-get update && apt-get install -y --no-install-recommends iproute2 \\
    && rm -rf /var/lib/apt/lists/*
"""
    _run(
        [
            "docker",
            "build",
            "--build-arg",
            f"BASE_IMAGE={image}",
            "-t",
            NETEM_IMAGE,
            "-",
        ],
        timeout=900,
        input_text=dockerfile,
    )
    return _run(
        ["docker", "image", "inspect", "--format", "{{.Id}}", NETEM_IMAGE],
        timeout=20,
    ).stdout.strip()


def _build_workspace(image: str) -> None:
    result = _run(
        [
            "docker",
            "run",
            "--rm",
            "-v",
            f"{ROOT}:/workspace",
            "-w",
            "/workspace",
            image,
            "bash",
            "-lc",
            "source /opt/ros/jazzy/setup.bash && colcon build --symlink-install",
        ],
        check=False,
        timeout=1200,
    )
    if result.returncode:
        raise RuntimeError("workspace build failed:\n" + result.stdout[-4000:])


@dataclass
class NavRun:
    run_id: str
    passed: bool
    evidence_path: Path
    log_path: Path
    duration_seconds: float
    container_id: str
    container_name: str
    exit_code: int
    active_at: str | None = None
    motion_at: str | None = None
    injection_at: str | None = None
    fault_at: str | None = None
    stop_at: str | None = None
    pressure_removed_at: str | None = None
    finished_at: str | None = None
    stop_latency_ms: float | None = None
    cgroup: dict[str, int] | None = None


def _container_logs(name: str) -> str:
    return _run(["docker", "logs", name], check=False, timeout=15).stdout


def _wait_log(name: str, marker: str, timeout_seconds: float) -> tuple[bool, str]:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        text = _container_logs(name)
        if marker in text:
            return True, text
        state = _run(
            ["docker", "inspect", "--format", "{{.State.Running}}", name],
            check=False,
            timeout=10,
        ).stdout.strip()
        if state == "false":
            return False, text
        time.sleep(0.2)
    return False, _container_logs(name)


def _last_log_time(logs: str, marker: str) -> tuple[str | None, float | None]:
    matches = re.findall(
        rf"\[([0-9]+\.[0-9]+)\].*{re.escape(marker)}",
        logs,
    )
    if not matches:
        return None, None
    epoch = float(matches[-1])
    observed_at = datetime.fromtimestamp(epoch, timezone.utc).isoformat().replace("+00:00", "Z")
    return observed_at, epoch


def _exec(name: str, command: str, *, timeout: float = 20) -> subprocess.CompletedProcess[str]:
    return _run(
        ["docker", "exec", name, "bash", "-lc", command],
        check=False,
        timeout=timeout,
    )


def _start_execution_observer(
    name: str,
    active_marker: Path,
    motion_marker: Path,
    output_path: Path,
) -> None:
    command = (
        "source /opt/ros/jazzy/setup.bash; "
        "source /workspace/install/setup.bash; "
        "python3 /workspace/scripts/ros2_execution_observer.py "
        f"--active-marker /workspace/{active_marker.relative_to(ROOT).as_posix()} "
        f"--motion-marker /workspace/{motion_marker.relative_to(ROOT).as_posix()} "
        f"--output /workspace/{output_path.relative_to(ROOT).as_posix()}"
    )
    result = _run(
        ["docker", "exec", "-d", name, "bash", "-lc", command],
        check=False,
        timeout=10,
    )
    if result.returncode:
        raise RuntimeError("failed to start execution observer: " + result.stdout)


def _wait_host_file(name: str, path: Path, timeout_seconds: float) -> bool:
    deadline = time.monotonic() + timeout_seconds
    while time.monotonic() < deadline:
        if path.is_file() and path.read_text(encoding="utf-8").strip():
            return True
        state = _run(
            ["docker", "inspect", "--format", "{{.State.Running}}", name],
            check=False,
            timeout=10,
        ).stdout.strip()
        if state == "false":
            return False
        time.sleep(0.05)
    return False


def _cgroup(name: str) -> dict[str, int]:
    script = """
set -e
printf 'memory_peak=%s\n' "$(cat /sys/fs/cgroup/memory.peak)"
while read -r key value; do
  case "$key" in
    usage_usec|throttled_usec) printf '%s=%s\n' "$key" "$value" ;;
  esac
done < /sys/fs/cgroup/cpu.stat
while read -r key value; do
  case "$key" in oom_kill) printf 'oom_kill=%s\n' "$value" ;; esac
done < /sys/fs/cgroup/memory.events
"""
    result = _exec(name, script)
    values: dict[str, int] = {}
    for line in result.stdout.splitlines():
        key, separator, value = line.partition("=")
        if separator and value.isdigit():
            values[key] = int(value)
    return values


def _read_evidence(path: Path) -> dict[str, Any] | None:
    if not path.is_file():
        return None
    try:
        return parse_ros2_execution_evidence(json.loads(path.read_text(encoding="utf-8")))
    except (ValueError, json.JSONDecodeError):
        return None


def _evidence_passes_scenario(
    evidence: dict[str, Any] | None,
    scenario: str,
) -> bool:
    """Classify a native scenario only from its strict terminal evidence."""

    expected = _SCENARIO_TERMINAL_EXPECTATIONS.get(scenario)
    if evidence is None or expected is None:
        return False
    expected_status, expected_safety_reason = expected
    if evidence["status"] != expected_status:
        return False
    if expected_safety_reason is None:
        return True
    return bool(
        evidence["safety_stop_observed"] is True
        and evidence["safety_stop_reason"] == expected_safety_reason
    )


def _nav_run(
    *,
    episode_dir: Path,
    run_id: str,
    domain_id: int,
    scenario: str,
    image: str,
    cpu_millicores: int,
    memory_mib: int,
    network: tuple[int, int, float] | None = None,
    fault_delay_seconds: float = 0.35,
    sensor_timeout_seconds: float = 0.55,
) -> NavRun:
    if not 0.45 <= float(sensor_timeout_seconds) <= 0.60:
        raise ValueError("sensor timeout must remain inside the L4 safety envelope")
    evidence_path = episode_dir / f"{run_id}.json"
    pressure_path = episode_dir / f"{run_id}.pressure"
    observer_path = episode_dir / f"{run_id}.observer.json"
    active_marker = episode_dir / f"{run_id}.active"
    motion_marker = episode_dir / f"{run_id}.motion"
    injection_marker = episode_dir / f"{run_id}.injected"
    removal_marker = episode_dir / f"{run_id}.removed"
    for marker in (active_marker, motion_marker, injection_marker, removal_marker):
        marker.unlink(missing_ok=True)
    log_path = episode_dir / f"{run_id}.log"
    container_name = f"flyto-resilience-{os.getpid()}-{domain_id}"
    command = f"""
set -o pipefail
record_pressure() {{
  status=$?
  trap - EXIT
  {{
    printf 'memory_peak=%s\n' "$(cat /sys/fs/cgroup/memory.peak)"
    while read -r key value; do
      case "$key" in
        usage_usec|throttled_usec) printf '%s=%s\n' "$key" "$value" ;;
      esac
    done < /sys/fs/cgroup/cpu.stat
    while read -r key value; do
      case "$key" in oom_kill) printf 'oom_kill=%s\n' "$value" ;; esac
    done < /sys/fs/cgroup/memory.events
    printf 'scenario_exit_code=%s\n' "$status"
  }} > /workspace/{pressure_path.relative_to(ROOT).as_posix()}
  exit "$status"
}}
trap record_pressure EXIT
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
timeout --signal=TERM --kill-after=15s 210s \
  ros2 launch flyto_robotics nav2_closed_loop.launch.py \
  headless:=true scenario:={scenario} \
  fault_delay_seconds:={fault_delay_seconds} \
  sensor_timeout_seconds:={float(sensor_timeout_seconds):.2f} \
  output_file:=/workspace/{evidence_path.relative_to(ROOT).as_posix()}
"""
    create = [
        "docker",
        "create",
        "--name",
        container_name,
        "--cpus",
        str(cpu_millicores / 1000),
        "--memory",
        f"{memory_mib}m",
        "--memory-swap",
        f"{memory_mib}m",
        "-e",
        f"ROS_DOMAIN_ID={domain_id}",
        "-e",
        "RCUTILS_LOGGING_USE_STDOUT=1",
        "-v",
        f"{ROOT}:/workspace",
        "-w",
        "/workspace",
    ]
    if network is not None:
        create.extend(["--cap-add", "NET_ADMIN"])
    create.extend([image, "bash", "-lc", command])
    started = time.monotonic()
    container_id = _run(create, timeout=30).stdout.strip()
    active_at = motion_at = injection_at = fault_at = stop_at = None
    pressure_removed_at = None
    injection_epoch: float | None = None
    stop_latency_ms = None
    cgroup: dict[str, int] = {}
    try:
        _run(["docker", "start", container_name], timeout=20)
        if network is not None:
            _start_execution_observer(
                container_name,
                active_marker,
                motion_marker,
                observer_path,
            )
            active = _wait_host_file(container_name, active_marker, 100)
            if active:
                active_at = active_marker.read_text(encoding="utf-8").strip()
            if active and _wait_host_file(container_name, motion_marker, 20):
                motion_at = motion_marker.read_text(encoding="utf-8").strip()
                delay, jitter, loss = network
                injection = _exec(
                    container_name,
                    "tc qdisc replace dev lo root netem "
                    f"delay {delay}ms {jitter}ms loss {loss}% && "
                    "tc qdisc show dev lo && "
                    "date -u +'%Y-%m-%dT%H:%M:%S.%6NZ' > "
                    f"/workspace/{injection_marker.relative_to(ROOT).as_posix()}",
                )
                if (
                    injection.returncode == 0
                    and "netem" in injection.stdout
                    and injection_marker.is_file()
                ):
                    injection_at = injection_marker.read_text(encoding="utf-8").strip()
                    injection_epoch = datetime.fromisoformat(
                        injection_at.replace("Z", "+00:00")
                    ).timestamp()
                    marker = (
                        "fault injection active: lidar_dropout"
                        if scenario == "lidar_dropout"
                        else "emergency stop latched:"
                    )
                    if scenario == "lidar_dropout":
                        _wait_log(container_name, marker, 30)
                    _wait_log(container_name, "emergency stop latched:", 30)
                    removed = _exec(
                        container_name,
                        "tc qdisc del dev lo root && "
                        "date -u +'%Y-%m-%dT%H:%M:%S.%6NZ' > "
                        f"/workspace/{removal_marker.relative_to(ROOT).as_posix()}",
                        timeout=10,
                    )
                    if removed.returncode == 0 and removal_marker.is_file():
                        pressure_removed_at = removal_marker.read_text(encoding="utf-8").strip()
        waited = _run(["docker", "wait", container_name], check=False, timeout=260).stdout.strip()
        exit_code = int(waited) if waited.lstrip("-").isdigit() else 255
        logs = _container_logs(container_name)
    except subprocess.TimeoutExpired:
        exit_code = 124
        logs = _container_logs(container_name)
    finally:
        _run(["docker", "rm", "-f", container_name], check=False, timeout=30)
    log_path.write_text(logs, encoding="utf-8")
    if pressure_path.is_file():
        for line in pressure_path.read_text(encoding="utf-8").splitlines():
            key, separator, value = line.partition("=")
            if separator and value.isdigit():
                cgroup[key] = int(value)
    if injection_epoch is not None:
        if scenario == "lidar_dropout":
            fault_at, _fault_epoch = _last_log_time(
                logs,
                "fault injection active: lidar_dropout",
            )
        stop_at, stop_epoch = _last_log_time(logs, "emergency stop latched:")
        if stop_epoch is not None:
            stop_latency_ms = round(max(0.0, (stop_epoch - injection_epoch) * 1000), 3)
    evidence = _read_evidence(evidence_path)
    if scenario == "success" and network is not None:
        passed = bool(
            evidence
            and evidence["status"] == "safety_stopped"
            and evidence["safety_stop_observed"] is True
            and evidence["safety_stop_reason"] is not None
            and active_at is not None
            and motion_at is not None
            and injection_at is not None
            and stop_at is not None
            and pressure_removed_at is not None
        )
    else:
        passed = _evidence_passes_scenario(evidence, scenario)
    evidence_for_digest = evidence_path if evidence_path.is_file() else log_path
    return NavRun(
        run_id=run_id,
        passed=passed,
        evidence_path=evidence_for_digest,
        log_path=log_path,
        duration_seconds=round(max(0.001, time.monotonic() - started), 3),
        container_id=container_id,
        container_name=container_name,
        exit_code=exit_code,
        active_at=active_at,
        motion_at=motion_at,
        injection_at=injection_at,
        fault_at=fault_at,
        stop_at=stop_at,
        pressure_removed_at=pressure_removed_at,
        finished_at=_utc_now(),
        stop_latency_ms=stop_latency_ms,
        cgroup=cgroup,
    )


def _artifact_record(path: Path) -> dict[str, Any]:
    return {
        "path": path.relative_to(ROOT).as_posix(),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
    }


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def _seal(
    *,
    episode_dir: Path,
    test_id: str,
    source_snapshot: str,
    image_id: str,
    started_at: str,
    finished_at: str,
    runs: list[NavRun],
    timeline: list[dict[str, Any]],
    metrics: dict[str, float],
    artifacts: list[tuple[str, Path]],
) -> dict[str, Any]:
    manifest = {
        "test_id": test_id,
        "source_snapshot": source_snapshot,
        "container_image_id": image_id,
        "started_at": started_at,
        "finished_at": finished_at,
        "runs": [
            {
                "run_id": run.run_id,
                "passed": run.passed,
                "evidence_path": run.evidence_path.relative_to(ROOT).as_posix(),
                "duration_seconds": run.duration_seconds,
            }
            for run in runs
        ],
        "timeline": timeline,
        "metrics": metrics,
        "artifacts": [
            {"kind": kind, "path": path.relative_to(ROOT).as_posix()} for kind, path in artifacts
        ],
    }
    manifest_path = episode_dir / "resilience-manifest.json"
    _write_json(manifest_path, manifest)
    report = build_from_manifest(ROOT, manifest)
    _write_json(episode_dir / "resilience-report.json", report)
    return report


def _timeline(events: list[tuple[str, str | None]], started_at: str) -> list[dict[str, Any]]:
    observed = [
        ("test_started", started_at),
        *((event, at) for event, at in events if at is not None),
        ("test_recorded", _utc_now()),
    ]
    return [
        {"sequence": index, "event": event, "at": at}
        for index, (event, at) in enumerate(observed, start=1)
    ]


def _raw_log(episode_dir: Path, runs: list[NavRun]) -> Path:
    path = episode_dir / "raw.log"
    with path.open("w", encoding="utf-8") as handle:
        for run in runs:
            handle.write(f"===== {run.run_id} =====\n")
            handle.write(run.log_path.read_text(encoding="utf-8", errors="replace"))
            handle.write("\n")
    return path


def _profile_stop_latency_ms(
    *,
    compound: bool,
    network_relative_ms: float | None,
    fault_evidence: dict[str, Any] | None,
) -> float:
    """Select the latency clock owned by the fault that must trigger the stop."""

    value: Any
    if compound:
        value = (
            fault_evidence.get("safety_stop_latency_ms")
            if fault_evidence is not None
            else None
        )
    else:
        value = network_relative_ms
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return 1e9
    latency = float(value)
    if not math.isfinite(latency) or latency < 0.0:
        return 1e9
    return latency


def _runtime_or_compound(
    *,
    test_id: str,
    run_id: str,
    source_snapshot: str,
    image_id: str,
    domain_base: int,
) -> dict[str, Any]:
    profile = ROS2_RESILIENCE_PROFILES[test_id]
    conditions = profile["conditions"]
    episode_dir = RESULTS_ROOT / run_id / test_id
    episode_dir.mkdir(parents=True, exist_ok=True)
    started_at = _utc_now()
    compound = test_id == "compound-chaos-c1"
    pressure = _nav_run(
        episode_dir=episode_dir,
        run_id="pressure",
        domain_id=domain_base,
        scenario="lidar_dropout" if compound else "success",
        image=NETEM_IMAGE,
        cpu_millicores=int(conditions.get("cpu_millicores", 2000)),
        memory_mib=int(conditions.get("memory_mib", 3072)),
        network=(
            int(conditions["network_delay_ms"]),
            int(conditions["network_jitter_ms"]),
            float(conditions["network_loss_percent"]),
        ),
        fault_delay_seconds=float(conditions.get("lidar_dropout_delay_seconds", 0.35)),
        sensor_timeout_seconds=float(conditions["sensor_timeout_seconds"]),
    )
    recovery = _nav_run(
        episode_dir=episode_dir,
        run_id="recovery",
        domain_id=domain_base + 1,
        scenario="success",
        image=NETEM_IMAGE,
        cpu_millicores=int(conditions.get("cpu_millicores", 2000)),
        memory_mib=int(conditions.get("memory_mib", 3072)),
        sensor_timeout_seconds=float(conditions["sensor_timeout_seconds"]),
    )
    runs = [pressure, recovery]
    event_values: list[tuple[str, str | None]] = [
        ("lifecycle_active", pressure.active_at),
        ("mission_motion_observed", pressure.motion_at),
        ("network_injected", pressure.injection_at),
    ]
    if compound:
        event_values.append(("lidar_dropout_injected", pressure.fault_at))
    event_values.extend(
        [
            ("safety_stop_observed", pressure.stop_at),
            (
                "pressure_removed" if compound else "network_removed",
                pressure.pressure_removed_at,
            ),
            ("recovery_mission_succeeded", recovery.finished_at if recovery.passed else None),
        ]
    )
    event_log = episode_dir / "event-log.json"
    timeline = _timeline(event_values, started_at)
    _write_json(event_log, timeline)
    raw_log = _raw_log(episode_dir, runs)
    recovery_evidence = episode_dir / "recovery-evidence.json"
    if recovery.evidence_path.suffix == ".json":
        shutil.copyfile(recovery.evidence_path, recovery_evidence)
    else:
        _write_json(
            recovery_evidence, {"passed": False, "log": _artifact_record(recovery.log_path)}
        )
    pressure_evidence = _read_evidence(pressure.evidence_path)
    stop_latency = _profile_stop_latency_ms(
        compound=compound,
        network_relative_ms=pressure.stop_latency_ms,
        fault_evidence=pressure_evidence,
    )
    drift = float(pressure_evidence["post_stop_drift_m"]) if pressure_evidence is not None else 1e9
    metrics = {
        "safety_stop_latency_ms": stop_latency,
        "post_stop_drift_m": drift,
        "oom_kill_count": float(sum((run.cgroup or {}).get("oom_kill", 0) for run in runs)),
        "unexpected_process_deaths": float(sum(run.exit_code != 0 for run in runs)),
    }
    artifacts = [
        ("event_log", event_log),
        ("raw_log", raw_log),
        ("recovery_evidence", recovery_evidence),
    ]
    observer_path = episode_dir / "pressure.observer.json"
    if observer_path.is_file():
        artifacts.append(("execution_observer", observer_path))
    return _seal(
        episode_dir=episode_dir,
        test_id=test_id,
        source_snapshot=source_snapshot,
        image_id=image_id,
        started_at=started_at,
        finished_at=_utc_now(),
        runs=runs,
        timeline=timeline,
        metrics=metrics,
        artifacts=artifacts,
    )


def _resource_cliff(
    *, run_id: str, source_snapshot: str, image_id: str, domain_base: int
) -> dict[str, Any]:
    test_id = "resource-cliff-r2"
    episode_dir = RESULTS_ROOT / run_id / test_id
    episode_dir.mkdir(parents=True, exist_ok=True)
    started_at = _utc_now()
    conditions = ROS2_RESILIENCE_PROFILES[test_id]["conditions"]
    runs: list[NavRun] = []
    cells = []
    domain = domain_base
    for cell_index, cell in enumerate(conditions["cells"], start=1):
        cell_runs = []
        for repeat in range(1, int(conditions["repetitions_per_cell"]) + 1):
            run = _nav_run(
                episode_dir=episode_dir,
                run_id=f"cell-{cell_index:02d}-run-{repeat:02d}",
                domain_id=domain,
                scenario="success",
                image=NETEM_IMAGE,
                cpu_millicores=int(cell["cpu_millicores"]),
                memory_mib=int(cell["memory_mib"]),
            )
            domain += 1
            runs.append(run)
            cell_runs.append(run)
        cells.append(
            {
                **cell,
                "runs": len(cell_runs),
                "passed_runs": sum(run.passed for run in cell_runs),
                "completed_at": _utc_now(),
                "peak_memory_bytes": max(
                    (run.cgroup or {}).get("memory_peak", 0) for run in cell_runs
                ),
                "oom_kills": sum((run.cgroup or {}).get("oom_kill", 0) for run in cell_runs),
            }
        )
    matrix_path = episode_dir / "matrix.json"
    _write_json(matrix_path, {"conditions": conditions, "cells": cells})
    raw_log = _raw_log(episode_dir, runs)
    safe_cells = sum(cell["passed_runs"] == cell["runs"] for cell in cells)
    timeline = _timeline(
        [
            ("matrix_started", started_at),
            *[("cell_completed", cell["completed_at"]) for cell in cells],
            ("matrix_completed", _utc_now()),
        ],
        started_at,
    )
    metrics = {
        "matrix_cells": float(len(cells)),
        "repetitions_per_cell": float(conditions["repetitions_per_cell"]),
        "safe_cells": float(safe_cells),
        "oom_kill_count": float(sum(cell["oom_kills"] for cell in cells)),
        "unexpected_process_deaths": float(sum(run.exit_code != 0 for run in runs)),
    }
    return _seal(
        episode_dir=episode_dir,
        test_id=test_id,
        source_snapshot=source_snapshot,
        image_id=image_id,
        started_at=started_at,
        finished_at=_utc_now(),
        runs=runs,
        timeline=timeline,
        metrics=metrics,
        artifacts=[("matrix", matrix_path), ("raw_log", raw_log)],
    )


def _slope_per_hour(samples: list[tuple[float, float]]) -> float:
    if len(samples) < 2:
        return 0.0
    x_mean = sum(item[0] for item in samples) / len(samples)
    y_mean = sum(item[1] for item in samples) / len(samples)
    denominator = sum((item[0] - x_mean) ** 2 for item in samples)
    if math.isclose(denominator, 0.0):
        return 0.0
    return round(
        sum((x - x_mean) * (y - y_mean) for x, y in samples) / denominator,
        6,
    )


def _inner_endurance_status(
    *,
    campaign: dict[str, Any] | None,
    pressure_report: dict[str, Any] | None,
    driver_exit_code: int,
) -> tuple[bool, int]:
    """Fail closed when either strict inner endurance verdict is absent or failed."""

    if campaign is None:
        return False, 1
    deaths = int(campaign["runtime_hygiene"]["unexpected_process_deaths"])
    passed = (
        driver_exit_code == 0
        and campaign["passed"] is True
        and pressure_report is not None
        and pressure_report["passed"] is True
        and deaths == 0
    )
    return passed, deaths


def _endurance(
    *, run_id: str, source_snapshot: str, image_id: str, domain_base: int
) -> dict[str, Any]:
    test_id = "gazebo-endurance-l4"
    episode_dir = RESULTS_ROOT / run_id / test_id
    episode_dir.mkdir(parents=True, exist_ok=True)
    campaign_run_id = f"{run_id}-endurance-e1"
    started_at = _utc_now()
    env = dict(os.environ)
    env.update(
        {
            "FLYTO_ROBOTICS_PRESSURE_PROFILE": "endurance-e1",
            "FLYTO_ROBOTICS_STRESS_RUN_ID": campaign_run_id,
            "FLYTO_ROBOTICS_ROS_DOMAIN_ID": str(domain_base),
            "FLYTO_ROBOTICS_IMAGE": NETEM_IMAGE,
        }
    )
    result = subprocess.run(
        [str(ROOT / "scripts" / "run_nav2_stress.sh")],
        cwd=ROOT,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=8 * 60 * 60,
    )
    driver_log = episode_dir / "raw.log"
    driver_log.write_text(result.stdout, encoding="utf-8")
    campaign_dir = ROOT / "results" / "nav2-stress" / campaign_run_id
    campaign_path = campaign_dir / "campaign.json"
    pressure_report = campaign_dir / "pressure-report.json"
    campaign: dict[str, Any] | None = None
    verified_pressure: dict[str, Any] | None = None
    try:
        campaign = parse_ros2_stress_campaign(
            json.loads(campaign_path.read_text(encoding="utf-8"))
        )
    except (OSError, ValueError, json.JSONDecodeError):
        campaign = None
    try:
        verified_pressure = parse_ros2_pressure_report(
            json.loads(pressure_report.read_text(encoding="utf-8"))
        )
    except (OSError, ValueError, json.JSONDecodeError):
        verified_pressure = None
    inner_passed, unexpected_process_deaths = _inner_endurance_status(
        campaign=campaign,
        pressure_report=verified_pressure,
        driver_exit_code=result.returncode,
    )
    evidence_paths = sorted(campaign_dir.glob("round-*/*.json"))
    evidence_paths = [
        path for path in evidence_paths if path.name not in {"grant-expiry.json", "report.json"}
    ]
    runs: list[NavRun] = []
    memory_samples: list[tuple[float, float]] = []
    latency_samples: list[tuple[float, float]] = []
    fault_finished_at: list[str] = []
    elapsed_hours = 0.0
    for index, path in enumerate(evidence_paths, start=1):
        evidence = _read_evidence(path)
        duration = float(evidence["duration_seconds"]) if evidence else 0.001
        elapsed_hours += duration / 3600.0
        pressure_path = path.with_suffix(".pressure")
        pressure: dict[str, int] = {}
        if pressure_path.is_file():
            for line in pressure_path.read_text().splitlines():
                key, separator, value = line.partition("=")
                if separator and value.isdigit():
                    pressure[key] = int(value)
        memory_samples.append((elapsed_hours, pressure.get("memory_peak", 0) / 1024 / 1024))
        if evidence and evidence["safety_stop_latency_ms"] is not None:
            latency_samples.append((elapsed_hours, float(evidence["safety_stop_latency_ms"])))
        if evidence and evidence["scenario"] != "success":
            fault_finished_at.append(str(evidence["finished_at"]))
        passed = bool(
            evidence
            and _evidence_passes_scenario(evidence, str(evidence["scenario"]))
        )
        runs.append(
            NavRun(
                run_id=f"endurance-{index:03d}",
                passed=passed,
                evidence_path=path,
                log_path=path.with_suffix(".log"),
                duration_seconds=max(0.001, duration),
                container_id="not-retained",
                container_name="bounded-scenario-container",
                exit_code=0 if passed else 1,
                finished_at=str(evidence["finished_at"]) if evidence else None,
                cgroup=pressure,
            )
        )
    if not pressure_report.is_file():
        pressure_report = episode_dir / "pressure-report-missing.json"
        _write_json(
            pressure_report,
            {
                "present": False,
                "driver_exit": result.returncode,
                "campaign_present": campaign is not None,
                "campaign_passed": campaign["passed"] if campaign is not None else False,
            },
        )
    trend_path = episode_dir / "trend.json"
    trend = {
        "basis": "sequential per-run container peaks; not one persistent process",
        "memory_samples": memory_samples,
        "stop_latency_samples": latency_samples,
        "memory_slope_mib_per_hour": _slope_per_hour(memory_samples),
        "stop_latency_slope_ms_per_hour": _slope_per_hour(latency_samples),
    }
    _write_json(trend_path, trend)
    finished_at = _utc_now()
    timeline = _timeline(
        [
            ("endurance_started", started_at),
            ("first_fault_completed", fault_finished_at[0] if fault_finished_at else None),
            ("last_fault_completed", fault_finished_at[-1] if fault_finished_at else None),
            ("endurance_completed", finished_at),
        ],
        started_at,
    )
    metrics = {
        "execution_runs": float(len(runs)),
        "pass_rate": (
            sum(run.passed for run in runs) / max(1, len(runs))
            if inner_passed
            else 0.0
        ),
        "memory_slope_mib_per_hour": float(trend["memory_slope_mib_per_hour"]),
        "stop_latency_slope_ms_per_hour": float(trend["stop_latency_slope_ms_per_hour"]),
        "oom_kill_count": float(sum((run.cgroup or {}).get("oom_kill", 0) for run in runs)),
        "unexpected_process_deaths": float(unexpected_process_deaths),
    }
    artifacts = [
        ("pressure_report", pressure_report),
        ("raw_log", driver_log),
        ("trend", trend_path),
    ]
    if campaign_path.is_file():
        artifacts.append(("campaign_report", campaign_path))
    return _seal(
        episode_dir=episode_dir,
        test_id=test_id,
        source_snapshot=source_snapshot,
        image_id=image_id,
        started_at=started_at,
        finished_at=_utc_now(),
        runs=runs
        or [
            NavRun(
                run_id="missing-endurance-evidence",
                passed=False,
                evidence_path=driver_log,
                log_path=driver_log,
                duration_seconds=0.001,
                container_id="none",
                container_name="none",
                exit_code=result.returncode,
            )
        ],
        timeline=timeline,
        metrics=metrics,
        artifacts=artifacts,
    )


def _cold_repro(
    *, run_id: str, source_snapshot: str, image_id: str, domain_base: int
) -> dict[str, Any]:
    test_id = "cold-repro-b3"
    episode_dir = RESULTS_ROOT / run_id / test_id
    episode_dir.mkdir(parents=True, exist_ok=True)
    started_at = _utc_now()
    runs = [
        _nav_run(
            episode_dir=episode_dir,
            run_id=f"cold-{index:02d}",
            domain_id=domain_base + index - 1,
            scenario="success",
            image=NETEM_IMAGE,
            cpu_millicores=2000,
            memory_mib=3072,
        )
        for index in range(1, 4)
    ]
    receipts = episode_dir / "build-receipts.json"
    _write_json(
        receipts,
        {
            "source_snapshot": source_snapshot,
            "image_id": image_id,
            "containers": [run.container_id for run in runs],
        },
    )
    cold_report = episode_dir / "cold-run-report.json"
    _write_json(
        cold_report,
        {
            "runs": [
                {
                    "run_id": run.run_id,
                    "passed": run.passed,
                    "duration_seconds": run.duration_seconds,
                    "container_id": run.container_id,
                }
                for run in runs
            ]
        },
    )
    raw_log = _raw_log(episode_dir, runs)
    finished_at = _utc_now()
    timeline = _timeline(
        [
            ("cold_run_started", started_at),
            ("cold_run_completed", finished_at),
        ],
        started_at,
    )
    metrics = {
        "cold_runs": 3.0,
        "passing_runs": float(sum(run.passed for run in runs)),
        "unique_container_ids": float(len({run.container_id for run in runs})),
        "source_snapshot_count": 1.0,
        "image_id_count": 1.0,
        "unexpected_process_deaths": float(sum(run.exit_code != 0 for run in runs)),
    }
    return _seal(
        episode_dir=episode_dir,
        test_id=test_id,
        source_snapshot=source_snapshot,
        image_id=image_id,
        started_at=started_at,
        finished_at=_utc_now(),
        runs=runs,
        timeline=timeline,
        metrics=metrics,
        artifacts=[
            ("build_receipts", receipts),
            ("cold_run_report", cold_report),
            ("raw_log", raw_log),
        ],
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", choices=[*ROS2_RESILIENCE_PROFILES, "all"], default="all")
    parser.add_argument("--run-id", default=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ"))
    parser.add_argument("--image", default=DEFAULT_IMAGE)
    parser.add_argument("--domain-base", type=int, default=1)
    parser.add_argument("--skip-build", action="store_true")
    args = parser.parse_args()
    if not _SAFE_ID.fullmatch(args.run_id):
        raise SystemExit("--run-id contains unsafe characters")
    if not 0 <= args.domain_base <= ROS_DOMAIN_MAX:
        raise SystemExit("--domain-base must be between 0 and ROS domain 232")
    RESULTS_ROOT.mkdir(parents=True, exist_ok=True)
    image_id = _ensure_image(args.image)
    if not args.skip_build:
        _build_workspace(NETEM_IMAGE)
    source_snapshot = _source_snapshot()
    profile_ids = list(ROS2_RESILIENCE_PROFILES) if args.profile == "all" else [args.profile]
    required_domains = sum(PROFILE_DOMAIN_SPANS[profile_id] for profile_id in profile_ids)
    if args.domain_base + required_domains - 1 > ROS_DOMAIN_MAX:
        raise SystemExit(
            f"--domain-base {args.domain_base} leaves too few ROS domains for "
            f"{required_domains} sequential scenarios"
        )
    reports: list[dict[str, Any]] = []
    domain = args.domain_base
    for test_id in profile_ids:
        print(f"[resilience] running {test_id}", flush=True)
        if test_id == "runtime-network-r2":
            report = _runtime_or_compound(
                test_id=test_id,
                run_id=args.run_id,
                source_snapshot=source_snapshot,
                image_id=image_id,
                domain_base=domain,
            )
            domain += PROFILE_DOMAIN_SPANS[test_id]
        elif test_id == "resource-cliff-r2":
            report = _resource_cliff(
                run_id=args.run_id,
                source_snapshot=source_snapshot,
                image_id=image_id,
                domain_base=domain,
            )
            domain += PROFILE_DOMAIN_SPANS[test_id]
        elif test_id == "compound-chaos-c1":
            report = _runtime_or_compound(
                test_id=test_id,
                run_id=args.run_id,
                source_snapshot=source_snapshot,
                image_id=image_id,
                domain_base=domain,
            )
            domain += PROFILE_DOMAIN_SPANS[test_id]
        elif test_id == "gazebo-endurance-l4":
            report = _endurance(
                run_id=args.run_id,
                source_snapshot=source_snapshot,
                image_id=image_id,
                domain_base=domain,
            )
            domain += PROFILE_DOMAIN_SPANS[test_id]
        else:
            report = _cold_repro(
                run_id=args.run_id,
                source_snapshot=source_snapshot,
                image_id=image_id,
                domain_base=domain,
            )
            domain += PROFILE_DOMAIN_SPANS[test_id]
        reports.append(report)
        print(
            json.dumps(
                {
                    "test_id": test_id,
                    "passed": report["passed"],
                    "snapshot": report["snapshot"],
                },
                indent=2,
            ),
            flush=True,
        )
    if args.profile == "all":
        series = build_ros2_resilience_series(reports)
        series_path = RESULTS_ROOT / args.run_id / "series.json"
        _write_json(series_path, series)
        print(f"[resilience] series: {series_path}", flush=True)
        if series["all_passed"] is not True:
            raise SystemExit(1)
    elif reports[0]["passed"] is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
