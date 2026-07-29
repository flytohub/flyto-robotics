from __future__ import annotations

import json
from pathlib import Path

import pytest

from flyto_robotics.matrix import (
    LAB_MATRIX_CONTRACT_VERSION,
    aggregate_lab_reports,
    render_matrix_junit,
    render_matrix_markdown,
)


def _write_report(path: Path, *, passed: bool, elapsed: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(
            {
                "contract_version": "flyto.robotics.lab-report.v1",
                "scenario_id": "gazebo.test.v1",
                "passed": passed,
                "checks": [
                    {"name": "physics", "passed": passed, "detail": "test"}
                ],
                "metrics": {
                    "elapsed_seconds": elapsed,
                    "safety_stop_count": 1,
                    "event_count": 30,
                    "gazebo_world_displacement": 4.2,
                },
            }
        ),
        encoding="utf-8",
    )


def test_matrix_aggregates_independent_passing_runs(tmp_path: Path) -> None:
    paths = [tmp_path / f"run-{index}" / "report.json" for index in range(3)]
    for index, path in enumerate(paths):
        _write_report(path, passed=True, elapsed=18.8 + index / 10)
    report = aggregate_lab_reports(paths)
    assert report["contract_version"] == LAB_MATRIX_CONTRACT_VERSION
    assert report["passed"] is True
    assert report["pass_rate"] == 1.0
    assert report["metrics"]["elapsed_seconds_mean"] == 18.9
    assert "Verdict: **PASS**" in render_matrix_markdown(report)
    assert 'failures="0"' in render_matrix_junit(report)


def test_matrix_exposes_failed_or_inconsistent_reports(tmp_path: Path) -> None:
    first = tmp_path / "run-a" / "report.json"
    second = tmp_path / "run-b" / "report.json"
    _write_report(first, passed=True, elapsed=18.8)
    _write_report(second, passed=False, elapsed=19.0)
    report = aggregate_lab_reports([first, second])
    assert report["passed"] is False
    assert report["failed_runs"] == 1
    assert report["consistent_assertions"] is False
    assert 'failures="2"' in render_matrix_junit(report)


def test_matrix_rejects_unbounded_report_lists() -> None:
    with pytest.raises(ValueError, match="between 1 and 20"):
        aggregate_lab_reports([])
