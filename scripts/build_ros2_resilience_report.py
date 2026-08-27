#!/usr/bin/env python3
"""Seal exact local artifacts into a #008-#012 resilience report."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from flyto_robotics.ros2_stress_evidence import (  # noqa: E402
    build_ros2_resilience_report,
    parse_ros2_resilience_report,
)


def _object(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def _local_file(root: Path, value: Any) -> tuple[Path, str]:
    if not isinstance(value, str) or not value:
        raise ValueError("artifact paths must be non-empty strings")
    relative = Path(value)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("artifact paths must stay below --root")
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError as exc:
        raise ValueError("artifact paths must stay below --root") from exc
    if not resolved.is_file():
        raise ValueError(f"artifact is missing: {relative.as_posix()}")
    return resolved, relative.as_posix()


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def build_from_manifest(root: Path, manifest: dict[str, Any]) -> dict[str, Any]:
    expected = {
        "test_id",
        "source_snapshot",
        "container_image_id",
        "started_at",
        "finished_at",
        "runs",
        "timeline",
        "metrics",
        "artifacts",
    }
    if set(manifest) != expected:
        raise ValueError("resilience manifest fields do not match")
    raw_runs = manifest["runs"]
    if not isinstance(raw_runs, list):
        raise ValueError("resilience manifest runs must be a list")
    runs = []
    for index, item in enumerate(raw_runs, start=1):
        if not isinstance(item, dict) or set(item) != {
            "run_id",
            "passed",
            "evidence_path",
            "duration_seconds",
        }:
            raise ValueError("resilience manifest run fields do not match")
        evidence, _relative = _local_file(root, item["evidence_path"])
        runs.append(
            {
                "run": index,
                "run_id": item["run_id"],
                "passed": item["passed"],
                "snapshot": _digest(evidence),
                "duration_seconds": item["duration_seconds"],
            }
        )
    raw_artifacts = manifest["artifacts"]
    if not isinstance(raw_artifacts, list):
        raise ValueError("resilience manifest artifacts must be a list")
    artifacts = []
    for item in raw_artifacts:
        if not isinstance(item, dict) or set(item) != {"kind", "path"}:
            raise ValueError("resilience manifest artifact fields do not match")
        path, relative = _local_file(root, item["path"])
        artifacts.append(
            {
                "kind": item["kind"],
                "path": relative,
                "sha256": _digest(path),
                "bytes": path.stat().st_size,
            }
        )
    return build_ros2_resilience_report(
        test_id=manifest["test_id"],
        source_snapshot=manifest["source_snapshot"],
        container_image_id=manifest["container_image_id"],
        started_at=manifest["started_at"],
        finished_at=manifest["finished_at"],
        run_summaries=runs,
        timeline=manifest["timeline"],
        metrics=manifest["metrics"],
        artifacts=artifacts,
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root = args.root.resolve()
    report = build_from_manifest(root, _object(args.manifest))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    verified = parse_ros2_resilience_report(_object(args.output))
    print(
        json.dumps(
            {
                "report_id": verified["report_id"],
                "episode": verified["episode"],
                "test_id": verified["test_id"],
                "passed": verified["passed"],
                "snapshot": verified["snapshot"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    if verified["passed"] is not True:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
