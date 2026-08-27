#!/usr/bin/env python3
"""Run one fixed LiDAR-dropout proof and seal it with the tracked adjudicator."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from flyto_robotics.ros2_acceptance import (  # noqa: E402
    Ros2AcceptanceError,
    build_ros2_acceptance_report,
    verify_ros2_acceptance_report,
    write_report_atomic,
)

RUNNER_PATH = ROOT / "scripts" / "run_ros2_resilience_series.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("lidar_fault_runner", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("resilience runner cannot be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _output_dir(value: str) -> Path:
    path = (ROOT / value).resolve() if not Path(value).is_absolute() else Path(value).resolve()
    allowed = (ROOT / "results" / "ros2-resilience").resolve()
    if path == allowed or allowed not in path.parents:
        raise argparse.ArgumentTypeError("output must be a child of results/ros2-resilience")
    return path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=_output_dir, required=True)
    parser.add_argument("--control-id", required=True)
    parser.add_argument("--domain-id", type=int, default=103)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if not 0 <= args.domain_id <= 232:
        raise Ros2AcceptanceError("fault proof ROS domain is invalid")
    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        raise Ros2AcceptanceError("output directory must be new or empty")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    runner = _load_runner()
    runner._ensure_image(runner.DEFAULT_IMAGE)
    runner._build_workspace(runner.NETEM_IMAGE)
    image_id = runner._run(
        ["docker", "image", "inspect", "--format", "{{.Id}}", runner.NETEM_IMAGE],
        timeout=20,
    ).stdout.strip()
    runner._nav_run(
        episode_dir=args.output_dir,
        run_id="lidar-dropout-001",
        domain_id=args.domain_id,
        scenario="lidar_dropout",
        image=runner.NETEM_IMAGE,
        cpu_millicores=2000,
        memory_mib=3072,
        sensor_timeout_seconds=0.55,
    )
    report = build_ros2_acceptance_report(
        root=ROOT,
        output_dir=args.output_dir,
        profile_id="lidar-fault-proof",
        control_id=args.control_id,
        container_image_id=image_id,
    )
    report_path = args.output_dir / "fault-proof-report.json"
    write_report_atomic(report_path, report)
    verified = verify_ros2_acceptance_report(
        ROOT,
        json.loads(report_path.read_text(encoding="utf-8")),
    )
    print(json.dumps(verified, ensure_ascii=False, indent=2))
    return 0 if verified["passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
