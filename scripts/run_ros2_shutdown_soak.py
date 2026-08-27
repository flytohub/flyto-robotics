#!/usr/bin/env python3
"""Run a fixed shutdown profile and seal it with the tracked adjudicator."""

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
    ROS2_ACCEPTANCE_PROFILES,
    Ros2AcceptanceError,
    build_ros2_acceptance_report,
    verify_ros2_acceptance_report,
    write_report_atomic,
)

RUNNER_PATH = ROOT / "scripts" / "run_ros2_resilience_series.py"


def _load_runner():
    spec = importlib.util.spec_from_file_location("shutdown_soak_runner", RUNNER_PATH)
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
        raise argparse.ArgumentTypeError(
            "output must be a child of results/ros2-resilience"
        )
    return path


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--profile",
        choices=("shutdown-smoke", "shutdown-soak"),
        required=True,
    )
    parser.add_argument("--output-dir", type=_output_dir, required=True)
    parser.add_argument("--control-id", required=True)
    parser.add_argument("--domain-start", type=int, default=104)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    profile = ROS2_ACCEPTANCE_PROFILES[args.profile]
    expected_runs = int(profile["expected_runs"])
    if not 0 <= args.domain_start <= 232 - expected_runs + 1:
        raise Ros2AcceptanceError("shutdown profile exceeds the ROS domain range")
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
    report_path = args.output_dir / "shutdown-soak-report.json"

    for index in range(1, expected_runs + 1):
        runner._nav_run(
            episode_dir=args.output_dir,
            run_id=f"success-{index:03d}",
            domain_id=args.domain_start + index - 1,
            scenario="success",
            image=runner.NETEM_IMAGE,
            cpu_millicores=2000,
            memory_mib=3072,
            sensor_timeout_seconds=0.55,
        )
        report = build_ros2_acceptance_report(
            root=ROOT,
            output_dir=args.output_dir,
            profile_id=args.profile,
            control_id=args.control_id,
            container_image_id=image_id,
        )
        write_report_atomic(report_path, report)
        current = report["run_summaries"][index - 1]
        print(
            f"[{args.control_id}] {index}/{expected_runs} "
            f"{'PASS' if current['passed'] else 'FAIL'}",
            flush=True,
        )

    verified = verify_ros2_acceptance_report(
        ROOT,
        json.loads(report_path.read_text(encoding="utf-8")),
    )
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
