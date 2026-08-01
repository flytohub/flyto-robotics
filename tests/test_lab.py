from __future__ import annotations

import json
import struct
import zlib
from pathlib import Path

import pytest

from flyto_robotics.evidence_image import (
    VideoFrameSequence,
    encode_rgb_png,
    write_rgb_png_atomic,
)
from flyto_robotics.lab import (
    LAB_REPORT_CONTRACT_VERSION,
    LabValidationError,
    evaluate_lab_result,
    load_lab_scenario,
    parse_lab_scenario,
    render_lab_junit,
    render_lab_markdown,
)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
SCENARIO = PROJECT_ROOT / "scenarios/gazebo/careflow-adversarial.json"


def _scenario_dict() -> dict[str, object]:
    return json.loads(SCENARIO.read_text(encoding="utf-8"))


def _passing_result() -> dict[str, object]:
    kinds = [
        ("mission_accepted", None, None),
        ("primitive_started", "navigate", None),
        ("obstacle_stop", "navigate", None),
        ("path_clear", "navigate", None),
        ("primitive_started", "wait_until_clear", None),
        ("clearance_window_started", "wait_until_clear", None),
        ("primitive_started", "ask_human", None),
        ("human_approval_requested", "ask_human", None),
        ("human_approved", "ask_human", "qr.ward-b.receiver"),
        ("human_decision_rejected", "ask_human", None),
        ("primitive_started", "resume", None),
        ("resume_authorized", "resume", "qr.ward-b.receiver"),
        ("primitive_started", "safe_stop", None),
        ("mission_completed", "safe_stop", None),
    ]
    events = [
        {
            "sequence": index,
            "at_seconds": float(index),
            "kind": kind,
            "state": "completed" if kind == "mission_completed" else "running",
            "detail": kind,
            "step_id": f"step.{index}",
            "capability": capability,
            "actor_id": actor,
        }
        for index, (kind, capability, actor) in enumerate(kinds, start=1)
    ]
    return {
        "contract_version": "flyto.robotics.result.v1",
        "job_id": "demo-pharmacy-to-ward-001",
        "robot_id": "flyto-rover-sim-001",
        "status": "succeeded",
        "reason": None,
        "generated_at": "2026-07-29T00:00:00Z",
        "elapsed_seconds": 40.0,
        "final_state": "completed",
        "final_pose": {"x": 4.3, "y": 0.0, "yaw": 0.0},
        "safety_stop_count": 1,
        "events": events,
        "simulation": {
            "mode": "gazebo_ros2",
            "gazebo_physics": True,
            "obstacle_injected": True,
            "human_approval_injected": True,
        },
    }


def test_lab_scenario_loads_and_validates_all_assets() -> None:
    scenario = load_lab_scenario(SCENARIO, project_root=PROJECT_ROOT)
    assert scenario.scenario_id == "gazebo.careflow.adversarial.v1"
    assert scenario.soak_runs == 50
    assert scenario.expectations.min_safety_stop_count == 1


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.update({"unexpected": True}), "unsupported fields"),
        (
            lambda value: value["assets"].update({"world": "../secret.sdf"}),
            "stay within",
        ),
        (
            lambda value: value["expectations"].update(
                {"required_event_kinds": ["safe_stop", "safe_stop"]}
            ),
            "duplicates",
        ),
        (
            lambda value: value.update({"scenario_id": "unsafe account"}),
            "safe identifier",
        ),
        (
            lambda value: value["expectations"].update(
                {"min_world_displacement": -1}
            ),
            "outside the supported range",
        ),
    ],
)
def test_lab_scenario_fails_closed(
    mutation: object,
    message: str,
) -> None:
    value = _scenario_dict()
    mutation(value)
    with pytest.raises(LabValidationError, match=message):
        parse_lab_scenario(value)


@pytest.mark.parametrize(
    "driver_contract",
    [
        "flyto.robotics.lab-driver-evidence.v1",
        "flyto.robotics.lab-driver-evidence.v2",
    ],
)
def test_lab_report_requires_every_safety_and_image_assertion(
    tmp_path: Path,
    driver_contract: str,
) -> None:
    scenario = load_lab_scenario(SCENARIO, project_root=PROJECT_ROOT)
    (tmp_path / "driver-manifest.json").write_text(
        json.dumps(
            {
                "contract_version": driver_contract,
                "world_displacement": 4.2,
            }
        ),
        encoding="utf-8",
    )
    for label in ("startup", "obstacle", "approval", "completed"):
        write_rgb_png_atomic(
            tmp_path / f"gazebo-{label}-001.png",
            width=1,
            height=1,
            encoding="rgb8",
            step=3,
            data=b"\x10\x20\x30",
        )
    report = evaluate_lab_result(
        scenario,
        _passing_result(),
        project_root=PROJECT_ROOT,
        evidence_dir=tmp_path,
    )
    assert report["contract_version"] == LAB_REPORT_CONTRACT_VERSION
    assert report["passed"] is True
    assert report["metrics"]["capture_count"] == 4
    assert report["metrics"]["gazebo_world_displacement"] == 4.2
    assert len(report["provenance"]) == 6
    markdown = render_lab_markdown(report)
    assert "Verdict: **PASS**" in markdown
    assert "gazebo-obstacle-001.png" in markdown
    junit = render_lab_junit(report)
    assert 'failures="0"' in junit


def test_lab_report_exposes_missing_stop_replay_and_capture(tmp_path: Path) -> None:
    scenario = load_lab_scenario(SCENARIO, project_root=PROJECT_ROOT)
    result = _passing_result()
    result["safety_stop_count"] = 0
    result["events"] = [
        event
        for event in result["events"]
        if event["kind"] not in {"obstacle_stop", "human_decision_rejected"}
    ]
    report = evaluate_lab_result(
        scenario,
        result,
        project_root=PROJECT_ROOT,
        evidence_dir=tmp_path,
    )
    failed_names = {
        item["name"] for item in report["checks"] if item["passed"] is False
    }
    assert report["passed"] is False
    assert "safety_stop_count" in failed_names
    assert "event:obstacle_stop" in failed_names
    assert "event:human_decision_rejected" in failed_names
    assert "capture:startup" in failed_names
    assert "driver_evidence_contract" in failed_names
    assert "gazebo_world_displacement" in failed_names
    assert 'failures="0"' not in render_lab_junit(report)


def test_png_encoder_preserves_dimensions_and_rgb_payload(tmp_path: Path) -> None:
    encoded = encode_rgb_png(
        width=2,
        height=1,
        encoding="bgr8",
        step=8,
        data=b"\x30\x20\x10\x60\x50\x40\xff\xff",
    )
    assert encoded.startswith(b"\x89PNG\r\n\x1a\n")
    assert struct.unpack(">II", encoded[16:24]) == (2, 1)
    idat_length = struct.unpack(">I", encoded[33:37])[0]
    assert encoded[37:41] == b"IDAT"
    raw = zlib.decompress(encoded[41 : 41 + idat_length])
    assert raw == b"\x00\x10\x20\x30\x40\x50\x60"
    destination = tmp_path / "frame.png"
    write_rgb_png_atomic(
        destination,
        width=1,
        height=1,
        encoding="rgb8",
        step=3,
        data=b"\x01\x02\x03",
    )
    assert destination.read_bytes().startswith(b"\x89PNG")


def test_png_encoder_rejects_unbounded_or_short_frames() -> None:
    with pytest.raises(ValueError, match="dimensions"):
        encode_rgb_png(width=0, height=1, encoding="rgb8", step=3, data=b"")
    with pytest.raises(ValueError, match="unsupported"):
        encode_rgb_png(width=1, height=1, encoding="mono8", step=1, data=b"\0")
    with pytest.raises(ValueError, match="shorter"):
        encode_rgb_png(width=1, height=1, encoding="rgb8", step=3, data=b"\0")
    with pytest.raises(ValueError, match="compression"):
        encode_rgb_png(
            width=1,
            height=1,
            encoding="rgb8",
            step=3,
            data=b"\0\0\0",
            compression_level=10,
        )


def test_video_frame_sequence_is_contiguous_and_bounded(tmp_path: Path) -> None:
    sequence = VideoFrameSequence(tmp_path / "frames", max_frames=2)
    arguments = {
        "width": 1,
        "height": 1,
        "encoding": "rgb8",
        "step": 3,
        "data": b"\x10\x20\x30",
    }
    first = sequence.write(**arguments)
    second = sequence.write(**arguments)
    dropped = sequence.write(**arguments)

    assert first is not None and first.name == "frame-000001.png"
    assert second is not None and second.name == "frame-000002.png"
    assert first.read_bytes().startswith(b"\x89PNG")
    assert second.read_bytes().startswith(b"\x89PNG")
    assert dropped is None
    assert sequence.frame_count == 2
    assert sequence.dropped_frames == 1
    assert sorted(path.name for path in sequence.directory.iterdir()) == [
        "frame-000001.png",
        "frame-000002.png",
    ]


@pytest.mark.parametrize("max_frames", [0, 3601, 1.5])
def test_video_frame_sequence_rejects_invalid_limits(
    tmp_path: Path,
    max_frames: object,
) -> None:
    with pytest.raises(ValueError, match="max_frames"):
        VideoFrameSequence(tmp_path, max_frames=max_frames)
