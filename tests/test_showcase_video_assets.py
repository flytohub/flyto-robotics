from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RENDER_SCRIPT = ROOT / "scripts/render-ai4all-showcase-video.sh"
FILTER = ROOT / "video/ai4all-showcase-filter.txt"
SUBTITLES = ROOT / "video/ai4all-showcase.ass"
MEDICATION_FILTER = ROOT / "video/ai4all-medication-showcase-filter.txt"
MEDICATION_SUBTITLES = ROOT / "video/ai4all-medication-showcase.ass"


def test_video_composes_both_real_gazebo_camera_streams() -> None:
    script = RENDER_SCRIPT.read_text(encoding="utf-8")
    filter_text = FILTER.read_text(encoding="utf-8")

    assert "-i /evidence/gazebo-active-camera.mp4" in script
    assert "-i /evidence/gazebo-overhead.mp4" in script
    assert "tpad=stop_mode=clone:stop_duration=3.0" in filter_text
    assert "[0:v]" in filter_text
    assert "[1:v]" in filter_text
    assert "[2:v]" in filter_text


def test_video_copy_exposes_branching_ai_and_runtime_proof() -> None:
    copy = SUBTITLES.read_text(encoding="utf-8")

    required = (
        "產生 8 條候選 → 黃—紫",
        "黃線 4 條候選已排除",
        "橘—紫 · 驗證通過",
        "計畫 sha256  9cbba151cd9b",
        "LiDAR 障礙 → 零速安全停止",
        "QR 簽章通過",
        "同一 nonce 重放拒絕",
        "16 / 16",
        "26 / 26",
        "5.526 m",
    )
    assert all(text in copy for text in required)
    assert "依序通過藍、黃、紫區" not in copy
    assert "12 / 12" not in copy
    assert "28 / 28" not in copy


def test_medication_video_exposes_fail_closed_handoff_proof() -> None:
    script = RENDER_SCRIPT.read_text(encoding="utf-8")
    filter_text = MEDICATION_FILTER.read_text(encoding="utf-8")
    copy = MEDICATION_SUBTITLES.read_text(encoding="utf-8")

    assert "driver-manifest.json" in script
    assert "guarded_handoff" in script
    assert "ai4all-medication-showcase-filter.txt" in script
    assert "ai4all-medication-showcase.ass" in script
    assert "d=180" in filter_text
    assert "B13 不符 → 保持上鎖" in copy
    assert "A12 通過 → checkpoint 恢復" in copy
    assert "patient-13 不符 → 保持上鎖" in copy
    assert "patient-12 通過 → 才能解鎖" in copy
    assert "21 / 21" in copy
    assert "28 / 28" in copy
