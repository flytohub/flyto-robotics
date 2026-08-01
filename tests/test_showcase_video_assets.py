from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RENDER_SCRIPT = ROOT / "scripts/render-ai4all-showcase-video.sh"
FILTER = ROOT / "video/ai4all-showcase-filter.txt"
SUBTITLES = ROOT / "video/ai4all-showcase.ass"
MEDICATION_FILTER = ROOT / "video/ai4all-medication-showcase-filter.txt"
MEDICATION_SUBTITLES = ROOT / "video/ai4all-medication-showcase.ass"
GUI_CAPTURE_SCRIPT = ROOT / "scripts/run-ai4all-gui-evidence.sh"
VERIFICATION_RENDER_SCRIPT = ROOT / "scripts/render-ai4all-verification-video.sh"
STORY_RENDER_SCRIPT = ROOT / "scripts/render-ai4all-story-video.sh"
WORLD = ROOT / "worlds/ai4all-branching-route.sdf"
ROVER = ROOT / "models/flyto_rover/model.sdf"
SHOWCASE_DOC = ROOT / "docs/AI4ALL_SHOWCASE.md"


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


def test_gui_evidence_records_a_real_gazebo_window_with_the_same_contract() -> None:
    script = GUI_CAPTURE_SCRIPT.read_text(encoding="utf-8")

    assert "gazebo-gui.mp4" in script
    assert "x11grab" in script
    assert "Xvfb" in script
    assert "headless:=false" in script
    assert "validated-plan.json" in script
    assert "planning-session.json" in script
    assert "driver-manifest.json" in script
    assert "mission-result.json" in script
    assert "gazebo-window-geometry.env" in script
    assert "ffprobe" in script
    assert "sha256sum" in script


def test_verification_video_keeps_one_continuous_gazebo_timeline() -> None:
    script = VERIFICATION_RENDER_SCRIPT.read_text(encoding="utf-8")

    assert "gazebo-gui.mp4" in script
    assert "driver-manifest.json" in script
    assert "mission-result.json" in script
    assert "planning-session.json" in script
    assert "validated-plan.json" in script
    assert "verification-events.ass" in script
    assert "gazebo-window-geometry.env" in script
    assert "verification-video-probe.json" in script
    assert "ffprobe" in script
    assert "sha256" in script
    assert "xfade" not in script
    assert "zoompan" not in script
    assert "-loop 1" not in script


def test_verification_video_exposes_actual_evidence_log_not_only_narration() -> None:
    script = VERIFICATION_RENDER_SCRIPT.read_text(encoding="utf-8")

    assert "Style: EvidenceLog" in script
    assert "實際規劃 LOG" in script
    assert "實際安全 LOG" in script
    assert "實際交付 LOG" in script
    assert "mission-result.json + driver-manifest.json" in script
    assert "minimum_range" in script
    assert "container_locked" in script
    assert "原始未裁切 Gazebo GUI" in script


def test_story_video_explains_the_mission_without_engineering_jargon() -> None:
    script = STORY_RENDER_SCRIPT.read_text(encoding="utf-8")

    required_plain_language = (
        "這是一台醫院送藥機器人",
        "幫 12 號病人送 A12 藥袋",
        "攝影機故障，AI 改走安全路線",
        "有人擋路，機器人自己停下來",
        "錯的藥、不對的人，都不會開鎖",
        "藥袋正確 + 病人正確 = 才能解鎖",
        "這是 Gazebo 模擬，不是實體醫院",
    )
    assert all(copy in script for copy in required_plain_language)
    assert "say -v" in script
    assert "flyto2-hospital-story.mp4" in script


def test_story_video_preserves_source_evidence_and_separates_ai_from_safety() -> None:
    script = STORY_RENDER_SCRIPT.read_text(encoding="utf-8")

    assert "gazebo-gui.mp4" in script
    assert "planning-session.json" in script
    assert "mission-result.json" in script
    assert "driver-manifest.json" in script
    assert "AI 負責理解任務與選路" in script
    assert "安全規則負責決定能不能執行" in script
    assert "不是生成式影片" in script


def test_hospital_scene_is_self_contained_and_visually_explains_the_mission() -> None:
    world = WORLD.read_text(encoding="utf-8")
    rover = ROVER.read_text(encoding="utf-8")

    required_world_models = (
        'name="hospital_corridor_shell"',
        'name="nurse_station"',
        'name="camera_a_marker"',
        'name="camera_b_marker"',
        'name="medication_handoff_zone"',
        'name="Flyto2 Gazebo 3D View"',
        "<camera_pose>0.45 -6.5 6.2 0 0.72 1.5708</camera_pose>",
    )
    required_rover_visuals = (
        'name="locked_medication_container"',
        'name="status_light"',
        'name="camera_housing"',
    )
    assert all(marker in world for marker in required_world_models)
    assert all(marker in rover for marker in required_rover_visuals)
    assert "http://" not in world
    assert "https://" not in world
    assert "http://" not in rover
    assert "https://" not in rover


def test_showcase_documentation_distinguishes_verification_from_promotion() -> None:
    documentation = SHOWCASE_DOC.read_text(encoding="utf-8")

    assert "run-ai4all-gui-evidence.sh" in documentation
    assert "render-ai4all-verification-video.sh" in documentation
    assert "連續時間線" in documentation
    assert "不使用生成式影像" in documentation
