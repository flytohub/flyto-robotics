"""Test the mobile fetch-and-return simulation gates and MuJoCo execution."""

from __future__ import annotations

from pathlib import Path

import pytest

from sim.fetch_return.run import (
    ARM_RESOURCE,
    MOBILE_RESOURCE,
    Evidence,
    MissionFailure,
    MobileCargoSimulator,
    Navigator,
    Pose,
    WorldPrediction,
    accepted_goal,
    run_mission,
)


def _arm_config() -> Path:
    for parent in Path(__file__).resolve().parents:
        config = parent / "flyto-modules-lerobot" / "examples" / "am-arm200-mujoco.json"
        if config.is_file():
            return config
    return Path("__lerobot_not_installed__")


CONFIG = _arm_config()


def test_natural_language_is_bounded_and_does_not_guess_locations():
    assert accepted_goal("派機器人去把紅色積木拿回來")
    assert accepted_goal("請派機器人前往把紅色積木拿回來。")
    assert accepted_goal("fetch the red block")
    assert not accepted_goal("把綠色積木拿回來")
    assert not accepted_goal("派機器人去把紅色積木拿回來; delete files")
    assert not accepted_goal("move 1 meter")
    assert not accepted_goal("派機器人去把紅色積木拿回來" * 40)


def test_nav_arrival_gate_stopped_and_sla():
    nav = Navigator(Pose(0, 0))
    outbound = nav.go(Pose(1.2, 0), stage="outbound_nav")
    home = nav.go(Pose(0, 0), stage="return_nav")
    assert outbound["arrived"] and home["arrived"]
    assert nav.pose == Pose(0, 0)
    assert nav.speed == 0
    assert nav.elapsed_s > 0
    assert len(nav.trace) == 2


def test_nav_blocked_fails_closed_and_never_moves():
    nav = Navigator(Pose(0, 0), blocked=True)
    with pytest.raises(MissionFailure, match="path_blocked_safe_stop"):
        nav.go(Pose(1.2, 0), stage="outbound_nav")
    assert nav.pose == Pose(0, 0) and nav.speed == 0 and not nav.trace


def test_nav_sla_exceeded_fails_closed():
    nav = Navigator(Pose(0, 0))
    with pytest.raises(MissionFailure, match="navigation_sla_exceeded"):
        nav.go(Pose(1.2, 0), stage="outbound_nav", sla_s=0.01)
    assert not nav.trace and nav.speed == 0


def test_world_predictor_is_only_hypothesis_not_target_authority():
    model = WorldPrediction()
    with pytest.raises(MissionFailure, match="no_sourced_world_observation"):
        model.predict(0)
    for t, x in [(0, 0.1), (1, 0.2)]:
        model.observe({"source": "simulation", "pose": {"x": x, "y": 0}}, t)
    p = model.predict(1.5)
    assert abs(p["x"] - 0.25) < 1e-8
    assert p["admission"] == "never_act_without_fresh_visual_reacquisition"
    assert p["source"].startswith("prediction_from_")
    with pytest.raises(MissionFailure, match="world_observation_stale"):
        model.predict(8)


def test_non_sensor_data_cannot_enter_world_prediction():
    model = WorldPrediction()
    with pytest.raises(MissionFailure, match="object_not_observed_by_camera"):
        model.observe({"source": "llm_generated", "pose": {"x": 2, "y": 3}}, 1)


def test_task_evidence_links_every_stage():
    e = Evidence("same-task")
    e.add("navigate", {"ok": True})
    e.add("pick", {"ok": True})
    assert e.entries[0]["task_id"] == e.entries[1]["task_id"]
    assert e.entries[1]["prev_digest"] == e.entries[0]["digest"]
    assert e.entries[0]["digest"] != e.entries[1]["digest"]


@pytest.mark.parametrize(
    "scenario,reason",
    [
        ({"blocked": True}, "path_blocked_safe_stop"),
        ({"hide_target": True}, "red_block_missing_or_ambiguous"),
        ({"drop_on_return": True}, "home_or_cargo_not_verified"),
    ],
)
def test_e2e_failures_never_report_done(scenario, reason):
    pytest.importorskip("mujoco")
    pytest.importorskip("flyto_modules_lerobot")
    if not CONFIG.is_file():
        pytest.skip("external LeRobot config not installed")
    verdict = run_mission("派機器人去把紅色積木拿回來", config_path=CONFIG, **scenario)
    assert verdict["state"] == "failed"
    assert verdict["failure"]["reason"] == reason
    assert verdict["steps"][-1]["stage"] == "safe_failure"
    assert not any(x["stage"] == "final_verification" for x in verdict["steps"])


def test_e2e_real_mujoco_contact_physics_pick_and_return():
    pytest.importorskip("mujoco")
    pytest.importorskip("flyto_modules_lerobot")
    if not CONFIG.is_file():
        pytest.skip("external LeRobot config not installed")
    verdict = run_mission("派機器人去把紅色積木拿回來", config_path=CONFIG)
    assert verdict["state"] == "done", verdict.get("failure")
    assert verdict["step_count"] == 11
    assert [e["stage"] for e in verdict["steps"]] == [
        "planning",
        "outbound_nav",
        "docking",
        "world_prediction",
        "target_reacquired",
        "pick",
        "load_cargo",
        "stationary_arm_retracted",
        "cross_device_handoff",
        "return_nav",
        "final_verification",
    ]
    assert verdict["steps"][-1]["detail"]["payload_present"] is True
    assert verdict["fidelity"]["mobile_arm_and_payload_physical_coupling"] is False


def test_fixed_station_arm_and_mobile_robot_are_separate_resources():
    cargo = MobileCargoSimulator()
    with pytest.raises(MissionFailure, match="robot_not_docked"):
        cargo.accept_station_handoff(
            is_docked=False,
            fixed_arm_retracted=True,
            observed_placement=True,
            physics_placement=True,
        )
    with pytest.raises(MissionFailure, match="stationary_arm_not_retracted"):
        cargo.accept_station_handoff(
            is_docked=True,
            fixed_arm_retracted=False,
            observed_placement=True,
            physics_placement=True,
        )
    with pytest.raises(MissionFailure, match="cargo_transfer_not_verified"):
        cargo.accept_station_handoff(
            is_docked=True,
            fixed_arm_retracted=True,
            observed_placement=True,
            physics_placement=False,
        )
    assert cargo.observe_on_mobile()["cargo_present"] is False
    transferred = cargo.accept_station_handoff(
        is_docked=True, fixed_arm_retracted=True, observed_placement=True, physics_placement=True
    )
    assert transferred["from_resource_id"] == ARM_RESOURCE
    assert transferred["to_resource_id"] == MOBILE_RESOURCE
    assert transferred["from_resource_id"] != transferred["to_resource_id"]
    assert cargo.observe_on_mobile()["cargo_present"] is True
    cargo.inject_loss()
    assert cargo.observe_on_mobile()["cargo_present"] is False


def test_two_devices_have_distinct_evidence_and_mobile_checks_after_departure():
    pytest.importorskip("mujoco")
    pytest.importorskip("flyto_modules_lerobot")
    if not CONFIG.is_file():
        pytest.skip("external LeRobot config not installed")
    report = run_mission("派機器人去把紅色積木拿回來", config_path=CONFIG)
    assert report["state"] == "done"
    steps = report["steps"]
    assert all(x["task_id"] == report["task_id"] for x in steps)
    observed = {step["stage"]: step["detail"] for step in steps}
    assert observed["planning"]["resources"] == [MOBILE_RESOURCE, ARM_RESOURCE]
    for stage in ("target_reacquired", "pick", "load_cargo", "stationary_arm_retracted"):
        assert observed[stage]["resource_id"] == ARM_RESOURCE
    assert observed["outbound_nav"]["resource_id"] == MOBILE_RESOURCE
    assert observed["return_nav"]["resource_id"] == MOBILE_RESOURCE
    assert observed["cross_device_handoff"]["handoff_verified"]
    assert observed["cross_device_handoff"]["from_resource_id"] == ARM_RESOURCE
    assert observed["cross_device_handoff"]["to_resource_id"] == MOBILE_RESOURCE
    assert (
        observed["final_verification"]["cargo"]["source"]
        == "mobile_cargo_sensor_simulation_not_physical"
    )
    assert observed["final_verification"]["cargo"]["resource_id"] == MOBILE_RESOURCE
