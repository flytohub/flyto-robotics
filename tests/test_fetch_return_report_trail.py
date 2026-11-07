"""The offline mission's tray and report trail, checked without MuJoCo.

``run.py`` is the producer of the cargo bridge's reports (finding 11). The
MuJoCo-backed tests of it are skipped by ``make verify`` in the repository
venv (no ``mujoco`` / ``flyto_modules_lerobot``), so the arm is stubbed here:
everything ``run_mission`` itself decides (the tray's unload at home, the
report lines it appends, their order) runs under the required gate.
"""

from __future__ import annotations

import sys
import types
from types import SimpleNamespace

import pytest

from sim.fetch_return import arm_reports as reports
from sim.fetch_return import run

GOAL = "派機器人去把紅色積木拿回來"


class StubArm:
    """The few calls ``run_mission`` makes of the MuJoCo arm, answered as a good run."""

    def __init__(self, _config_path):
        block = SimpleNamespace(xpos=[0.22, 0.10, 0.08])
        self.world = SimpleNamespace(data=SimpleNamespace(body=lambda _name: block))

    def find(self, _label):
        return {"id": "red-1", "source": "simulation", "pose": {"x": 0.2, "y": 0.1, "z": 0.01}}

    def call(self, capability, _args=None):
        evidence = {
            "manipulation.pick": {"object_held": True},
            "manipulation.place": {
                "perception_after": {"object_in_container": True},
                "released": True,
                "object_held": False,
            },
        }.get(capability, {})
        return SimpleNamespace(outcome="completed", evidence=evidence)

    def independent_cargo_check(self):
        return {"source": "stub", "in_cargo": True, "xy_error_m": 0.0, "z_m": 0.012}

    def push_settle_readings(self, path, *, count=8, every_s=0.1):
        for i in range(count):
            reports.append_report(
                path,
                reports.tray_reading(
                    inside=True, held=False, block=[0.22, 0.10, 0.012], sim_t=i * every_s
                ),
            )

    def inject_block_displacement(self, *_xyz):
        raise AssertionError("not used by these runs")


@pytest.fixture
def stub_arm(monkeypatch):
    monkeypatch.setattr(run, "ArmRig", StubArm)
    package = types.ModuleType("flyto_modules_lerobot")
    mujoco_sim = types.ModuleType("flyto_modules_lerobot.mujoco_sim")
    mujoco_sim.reset_worlds = lambda: None
    package.mujoco_sim = mujoco_sim
    monkeypatch.setitem(sys.modules, "flyto_modules_lerobot", package)
    monkeypatch.setitem(sys.modules, "flyto_modules_lerobot.mujoco_sim", mujoco_sim)


def test_a_delivered_mission_unloads_the_tray_at_home(stub_arm, tmp_path):
    verdict = run.run_mission(GOAL, config_path=tmp_path / "unused.json")
    assert verdict["state"] == "done", verdict.get("failure")
    stages = [step["stage"] for step in verdict["steps"]]
    assert stages[-2:] == ["final_verification", "unload"]
    unload = verdict["steps"][-1]["detail"]
    assert unload["unloaded"] is True and unload["cargo_present"] is False


def test_a_delivered_mission_reports_the_unload_and_the_mission_end_last(stub_arm, tmp_path):
    path = tmp_path / "reports.jsonl"
    path.write_text("")
    tail = reports.ArmReportTail(path)
    verdict = run.run_mission(GOAL, config_path=tmp_path / "unused.json", reports_path=path)
    assert verdict["state"] == "done"
    trail = tail.read_new()
    assert tail.rejected == 0
    kinds = [report.kind for report in trail]
    assert kinds[0] == "place"
    assert kinds[-3:] == ["retract", "unload", "mission_end"]
    assert trail[-1].fields["state"] == "done"


def test_a_failed_mission_still_reports_its_end(stub_arm, tmp_path):
    path = tmp_path / "reports.jsonl"
    path.write_text("")
    tail = reports.ArmReportTail(path)
    verdict = run.run_mission(
        GOAL, config_path=tmp_path / "unused.json", reports_path=path, drop_on_return=True
    )
    assert verdict["state"] == "failed"
    kinds = [report.kind for report in tail.read_new()]
    assert "unload" not in kinds
    assert kinds[-2:] == ["loss", "mission_end"]
