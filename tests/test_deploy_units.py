"""`validate_unit` over the units that are actually installed on the robot.

The validator already runs over *rendered* lifecycle units
(`tests/test_lifecycle.py`, `tests/test_service_readiness.py`) and over
`/etc/systemd/system/flyto-*` in `support_bundle.py`. The hand-written units in
`deploy/systemd/` — the ones on this Pi, placed there by hand rather than by the
installer — were checked by nothing, and two units added on 2026-08-27 shipped
with `Restart=on-failure` and no start limit at all.

So this pins an explicit expectation per file rather than asserting everything
is clean. A blanket "no defects" assertion would have to be introduced red, and
a red suite gets an `xfail` rather than a fix. A table means the debt is visible,
each entry has to be justified in writing, and **anything new fails**.
"""

from __future__ import annotations

import pathlib

import pytest

from flyto_robotics.systemd_units import validate_unit

UNIT_DIR = pathlib.Path(__file__).resolve().parents[1] / "deploy" / "systemd"

# `home_path_hardcoded` is one finding repeated across this whole directory, and
# it is a true statement about a deliberate deployment shape rather than a bug
# to fix unit by unit. This robot was commissioned by hand: the release tree, the
# environment files, the runner venv and the map directory all live under
# /home/ubuntu, and the validator's rule is written for *product* units that a
# customer installs, where a home path means the unit breaks for any account but
# the one that built it. Moving one unit's paths to /var/lib while its siblings,
# its release tree and the executor that writes its inputs all stay in /home
# would make the check pass and the deployment less coherent.
#
# Recorded here rather than silenced with `allow_home_paths=True`, because the
# flag would also hide a genuinely new home-path dependency in a unit that has
# none today.
HOME_PATHS = frozenset({"home_path_hardcoded"})

EXPECTED: dict[str, frozenset[str]] = {
    # Added 2026-08-27. The stream unit is clean, and both are the proof this
    # table is reachable: each had `unbounded_restart` when written and this
    # test is what found it. The v4l2 unit gained a home path later the same
    # day, when `camera_info_url` was pointed at a stable operator-chosen
    # calibration file -- the alternative was leaving it on a default derived
    # from the camera's USB product string, which changes when the camera is
    # swapped and takes the intrinsics silently with it.
    "flyto-camera-v4l2.service": HOME_PATHS,
    "flyto-camera-stream.service": frozenset(),
    "flyto-slam.service": frozenset(),
    "flyto-camera-gateway.service": HOME_PATHS,
    "flyto-nav2.service": HOME_PATHS,
    # Pre-existing. Not this file's job to fix, but recorded so the next change
    # to one of them cannot quietly add a defect class it did not already have.
    "flyto-delivery.service": HOME_PATHS | {"unbounded_restart"},
    "flyto-job-runner.service": HOME_PATHS | {"start_limit_unreachable"},
    "flyto-recovery-portal.service": HOME_PATHS | {"unbounded_restart"},
    "flyto-robot-doctor.service": HOME_PATHS,
    "flyto-shortcut.service": HOME_PATHS | {"unbounded_restart"},
    "turtlebot3-bringup.service": HOME_PATHS,
    "flyto-robot-doctor.timer": frozenset(),
}


def _units() -> list[pathlib.Path]:
    """Every unit and timer, but not drop-ins.

    A `.conf` drop-in legitimately carries a fragment — `ExecStart=` alone to
    clear an inherited value, or a bare `[Unit]` — so validating one as a whole
    unit reports defects that are the file doing its job.
    """
    return sorted(
        [*UNIT_DIR.glob("*.service"), *UNIT_DIR.glob("*.timer")],
        key=lambda p: p.name,
    )


def test_every_shipped_unit_has_an_expectation():
    """A new unit must be added to the table, which is where it gets read."""
    assert {p.name for p in _units()} == set(EXPECTED)


@pytest.mark.parametrize("path", _units(), ids=lambda p: p.name)
def test_a_unit_carries_no_defect_class_it_is_not_recorded_for(path):
    found = {defect.code for defect in validate_unit(path.read_text(encoding="utf-8"),
                                                     name=path.name)}
    expected = EXPECTED[path.name]
    new = found - expected
    assert not new, (
        f"{path.name} gained {sorted(new)}. Fix the unit, or add the code to "
        f"EXPECTED with a written reason for why it is acceptable."
    )
    fixed = expected - found
    assert not fixed, (
        f"{path.name} no longer has {sorted(fixed)} — remove it from EXPECTED so "
        f"the table keeps meaning what it says."
    )


def test_a_restarting_unit_bounds_its_own_restarts():
    """The defect that shipped: `Restart=on-failure` with no start limit.

    systemd will retry such a unit forever. `flyto-shortcut` was found at
    NRestarts=193 flapping a healthy lidar for exactly this reason (see
    handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md), which is why the
    validator learned to look for it.
    """
    for path in _units():
        text = path.read_text(encoding="utf-8")
        if "Restart=on-failure" not in text and "Restart=always" not in text:
            continue
        codes = {d.code for d in validate_unit(text, name=path.name)}
        if "unbounded_restart" in EXPECTED[path.name]:
            continue
        assert "unbounded_restart" not in codes, (
            f"{path.name} restarts without a start limit and is not recorded as "
            f"doing so"
        )
