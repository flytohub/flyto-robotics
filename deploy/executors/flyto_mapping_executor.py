#!/usr/bin/python3
"""Record a venue map as a job flyto-cloud dispatches, not a shell someone ssh'es into.

`deploy/make-map.sh` does the same work and is the wrong shape for the place it
is needed: at a venue, on the day, nobody opens a laptop and ssh'es into the
robot. Everything else this robot can do arrives as a capability in the catalog
at ``GET /v1/capabilities`` and is invoked as a workflow step. Mapping was the
one operation that did not, so it was the one operation that needed a person
with a terminal.

## Why this is a device executor and not a mission primitive

Adding ``start_slam`` to :mod:`flyto_robotics.workflow` would put it in the same
vocabulary the AI planner composes delivery plans out of, and a plan that can
emit "begin remapping the building" halfway through carrying something is a
plan shape nobody wants to have to refuse. Mapping is commissioning: it happens
once per venue, before deliveries, and it belongs on the generic installed
executor protocol that ``flyto_job_runner`` already routes anything not
prefixed ``robotics.`` to.

## The environment this runs in

The registry starts this with ``env={}``, ``cwd="/"``, stderr discarded, and a
bounded timeout it will kill through. So: no PATH, no ROS environment, no
inherited anything — every binary is named absolutely, and ROS is sourced by an
explicit ``bash -lc`` for the two calls that need it. Nothing here may block for
longer than the manifest's timeout, which is why starting SLAM hands off to
systemd rather than holding it: a mapping run outlives this process by design,
since the driving happens between the request that starts it and the one that
saves the map.

## What it refuses, and why refusing is the point

A mapping run is minutes of continuous driving. Starting one that cannot finish
wastes the drive and, on a pack taken below its floor, damages it. So the
preconditions are checked before SLAM starts rather than discovered when the
base browns out mid-loop, and a refusal is reported as ``refused`` with a code —
a status the contract keeps distinct from ``failed`` precisely so "this was not
safe to begin" does not read as "this broke".
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

CONTRACT_VERSION = "device-executor-v1"

BASH = "/bin/bash"
SUDO = "/usr/bin/sudo"
SYSTEMCTL = "/usr/bin/systemctl"

SLAM_UNIT = "flyto-slam.service"
BRINGUP_UNIT = "turtlebot3-bringup.service"
NAV2_UNIT = "flyto-nav2.service"

MAP_DIR = Path("/home/ubuntu/.flyto/maps")
ROS_SETUP = "/opt/ros/jazzy/setup.bash"
ROS_DOMAIN_ID = "30"

# 11.6 V and not the 11.0 V an idle pack sits at: a mapping run drives for
# minutes and the sag under motor load is immediate. Checked once, at the
# start, so the number has to carry the whole run.
MIN_MAPPING_VOLTS = 11.6

# Bounded well inside the manifest's own timeout so a slow ROS call is reported
# as a refusal by this process rather than as a kill by the registry, which
# would lose the reason.
ROS_CALL_TIMEOUT = 20
SAVE_TIMEOUT = 60

MAX_MAP_NAME = 64


def _result(status: str, reason_code: str, *, detail: str = "",
            evidence: list | None = None) -> dict:
    """One result shape, so no caller has to remember the evidence rule.

    The contract refuses evidence on any status other than ``succeeded`` — a
    step that did not happen must not be able to hand back proof that it did.
    """
    payload = {
        "contract_version": CONTRACT_VERSION,
        "status": status,
        "reason_code": reason_code,
        "evidence": list(evidence or ()) if status == "succeeded" else [],
    }
    if detail:
        payload["detail"] = detail[:1024]
    return payload


def _unit_active(unit: str) -> bool:
    try:
        done = subprocess.run(
            [SYSTEMCTL, "is-active", "--quiet", unit],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=10, env={},
        )
    except (OSError, subprocess.SubprocessError):
        return False
    return done.returncode == 0


def _ros(command: str, timeout: int) -> tuple[int, str]:
    """Run one ROS command in a sourced login shell. Returns (code, stdout)."""
    try:
        done = subprocess.run(
            [BASH, "-lc", f"source {ROS_SETUP} && export ROS_DOMAIN_ID={ROS_DOMAIN_ID} "
                          f"&& {command}"],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=timeout, env={"HOME": "/home/ubuntu", "USER": "ubuntu"},
        )
    except subprocess.TimeoutExpired:
        return 124, ""
    except (OSError, subprocess.SubprocessError):
        return 1, ""
    return done.returncode, (done.stdout or "")


def _battery_volts() -> float | None:
    code, out = _ros(
        "timeout 15 ros2 topic echo /battery_state --once --field voltage", ROS_CALL_TIMEOUT,
    )
    if code != 0:
        return None
    for line in out.splitlines():
        try:
            return float(line.strip())
        except ValueError:
            continue
    return None


def _map_name(params: dict) -> str:
    """A map name that cannot escape the map directory.

    This value reaches a filesystem path, so it is whitelisted rather than
    sanitised: anything with a separator, a dot-segment or an unexpected
    character is refused outright instead of being rewritten into something
    that looks safe and is not.
    """
    raw = params.get("map_name", "lab")
    if not isinstance(raw, str) or not 0 < len(raw) <= MAX_MAP_NAME:
        raise ValueError("map_name_invalid")
    if not raw.replace("-", "").replace("_", "").isalnum():
        raise ValueError("map_name_invalid")
    return raw


def start(params: dict) -> dict:
    if _unit_active(SLAM_UNIT):
        return _result("refused", "mapping_already_running",
                       detail="SLAM is already recording. Save or abort the run in progress.")
    if _unit_active(NAV2_UNIT):
        return _result("refused", "navigation_running",
                       detail="Nav2 is running and both publish /map. Stop navigation first.")
    if not _unit_active(BRINGUP_UNIT):
        return _result("refused", "sensors_unavailable",
                       detail="turtlebot3-bringup is not running, so there is no scan or odometry.")

    volts = _battery_volts()
    if volts is None:
        return _result("refused", "battery_unknown",
                       detail="Battery state could not be read, so a run long enough to "
                              "matter cannot be started safely.")
    if volts < MIN_MAPPING_VOLTS:
        return _result("refused", "battery_too_low",
                       detail=f"{volts:.2f} V is below the {MIN_MAPPING_VOLTS} V a mapping "
                              f"run needs. Charge before starting.")

    try:
        done = subprocess.run(
            [SUDO, "-n", SYSTEMCTL, "start", SLAM_UNIT],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=30, env={},
        )
    except (OSError, subprocess.SubprocessError):
        return _result("failed", "slam_start_failed")
    if done.returncode != 0:
        return _result("failed", "slam_start_failed",
                       detail="systemctl refused to start SLAM.")

    return _result(
        "succeeded", "mapping_started",
        detail=f"SLAM recording at {volts:.2f} V. Drive the robot over the whole space, "
               f"revisiting where it has already been, then save.",
        evidence=[{
            "kind": "mapping.session",
            "usable": True,
            "detail": f"slam_toolbox recording, battery {volts:.2f} V",
        }],
    )


def save(params: dict) -> dict:
    try:
        name = _map_name(params)
    except ValueError:
        return _result("refused", "map_name_invalid")

    if not _unit_active(SLAM_UNIT):
        return _result("refused", "mapping_not_running",
                       detail="Nothing is recording, so there is no map to save.")

    target = MAP_DIR / name
    try:
        MAP_DIR.mkdir(parents=True, exist_ok=True)
    except OSError:
        return _result("failed", "map_directory_unwritable")

    code, _ = _ros(
        f"ros2 run nav2_map_server map_saver_cli -f {target} "
        f"--ros-args -p save_map_timeout:=10000.0",
        SAVE_TIMEOUT,
    )
    yaml_path, image_path = target.with_suffix(".yaml"), target.with_suffix(".pgm")
    if code != 0 or not yaml_path.is_file() or not image_path.is_file():
        return _result("failed", "map_save_failed",
                       detail="map_saver_cli did not produce both a .yaml and a .pgm.")

    # Read back what was written rather than reporting what was asked for. A
    # saver that wrote a map of nothing still exits zero, and a one-cell map is
    # the failure that would otherwise reach Nav2 as a working venue.
    resolution, cells = "", ""
    try:
        for line in yaml_path.read_text(encoding="utf-8").splitlines():
            if line.startswith("resolution:"):
                resolution = line.split(":", 1)[1].strip()
        header = image_path.read_bytes()[:64].split(b"\n")
        for part in header[1:4]:
            text = part.decode("ascii", "replace").strip()
            if text and not text.startswith("#") and " " in text:
                cells = text
                break
    except (OSError, UnicodeError):
        pass

    # SLAM has done its job; leaving it holding /map is what stops Nav2 from
    # being startable next, and Conflicts= would then stop the map from ever
    # being used by the thing it was recorded for.
    subprocess.run([SUDO, "-n", SYSTEMCTL, "stop", SLAM_UNIT],
                   stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL, timeout=40, env={}, check=False)

    detail = f"{name}: {cells or 'unknown'} cells at {resolution or 'unknown'} m"
    return _result(
        "succeeded", "map_recorded", detail=detail,
        evidence=[{"kind": "map.recorded", "usable": True, "detail": detail}],
    )


def abort(params: dict) -> dict:
    if not _unit_active(SLAM_UNIT):
        return _result("succeeded", "mapping_not_running",
                       detail="Nothing was recording.", evidence=[])
    try:
        subprocess.run([SUDO, "-n", SYSTEMCTL, "stop", SLAM_UNIT],
                       stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=40, env={}, check=False)
    except (OSError, subprocess.SubprocessError):
        return _result("failed", "slam_stop_failed")
    return _result("succeeded", "mapping_aborted", detail="Recording discarded.")


HANDLERS = {"mapping.start": start, "mapping.save": save, "mapping.abort": abort}


def main() -> int:
    try:
        request = json.loads(sys.stdin.read(65_536) or "{}")
    except (ValueError, OSError):
        print(json.dumps(_result("failed", "request_unreadable")), flush=True)
        return 0
    if not isinstance(request, dict) or request.get("contract_version") != CONTRACT_VERSION:
        print(json.dumps(_result("failed", "contract_version_unsupported")), flush=True)
        return 0

    handler = HANDLERS.get(request.get("module_id"))
    if handler is None:
        print(json.dumps(_result("refused", "module_not_supported")), flush=True)
        return 0

    params = request.get("params")
    if not isinstance(params, dict):
        params = {}

    try:
        result = handler(params)
    except Exception:  # noqa: BLE001 - a crash must still be a valid result
        # The registry reads stdout, not exit codes, and an executor that dies
        # without answering is indistinguishable from one that hung.
        result = _result("failed", "executor_error")
    print(json.dumps(result), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
