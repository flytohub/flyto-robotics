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

## The wire protocol, which the first version of this file got wrong twice

``_StdioOwner`` in ``deploy/device_executor_registry.py`` is the only caller
that matters, and it is stricter than it looks. Both mistakes below shipped and
made this executor fail 100% of the time, for every module id, silently to
anyone reading only this file:

**It is two phases, not one.** ``prepare`` receives
``{"contract_version","operation":"prepare","request":{...}}`` and returns an
opaque JSON payload; ``execute`` receives that payload back under
``{"operation":"execute","prepared":{...}}`` and returns the result. The
``module_id`` is *nested inside* ``request``, never at the top level.

**Exactly one JSON value, and nothing after it.** ``_call`` does
``raw_decode(text)`` and then ``if end != len(text): raise
RegistryError("stdio_output_invalid")``. ``print()`` appends a newline, so every
response it wrote was rejected before it was ever parsed. Use
``sys.stdout.write`` and emit no trailing byte.

Everything checkable is checked in ``execute``. ``prepare`` can only fail by
raising, which the registry reports as ``prepare_failed`` with no reason code
of ours attached, so a refusal decided there would lose the very thing the
contract keeps ``refused`` distinct from ``failed`` to carry.

## Where its settings come from

Everything this needs to know about the machine it is on lives in a JSON file
named by ``--config`` in the manifest's own argv — see
:mod:`mapping_settings`. It used to be thirteen constants, which meant a second
robot with a different SLAM unit or a 4S pack was a fork of this file rather
than a settings edit.

Not the environment, though the camera gateway built the same day uses exactly
that: the registry starts an executor with ``env={}``, so there is nothing to
read. The manifest is already per-robot data and already carries the full argv,
so it carries the path too.

## The environment this runs in

The registry starts this with ``env={}``, ``cwd="/"``, stderr discarded, and a
bounded timeout it will kill through. So every binary is named absolutely.
``_ros`` is the exception and is honest about it: ROS needs a login shell, so
those two calls run ``bash -lc`` with ``HOME`` set, which sources a profile
owned by the account this already runs as. That crosses no boundary here, but
it is not "no inherited anything" and the earlier docstring claiming so was
wrong.

Nothing may block longer than the manifest's timeout, which is why starting
SLAM hands off to systemd rather than holding it: a mapping run outlives this
process by design, since the driving happens between the request that starts it
and the one that saves the map.

## What it refuses, and why refusing is the point

A mapping run is minutes of continuous driving. Starting one that cannot finish
wastes the drive and, on a pack taken below its floor, damages it. So the
preconditions are checked before SLAM starts rather than discovered when the
base browns out mid-loop, and a refusal is reported as ``refused`` with a code —
a status the contract keeps distinct from ``failed`` precisely so "this was not
safe to begin" does not read as "this broke".
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pwd
import re
import shlex
import subprocess
import sys
from pathlib import Path

if __package__:
    from .mapping_settings import MappingSettings, MappingSettingsError
else:  # Installed as a standalone executor beside mapping_settings.py.
    from mapping_settings import MappingSettings, MappingSettingsError

CONTRACT_VERSION = "device-executor-v1"

BASH = "/bin/bash"
SUDO = "/usr/bin/sudo"
SYSTEMCTL = "/usr/bin/systemctl"

DEFAULT_CONFIG = Path("/etc/flyto/mapping.json")

# Bounded well inside the manifest's own timeout so a slow ROS call is reported
# as a refusal by this process rather than as a kill by the registry, which
# would lose the reason.
ROS_CALL_TIMEOUT = 20
SAVE_TIMEOUT = 60
# `timeout` inside the shell, not just around it. `bash -lc` execs into `ros2`,
# which spawns map_saver_cli as its own child, so killing the process this
# started leaves the saver orphaned to PID 1 -- still holding the deadline it
# was given. GNU timeout signals the process group, so nothing survives it.
SAVER_SHELL_TIMEOUT = 55
SAVER_MAP_TIMEOUT = 45.0

# A map name reaches a filesystem path and a shell command line, so it is
# whitelisted rather than sanitised. `.isalnum()` is not this: it spans all of
# Unicode's letter and number categories, so `café`, `лаб` and -- the one that
# actually hurts -- fullwidth `ｌａｂ`, indistinguishable from `lab` in a
# directory listing, all passed.
MAP_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}\Z")


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


def _unit_active(unit: str) -> bool | None:
    """Whether a unit is active; ``None`` means systemd could not answer.

    ``systemctl is-active`` uses 3 for an honestly inactive unit. Other
    non-zero values include unknown units and D-Bus failures, neither of which
    is evidence that it is safe to start a conflicting stack.
    """
    try:
        done = subprocess.run(
            [SYSTEMCTL, "is-active", "--quiet", unit],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=10, env={},
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if done.returncode == 0:
        return True
    if done.returncode == 3:
        return False
    return None


def _stop_slam(settings: MappingSettings) -> int | None:
    """Stop SLAM and report the return code. ``None`` means it could not be run.

    Returned rather than discarded because a stop can genuinely fail: a
    concurrent ``flyto-nav2`` start cancels this job (the unit declares
    ``Conflicts=``), and D-Bus errors surface the same way. Reporting success
    over a unit that is still holding ``/map`` would leave the next
    ``mapping.start`` refusing for a reason nobody was told about.
    """
    try:
        done = subprocess.run(
            [SUDO, "-n", SYSTEMCTL, "stop", settings.slam_unit],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=40, env={},
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return done.returncode


def _ros(settings: MappingSettings, command: str, timeout: int) -> tuple[int, str]:
    """Run one ROS command in a sourced login shell. Returns (code, stdout)."""
    shell_parts = []
    if settings.ros_setup is not None:
        shell_parts.append(f"source {shlex.quote(settings.ros_setup)}")
    if settings.ros_domain_id is not None:
        shell_parts.append(f"export ROS_DOMAIN_ID={settings.ros_domain_id}")
    shell_parts.append(command)
    runtime_account = pwd.getpwuid(os.getuid())
    try:
        done = subprocess.run(
            [BASH, "-lc", " && ".join(shell_parts)],
            stdin=subprocess.DEVNULL, capture_output=True, text=True,
            timeout=timeout,
            env={"HOME": runtime_account.pw_dir, "USER": runtime_account.pw_name},
        )
    except subprocess.TimeoutExpired:
        return 124, ""
    except (OSError, subprocess.SubprocessError):
        return 1, ""
    return done.returncode, (done.stdout or "")


def _battery_volts(settings: MappingSettings) -> float | None:
    """The pack voltage, or ``None`` when it cannot be believed.

    ``None`` covers unreadable *and* unbelievable. A bare ``float()`` accepts
    ``nan``, and every comparison against NaN is False — so a NaN reading would
    slip past the floor check and start a mapping run whose evidence reads
    "battery nan V". ``0.0`` gets the same treatment: turtlebot3_node fills
    unmeasured fields with zero rather than NaN, so a zero is far more likely to
    be "nothing reported" than a pack at zero volts, and answering
    ``battery_too_low`` to it would send someone to charge a battery that is
    fine.
    """
    if not settings.checks_battery:
        return None
    code, out = _ros(
        settings,
        f"timeout 15 ros2 topic echo {shlex.quote(settings.battery_topic)} "
        "--once --field voltage",
        ROS_CALL_TIMEOUT,
    )
    if code != 0:
        return None
    for line in out.splitlines():
        try:
            value = float(line.strip())
        except ValueError:
            continue
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value
    return None


def _map_name(params: dict) -> str:
    """A map name that cannot escape the map directory or the shell."""
    raw = params.get("map_name", "lab")
    if not isinstance(raw, str) or MAP_NAME.fullmatch(raw) is None:
        raise ValueError("map_name_invalid")
    return raw


def start(params: dict, settings: MappingSettings) -> dict:
    slam_active = _unit_active(settings.slam_unit)
    if slam_active is None:
        return _result("failed", "unit_state_unknown",
                       detail="The configured SLAM unit could not be inspected.")
    if slam_active:
        return _result("refused", "mapping_already_running",
                       detail="SLAM is already recording. Save or abort the run in progress.")
    if settings.navigation_unit is not None:
        navigation_active = _unit_active(settings.navigation_unit)
        if navigation_active is None:
            return _result("failed", "unit_state_unknown",
                           detail="The configured navigation unit could not be inspected.")
        if navigation_active:
            return _result("refused", "navigation_running",
                           detail="Navigation is running and both stacks publish /map. "
                                  "Stop navigation first.")
    if settings.readiness_unit is not None:
        readiness_active = _unit_active(settings.readiness_unit)
        if readiness_active is None:
            return _result("failed", "unit_state_unknown",
                           detail="The configured readiness unit could not be inspected.")
        if not readiness_active:
            return _result("refused", "sensors_unavailable",
                           detail="The configured readiness unit is not running, so scan or "
                                  "odometry cannot be trusted.")

    volts = None
    if settings.checks_battery:
        volts = _battery_volts(settings)
        if volts is None:
            return _result("refused", "battery_unknown",
                           detail="Battery state could not be read or was not a believable "
                                  "voltage, so a run long enough to matter cannot be "
                                  "started safely.")
        if volts < settings.min_mapping_volts:
            return _result(
                "refused",
                "battery_too_low",
                detail=f"{volts:.2f} V is below the {settings.min_mapping_volts} V a "
                       "mapping run needs. Charge before starting.",
            )

    try:
        done = subprocess.run(
            [SUDO, "-n", SYSTEMCTL, "start", settings.slam_unit],
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL, timeout=30, env={},
        )
    except (OSError, subprocess.SubprocessError):
        return _result("failed", "slam_start_failed")
    if done.returncode != 0:
        return _result("failed", "slam_start_failed",
                       detail="systemctl refused to start SLAM.")

    battery_detail = (f"battery {volts:.2f} V" if volts is not None
                      else "battery check not configured for this machine")
    return _result(
        "succeeded", "mapping_started",
        detail=f"SLAM recording ({battery_detail}). Drive the robot over the whole space, "
               f"revisiting where it has already been, then save.",
        evidence=[{
            "kind": "mapping.session",
            "usable": True,
            "detail": f"SLAM recording, {battery_detail}",
        }],
    )


def _promote(staged: Path, target: Path) -> bool:
    """Move a staged map onto its published name, fixing the yaml's own pointer.

    The saver is written to a staging basename and renamed into place because
    ``flyto-nav2.service`` gates on ``ConditionPathExists`` for the published
    ``.yaml``. Writing there directly means a partial or late write is the thing
    that flips Nav2 from "will not start" to "starts on a map no job certified".

    ``map_saver_cli`` writes ``image: <staged>.pgm`` into its yaml, so the
    pointer has to be rewritten or the renamed pair points at a file that is no
    longer there.
    """
    staged_yaml, staged_pgm = staged.with_suffix(".yaml"), staged.with_suffix(".pgm")
    target_yaml, target_pgm = target.with_suffix(".yaml"), target.with_suffix(".pgm")
    try:
        text = staged_yaml.read_text(encoding="utf-8")
        fixed = "\n".join(
            f"image: {target_pgm.name}" if line.startswith("image:") else line
            for line in text.splitlines()
        ) + "\n"
        staged_pgm.replace(target_pgm)
        target_yaml.write_text(fixed, encoding="utf-8")
        staged_yaml.unlink(missing_ok=True)
    except (OSError, UnicodeError):
        return False
    return True


def _map_shape(yaml_path: Path, image_path: Path) -> str:
    """Resolution and cell count, read back from what was actually written.

    ``map_saver_cli`` exits zero having written a map of nothing, and a one-cell
    map reaching Nav2 as a working venue is the failure worth catching here.
    """
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
    return f"{cells or 'unknown'} cells at {resolution or 'unknown'} m"


def save(params: dict, settings: MappingSettings) -> dict:
    try:
        name = _map_name(params)
    except ValueError:
        return _result("refused", "map_name_invalid")

    slam_active = _unit_active(settings.slam_unit)
    if slam_active is None:
        return _result("failed", "unit_state_unknown",
                       detail="The configured SLAM unit could not be inspected.")
    if not slam_active:
        return _result("refused", "mapping_not_running",
                       detail="Nothing is recording, so there is no map to save.")

    try:
        map_dir = Path(settings.map_dir)
        map_dir.mkdir(parents=True, exist_ok=True)
    except OSError:
        return _result("failed", "map_directory_unwritable")

    target = map_dir / name
    staged = map_dir / f".staging-{name}"
    code, _ = _ros(
        settings,
        f"timeout {SAVER_SHELL_TIMEOUT} ros2 run nav2_map_server map_saver_cli "
        f"-f {shlex.quote(str(staged))} "
        f"--ros-args -p save_map_timeout:={SAVER_MAP_TIMEOUT}",
        SAVE_TIMEOUT,
    )
    staged_yaml, staged_pgm = staged.with_suffix(".yaml"), staged.with_suffix(".pgm")
    if code != 0 or not staged_yaml.is_file() or not staged_pgm.is_file():
        staged_yaml.unlink(missing_ok=True)
        staged_pgm.unlink(missing_ok=True)
        return _result("failed", "map_save_failed",
                       detail="map_saver_cli did not produce both a .yaml and a .pgm.")

    if not _promote(staged, target):
        return _result("failed", "map_publish_failed",
                       detail="The map was recorded but could not be renamed into place.")

    shape = _map_shape(target.with_suffix(".yaml"), target.with_suffix(".pgm"))
    evidence = [{"kind": "map.recorded", "usable": True, "detail": f"{name}: {shape}"}]

    # SLAM has done its job; leaving it holding /map is what stops Nav2 from
    # being startable next, and Conflicts= would then stop the map from ever
    # being used by the thing it was recorded for.
    #
    # Reported as evidence rather than as a reason_code, because the job runner
    # reads only `status` and `evidence` from this result and substitutes its
    # own detail -- a reason_code saying SLAM is still up would reach nobody.
    stopped = _stop_slam(settings)
    if stopped != 0:
        evidence.append({
            "kind": "mapping.session",
            "usable": False,
            "detail": "The map was saved but slam_toolbox is still running; "
                      "flyto-nav2 will refuse to start until it is stopped.",
        })

    return _result("succeeded", "map_recorded", detail=f"{name}: {shape}", evidence=evidence)


def abort(params: dict, settings: MappingSettings) -> dict:
    slam_active = _unit_active(settings.slam_unit)
    if slam_active is None:
        return _result("failed", "unit_state_unknown",
                       detail="The configured SLAM unit could not be inspected.")
    if not slam_active:
        return _result("succeeded", "mapping_not_running",
                       detail="Nothing was recording.")
    stopped = _stop_slam(settings)
    if stopped is None:
        return _result("failed", "slam_stop_failed",
                       detail="systemctl could not be run.")
    if stopped != 0:
        return _result("failed", "slam_stop_failed",
                       detail="systemctl did not stop SLAM; it may have been cancelled "
                              "by a conflicting unit.")
    return _result("succeeded", "mapping_aborted", detail="Recording discarded.")


HANDLERS = {"mapping.start": start, "mapping.save": save, "mapping.abort": abort}

# A marker the prepared payload carries so `execute` can tell a payload this
# executor minted from arbitrary JSON that reached it another way.
PREPARED_MARKER = "flyto.mapping.prepared.v1"


def _emit(payload: dict) -> int:
    """Write exactly one JSON value and nothing after it.

    `_call` does `raw_decode` and then refuses anything with trailing bytes, so
    a newline here rejects the whole response as `stdio_output_invalid` before
    it is ever looked at. `print` was what broke every call this file made.
    """
    sys.stdout.write(json.dumps(payload, separators=(",", ":")))
    sys.stdout.flush()
    return 0


def _prepare(envelope: dict) -> int:
    request = envelope.get("request")
    if not isinstance(request, dict):
        return _emit({"marker": PREPARED_MARKER, "module_id": "", "params": {}})
    module_id = request.get("module_id")
    params = request.get("params")
    return _emit({
        "marker": PREPARED_MARKER,
        "module_id": module_id if isinstance(module_id, str) else "",
        "params": params if isinstance(params, dict) else {},
    })


def _execute(envelope: dict, config_path: Path) -> int:
    prepared = envelope.get("prepared")
    if not isinstance(prepared, dict) or prepared.get("marker") != PREPARED_MARKER:
        return _emit(_result("failed", "prepared_payload_invalid"))
    handler = HANDLERS.get(prepared.get("module_id"))
    if handler is None:
        return _emit(_result("refused", "module_not_supported"))
    try:
        settings = MappingSettings.from_file(config_path)
    except MappingSettingsError as exc:
        return _emit(_result("failed", "mapping_settings_invalid", detail=str(exc)))
    params = prepared.get("params")
    if not isinstance(params, dict):
        params = {}
    try:
        return _emit(handler(params, settings))
    except Exception:  # noqa: BLE001 - a crash must still be a valid result
        # The registry reads stdout, not exit codes, and an executor that dies
        # without answering is indistinguishable from one that hung.
        return _emit(_result("failed", "executor_error"))


def _arguments(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Flyto mapping device executor")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = _arguments([] if argv is None else argv)
    try:
        envelope = json.loads(sys.stdin.read(65_536) or "{}")
    except (ValueError, OSError):
        return _emit(_result("failed", "request_unreadable"))
    if not isinstance(envelope, dict) or envelope.get("contract_version") != CONTRACT_VERSION:
        return _emit(_result("failed", "contract_version_unsupported"))

    operation = envelope.get("operation")
    if operation == "prepare":
        return _prepare(envelope)
    if operation == "execute":
        return _execute(envelope, args.config)
    return _emit(_result("failed", "operation_unsupported"))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
