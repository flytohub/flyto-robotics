"""The mapping executor, tested against the protocol rather than against an idea of it.

This file exists because the first version of the executor shipped 100% broken
and every hand-check passed. The checks fed it the envelope its author imagined
and read back whatever it wrote; the registry sends a different envelope and
refuses a response with a trailing byte. Both mistakes were invisible to any
test that did not model the caller.

So the first three tests here are wire-protocol tests, and they are the point of
the file. The behaviour tests below them are worth having, but they are not what
was missing.
"""

from __future__ import annotations

import json
import math
import subprocess

import pytest

from deploy.device_executor_contract import CONTRACT_VERSION, validate_result
from deploy.executors import flyto_mapping_executor as executor

# ---------------------------------------------------------------------------
# Wire protocol. These are the tests that would have caught the shipped bug.
# ---------------------------------------------------------------------------


def _call(operation: str, payload_name: str, payload, capsys) -> str:
    """Send exactly what `_StdioOwner._call` sends, and return raw stdout."""
    envelope = {"contract_version": CONTRACT_VERSION, "operation": operation,
                payload_name: payload}
    monkey_stdin = json.dumps(envelope, ensure_ascii=False, allow_nan=False,
                              separators=(",", ":"))
    import io
    import sys
    saved, sys.stdin = sys.stdin, io.StringIO(monkey_stdin)
    try:
        executor.main()
    finally:
        sys.stdin = saved
    return capsys.readouterr().out


def _decode_as_registry_does(text: str):
    """Reject exactly what the registry rejects.

    `_call` does `raw_decode` and then refuses any trailing byte. `print()`
    appends a newline, so this assertion is the whole bug.
    """
    value, end = json.JSONDecoder().raw_decode(text)
    assert end == len(text), (
        f"{len(text) - end} trailing byte(s) after the JSON value; the registry "
        f"raises stdio_output_invalid for this"
    )
    assert isinstance(value, dict)
    return value


def test_a_response_carries_no_trailing_byte(capsys):
    out = _call("execute", "prepared",
                {"marker": executor.PREPARED_MARKER, "module_id": "nope", "params": {}},
                capsys)
    _decode_as_registry_does(out)


def test_prepare_reads_module_id_from_inside_the_request(capsys):
    """`module_id` is nested under `request`, never at the top level.

    The first version read it off the top, so every prepared payload carried an
    empty module id and every execute refused as module_not_supported.
    """
    out = _call("prepare", "request",
                {"contract_version": CONTRACT_VERSION,
                 "module_id": "mapping.abort", "params": {"map_name": "lab"}},
                capsys)
    prepared = _decode_as_registry_does(out)
    assert prepared["module_id"] == "mapping.abort"
    assert prepared["params"] == {"map_name": "lab"}
    assert prepared["marker"] == executor.PREPARED_MARKER


def test_a_prepared_payload_survives_the_contract_validator(capsys):
    from deploy.device_executor_contract import validate_prepared_payload

    out = _call("prepare", "request",
                {"contract_version": CONTRACT_VERSION,
                 "module_id": "mapping.save", "params": {}},
                capsys)
    validate_prepared_payload(_decode_as_registry_does(out))


def test_execute_refuses_a_payload_this_executor_did_not_mint(capsys):
    out = _call("execute", "prepared", {"module_id": "mapping.abort"}, capsys)
    result = validate_result(_decode_as_registry_does(out))
    assert result["status"] == "failed"
    assert result["reason_code"] == "prepared_payload_invalid"


def test_an_unknown_operation_is_a_result_and_not_a_crash(capsys):
    out = _call("sabotage", "request", {}, capsys)
    result = validate_result(_decode_as_registry_does(out))
    assert result["reason_code"] == "operation_unsupported"


def test_a_wrong_contract_version_is_refused(capsys):
    import io
    import sys
    saved, sys.stdin = sys.stdin, io.StringIO(json.dumps({"contract_version": "v99"}))
    try:
        executor.main()
    finally:
        sys.stdin = saved
    result = validate_result(_decode_as_registry_does(capsys.readouterr().out))
    assert result["reason_code"] == "contract_version_unsupported"


# ---------------------------------------------------------------------------
# Every result the handlers can produce must satisfy the contract.
# ---------------------------------------------------------------------------


def test_no_result_carries_evidence_unless_it_succeeded():
    """A step that did not happen must not hand back proof that it did."""
    refused = executor._result("refused", "battery_too_low",
                               evidence=[{"kind": "map.recorded", "usable": True}])
    assert refused["evidence"] == []
    validate_result(refused)


@pytest.mark.parametrize("status", ["succeeded", "failed", "refused"])
def test_every_status_produces_a_contract_valid_result(status):
    validate_result(executor._result(status, "some_reason", detail="x" * 4000))


# ---------------------------------------------------------------------------
# Battery. `None` means unreadable *or* unbelievable.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "raw",
    ["nan", "NaN", "inf", "-inf", "Infinity", "0.0", "0", "-3.2"],
)
def test_an_unbelievable_voltage_is_not_a_voltage(monkeypatch, raw):
    """NaN slips past every comparison, and zero means "nothing reported".

    turtlebot3_node fills unmeasured fields with 0.0 rather than NaN, so a zero
    is far more likely to be an absent reading than a flat pack — and answering
    battery_too_low to it sends someone to charge a battery that is fine.
    """
    monkeypatch.setattr(executor, "_ros", lambda *a, **k: (0, raw))
    assert executor._battery_volts() is None


@pytest.mark.parametrize("raw,expected", [("11.45", 11.45), ("12.6", 12.6)])
def test_a_believable_voltage_is_returned(monkeypatch, raw, expected):
    monkeypatch.setattr(executor, "_ros", lambda *a, **k: (0, raw))
    assert math.isclose(executor._battery_volts(), expected)


def test_an_unreadable_battery_refuses_rather_than_guessing(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active",
                        lambda unit: unit == executor.BRINGUP_UNIT)
    monkeypatch.setattr(executor, "_battery_volts", lambda: None)
    result = executor.start({})
    assert (result["status"], result["reason_code"]) == ("refused", "battery_unknown")
    validate_result(result)


def test_a_low_pack_refuses_before_slam_is_touched(monkeypatch):
    started = []
    monkeypatch.setattr(executor, "_unit_active",
                        lambda unit: unit == executor.BRINGUP_UNIT)
    monkeypatch.setattr(executor, "_battery_volts", lambda: 11.45)
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: started.append(a))
    result = executor.start({})
    assert (result["status"], result["reason_code"]) == ("refused", "battery_too_low")
    assert not started, "SLAM must not be started by a refused run"


# ---------------------------------------------------------------------------
# Map names reach a filesystem path and a shell command line.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize("name", ["lab", "ward-3b", "venue_2", "a", "A1", "x" * 64])
def test_a_plain_ascii_name_is_accepted(name):
    assert executor._map_name({"map_name": name}) == name


@pytest.mark.parametrize(
    "name",
    [
        "../../etc/passwd",   # traversal
        "lab/../..",          # traversal
        "lab; rm -rf /",      # shell metacharacters
        "lab name",           # space
        "ｌａｂ",              # fullwidth -- reads as "lab" in a listing
        "café",               # non-ASCII letter
        "лаб",                # non-ASCII letter
        "１２３",              # fullwidth digits
        "½",                  # Unicode numeric
        "-lab",               # leading dash
        "_lab",               # leading underscore
        "",                   # empty
        "x" * 65,             # too long
        "lab\n",              # trailing newline
        "$(whoami)",          # substitution
    ],
)
def test_anything_that_is_not_a_plain_ascii_name_is_refused(name):
    with pytest.raises(ValueError):
        executor._map_name({"map_name": name})


@pytest.mark.parametrize("value", [None, 42, ["lab"], {"a": 1}])
def test_a_non_string_name_is_refused(value):
    with pytest.raises(ValueError):
        executor._map_name({"map_name": value})


def test_save_refuses_a_bad_name_before_looking_at_slam(monkeypatch):
    looked = []
    monkeypatch.setattr(executor, "_unit_active", lambda u: looked.append(u) or True)
    result = executor.save({"map_name": "../escape"})
    assert (result["status"], result["reason_code"]) == ("refused", "map_name_invalid")
    assert not looked


# ---------------------------------------------------------------------------
# Stopping SLAM can genuinely fail, and used to be reported as success.
# ---------------------------------------------------------------------------


def _fake_run(returncode):
    def run(*args, **kwargs):
        return subprocess.CompletedProcess(args=args, returncode=returncode)
    return run


def test_abort_reports_a_stop_that_did_not_happen(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)
    monkeypatch.setattr(subprocess, "run", _fake_run(5))
    result = executor.abort({})
    assert (result["status"], result["reason_code"]) == ("failed", "slam_stop_failed")
    validate_result(result)


def test_abort_reports_a_stop_that_could_not_be_run(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)

    def boom(*a, **k):
        raise OSError("no systemctl")

    monkeypatch.setattr(subprocess, "run", boom)
    result = executor.abort({})
    assert (result["status"], result["reason_code"]) == ("failed", "slam_stop_failed")


def test_abort_is_idempotent_when_nothing_is_recording(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active", lambda unit: False)
    result = executor.abort({})
    assert (result["status"], result["reason_code"]) == ("succeeded", "mapping_not_running")
    assert result["evidence"] == []


def test_a_leftover_slam_is_carried_as_evidence_not_a_reason_code(monkeypatch, tmp_path):
    """The job runner reads only `status` and `evidence` and substitutes its own
    detail, so a reason_code saying SLAM is still up would reach nobody."""
    monkeypatch.setattr(executor, "MAP_DIR", tmp_path)
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)

    def fake_ros(command, timeout):
        staged = tmp_path / ".staging-lab"
        staged.with_suffix(".yaml").write_text(
            "image: .staging-lab.pgm\nresolution: 0.05\n", encoding="utf-8")
        staged.with_suffix(".pgm").write_bytes(b"P5\n# made\n40 30\n255\n")
        return 0, ""

    monkeypatch.setattr(executor, "_ros", fake_ros)
    monkeypatch.setattr(executor, "_stop_slam", lambda: 1)

    result = executor.save({"map_name": "lab"})
    validate_result(result)
    assert result["status"] == "succeeded"
    kinds = {(e["kind"], e["usable"]) for e in result["evidence"]}
    assert ("map.recorded", True) in kinds
    assert ("mapping.session", False) in kinds


# ---------------------------------------------------------------------------
# Publishing the map. flyto-nav2 gates on the published .yaml existing.
# ---------------------------------------------------------------------------


def test_a_failed_save_leaves_the_published_name_untouched(monkeypatch, tmp_path):
    """The published name is what flyto-nav2's ConditionPathExists watches.

    A partial or late write there is exactly what flips Nav2 from "will not
    start" to "starts on a map no job certified", so the saver writes to a
    staging name and only a complete run is renamed into place.
    """
    monkeypatch.setattr(executor, "MAP_DIR", tmp_path)
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)
    monkeypatch.setattr(executor, "_ros", lambda command, timeout: (124, ""))

    result = executor.save({"map_name": "lab"})
    assert (result["status"], result["reason_code"]) == ("failed", "map_save_failed")
    assert not (tmp_path / "lab.yaml").exists()
    assert not (tmp_path / "lab.pgm").exists()


def test_a_partial_save_is_cleaned_up_and_not_published(monkeypatch, tmp_path):
    monkeypatch.setattr(executor, "MAP_DIR", tmp_path)
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)

    def half(command, timeout):
        (tmp_path / ".staging-lab.yaml").write_text("image: x\n", encoding="utf-8")
        return 0, ""

    monkeypatch.setattr(executor, "_ros", half)
    result = executor.save({"map_name": "lab"})
    assert result["reason_code"] == "map_save_failed"
    assert not (tmp_path / ".staging-lab.yaml").exists()
    assert not (tmp_path / "lab.yaml").exists()


def test_promotion_rewrites_the_image_pointer(tmp_path):
    """map_saver_cli writes `image: <staged>.pgm`; renaming without fixing that
    leaves the pair pointing at a file that is no longer there."""
    staged, target = tmp_path / ".staging-lab", tmp_path / "lab"
    staged.with_suffix(".yaml").write_text(
        "image: .staging-lab.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n", encoding="utf-8")
    staged.with_suffix(".pgm").write_bytes(b"P5\n40 30\n255\n")

    assert executor._promote(staged, target)
    published = target.with_suffix(".yaml").read_text(encoding="utf-8")
    assert "image: lab.pgm" in published
    assert "resolution: 0.05" in published
    assert target.with_suffix(".pgm").is_file()
    assert not staged.with_suffix(".yaml").exists()


def test_the_saver_is_bounded_inside_the_shell_not_only_around_it(monkeypatch, tmp_path):
    """`bash -lc` execs into `ros2`, which spawns map_saver_cli as its own child.

    Killing the process this started orphans the saver to PID 1, still holding
    whatever deadline it was given — and its late write lands on the published
    name. The bound has to be inside the shell.
    """
    seen = {}
    monkeypatch.setattr(executor, "MAP_DIR", tmp_path)
    monkeypatch.setattr(executor, "_unit_active", lambda unit: True)

    def capture(command, timeout):
        seen["command"] = command
        return 124, ""

    monkeypatch.setattr(executor, "_ros", capture)
    executor.save({"map_name": "lab"})
    assert seen["command"].startswith(f"timeout {executor.SAVER_SHELL_TIMEOUT} ")
    assert f"save_map_timeout:={executor.SAVER_MAP_TIMEOUT}" in seen["command"]
    assert executor.SAVER_MAP_TIMEOUT < executor.SAVER_SHELL_TIMEOUT < executor.SAVE_TIMEOUT


def test_the_map_shape_is_read_back_from_what_was_written(tmp_path):
    """map_saver_cli exits zero having written a map of nothing."""
    yaml_path, pgm_path = tmp_path / "lab.yaml", tmp_path / "lab.pgm"
    yaml_path.write_text("image: lab.pgm\nresolution: 0.05\n", encoding="utf-8")
    pgm_path.write_bytes(b"P5\n# recorded\n812 604\n255\n")
    assert executor._map_shape(yaml_path, pgm_path) == "812 604 cells at 0.05 m"


# ---------------------------------------------------------------------------
# Ordering of the start preconditions.
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "active,expected",
    [
        ({executor.SLAM_UNIT}, "mapping_already_running"),
        ({executor.NAV2_UNIT}, "navigation_running"),
        (set(), "sensors_unavailable"),
    ],
)
def test_start_refuses_for_the_nearest_reason(monkeypatch, active, expected):
    monkeypatch.setattr(executor, "_unit_active", lambda unit: unit in active)
    monkeypatch.setattr(executor, "_battery_volts", lambda: 12.4)
    result = executor.start({})
    assert (result["status"], result["reason_code"]) == ("refused", expected)
    validate_result(result)


def test_a_healthy_start_carries_the_voltage_as_evidence(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active",
                        lambda unit: unit == executor.BRINGUP_UNIT)
    monkeypatch.setattr(executor, "_battery_volts", lambda: 12.4)
    monkeypatch.setattr(subprocess, "run", _fake_run(0))
    result = executor.start({})
    validate_result(result)
    assert (result["status"], result["reason_code"]) == ("succeeded", "mapping_started")
    assert result["evidence"][0]["kind"] == "mapping.session"
    assert "12.40 V" in result["evidence"][0]["detail"]


def test_a_start_that_systemctl_refuses_is_failed_not_succeeded(monkeypatch):
    monkeypatch.setattr(executor, "_unit_active",
                        lambda unit: unit == executor.BRINGUP_UNIT)
    monkeypatch.setattr(executor, "_battery_volts", lambda: 12.4)
    monkeypatch.setattr(subprocess, "run", _fake_run(1))
    result = executor.start({})
    assert (result["status"], result["reason_code"]) == ("failed", "slam_start_failed")


def test_the_module_ids_the_manifest_declares_are_the_ones_handled():
    """A manifest that advertises a module the executor does not handle sends
    every job for it to a refusal the operator cannot act on."""
    import pathlib

    manifest = json.loads(
        (pathlib.Path(__file__).resolve().parents[1]
         / "deploy/executors/flyto-mapping.json").read_text(encoding="utf-8")
    )
    assert set(manifest["module_ids"]) == set(executor.HANDLERS)
