from __future__ import annotations

import copy
import inspect
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from deploy.device_executor_contract import CONTRACT_VERSION
from deploy.device_executor_registry import (
    ENTRY_POINT_GROUP,
    MAX_PREPARED_HANDLES,
    DeviceExecutorRegistry,
    PreparedHandle,
    RegistryError,
)


def python_manifest(module_ids=None, target="sample.provider:executor"):
    return {
        "contract_version": CONTRACT_VERSION,
        "provider": "sample-provider",
        "module_ids": module_ids or ["sample.action"],
        "transport": "python_entry_point",
        "entry_point": target,
    }


def stdio_manifest(module_ids=None, command=None, timeout=3):
    return {
        "contract_version": CONTRACT_VERSION,
        "provider": "local-provider",
        "module_ids": module_ids or ["local.action"],
        "transport": "json_stdio",
        "command": command or ["/usr/bin/local-executor", "--fixed"],
        "timeout_seconds": timeout,
    }


def result(reason="done"):
    return {
        "contract_version": CONTRACT_VERSION,
        "status": "succeeded",
        "reason_code": reason,
        "evidence": [],
    }


class Provider:
    def __init__(self, module_ids=None, target="sample.provider:executor"):
        self._manifest = python_manifest(module_ids, target)
        self.prepared_requests = []
        self.executed = []

    def manifest(self):
        return self._manifest

    def prepare(self, request):
        self.prepared_requests.append(request)
        return {"copied": request["params"]}

    def execute(self, prepared):
        self.executed.append(prepared)
        return result()


class EntryPoint:
    group = ENTRY_POINT_GROUP

    def __init__(self, provider, value="sample.provider:executor", name="untrusted"):
        self.provider = provider
        self.value = value
        self.name = name

    def load(self):
        return self.provider


class SelectableEntryPoints:
    def __init__(self, entries):
        self.entries = entries
        self.selected_groups = []

    def select(self, *, group):
        self.selected_groups.append(group)
        return [entry for entry in self.entries if entry.group == group]


def write_manifest(directory: Path, name="executor.json", **changes):
    value = stdio_manifest()
    value.update(changes)
    (directory / name).write_text(json.dumps(value), encoding="utf-8")


def assert_code(code, call):
    with pytest.raises(RegistryError) as caught:
        call()
    assert caught.value.reason_code == code
    assert str(caught.value) == code


def test_missing_sources_are_empty(tmp_path):
    registry = DeviceExecutorRegistry(tmp_path / "missing", entry_points=[])
    assert dict(registry.module_owners) == {}
    assert_code("module_not_found", lambda: registry.prepare("absent.action", {}))


def test_entry_point_group_is_canonical_and_not_legacy():
    assert ENTRY_POINT_GROUP == "flyto.device_executors"
    assert ENTRY_POINT_GROUP != "flyto.device-executors"


@pytest.mark.parametrize("source_kind", ["select", "dict", "list"])
def test_canonical_entry_point_discovery_shapes_load_arbitrary_modules(
        tmp_path, source_kind):
    provider = Provider([f"arbitrary.{source_kind}"])
    entry = EntryPoint(provider)
    if source_kind == "select":
        source = SelectableEntryPoints([entry])
    elif source_kind == "dict":
        source = {ENTRY_POINT_GROUP: [entry]}
    else:
        source = [entry]

    registry = DeviceExecutorRegistry(tmp_path, entry_points=source)

    assert list(registry.module_owners) == [f"arbitrary.{source_kind}"]
    if source_kind == "select":
        assert source.selected_groups == [ENTRY_POINT_GROUP]


def test_python_discovery_is_deterministic_and_ignores_entrypoint_name(tmp_path):
    first = Provider(["z.action"])
    first._manifest["entry_point"] = "z.provider:executor"
    second = Provider(["a.action"])
    second._manifest["entry_point"] = "a.provider:executor"
    entries = [EntryPoint(first, "z.provider:executor", "a.action"),
               EntryPoint(second, "a.provider:executor", "z.action")]
    registry = DeviceExecutorRegistry(tmp_path, entry_points=entries)
    assert list(registry.module_owners) == ["a.action", "z.action"]


def test_entry_point_target_must_equal_manifest_and_errors_are_private(tmp_path):
    secret = "do-not-echo-this-target"
    provider = Provider(target="actual.provider:executor")
    assert_code("entry_point_mismatch", lambda: DeviceExecutorRegistry(
        tmp_path, entry_points=[EntryPoint(provider, f"{secret}:executor")]))
    try:
        DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(provider, f"{secret}:executor")])
    except RegistryError as error:
        assert secret not in str(error)


@pytest.mark.parametrize("provider", [object(), SimpleNamespace(manifest=lambda: {}),
                                       SimpleNamespace(manifest=lambda: python_manifest(),
                                                       prepare=lambda request: request)])
def test_partial_or_malformed_python_provider_fails_atomically(tmp_path, provider):
    assert_code("provider_invalid", lambda: DeviceExecutorRegistry(
        tmp_path, entry_points=[EntryPoint(Provider(["good.action"])),
                                EntryPoint(provider, name="later")]))


def test_duplicate_module_ids_across_sources_fail_atomically(tmp_path):
    write_manifest(tmp_path, module_ids=["same.action"])
    assert_code("module_id_duplicate", lambda: DeviceExecutorRegistry(
        tmp_path, entry_points=[EntryPoint(Provider(["same.action"]))]))


def test_local_discovery_is_lexical_nonrecursive_and_json_only(tmp_path):
    write_manifest(tmp_path, "b.json", module_ids=["b.action"])
    write_manifest(tmp_path, "a.json", module_ids=["a.action"])
    (tmp_path / "ignored.txt").write_text("not json", encoding="utf-8")
    nested = tmp_path / "nested"
    nested.mkdir()
    write_manifest(nested, module_ids=["nested.action"])
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[])
    assert list(registry.module_owners) == ["a.action", "b.action"]


def test_local_discovery_rejects_symlink_nonregular_and_invalid_utf8(tmp_path):
    target = tmp_path / "target"
    target.write_text(json.dumps(stdio_manifest()), encoding="utf-8")
    (tmp_path / "linked.json").symlink_to(target)
    assert_code("manifest_file_invalid", lambda: DeviceExecutorRegistry(tmp_path, entry_points=[]))
    (tmp_path / "linked.json").unlink()
    (tmp_path / "pipe.json").mkdir()
    assert_code("manifest_file_invalid", lambda: DeviceExecutorRegistry(tmp_path, entry_points=[]))
    (tmp_path / "pipe.json").rmdir()
    (tmp_path / "bad.json").write_bytes(b"\xff")
    assert_code("manifest_file_invalid", lambda: DeviceExecutorRegistry(tmp_path, entry_points=[]))


def test_local_discovery_rejects_symlinked_directory(tmp_path):
    real = tmp_path / "real"
    real.mkdir()
    write_manifest(real)
    linked = tmp_path / "linked"
    linked.symlink_to(real, target_is_directory=True)
    assert_code("manifest_directory_invalid",
                lambda: DeviceExecutorRegistry(linked, entry_points=[]))


def test_prepare_detaches_request_and_routes_only_selected_owner(tmp_path):
    selected = Provider(["selected.action"])
    other = Provider(["other.action"])
    registry = DeviceExecutorRegistry(
        tmp_path,
        entry_points=[EntryPoint(selected), EntryPoint(other, name="other")],
    )
    params = {"items": [1]}
    handle = registry.prepare("selected.action", params)
    params["items"].append(2)
    assert selected.prepared_requests[0]["params"] == {"items": [1]}
    assert other.prepared_requests == []
    registry.execute(handle)
    assert selected.executed == [{"copied": {"items": [1]}}]


def test_provider_failures_and_bad_payloads_have_fixed_errors(tmp_path):
    provider = Provider()
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(provider)])
    provider.prepare = lambda request: object()
    assert_code("prepare_failed", lambda: registry.prepare("sample.action", {}))
    provider.prepare = lambda request: {}
    handle = registry.prepare("sample.action", {})
    provider.execute = lambda prepared: {"secret": "must-not-escape"}
    assert_code("execute_failed", lambda: registry.execute(handle))


def test_handle_forgery_cross_registry_and_reuse_are_refused(tmp_path):
    provider = Provider()
    one = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(provider)])
    two = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(Provider())])
    handle = one.prepare("sample.action", {})
    assert_code("handle_invalid", lambda: two.execute(handle))
    with pytest.raises(TypeError):
        PreparedHandle()
    cloned = copy.copy(handle)
    assert_code("handle_invalid", lambda: one.execute(cloned))
    assert one.execute(handle) == result()
    assert_code("handle_invalid", lambda: one.execute(handle))


def child_command(source):
    return [sys.executable, "-c", source]


class CountingPipe:
    def __init__(self, pipe):
        self._pipe = pipe
        self.close_calls = 0

    def close(self):
        self.close_calls += 1
        return self._pipe.close()

    def __getattr__(self, name):
        return getattr(self._pipe, name)


class InstrumentedProcess:
    def __init__(self, argv, **kwargs):
        self._process = subprocess.Popen(argv, **kwargs)
        self.stdin = CountingPipe(self._process.stdin)
        self.stdout = CountingPipe(self._process.stdout)

    def __getattr__(self, name):
        return getattr(self._process, name)


def instrumented_factory(processes):
    def factory(argv, **kwargs):
        process = InstrumentedProcess(argv, **kwargs)
        processes.append(process)
        return process
    return factory


def assert_process_resources_closed(process):
    assert process.stdin.close_calls == 1
    assert process.stdout.close_calls == 1
    assert process.poll() is not None


def test_public_metadata_is_immutable_and_non_executable(tmp_path):
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(Provider())])
    metadata = registry.module_owners["sample.action"]
    assert metadata.provider == "sample-provider"
    assert metadata.transport == "python_entry_point"
    assert not callable(metadata)
    for forbidden in ("prepare", "execute", "command"):
        assert not hasattr(metadata, forbidden)
    with pytest.raises((AttributeError, TypeError)):
        metadata.provider = "changed"


def test_prepared_cap_discard_reuse_and_failed_execution_consumes(tmp_path):
    provider = Provider()
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(provider)])
    handles = [registry.prepare("sample.action", {}) for _ in range(MAX_PREPARED_HANDLES)]
    assert_code("prepared_limit_exceeded",
                lambda: registry.prepare("sample.action", {}))
    registry.discard(handles.pop())
    registry.discard(handles[-1])
    replacement = registry.prepare("sample.action", {})
    provider.execute = lambda prepared: (_ for _ in ()).throw(RuntimeError("secret"))
    assert_code("execute_failed", lambda: registry.execute(replacement))
    assert_code("handle_invalid", lambda: registry.execute(replacement))


def test_discard_rejects_foreign_but_is_idempotent_for_authentic(tmp_path):
    one = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(Provider())])
    two = DeviceExecutorRegistry(tmp_path, entry_points=[EntryPoint(Provider())])
    handle = one.prepare("sample.action", {})
    assert_code("handle_invalid", lambda: two.discard(handle))
    one.cancel(handle)
    one.discard(handle)


def test_stdio_uses_closed_process_boundary_and_no_job_data_in_argv(tmp_path):
    source = (
        "import json,sys; x=json.load(sys.stdin); sys.stdout.write(json.dumps("
        "{'ticket': 7} if x['operation']=='prepare' else "
        + repr(result("stdio-done")) + "))"
    )
    write_manifest(tmp_path, command=child_command(source))
    calls = []
    processes = []
    def factory(argv, **kwargs):
        calls.append((argv, kwargs))
        process = InstrumentedProcess(argv, **kwargs)
        processes.append(process)
        return process
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=factory)
    handle = registry.prepare("local.action", {"private": "only-stdin"})
    argv, options = calls[0]
    assert argv == child_command(source)
    assert "private" not in " ".join(argv)
    assert options == {
        "stdin": subprocess.PIPE, "stdout": subprocess.PIPE,
        "stderr": subprocess.DEVNULL, "shell": False,
        "cwd": "/", "env": {}, "close_fds": True,
    }
    assert registry.execute(handle)["reason_code"] == "stdio-done"
    assert all(process.stdin.close_calls == process.stdout.close_calls == 1
               for process in processes)
    assert all(process.poll() is not None for process in processes)


@pytest.mark.parametrize(
    ("source", "code"),
    [
        ("pass", "stdio_output_invalid"),
        ("print('{} {}')", "stdio_output_invalid"),
        ("print('[]')", "stdio_output_invalid"),
        ("print('{')", "stdio_output_invalid"),
        ("import sys; print('{}'); sys.exit(2)", "stdio_nonzero"),
    ],
)
def test_stdio_failures_are_stable(tmp_path, source, code):
    write_manifest(tmp_path, command=child_command(source))
    processes = []
    registry = DeviceExecutorRegistry(
        tmp_path, entry_points=[], run=instrumented_factory(processes))
    assert_code(code, lambda: registry.prepare("local.action", {}))
    assert_process_resources_closed(processes[0])


def test_stdio_nonzero_child_is_reaped(tmp_path):
    write_manifest(tmp_path, command=child_command("raise SystemExit(9)"))
    processes = []
    def factory(argv, **kwargs):
        process = InstrumentedProcess(argv, **kwargs)
        processes.append(process)
        return process
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=factory)
    assert_code("stdio_nonzero", lambda: registry.prepare("local.action", {}))
    assert processes[0].poll() == 9
    assert_process_resources_closed(processes[0])


def test_stdio_spawn_error_is_private(tmp_path):
    write_manifest(tmp_path)
    def fail(*args, **kwargs):
        raise OSError("private path")
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=fail)
    assert_code("stdio_os_error", lambda: registry.prepare("local.action", {}))


def test_stdio_timeout_kills_and_reaps(tmp_path):
    write_manifest(tmp_path, timeout_seconds=1,
                   command=child_command("import time; time.sleep(30)"))
    processes = []
    def factory(argv, **kwargs):
        process = InstrumentedProcess(argv, **kwargs)
        processes.append(process)
        return process
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=factory)
    assert_code("stdio_timeout", lambda: registry.prepare("local.action", {}))
    assert_process_resources_closed(processes[0])


def test_streaming_oversize_child_is_terminated_and_reaped(tmp_path):
    source = "import itertools,os;[os.write(1,b'x'*65536) for _ in itertools.count()]"
    write_manifest(tmp_path, command=child_command(source))
    processes = []
    def factory(argv, **kwargs):
        process = InstrumentedProcess(argv, **kwargs)
        processes.append(process)
        return process
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=factory)
    started = time.monotonic()
    assert_code("stdio_output_invalid", lambda: registry.prepare("local.action", {}))
    assert time.monotonic() - started < 2
    assert_process_resources_closed(processes[0])


def test_stdio_broken_pipe_closes_both_pipes_and_reaps(tmp_path):
    source = "import os; os.close(0); os.write(1, b'{}')"
    write_manifest(tmp_path, command=child_command(source))
    processes = []
    registry = DeviceExecutorRegistry(
        tmp_path, entry_points=[], run=instrumented_factory(processes))
    registry.prepare("local.action", {})
    assert_process_resources_closed(processes[0])


def test_stdio_os_error_after_spawn_closes_both_pipes_and_reaps(tmp_path, monkeypatch):
    write_manifest(tmp_path, command=child_command("print('{}')"))
    processes = []
    registry = DeviceExecutorRegistry(
        tmp_path, entry_points=[], run=instrumented_factory(processes))
    monkeypatch.setattr(os, "set_blocking",
                        lambda *_args: (_ for _ in ()).throw(OSError("private")))
    assert_code("stdio_os_error", lambda: registry.prepare("local.action", {}))
    assert_process_resources_closed(processes[0])


def test_repeated_successful_stdio_calls_do_not_accumulate_parent_resources(tmp_path):
    source = (
        "import json,sys; x=json.load(sys.stdin); sys.stdout.write(json.dumps("
        "{} if x['operation']=='prepare' else " + repr(result()) + "))"
    )
    write_manifest(tmp_path, command=child_command(source))
    processes = []
    registry = DeviceExecutorRegistry(
        tmp_path, entry_points=[], run=instrumented_factory(processes))
    fd_directory = Path("/proc/self/fd")
    before = len(list(fd_directory.iterdir())) if fd_directory.is_dir() else None
    for _ in range(20):
        registry.execute(registry.prepare("local.action", {}))
    after = len(list(fd_directory.iterdir())) if fd_directory.is_dir() else None
    assert len(processes) == 40
    assert all(process.stdin.close_calls == process.stdout.close_calls == 1
               for process in processes)
    assert all(process.poll() is not None for process in processes)
    if before is not None:
        assert after == before


def test_stdio_stderr_is_discarded_and_result_contract_is_enforced(tmp_path):
    source = (
        "import json,sys; x=json.load(sys.stdin); sys.stdout.write("
        "'{}' if x['operation']=='prepare' else json.dumps({'private':'bad'}))"
    )
    write_manifest(tmp_path, command=child_command(source))
    calls = []
    def factory(argv, **kwargs):
        calls.append(kwargs)
        return subprocess.Popen(argv, **kwargs)
    registry = DeviceExecutorRegistry(tmp_path, entry_points=[], run=factory)
    handle = registry.prepare("local.action", {})
    assert_code("execute_failed", lambda: registry.execute(handle))
    assert calls[-1]["stderr"] is subprocess.DEVNULL


def test_registry_has_no_domain_or_forbidden_runtime_dependencies():
    source = inspect.getsource(__import__(
        "deploy.device_executor_registry", fromlist=["DeviceExecutorRegistry"]
    ))
    lowered = source.lower()
    assert "flyto-core" not in lowered and "flyto_robotics" not in source
    assert "shell=true" not in lowered
    for domain in ("camera", "robot", "motion", "patient"):
        assert domain not in lowered
