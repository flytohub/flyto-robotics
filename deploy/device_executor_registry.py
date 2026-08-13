"""Bounded, domain-neutral discovery and execution for device executors."""

from __future__ import annotations

import importlib.metadata
import json
import os
import selectors
import stat
import subprocess
import time
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Callable

from .device_executor_contract import (
    CONTRACT_VERSION,
    MAX_JSON_BYTES,
    ContractValidationError,
    validate_executor_manifest,
    validate_prepared_payload,
    validate_request,
    validate_result,
)

ENTRY_POINT_GROUP = "flyto.device_executors"
MAX_MANIFEST_FILES = 256
MAX_MANIFEST_BYTES = 65_536
MAX_MANIFEST_TOTAL_BYTES = 1_048_576
MAX_STDIO_BYTES = MAX_JSON_BYTES
MAX_PREPARED_HANDLES = 32
_HANDLE_KEY = object()


class RegistryError(RuntimeError):
    """A content-free registry failure safe to expose to a caller."""

    def __init__(self, reason_code: str) -> None:
        self.reason_code = reason_code
        super().__init__(reason_code)


class PreparedHandle:
    """Opaque authority created only by a registry."""

    __slots__ = ("__registry",)

    def __init__(self, key: object = None, registry: object = None) -> None:
        if key is not _HANDLE_KEY:
            raise TypeError("PreparedHandle objects are registry-created")
        self.__registry = registry

    def _belongs_to(self, registry: object) -> bool:
        return self.__registry is registry


@dataclass(frozen=True, slots=True)
class ModuleMetadata:
    """Non-executable public description of a registered module."""

    provider: str
    transport: str


class _PythonOwner:
    __slots__ = ("provider",)

    def __init__(self, provider: Any) -> None:
        self.provider = provider

    def prepare(self, request: dict[str, Any]) -> Any:
        return self.provider.prepare(request)

    def execute(self, prepared: Any) -> Any:
        return self.provider.execute(prepared)


class _StdioOwner:
    __slots__ = ("manifest", "process_factory")

    def __init__(self, manifest: dict[str, Any], process_factory: Callable[..., Any]) -> None:
        self.manifest = manifest
        self.process_factory = process_factory

    @staticmethod
    def _terminate(process: Any) -> None:
        with suppress(OSError, ProcessLookupError):
            process.kill()
        with suppress(OSError, subprocess.SubprocessError):
            process.wait()

    def _run_bounded(self, encoded: bytes) -> tuple[int, bytes]:
        process = None
        reaped = False
        try:
            process = self.process_factory(
                list(self.manifest["command"]),
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                shell=False,
                cwd="/",
                env={},
                close_fds=True,
            )
            if process.stdin is None or process.stdout is None:
                raise OSError("missing pipe")
            os.set_blocking(process.stdin.fileno(), False)
            os.set_blocking(process.stdout.fileno(), False)
            deadline = time.monotonic() + self.manifest["timeout_seconds"]
            output = bytearray()
            sent = 0
            stdin_open = True
            stdout_open = True
            with selectors.DefaultSelector() as selector:
                selector.register(process.stdout, selectors.EVENT_READ, "stdout")
                selector.register(process.stdin, selectors.EVENT_WRITE, "stdin")
                while stdout_open or process.poll() is None:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise TimeoutError
                    events = selector.select(remaining)
                    if not events and process.poll() is None:
                        raise TimeoutError
                    for key, _ in events:
                        if key.data == "stdin":
                            try:
                                written = os.write(process.stdin.fileno(), encoded[sent:])
                            except BrokenPipeError:
                                written = 0
                                sent = len(encoded)
                            sent += written
                            if sent == len(encoded) and stdin_open:
                                selector.unregister(process.stdin)
                                stdin_open = False
                                process.stdin.close()
                        else:
                            chunk = os.read(process.stdout.fileno(),
                                            MAX_STDIO_BYTES + 1 - len(output))
                            if chunk:
                                output.extend(chunk)
                                if len(output) > MAX_STDIO_BYTES:
                                    raise OverflowError
                            else:
                                selector.unregister(process.stdout)
                                stdout_open = False
                    if process.poll() is not None and not stdout_open:
                        break
            returncode = process.wait(timeout=max(0, deadline - time.monotonic()))
            reaped = True
            return returncode, bytes(output)
        except TimeoutError:
            raise RegistryError("stdio_timeout") from None
        except OverflowError:
            raise RegistryError("stdio_output_invalid") from None
        except (OSError, ValueError, subprocess.SubprocessError):
            raise RegistryError("stdio_os_error") from None
        finally:
            if process is not None:
                for pipe in (process.stdin, process.stdout):
                    if pipe is not None and not pipe.closed:
                        with suppress(OSError, ValueError):
                            pipe.close()
                if not reaped:
                    self._terminate(process)

    def _call(self, operation: str, payload_name: str, payload: Any) -> Any:
        envelope = {"contract_version": CONTRACT_VERSION, "operation": operation,
                    payload_name: payload}
        try:
            encoded = json.dumps(envelope, ensure_ascii=False, allow_nan=False,
                                 separators=(",", ":")).encode("utf-8")
        except (TypeError, ValueError, UnicodeEncodeError):
            raise RegistryError("stdio_input_invalid") from None
        if len(encoded) > MAX_STDIO_BYTES:
            raise RegistryError("stdio_input_too_large")
        returncode, output = self._run_bounded(encoded)
        if returncode != 0:
            raise RegistryError("stdio_nonzero")
        if not output:
            raise RegistryError("stdio_output_invalid")
        try:
            text = output.decode("utf-8", errors="strict")
            value, end = json.JSONDecoder().raw_decode(text)
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise RegistryError("stdio_output_invalid") from None
        if end != len(text) or not isinstance(value, dict):
            raise RegistryError("stdio_output_invalid")
        return value

    def prepare(self, request: dict[str, Any]) -> Any:
        return self._call("prepare", "request", request)

    def execute(self, prepared: Any) -> Any:
        return self._call("execute", "prepared", prepared)


class DeviceExecutorRegistry:
    """Immutable module ownership assembled atomically from local sources."""

    def __init__(self, manifest_directory: os.PathLike[str] | str, *,
                 entry_points: Any = None,
                 run: Callable[..., Any] = subprocess.Popen) -> None:
        self._prepared: dict[PreparedHandle, tuple[Any, Any]] = {}
        owners: dict[str, Any] = {}
        metadata: dict[str, ModuleMetadata] = {}
        try:
            python_entries = self._entry_points(entry_points)
            for entry in sorted(python_entries, key=lambda item: (item.value, item.name)):
                provider = entry.load()
                if any(not callable(getattr(provider, name, None))
                       for name in ("manifest", "prepare", "execute")):
                    raise RegistryError("provider_invalid")
                manifest = validate_executor_manifest(provider.manifest())
                if manifest["transport"] != "python_entry_point":
                    raise RegistryError("provider_invalid")
                if manifest["entry_point"] != entry.value:
                    raise RegistryError("entry_point_mismatch")
                self._add_owner(owners, metadata, manifest, _PythonOwner(provider))
            for manifest in self._local_manifests(Path(manifest_directory)):
                self._add_owner(owners, metadata, manifest, _StdioOwner(manifest, run))
        except RegistryError:
            raise
        except Exception:
            raise RegistryError("discovery_invalid") from None
        self._owners = owners
        self._module_metadata = MappingProxyType(metadata)

    @staticmethod
    def _entry_points(injected: Any) -> list[Any]:
        source = importlib.metadata.entry_points() if injected is None else injected
        if callable(source):
            source = source()
        if hasattr(source, "select"):
            return list(source.select(group=ENTRY_POINT_GROUP))
        if isinstance(source, dict):
            return list(source.get(ENTRY_POINT_GROUP, ()))
        return [item for item in source
                if getattr(item, "group", ENTRY_POINT_GROUP) == ENTRY_POINT_GROUP]

    @staticmethod
    def _add_owner(owners: dict[str, Any], metadata: dict[str, ModuleMetadata],
                   manifest: dict[str, Any], owner: Any) -> None:
        if any(module_id in owners for module_id in manifest["module_ids"]):
            raise RegistryError("module_id_duplicate")
        public = ModuleMetadata(manifest["provider"], manifest["transport"])
        for module_id in manifest["module_ids"]:
            owners[module_id] = owner
            metadata[module_id] = public

    @staticmethod
    def _local_manifests(directory: Path) -> list[dict[str, Any]]:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | getattr(os, "O_CLOEXEC", 0)
        flags |= getattr(os, "O_NOFOLLOW", 0)
        try:
            directory_fd = os.open(directory, flags)
        except FileNotFoundError:
            return []
        except OSError:
            raise RegistryError("manifest_directory_invalid") from None
        manifests: list[dict[str, Any]] = []
        total = 0
        try:
            directory_info = os.fstat(directory_fd)
            if not stat.S_ISDIR(directory_info.st_mode):
                raise RegistryError("manifest_directory_invalid")
            try:
                names = sorted(name for name in os.listdir(directory_fd)
                               if name.endswith(".json"))
            except OSError:
                raise RegistryError("manifest_directory_invalid") from None
            if len(names) > MAX_MANIFEST_FILES:
                raise RegistryError("manifest_limit_exceeded")
            file_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
            file_flags |= getattr(os, "O_NOFOLLOW", 0)
            for name in names:
                try:
                    fd = os.open(name, file_flags, dir_fd=directory_fd)
                    try:
                        before = os.fstat(fd)
                        if not stat.S_ISREG(before.st_mode):
                            raise RegistryError("manifest_file_invalid")
                        if before.st_size > MAX_MANIFEST_BYTES:
                            raise RegistryError("manifest_limit_exceeded")
                        chunks = bytearray()
                        while len(chunks) <= MAX_MANIFEST_BYTES:
                            chunk = os.read(fd, MAX_MANIFEST_BYTES + 1 - len(chunks))
                            if not chunk:
                                break
                            chunks.extend(chunk)
                        after = os.fstat(fd)
                    finally:
                        os.close(fd)
                    identity = (before.st_dev, before.st_ino, before.st_mode, before.st_size)
                    if identity != (after.st_dev, after.st_ino, after.st_mode, after.st_size):
                        raise RegistryError("manifest_file_invalid")
                    if len(chunks) != before.st_size or len(chunks) > MAX_MANIFEST_BYTES:
                        raise RegistryError("manifest_file_invalid")
                    total += len(chunks)
                    if total > MAX_MANIFEST_TOTAL_BYTES:
                        raise RegistryError("manifest_limit_exceeded")

                    def unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
                        value: dict[str, Any] = {}
                        for key, item in pairs:
                            if key in value:
                                raise ValueError("duplicate JSON key")
                            value[key] = item
                        return value

                    value = json.loads(bytes(chunks).decode("utf-8", errors="strict"),
                                       object_pairs_hook=unique_object,
                                       parse_constant=lambda _value: (_ for _ in ()).throw(
                                           ValueError("invalid constant")))
                    manifest = validate_executor_manifest(value)
                except RegistryError:
                    raise
                except (OSError, ValueError):
                    raise RegistryError("manifest_file_invalid") from None
                if manifest["transport"] != "json_stdio":
                    raise RegistryError("manifest_file_invalid")
                manifests.append(manifest)
        finally:
            os.close(directory_fd)
        return manifests

    @property
    def module_owners(self) -> MappingProxyType:
        """Compatibility name exposing metadata, never executable owners."""
        return self._module_metadata

    @property
    def module_metadata(self) -> MappingProxyType:
        return self._module_metadata

    def prepare(self, module_id: str, params: Any) -> PreparedHandle:
        if len(self._prepared) >= MAX_PREPARED_HANDLES:
            raise RegistryError("prepared_limit_exceeded")
        try:
            request = validate_request({"contract_version": CONTRACT_VERSION,
                                        "module_id": module_id, "params": params})
        except ContractValidationError:
            raise RegistryError("request_invalid") from None
        owner = self._owners.get(request["module_id"])
        if owner is None:
            raise RegistryError("module_not_found")
        try:
            prepared = validate_prepared_payload(owner.prepare(request))
        except RegistryError:
            raise
        except Exception:
            raise RegistryError("prepare_failed") from None
        handle = PreparedHandle(_HANDLE_KEY, self)
        self._prepared[handle] = (owner, prepared)
        return handle

    def _authentic(self, handle: PreparedHandle) -> bool:
        return type(handle) is PreparedHandle and handle._belongs_to(self)

    def execute(self, handle: PreparedHandle) -> dict[str, Any]:
        if not self._authentic(handle) or handle not in self._prepared:
            raise RegistryError("handle_invalid")
        owner, prepared = self._prepared.pop(handle)
        try:
            return validate_result(owner.execute(prepared))
        except RegistryError:
            raise
        except Exception:
            raise RegistryError("execute_failed") from None

    def discard(self, handle: PreparedHandle) -> None:
        """Release an authentic handle; repeated release reveals no state."""
        if not self._authentic(handle):
            raise RegistryError("handle_invalid")
        self._prepared.pop(handle, None)

    cancel = discard


__all__ = ["DeviceExecutorRegistry", "ModuleMetadata", "PreparedHandle", "RegistryError"]
