"""Read-only, replay-resistant planning for adopting legacy systemd units."""

from __future__ import annotations

import base64
import contextlib
import hashlib
import hmac
import json
import os
import re
import secrets
import stat
from pathlib import Path
from typing import Any
from urllib.parse import urlsplit

from . import lifecycle
from .lifecycle import Layout
from .lifecycle_profiles import ProfileError, load_profiles

SCHEMA = "flyto.legacy-takeover-plan.v1"
REVALIDATION_SCHEMA = "flyto.legacy-takeover-revalidation.v1"
CAPSULE_SCHEMA = "flyto.legacy-takeover-capsule.v1"
WINDOW_SCHEMA = "flyto.legacy-takeover-window.v1"
MAX_UNIT_BYTES = 128 * 1024
MAX_PREREQUISITE_BYTES = 64 * 1024
MAX_UNITS = 64
MAX_CAPSULE_BYTES = MAX_UNIT_BYTES * MAX_UNITS + 64 * 1024
MAX_PROFILE = 64
_PROFILE = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")
_UNIT = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.@:-]{0,127}\.(?:service|timer|path)\Z")
_SAFE_STATE = re.compile(r"[a-z][a-z0-9_-]{0,31}\Z")
_RESTORABLE_ACTIVE = frozenset({"active", "inactive"})
_RESTORABLE_ENABLED = frozenset({"enabled", "disabled"})
_DEVICE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._:-]{0,127}\Z")
_RESOURCE_ID = re.compile(r"[A-Za-z0-9][A-Za-z0-9._:-]{0,127}\Z")
_SAFE_PATH_KEYS = frozenset(
    {"WorkingDirectory", "EnvironmentFile", "ConditionPathExists", "ReadWritePaths"}
)
_CRITICAL_DIRECTIVES = _SAFE_PATH_KEYS | {"ExecStart"}
_SINGLETON_DIRECTIVES = frozenset({"ExecStart", "WorkingDirectory"})
_MACHINE_ID = re.compile(r"[0-9a-f]{32}\Z")


class TakeoverError(RuntimeError):
    """A bounded legacy-takeover refusal."""

    def __init__(self, reason: str) -> None:
        self.reason = reason
        super().__init__(reason)


class _PrivateMissing(RuntimeError):
    pass


def _fail(reason: str) -> None:
    raise TakeoverError(reason)


def _read_regular(path: Path, limit: int, reason: str) -> tuple[bytes, os.stat_result]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
        try:
            before = os.fstat(fd)
            if not stat.S_ISREG(before.st_mode) or not 0 < before.st_size <= limit:
                _fail(reason)
            data = bytearray()
            while len(data) <= limit:
                chunk = os.read(fd, min(65536, limit + 1 - len(data)))
                if not chunk:
                    break
                data.extend(chunk)
            after = os.fstat(fd)

            def identity(item: os.stat_result) -> tuple[int, int, int, int, int]:
                return (
                    item.st_dev,
                    item.st_ino,
                    item.st_size,
                    item.st_mtime_ns,
                    item.st_ctime_ns,
                )

            if (
                len(data) != before.st_size
                or len(data) > limit
                or identity(before) != identity(after)
            ):
                _fail("snapshot_race")
            return bytes(data), before
        finally:
            os.close(fd)
    except TakeoverError:
        raise
    except FileNotFoundError:
        _fail(reason.replace("_invalid", "_missing"))
    except OSError:
        _fail(reason)


def _strict_json(
    path: Path, label: str
) -> tuple[dict[str, Any] | None, os.stat_result | None, bytes | None]:
    try:
        path.lstat()
    except FileNotFoundError:
        return None, None, None
    except OSError:
        return {}, None, None
    try:
        raw, metadata = _read_regular(path, MAX_PREREQUISITE_BYTES, f"{label}_invalid")

        def pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
            value: dict[str, Any] = {}
            for key, item in items:
                if key in value:
                    raise ValueError("duplicate key")
                value[key] = item
            return value

        value = json.loads(
            raw.decode("utf-8", "strict"),
            object_pairs_hook=pairs,
            parse_constant=lambda _value: (_ for _ in ()).throw(ValueError("constant")),
        )
        if not isinstance(value, dict):
            return {}, metadata, raw
        return value, metadata, raw
    except (ValueError, UnicodeError):
        return {}, None, None


def _result(present: bool, valid: bool, label: str) -> dict[str, Any]:
    reason = "ok" if valid else f"{label}_{'invalid' if present else 'missing'}"
    return {"present": present, "valid": valid, "reason": reason}


def _credential(path: Path) -> tuple[dict[str, Any], str | None, str | None]:
    value, metadata, raw = _strict_json(path, "credential")
    present = value is not None
    device_id = value.get("device_id") if value else None
    secret = value.get("device_secret") if value else None
    valid = bool(
        present
        and metadata is not None
        and metadata.st_mode & 0o077 == 0
        and set(value or {}) == {"device_id", "device_secret"}
        and isinstance(device_id, str)
        and _DEVICE_ID.fullmatch(device_id)
        and isinstance(secret, str)
        and 0 < len(secret) <= 512
        and all(0x21 <= ord(character) <= 0x7E for character in secret)
    )
    digest = hashlib.sha256(raw).hexdigest() if valid and raw is not None else None
    return _result(present, valid, "credential"), device_id if valid else None, digest


def _identity(path: Path) -> tuple[dict[str, Any], str | None, str | None]:
    value, _metadata, raw = _strict_json(path, "identity")
    present = value is not None
    device_id = value.get("device_id") if value else None
    valid = bool(
        present
        and set(value or {}) == {"device_id"}
        and isinstance(device_id, str)
        and _DEVICE_ID.fullmatch(device_id)
    )
    digest = hashlib.sha256(raw).hexdigest() if valid and raw is not None else None
    return _result(present, valid, "identity"), device_id if valid else None, digest


def _env_prerequisite(path: Path) -> tuple[dict[str, Any], str | None]:
    try:
        path.lstat()
    except FileNotFoundError:
        return {"present": False, "valid": False, "reason": "config_missing"}, None
    except OSError:
        return {"present": True, "valid": False, "reason": "config_invalid"}, None
    try:
        raw, _ = _read_regular(path, MAX_PREREQUISITE_BYTES, "config_invalid")
        text = raw.decode("utf-8", "strict")
        values: dict[str, str] = {}
        valid = True
        for line in text.splitlines():
            if not line or line.startswith("#"):
                continue
            key, separator, value = line.partition("=")
            if (
                not separator
                or key not in {"FLYTO_CLOUD_URL", "FLYTO_ROBOT_RESOURCE_ID"}
                or key in values
            ):
                valid = False
                break
            if (
                not value
                or len(value) > 2048
                or any(ord(char) < 0x21 or ord(char) > 0x7E for char in value)
            ):
                valid = False
                break
            values[key] = value
        resource_id = values.get("FLYTO_ROBOT_RESOURCE_ID", "")
        cloud_url = values.get("FLYTO_CLOUD_URL", "")
        parsed = urlsplit(cloud_url)
        valid = bool(
            valid
            and set(values) == {"FLYTO_CLOUD_URL", "FLYTO_ROBOT_RESOURCE_ID"}
            and _RESOURCE_ID.fullmatch(resource_id)
            and parsed.scheme in {"http", "https"}
            and parsed.hostname
            and parsed.username is None
            and parsed.password is None
            and parsed.path in {"", "/"}
            and not parsed.query
            and not parsed.fragment
        )
        if valid:
            try:
                valid = parsed.port is None or 0 < parsed.port <= 65535
            except ValueError:
                valid = False
    except (ValueError, UnicodeError):
        valid = False
    result = {"present": True, "valid": valid, "reason": "ok" if valid else "config_invalid"}
    digest = hashlib.sha256(raw).hexdigest() if valid else None
    return result, digest


def _safe_source(value: str) -> str:
    value = value.removeprefix("-")
    if (
        not value
        or len(value) > 4096
        or any(ord(char) < 0x20 or ord(char) > 0x7E for char in value)
    ):
        _fail("unit_source_path_unsafe")
    path = Path(value)
    if not path.is_absolute() or any(part in {".", ".."} for part in value.split("/")):
        _fail("unit_source_path_unsafe")
    return value


def _reject_symlink_chain(root: Path, path: Path) -> None:
    try:
        relative = path.relative_to(root)
    except ValueError:
        _fail("unit_source_path_unsafe")
    current = root
    for part in relative.parts:
        current /= part
        try:
            if stat.S_ISLNK(current.lstat().st_mode):
                _fail("unit_symlink")
        except FileNotFoundError:
            return
        except OSError:
            _fail("unit_invalid")


def _source_metadata(root: Path, directive: str, value: str) -> tuple[str, str]:
    source = Path(_safe_source(value))
    rooted = source if source.is_relative_to(root) else root / source.relative_to("/")
    _reject_symlink_chain(root, rooted)
    digest = hashlib.sha256(str(source).encode()).hexdigest()
    return directive, digest


def _unit_metadata(raw: bytes, root: Path) -> list[dict[str, str]]:
    try:
        text = raw.decode("utf-8", "strict")
    except UnicodeError:
        _fail("unit_malformed")
    paths: set[tuple[str, str]] = set()
    critical: set[tuple[str, str, str]] = set()
    singleton: set[tuple[str, str]] = set()
    section = ""
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith(("#", ";")):
            continue
        if stripped.startswith("[") and stripped.endswith("]"):
            section = stripped[1:-1]
            if section not in {"Unit", "Service", "Install", "Timer", "Path"}:
                _fail("unit_malformed")
            continue
        key, separator, value = stripped.partition("=")
        if not section or not separator or not key or "\x00" in value:
            _fail("unit_malformed")
        if key in _CRITICAL_DIRECTIVES:
            marker = (section, key, value)
            single = (section, key)
            if marker in critical or (key in _SINGLETON_DIRECTIVES and single in singleton):
                _fail("unit_duplicate_directive")
            critical.add(marker)
            singleton.add(single)
        if key in _SAFE_PATH_KEYS:
            for item in value.split():
                paths.add(_source_metadata(root, key, item))
        elif key == "ExecStart":
            command = value.removeprefix("-").split(maxsplit=1)[0]
            paths.add(_source_metadata(root, key, command))
    if len(paths) > 64:
        _fail("unit_source_metadata_oversized")
    return [{"directive": key, "path_digest": value} for key, value in sorted(paths)]


def _states(systemd: Any, names: list[str]) -> list[dict[str, str]]:
    try:
        observed = systemd.health(names)
    except Exception:
        _fail("systemd_observation_failed")
    if not isinstance(observed, list) or len(observed) != len(names):
        _fail("systemd_observation_inconsistent")
    result = []
    for entry, name in zip(observed, names, strict=True):
        if not isinstance(entry, dict) or entry.get("unit") != name:
            _fail("systemd_observation_inconsistent")
        active, enabled = entry.get("active"), entry.get("enabled")
        if not isinstance(active, str) or not _SAFE_STATE.fullmatch(active):
            _fail("systemd_observation_invalid")
        if not isinstance(enabled, str) or not _SAFE_STATE.fullmatch(enabled):
            _fail("systemd_observation_invalid")
        result.append({"unit": name, "active": active, "enabled": enabled})
    return result


def _snapshot(layout: Layout, names: list[str], systemd: Any, owner: int) -> list[dict[str, Any]]:
    units: list[dict[str, Any]] = []
    for name in names:
        path = layout.unit_dir / name
        _reject_symlink_chain(layout.root, path)
        try:
            path.lstat()
        except FileNotFoundError:
            continue
        except OSError:
            _fail("unit_invalid")
        raw, info = _read_regular(path, MAX_UNIT_BYTES, "unit_invalid")
        if info.st_uid != owner:
            _fail("unit_foreign")
        units.append(
            {
                "name": name,
                "size": len(raw),
                "sha256": hashlib.sha256(raw).hexdigest(),
                "source_paths": _unit_metadata(raw, layout.root),
            }
        )
    states = {entry["unit"]: entry for entry in _states(systemd, [u["name"] for u in units])}
    for unit in units:
        unit.update(active=states[unit["name"]]["active"], enabled=states[unit["name"]]["enabled"])
    return units


def _authority(layout: Layout, profile: str) -> str:
    raw, _info = _read_regular(layout.root / "etc/machine-id", 64, "host_authority_invalid")
    try:
        machine_id = raw.decode("ascii", "strict").strip().lower()
    except UnicodeError:
        _fail("host_authority_invalid")
    if not _MACHINE_ID.fullmatch(machine_id):
        _fail("host_authority_invalid")
    try:
        root_info = layout.root.stat()
    except OSError:
        _fail("root_invalid")
    material = json.dumps(
        {
            "machine_id": machine_id,
            "machine_id_content": hashlib.sha256(raw).hexdigest(),
            "profile": profile,
            "root": str(layout.root),
            "root_device": root_info.st_dev,
            "root_inode": root_info.st_ino,
        },
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    return hashlib.sha256(material).hexdigest()


def _lifecycle_observation(layout: Layout, profile: str) -> tuple[dict[str, str], str]:
    """Bind the exact committed authority without trusting mutable views."""

    state_marker = "missing"
    snapshot_marker = "none"
    managed: dict[str, str] = {}
    try:
        raw, _metadata = _read_regular(
            layout.state_file, MAX_PREREQUISITE_BYTES, "lifecycle_state_invalid"
        )
        state_marker = hashlib.sha256(raw).hexdigest()
        state, v1_compatible = lifecycle._read_runtime_state_from(
            layout, raw.decode("utf-8", "strict")
        )
        # This is the same committed-record validation used by runtime.  A
        # mutable per-version snapshot is deliberately never consulted.
        snapshot = lifecycle.current_activation_snapshot(layout, state)
        committed = lifecycle.committed_activation(
            layout, state, v1_compatible=v1_compatible
        )
        if snapshot is not None and committed is not None and snapshot == committed:
            document = committed.document()
            snapshot_marker = hashlib.sha256(
                json.dumps(document, sort_keys=True, separators=(",", ":")).encode()
            ).hexdigest()
            managed = {
                name: hashlib.sha256(text.encode()).hexdigest()
                for name, text in committed.units.items()
            }
    except FileNotFoundError:
        pass
    except (lifecycle.LifecycleError, OSError, UnicodeError, ValueError, TakeoverError):
        snapshot_marker = "invalid"
    material = {
        "managed": managed,
        "snapshot": snapshot_marker,
        "state": state_marker,
    }
    generation = hashlib.sha256(
        json.dumps(material, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return managed, generation


def _commissioning(layout: Layout) -> str:
    credential, credential_id, credential_digest = _credential(
        layout.credentials_dir / "runner-credentials.json"
    )
    identity, identity_id, identity_digest = _identity(layout.identity_file)
    if credential["valid"] and identity["valid"] and credential_id != identity_id:
        identity = _result(True, False, "identity")
    robot_env, env_digest = _env_prerequisite(layout.config_file)
    prerequisites = (credential, identity, robot_env)
    if not all(item["valid"] for item in prerequisites):
        reason = next(item["reason"] for item in prerequisites if not item["valid"])
        _fail(reason)
    material = {
        "credential": credential_digest,
        "identity": identity_digest,
        "robot_env": env_digest,
    }
    return hashlib.sha256(
        json.dumps(material, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()


def _observe(
    layout: Layout, profile: str, names: list[str], systemd: Any
) -> dict[str, Any]:
    authority = _authority(layout, profile)
    commissioning = _commissioning(layout)
    managed, lifecycle_generation = _lifecycle_observation(layout, profile)
    try:
        owner = layout.root.stat().st_uid
    except OSError:
        _fail("root_invalid")
    units = _snapshot(layout, names, systemd, owner)
    foreign = [unit for unit in units if managed.get(unit["name"]) != unit["sha256"]]
    classification = [
        [unit["name"], "foreign" if unit in foreign else "managed"] for unit in units
    ]
    generation_material = {"classification": classification, "units": units}
    systemd_generation = hashlib.sha256(
        json.dumps(generation_material, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return {
        "authority_digest": authority,
        "commissioning_digest": commissioning,
        "lifecycle_generation": lifecycle_generation,
        "systemd_generation": systemd_generation,
        "unit_count": len(units),
        "foreign_count": len(foreign),
        # Private execution material.  Plans deliberately select only the
        # generation fields above, so unit bytes and names never enter a
        # receipt or a command response.
        "_foreign": foreign,
    }


def plan_takeover(
    *,
    layout: Layout,
    profile: str,
    profiles: Path | None,
    systemd: Any,
    acknowledged: bool,
) -> dict[str, Any]:
    """Return one content-addressed plan without writing or mutating systemd."""

    if not acknowledged:
        _fail("legacy_takeover_not_acknowledged")
    if not isinstance(profile, str) or not _PROFILE.fullmatch(profile):
        _fail("profile_invalid")
    try:
        registry = load_profiles(profiles)
    except ProfileError:
        _fail("profiles_invalid")
    if profile not in registry:
        _fail("profile_unknown")
    names = sorted(registry[profile].unit_names())
    if not names or len(names) > MAX_UNITS or any(not _UNIT.fullmatch(name) for name in names):
        _fail("profile_invalid")
    root = layout.root.resolve(strict=False)
    if not root.is_absolute() or len(str(root)) > 4096:
        _fail("root_invalid")
    layout = Layout(root)
    first = _observe(layout, profile, names, systemd)
    second = _observe(layout, profile, names, systemd)
    if first != second:
        _fail("snapshot_race")
    if not first["unit_count"]:
        _fail("no_legacy_units")
    if not first["foreign_count"]:
        _fail("already_managed")
    body = {
        "schema": SCHEMA,
        "ok": True,
        "authority_digest": first["authority_digest"],
        "commissioning_digest": first["commissioning_digest"],
        "lifecycle_generation": first["lifecycle_generation"],
        "systemd_generation": first["systemd_generation"],
    }
    digest = hashlib.sha256(
        json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    return {**body, "receipt_digest": digest}


_RECEIPT_FIELDS = frozenset(
    {
        "schema",
        "ok",
        "authority_digest",
        "commissioning_digest",
        "lifecycle_generation",
        "systemd_generation",
        "receipt_digest",
    }
)
_DIGEST = re.compile(r"[0-9a-f]{64}\Z")


def revalidate_takeover_receipt(
    *,
    receipt: Any,
    layout: Layout,
    profile: str,
    profiles: Path | None,
    systemd: Any,
) -> dict[str, Any]:
    """Revalidate a plan against two fresh, read-only observations."""

    def result(ok: bool, reason: str) -> dict[str, Any]:
        return {"schema": REVALIDATION_SCHEMA, "ok": ok, "reason": reason}

    if not isinstance(receipt, dict) or set(receipt) != _RECEIPT_FIELDS:
        return result(False, "receipt_invalid")
    if receipt.get("schema") != SCHEMA or receipt.get("ok") is not True:
        return result(False, "receipt_invalid")
    if any(
        not isinstance(receipt.get(key), str) or not _DIGEST.fullmatch(receipt[key])
        for key in _RECEIPT_FIELDS - {"schema", "ok"}
    ):
        return result(False, "receipt_invalid")
    body = {key: receipt[key] for key in receipt if key != "receipt_digest"}
    expected = hashlib.sha256(
        json.dumps(body, sort_keys=True, separators=(",", ":")).encode()
    ).hexdigest()
    if not hmac.compare_digest(expected, receipt["receipt_digest"]):
        return result(False, "receipt_tampered")
    try:
        if not isinstance(profile, str) or not _PROFILE.fullmatch(profile):
            _fail("profile_invalid")
        registry = load_profiles(profiles)
        if profile not in registry:
            _fail("profile_unknown")
        names = sorted(registry[profile].unit_names())
        if not names or len(names) > MAX_UNITS or any(
            not _UNIT.fullmatch(name) for name in names
        ):
            _fail("profile_invalid")
        root = layout.root.resolve(strict=False)
        if not root.is_absolute() or len(str(root)) > 4096:
            _fail("root_invalid")
        layout = Layout(root)
        first = _observe(layout, profile, names, systemd)
        second = _observe(layout, profile, names, systemd)
        if first != second:
            return result(False, "snapshot_race")
        for key in (
            "authority_digest",
            "commissioning_digest",
            "lifecycle_generation",
            "systemd_generation",
        ):
            if not hmac.compare_digest(receipt[key], first[key]):
                return result(False, f"{key.removesuffix('_digest')}_changed")
        if not first["unit_count"] or not first["foreign_count"]:
            return result(False, "takeover_target_changed")
        return result(True, "valid")
    except (ProfileError, TakeoverError):
        return result(False, "observation_invalid")


def read_takeover_receipt(path: Path) -> dict[str, Any]:
    """Read one bounded, regular, no-follow receipt for CLI revalidation."""

    value, metadata, _raw = _strict_json(path, "receipt")
    if value is None or metadata is None:
        _fail("receipt_invalid")
    return value


def _takeover_dir(layout: Layout) -> Path:
    return layout.state_dir / "legacy-takeover"


def _capsule_path(layout: Layout, digest: str) -> Path:
    return _takeover_dir(layout) / "capsules" / f"{digest}.json"


def takeover_window_path(layout: Layout) -> Path:
    """Private durable marker for an incomplete legacy takeover."""

    return _takeover_dir(layout) / "window.json"


def _json_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode("utf-8")


@contextlib.contextmanager
def _private_dirfd(layout: Layout, parts: tuple[str, ...], *, create: bool):
    """Walk from the already-open root, retaining every nofollow dirfd."""

    flags = (
        os.O_RDONLY
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_DIRECTORY", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptors: list[int] = []
    try:
        descriptor = os.open(layout.root, flags)
        descriptors.append(descriptor)
        authority = os.fstat(descriptor).st_uid
        base = Path("var/lib/flyto-robot").parts
        for index, part in enumerate((*base, *parts)):
            try:
                child = os.open(part, flags, dir_fd=descriptor)
            except FileNotFoundError as error:
                if not create:
                    raise _PrivateMissing from error
                os.mkdir(part, 0o700, dir_fd=descriptor)
                os.fsync(descriptor)
                child = os.open(part, flags, dir_fd=descriptor)
            metadata = os.fstat(child)
            private = index >= len(base)
            if (
                not stat.S_ISDIR(metadata.st_mode)
                or metadata.st_uid != authority
                or (private and stat.S_IMODE(metadata.st_mode) != 0o700)
            ):
                _fail("takeover_storage_invalid")
            descriptor = child
            descriptors.append(descriptor)
        yield descriptor
    except _PrivateMissing:
        raise
    except OSError:
        _fail("takeover_storage_invalid")
    finally:
        for descriptor in reversed(descriptors):
            with contextlib.suppress(OSError):
                os.close(descriptor)


def _write_private(
    layout: Layout, parts: tuple[str, ...], name: str, value: dict[str, Any]
) -> None:
    """Atomic write using only the retained nofollow directory descriptor."""

    data = _json_bytes(value) + b"\n"
    temporary = f".{name}.{secrets.token_hex(16)}.tmp"
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    with _private_dirfd(layout, parts, create=True) as directory_fd:
        try:
            authority = os.fstat(directory_fd).st_uid
            try:
                existing = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            except FileNotFoundError:
                existing = None
            if existing is not None and (
                not stat.S_ISREG(existing.st_mode)
                or existing.st_uid != authority
                or stat.S_IMODE(existing.st_mode) != 0o600
            ):
                _fail("takeover_storage_invalid")
            descriptor = os.open(temporary, flags, 0o600, dir_fd=directory_fd)
            try:
                os.fchmod(descriptor, 0o600)
                offset = 0
                while offset < len(data):
                    offset += os.write(descriptor, data[offset:])
                os.fsync(descriptor)
            finally:
                os.close(descriptor)
            os.replace(temporary, name, src_dir_fd=directory_fd, dst_dir_fd=directory_fd)
            os.fsync(directory_fd)
        except BaseException:
            with contextlib.suppress(OSError):
                os.unlink(temporary, dir_fd=directory_fd)
            raise


def _remove_private_exact(
    layout: Layout, parts: tuple[str, ...], name: str, expected: dict[str, Any]
) -> bool:
    """Unlink ``name`` only while it is still the exact private file expected."""

    wanted = _json_bytes(expected) + b"\n"
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    with _private_dirfd(layout, parts, create=False) as directory_fd:
        try:
            descriptor = os.open(name, flags, dir_fd=directory_fd)
        except (FileNotFoundError, OSError):
            return False
        try:
            metadata = os.fstat(descriptor)
            authority = os.fstat(directory_fd).st_uid
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != authority
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or metadata.st_size != len(wanted)
            ):
                return False
            raw = bytearray()
            while len(raw) < len(wanted):
                chunk = os.read(descriptor, len(wanted) - len(raw))
                if not chunk:
                    break
                raw.extend(chunk)
            current = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if (
                bytes(raw) != wanted
                or (current.st_dev, current.st_ino) != (metadata.st_dev, metadata.st_ino)
            ):
                return False
            os.unlink(name, dir_fd=directory_fd)
            os.fsync(directory_fd)
            return True
        except OSError:
            return False
        finally:
            os.close(descriptor)


def _replace_private_exact(
    layout: Layout,
    parts: tuple[str, ...],
    name: str,
    expected: dict[str, Any],
    replacement: dict[str, Any],
) -> bool:
    """Atomically replace one exact private document, never a substitute."""

    wanted = _json_bytes(expected) + b"\n"
    data = _json_bytes(replacement) + b"\n"
    temporary = f".{name}.{secrets.token_hex(16)}.tmp"
    read_flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    write_flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    with _private_dirfd(layout, parts, create=False) as directory_fd:
        try:
            authority = os.fstat(directory_fd).st_uid
            descriptor = os.open(name, read_flags, dir_fd=directory_fd)
            try:
                metadata = os.fstat(descriptor)
                raw = bytearray()
                while len(raw) < len(wanted):
                    chunk = os.read(descriptor, len(wanted) - len(raw))
                    if not chunk:
                        break
                    raw.extend(chunk)
                if (
                    not stat.S_ISREG(metadata.st_mode)
                    or metadata.st_uid != authority
                    or stat.S_IMODE(metadata.st_mode) != 0o600
                    or metadata.st_size != len(wanted)
                    or bytes(raw) != wanted
                ):
                    return False
            finally:
                os.close(descriptor)
            temporary_fd = os.open(temporary, write_flags, 0o600, dir_fd=directory_fd)
            try:
                os.fchmod(temporary_fd, 0o600)
                offset = 0
                while offset < len(data):
                    offset += os.write(temporary_fd, data[offset:])
                os.fsync(temporary_fd)
            finally:
                os.close(temporary_fd)
            current = os.stat(name, dir_fd=directory_fd, follow_symlinks=False)
            if (current.st_dev, current.st_ino) != (metadata.st_dev, metadata.st_ino):
                return False
            os.replace(temporary, name, src_dir_fd=directory_fd, dst_dir_fd=directory_fd)
            os.fsync(directory_fd)
            return True
        except OSError:
            return False
        finally:
            with contextlib.suppress(OSError):
                os.unlink(temporary, dir_fd=directory_fd)


def seal_takeover_window(
    *,
    receipt: Any,
    layout: Layout,
    profile: str,
    profiles: Path | None,
    systemd: Any,
    version: str = "1.0.0",
) -> str:
    """Seal one exact B1 observation before any foreign unit is touched.

    The return value is only the content digest.  Raw unit bytes remain in the
    root-only capsule and are never placed in reports or exception details.
    Callers must hold the lifecycle advisory lock.
    """

    checked = revalidate_takeover_receipt(
        receipt=receipt, layout=layout, profile=profile, profiles=profiles, systemd=systemd
    )
    if not checked["ok"]:
        _fail(checked["reason"])
    if layout.current.exists() or layout.current.is_symlink() or layout.state_file.exists():
        _fail("takeover_not_first_install")
    registry = load_profiles(profiles)
    names = sorted(registry[profile].unit_names())
    observation = _observe(layout, profile, names, systemd)
    for key in (
        "authority_digest",
        "commissioning_digest",
        "lifecycle_generation",
        "systemd_generation",
    ):
        if not hmac.compare_digest(observation[key], receipt[key]):
            _fail(f"{key.removesuffix('_digest')}_changed")

    units: list[dict[str, Any]] = []
    for metadata in observation["_foreign"]:
        if (
            metadata["active"] not in _RESTORABLE_ACTIVE
            or metadata["enabled"] not in _RESTORABLE_ENABLED
        ):
            _fail("systemd_state_not_reproducible")
        name = metadata["name"]
        raw, _ = _read_regular(layout.unit_dir / name, MAX_UNIT_BYTES, "unit_invalid")
        if hashlib.sha256(raw).hexdigest() != metadata["sha256"]:
            _fail("unit_changed")
        # Parsing here proves the bytes are bounded UTF-8 systemd text, not an
        # arbitrary binary blob smuggled into the recovery store.
        _unit_metadata(raw, layout.root)
        units.append(
            {
                "name": name,
                "bytes": base64.b64encode(raw).decode("ascii"),
                "size": len(raw),
                "sha256": metadata["sha256"],
                "source_paths": metadata["source_paths"],
                "active": metadata["active"],
                "enabled": metadata["enabled"],
            }
        )
    # Unit capture is itself a race window.  Bind the capsule only if a final
    # full observation (authority, commissioning, lifecycle, unit bytes and
    # systemd states) is still exactly the observation the receipt authorized.
    final_observation = _observe(layout, profile, names, systemd)
    if final_observation != observation:
        _fail("snapshot_race")
    body = {
        "schema": CAPSULE_SCHEMA,
        "profile": profile,
        "version": version,
        "pre_current": "absent",
        "pre_state": "absent",
        "receipt_digest": receipt["receipt_digest"],
        "authority_digest": observation["authority_digest"],
        "commissioning_digest": observation["commissioning_digest"],
        "lifecycle_generation": observation["lifecycle_generation"],
        "systemd_generation": observation["systemd_generation"],
        "units": units,
    }
    digest = hashlib.sha256(_json_bytes(body)).hexdigest()
    capsule = {**body, "capsule_digest": digest}
    encoded = _json_bytes(capsule)
    if len(encoded) > MAX_CAPSULE_BYTES:
        _fail("capsule_oversized")
    window = {
        "schema": WINDOW_SCHEMA,
        "capsule_digest": digest,
        "phase": "sealed",
    }
    # A sealed window is a transaction, not a last-writer-wins pointer.  Check
    # it before publishing even an additional capsule: an incomplete takeover
    # must be recovered, never hidden beneath a later seal.
    try:
        existing_window = _load_private(
            layout, ("legacy-takeover",), "window.json", 4096, "takeover_in_progress"
        )
    except _PrivateMissing:
        existing_window = None
    if existing_window is not None and existing_window != window:
        _fail("takeover_in_progress")
    try:
        existing = load_takeover_capsule(layout, digest)
        if existing != capsule:
            _fail("capsule_invalid")
    except _PrivateMissing:
        _write_private(layout, ("legacy-takeover", "capsules"), f"{digest}.json", capsule)
    if existing_window is None:
        _write_private(layout, ("legacy-takeover",), "window.json", window)
        # The window publication is the last authorization boundary before a
        # caller may touch systemd or unit files.  Re-observe only after both
        # the file and its parent have been fsync-ed, and retract precisely the
        # window published above if any sealed fact moved in that interval.
        try:
            published_observation = _observe(layout, profile, names, systemd)
        except Exception:
            published_observation = None
        if published_observation != observation:
            # Cleanup failure must not replace the stable race refusal.  In
            # particular, a swapped parent or a different window is evidence
            # to preserve, not authority to unlink anything else.
            with contextlib.suppress(TakeoverError, _PrivateMissing):
                _remove_private_exact(
                    layout, ("legacy-takeover",), "window.json", window
                )
            _fail("snapshot_race")
    return digest


def _load_private(
    layout: Layout, parts: tuple[str, ...], name: str, limit: int, reason: str
) -> dict[str, Any]:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    with _private_dirfd(layout, parts, create=False) as directory_fd:
        try:
            descriptor = os.open(name, flags, dir_fd=directory_fd)
        except FileNotFoundError as error:
            raise _PrivateMissing from error
        except OSError:
            _fail(reason)
        try:
            metadata = os.fstat(descriptor)
            authority = os.fstat(directory_fd).st_uid
            if (
                not stat.S_ISREG(metadata.st_mode)
                or metadata.st_uid != authority
                or stat.S_IMODE(metadata.st_mode) != 0o600
                or not 0 < metadata.st_size <= limit
            ):
                _fail(reason)
            raw = bytearray()
            while len(raw) <= limit:
                chunk = os.read(descriptor, min(65536, limit + 1 - len(raw)))
                if not chunk:
                    break
                raw.extend(chunk)
            after = os.fstat(descriptor)
            if (
                len(raw) != metadata.st_size
                or len(raw) > limit
                or (metadata.st_dev, metadata.st_ino, metadata.st_size, metadata.st_mtime_ns)
                != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
            ):
                _fail(reason)
            raw = bytes(raw)
        finally:
            os.close(descriptor)
    try:
        def pairs(items: list[tuple[str, Any]]) -> dict[str, Any]:
            result: dict[str, Any] = {}
            for key, value in items:
                if key in result:
                    raise ValueError("duplicate")
                result[key] = value
            return result
        value = json.loads(raw.decode("utf-8", "strict"), object_pairs_hook=pairs)
    except (ValueError, UnicodeError):
        _fail(reason)
    if not isinstance(value, dict):
        _fail(reason)
    return value


def load_takeover_capsule(layout: Layout, digest: str) -> dict[str, Any]:
    """Load and strictly validate one content-addressed private capsule."""

    if not isinstance(digest, str) or not _DIGEST.fullmatch(digest):
        _fail("capsule_invalid")
    value = _load_private(
        layout,
        ("legacy-takeover", "capsules"),
        f"{digest}.json",
        MAX_CAPSULE_BYTES,
        "capsule_invalid",
    )
    required = {
        "schema", "profile", "version", "pre_current", "pre_state", "receipt_digest",
        "authority_digest", "commissioning_digest", "lifecycle_generation",
        "systemd_generation", "units", "capsule_digest",
    }
    if set(value) != required or value.get("schema") != CAPSULE_SCHEMA:
        _fail("capsule_invalid")
    if not isinstance(value.get("profile"), str) or not _PROFILE.fullmatch(value["profile"]):
        _fail("capsule_invalid")
    if (
        not isinstance(value.get("version"), str)
        or not value["version"]
        or value.get("pre_current") != "absent"
        or value.get("pre_state") != "absent"
    ):
        _fail("capsule_invalid")
    if any(not isinstance(value.get(key), str) or not _DIGEST.fullmatch(value[key])
           for key in required
           - {"schema", "profile", "version", "pre_current", "pre_state", "units"}):
        _fail("capsule_invalid")
    units = value.get("units")
    if not isinstance(units, list) or not units or len(units) > MAX_UNITS:
        _fail("capsule_invalid")
    seen: set[str] = set()
    for unit in units:
        if not isinstance(unit, dict) or set(unit) != {
            "name", "bytes", "size", "sha256", "source_paths", "active", "enabled"
        }:
            _fail("capsule_invalid")
        name = unit.get("name")
        if not isinstance(name, str) or not _UNIT.fullmatch(name) or name in seen:
            _fail("capsule_invalid")
        seen.add(name)
        try:
            raw = base64.b64decode(unit["bytes"], validate=True)
        except (ValueError, TypeError):
            _fail("capsule_invalid")
        if (not 0 < len(raw) <= MAX_UNIT_BYTES or unit.get("size") != len(raw)
                or unit.get("sha256") != hashlib.sha256(raw).hexdigest()):
            _fail("capsule_invalid")
        try:
            raw.decode("utf-8", "strict")
        except UnicodeError:
            _fail("capsule_invalid")
        sources = unit.get("source_paths")
        if (
            not isinstance(sources, list)
            or len(sources) > 64
            or any(
                not isinstance(source, dict)
                or set(source) != {"directive", "path_digest"}
                or source.get("directive") not in _CRITICAL_DIRECTIVES
                or not isinstance(source.get("path_digest"), str)
                or not _DIGEST.fullmatch(source["path_digest"])
                for source in sources
            )
            or len({(source["directive"], source["path_digest"]) for source in sources})
            != len(sources)
        ):
            _fail("capsule_invalid")
        if (
            unit.get("active") not in _RESTORABLE_ACTIVE
            or unit.get("enabled") not in _RESTORABLE_ENABLED
        ):
            _fail("capsule_invalid")
    body = {key: value[key] for key in value if key != "capsule_digest"}
    if not hmac.compare_digest(hashlib.sha256(_json_bytes(body)).hexdigest(), digest):
        _fail("capsule_invalid")
    return value


def _takeover_window(layout: Layout) -> tuple[dict[str, Any], dict[str, Any]] | None:
    try:
        value = _load_private(
            layout, ("legacy-takeover",), "window.json", 4096, "takeover_recovery_required"
        )
    except _PrivateMissing:
        return None
    if value.get("schema") != WINDOW_SCHEMA:
        _fail("takeover_recovery_required")
    digest = value.get("capsule_digest")
    phase = value.get("phase")
    sealed = {"schema", "capsule_digest", "phase"}
    intent = {*sealed, "activation_id", "state_digest", "intent_digest"}
    if not isinstance(digest, str) or not _DIGEST.fullmatch(digest):
        _fail("takeover_recovery_required")
    if phase == "sealed" and set(value) != sealed:
        _fail("takeover_recovery_required")
    if phase == "commit_intent" and (
        set(value) != intent
        or not isinstance(value.get("activation_id"), str)
        or not _DIGEST.fullmatch(value["activation_id"])
        or not isinstance(value.get("state_digest"), str)
        or not _DIGEST.fullmatch(value["state_digest"])
        or not isinstance(value.get("intent_digest"), str)
        or not _DIGEST.fullmatch(value["intent_digest"])
        or not hmac.compare_digest(
            hashlib.sha256(
                _json_bytes({key: value[key] for key in value if key != "intent_digest"})
            ).hexdigest(),
            value["intent_digest"],
        )
    ):
        _fail("takeover_recovery_required")
    if phase not in {"sealed", "commit_intent"}:
        _fail("takeover_recovery_required")
    return value, load_takeover_capsule(layout, digest)


def incomplete_takeover(layout: Layout) -> tuple[str, dict[str, Any]] | None:
    pending = _takeover_window(layout)
    if pending is None:
        return None
    window, capsule = pending
    return window["capsule_digest"], capsule


def publish_takeover_commit_intent(
    layout: Layout, digest: str, activation_id: str, state: dict[str, Any]
) -> None:
    """Durably bind the one lifecycle commit that may consume a capsule."""

    if not _DIGEST.fullmatch(activation_id):
        _fail("takeover_recovery_required")
    sealed = {"schema": WINDOW_SCHEMA, "capsule_digest": digest, "phase": "sealed"}
    intent_body = {
        "schema": WINDOW_SCHEMA,
        "capsule_digest": digest,
        "phase": "commit_intent",
        "activation_id": activation_id,
        "state_digest": hashlib.sha256(_json_bytes(state)).hexdigest(),
    }
    intent = {
        **intent_body,
        "intent_digest": hashlib.sha256(_json_bytes(intent_body)).hexdigest(),
    }
    pending = _takeover_window(layout)
    if pending is None or pending[0] != sealed:
        _fail("takeover_recovery_required")
    if not _replace_private_exact(layout, ("legacy-takeover",), "window.json", sealed, intent):
        _fail("takeover_recovery_required")


def clear_takeover_window(layout: Layout, digest: str) -> None:
    """Commit one sealed takeover without exposing or deleting its capsule."""

    pending = _takeover_window(layout)
    if pending is None or pending[0].get("capsule_digest") != digest:
        _fail("takeover_recovery_required")
    window = pending[0]
    if window.get("phase") != "commit_intent":
        _fail("takeover_recovery_required")
    if not _remove_private_exact(layout, ("legacy-takeover",), "window.json", window):
        _fail("takeover_recovery_required")


def _verify_committed_takeover(
    layout: Layout, systemd: Any, window: dict[str, Any], capsule: dict[str, Any]
) -> None:
    """Prove every authority surface before consuming a commit intent."""

    state = lifecycle._read_state(layout)
    snapshot = lifecycle.current_activation_snapshot(layout, state)
    if (
        snapshot is None
        or snapshot.activation_id != window["activation_id"]
        or state.get("current_activation") != window["activation_id"]
        or snapshot.profile != capsule["profile"]
        or snapshot.version != capsule["version"]
        or sorted(snapshot.units) != sorted(unit["name"] for unit in capsule["units"])
        or not hmac.compare_digest(
            hashlib.sha256(_json_bytes(state)).hexdigest(), window["state_digest"]
        )
    ):
        _fail("takeover_recovery_failed")
    target = layout.release(snapshot.version)
    if (
        not layout.current.is_symlink()
        or Path(os.readlink(layout.current)) != target
        or not target.is_dir()
        or not hmac.compare_digest(lifecycle.release_digest(target), snapshot.release_digest)
    ):
        _fail("takeover_recovery_failed")
    for name, rendered in snapshot.units.items():
        raw, _metadata = _read_regular(
            layout.unit_dir / name, MAX_UNIT_BYTES, "takeover_recovery_failed"
        )
        if raw != rendered.encode("utf-8"):
            _fail("takeover_recovery_failed")
    spec = snapshot.spec()
    fields = lifecycle.render_fields(layout, snapshot.python)
    conditions = lifecycle._activation_conditions(spec, fields)
    selected_restart = set(lifecycle._selected_unit_names(spec, conditions, "restart"))
    selected_verify = set(lifecycle._selected_unit_names(spec, conditions, "verify"))
    inactive = set(lifecycle._inactive_conditional_units(spec, conditions))
    observed = {entry["unit"]: entry for entry in systemd.health(sorted(snapshot.units))}
    for unit in spec.units:
        actual = observed.get(unit.name, {})
        if (actual.get("enabled") == "enabled") != unit.enable:
            _fail("takeover_recovery_failed")
        if unit.name in selected_verify and actual.get("active") != "active":
            _fail("takeover_recovery_failed")
        if unit.name in inactive and actual.get("active") not in lifecycle._INERT_ACTIVE_STATES:
            _fail("takeover_recovery_failed")
        if unit.name in selected_restart and actual.get("active") != "active":
            _fail("takeover_recovery_failed")
    if not lifecycle.evaluate(spec, fields, config_file=layout.config_file).ok:
        _fail("takeover_recovery_failed")


def _restore_absent_first_install_authority(layout: Layout, capsule: dict[str, Any]) -> None:
    """Restore the durable first-install authority facts captured by the capsule."""

    if capsule["pre_current"] != "absent" or capsule["pre_state"] != "absent":
        _fail("takeover_recovery_failed")
    if layout.state_file.exists() or layout.state_file.is_symlink():
        _fail("takeover_recovery_failed")
    target = layout.release(capsule["version"])
    if layout.current.is_symlink():
        if Path(os.readlink(layout.current)) != target:
            _fail("takeover_recovery_failed")
    elif layout.current.exists():
        _fail("takeover_recovery_failed")
    view = layout.activation_file(capsule["version"])
    if view.exists() or view.is_symlink():
        if view.is_symlink():
            _fail("takeover_recovery_failed")
        snapshot = lifecycle._load_snapshot_file(view, version=capsule["version"])
        if snapshot.profile != capsule["profile"]:
            _fail("takeover_recovery_failed")
    if layout.current.is_symlink():
        layout.current.unlink()
        lifecycle._fsync_path(layout.current.parent, directory=True)
    if view.is_file():
        view.unlink()
        lifecycle._fsync_path(view.parent, directory=True)


def _verify_uncommitted_intent(
    layout: Layout, window: dict[str, Any], capsule: dict[str, Any]
) -> None:
    """Reject a substituted intent before performing its authorized rollback."""

    view = layout.activation_file(capsule["version"])
    snapshot = lifecycle._load_snapshot_file(
        view, version=capsule["version"], activation_id=window["activation_id"]
    )
    if snapshot.profile != capsule["profile"]:
        _fail("takeover_recovery_failed")
    state = lifecycle._empty_state()
    lifecycle._record_activation(
        state,
        snapshot.version,
        snapshot.release_digest,
        snapshot.profile,
        snapshot.python,
        snapshot.units,
        snapshot.activation_id,
    )
    if not hmac.compare_digest(
        hashlib.sha256(_json_bytes(state)).hexdigest(), window["state_digest"]
    ):
        _fail("takeover_recovery_failed")


def recover_takeover(layout: Layout, systemd: Any) -> dict[str, Any]:
    """Restore exact foreign bytes and four-state systemd state, fail closed."""

    try:
        pending = _takeover_window(layout)
    except (TakeoverError, _PrivateMissing, OSError):
        return {"ok": False, "reason": "takeover_rollback_failed"}
    if pending is None:
        return {"ok": True, "reason": "no_recovery_required"}
    window, capsule = pending
    try:
        if window["phase"] == "commit_intent" and (
            layout.state_file.exists() or layout.state_file.is_symlink()
        ):
            _verify_committed_takeover(layout, systemd, window, capsule)
            if not _remove_private_exact(
                layout, ("legacy-takeover",), "window.json", window
            ):
                _fail("takeover_recovery_failed")
            return {"ok": True, "reason": "takeover_finalized"}
        if window["phase"] == "commit_intent":
            _verify_uncommitted_intent(layout, window, capsule)
        _restore_absent_first_install_authority(layout, capsule)
        names = [unit["name"] for unit in capsule["units"]]
        if (
            _authority(layout, capsule["profile"]) != capsule["authority_digest"]
            or _commissioning(layout) != capsule["commissioning_digest"]
            or _lifecycle_observation(layout, capsule["profile"])[1]
            != capsule["lifecycle_generation"]
        ):
            _fail("takeover_recovery_failed")
        expected_sources: dict[str, list[dict[str, str]]] = {}
        for unit in capsule["units"]:
            raw = base64.b64decode(unit["bytes"], validate=True)
            sources = _unit_metadata(raw, layout.root)
            if sources != unit["source_paths"]:
                _fail("takeover_recovery_failed")
            expected_sources[unit["name"]] = sources
        observed_before = {item["unit"]: item for item in systemd.health(names)}
        for name in names:
            path = layout.unit_dir / name
            if path.exists() or path.is_symlink():
                _read_regular(path, MAX_UNIT_BYTES, "takeover_recovery_failed")
        # Stop what is running now, not what happened to be running when the
        # capsule was sealed.  A crashed activation may have started any of the
        # incoming names before losing its response.
        not_proven_inactive = [
            name for name in names if observed_before.get(name, {}).get("active") != "inactive"
        ]
        if not_proven_inactive:
            systemd.stop(not_proven_inactive)
            quiesced = {item["unit"]: item for item in systemd.health(names)}
            if any(quiesced.get(name, {}).get("active") != "inactive" for name in names):
                _fail("takeover_recovery_failed")
        for unit in capsule["units"]:
            raw = base64.b64decode(unit["bytes"], validate=True)
            path = layout.unit_dir / unit["name"]
            if path.is_symlink():
                _fail("takeover_recovery_failed")
            lifecycle._atomic_write(path, raw.decode("utf-8"), 0o644)
        systemd.daemon_reload()
        disabled = [u["name"] for u in capsule["units"] if u["enabled"] != "enabled"]
        enabled = [u["name"] for u in capsule["units"] if u["enabled"] == "enabled"]
        if disabled:
            systemd.disable(disabled)
        if enabled:
            systemd.enable(enabled)
        inactive = [u["name"] for u in capsule["units"] if u["active"] == "inactive"]
        if inactive:
            systemd.stop(inactive)
        active = [u["name"] for u in capsule["units"] if u["active"] == "active"]
        if active:
            systemd.restart(active)
        observed = {item["unit"]: item for item in systemd.health(names)}
        for unit in capsule["units"]:
            raw, _ = _read_regular(layout.unit_dir / unit["name"], MAX_UNIT_BYTES,
                                   "takeover_recovery_failed")
            if hashlib.sha256(raw).hexdigest() != unit["sha256"]:
                _fail("takeover_recovery_failed")
            if _unit_metadata(raw, layout.root) != expected_sources[unit["name"]]:
                _fail("takeover_recovery_failed")
            state = observed.get(unit["name"], {})
            wanted_active = unit["active"] == "active"
            if (state.get("active") == "active") != wanted_active:
                _fail("takeover_recovery_failed")
            if (state.get("enabled") == "enabled") != (unit["enabled"] == "enabled"):
                _fail("takeover_recovery_failed")
        final = _observe(layout, capsule["profile"], names, systemd)
        for key in (
            "authority_digest",
            "commissioning_digest",
            "lifecycle_generation",
            "systemd_generation",
        ):
            if final[key] != capsule[key]:
                _fail("takeover_recovery_failed")
        if not _remove_private_exact(layout, ("legacy-takeover",), "window.json", window):
            _fail("takeover_recovery_failed")
        return {"ok": True, "reason": "takeover_recovered"}
    except Exception:
        return {"ok": False, "reason": "takeover_rollback_failed"}
