"""Fail-closed construction of immutable, offline Flyto runtime releases."""

from __future__ import annotations

import base64
import contextlib
import csv
import hashlib
import io
import json
import os
import re
import selectors
import shutil
import stat
import subprocess
import time
import zipfile
from collections.abc import Mapping
from email.parser import BytesParser
from pathlib import Path, PurePosixPath
from typing import Any

MAX_MANIFEST_BYTES = 64 * 1024
MAX_WHEEL_BYTES = 128 * 1024 * 1024
MAX_WHEEL_FILES = 4096
MAX_MEMBER_BYTES = 32 * 1024 * 1024
MAX_EXPANDED_BYTES = 128 * 1024 * 1024
MAX_CHILD_OUTPUT = 128 * 1024
INSTALL_TIMEOUT = 120
_PROFILES = {
    "generic": frozenset(("flyto-robotics", "flyto-modules-robotics")),
    "ros2": frozenset(("flyto-robotics", "flyto-modules-robotics")),
    "camera-host": frozenset(("flyto-robotics", "flyto-modules-vision")),
}
_ENTRY_POINTS = {
    "generic": {
        (
            "flyto.modules",
            "robotics",
            "flyto_modules_robotics:register_all",
        ): "flyto-modules-robotics"
    },
    "ros2": {
        (
            "flyto.modules",
            "robotics",
            "flyto_modules_robotics:register_all",
        ): "flyto-modules-robotics"
    },
    "camera-host": {
        ("flyto.modules", "vision", "flyto_modules_vision:register_all"): "flyto-modules-vision",
        (
            "flyto.device_executors",
            "vision",
            "flyto_modules_vision.device_executor:executor",
        ): "flyto-modules-vision",
    },
}
_HEX = re.compile(r"[0-9a-f]{64}\Z")
_VERSION = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.!+-]{0,127}\Z")
_WHEEL = re.compile(r"[A-Za-z0-9_.]+-[A-Za-z0-9_.!+]+(?:-[A-Za-z0-9_.]+){3,4}\.whl\Z")


class BootstrapError(RuntimeError):
    def __init__(self, code: str):
        self.code = code
        super().__init__(code)


def _fail(code: str) -> None:
    raise BootstrapError(code)


def normalize_distribution(value: str) -> str:
    if not isinstance(value, str) or not value or len(value) > 128:
        _fail("manifest_invalid")
    return re.sub(r"[-_.]+", "-", value).lower()


def _read_fd_once(path: Path, limit: int, code: str) -> bytes:
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
        try:
            before = os.fstat(fd)
            if not stat.S_ISREG(before.st_mode) or before.st_size < 1 or before.st_size > limit:
                _fail(code)
            chunks, remaining = [], before.st_size
            while remaining:
                chunk = os.read(fd, min(1024 * 1024, remaining))
                if not chunk:
                    _fail(code)
                chunks.append(chunk)
                remaining -= len(chunk)
            if os.read(fd, 1):
                _fail(code)
            after = os.fstat(fd)
            if (
                before.st_dev,
                before.st_ino,
                before.st_size,
                before.st_mtime_ns,
                before.st_ctime_ns,
            ) != (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns, after.st_ctime_ns):
                _fail(code)
            return b"".join(chunks)
        finally:
            os.close(fd)
    except BootstrapError:
        raise
    except OSError:
        _fail(code)


def _manifest(value: Mapping[str, Any] | str | os.PathLike[str]) -> Mapping[str, Any]:
    if isinstance(value, Mapping):
        try:
            encoded = json.dumps(value, separators=(",", ":")).encode("utf-8")
            if len(encoded) > MAX_MANIFEST_BYTES:
                _fail("manifest_invalid")
            parsed = json.loads(encoded)
        except (TypeError, ValueError, UnicodeError):
            _fail("manifest_invalid")
    else:
        try:
            parsed = json.loads(_read_fd_once(Path(value), MAX_MANIFEST_BYTES, "manifest_invalid"))
        except (ValueError, UnicodeError):
            _fail("manifest_invalid")
    if not isinstance(parsed, dict):
        _fail("manifest_invalid")
    return parsed


def _pins(value: Mapping[str, Any], profile: str) -> list[dict[str, Any]]:
    if profile not in _PROFILES or set(value) != {"schema_version", "profile", "packages"}:
        _fail("manifest_invalid")
    if value["schema_version"] != 1 or value["profile"] != profile:
        _fail("profile_mismatch")
    packages = value["packages"]
    if not isinstance(packages, list) or len(packages) != 2:
        _fail("package_matrix")
    result, names, wheels = [], set(), set()
    for item in packages:
        if not isinstance(item, dict) or set(item) != {
            "name",
            "version",
            "wheel",
            "size",
            "sha256",
        }:
            _fail("manifest_invalid")
        name, version, wheel = normalize_distribution(item["name"]), item["version"], item["wheel"]
        size, digest = item["size"], item["sha256"]
        if (
            name in names
            or not isinstance(wheel, str)
            or wheel in wheels
            or PurePosixPath(wheel).name != wheel
            or not _WHEEL.fullmatch(wheel)
            or not isinstance(version, str)
            or not _VERSION.fullmatch(version)
            or not isinstance(size, int)
            or isinstance(size, bool)
            or not 0 < size <= MAX_WHEEL_BYTES
            or not isinstance(digest, str)
            or not _HEX.fullmatch(digest)
        ):
            _fail("manifest_invalid")
        names.add(name)
        wheels.add(wheel)
        result.append(
            {"name": name, "version": version, "wheel": wheel, "size": size, "sha256": digest}
        )
    if names != _PROFILES[profile]:
        _fail("package_matrix")
    return sorted(result, key=lambda pin: pin["name"])


def _digest(data: bytes) -> str:
    return "sha256=" + base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()


def _validate_wheel(data: bytes, pin: Mapping[str, Any]) -> None:
    fields = pin["wheel"][:-4].split("-")
    if (
        len(fields) not in (5, 6)
        or normalize_distribution(fields[0]) != pin["name"]
        or fields[1] != pin["version"].replace("-", "_")
    ):
        _fail("wheel_metadata")
    if len(data) != pin["size"] or hashlib.sha256(data).hexdigest() != pin["sha256"]:
        _fail("wheel_tampered")
    try:
        with zipfile.ZipFile(io.BytesIO(data)) as archive:
            infos, seen, expanded = archive.infolist(), set(), 0
            if len(infos) > MAX_WHEEL_FILES:
                _fail("wheel_malformed")
            contents = {}
            for info in infos:
                path = PurePosixPath(info.filename)
                mode = (info.external_attr >> 16) & 0o170000
                expanded += info.file_size
                if (
                    not info.filename
                    or info.filename in seen
                    or info.filename.startswith("/")
                    or "\\" in info.filename
                    or any(part in ("", ".", "..") for part in path.parts)
                    or mode == stat.S_IFLNK
                    or info.file_size > MAX_MEMBER_BYTES
                    or info.compress_size > MAX_MEMBER_BYTES
                    or expanded > MAX_EXPANDED_BYTES
                ):
                    _fail("wheel_malformed")
                seen.add(info.filename)
                contents[info.filename] = archive.read(info)
            metadata = [name for name in seen if name.endswith(".dist-info/METADATA")]
            records = [name for name in seen if name.endswith(".dist-info/RECORD")]
            if len(metadata) != 1 or len(records) != 1 or records[0] != metadata[0][:-8] + "RECORD":
                _fail("wheel_malformed")
            headers = BytesParser().parsebytes(contents[metadata[0]])
            if (
                normalize_distribution(headers.get("Name", "")) != pin["name"]
                or headers.get("Version") != pin["version"]
            ):
                _fail("wheel_metadata")
            rows = list(csv.reader(contents[records[0]].decode().splitlines()))
            index = {row[0]: row[1:] for row in rows if len(row) == 3}
            if len(index) != len(rows) or set(index) != seen:
                _fail("wheel_record")
            for name, content in contents.items():
                digest, size = index[name]
                if name == records[0]:
                    if digest or size:
                        _fail("wheel_record")
                elif digest != _digest(content) or size != str(len(content)):
                    _fail("wheel_record")
    except BootstrapError:
        raise
    except (OSError, ValueError, UnicodeError, KeyError, zipfile.BadZipFile):
        _fail("wheel_malformed")


def _run(args: list[str], timeout: int, output: bool = False) -> bytes:
    process = None
    selector = None
    try:
        process = subprocess.Popen(
            args,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE if output else subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env={
                "PATH": os.defpath,
                "PYTHONNOUSERSITE": "1",
                "PIP_CONFIG_FILE": os.devnull,
                "PIP_DISABLE_PIP_VERSION_CHECK": "1",
                "PIP_NO_INDEX": "1",
            },
        )
        raw, deadline = bytearray(), time.monotonic() + timeout
        if output:
            selector = selectors.DefaultSelector()
            selector.register(process.stdout, selectors.EVENT_READ)
            while process.poll() is None:
                if len(raw) >= MAX_CHILD_OUTPUT:
                    _fail("install_failed")
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired(args, timeout)
                for key, _mask in selector.select(min(remaining, 0.2)):
                    capacity = MAX_CHILD_OUTPUT - len(raw)
                    if capacity <= 0:
                        _fail("install_failed")
                    chunk = os.read(key.fileobj.fileno(), min(65536, capacity))
                    if chunk:
                        raw.extend(chunk)
            capacity = MAX_CHILD_OUTPUT - len(raw)
            if capacity <= 0:
                _fail("install_failed")
            raw.extend(process.stdout.read(capacity))
        else:
            process.wait(timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        _fail("install_failed")
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        if selector is not None:
            selector.close()
        if process is not None and process.stdout is not None:
            process.stdout.close()
    if process.returncode or len(raw) > MAX_CHILD_OUTPUT:
        _fail("install_failed")
    return bytes(raw)


def _installed(venv: Path, pins: list[dict[str, Any]], profile: str, timeout: int) -> None:
    python = venv / ("Scripts/python.exe" if os.name == "nt" else "bin/python")
    script = r"""import importlib,importlib.metadata as m,json
ds=[]
for d in m.distributions():
 n=d.metadata.get("Name")
 if n:
  es=[]
  for e in d.entry_points:
   if e.group in ("flyto.modules","flyto.device_executors"): e.load()
   es.append([e.group,e.name,e.value])
  ds.append([n,d.version,str(d._path),es])
mods={}
for d in m.distributions():
 n=d.metadata.get("Name")
 if n and n.lower().replace("_","-") in WANT:
  top=d.read_text("top_level.txt") or ""
  for x in top.splitlines():
   if x and x.isidentifier(): importlib.import_module(x); mods[x]=n
print(json.dumps({"d":ds,"m":mods},separators=(",",":")))"""
    script = "WANT=" + repr([p["name"] for p in pins]) + "\n" + script
    try:
        report = json.loads(_run([str(python), "-I", "-c", script], timeout, True))
    except (ValueError, UnicodeError):
        _fail("installed_invalid")
    expected = {p["name"]: p["version"] for p in pins}
    owned = {}
    for name, version, root, entries in report.get("d", []):
        normalized = normalize_distribution(name)
        if normalized in expected:
            if normalized in owned or version != expected[normalized]:
                _fail("installed_invalid")
            owned[normalized] = (Path(root), entries)
    if set(owned) != set(expected):
        _fail("installed_invalid")
    imported_owners = {normalize_distribution(owner) for owner in report.get("m", {}).values()}
    if imported_owners != set(expected):
        _fail("installed_invalid")
    actual_profile_entries = {}
    canonical_venv = Path(os.path.realpath(venv))
    for owner, (dist_info, entries) in owned.items():
        # macOS exposes /tmp through the canonical /private/tmp spelling.  The
        # child interpreter reports the latter even when the release was
        # created through the former, so compare canonical filesystem paths.
        # Wheel and manifest bytes remain authorized separately by the
        # O_NOFOLLOW/fstat reads in _read_fd_once.
        if not Path(os.path.realpath(dist_info)).is_relative_to(canonical_venv):
            _fail("installed_invalid")
        _validate_installed_record(venv, dist_info)
        for group, name, value in entries:
            if group in ("flyto.modules", "flyto.device_executors"):
                key = (group, name, value)
                if key in actual_profile_entries:
                    _fail("installed_invalid")
                actual_profile_entries[key] = owner
    # No unpinned distribution may own a Flyto profile entry point.
    for name, _version, _root, entries in report.get("d", []):
        if (
            any(group.startswith("flyto.") for group, _n, _v in entries)
            and normalize_distribution(name) not in expected
        ):
            _fail("installed_invalid")
    if actual_profile_entries != _ENTRY_POINTS[profile]:
        _fail("installed_invalid")


def _validate_installed_record(venv: Path, dist_info: Path) -> None:
    try:
        rows = list(csv.reader((dist_info / "RECORD").read_text(encoding="utf-8").splitlines()))
        seen = set()
        for row in rows:
            if len(row) != 3 or row[0] in seen:
                _fail("installed_invalid")
            seen.add(row[0])
            target = dist_info.parent / row[0]
            if not target.resolve().is_relative_to(venv.resolve()) or not target.is_file():
                _fail("installed_invalid")
            if row[1] and (
                _digest(target.read_bytes()) != row[1] or str(target.stat().st_size) != row[2]
            ):
                _fail("installed_invalid")
    except BootstrapError:
        raise
    except (OSError, UnicodeError):
        _fail("installed_invalid")


def _tree(root: Path) -> list[dict[str, Any]]:
    result = []
    for path in sorted(root.rglob("*")):
        relative = path.relative_to(root).as_posix()
        info = path.lstat()
        mode = stat.S_IMODE(info.st_mode)
        if stat.S_ISLNK(info.st_mode):
            target = os.readlink(path)
            if os.path.isabs(target) or not (path.parent / target).resolve().is_relative_to(
                root.resolve()
            ):
                _fail("publish_failed")
            result.append({"path": relative, "type": "symlink", "target": target})
        elif stat.S_ISDIR(info.st_mode):
            result.append({"path": relative, "type": "dir", "mode": mode})
        elif stat.S_ISREG(info.st_mode):
            result.append(
                {
                    "path": relative,
                    "type": "file",
                    "mode": mode,
                    "size": info.st_size,
                    "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                }
            )
        else:
            _fail("publish_failed")
    return result


def _fsync_tree(root: Path) -> None:
    for path in root.rglob("*"):
        if path.is_file() and not path.is_symlink():
            with path.open("rb") as stream:
                os.fsync(stream.fileno())
    for path in sorted(
        (p for p in root.rglob("*") if p.is_dir()), key=lambda p: len(p.parts), reverse=True
    ):
        fd = os.open(path, os.O_RDONLY)
        os.fsync(fd)
        os.close(fd)
    fd = os.open(root, os.O_RDONLY)
    os.fsync(fd)
    os.close(fd)


def _remove_private_tree(root: Path) -> None:
    """Make an unpublished private tree owner-writable, then remove it completely."""
    try:
        for path in sorted(root.rglob("*"), key=lambda item: len(item.parts), reverse=True):
            if not path.is_symlink():
                os.chmod(path, 0o700 if path.is_dir() else 0o600)
        os.chmod(root, 0o700)
        shutil.rmtree(root)
    except OSError:
        _fail("cleanup_failed")


def _existing(release: Path, inventory: bytes) -> bool:
    try:
        if release.is_symlink() or not release.is_dir():
            return False
        if (release / "inventory.json").read_bytes() != inventory + b"\n":
            return False
        completion = json.loads((release / "complete.json").read_bytes())
        tree = _tree(release)
        tree = [item for item in tree if item["path"] != "complete.json"]
        return completion == {"schema_version": 1, "tree": tree}
    except (OSError, ValueError, UnicodeError, BootstrapError):
        return False


def _validate_release(
    release: Path, inventory: bytes, pins: list[dict[str, Any]], profile: str, timeout: int
) -> bool:
    if not _existing(release, inventory):
        return False
    try:
        _installed(release / "venv", pins, profile, timeout)
        return True
    except BootstrapError:
        return False


def bootstrap_release(
    manifest, wheel_dir, releases_dir, python, profile, *, timeout=INSTALL_TIMEOUT
) -> Path:
    pins = _pins(_manifest(manifest), profile)
    source, destination, interpreter = Path(wheel_dir), Path(releases_dir), Path(python)
    if not isinstance(timeout, int) or isinstance(timeout, bool) or not 1 <= timeout <= 600:
        _fail("manifest_invalid")
    try:
        if (
            source.is_symlink()
            or not source.is_dir()
            or destination.is_symlink()
            or not interpreter.is_file()
        ):
            _fail("unsafe_path")
        if {p.name for p in source.iterdir()} != {p["wheel"] for p in pins}:
            _fail("wheel_set")
    except OSError:
        _fail("unsafe_path")
    blobs = {}
    for pin in pins:
        blobs[pin["wheel"]] = _read_fd_once(
            source / pin["wheel"], MAX_WHEEL_BYTES, "wheel_tampered"
        )
        _validate_wheel(blobs[pin["wheel"]], pin)
    inventory = json.dumps(
        {"schema_version": 1, "profile": profile, "packages": pins},
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    # The canonical inventory is the JSON byte string above.  The newline is a
    # storage delimiter, not part of the release identity.
    inventory_file = inventory + b"\n"
    release = destination / hashlib.sha256(inventory).hexdigest()
    lock = destination / (".building-" + release.name)
    owner = False
    created_release = False
    published = False
    try:
        destination.mkdir(0o700, parents=True, exist_ok=True)
        os.chmod(destination, 0o700)
        try:
            lock.mkdir(0o700)
            owner = True
        except FileExistsError:
            deadline = time.monotonic() + timeout
            while lock.exists() and time.monotonic() < deadline:
                time.sleep(0.02)
            if _validate_release(release, inventory, pins, profile, timeout):
                return release
            _fail("release_tampered")
        if release.exists() or release.is_symlink():
            if _validate_release(release, inventory, pins, profile, timeout):
                return release
            _fail("release_tampered")
        release.mkdir(0o700)
        created_release = True
        cache = release / "wheels"
        cache.mkdir(0o700)
        for name, data in blobs.items():
            fd = os.open(cache / name, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o400)
            try:
                os.write(fd, data)
                os.fsync(fd)
            finally:
                os.close(fd)
        venv = release / "venv"
        _run([str(interpreter), "-I", "-m", "venv", "--copies", str(venv)], timeout)
        pip = venv / ("Scripts/pip.exe" if os.name == "nt" else "bin/pip")
        _run(
            [
                str(pip),
                "install",
                "--isolated",
                "--no-index",
                "--no-deps",
                *[str(cache / p["wheel"]) for p in pins],
            ],
            timeout,
        )
        _installed(venv, pins, profile, timeout)
        shutil.rmtree(cache)
        (release / "inventory.json").write_bytes(inventory_file)
        (release / "inventory.sha256").write_text(
            release.name + "\n", encoding="ascii"
        )
        for root, dirs, files in os.walk(release):
            for name in files:
                item = Path(root) / name
                if not item.is_symlink():
                    os.chmod(item, 0o500 if item.stat().st_mode & 0o111 else 0o400)
            for name in dirs:
                item = Path(root) / name
                if not item.is_symlink():
                    os.chmod(item, 0o500)
        tree = _tree(release)
        completion = (
            json.dumps({"schema_version": 1, "tree": tree}, sort_keys=True, separators=(",", ":"))
            + "\n"
        )
        marker = release / ".complete.tmp"
        marker.write_text(completion)
        os.chmod(marker, 0o400)
        with marker.open("rb") as stream:
            os.fsync(stream.fileno())
        os.rename(marker, release / "complete.json")
        os.chmod(release, 0o500)
        _fsync_tree(release)
        fd = os.open(destination, os.O_RDONLY)
        os.fsync(fd)
        os.close(fd)
        published = True
        return release
    except BootstrapError:
        raise
    except (OSError, ValueError):
        _fail("publish_failed")
    finally:
        if owner:
            if created_release and not published and release.exists():
                _remove_private_tree(release)
            with contextlib.suppress(OSError):
                lock.rmdir()
