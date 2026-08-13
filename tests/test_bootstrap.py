"""Focused contract checks for the immutable offline bootstrap."""

from __future__ import annotations

import base64
import csv
import hashlib
import io
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import zipfile
from pathlib import Path

import pytest

import flyto_robotics.bootstrap as bootstrap_module
from flyto_robotics.bootstrap import BootstrapError, bootstrap_release


def _wheel(
    tmp_path: Path,
    name: str,
    version: str,
    module: str,
    entry: bool = False,
    canonical: bool = True,
) -> dict:
    filename = f"{name.replace('-', '_')}-{version}-py3-none-any.whl"
    dist = f"{name.replace('-', '_')}-{version}.dist-info"
    files = {
        f"{module}/__init__.py": (
            f"VERSION = {version!r}\ndef main():\n print(VERSION)\ndef register_all(): pass\n"
        ).encode(),
        f"{dist}/METADATA": f"Metadata-Version: 2.1\nName: {name}\nVersion: {version}\n".encode(),
        f"{dist}/WHEEL": (
            b"Wheel-Version: 1.0\nGenerator: tests\nRoot-Is-Purelib: true\nTag: py3-none-any\n"
        ),
        f"{dist}/top_level.txt": f"{module}\n".encode(),
    }
    if entry:
        files[f"{dist}/entry_points.txt"] = (
            f"[flyto.test]\n{name} = {module}:VERSION\n"
            f"[console_scripts]\n{name}-probe = {module}:main\n"
        ).encode()
    elif name == "flyto-modules-robotics" and canonical:
        files[f"{dist}/entry_points.txt"] = (
            b"[flyto.modules]\nrobotics = flyto_modules_robotics:register_all\n"
        )
    elif name == "flyto-modules-vision" and canonical:
        files[f"{module}/device_executor.py"] = b"def executor(): pass\n"
        files[f"{dist}/entry_points.txt"] = (
            b"[flyto.modules]\nvision = flyto_modules_vision:register_all\n"
            b"[flyto.device_executors]\n"
            b"vision = flyto_modules_vision.device_executor:executor\n"
        )
    rows = []
    for path, data in files.items():
        digest = base64.urlsafe_b64encode(hashlib.sha256(data).digest()).rstrip(b"=").decode()
        rows.append((path, f"sha256={digest}", str(len(data))))
    rows.append((f"{dist}/RECORD", "", ""))
    output = io.StringIO(newline="")
    csv.writer(output, lineterminator="\n").writerows(rows)
    files[f"{dist}/RECORD"] = output.getvalue().encode()
    path = tmp_path / filename
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as archive:
        for member, data in files.items():
            archive.writestr(member, data)
    raw = path.read_bytes()
    return {
        "name": name,
        "version": version,
        "wheel": filename,
        "size": len(raw),
        "sha256": hashlib.sha256(raw).hexdigest(),
    }


def _manifest(tmp_path: Path, profile: str, versions=("1.2", "7.0")) -> dict:
    companion = "flyto-modules-vision" if profile == "camera-host" else "flyto-modules-robotics"
    return {
        "schema_version": 1,
        "profile": profile,
        "packages": [
            _wheel(tmp_path, "flyto-robotics", versions[0], "flyto_robotics_fixture", True),
            _wheel(tmp_path, companion, versions[1], companion.replace("-", "_")),
        ],
    }


@pytest.mark.parametrize("profile", ["generic", "ros2", "camera-host"])
def test_each_closed_profile_installs_offline(tmp_path: Path, profile: str):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    release = bootstrap_release(
        _manifest(wheels, profile), wheels, tmp_path / "releases", sys.executable, profile
    )
    python = release / "venv/bin/python"
    assert python.exists()
    assert (release / "inventory.sha256").is_file()
    companion_module = (
        "flyto_modules_vision" if profile == "camera-host" else "flyto_modules_robotics"
    )
    expected_entries = (
        "assert any(e.name == 'vision' for e in m.entry_points(group='flyto.modules')); "
        "assert any(e.name == 'vision' for e in m.entry_points(group='flyto.device_executors'))"
        if profile == "camera-host"
        else "assert any(e.name == 'robotics' for e in m.entry_points(group='flyto.modules'))"
    )
    probe = subprocess.run(
        [
            str(python),
            "-I",
            "-c",
            (
                "import importlib.metadata as m, flyto_robotics_fixture as f; "
                f"import {companion_module} as c; "
                f"assert f.VERSION == '1.2' and c.VERSION == '7.0'; "
                "assert any(e.name == 'flyto-robotics' "
                "for e in m.entry_points(group='flyto.test')); " + expected_entries
            ),
        ],
        cwd=tmp_path,
        env={"PATH": ""},
        capture_output=True,
        timeout=10,
    )
    assert probe.returncode == 0, probe.stderr.decode(errors="replace")
    console = subprocess.run(
        [str(release / "venv/bin/flyto-robotics-probe")],
        cwd=tmp_path,
        env={"PATH": str(release / "venv/bin")},
        capture_output=True,
        timeout=10,
    )
    assert console.returncode == 0 and console.stdout.strip() == b"1.2"


@pytest.fixture(scope="module")
def real_workspace_wheels():
    robotics = Path(__file__).resolve().parents[1]
    repositories = {
        "flyto-robotics": (robotics, "0.1.0"),
        "flyto-modules-robotics": (robotics.parent / "flyto-modules-robotics", "0.1.1"),
        "flyto-modules-vision": (robotics.parent / "flyto-modules-vision", "0.1.0"),
    }
    missing = [str(repo) for repo, _version in repositories.values() if not repo.is_dir()]
    if missing:
        pytest.skip("neighboring workspace repositories are absent: " + ", ".join(missing))
    with tempfile.TemporaryDirectory(prefix="flyto-bootstrap-real-", dir="/tmp") as raw_root:
        root = Path(raw_root)
        built = root / "built"
        built.mkdir()
        environment = {
            "PATH": os.defpath,
            "PYTHONNOUSERSITE": "1",
            "PIP_CONFIG_FILE": os.devnull,
            "PIP_DISABLE_PIP_VERSION_CHECK": "1",
            "PIP_NO_INDEX": "1",
        }
        wheels = {}
        for name, (repository, version) in repositories.items():
            clean_source = root / "sources" / name
            clean_source.mkdir(parents=True)
            listing = subprocess.run(
                [
                    "git",
                    "-C",
                    str(repository),
                    "ls-files",
                    "-z",
                    "--cached",
                    "--others",
                    "--exclude-standard",
                ],
                capture_output=True,
                timeout=30,
            )
            assert listing.returncode == 0, listing.stderr.decode(errors="replace")
            for encoded in listing.stdout.split(b"\0"):
                if not encoded:
                    continue
                relative = Path(os.fsdecode(encoded))
                source_file = repository / relative
                if source_file.is_file():
                    target_file = clean_source / relative
                    target_file.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copy2(source_file, target_file)
            result = subprocess.run(
                [
                    sys.executable,
                    "-I",
                    "-m",
                    "pip",
                    "wheel",
                    "--no-index",
                    "--no-deps",
                    "--no-build-isolation",
                    "--wheel-dir",
                    str(built),
                    str(clean_source),
                ],
                cwd=root,
                env=environment,
                capture_output=True,
                timeout=120,
            )
            assert result.returncode == 0, result.stderr.decode(errors="replace")
            matches = list(built.glob(f"{name.replace('-', '_')}-{version}-*.whl"))
            assert len(matches) == 1
            wheels[name] = (matches[0], version)
        yield root, wheels


@pytest.mark.skipif(sys.platform != "darwin", reason="macOS /tmp is a canonical path alias")
@pytest.mark.parametrize("profile", ["generic", "ros2", "camera-host"])
def test_real_clean_offline_install_through_macos_tmp_alias(real_workspace_wheels, profile):
    root, built = real_workspace_wheels
    assert str(root).startswith("/tmp/")
    assert os.path.realpath(root).startswith("/private/tmp/")
    companion = "flyto-modules-vision" if profile == "camera-host" else "flyto-modules-robotics"
    wheels = root / f"{profile}-wheels"
    wheels.mkdir()
    pins = []
    for name in ("flyto-robotics", companion):
        source, version = built[name]
        target = wheels / source.name
        shutil.copy2(source, target)
        raw = target.read_bytes()
        pins.append(
            {
                "name": name,
                "version": version,
                "wheel": target.name,
                "size": len(raw),
                "sha256": hashlib.sha256(raw).hexdigest(),
            }
        )
    manifest = {"schema_version": 1, "profile": profile, "packages": pins}
    releases = root / f"{profile}-releases"
    release = bootstrap_release(manifest, wheels, releases, sys.executable, profile)
    expected_entries = (
        {
            ("flyto.modules", "vision", "flyto_modules_vision:register_all"),
            (
                "flyto.device_executors",
                "vision",
                "flyto_modules_vision.device_executor:executor",
            ),
        }
        if profile == "camera-host"
        else {("flyto.modules", "robotics", "flyto_modules_robotics:register_all")}
    )
    probe = subprocess.run(
        [
            str(release / "venv/bin/python"),
            "-I",
            "-c",
            (
                "import importlib,importlib.metadata as m,json;"
                "names=['flyto-robotics'," + repr(companion) + "];"
                "[importlib.import_module(x) for x in ['flyto_robotics',"
                + repr(companion.replace("-", "_"))
                + "]];"
                "eps={(e.group,e.name,e.value) for d in m.distributions() for e in d.entry_points "
                "if e.group in ('flyto.modules','flyto.device_executors')};"
                "job=[e.value for e in m.entry_points(group='console_scripts') "
                "if e.name=='flyto-job-runner'];"
                "print(json.dumps({'versions':{n:m.version(n) for n in names},"
                "'entries':sorted(eps),'job':job}))"
            ),
        ],
        cwd=root,
        env={"PATH": ""},
        capture_output=True,
        timeout=10,
    )
    assert probe.returncode == 0, probe.stderr.decode(errors="replace")
    report = bootstrap_module.json.loads(probe.stdout)
    assert report["versions"] == {"flyto-robotics": "0.1.0", companion: built[companion][1]}
    assert {tuple(entry) for entry in report["entries"]} == expected_entries
    assert report["job"] == ["deploy:job_runner_main"]
    assert (release / "venv/bin/flyto-job-runner").is_file()
    assert (release / "inventory.json").is_file()
    assert (release / "inventory.sha256").is_file()
    assert (release / "complete.json").is_file()
    assert bootstrap_release(manifest, wheels, releases, sys.executable, profile) == release


def test_tamper_and_matrix_refused(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    manifest["packages"][1]["name"] = "flyto-modules-vision"
    with pytest.raises(BootstrapError, match="package_matrix"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_wheel_digest_tamper_refused(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    (wheels / manifest["packages"][0]["wheel"]).write_bytes(b"not a wheel")
    with pytest.raises(BootstrapError, match="wheel_tampered"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_independent_version_pin_must_match_wheel(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    manifest["packages"][0]["version"] = "99"
    with pytest.raises(BootstrapError, match="wheel_metadata"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_incomplete_record_refused(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    pin = manifest["packages"][0]
    path = wheels / pin["wheel"]
    members = {}
    with zipfile.ZipFile(path) as archive:
        for name in archive.namelist():
            members[name] = archive.read(name)
    record = next(name for name in members if name.endswith(".dist-info/RECORD"))
    members[record] = b""
    with zipfile.ZipFile(path, "w") as archive:
        for name, data in members.items():
            archive.writestr(name, data)
    raw = path.read_bytes()
    pin.update(size=len(raw), sha256=hashlib.sha256(raw).hexdigest())
    with pytest.raises(BootstrapError, match="wheel_record"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_failed_install_removes_staging_and_preserves_release(tmp_path: Path):
    releases = tmp_path / "releases"
    good_wheels = tmp_path / "good"
    bad_wheels = tmp_path / "bad"
    good_wheels.mkdir()
    bad_wheels.mkdir()
    prior = bootstrap_release(
        _manifest(good_wheels, "generic"), good_wheels, releases, sys.executable, "generic"
    )
    fake_python = tmp_path / "python"
    fake_python.write_text("#!/bin/sh\nexit 9\n")
    fake_python.chmod(0o700)
    with pytest.raises(BootstrapError, match="install_failed"):
        bootstrap_release(
            _manifest(bad_wheels, "generic", ("8", "9")),
            bad_wheels,
            releases,
            fake_python,
            "generic",
        )
    assert prior.exists()
    assert not list(releases.glob(".staging-*"))
    assert not list(releases.glob(".building-*"))


def test_two_releases_are_non_overwriting(tmp_path: Path):
    releases = tmp_path / "releases"
    first_wheels = tmp_path / "one"
    second_wheels = tmp_path / "two"
    first_wheels.mkdir()
    second_wheels.mkdir()
    first = bootstrap_release(
        _manifest(first_wheels, "generic", ("1", "2")),
        first_wheels,
        releases,
        sys.executable,
        "generic",
    )
    second = bootstrap_release(
        _manifest(second_wheels, "generic", ("3", "4")),
        second_wheels,
        releases,
        sys.executable,
        "generic",
    )
    assert first != second
    assert first.exists() and second.exists()
    for release, version in ((first, b"1"), (second, b"3")):
        result = subprocess.run(
            [str(release / "venv/bin/flyto-robotics-probe")],
            env={"PATH": str(release / "venv/bin")},
            capture_output=True,
            timeout=10,
        )
        assert result.returncode == 0 and result.stdout.strip() == version


def test_identical_release_is_revalidated_and_idempotent(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    first = bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
    assert (
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
        == first
    )
    target = next((first / "venv").rglob("flyto_robotics_fixture/__init__.py"))
    target.chmod(0o600)
    target.write_text("VERSION='tampered'\n")
    with pytest.raises(BootstrapError, match="release_tampered"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_oversized_pin_refused_before_read(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    manifest["packages"][0]["size"] = 128 * 1024 * 1024 + 1
    with pytest.raises(BootstrapError, match="manifest_invalid"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_source_swap_after_bounded_read_cannot_change_install(tmp_path: Path, monkeypatch):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    original = bootstrap_module._read_fd_once
    swapped = False

    def read_then_swap(path, limit, code):
        nonlocal swapped
        data = original(path, limit, code)
        if path.suffix == ".whl" and not swapped:
            path.write_bytes(b"swapped after read")
            swapped = True
        return data

    monkeypatch.setattr(bootstrap_module, "_read_fd_once", read_then_swap)
    release = bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
    result = subprocess.run(
        [str(release / "venv/bin/flyto-robotics-probe")], capture_output=True, timeout=10
    )
    assert result.returncode == 0 and result.stdout.strip() == b"1.2"


def test_post_install_record_tamper_refused_and_cleaned(tmp_path: Path, monkeypatch):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    original = bootstrap_module._installed

    def tamper_then_validate(venv, pins, profile, timeout):
        target = next(venv.rglob("flyto_robotics_fixture/__init__.py"))
        target.write_text("VERSION='tampered'\n")
        original(venv, pins, profile, timeout)

    monkeypatch.setattr(bootstrap_module, "_installed", tamper_then_validate)
    with pytest.raises(BootstrapError, match="installed_invalid"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
    assert not list((tmp_path / "releases").glob(".building-*"))


def test_unpinned_profile_entry_point_owner_refused(tmp_path: Path, monkeypatch):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    original = bootstrap_module._installed

    def add_owner_then_validate(venv, pins, profile, timeout):
        site = next(venv.rglob("site-packages"))
        info = site / "foreign_owner-1.dist-info"
        info.mkdir()
        (info / "METADATA").write_text("Metadata-Version: 2.1\nName: foreign-owner\nVersion: 1\n")
        (info / "entry_points.txt").write_text("[flyto.test]\nforeign = json:loads\n")
        (info / "RECORD").write_text("")
        original(venv, pins, profile, timeout)

    monkeypatch.setattr(bootstrap_module, "_installed", add_owner_then_validate)
    with pytest.raises(BootstrapError, match="installed_invalid"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_required_companion_owner_is_not_optional(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = {
        "schema_version": 1,
        "profile": "generic",
        "packages": [
            _wheel(wheels, "flyto-robotics", "1", "flyto_robotics_fixture", True),
            _wheel(
                wheels, "flyto-modules-robotics", "2", "flyto_modules_robotics", canonical=False
            ),
        ],
    }
    with pytest.raises(BootstrapError, match="installed_invalid"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_concurrent_same_release_validates_winner(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    releases = tmp_path / "releases"
    results, errors = [], []

    def build():
        try:
            results.append(bootstrap_release(manifest, wheels, releases, sys.executable, "generic"))
        except Exception as exc:
            errors.append(exc)

    threads = [threading.Thread(target=build) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join(120)
    assert not errors and len(results) == 2 and results[0] == results[1]
    assert not list(releases.glob(".building-*"))


def test_completion_records_modes_and_rejects_outside_symlink(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    release = bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
    import json

    tree = json.loads((release / "complete.json").read_text())["tree"]
    assert all("mode" in item for item in tree if item["type"] in ("file", "dir"))
    link = release / "outside"
    release.chmod(0o700)
    link.symlink_to("/tmp")
    release.chmod(0o500)
    with pytest.raises(BootstrapError, match="release_tampered"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")


def test_failed_completion_restores_permissions_and_removes_release(tmp_path: Path, monkeypatch):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")

    def fail_fsync(_root):
        raise OSError("synthetic durable publication failure")

    monkeypatch.setattr(bootstrap_module, "_fsync_tree", fail_fsync)
    with pytest.raises(BootstrapError, match="publish_failed"):
        bootstrap_release(manifest, wheels, tmp_path / "releases", sys.executable, "generic")
    releases = tmp_path / "releases"
    assert not [path for path in releases.iterdir() if not path.name.startswith(".building-")]
    assert not list(releases.glob(".building-*"))


def test_preexisting_incomplete_release_is_preserved_and_refused(tmp_path: Path):
    wheels = tmp_path / "wheels"
    wheels.mkdir()
    manifest = _manifest(wheels, "generic")
    pins = bootstrap_module._pins(bootstrap_module._manifest(manifest), "generic")
    inventory = bootstrap_module.json.dumps(
        {"schema_version": 1, "profile": "generic", "packages": pins},
        sort_keys=True,
        separators=(",", ":"),
    ).encode()
    releases = tmp_path / "releases"
    incomplete = releases / hashlib.sha256(inventory).hexdigest()
    incomplete.mkdir(parents=True)
    marker = incomplete / "owner-data"
    marker.write_text("preserve")
    with pytest.raises(BootstrapError, match="release_tampered"):
        bootstrap_release(manifest, wheels, releases, sys.executable, "generic")
    assert marker.read_text() == "preserve"


def test_child_output_limit_terminates_before_a_zero_length_read():
    script = (
        "import sys,time;"
        f"sys.stdout.buffer.write(b'x'*{bootstrap_module.MAX_CHILD_OUTPUT});"
        "sys.stdout.flush();time.sleep(30)"
    )
    started = bootstrap_module.time.monotonic()
    with pytest.raises(BootstrapError, match="install_failed"):
        bootstrap_module._run([sys.executable, "-I", "-c", script], 5, output=True)
    assert bootstrap_module.time.monotonic() - started < 5
