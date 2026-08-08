"""The offline path where the device secret never reaches a disk.

Three files have to agree on one string: the drop-in unit names a credential,
the provisioning script encrypts under that name, and the runner reads it. If
any of them drifts the robot does not fail — it quietly falls back to the file
on disk and keeps working, which is the whole failure mode this repository
spent a day removing. So the agreement is asserted rather than assumed.
"""

from __future__ import annotations

import importlib.util
import os
import re
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
DROP_IN = ROOT / "deploy/systemd/flyto-job-runner.service.d/enterprise-credential.conf"
BASE_UNIT = ROOT / "deploy/systemd/flyto-job-runner.service"
SCRIPT = ROOT / "scripts/provision-device-credential.sh"
RUNNER = ROOT / "deploy/flyto_job_runner.py"


def runner_module():
    spec = importlib.util.spec_from_file_location("flyto_job_runner_enterprise", RUNNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def directive(text: str, name: str) -> str | None:
    match = re.search(rf"^{name}=(.+)$", text, re.M)
    return match.group(1).strip() if match else None


class TestTheThreeFilesAgree:
    def test_the_unit_loads_the_name_the_runner_reads(self):
        loaded = directive(DROP_IN.read_text(), "LoadCredentialEncrypted")
        assert loaded, "the drop-in must load a credential"
        name = loaded.split(":", 1)[0]
        assert name == runner_module().SYSTEMD_CREDENTIAL_NAME, (
            "the unit and the runner name different credentials, so the runner "
            "would find nothing and silently use the on-disk file instead"
        )

    def test_the_provisioning_script_encrypts_under_that_same_name(self):
        assert (
            directive(SCRIPT.read_text(), "CREDENTIAL_NAME")
            == runner_module().SYSTEMD_CREDENTIAL_NAME
        )

    def test_the_script_writes_where_the_unit_looks(self):
        loaded = directive(DROP_IN.read_text(), "LoadCredentialEncrypted")
        _, path = loaded.split(":", 1)
        assert directive(SCRIPT.read_text(), "OUTPUT") == path.strip()


class TestTheDropInIsADropInAndNotAFork:
    def test_it_lives_in_a_service_d_directory(self):
        assert DROP_IN.parent.name == "flyto-job-runner.service.d"
        assert DROP_IN.suffix == ".conf"

    def test_it_does_not_restate_the_base_unit(self):
        """A copied ExecStart is a copy that stops receiving fixes.

        This repository has already paid for one: a second runner beside the
        deployed tree kept executing after every deploy and cost a live
        mission.
        """
        text = DROP_IN.read_text()
        for directive_name in ("ExecStart", "User", "Restart", "Type"):
            assert directive(text, directive_name) is None, (
                f"{directive_name} belongs to the base unit; restating it here "
                "creates a second copy that will drift"
            )

    def test_the_base_unit_is_the_one_that_carries_the_hardening(self):
        text = BASE_UNIT.read_text()
        assert directive(text, "UMask") == "0077"
        assert directive(text, "NoNewPrivileges") == "yes"


class TestTheProvisioningScript:
    def test_it_is_executable(self):
        assert os.access(SCRIPT, os.X_OK)

    def test_it_is_valid_shell(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True)

    def test_it_refuses_a_secret_on_the_command_line(self):
        """argv is world readable through ps for as long as the process lives."""
        text = SCRIPT.read_text()
        assert "PLAINTEXT=\"$(cat)\"" in text, "the credential must arrive on stdin"
        assert "--help" in text and "visible in ps" in text, (
            "the reason must be stated where someone would otherwise add a flag"
        )

    def test_it_stops_before_writing_when_the_credential_is_malformed(self):
        """Encrypting a broken credential succeeds and fails at boot instead."""
        text = SCRIPT.read_text()
        assert "nothing was written" in text

    def test_it_verifies_the_round_trip_before_replacing_anything(self):
        text = SCRIPT.read_text()
        assert "systemd-creds decrypt" in text
        assert "did not decrypt back" in text

    def test_it_never_prints_the_secret_even_when_comparing(self):
        text = SCRIPT.read_text()
        assert "sha256sum" in text, "compare through a hash, not by echoing values"

    def test_it_reports_what_the_key_is_actually_bound_to(self):
        """An operator who believes they have TPM sealing and do not has the
        wrong threat model, and will make decisions on it."""
        text = SCRIPT.read_text()
        assert "/dev/tpmrm0" in text
        assert "NOT protection" in text, "the weaker case must say so plainly"


class TestTheRunnerPrefersItWithoutBeingTold:
    def test_the_environment_variable_is_the_systemd_one(self):
        """$CREDENTIALS_DIRECTORY is set by systemd itself; inventing our own
        name would mean the unit and the code had to agree twice."""
        assert "CREDENTIALS_DIRECTORY" in RUNNER.read_text()

    def test_reading_it_needs_no_configuration(self, tmp_path, monkeypatch):
        """No flag, no environment switch: if systemd supplied one, use it.

        A deployment that had to opt in would eventually be one that forgot to,
        and the symptom would be a secret quietly back on disk.
        """
        monkeypatch.setenv("FLYTO_RUNNER_DATA_DIR", str(tmp_path / "data"))
        module = runner_module()
        directory = tmp_path / "creds"
        directory.mkdir()
        (directory / module.SYSTEMD_CREDENTIAL_NAME).write_text(
            '{"device_id": "d", "device_secret": "s"}'
        )
        monkeypatch.setenv("CREDENTIALS_DIRECTORY", str(directory))
        assert module._read_credentials() == {"device_id": "d", "device_secret": "s"}
        assert not module.CREDENTIAL_FILE.exists()


@pytest.mark.skipif(
    subprocess.run(["which", "systemd-creds"], capture_output=True).returncode != 0,
    reason="systemd-creds is not on this host; the round trip is covered on the robot",
)
def test_the_script_round_trips_on_a_host_that_has_systemd_creds(tmp_path):
    output = tmp_path / "device.cred"
    result = subprocess.run(
        ["bash", str(SCRIPT), "--output", str(output)],
        input='{"device_id": "d", "device_secret": "s"}',
        capture_output=True,
        text=True,
    )
    if result.returncode != 0 and "must run as root" in result.stderr:
        pytest.skip("provisioning needs root")
    assert result.returncode == 0, result.stderr
    assert output.stat().st_mode & 0o777 == 0o600


class TestItDoesNotTouchDirectoriesItDidNotMake:
    """Found by running it: `install -d -m 0700 "$(dirname "$OUTPUT")"` with
    --output under /tmp set /tmp itself to 0700 root on a live robot, locking
    every other user out. Creating a directory is this script's business;
    re-permissioning an existing one is not."""

    def code(self) -> str:
        """The script with comment lines dropped.

        The comment explains the hazard by quoting the line that caused it, and
        a guard that reads its own explanation as a violation would force the
        explanation out — losing the one thing that stops someone restoring the
        bug deliberately.
        """
        return "\n".join(
            line
            for line in SCRIPT.read_text().splitlines()
            if not line.lstrip().startswith("#")
        )

    def test_it_only_creates_a_directory_that_is_absent(self):
        body = self.code()
        assert 'if [ ! -d "$OUTPUT_DIR" ]; then' in body
        assert body.count("install -d -m 0700") == 1, (
            "an unconditional install -d is what caused this"
        )

    def test_the_hazard_is_recorded_where_someone_would_undo_it(self):
        assert "/tmp itself into 0700 root" in SCRIPT.read_text()

    def test_it_still_creates_the_default_location(self, tmp_path):
        """A fresh /etc/flyto must still come out owner-only."""
        target = tmp_path / "etc" / "flyto"
        result = subprocess.run(
            ["bash", "-c", f'OUTPUT="{target}/device.cred"; '
             'OUTPUT_DIR="$(dirname "$OUTPUT")"; '
             '[ ! -d "$OUTPUT_DIR" ] && install -d -m 0700 "$OUTPUT_DIR"'],
            capture_output=True,
            text=True,
        )
        assert result.returncode == 0, result.stderr
        assert target.stat().st_mode & 0o777 == 0o700

    def test_an_existing_directory_keeps_its_mode(self, tmp_path):
        shared = tmp_path / "shared"
        shared.mkdir(mode=0o1777)
        before = shared.stat().st_mode & 0o7777
        subprocess.run(
            ["bash", "-c", f'OUTPUT="{shared}/device.cred"; '
             'OUTPUT_DIR="$(dirname "$OUTPUT")"; '
             '[ ! -d "$OUTPUT_DIR" ] && install -d -m 0700 "$OUTPUT_DIR"; true'],
            check=True,
        )
        assert shared.stat().st_mode & 0o7777 == before, "a shared directory was re-permissioned"
