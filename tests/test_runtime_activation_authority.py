"""Who gets to say what this device is running.

Three files could answer that question and only one of them is allowed to:

``state.current_activation`` + ``activations/by-id/<id>.json``
    The authority. The state names one activation; the record is immutable and
    content addressed, so "same activation" and "same bytes" are the same
    question. Both are read, and they are made to corroborate each other before
    either is believed.

``activations/<version>.json``
    A *view*, and mutable by design: re-activating one version under a different
    profile legitimately repoints it. It is the newest activation of a name,
    which stops being the committed one the moment a device rolls back to an
    earlier activation of that same name. Accepting it whenever a record happens
    to be absent would let "delete one file" promote it back to authority --
    invisibly, because every digest still verifies. It is simply not *this*
    activation.

The shipped registry
    Not an answer at all. A machine installed under a site profile that has
    since been deleted still runs the units it was installed with.

Every runtime assertion here executes the *exact rendered* ``ExecStart=`` in a
subprocess. A fake systemd marks a unit active without ever running it, which is
how two argparse usage errors shipped green; a test that imports the entry point
and calls it would prove the function works and say nothing about the command
the device actually runs. Nothing below names ROS: the two profiles a switch is
proved across are derived from whatever the shipped registry declares.
"""

from __future__ import annotations

import json
import os
import queue
import shlex
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

from flyto_robotics.fsio import atomic_write
from flyto_robotics.lifecycle import (
    LIFECYCLE_WINDOW_VERSION,
    Layout,
    LifecycleError,
    install,
    open_activation_window,
    rollback,
    runtime_activation,
    status,
)
from flyto_robotics.lifecycle_profiles import default_profiles_path
from flyto_robotics.robot_service import BOOTSTRAP_POLL_SECONDS, BOOTSTRAP_WINDOW_SECONDS
from flyto_robotics.support_bundle import build_support_bundle, write_support_bundle
from flyto_robotics.systemd_control import FakeSystemctl, SystemdController
from flyto_robotics.systemd_units import parse_unit

REPO_ROOT = Path(__file__).resolve().parent.parent

#: The convergence target. The activation transaction restarts this service and
#: commits milliseconds later; anything approaching a second is the supervision
#: interval leaking into a window it does not belong in.
CONVERGE_SECONDS = 1.0

#: The bound used where a test has to watch a window *lapse*. A small number
#: passed to the same writer production uses, rather than a patched constant: the
#: shipped ceiling is untouched and unread by these tests, and what is proved is
#: the real marker-driven path rather than a shortened copy of it. Sleeping the
#: production window instead would put a two-minute wait in the suite.
WINDOW_UNDER_TEST = 2.0

#: How far past the bound a lapse is still allowed to be noticed. Process start,
#: one poll interval, and a loaded CI box; nothing that hides a loop which never
#: transitions at all.
LAPSE_SLACK_SECONDS = 5.0

SECRET = "SUPERSECRET"


@pytest.fixture()
def layout(tmp_path: Path) -> Layout:
    return Layout(root=tmp_path.resolve())


def _fake() -> SystemdController:
    return SystemdController(runner=FakeSystemctl(), dry_run=False, mode="fake")


def _payload(tmp_path: Path, name: str, body: str) -> Path:
    root = tmp_path / "payloads" / name
    package = root / "flyto_robotics"
    package.mkdir(parents=True, exist_ok=True)
    package.joinpath("__init__.py").write_text(body, encoding="utf-8")
    return root


def _provision(layout: Layout) -> None:
    layout.identity_file.write_text('{"device_id": "d-1"}', encoding="utf-8")
    layout.config_file.write_text(
        "FLYTO_ROBOT_RESOURCE_ID=flyto-rover-1\nFLYTO_CLOUD_URL=https://cloud.example\n",
        encoding="utf-8",
    )


def _exec_start(layout: Layout, unit: str = "flyto-robot-agent.service") -> list[str]:
    """The command systemd runs, taken verbatim from the rendered unit."""

    text = (layout.unit_dir / unit).read_text(encoding="utf-8")
    return shlex.split(parse_unit(text).values("Service", "ExecStart")[0])


def _env(**extra: str) -> dict[str, str]:
    # PYTHONPATH supplies the package exactly as the unit's Environment= does.
    return {**os.environ, "PYTHONPATH": str(REPO_ROOT), **extra}


def _one_cycle(argv: list[str]) -> tuple[int, dict, str]:
    """Run the unit's command for exactly one supervision cycle."""

    completed = subprocess.run(
        argv, capture_output=True, text=True, timeout=60, check=False,
        env=_env(FLYTO_ROBOT_MAX_CYCLES="1"),
    )
    lines = [line for line in completed.stdout.splitlines() if line.strip()]
    # One document per cycle, and it is a document -- not a traceback, and not
    # argparse's exit 2 with nothing on stdout at all.
    assert len(lines) == 1, completed.stdout
    return completed.returncode, json.loads(lines[0]), completed.stderr


def _two_profiles(tmp_path: Path) -> tuple[Path, str, str]:
    """A site registry with two profiles, derived rather than named.

    The point of the switch tests is that one *version* can be activated twice
    under different contracts. Which contracts is irrelevant, and hardcoding a
    middleware here would tie a lifecycle test to whichever adapters the product
    happens to ship. So the shipped registry's own first profile is copied under
    two site names; they differ only in what they are called, which is enough --
    the profile name is inside the digest-covered body, so the two activations
    have different ids.
    """

    registry = json.loads(default_profiles_path().read_text(encoding="utf-8"))
    # A custom registry must be self-contained.  Select the transport-neutral
    # root explicitly: the alphabetically first shipped profile is now camera,
    # whose ``extends: generic`` would dangle after the profiles are replaced.
    base = registry["profiles"]["generic"]
    registry["profiles"] = {
        "site-alpha": json.loads(json.dumps(base)),
        "site-beta": json.loads(json.dumps(base)),
    }
    path = tmp_path / "site-profiles.json"
    path.write_text(json.dumps(registry), encoding="utf-8")
    return path, "site-alpha", "site-beta"


class _Agent:
    """A long-running agent, read without ever blocking the test forever.

    ``readline`` on a pipe is the natural way to do this and the wrong one: if
    the loop under test stops emitting -- which is precisely the regression these
    tests exist to catch -- the suite hangs instead of failing.
    """

    def __init__(self, argv: list[str]) -> None:
        self.started = time.monotonic()
        #: How many documents this process has published. A supervisor that polls
        #: correctly emits one per interval; one that spins emits thousands, and
        #: "it converged fast" is not evidence that it did not.
        self.published = 0
        self.process = subprocess.Popen(  # noqa: S603 - the unit's own command, verbatim
            argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=_env(),
        )
        self.lines: queue.Queue[str] = queue.Queue()
        self._reader = threading.Thread(target=self._pump, daemon=True)
        self._reader.start()

    def _pump(self) -> None:
        assert self.process.stdout is not None
        for line in self.process.stdout:
            self.published += 1
            self.lines.put(line)

    def assert_not_spinning(self) -> None:
        """The published rate is bounded by the bootstrap poll, not by the CPU."""

        elapsed = time.monotonic() - self.started
        ceiling = elapsed / BOOTSTRAP_POLL_SECONDS + 4
        assert self.published <= ceiling, (
            f"{self.published} documents in {elapsed:.2f}s is a spin, not a poll"
        )

    def document(self, timeout: float = 30.0) -> dict:
        try:
            return json.loads(self.lines.get(timeout=timeout))
        except queue.Empty:  # pragma: no cover - only on a regression
            raise AssertionError("the agent published no document in time") from None

    def until(self, predicate, *, timeout: float) -> tuple[dict, float]:
        """The first document satisfying ``predicate``, and when it arrived."""

        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:  # pragma: no cover - only on a regression
                raise AssertionError("the agent never converged")
            document = self.document(timeout=remaining)
            if predicate(document):
                return document, time.monotonic()

    @property
    def running(self) -> bool:
        return self.process.poll() is None

    def close(self) -> None:
        self.process.terminate()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:  # pragma: no cover
            self.process.kill()


def _installed(layout: Layout, tmp_path: Path, **kwargs) -> dict:
    return install(
        payload=_payload(tmp_path, kwargs.pop("payload_name", "a"), "v1"),
        version=kwargs.pop("version", "1.0.0"),
        layout=layout,
        python=sys.executable,
        systemd=_fake(),
        **kwargs,
    )


def _record_files(layout: Layout) -> list[Path]:
    return sorted(layout.activation_record_dir.glob("*.json"))


def _committed_id(layout: Layout) -> str:
    return json.loads(layout.state_file.read_text(encoding="utf-8"))["current_activation"]


def _precommit(layout: Layout, *, seconds: float = 60.0, version: str = "1.0.0") -> None:
    """Rewind to the instant systemd first ran ``ExecStart=``.

    Units on disk, the transaction's own window marker open, nothing committed.
    The marker is written by the *shipped* writer rather than hand-rolled here,
    so a test cannot pass against a marker format production does not produce.
    """

    open_activation_window(layout, action="install", version=version, seconds=seconds)
    layout.state_file.unlink()


def _forge_window(layout: Layout, document: dict) -> None:
    """Put arbitrary bytes where the transaction puts its marker.

    Deliberately not the shipped writer: the point of these cases is what an
    attacker with write access to ``/var/lib`` can make a reader believe.
    """

    atomic_write(layout.activation_window_file, json.dumps(document), 0o640)


class TestTheAgentConvergesOnTheCommitItStartedBefore:
    """Requirement: start before the commit, observe it in under a second."""

    def test_a_clean_first_install_converges_without_exiting(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The ordering window every install goes through, at full speed.

        ``_activate_unit_set`` restarts and verifies the agent *before* the
        state write that commits the activation -- that is what makes the whole
        thing a transaction. So systemd's first ExecStart runs against a state
        directory that describes nothing. Exiting there is a flap; sleeping the
        supervision interval there leaves the published document wrong for the
        exact half minute an operator is watching the install.
        """

        report = _installed(layout, tmp_path)
        argv = _exec_start(layout)
        committed = layout.state_file.read_text(encoding="utf-8")
        _precommit(layout)

        agent = _Agent(argv)
        try:
            first = agent.document()
            assert first["state"] == "activation_pending"
            assert first["ok"] is True
            assert first["activation"] == ""

            # Wait inside the window before committing, so the poll that matters
            # is exercised rather than skipped by an instant commit -- and so the
            # published rate below is measured over a real pending interval.
            time.sleep(0.6)
            assert agent.running

            atomic_write(layout.state_file, committed, 0o640)
            committed_at = time.monotonic()

            document, observed_at = agent.until(
                lambda doc: doc["state"] != "activation_pending", timeout=30.0
            )
            assert observed_at - committed_at <= CONVERGE_SECONDS
            assert document["profile"] == report["profile"]
            assert document["activation"] == _committed_id(layout)
            # It never exited to get that fresh look. Exiting is what
            # Restart=on-failure turns into the rate-limited flap.
            assert agent.running
            # And it hurried without burning the CPU to do it.
            agent.assert_not_spinning()
        finally:
            agent.close()

    def test_the_transaction_opens_the_window_and_closes_it_on_commit(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The marker is transaction state, not device state.

        It has to be gone the moment the state write lands -- a marker that
        outlived its transaction would excuse a missing state file that nothing
        is coming to write. And it must not register as a *change*: counting it
        would make every idempotent re-install report ``ok`` where it correctly
        reports ``no_change``, which is the signal a fleet tool uses to tell "I
        did something" from "there was nothing to do".
        """

        report = _installed(layout, tmp_path)

        assert report["ok"] is True
        assert not layout.activation_window_file.exists()

        second = _installed(layout, tmp_path)
        assert second["changed"] == []
        assert second["reason"] == "no_change"
        assert not layout.activation_window_file.exists()

    def test_a_rollback_passes_through_the_same_window_and_leaves_none_behind(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Rollback restarts the same services before its own state write."""

        _installed(layout, tmp_path)
        install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                python=sys.executable, systemd=_fake())

        assert rollback(layout=layout, systemd=_fake())["ok"] is True
        assert not layout.activation_window_file.exists()
        assert status(layout)["recorded_current"] == "1.0.0"

    def test_a_same_version_profile_switch_is_observed_by_the_service_it_restarts(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The first observation is the *outgoing* activation, not `pending`.

        A profile switch restarts this service while a perfectly good state file
        is still on disk naming the activation being left. A loop that only
        hurried while it had nothing at all would settle straight into the
        supervision interval here and publish the old profile for thirty
        seconds -- on the one operation whose entire purpose is to change it.
        """

        profiles, alpha, beta = _two_profiles(tmp_path)
        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, profile=alpha,
                python=sys.executable, profiles=profiles, systemd=_fake())
        outgoing = _committed_id(layout)

        agent = _Agent(_exec_start(layout))
        try:
            first = agent.document()
            assert first["profile"] == alpha
            assert first["activation"] == outgoing

            # The same version, activated again under the other contract.
            install(payload=payload, version="1.0.0", layout=layout, profile=beta,
                    python=sys.executable, profiles=profiles, systemd=_fake())
            committed_at = time.monotonic()
            incoming = _committed_id(layout)
            assert incoming != outgoing

            document, observed_at = agent.until(
                lambda doc: doc["activation"] != outgoing, timeout=30.0
            )
            assert observed_at - committed_at <= CONVERGE_SECONDS
            assert document["activation"] == incoming
            assert document["profile"] == beta
            assert agent.running
        finally:
            agent.close()

    def test_the_hurry_is_bounded_and_is_not_a_spin(self) -> None:
        """Catching up is a floor as well as a ceiling.

        ``--interval-seconds 0`` is a legitimate way to ask a *bounded* run to go
        as fast as it can. Taking the smaller of the two numbers alone would make
        a long-running supervisor spin the CPU flat for the whole bootstrap
        window, on every start, on every device.
        """

        from flyto_robotics.robot_service import BOOTSTRAP_POLL_SECONDS, _delay

        assert _delay(0.0, settled=False) == BOOTSTRAP_POLL_SECONDS
        assert _delay(30.0, settled=False) == BOOTSTRAP_POLL_SECONDS
        assert _delay(0.1, settled=False) == 0.1
        # Settled is exactly what the operator configured; nothing second
        # guesses the supervision interval once there is nothing to catch up on.
        assert _delay(30.0, settled=True) == 30.0
        assert BOOTSTRAP_WINDOW_SECONDS > 0


class TestPendingIsProvedByAWindowAndNeverAssumed:
    """Requirement: a missing state file is only ever *briefly* excusable.

    ``activation_pending`` used to mean "the state file is not there", which is
    an answer with no expiry: a device whose state was deleted, or whose commit
    never arrived, published a cheerful ``ok: true`` document forever and every
    surface downstream -- the doctor snapshot, the support bundle, the console --
    repeated it. The state is now granted only while the durable, bounded marker
    an activation writes before it touches systemd is live.
    """

    def test_a_commit_that_never_arrives_becomes_unhealthy_within_the_window(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The window is a bound, so something has to happen when it is reached."""

        _installed(layout, tmp_path)
        argv = _exec_start(layout)
        _precommit(layout, seconds=WINDOW_UNDER_TEST)
        opened_at = time.monotonic()

        agent = _Agent(argv)
        try:
            first = agent.document()
            assert first["state"] == "activation_pending"
            assert first["ok"] is True

            document, observed_at = agent.until(
                lambda doc: doc["state"] != "activation_pending", timeout=30.0
            )

            assert document["state"] == "unhealthy"
            assert document["ok"] is False
            # The same code `status` publishes for the same device: units and
            # `current` were switched and the state write never landed.
            assert document["reason"] == "state_drift"
            assert document["action_code"] == "rerun_install_to_reconcile"
            assert observed_at - opened_at <= WINDOW_UNDER_TEST + LAPSE_SLACK_SECONDS
            # It reports the failure; it does not exit into a restart flap.
            assert agent.running
            agent.assert_not_spinning()
        finally:
            agent.close()

    def test_deleting_a_committed_state_is_unhealthy_not_a_fresh_install(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The regression in its purest form: a device going *backwards*.

        A supervisor that has already published a committed activation must not
        answer "still installing" when that state disappears underneath it.
        """

        _installed(layout, tmp_path)

        agent = _Agent(_exec_start(layout))
        try:
            first = agent.document()
            assert first["state"] == "provisioning_pending"
            assert first["activation"] == _committed_id(layout)

            layout.state_file.unlink()

            document, _ = agent.until(lambda doc: doc["state"] == "unhealthy", timeout=30.0)
            assert document["ok"] is False
            assert document["reason"] == "state_drift"
            assert document["action_code"] == "rerun_install_to_reconcile"
            assert document["profile"] == ""
            assert agent.running
        finally:
            agent.close()

    def test_a_missing_state_with_no_window_at_all_is_refused_immediately(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        layout.state_file.unlink()

        code, document, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "state_drift"
        assert document["action_code"] == "rerun_install_to_reconcile"
        assert "Traceback" not in stderr

    def test_a_lapsed_window_stops_excusing_the_missing_state(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Leaving the file behind must not be a way to stay pending forever."""

        _installed(layout, tmp_path)
        layout.state_file.unlink()
        _forge_window(
            layout,
            {
                "schema": LIFECYCLE_WINDOW_VERSION,
                "action": "install",
                "version": "1.0.0",
                "opened_at": time.time() - 3600.0,
                "window_seconds": 120.0,
            },
        )

        code, document, _ = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "state_drift"

    def test_a_marker_cannot_ask_for_a_longer_window_than_this_build_allows(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The duration is clamped, so a forged one can only ever shorten it."""

        _installed(layout, tmp_path)
        layout.state_file.unlink()
        _forge_window(
            layout,
            {
                "schema": LIFECYCLE_WINDOW_VERSION,
                "action": "install",
                "version": "1.0.0",
                # Opened comfortably beyond the shipped ceiling, asking for a year.
                "opened_at": time.time() - 600.0,
                "window_seconds": 31_536_000.0,
            },
        )

        code, document, _ = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "state_drift"

    @pytest.mark.parametrize(
        "opened_at,window_seconds",
        [
            (float("nan"), 30.0),
            (float("inf"), 30.0),
            (0.0, float("nan")),
            (0.0, float("inf")),
            (0.0, -1.0),
            (True, 30.0),
            ("now", 30.0),
        ],
        ids=["nan-open", "inf-open", "nan-bound", "inf-bound", "negative", "boolean", "string"],
    )
    def test_a_marker_that_is_not_a_finite_number_grants_nothing(
        self, layout: Layout, tmp_path: Path, opened_at: object, window_seconds: object
    ) -> None:
        """``json.loads`` accepts ``NaN``, and every comparison against it is false.

        ``min(NaN, ceiling)`` is ``NaN`` and ``elapsed >= NaN`` never fires, so a
        marker carrying one token would have restored the unbounded
        ``activation_pending`` this whole design exists to abolish -- with the
        difference that an attacker, rather than a race, chooses when.
        """

        _installed(layout, tmp_path)
        layout.state_file.unlink()
        _forge_window(
            layout,
            {
                "schema": LIFECYCLE_WINDOW_VERSION,
                "action": "install",
                "version": "1.0.0",
                "opened_at": opened_at,
                "window_seconds": window_seconds,
            },
        )

        code, document, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] != "activation_pending"
        assert document["state"] == "unhealthy"
        assert document["reason"] == "config_unreadable"
        assert document["action_code"] == "restore_config"
        assert "Traceback" not in stderr

    def test_a_tampered_marker_cannot_choose_what_the_device_publishes(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The marker is attacker-writable; the refusal is published verbatim.

        Interpolating a rejected field into the detail would let whoever wrote
        the file pick bytes that land in the agent's JSON document, the doctor
        snapshot a bundle collects, and the journal.
        """

        _installed(layout, tmp_path)
        layout.state_file.unlink()
        _forge_window(
            layout,
            {
                "schema": SECRET,
                "action": SECRET,
                "version": SECRET,
                "opened_at": SECRET,
                "window_seconds": SECRET,
            },
        )

        code, document, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "config_unreadable"
        assert SECRET not in json.dumps(document)
        assert SECRET not in stderr

    def test_the_one_shots_refuse_a_missing_state_even_inside_a_live_window(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """A command that answers once cannot wait a window out.

        The supervisor can, and does, which is the negative control here: the
        same device, the same instant, and two correct but different answers.
        """

        _installed(layout, tmp_path)
        _precommit(layout)

        # The supervisor is entitled to wait: it will look again in 250ms.
        code, document, _ = _one_cycle(_exec_start(layout))
        assert code == 0
        assert document["state"] == "activation_pending"

        doctor = _exec_start(layout, "flyto-robot-doctor.service")
        # `readiness` is the other one-shot. It has no unit of its own, so the
        # subcommand is swapped in the doctor's own rendered argv rather than
        # invented: everything else about the command stays exactly as shipped.
        readiness = list(doctor)
        readiness[readiness.index("doctor")] = "readiness"

        for argv in (doctor, readiness):
            completed = subprocess.run(
                argv, capture_output=True, text=True, timeout=60, check=False, env=_env()
            )
            published = json.loads(completed.stdout.strip().splitlines()[-1])

            assert completed.returncode == 1, argv
            assert published["state"] == "unhealthy", argv
            assert published["reason"] == "state_drift", argv
            assert published["action_code"] == "rerun_install_to_reconcile", argv
            assert "Traceback" not in completed.stderr

        # And the evidence lands where a bundle looks for it.
        snapshot = json.loads((layout.state_dir / "doctor-status.json").read_text())
        assert snapshot["state"] == "unhealthy"
        assert snapshot["reason"] == "state_drift"

    def test_a_device_that_was_never_installed_is_told_to_install(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The other half of the refusal: nothing active, so nothing drifted."""

        _installed(layout, tmp_path)
        argv = _exec_start(layout)
        layout.state_file.unlink()
        layout.current.unlink()

        code, document, _ = _one_cycle(argv)

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "not_installed"
        assert document["action_code"] == "run_install"


class TestOneDocumentDecidesBothSchemaAndFields:
    """Requirement: classification and validation read the *same* bytes."""

    def test_the_answer_comes_from_the_one_document_the_single_read_returned(
        self, layout: Layout, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        """The atomic replacement every install performs, aimed at the reader.

        The runtime asked the state file two questions -- "which schema are you?"
        and "what do you say?" -- in two separate reads. ``os.replace`` landing
        between them meant the answers could come from two different documents,
        and the pair that matters is reachable: the schema decides whether the
        *mutable* per-version view may stand in for the immutable by-id record,
        while the fields decide what is being resolved. Classify one document and
        validate another and the reader is enforcing a policy no document on this
        device ever stated.

        The property under test is that there is exactly **one** document. What
        that document is entitled to is not in dispute here: the read returns a
        raw v1 state, v1 devices legitimately resolve through their digest
        checked view, and so that is the right answer. A reader may only decide
        from the bytes its read returned -- asserting that v2 bytes it never saw
        should have changed the verdict would be asserting the old double read
        back into existence, from the other side.

        The swap is deterministic rather than raced: the first read of that path
        returns the v1 bytes, every later read returns what is on disk. A reader
        that looks twice therefore sees the replacement; a reader that looks once
        cannot, which is what the read count below pins down.
        """

        report = _installed(layout, tmp_path)
        committed = _committed_id(layout)
        v2_document = layout.state_file.read_text(encoding="utf-8")
        _downgrade_state_to_v1(layout)
        v1_document = layout.state_file.read_text(encoding="utf-8")
        # What is on disk *after* the swap: the v2 document, with no by-id record
        # -- the shape whose authority differs from the v1 one the read returns.
        atomic_write(layout.state_file, v2_document, 0o640)

        reads: list[str] = []
        original = Path.read_text
        target = layout.state_file

        def swapping(self: Path, *args: object, **kwargs: object) -> str:
            if self == target:
                reads.append(str(self))
                if len(reads) == 1:
                    return v1_document
            return original(self, *args, **kwargs)

        monkeypatch.setattr(Path, "read_text", swapping)

        snapshot = runtime_activation(layout)

        # The v1 document was read, so the v1 device's own activation resolves,
        # through the view whose bytes hash to exactly the id history names.
        assert snapshot is not None
        assert snapshot.activation_id == committed
        assert snapshot.profile == report["profile"]
        # And the replacement had nothing to bite on: one question, one document.
        assert len(reads) == 1, f"the state file was read {len(reads)} times"


class TestTheCommittedRecordIsTheAuthority:
    """Requirement: resolve the exact by-id record, or refuse."""

    def test_a_committed_activation_with_no_record_is_unhealthy_not_pending(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        for path in _record_files(layout):
            path.unlink()

        code, document, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["ok"] is False
        assert document["reason"] == "activation_not_recorded"
        assert document["action_code"] == "install_that_version_explicitly"
        # Not the version view's profile, and not the shipped registry's.
        assert document["profile"] == ""
        assert "Traceback" not in stderr

    def test_a_tampered_record_is_refused_without_echoing_what_it_says(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Absent is a window. Altered is an attack, and gets no benefit of doubt."""

        _installed(layout, tmp_path)
        record = _record_files(layout)[0]
        record.chmod(0o640)
        document = json.loads(record.read_text(encoding="utf-8"))
        document["profile"] = SECRET
        record.write_text(json.dumps(document), encoding="utf-8")

        code, published, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert published["state"] == "unhealthy"
        assert published["reason"] == "activation_snapshot_invalid"
        assert published["profile"] == ""
        # The device does not repeat what the tamperer wrote back at anyone.
        assert SECRET not in json.dumps(published)
        assert "Traceback" not in stderr

    def test_a_tampered_version_view_does_not_change_what_the_device_reports(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The view is a compatibility surface, so editing it must do nothing.

        This is the negative control for the whole boundary: before the record
        became authority, this edit changed the readiness contract a running
        machine evaluated itself against.
        """

        report = _installed(layout, tmp_path)
        view = layout.activation_file("1.0.0")
        view.chmod(0o640)
        document = json.loads(view.read_text(encoding="utf-8"))
        document["profile"] = SECRET
        view.write_text(json.dumps(document), encoding="utf-8")

        code, published, _ = _one_cycle(_exec_start(layout))

        assert code == 0
        assert published["state"] == "provisioning_pending"
        assert published["profile"] == report["profile"]
        assert published["activation"] == _committed_id(layout)

    def test_a_state_file_that_names_nothing_is_not_the_pre_commit_window(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Absent means pending. Present-and-empty is a demotion, not a window."""

        _installed(layout, tmp_path)
        atomic_write(
            layout.state_file,
            json.dumps(
                {
                    "schema": "flyto.lifecycle-state.v2",
                    "current": None,
                    "current_activation": None,
                    "profile": None,
                    "history": [],
                }
            ),
            0o640,
        )

        code, document, _ = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "activation_not_recorded"

    def test_an_unreadable_state_file_is_refused_with_a_code(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        atomic_write(layout.state_file, "{not json at all", 0o640)

        code, document, stderr = _one_cycle(_exec_start(layout))

        assert code == 1
        assert document["state"] == "unhealthy"
        assert document["reason"] == "config_unreadable"
        assert document["action_code"] == "restore_config"
        assert "Traceback" not in stderr

    def test_the_doctor_writes_the_same_refusal_where_the_bundle_looks(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """One shot, same authority, and the evidence lands on disk."""

        _installed(layout, tmp_path)
        for path in _record_files(layout):
            path.unlink()

        argv = _exec_start(layout, "flyto-robot-doctor.service")
        completed = subprocess.run(
            argv, capture_output=True, text=True, timeout=60, check=False, env=_env()
        )

        assert completed.returncode == 1
        snapshot = json.loads((layout.state_dir / "doctor-status.json").read_text())
        assert snapshot["service"] == "doctor"
        assert snapshot["state"] == "unhealthy"
        assert snapshot["reason"] == "activation_not_recorded"


class TestStatusReadsTheSameAuthority:
    """Requirement: enumerate from the exact current activation, or fail closed."""

    def test_status_enumerates_the_record_even_with_no_version_view_at_all(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        profiles, alpha, _ = _two_profiles(tmp_path)
        install(payload=_payload(tmp_path, "a", "v1"), version="1.0.0", layout=layout,
                profile=alpha, profiles=profiles, systemd=_fake())
        layout.activation_file("1.0.0").unlink()

        report = status(layout, profiles=profiles)

        assert report["active_profile"] == alpha
        assert report["installed_units"]

    def test_a_rollback_to_an_earlier_activation_of_one_version_is_what_status_reports(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The case the version view cannot answer at all.

        Two activations, one version. The view holds the newer one; the device is
        running the older one. Resolving by version names the activation this
        machine deliberately left.
        """

        profiles, alpha, beta = _two_profiles(tmp_path)
        payload = _payload(tmp_path, "a", "v1")
        install(payload=payload, version="1.0.0", layout=layout, profile=alpha,
                profiles=profiles, systemd=_fake())
        install(payload=payload, version="1.0.0", layout=layout, profile=beta,
                profiles=profiles, systemd=_fake())
        assert status(layout, profiles=profiles)["active_profile"] == beta

        assert rollback(layout=layout, profiles=profiles, systemd=_fake())["ok"] is True
        report = status(layout, profiles=profiles)
        assert report["active_profile"] == alpha
        assert report["recorded_current"] == "1.0.0"

    def test_status_fails_closed_when_the_committed_record_is_missing(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        for path in _record_files(layout):
            path.unlink()

        with pytest.raises(LifecycleError) as caught:
            status(layout)
        assert caught.value.reason == "activation_not_recorded"

    def test_status_fails_closed_when_the_committed_record_is_tampered(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        record = _record_files(layout)[0]
        record.chmod(0o640)
        document = json.loads(record.read_text(encoding="utf-8"))
        document["units"][sorted(document["units"])[0]] += "\n# edited\n"
        record.write_text(json.dumps(document), encoding="utf-8")

        with pytest.raises(LifecycleError) as caught:
            status(layout)
        assert caught.value.reason == "activation_snapshot_invalid"


def _downgrade_state_to_v1(layout: Layout) -> None:
    """Put the device back in the shape the previous build left it in.

    v1 had no activation identity and wrote no by-id records: one snapshot per
    version, at the path v2 keeps as the view. Reproduced exactly, so the
    compatibility path is exercised by a v1 *document* rather than by a v2 one
    with a file removed -- those are different devices and only one of them is
    entitled to the fallback.
    """

    state = json.loads(layout.state_file.read_text(encoding="utf-8"))
    state["schema"] = "flyto.lifecycle-state.v1"
    state.pop("current_activation", None)
    for entry in state["history"]:
        entry.pop("activation_id", None)
    atomic_write(layout.state_file, json.dumps(state, indent=2, sort_keys=True) + "\n", 0o640)
    for path in _record_files(layout):
        path.unlink()


class TestV1DevicesAreNotStranded:
    """The one narrow exception, and its limit."""

    def test_a_raw_v1_state_still_resolves_through_its_digest_checked_view(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        report = _installed(layout, tmp_path)
        _downgrade_state_to_v1(layout)

        assert status(layout)["active_profile"] == report["profile"]
        code, document, _ = _one_cycle(_exec_start(layout))
        assert code == 0
        assert document["profile"] == report["profile"]

    def test_a_v1_device_whose_view_was_also_edited_is_still_refused(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Compatibility is not credulity: the bytes still have to hash right."""

        _installed(layout, tmp_path)
        _downgrade_state_to_v1(layout)
        view = layout.activation_file("1.0.0")
        view.chmod(0o640)
        document = json.loads(view.read_text(encoding="utf-8"))
        document["profile"] = SECRET
        view.write_text(json.dumps(document), encoding="utf-8")

        with pytest.raises(LifecycleError):
            status(layout)

    def test_the_next_mutating_operation_leaves_a_v1_device_on_records(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The fallback is a migration path, so it has to stop being taken."""

        _installed(layout, tmp_path)
        _downgrade_state_to_v1(layout)

        install(payload=_payload(tmp_path, "b", "v2"), version="2.0.0", layout=layout,
                python=sys.executable, systemd=_fake())

        assert _record_files(layout)
        # And the new state is v2, so the compatibility path is not taken again.
        assert json.loads(layout.state_file.read_text())["schema"] == "flyto.lifecycle-state.v2"
        assert status(layout)["active_profile"]


class TestABundleSurvivesTheDeviceItIsCollectedOn:
    """Requirement: still collectible, still redacted, still 0600, on corruption."""

    @pytest.fixture()
    def corrupted(self, layout: Layout, tmp_path: Path) -> Layout:
        _installed(layout, tmp_path)
        _provision(layout)
        (layout.credentials_dir / "device.cred").write_text(SECRET, encoding="utf-8")
        for path in _record_files(layout):
            path.unlink()
        return layout

    def _bundle(self, layout: Layout) -> dict:
        return build_support_bundle(
            layout, now="2026-01-01T00:00:00Z", systemd=_fake()
        )

    def test_a_missing_record_names_a_reason_and_an_action(self, corrupted: Layout) -> None:
        bundle = self._bundle(corrupted)

        assert bundle["reason"] == "activation_not_recorded"
        assert bundle["action_code"] == "install_that_version_explicitly"
        assert bundle["reason_text"] != "unrecognised reason code"
        # "could not be read" is not the same claim as "there is nothing here",
        # and a responder has to be able to tell them apart.
        assert bundle["lifecycle"]["readable"] is False
        assert bundle["lifecycle"]["reason"] == "activation_not_recorded"

    def test_it_still_says_which_units_are_running(self, corrupted: Layout) -> None:
        """The lifecycle could not say what *should* run. That is when this matters."""

        bundle = self._bundle(corrupted)
        assert bundle["units"], "the unit files are on disk and are still evidence"
        assert [entry["unit"] for entry in bundle["unit_health"]] == [
            unit["name"] for unit in bundle["units"]
        ]

    def test_it_is_writable_deterministic_and_0600(
        self, corrupted: Layout, tmp_path: Path
    ) -> None:
        first = write_support_bundle(tmp_path / "one.json", self._bundle(corrupted))
        second = write_support_bundle(tmp_path / "two.json", self._bundle(corrupted))

        assert first.stat().st_mode & 0o777 == 0o600
        assert second.stat().st_mode & 0o777 == 0o600
        assert first.read_text(encoding="utf-8") == second.read_text(encoding="utf-8")

    def test_it_carries_no_secret_and_no_traceback(
        self, corrupted: Layout, tmp_path: Path
    ) -> None:
        text = write_support_bundle(
            tmp_path / "bundle.json", self._bundle(corrupted)
        ).read_text(encoding="utf-8")

        assert SECRET not in text
        assert "Traceback" not in text
        assert "File \"" not in text

    def test_a_corrupted_state_file_cannot_choose_what_the_bundle_says(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The refusal text quotes the state file. The state file is the attacker.

        Carrying the exception's own message would let whatever a tamperer wrote
        into ``lifecycle-state.json`` ride into a mailbox inside a document whose
        entire promise is that it carries nothing of the sort.
        """

        _installed(layout, tmp_path)
        atomic_write(
            layout.state_file,
            json.dumps({"schema": "flyto.lifecycle-state.v2", "current": SECRET}),
            0o640,
        )

        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())
        text = write_support_bundle(tmp_path / "bundle.json", bundle).read_text(encoding="utf-8")

        assert bundle["reason"] == "config_unreadable"
        assert bundle["action_code"] == "restore_config"
        assert bundle["lifecycle"]["readable"] is False
        assert SECRET not in text

    def test_a_device_that_lost_its_state_file_does_not_lead_with_ok(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """A readable lifecycle that says something is wrong is still wrong.

        Only *thrown* refusals were promoted to the top level, so the one device
        this whole window design exists for -- units switched, state write never
        landed -- produced a bundle whose first two fields said ``reason: ok``
        and ``action_code: none``. The section below it said ``state_drift`` the
        whole time; nobody reads a bundle bottom-up.
        """

        _installed(layout, tmp_path)
        _provision(layout)
        (layout.credentials_dir / "device.cred").write_text(SECRET, encoding="utf-8")
        layout.state_file.unlink()

        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())
        first = write_support_bundle(tmp_path / "drift-one.json", bundle)
        second = write_support_bundle(
            tmp_path / "drift-two.json",
            build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake()),
        )

        assert bundle["lifecycle"]["readable"] is True
        assert bundle["reason"] == "state_drift"
        assert bundle["action_code"] == "rerun_install_to_reconcile"
        assert bundle["reason_text"] != "unrecognised reason code"
        # No marker, so nothing was in flight -- which is the fact that turns
        # "mid-install" into "this device lost its state file".
        assert bundle["lifecycle"]["activation_window"] == {
            "present": False,
            "verified": False,
            "window_seconds": None,
        }
        # Still deterministic, still 0600, still carrying no credential bytes.
        assert first.stat().st_mode & 0o777 == 0o600
        assert second.stat().st_mode & 0o777 == 0o600
        assert first.read_text(encoding="utf-8") == second.read_text(encoding="utf-8")
        assert SECRET not in first.read_text(encoding="utf-8")

    def test_the_reason_the_operator_gave_is_never_overwritten(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Why the bundle was collected outranks what it happened to find."""

        _installed(layout, tmp_path)
        layout.state_file.unlink()

        bundle = build_support_bundle(
            layout, now="2026-01-01T00:00:00Z", reason="note_rejected", systemd=_fake()
        )

        assert bundle["reason"] == "note_rejected"
        assert bundle["lifecycle"]["reason"] == "state_drift"

    def test_a_bundle_says_whether_an_activation_was_in_flight(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        _installed(layout, tmp_path)
        open_activation_window(layout, action="install", version="1.0.0", seconds=30.0)

        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())

        assert bundle["lifecycle"]["activation_window"] == {
            "present": True,
            "verified": True,
            "window_seconds": 30.0,
        }

    def test_a_bundle_carries_no_field_the_marker_chose(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The marker is writable on exactly the devices bundles come from."""

        _installed(layout, tmp_path)
        _forge_window(
            layout,
            {
                "schema": LIFECYCLE_WINDOW_VERSION,
                "action": SECRET,
                "version": SECRET,
                "opened_at": SECRET,
                "window_seconds": SECRET,
            },
        )

        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())
        text = write_support_bundle(tmp_path / "window.json", bundle).read_text(encoding="utf-8")

        assert bundle["lifecycle"]["activation_window"] == {
            "present": True,
            "verified": False,
            "window_seconds": None,
        }
        assert SECRET not in text

    def test_a_healthy_device_still_reports_a_readable_lifecycle(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """The negative control: this path must not swallow a working device.

        Provisioned deliberately. Promotion now covers any refusal the lifecycle
        *reports* as well as any it throws, so "healthy" has to mean a device
        whose status is genuinely ``ok`` -- an unpaired one correctly leads with
        ``identity_missing``, and asserting ``ok`` there would only be asserting
        that the promotion is broken.
        """

        _installed(layout, tmp_path)
        _provision(layout)
        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())

        assert bundle["lifecycle"]["readable"] is True
        assert bundle["lifecycle"]["active_profile"]
        assert bundle["reason"] == "ok"
        assert bundle["action_code"] == "none"

    def test_an_unpaired_device_leads_with_the_thing_an_operator_must_do(
        self, layout: Layout, tmp_path: Path
    ) -> None:
        """Promotion is not only for corruption; it is for anything actionable."""

        _installed(layout, tmp_path)

        bundle = build_support_bundle(layout, now="2026-01-01T00:00:00Z", systemd=_fake())

        assert bundle["lifecycle"]["readable"] is True
        assert bundle["reason"] == "identity_missing"
        assert bundle["action_code"] == "provision_identity"
