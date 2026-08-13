import subprocess
import threading
import time

import pytest

from flyto_robotics.camera_observation import CameraConfigurationError, CameraObservation
from flyto_robotics.camera_sources import (
    AvfoundationRuntime,
    CameraSettings,
    _avfoundation_argv,
    _avfoundation_runtime_argv,
    probe_source,
)


def settings(**updates):
    env = {"FLYTO_CAMERA_PROVIDER": "avfoundation", "FLYTO_CAMERA_DEVICE": "0"}
    env.update(updates)
    return CameraSettings.from_environ(env)


@pytest.mark.parametrize(("name", "value", "reason"), [
    ("FLYTO_CAMERA_PROVIDER", "usb", "camera_provider_invalid"),
    ("FLYTO_CAMERA_SOURCE_ID", "bad/source", "camera_source_id_invalid"),
    ("FLYTO_CAMERA_ROTATION", "45", "camera_rotation_invalid"),
    ("FLYTO_CAMERA_FLIP", "diagonal", "camera_flip_invalid"),
    ("FLYTO_CAMERA_DEVICE", "x\nsecret", "camera_device_invalid"),
])
def test_operator_settings_fail_closed(name, value, reason):
    with pytest.raises(CameraConfigurationError, match=reason):
        settings(**{name: value})


@pytest.mark.parametrize(("name", "value", "reason"), [
    ("FLYTO_CAMERA_WIDTH", "0", "camera_width_invalid"),
    ("FLYTO_CAMERA_WIDTH", "8193", "camera_width_invalid"),
    ("FLYTO_CAMERA_WIDTH", "1.5", "camera_width_invalid"),
    ("FLYTO_CAMERA_WIDTH", True, "camera_width_invalid"),
    ("FLYTO_CAMERA_HEIGHT", "0", "camera_height_invalid"),
    ("FLYTO_CAMERA_HEIGHT", "8193", "camera_height_invalid"),
    ("FLYTO_CAMERA_HEIGHT", False, "camera_height_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "0", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "120.1", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "NaN", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "inf", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", " 10", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "10\n", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "１０", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "1e-100000", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "1" * 33, "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", ".5", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", "10.", "camera_framerate_invalid"),
    ("FLYTO_CAMERA_FRAMERATE", True, "camera_framerate_invalid"),
])
def test_camera_capture_settings_fail_closed(name, value, reason):
    with pytest.raises(CameraConfigurationError, match=f"^{reason}$"):
        settings(**{name: value})


def test_camera_capture_defaults_and_ros_carries_validated_values():
    avfoundation = settings()
    ros = CameraSettings.from_environ({"FLYTO_CAMERA_PROVIDER": "ros_image"})

    assert (avfoundation.width, avfoundation.height, avfoundation.framerate) == (1280, 720, "10")
    assert (ros.width, ros.height, ros.framerate) == (1280, 720, "10")


def test_avfoundation_input_args_are_shared_ordered_and_normalized():
    configured = settings(
        FLYTO_CAMERA_WIDTH="1920",
        FLYTO_CAMERA_HEIGHT="1080",
        FLYTO_CAMERA_FRAMERATE="010.5000",
        FLYTO_CAMERA_ROTATION="90",
    )
    probe = _avfoundation_argv(configured, "/ffmpeg")
    runtime = _avfoundation_runtime_argv(configured, "/ffmpeg")
    expected_input = [
        "-f", "avfoundation", "-framerate", "10.5", "-video_size", "1920x1080",
        "-i", "0",
    ]

    input_start = probe.index("-f")
    assert probe[input_start:input_start + len(expected_input)] == expected_input
    assert runtime[input_start:input_start + len(expected_input)] == expected_input
    assert probe.index("-framerate") < probe.index("-i")
    assert probe.index("-video_size") < probe.index("-i")
    assert probe[probe.index("-vf") + 1] == "transpose=1"


@pytest.mark.parametrize(("rotation", "flip", "expected"), [
    ("0", "none", None),
    ("90", "none", "transpose=1"),
    ("180", "none", "hflip,vflip"),
    ("270", "none", "transpose=2"),
    ("0", "horizontal", "hflip"),
    ("0", "vertical", "vflip"),
    ("90", "both", "transpose=1,hflip,vflip"),
])
def test_avfoundation_orientation_filters(rotation, flip, expected):
    argv = _avfoundation_argv(
        settings(FLYTO_CAMERA_ROTATION=rotation, FLYTO_CAMERA_FLIP=flip), "/ffmpeg",
    )
    if expected is None:
        assert "-vf" not in argv
    else:
        assert argv[argv.index("-vf") + 1] == expected


def test_avfoundation_probe_uses_argv_null_output_and_never_exposes_device():
    calls = []

    def runner(argv, **kwargs):
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0)

    result = probe_source(settings(FLYTO_CAMERA_DEVICE="private-selector"), runner=runner,
                          which=lambda _name: "/synthetic/ffmpeg", clock=lambda: 1)
    argv, kwargs = calls[0]
    assert result["ok"] is True
    assert "private-selector" not in str(result)
    assert argv[-4:] == ["-an", "-f", "null", "-"]
    assert kwargs["shell"] is False
    assert kwargs["stdout"] is subprocess.DEVNULL
    assert kwargs["stderr"] is subprocess.DEVNULL


def test_probe_failure_is_stable_and_does_not_return_runner_text():
    def runner(argv, **kwargs):
        return subprocess.CompletedProcess(argv, 1, stderr=b"arbitrary device text")

    result = probe_source(settings(), runner=runner, which=lambda _name: "/ffmpeg",
                          clock=lambda: 1)
    assert result["reason"] == "camera_device_unavailable"
    assert "arbitrary" not in str(result)


def test_missing_dependency_fails_without_starting_a_process():
    result = probe_source(settings(), which=lambda _name: None, clock=lambda: 1)
    assert result["ok"] is False
    assert result["reason"] == "camera_dependency_missing"


def test_ros_dependency_probe_never_claims_an_observed_frame(monkeypatch):
    monkeypatch.setattr("builtins.__import__", lambda *args, **kwargs: object())
    ros = CameraSettings.from_environ({"FLYTO_CAMERA_PROVIDER": "ros_image"})
    result = probe_source(ros, clock=lambda: 1)
    assert result["ok"] is True
    assert result["usable"] is False
    assert result["reason"] == "camera_dependency_available"


class FakeProgress:
    def __init__(self, chunks=()):
        self.chunks = list(chunks)
        self.closed = threading.Event()

    def read(self, size):
        assert size <= 512
        if self.chunks:
            return self.chunks.pop(0)
        self.closed.wait(1)
        return b""


class FakeProcess:
    def __init__(self, chunks=(), returncode=None):
        self.stdout = FakeProgress(chunks)
        self.returncode = returncode
        self.terminated = False
        self.killed = False

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        if self.returncode is None:
            self.stdout.closed.wait(timeout or 1)
            self.returncode = 0
        return self.returncode

    def terminate(self):
        self.terminated = True
        self.returncode = 0
        self.stdout.closed.set()

    def kill(self):
        self.killed = True
        self.returncode = -9
        self.stdout.closed.set()


class RecordingStop:
    def __init__(self, limit):
        self.limit = limit
        self.waits = []

    def is_set(self):
        return len(self.waits) >= self.limit

    def wait(self, seconds):
        self.waits.append(seconds)
        return self.is_set()

    def set(self):
        self.limit = 0


def _wait_until(predicate):
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition not reached")


def test_avfoundation_runtime_uses_one_long_lived_process_and_frame_event():
    process = FakeProcess([b"frame=1\nprogress=continue\n"])
    calls = []

    def popen(argv, **kwargs):
        calls.append((argv, kwargs))
        return process

    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: 1)
    runtime = AvfoundationRuntime(settings(), observation, popen=popen,
                                  which=lambda _name: "/ffmpeg")
    runtime.start()
    _wait_until(lambda: bool(observation.payload()))
    assert len(calls) == 1
    argv, kwargs = calls[0]
    assert argv.count("-progress") == 1 and "pipe:1" in argv
    assert "-frames:v" not in argv
    assert argv[-4:] == ["-an", "-f", "null", "-"]
    assert kwargs["shell"] is False
    assert kwargs["stdout"] is subprocess.PIPE
    assert kwargs["stderr"] is subprocess.DEVNULL
    assert "frame=1" not in str(observation.payload())
    runtime.stop()
    assert process.terminated and not process.killed
    assert observation.payload() == []


def test_avfoundation_runtime_no_event_and_exit_never_claim_usable():
    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: 1)
    runtime = AvfoundationRuntime(settings(), observation, popen=lambda *_a, **_k: None)
    for chunks in ([b"progress=continue\n"], []):
        process = FakeProcess(chunks, returncode=1)
        assert runtime._consume_progress(process) is False
        assert observation.payload() == []


def test_avfoundation_progress_parser_is_bounded_and_ignores_raw_output():
    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: 1)
    runtime = AvfoundationRuntime(settings(), observation, popen=lambda *_a, **_k: None)
    process = FakeProcess([b"secret-pixels" * 1000, b"\nframe=0\n"], returncode=1)
    assert runtime._consume_progress(process) is False
    assert observation.payload() == []


def test_progress_only_increasing_frame_counters_refresh_freshness():
    now = [0.0]

    class TimedProgress(FakeProgress):
        def read(self, size):
            chunk = super().read(size)
            now[0] += 1.0
            return chunk

    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: now[0])
    runtime = AvfoundationRuntime(settings(), observation, clock=lambda: now[0])
    process = FakeProcess()
    process.stdout = TimedProgress([b"frame=4\n", b"frame=4\n", b"frame=3\n", b"frame=5\n"])

    assert runtime._consume_progress(process) is True
    assert observation._accepted_at == 4.0


@pytest.mark.parametrize("counter", [b"9223372036854775808", b"1" * 1000, b"12x"])
def test_progress_rejects_oversized_or_non_decimal_frame_counters(counter):
    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: 1)
    runtime = AvfoundationRuntime(settings(), observation)

    assert runtime._consume_progress(FakeProcess([b"frame=" + counter + b"\n"])) is False
    assert observation.payload() == []


def test_progress_frame_counter_resets_for_each_new_process():
    now = [1.0]
    observation = CameraObservation("ward-a", 2, provider="avfoundation", clock=lambda: now[0])
    runtime = AvfoundationRuntime(settings(), observation)

    assert runtime._consume_progress(FakeProcess([b"frame=9\n"])) is True
    now[0] = 2.0
    assert runtime._consume_progress(FakeProcess([b"frame=1\n"])) is True
    assert observation._accepted_at == 2.0


def test_immediate_exits_use_bounded_exponential_restart_backoff():
    stop = RecordingStop(5)
    processes = []

    def popen(*_args, **_kwargs):
        process = FakeProcess(returncode=1)
        processes.append(process)
        return process

    runtime = AvfoundationRuntime(
        settings(), CameraObservation("ward-a", 2, provider="avfoundation"),
        popen=popen, which=lambda _name: "/ffmpeg", clock=lambda: 0,
        stop_event=stop,
    )
    runtime._run()
    assert len(processes) == 5
    assert stop.waits == [2.0, 4.0, 8.0, 16.0, 30.0]


@pytest.mark.parametrize("stdout", [None, object()])
def test_missing_or_invalid_progress_pipe_fails_closed_and_reaps(stdout):
    process = FakeProcess(returncode=None)
    process.stdout = stdout
    observation = CameraObservation("ward-a", 2, provider="avfoundation")
    runtime = AvfoundationRuntime(settings(), observation)
    assert runtime._consume_progress(process) is False
    runtime._stop_process(process)
    assert process.terminated
    assert observation.payload() == []


def test_progress_read_error_fails_closed_without_leaking_or_raising():
    class BrokenProgress:
        def read(self, _size):
            raise OSError("private child output")

    process = FakeProcess(returncode=None)
    process.stdout = BrokenProgress()
    observation = CameraObservation("ward-a", 2, provider="avfoundation")
    runtime = AvfoundationRuntime(settings(), observation)
    assert runtime._consume_progress(process) is False
    runtime._stop_process(process)
    assert process.terminated
    assert "private" not in str(observation.payload())


def test_unbounded_process_wait_is_terminated_then_killed_with_timeouts():
    class StubbornProcess(FakeProcess):
        def wait(self, timeout=None):
            assert timeout == 1.0
            raise subprocess.TimeoutExpired("ffmpeg", timeout)

        def terminate(self):
            self.terminated = True

        def poll(self):
            return None

    process = StubbornProcess()
    AvfoundationRuntime._stop_process(process)
    assert process.terminated and process.killed


def test_avfoundation_stop_kills_a_process_that_ignores_terminate():
    class StubbornProcess(FakeProcess):
        def poll(self):
            return None

        def terminate(self):
            self.terminated = True

        def wait(self, timeout=None):
            raise subprocess.TimeoutExpired("ffmpeg", timeout)

    process = StubbornProcess()
    runtime = AvfoundationRuntime(
        settings(), CameraObservation("ward-a", 2, provider="avfoundation"),
        popen=lambda *_a, **_k: process, which=lambda _name: "/ffmpeg",
    )
    runtime.start()
    _wait_until(lambda: runtime._process is process)
    runtime.stop()
    assert process.terminated and process.killed
