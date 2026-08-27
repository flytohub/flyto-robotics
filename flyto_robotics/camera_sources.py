"""Provider-neutral camera source settings and bounded availability probes."""

from __future__ import annotations

import contextlib
import os
import re
import shutil
import subprocess
import threading
import time
from collections.abc import Mapping
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation

from .camera_observation import (
    DEFAULT_STREAM_TTL,
    MAX_STREAM_LABEL,
    STREAM_PROTOCOLS,
    CameraConfigurationError,
    validate_bind,
    validate_source_id,
    validate_stream_url,
    validate_zone,
)

PROVIDERS = frozenset({"ros_image", "avfoundation"})
FLIPS = frozenset({"none", "horizontal", "vertical", "both"})
ROTATIONS = frozenset({0, 90, 180, 270})
MAX_TOPIC = 256
MAX_DEVICE = 128
PROBE_TIMEOUT_SECONDS = 5.0
MAX_PROGRESS_BUFFER = 4096
PROGRESS_READ_SIZE = 512
MAX_FRAME_COUNTER = (1 << 63) - 1
MAX_FRAME_COUNTER_DIGITS = len(str(MAX_FRAME_COUNTER))
MIN_RESTART_BACKOFF_SECONDS = 1.0
MAX_RESTART_BACKOFF_SECONDS = 30.0
DURABLE_RUN_SECONDS = 10.0
PROCESS_EXIT_TIMEOUT_SECONDS = 1.0
MAX_FRAMERATE_TEXT = 32
FRAMERATE_PATTERN = re.compile(r"[0-9]+(?:\.[0-9]+)?", re.ASCII)


def _integer(value: str, reason: str) -> int:
    try:
        return int(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CameraConfigurationError(reason) from exc


def _decimal(value: str, reason: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CameraConfigurationError(reason) from exc
    if result != result or result in {float("inf"), float("-inf")}:
        raise CameraConfigurationError(reason)
    return result


def _camera_dimension(value: str, reason: str) -> int:
    if not isinstance(value, str) or not value.isascii() or not value.isdigit():
        raise CameraConfigurationError(reason)
    result = _integer(value, reason)
    if not 1 <= result <= 8192:
        raise CameraConfigurationError(reason)
    return result


def _camera_framerate(value: str) -> str:
    if (
        not isinstance(value, str)
        or not 0 < len(value) <= MAX_FRAMERATE_TEXT
        or FRAMERATE_PATTERN.fullmatch(value) is None
    ):
        raise CameraConfigurationError("camera_framerate_invalid")
    try:
        result = Decimal(value)
    except (InvalidOperation, ValueError, TypeError) as exc:
        raise CameraConfigurationError("camera_framerate_invalid") from exc
    if not result.is_finite() or not Decimal("0") < result <= Decimal("120"):
        raise CameraConfigurationError("camera_framerate_invalid")
    return format(result.normalize(), "f")


def _bounded_text(value: str, maximum: int, reason: str, *, allow_empty: bool = False) -> str:
    if (not isinstance(value, str) or len(value) > maximum or (not value and not allow_empty)
            or value != value.strip() or any(ord(char) < 32 for char in value)):
        raise CameraConfigurationError(reason)
    return value


@dataclass(frozen=True)
class CameraSettings:
    provider: str
    source_id: str
    rotation: int
    flip: str
    device: str
    topic: str
    bind: str
    port: int
    zone: str
    freshness_seconds: float
    width: int
    height: int
    framerate: str
    #: Where this camera can be watched. Empty is the honest default: most
    #: robots have no media server, and a missing optional address is not a
    #: fault -- the catalog says `configured: false` and the observation route
    #: carries on unaffected.
    stream_url: str
    stream_protocol: str
    stream_label: str
    stream_ttl_seconds: int

    @classmethod
    def from_environ(cls, environ: Mapping[str, str] | None = None) -> CameraSettings:
        env = os.environ if environ is None else environ
        provider = _bounded_text(env.get("FLYTO_CAMERA_PROVIDER", "ros_image"), 32,
                                 "camera_provider_invalid")
        if provider not in PROVIDERS:
            raise CameraConfigurationError("camera_provider_invalid")
        source_id = validate_source_id(env.get("FLYTO_CAMERA_SOURCE_ID", "camera-0"))
        rotation = _integer(env.get("FLYTO_CAMERA_ROTATION", "0"), "camera_rotation_invalid")
        if rotation not in ROTATIONS:
            raise CameraConfigurationError("camera_rotation_invalid")
        flip = _bounded_text(env.get("FLYTO_CAMERA_FLIP", "none"), 16,
                             "camera_flip_invalid")
        if flip not in FLIPS:
            raise CameraConfigurationError("camera_flip_invalid")
        device = _bounded_text(env.get("FLYTO_CAMERA_DEVICE", ""), MAX_DEVICE,
                               "camera_device_invalid", allow_empty=True)
        topic = _bounded_text(env.get("FLYTO_CAMERA_TOPIC", "/camera/image_raw"), MAX_TOPIC,
                              "camera_topic_invalid")
        if not topic.startswith("/") or any(char.isspace() for char in topic):
            raise CameraConfigurationError("camera_topic_invalid")
        if provider == "avfoundation" and not device:
            raise CameraConfigurationError("camera_device_missing")
        port = _integer(env.get("FLYTO_CAMERA_PORT", "9000"), "camera_port_invalid")
        if not 1 <= port <= 65535:
            raise CameraConfigurationError("camera_port_invalid")
        freshness = _decimal(env.get("FLYTO_CAMERA_FRESHNESS_SECONDS", "2"),
                             "camera_freshness_invalid")
        if not 0.1 <= freshness <= 300:
            raise CameraConfigurationError("camera_freshness_invalid")
        width = _camera_dimension(env.get("FLYTO_CAMERA_WIDTH", "1280"),
                                  "camera_width_invalid")
        height = _camera_dimension(env.get("FLYTO_CAMERA_HEIGHT", "720"),
                                   "camera_height_invalid")
        framerate = _camera_framerate(env.get("FLYTO_CAMERA_FRAMERATE", "10"))
        stream_url = env.get("FLYTO_CAMERA_STREAM_URL", "")
        if stream_url:
            stream_url = validate_stream_url(stream_url)
        stream_protocol = _bounded_text(env.get("FLYTO_CAMERA_STREAM_PROTOCOL", "mjpeg"),
                                        16, "camera_stream_protocol_invalid")
        if stream_protocol not in STREAM_PROTOCOLS:
            raise CameraConfigurationError("camera_stream_protocol_invalid")
        stream_label = _bounded_text(env.get("FLYTO_CAMERA_STREAM_LABEL", ""),
                                     MAX_STREAM_LABEL, "camera_stream_label_invalid",
                                     allow_empty=True)
        stream_ttl = _integer(env.get("FLYTO_CAMERA_STREAM_TTL_SECONDS",
                                      str(DEFAULT_STREAM_TTL)), "camera_stream_ttl_invalid")
        return cls(provider, source_id, rotation, flip, device, topic,
                   validate_bind(env.get("FLYTO_CAMERA_BIND", "127.0.0.1")), port,
                   validate_zone(env.get("FLYTO_CAMERA_ZONE", "camera-zone")), freshness,
                   width, height, framerate,
                   stream_url, stream_protocol, stream_label, stream_ttl)

    def public_metadata(self) -> dict:
        return {"provider": self.provider, "source_id": self.source_id}


def _avfoundation_argv(settings: CameraSettings, executable: str) -> list[str]:
    filters = []
    if settings.rotation == 90:
        filters.append("transpose=1")
    elif settings.rotation == 180:
        filters.extend(("hflip", "vflip"))
    elif settings.rotation == 270:
        filters.append("transpose=2")
    if settings.flip in {"horizontal", "both"}:
        filters.append("hflip")
    if settings.flip in {"vertical", "both"}:
        filters.append("vflip")
    argv = [executable, "-nostdin", "-hide_banner", "-loglevel", "error",
            "-f", "avfoundation", "-framerate", settings.framerate,
            "-video_size", f"{settings.width}x{settings.height}",
            "-i", settings.device]
    if filters:
        argv.extend(("-vf", ",".join(filters)))
    return [*argv, "-frames:v", "1", "-an", "-f", "null", "-"]


def _avfoundation_runtime_argv(settings: CameraSettings, executable: str) -> list[str]:
    """Run continuously, emitting metadata on stdout while discarding video."""

    argv = _avfoundation_argv(settings, executable)
    frames = argv.index("-frames:v")
    del argv[frames:frames + 2]
    return [*argv[:-4], "-nostats", "-progress", "pipe:1", *argv[-4:]]


def probe_source(settings: CameraSettings, *, runner=subprocess.run, which=shutil.which,
                 clock=time.monotonic, timeout: float = PROBE_TIMEOUT_SECONDS) -> dict:
    """Probe once; discard all child output and return only bounded public metadata."""

    started = clock()
    reason = "camera_source_available"
    ok = True
    if settings.provider == "ros_image":
        try:
            __import__("sensor_msgs.msg")
        except (ImportError, ModuleNotFoundError):
            ok, reason = False, "camera_dependency_missing"
        else:
            reason = "camera_dependency_available"
    elif settings.provider == "avfoundation":
        executable = which("ffmpeg")
        if not executable:
            ok, reason = False, "camera_dependency_missing"
        else:
            try:
                completed = runner(
                    _avfoundation_argv(settings, executable), stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=min(max(float(timeout), 0.1), PROBE_TIMEOUT_SECONDS),
                    check=False, shell=False,
                )
                ok = completed.returncode == 0
                reason = "camera_source_available" if ok else "camera_device_unavailable"
            except subprocess.TimeoutExpired:
                ok, reason = False, "camera_probe_timeout"
            except OSError:
                ok, reason = False, "camera_probe_failed"
    else:  # Defensive: constructed settings need not have come from from_environ().
        ok, reason = False, "camera_provider_invalid"
    elapsed_ms = max(0, min(int((clock() - started) * 1000), 60_000))
    usable = ok and settings.provider == "avfoundation"
    return {"action_code": "none", "elapsed_ms": elapsed_ms, "ok": ok,
            "provider": settings.provider if settings.provider in PROVIDERS else "invalid",
            "reason": reason, "source_id": settings.source_id, "usable": usable}


class AvfoundationRuntime:
    """Continuously turn bounded ffmpeg frame proofs into metadata-only observations."""

    def __init__(self, settings: CameraSettings, observation, *, popen=subprocess.Popen,
                 which=shutil.which, wait_seconds: float = MIN_RESTART_BACKOFF_SECONDS,
                 clock=time.monotonic, stop_event=None):
        if settings.provider != "avfoundation":
            raise CameraConfigurationError("camera_provider_invalid")
        self.settings = settings
        self.observation = observation
        self.popen = popen
        self.which = which
        self.wait_seconds = min(
            max(float(wait_seconds), MIN_RESTART_BACKOFF_SECONDS),
            MAX_RESTART_BACKOFF_SECONDS,
        )
        self.clock = clock
        self._stop = threading.Event() if stop_event is None else stop_event
        self._thread: threading.Thread | None = None
        self._process = None

    def _consume_progress(self, process) -> bool:
        """Accept frame counters from a bounded byte parser; retain no child text."""

        stream = getattr(process, "stdout", None)
        if stream is None or not callable(getattr(stream, "read", None)):
            return False
        pending = bytearray()
        proved_frame = False
        last_frame = 0
        while not self._stop.is_set():
            try:
                chunk = stream.read(PROGRESS_READ_SIZE)
            except (AttributeError, OSError, ValueError):
                return False
            if not chunk:
                break
            if isinstance(chunk, str):
                chunk = chunk.encode("ascii", "ignore")
            pending.extend(chunk[:PROGRESS_READ_SIZE])
            if len(pending) > MAX_PROGRESS_BUFFER:
                del pending[:-MAX_PROGRESS_BUFFER]
            while b"\n" in pending:
                line, _, remainder = pending.partition(b"\n")
                pending[:] = remainder
                key, separator, value = line.rstrip(b"\r").partition(b"=")
                if (
                    key == b"frame"
                    and separator
                    and 0 < len(value) <= MAX_FRAME_COUNTER_DIGITS
                    and value.isascii()
                    and value.isdigit()
                ):
                    frame = int(value)
                    if not last_frame < frame <= MAX_FRAME_COUNTER:
                        continue
                    last_frame = frame
                    self.observation.accept_frame()
                    proved_frame = True
        return proved_frame

    def _start_process(self, executable: str):
        return self.popen(
            _avfoundation_runtime_argv(self.settings, executable),
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            shell=False, bufsize=0,
        )

    @staticmethod
    def _stop_process(process) -> None:
        """Bound process exit; a broken progress pipe must never strand the runtime."""

        with contextlib.suppress(AttributeError, OSError, ValueError):
            if process.poll() is None:
                process.terminate()
        try:
            process.wait(timeout=PROCESS_EXIT_TIMEOUT_SECONDS)
            return
        except (AttributeError, subprocess.TimeoutExpired, OSError, ValueError):
            pass
        with contextlib.suppress(AttributeError, OSError, ValueError):
            if process.poll() is None:
                process.kill()
        with contextlib.suppress(
            AttributeError, subprocess.TimeoutExpired, OSError, ValueError,
        ):
            process.wait(timeout=PROCESS_EXIT_TIMEOUT_SECONDS)

    def _run(self) -> None:
        backoff = self.wait_seconds
        while not self._stop.is_set():
            executable = self.which("ffmpeg")
            if not executable:
                self.observation.clear()
                self._stop.wait(backoff)
                backoff = min(backoff * 2, MAX_RESTART_BACKOFF_SECONDS)
                continue
            process = None
            proved_frame = False
            started = self.clock()
            try:
                process = self._start_process(executable)
                self._process = process
                if self._stop.is_set() and process.poll() is None:
                    process.terminate()
                proved_frame = self._consume_progress(process)
            except (AttributeError, OSError, ValueError):
                pass
            finally:
                if process is not None:
                    self._stop_process(process)
                self._process = None
                self.observation.clear()
            durable = self.clock() - started >= DURABLE_RUN_SECONDS
            backoff = self.wait_seconds if proved_frame or durable else min(
                backoff * 2, MAX_RESTART_BACKOFF_SECONDS,
            )
            self._stop.wait(backoff)

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="camera-avfoundation", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        process = self._process
        if process is not None:
            with contextlib.suppress(OSError, ValueError):
                if process.poll() is None:
                    process.terminate()
        if self._thread is not None:
            self._thread.join(PROBE_TIMEOUT_SECONDS)
        if process is not None:
            self._stop_process(process)
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(1.0)
