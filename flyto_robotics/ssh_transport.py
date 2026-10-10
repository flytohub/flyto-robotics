"""The SSH local forward that carries the adapter to a loopback-bound robot.

Everything the robot serves (rosbridge, camera) binds to 127.0.0.1 on the
robot, so the tunnel is the transport, not a workaround for one. This module
owns it: configured by data (environment), started by the adapter, supervised
and restarted with exponential backoff and jitter when it exits (robot reboot,
Wi-Fi drop), and torn down with the last adapter that holds it.

Security, enforced here and nowhere else:

- key-only: ``BatchMode=yes``, password and keyboard-interactive off;
- ``StrictHostKeyChecking=yes`` against known_hosts, never ``accept-new``: an
  unknown or changed host key is a ``failed`` state with the commands to trust
  the robot once, not a silent trust;
- loopback only: every forward is ``127.0.0.1:<local> -> 127.0.0.1:<remote>``
  and a configured non-loopback address on either side is refused;
- ``-N``: no remote command is ever run.

No ROS here: the adapter asks for a URL and a state, nothing more.
"""

from __future__ import annotations

import atexit
import contextlib
import hashlib
import os
import random
import re
import shutil
import socket
import stat
import subprocess
import tempfile
import threading
import time
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Protocol

# -- configuration -----------------------------------------------------------------

ENV_HOST = "FLYTO_ROS2_SSH_HOST"
ENV_IDENTITY = "FLYTO_ROS2_SSH_IDENTITY"
ENV_KNOWN_HOSTS = "FLYTO_ROS2_SSH_KNOWN_HOSTS"
ENV_FORWARDS = "FLYTO_ROS2_SSH_FORWARDS"
ENV_LOCAL_PORTS = "FLYTO_ROS2_SSH_LOCAL_PORTS"
ENV_READY_TIMEOUT = "FLYTO_ROS2_SSH_READY_TIMEOUT_S"
ENV_LINGER = "FLYTO_ROS2_SSH_LINGER_S"
ENV_ROSBRIDGE_URL = "FLYTO_ROSBRIDGE_URL"

ROSBRIDGE = "rosbridge"
DEFAULT_FORWARDS = "rosbridge=9090"
LOCAL_BIND = "127.0.0.1"
REMOTE_BIND = "127.0.0.1"
LOOPBACK_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})

_HOST_PATTERN = re.compile(r"^(?:[A-Za-z0-9._-]+@)?[A-Za-z0-9][A-Za-z0-9.:-]*$")
_NAME_PATTERN = re.compile(r"^[a-z][a-z0-9_-]{0,31}$")

# -- states --------------------------------------------------------------------------

STATE_CONNECTED = "connected"
STATE_RECONNECTING = "reconnecting"
STATE_FAILED = "failed"
STATE_STOPPED = "stopped"

REASON_TRANSPORT_UNAVAILABLE = "transport_unavailable"

# (base seconds, multiplier, ceiling seconds); each delay is drawn from
# [delay / 2, delay] so hosts that lost the same robot do not retry in step.
BACKOFF = (1.0, 2.0, 30.0)

# What ssh's own options must be, whatever ~/.ssh/config says: the command line
# is read first and the first value obtained wins.
SSH_OPTIONS: tuple[tuple[str, str], ...] = (
    ("BatchMode", "yes"),
    ("StrictHostKeyChecking", "yes"),
    ("PasswordAuthentication", "no"),
    ("KbdInteractiveAuthentication", "no"),
    ("PubkeyAuthentication", "yes"),
    ("ExitOnForwardFailure", "yes"),
    ("ServerAliveInterval", "5"),
    ("ServerAliveCountMax", "3"),
    ("ConnectTimeout", "5"),
    ("ControlMaster", "yes"),
    ("ControlPersist", "no"),
    ("ForwardAgent", "no"),
    ("ForwardX11", "no"),
    ("PermitLocalCommand", "no"),
    ("GatewayPorts", "no"),
    ("RequestTTY", "no"),
    ("LogLevel", "ERROR"),
)

# ssh's stderr -> (error code, fatal, operator text). Fatal errors stop the
# automatic retries (an auth failure retried forever trips fail2ban and never
# heals); the operator fixes the cause and asks for a reconnect.
_ERROR_TABLE: tuple[tuple[str, str, bool, str], ...] = (
    (
        "REMOTE HOST IDENTIFICATION HAS CHANGED",
        "host_key_changed",
        True,
        "the host key of {host} changed since it was trusted; refused. If the robot "
        "was reinstalled, remove the old key (ssh-keygen -R {hostname}) and trust the "
        "new one: {trust}",
    ),
    (
        "Host key verification failed",
        "host_key_unknown",
        True,
        "the host key of {host} is not in known_hosts (StrictHostKeyChecking=yes); "
        "trust it once: {trust}",
    ),
    (
        "host key is known",
        "host_key_unknown",
        True,
        "the host key of {host} is not in known_hosts (StrictHostKeyChecking=yes); "
        "trust it once: {trust}",
    ),
    (
        "Permission denied",
        "auth_refused",
        True,
        "{host} refused this computer's key (key-only login); install the public key "
        "of {identity} in the robot's ~/.ssh/authorized_keys",
    ),
    (
        "Could not resolve hostname",
        "unresolved",
        False,
        "cannot resolve {hostname} (mDNS); is the robot on this network?",
    ),
    (
        "cannot listen to port",
        "local_port_busy",
        False,
        "a local port of the forward is already in use on this computer",
    ),
    (
        "Address already in use",
        "local_port_busy",
        False,
        "a local port of the forward is already in use on this computer",
    ),
    ("Connection refused", "refused", False, "{hostname} refused SSH (booting?)"),
    ("timed out", "timeout", False, "{hostname} did not answer SSH in time"),
    ("No route to host", "no_route", False, "no route to {hostname}"),
    ("Network is unreachable", "no_route", False, "network unreachable for {hostname}"),
)


class TransportConfigError(ValueError):
    """The SSH transport configuration is refused (never silently adjusted)."""


@dataclass(frozen=True)
class Forward:
    name: str
    remote_port: int
    local_port: int = 0  # 0: an ephemeral loopback port chosen at start


@dataclass(frozen=True)
class TransportConfig:
    host: str
    forwards: tuple[Forward, ...]
    identity: str = ""
    known_hosts: str = ""
    ready_timeout_s: float = 10.0
    linger_s: float = 30.0

    @property
    def hostname(self) -> str:
        return self.host.rsplit("@", 1)[-1]

    def forward(self, name: str) -> Forward:
        for item in self.forwards:
            if item.name == name:
                return item
        raise KeyError(name)


def _port(text: str, *, what: str, allow_zero: bool) -> int:
    try:
        value = int(text)
    except ValueError as error:
        raise TransportConfigError(f"{what}: {text!r} is not a port") from error
    if not (0 if allow_zero else 1) <= value <= 65535:
        raise TransportConfigError(f"{what}: {value} is out of range")
    return value


def _loopback_endpoint(text: str, *, what: str, allow_zero: bool) -> int:
    """``port`` or ``loopback-host:port``; any other host is refused."""
    host, sep, port = text.strip().rpartition(":")
    if sep and host.strip("[]") not in LOOPBACK_HOSTS:
        raise TransportConfigError(
            f"{what}: {host!r} is not loopback; only 127.0.0.1 forwards are allowed"
        )
    return _port(port, what=what, allow_zero=allow_zero)


def _pairs(raw: str, *, what: str) -> dict[str, str]:
    pairs: dict[str, str] = {}
    for item in filter(None, (part.strip() for part in raw.split(","))):
        name, sep, value = item.partition("=")
        name = name.strip().lower()
        if not sep or not _NAME_PATTERN.fullmatch(name):
            raise TransportConfigError(f"{what}: {item!r} is not name=port")
        if name in pairs:
            raise TransportConfigError(f"{what}: {name!r} is given twice")
        pairs[name] = value.strip()
    return pairs


def _forwards(raw: str, local_raw: str) -> tuple[Forward, ...]:
    remote = _pairs(raw or DEFAULT_FORWARDS, what=ENV_FORWARDS)
    remote.setdefault(ROSBRIDGE, DEFAULT_FORWARDS.split("=", 1)[1])
    local = _pairs(local_raw, what=ENV_LOCAL_PORTS)
    unknown = sorted(set(local) - set(remote))
    if unknown:
        raise TransportConfigError(f"{ENV_LOCAL_PORTS}: no forward named {unknown[0]!r}")
    forwards = tuple(
        Forward(
            name=name,
            remote_port=_loopback_endpoint(value, what=f"{ENV_FORWARDS} {name}", allow_zero=False),
            local_port=_loopback_endpoint(
                local.get(name, "0"), what=f"{ENV_LOCAL_PORTS} {name}", allow_zero=True
            ),
        )
        for name, value in sorted(remote.items())
    )
    pinned = [item.local_port for item in forwards if item.local_port]
    if len(pinned) != len(set(pinned)):
        raise TransportConfigError(f"{ENV_LOCAL_PORTS}: two forwards share a local port")
    return forwards


def _seconds(env: Mapping[str, str], name: str, default: float, high: float) -> float:
    raw = env.get(name, "").strip()
    if not raw:
        return default
    try:
        return min(high, max(0.0, float(raw)))
    except ValueError as error:
        raise TransportConfigError(f"{name}: {raw!r} is not a number") from error


def config_from_env(env: Mapping[str, str] | None = None) -> TransportConfig | None:
    """The configured transport, None when ``FLYTO_ROS2_SSH_HOST`` is unset."""
    env = os.environ if env is None else env
    host = env.get(ENV_HOST, "").strip()
    if not host:
        return None
    if host.startswith("-") or not _HOST_PATTERN.fullmatch(host):
        raise TransportConfigError(f"{ENV_HOST}: {host!r} is not [user@]host")
    identity = os.path.expanduser(env.get(ENV_IDENTITY, "").strip())
    if identity and not os.path.isfile(identity):
        raise TransportConfigError(f"{ENV_IDENTITY}: {identity!r} is not a file")
    known_hosts = os.path.expanduser(env.get(ENV_KNOWN_HOSTS, "").strip())
    config = TransportConfig(
        host=host,
        forwards=_forwards(env.get(ENV_FORWARDS, ""), env.get(ENV_LOCAL_PORTS, "")),
        identity=identity,
        known_hosts=known_hosts,
        ready_timeout_s=_seconds(env, ENV_READY_TIMEOUT, 10.0, 120.0),
        linger_s=_seconds(env, ENV_LINGER, 30.0, 3600.0),
    )
    _check_url(config, env.get(ENV_ROSBRIDGE_URL, "").strip())
    return config


def _check_url(config: TransportConfig, url: str) -> None:
    """An explicit rosbridge URL must be the one the forward derives."""
    if not url:
        return
    pinned = config.forward(ROSBRIDGE).local_port
    expected = {f"ws://{host}:{pinned}" for host in ("127.0.0.1", "localhost")}
    if not pinned or url.rstrip("/") not in expected:
        raise TransportConfigError(
            f"{ENV_ROSBRIDGE_URL}={url!r} contradicts {ENV_HOST}: with the SSH transport the "
            f"URL is derived from the forward; unset it, or pin {ENV_LOCAL_PORTS}=rosbridge=<port> "
            "to the same port"
        )


def ssh_argv(
    config: TransportConfig, local_ports: Mapping[str, int], control_path: str
) -> list[str]:
    """The ssh command line: forwards only, loopback on both ends, no command."""
    argv = ["ssh", "-N", "-T"]
    for key, value in SSH_OPTIONS:
        argv += ["-o", f"{key}={value}"]
    argv += ["-o", f"ControlPath={control_path}"]
    if config.identity:
        argv += ["-i", config.identity, "-o", "IdentitiesOnly=yes"]
    if config.known_hosts:
        argv += ["-o", f"UserKnownHostsFile={config.known_hosts}"]
    for item in config.forwards:
        argv += ["-L", f"{LOCAL_BIND}:{local_ports[item.name]}:{REMOTE_BIND}:{item.remote_port}"]
    # "--" so a host can never be read as an option.
    return [*argv, "--", config.host]


def classify(stderr: str, config: TransportConfig) -> tuple[str, bool, str]:
    """(error code, fatal, operator text) for what ssh said when it exited."""
    trust = (
        f"compare `ssh-keyscan -t ed25519 {config.hostname} | ssh-keygen -lf -` with "
        "`ssh-keygen -lf /etc/ssh/ssh_host_ed25519_key.pub` run on the robot itself, "
        f"then append that ssh-keyscan line to {config.known_hosts or '~/.ssh/known_hosts'}"
    )
    values = {
        "host": config.host,
        "hostname": config.hostname,
        "identity": config.identity or "the default key",
        "trust": trust,
    }
    for needle, code, fatal, text in _ERROR_TABLE:
        if needle.lower() in stderr.lower():
            return code, fatal, text.format(**values)
    tail = " ".join(stderr.split())[-300:]
    return "ssh_exited", False, f"ssh to {config.hostname} exited: {tail or 'no output'}"


def backoff_delay(attempts: int, rand: Callable[[], float]) -> float:
    base, factor, ceiling = BACKOFF
    delay = min(ceiling, base * factor ** max(0, attempts - 1))
    return delay * (0.5 + 0.5 * rand())


# -- process seams (replaced by fakes in tests) --------------------------------------


class SshProcess(Protocol):
    def alive(self) -> bool: ...

    def stop(self) -> None: ...

    def error_text(self) -> str: ...


class _PopenProcess:
    """ssh as a child process; ``on_exit`` fires from a watcher thread."""

    def __init__(self, argv: list[str], on_exit: Callable[[], None]) -> None:
        self._popen = subprocess.Popen(  # noqa: S603 - argv built by ssh_argv only
            argv,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        self._stderr = ""
        threading.Thread(target=self._watch, args=(on_exit,), daemon=True).start()

    def _watch(self, on_exit: Callable[[], None]) -> None:
        stream = self._popen.stderr
        if stream is not None:
            with contextlib.suppress(Exception):
                self._stderr = stream.read()[-4000:]
        self._popen.wait()
        on_exit()

    def alive(self) -> bool:
        return self._popen.poll() is None

    def stop(self) -> None:
        if self._popen.poll() is not None:
            return
        self._popen.terminate()
        try:
            self._popen.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._popen.kill()
            with contextlib.suppress(subprocess.TimeoutExpired):
                self._popen.wait(timeout=2.0)

    def error_text(self) -> str:
        return self._stderr


def _spawn(argv: list[str], on_exit: Callable[[], None]) -> SshProcess:
    if shutil.which(argv[0]) is None:
        raise TransportConfigError("no ssh client on PATH")
    return _PopenProcess(argv, on_exit)


def _probe(port: int) -> bool:
    with contextlib.suppress(OSError), socket.create_connection((LOCAL_BIND, port), timeout=0.5):
        return True
    return False


def _resolve(hostname: str) -> str:
    return str(socket.getaddrinfo(hostname, 22, proto=socket.IPPROTO_TCP)[0][4][0])


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind((LOCAL_BIND, 0))
        return int(probe.getsockname()[1])


def _reap_stale_master(control_path: str, host: str) -> None:
    """Stop a master a crashed host left holding the forward's ports."""
    if not os.path.exists(control_path):
        return
    with contextlib.suppress(Exception):
        subprocess.run(  # noqa: S603 - local control socket only, no remote command
            ["ssh", "-o", f"ControlPath={control_path}", "-O", "exit", "--", host],
            stdin=subprocess.DEVNULL,
            capture_output=True,
            timeout=5,
            check=False,
        )
    with contextlib.suppress(OSError):
        os.unlink(control_path)


def _control_dir() -> str:
    path = os.path.join(tempfile.gettempdir(), f"flyto-ssh-{os.getuid()}")
    os.makedirs(path, mode=0o700, exist_ok=True)
    info = os.lstat(path)
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise TransportConfigError(f"{path} must be a private directory owned by this user")
    return path


def _control_path(config: TransportConfig) -> str:
    digest = hashlib.sha256(repr((config.host, config.forwards)).encode()).hexdigest()[:16]
    return os.path.join(_control_dir(), f"{digest}.sock")


@dataclass
class Seams:
    spawn: Callable[[list[str], Callable[[], None]], SshProcess] = _spawn
    probe: Callable[[int], bool] = _probe
    resolve: Callable[[str], str] = _resolve
    free_port: Callable[[], int] = _free_port
    reap: Callable[[str, str], None] = _reap_stale_master
    control_path: Callable[[TransportConfig], str] = _control_path
    backoff: Callable[[int], float] = lambda attempts: backoff_delay(attempts, random.random)
    now: Callable[[], float] = time.time
    poll_s: float = 0.1


# -- the supervised tunnel -----------------------------------------------------------


def _iso(seconds: float) -> str:
    return datetime.fromtimestamp(seconds, timezone.utc).isoformat()


@dataclass
class _Status:
    state: str
    since: float
    attempts: int = 0
    last_error: str = ""
    error_code: str = ""
    resolved_address: str = ""


class CallGate:
    """Capability calls admitted on one link, and whether it admits more.

    An operator refresh pauses the gate (new calls fail fast), stops the
    actuating calls still running, and only then tears the link down.
    """

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._paused = False
        self._active: dict[Any, bool] = {}  # token -> actuating

    @property
    def paused(self) -> bool:
        with self._lock:
            return self._paused

    def begin(self, token: Any, *, actuating: bool) -> bool:
        with self._lock:
            if self._paused:
                return False
            self._active[token] = actuating
            return True

    def end(self, token: Any) -> None:
        with self._lock:
            self._active.pop(token, None)

    def pause(self) -> list[Any]:
        """Refuse new calls; return the actuating calls still running."""
        with self._lock:
            self._paused = True
            return [token for token, actuating in self._active.items() if actuating]

    def resume(self) -> None:
        with self._lock:
            self._paused = False


class ManagedTunnel:
    """One supervised ``ssh -N -L ...`` for one configuration."""

    def __init__(self, config: TransportConfig, seams: Seams | None = None) -> None:
        self.config = config
        self._seams = seams or Seams()
        self._lock = threading.Condition()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._refresh = threading.Event()
        self._status = _Status(STATE_RECONNECTING, self._seams.now())
        self._ever_connected = False
        self._process: SshProcess | None = None
        self._ports = {
            item.name: item.local_port or self._seams.free_port() for item in config.forwards
        }
        self._control_path = self._seams.control_path(config)
        self._listeners: list[Callable[[dict[str, Any]], None]] = []
        self.calls = CallGate()
        self._holders = 0
        self._linger: threading.Timer | None = None
        self._thread = threading.Thread(target=self._run, name="ros2-ssh-transport", daemon=True)
        self._thread.start()

    # -- what the adapter reads

    def url(self, name: str = ROSBRIDGE) -> str:
        with self._lock:
            return f"ws://{LOCAL_BIND}:{self._ports[name]}"

    def local_port(self, name: str) -> int:
        with self._lock:
            return self._ports[name]

    @property
    def connected(self) -> bool:
        with self._lock:
            return self._status.state == STATE_CONNECTED

    def status(self) -> dict[str, Any]:
        with self._lock:
            current = self._status
            return {
                "transport": "ssh",
                "state": current.state,
                "since": _iso(current.since),
                "attempts": current.attempts,
                "last_error": current.last_error,
                "error_code": current.error_code,
                "host": self.config.host,
                "resolved_address": current.resolved_address,
                "forwards": {
                    item.name: {
                        "local": f"{LOCAL_BIND}:{self._ports[item.name]}",
                        "remote": f"{REMOTE_BIND}:{item.remote_port}",
                    }
                    for item in self.config.forwards
                },
                "accepting_calls": not self.calls.paused,
            }

    def add_listener(self, listener: Callable[[dict[str, Any]], None]) -> None:
        with self._lock:
            self._listeners.append(listener)

    def remove_listener(self, listener: Callable[[dict[str, Any]], None]) -> None:
        with self._lock, contextlib.suppress(ValueError):
            self._listeners.remove(listener)

    def wait_connected(self, timeout: float) -> bool:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._lock:
            while self._status.state != STATE_CONNECTED and not self._stop.is_set():
                left = deadline - time.monotonic()
                if left <= 0 or self._status.state == STATE_FAILED:
                    break
                self._lock.wait(min(left, 0.5))
            return self._status.state == STATE_CONNECTED

    def wait_first(self) -> bool:
        """Wait for a first connection only; once ever connected, fail fast."""
        with self._lock:
            first = not self._ever_connected
        return self.wait_connected(self.config.ready_timeout_s if first else 0.0)

    # -- lifecycle

    def refresh(self) -> None:
        """Tear the forward down now and establish it again, attempts reset."""
        self._reset_attempts()
        self._refresh.set()
        self._wake.set()

    def hold(self) -> None:
        with self._lock:
            self._holders += 1
            if self._linger is not None:
                self._linger.cancel()
                self._linger = None

    def release(self) -> None:
        with self._lock:
            self._holders = max(0, self._holders - 1)
            if self._holders or self._stop.is_set():
                return
            if self.config.linger_s <= 0:
                closing = True
            else:
                closing = False
                self._linger = threading.Timer(self.config.linger_s, self._linger_expired)
                self._linger.daemon = True
                self._linger.start()
        if closing:
            _forget(self)

    def _linger_expired(self) -> None:
        _forget(self)

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        if threading.current_thread() is not self._thread:
            self._thread.join(timeout=5.0)
        self._stop_process()
        self._set(STATE_STOPPED, error="")

    # -- supervisor

    def _run(self) -> None:
        while not self._stop.is_set():
            self._wake.clear()
            if self._refresh.is_set():
                self._refresh.clear()
                self._reset_attempts()
            self._attempt()
            # The exit that ended this attempt also woke us; only a stop or a
            # refresh may cut the back-off short, and both have their own flag.
            self._wake.clear()
            if self._stop.is_set():
                break
            if self._refresh.is_set():
                continue
            with self._lock:
                failed = self._status.state == STATE_FAILED
                attempts = self._status.attempts
            # A fatal error waits for the operator; a transient one backs off.
            self._wake.wait(None if failed else self._seams.backoff(attempts))
        self._stop_process()

    def _attempt(self) -> None:
        try:
            address = self._seams.resolve(self.config.hostname)
        except Exception:  # noqa: BLE001 - mDNS answers again after a reboot
            self._fail(*classify("Could not resolve hostname", self.config))
            return
        with self._lock:
            self._status.resolved_address = address
        self._seams.reap(self._control_path, self.config.host)
        argv = ssh_argv(self.config, self.local_ports(), self._control_path)
        try:
            process = self._seams.spawn(argv, self._wake.set)
        except TransportConfigError as error:
            self._fail("ssh_missing", True, str(error))
            return
        self._process = process
        if not self._await_ready(process):
            return
        self._set(STATE_CONNECTED, error=None)
        while process.alive() and not self._stop.is_set() and not self._refresh.is_set():
            self._wake.wait()
            self._wake.clear()
        if self._stop.is_set() or self._refresh.is_set():
            self._stop_process()
            if self._refresh.is_set():
                self._set(STATE_RECONNECTING, error=None)
            return
        self._exited(process)

    def local_ports(self) -> dict[str, int]:
        with self._lock:
            return dict(self._ports)

    def _await_ready(self, process: SshProcess) -> bool:
        deadline = time.monotonic() + self.config.ready_timeout_s
        while True:
            if self._stop.is_set() or self._refresh.is_set():
                self._stop_process()
                return False
            if not process.alive():
                self._exited(process)
                return False
            if all(self._seams.probe(port) for port in self.local_ports().values()):
                return True
            if time.monotonic() >= deadline:
                self._stop_process()
                self._fail("not_ready", False, f"forward to {self.config.hostname} not ready")
                return False
            self._wake.wait(self._seams.poll_s)
            self._wake.clear()

    def _exited(self, process: SshProcess) -> None:
        self._process = None
        code, fatal, text = classify(process.error_text(), self.config)
        if code == "local_port_busy":
            self._repick_ephemeral_ports()
        self._fail(code, fatal, text)

    def _repick_ephemeral_ports(self) -> None:
        with self._lock:
            for item in self.config.forwards:
                if not item.local_port:
                    self._ports[item.name] = self._seams.free_port()

    def _stop_process(self) -> None:
        process, self._process = self._process, None
        if process is not None:
            with contextlib.suppress(Exception):
                process.stop()

    def _reset_attempts(self) -> None:
        with self._lock:
            self._status.attempts = 0
            self._ever_connected = False
        self._set(STATE_RECONNECTING, error=None)

    def _fail(self, code: str, fatal: bool, text: str) -> None:
        with self._lock:
            self._status.attempts += 1
            self._status.error_code = code
        self._set(STATE_FAILED if fatal else STATE_RECONNECTING, error=text)

    def _set(self, state: str, *, error: str | None) -> None:
        with self._lock:
            changed = state != self._status.state
            if changed:
                self._status.state = state
                self._status.since = self._seams.now()
            if state == STATE_CONNECTED:
                self._status.attempts = 0
                self._ever_connected = True
            if error is not None:
                self._status.last_error = error
            self._lock.notify_all()
            listeners = list(self._listeners)
        if not changed:
            return
        snapshot = self.status()
        for listener in listeners:
            with contextlib.suppress(Exception):
                listener(snapshot)


# -- one tunnel per configuration, held by the adapters that use it ------------------

_tunnels: dict[TransportConfig, ManagedTunnel] = {}
_tunnels_lock = threading.Lock()


def acquire(
    config: TransportConfig | None = None, *, seams: Seams | None = None
) -> ManagedTunnel | None:
    """The running tunnel for ``config`` (from the environment by default), held.

    None when no SSH transport is configured. Raises ``TransportConfigError``
    for a configuration that is refused. Every ``acquire`` is paired with
    ``tunnel.release()``; the tunnel stops ``linger_s`` after the last one.
    """
    config = config if config is not None else config_from_env()
    if config is None:
        return None
    with _tunnels_lock:
        tunnel = _tunnels.get(config)
        if tunnel is None:
            tunnel = ManagedTunnel(config, seams)
            _tunnels[config] = tunnel
        tunnel.hold()
        return tunnel


def current(config: TransportConfig | None = None) -> ManagedTunnel | None:
    """The tunnel already running for ``config``, without holding it."""
    config = config if config is not None else config_from_env()
    if config is None:
        return None
    with _tunnels_lock:
        return _tunnels.get(config)


def _forget(tunnel: ManagedTunnel) -> None:
    with _tunnels_lock:
        with tunnel._lock:
            if tunnel._holders:
                return  # held again while its linger ran out
        if _tunnels.get(tunnel.config) is tunnel:
            del _tunnels[tunnel.config]
    tunnel.close()


def close_all() -> None:
    with _tunnels_lock:
        tunnels = list(_tunnels.values())
        _tunnels.clear()
    for tunnel in tunnels:
        tunnel.close()


# A host that exits must not leave an ssh child holding the robot's ports.
atexit.register(close_all)


def unavailable_detail(status: Mapping[str, Any]) -> str:
    error = str(status.get("last_error") or "").strip()
    return (
        f"{REASON_TRANSPORT_UNAVAILABLE}: the SSH transport to {status.get('host')} is "
        f"{status.get('state')}" + (f" ({error})" if error else "") + "; nothing was sent"
    )

