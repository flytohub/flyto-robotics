"""Is the release that just became ``current`` actually working?

``systemctl is-active`` answers a narrower question than anyone reading a green
install report believes it does: it says the process has not exited. A service
that starts, cannot find its identity file, logs a line nobody reads, and sits
in its retry loop is *active*. So is one whose release directory is missing the
module its ``ExecStart=`` names, right up until the moment systemd tries to run
it again after a reboot.

This module answers the wider question, and it answers it from **data**. The
checks live in the profile registry beside the units they belong to, so a site
running something this build has never heard of declares its own readiness in
the same file it declares its units, and nothing here imports a middleware,
names a transport, or knows ROS exists.

Two outcomes are deliberately not the same thing:

``provisioning_pending``
    Everything that depends on the *release* is fine; what is missing is the
    pairing a human does afterwards -- the device identity, the cloud URL, the
    credential. This is the normal state of every machine between installation
    and commissioning. Rolling a good release back because nobody had typed the
    cloud URL in yet would make the installer unusable in the field.

``unhealthy``
    Something that does not depend on pairing is wrong: the active release does
    not carry the package its units execute, the configuration the units read is
    absent. That is the release failing on this device, and it is exactly what
    the activation transaction exists to undo.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from .lifecycle_profiles import Profile, ReadinessCheck

__all__ = [
    "READINESS_VERSION",
    "Readiness",
    "evaluate",
    "read_config",
]

READINESS_VERSION = "flyto.readiness.v1"

#: States, worst last. ``evaluate`` reports the worst one it finds.
READY = "ready"
PROVISIONING_PENDING = "provisioning_pending"
UNHEALTHY = "unhealthy"


@dataclass(frozen=True)
class Readiness:
    """The verdict, plus every check that produced it."""

    state: str
    checks: tuple[dict, ...]

    @property
    def ok(self) -> bool:
        """True when the *release* is working, paired or not.

        ``provisioning_pending`` is an ``ok`` state on purpose: an installer
        that refused to finish until someone had already commissioned the device
        could never be the thing that commissions it.
        """

        return self.state != UNHEALTHY

    def as_dict(self) -> dict:
        return {
            "schema": READINESS_VERSION,
            "state": self.state,
            "ok": self.ok,
            "checks": [dict(check) for check in self.checks],
        }

    def failures(self) -> tuple[str, ...]:
        return tuple(check["id"] for check in self.checks if not check["passed"])


def read_config(path: Path) -> dict[str, str]:
    """Parse a ``KEY=value`` environment file the way systemd's does.

    Comments and blank lines are ignored, surrounding quotes are stripped, and
    an unreadable file is an empty mapping rather than an exception: "the
    configuration says nothing" is a readiness answer, not a crash.
    """

    values: dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return values
    for line in text.splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        values[key.strip()] = value.strip().strip("'\"")
    return values


def _passed(check: ReadinessCheck, fields: dict[str, str], config: dict[str, str]) -> bool:
    if check.kind == "path_exists":
        return Path(check.target.format(**fields)).exists()
    if check.kind == "config_value_set":
        return bool(config.get(check.target, "").strip())
    # Unreachable: the registry loader refuses an unknown kind. Failing closed
    # anyway, because a check nobody can evaluate is not a check that passed.
    return False


def evaluate(profile: Profile, fields: dict[str, str], *, config_file: Path) -> Readiness:
    """Run ``profile``'s readiness checks against a rendered set of paths.

    ``fields`` is the same substitution mapping the unit templates are rendered
    from, so a readiness check and the unit it guards can never disagree about
    where a file lives.
    """

    config = read_config(config_file)
    checks: list[dict] = []
    state = READY
    for check in profile.readiness:
        passed = _passed(check, fields, config)
        checks.append(
            {
                "id": check.id,
                "kind": check.kind,
                "target": check.target,
                "description": check.description,
                "provisioning": check.provisioning,
                "passed": passed,
            }
        )
        if passed:
            continue
        if check.provisioning:
            if state == READY:
                state = PROVISIONING_PENDING
        else:
            state = UNHEALTHY
    return Readiness(state=state, checks=tuple(checks))
