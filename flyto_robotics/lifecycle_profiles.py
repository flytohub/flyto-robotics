"""Data-driven unit profiles for the product lifecycle.

The generic lifecycle must not know that ROS 2 exists. If it did, every customer
who runs something else would be installing a ROS-shaped product with the ROS
parts switched off, and the first non-ROS transport would arrive as a patch to
``lifecycle.py`` rather than as a file they can drop in.

So the unit set is *data*: :mod:`flyto_robotics.lifecycle` renders whatever
``data/lifecycle-profiles.json`` declares and never names a middleware. ROS 2 is
one profile in that file, additive by construction -- ``extends`` is verified to
leave the base units byte-identical, because a profile that quietly rewrote a
unit a site already runs would be an upgrade disguised as an option.

Sites with their own transport point ``--profiles`` at their own file and get
the identical ``install`` / ``update`` / ``rollback`` commands.
"""

from __future__ import annotations

import json
import string
from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

__all__ = [
    "LIFECYCLE_PROFILES_VERSION",
    "ActivationCondition",
    "PROFILE_FIELDS",
    "READINESS_KINDS",
    "Profile",
    "ProfileError",
    "ReadinessCheck",
    "UnitSpec",
    "default_profiles_path",
    "load_profiles",
    "runbook_text",
]

LIFECYCLE_PROFILES_VERSION = "flyto.lifecycle-profiles.v1"

#: Substitution names a template may use. Anything else is a typo, and a typo in
#: a unit template is an ``ExecStart=`` that points nowhere.
PROFILE_FIELDS = (
    "current",
    "config_dir",
    "config_file",
    "identity_file",
    "state_dir",
    "log_dir",
    "python",
)

#: What a readiness check is allowed to look at. Deliberately two predicates and
#: no escape hatch: a registry that could run an arbitrary command would be a
#: remote-code-execution surface that every site edits, and a check that needed a
#: middleware to answer would drag the transport back into the generic path.
READINESS_KINDS = ("path_exists", "config_value_set")
ACTIVATION_CONDITION_KINDS = ("path_exists",)
MAX_ACTIVATION_CONDITION_PATH = 4096


class ProfileError(ValueError):
    """The profile registry is malformed. Carries a lifecycle reason code."""

    reason = "profiles_invalid"


@dataclass(frozen=True)
class ActivationCondition:
    """A bounded, declarative condition attached to activation policy."""

    kind: str
    path: str

    def render(self, fields: dict[str, str]) -> Path:
        try:
            rendered = self.path.format(**fields)
        except (KeyError, IndexError, ValueError) as error:
            raise ProfileError(f"condition: cannot render path ({error})") from error
        return _validate_condition_path("condition", rendered, rendered=True)


@dataclass(frozen=True)
class UnitSpec:
    """One systemd unit and what the lifecycle is expected to do with it."""

    name: str
    template: str
    enable: bool = True
    restart: bool = True
    verify: bool = True
    condition: ActivationCondition | None = None

    def render(self, fields: dict[str, str]) -> str:
        try:
            return self.template.format(**fields)
        except KeyError as error:  # pragma: no cover - guarded by _check_template
            raise ProfileError(f"{self.name}: unknown template field {error}") from error


@dataclass(frozen=True)
class ReadinessCheck:
    """One thing that has to be true before a release counts as working.

    ``systemctl is-active`` answers "the process did not exit". It does not
    answer "the process can find its identity", "the site ever set a cloud URL",
    or "the release actually contains the module the unit executes" -- and a
    service that starts, fails to load its configuration, and sits there is
    active by every measure the lifecycle had before this existed.

    ``provisioning`` is the distinction that makes the check usable at install
    time. A device that has been installed but not yet paired is *supposed* to
    be missing its identity and its credential; that is the normal first state
    of every machine and must not roll a good release back. Anything else
    failing is the release not working on this device.
    """

    id: str
    kind: str
    target: str
    description: str = ""
    #: True when failing this check means "not paired yet", not "broken".
    provisioning: bool = False


@dataclass(frozen=True)
class Profile:
    """A named unit set. ``units`` is ordered; activation follows that order."""

    name: str
    description: str
    units: tuple[UnitSpec, ...]
    readiness: tuple[ReadinessCheck, ...] = ()

    def unit_names(self) -> tuple[str, ...]:
        return tuple(unit.name for unit in self.units)

    def render(self, fields: dict[str, str]) -> dict[str, str]:
        return {unit.name: unit.render(fields) for unit in self.units}


def default_profiles_path() -> Path:
    return Path(__file__).resolve().parent / "data" / "lifecycle-profiles.json"


def _check_template(name: str, lines: object) -> str:
    if not isinstance(lines, list) or not all(isinstance(line, str) for line in lines):
        raise ProfileError(f"{name}: template must be a list of strings")
    text = "\n".join(lines) + "\n"
    # Reject unknown/typo'd placeholders up front. A template that renders with a
    # stray `{foo}` would either raise at install time on a device or, worse,
    # install a unit with a literal brace in an ExecStart= path.
    probe = dict.fromkeys(PROFILE_FIELDS, "x")
    try:
        text.format(**probe)
    except (KeyError, IndexError, ValueError) as error:
        raise ProfileError(f"{name}: bad template placeholder ({error})") from error
    return text


_UNIT_KEYS = frozenset(
    {"name", "template", "enable", "restart", "verify", "condition", "note"}
)
_PROFILE_KEYS = frozenset({"extends", "description", "units", "readiness", "note"})
_READINESS_KEYS = frozenset({"id", "kind", "target", "description", "provisioning", "note"})
_CONDITION_KEYS = frozenset({"kind", "path"})


def _validate_condition_path(name: str, value: object, *, rendered: bool) -> Path:
    label = "rendered condition path" if rendered else "condition path template"
    if not isinstance(value, str) or not value:
        raise ProfileError(f"{name}: {label} must be a non-empty string")
    if len(value) > MAX_ACTIVATION_CONDITION_PATH:
        raise ProfileError(f"{name}: {label} exceeds {MAX_ACTIVATION_CONDITION_PATH} characters")
    if any(ord(character) < 0x20 or ord(character) > 0x7E for character in value):
        raise ProfileError(f"{name}: {label} must contain printable ASCII only")
    path = Path(value)
    if rendered and not path.is_absolute():
        raise ProfileError(f"{name}: rendered condition path must be absolute")
    if any(part in {".", ".."} for part in value.split("/")):
        raise ProfileError(f"{name}: condition path may not contain dot traversal")
    return path


def _load_activation_condition(name: str, raw: object) -> ActivationCondition | None:
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ProfileError(f"{name}: condition must be an object")
    unknown = sorted(set(raw) - _CONDITION_KEYS)
    if unknown:
        raise ProfileError(f"{name}: unknown condition key(s) {unknown}")
    kind = raw.get("kind")
    if kind not in ACTIVATION_CONDITION_KINDS:
        raise ProfileError(f"{name}: unknown condition kind {kind!r}")
    target = raw.get("path")
    _validate_condition_path(name, target, rendered=False)
    assert isinstance(target, str)  # noqa: S101 - validated immediately above
    try:
        parsed = list(string.Formatter().parse(target))
    except ValueError as error:
        raise ProfileError(f"{name}: bad condition path template ({error})") from error
    for _, field, format_spec, conversion in parsed:
        if field is not None and (
            field not in PROFILE_FIELDS or bool(format_spec) or conversion is not None
        ):
            raise ProfileError(f"{name}: unsupported condition path field {field!r}")
    probe = {field: f"/{field}" for field in PROFILE_FIELDS}
    try:
        rendered = target.format(**probe)
    except (KeyError, IndexError, ValueError) as error:
        raise ProfileError(f"{name}: bad condition path template ({error})") from error
    _validate_condition_path(name, rendered, rendered=True)
    return ActivationCondition(kind=kind, path=target)


def _load_readiness(profile_name: str, raw: object) -> tuple[ReadinessCheck, ...]:
    if not isinstance(raw, list):
        raise ProfileError(f"{profile_name}: readiness must be a list")
    checks: list[ReadinessCheck] = []
    seen: set[str] = set()
    for entry in raw:
        if not isinstance(entry, dict):
            raise ProfileError(f"{profile_name}: each readiness check must be an object")
        unknown = sorted(set(entry) - _READINESS_KEYS)
        if unknown:
            raise ProfileError(f"{profile_name}: unknown readiness key(s) {unknown}")
        check_id = entry.get("id")
        if not isinstance(check_id, str) or not check_id:
            raise ProfileError(f"{profile_name}: readiness check has no id")
        if check_id in seen:
            raise ProfileError(f"{profile_name}: duplicate readiness check {check_id}")
        seen.add(check_id)
        kind = entry.get("kind")
        if kind not in READINESS_KINDS:
            raise ProfileError(f"{profile_name}/{check_id}: unknown readiness kind {kind!r}")
        target = entry.get("target")
        if not isinstance(target, str) or not target:
            raise ProfileError(f"{profile_name}/{check_id}: readiness target must be a string")
        if kind == "path_exists":
            # Same placeholder discipline as a unit template: a typo'd field in
            # a readiness path would silently check a literal `{foo}` that never
            # exists and fail every install.
            _check_template(f"{profile_name}/{check_id}", [target])
        provisioning = entry.get("provisioning", False)
        if not isinstance(provisioning, bool):
            raise ProfileError(
                f"{profile_name}/{check_id}: provisioning must be true or false, "
                f"not {provisioning!r}"
            )
        checks.append(
            ReadinessCheck(
                id=check_id,
                kind=kind,
                target=target,
                description=str(entry.get("description", "")),
                provisioning=provisioning,
            )
        )
    return tuple(checks)


def _flag(profile_name: str, unit_name: str, entry: dict, key: str) -> bool:
    """A policy flag must be a real JSON boolean.

    ``bool("false")`` is ``True``. This registry decides whether a unit is
    enabled at boot and whether its liveness is verified before an update is
    allowed to stand, so a quoted ``"false"`` silently becoming ``true`` is a
    policy inversion, not a formatting nit. Malformed policy fails closed.
    """

    value = entry.get(key, True)
    if not isinstance(value, bool):
        raise ProfileError(
            f"{profile_name}/{unit_name}: {key} must be true or false, not {value!r}"
        )
    return value


def _load_units(profile_name: str, raw: object) -> tuple[UnitSpec, ...]:
    if not isinstance(raw, list):
        raise ProfileError(f"{profile_name}: units must be a list")
    units: list[UnitSpec] = []
    seen: set[str] = set()
    for entry in raw:
        if not isinstance(entry, dict):
            raise ProfileError(f"{profile_name}: each unit must be an object")
        unknown = sorted(set(entry) - _UNIT_KEYS)
        if unknown:
            # An unrecognised key is almost always a misspelled one, and a
            # misspelled `verify` reads as "verification requested" while
            # actually disabling it. Refuse rather than guess.
            raise ProfileError(f"{profile_name}: unknown unit key(s) {unknown}")
        name = entry.get("name")
        if not isinstance(name, str) or not name or "/" in name or name.startswith("."):
            raise ProfileError(f"{profile_name}: unsafe unit name {name!r}")
        if not name.endswith((".service", ".timer", ".socket", ".path", ".target")):
            raise ProfileError(f"{profile_name}: {name} has no systemd unit suffix")
        if name in seen:
            raise ProfileError(f"{profile_name}: duplicate unit {name}")
        seen.add(name)
        if "condition" in entry and entry["condition"] is None:
            raise ProfileError(f"{profile_name}/{name}: condition must be an object")
        units.append(
            UnitSpec(
                name=name,
                template=_check_template(f"{profile_name}/{name}", entry.get("template")),
                enable=_flag(profile_name, name, entry, "enable"),
                restart=_flag(profile_name, name, entry, "restart"),
                verify=_flag(profile_name, name, entry, "verify"),
                condition=_load_activation_condition(
                    f"{profile_name}/{name}", entry.get("condition")
                ),
            )
        )
    if not units:
        raise ProfileError(f"{profile_name}: declares no units")
    return tuple(units)


def _resolve(name: str, raw_profiles: dict, seen: tuple[str, ...] = ()) -> Profile:
    if name in seen:
        raise ProfileError(f"profile inheritance cycle: {' -> '.join((*seen, name))}")
    raw = raw_profiles.get(name)
    if not isinstance(raw, dict):
        raise ProfileError(f"unknown profile {name!r}")
    unknown = sorted(set(raw) - _PROFILE_KEYS)
    if unknown:
        raise ProfileError(f"{name}: unknown profile key(s) {unknown}")

    units: tuple[UnitSpec, ...] = ()
    readiness: tuple[ReadinessCheck, ...] = ()
    parent = raw.get("extends")
    if parent is not None:
        if not isinstance(parent, str):
            raise ProfileError(f"{name}: extends must be a profile name")
        inherited_profile = _resolve(parent, raw_profiles, (*seen, name))
        units = inherited_profile.units
        readiness = inherited_profile.readiness

    own_readiness = _load_readiness(name, raw.get("readiness", []))
    inherited_ids = {check.id for check in readiness}
    for check in own_readiness:
        if check.id in inherited_ids:
            # Additive here too: an adapter profile that could redefine an
            # inherited readiness check could quietly turn a hard failure into
            # "not provisioned yet" for every site that adopts it.
            raise ProfileError(
                f"{name}: readiness check {check.id} would override an inherited one"
            )
    readiness = (*readiness, *own_readiness)

    own = _load_units(name, raw.get("units", []))
    inherited = {unit.name for unit in units}
    for unit in own:
        if unit.name in inherited:
            # Additive means additive. Redefining an inherited unit would make
            # "adopt the ROS 2 profile" silently rewrite the base units a site
            # already runs -- the exact surprise this boundary exists to prevent.
            raise ProfileError(f"{name}: {unit.name} would override an inherited unit")
    return Profile(
        name=name,
        description=str(raw.get("description", "")),
        units=(*units, *own),
        readiness=readiness,
    )


def _load(path: Path) -> tuple[dict[str, Profile], str]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except OSError as error:
        raise ProfileError(f"cannot read {path}: {error}") from error
    except ValueError as error:
        raise ProfileError(f"{path} is not valid JSON: {error}") from error

    if not isinstance(document, dict):
        raise ProfileError(f"{path}: top level must be an object")
    if document.get("schema") != LIFECYCLE_PROFILES_VERSION:
        raise ProfileError(f"{path}: schema must be {LIFECYCLE_PROFILES_VERSION}")
    raw_profiles = document.get("profiles")
    if not isinstance(raw_profiles, dict) or not raw_profiles:
        raise ProfileError(f"{path}: no profiles declared")

    profiles = {name: _resolve(name, raw_profiles) for name in sorted(raw_profiles)}
    runbook_lines = document.get("runbook", [])
    if not isinstance(runbook_lines, list):
        raise ProfileError(f"{path}: runbook must be a list of strings")
    runbook = "\n".join(str(line) for line in runbook_lines) + "\n"
    return profiles, runbook


@lru_cache(maxsize=8)
def _load_cached(resolved: str) -> tuple[dict[str, Profile], str]:
    return _load(Path(resolved))


def load_profiles(path: Path | str | None = None) -> dict[str, Profile]:
    """Return every declared profile, resolved and validated.

    ``path`` defaults to the registry shipped inside the installed package, so a
    device needs no source checkout and no environment variable to know what a
    ``generic`` install is.
    """

    return dict(_load_cached(str(Path(path or default_profiles_path()).resolve()))[0])


def runbook_text(path: Path | str | None = None) -> str:
    """The operator runbook the installer drops beside the configuration."""

    return _load_cached(str(Path(path or default_profiles_path()).resolve()))[1]
