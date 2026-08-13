"""What a release was actually activated *as*, captured at the moment it was.

Recording a version, a digest and a set of hashes is enough to notice that a
rollback would no longer reproduce the original activation. It is not enough to
*perform* one. A site edits a template under the same profile name, or points
``--profiles`` somewhere new, or simply deletes the registry that declared a
profile the fleet no longer ships -- and a device that was running fine six
months ago can now only be told "that no longer renders the same way". Detecting
the problem and then refusing is a better failure than silently activating the
wrong units, but it is still a device an operator cannot roll back.

So the activation is snapshotted in full. The snapshot carries the rendered unit
text verbatim, the per-unit enable/restart/verify policy, the readiness contract
that was in force, the interpreter the units were rendered with, and the release
digest. Rollback replays it and needs no registry at all: not the shipped one,
not the site's, not the one that has since been deleted.

Three properties make that safe to trust:

* **Immutable.** A snapshot is written once. A second write with different
  content is refused, exactly as release provenance is, because a snapshot that
  can be rewritten proves nothing about what ran.
* **Tamper-evident.** The document carries a digest over its own canonical body.
  A snapshot that does not hash to its own digest is refused rather than
  replayed -- it is the one input to a rollback that nobody re-derives.
* **Identified by what it is, not by what it is called.** That same digest *is*
  the activation id. A version is a name a release was published under; an
  activation is one particular thing that name was made to mean on one device --
  a profile, an interpreter, a rendered unit set. Installing ``1.0.0`` under
  ``generic`` and then under ``ros2`` is two activations of one version, and a
  device that cannot tell them apart cannot roll back the second without
  silently undoing the first as well. Deriving the id from the covered body
  makes "same activation" and "same bytes" the same question, so identity
  cannot be forged independently of the content it names.
* **Bounded.** Unit text is capped, and old snapshots are pruned alongside the
  history they belong to, so a device that has updated for years does not
  accumulate an unbounded pile of them in ``/var``.

Nothing secret is stored. Unit text is rendered from templates and product
paths; ``EnvironmentFile=`` names the configuration file rather than quoting it,
and credentials are never read here at all.
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

from .lifecycle_profiles import (
    Profile,
    ProfileError,
    ReadinessCheck,
    UnitSpec,
    _load_activation_condition,
)

__all__ = [
    "ACTIVATION_SNAPSHOT_VERSION",
    "MAX_SNAPSHOT_BYTES",
    "Snapshot",
    "SnapshotError",
    "body_digest",
    "build",
    "load_document",
]

ACTIVATION_SNAPSHOT_VERSION = "flyto.activation-snapshot.v1"

#: A unit set is a handful of small text files. Anything approaching this is a
#: registry doing something a unit template should not do, and the bound is what
#: keeps a state directory from being filled by one pathological profile.
MAX_SNAPSHOT_BYTES = 256 * 1024

_DIGEST_HEX = 64
_POLICY_KEYS = ("enable", "restart", "verify")


class SnapshotError(ValueError):
    """A snapshot is missing, malformed, or does not match its own digest."""

    reason = "activation_snapshot_invalid"


@dataclass(frozen=True)
class Snapshot:
    """One activation, reconstructable without any registry."""

    version: str
    profile: str
    python: str
    release_digest: str
    units: dict[str, str]
    policy: dict[str, dict[str, object]]
    readiness: tuple[dict, ...]

    def body(self) -> dict:
        """The canonical, digest-covered content. Sorted, so it is stable."""

        return {
            "schema": ACTIVATION_SNAPSHOT_VERSION,
            "version": self.version,
            "profile": self.profile,
            "python": self.python,
            "release_digest": self.release_digest,
            "units": dict(sorted(self.units.items())),
            "policy": {name: dict(flags) for name, flags in sorted(self.policy.items())},
            "readiness": [dict(sorted(check.items())) for check in self.readiness],
        }

    def document(self) -> dict:
        body = self.body()
        return {**body, "digest": body_digest(body)}

    @property
    def activation_id(self) -> str:
        """This activation's identity: the digest over its own covered body.

        Deliberately the same value as ``digest`` rather than a second field.
        An id stored *inside* the body could not cover itself, so it would be
        the one part of the document a tamperer could edit freely -- and an
        activation whose identity can be changed without changing its digest is
        an identity that names nothing. Two activations are the same activation
        exactly when they would replay the same way, which is what a rollback
        target has to mean.
        """

        return body_digest(self.body())

    def spec(self) -> Profile:
        """Rebuild the unit *policy* this activation ran under.

        Templates are deliberately empty: the rendered text is already in
        ``units`` and must be replayed byte for byte rather than re-rendered.
        Re-rendering is the thing this whole module exists to stop.
        """

        return Profile(
            name=self.profile,
            description=f"replayed activation of {self.version}",
            units=tuple(
                UnitSpec(
                    name=name,
                    template="",
                    enable=bool(self.policy[name]["enable"]),
                    restart=bool(self.policy[name]["restart"]),
                    verify=bool(self.policy[name]["verify"]),
                    condition=(
                        _snapshot_condition(
                            self.policy[name]["condition"], f"{self.profile}/{name}"
                        )
                        if "condition" in self.policy[name]
                        else None
                    ),
                )
                for name in sorted(self.units)
            ),
            readiness=tuple(
                ReadinessCheck(
                    id=str(check["id"]),
                    kind=str(check["kind"]),
                    target=str(check["target"]),
                    description=str(check.get("description", "")),
                    provisioning=bool(check.get("provisioning", False)),
                )
                for check in self.readiness
            ),
        )


def body_digest(body: dict) -> str:
    """sha256 over the canonical JSON encoding of a snapshot body."""

    encoded = json.dumps(body, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _snapshot_condition(raw: object, name: str):
    if raw is None:
        raise SnapshotError(f"{name}: condition must be an object")
    try:
        return _load_activation_condition(name, raw)
    except ProfileError as error:
        raise SnapshotError(str(error)) from error


def build(
    *,
    version: str,
    profile: Profile,
    python: str,
    release_digest: str,
    units: dict[str, str],
) -> Snapshot:
    """Capture ``units`` as they were rendered, with the policy that governed them."""

    total = sum(len(text.encode("utf-8")) for text in units.values())
    if total > MAX_SNAPSHOT_BYTES:
        raise SnapshotError(
            f"rendered unit set is {total} bytes; the activation snapshot bound is "
            f"{MAX_SNAPSHOT_BYTES}"
        )
    declared = {unit.name: unit for unit in profile.units}
    policy: dict[str, dict[str, object]] = {}
    for name in units:
        unit = declared.get(name)
        if unit is None:  # pragma: no cover - render() only emits declared units
            raise SnapshotError(f"{name} was rendered but is not declared by {profile.name}")
        policy[name] = {
            "enable": bool(unit.enable),
            "restart": bool(unit.restart),
            "verify": bool(unit.verify),
        }
        if unit.condition is not None:
            policy[name]["condition"] = {
                "kind": unit.condition.kind,
                "path": unit.condition.path,
            }
    readiness = tuple(
        {
            "id": check.id,
            "kind": check.kind,
            "target": check.target,
            "description": check.description,
            "provisioning": check.provisioning,
        }
        for check in profile.readiness
    )
    return Snapshot(
        version=version,
        profile=profile.name,
        python=python,
        release_digest=release_digest,
        units=dict(units),
        policy=policy,
        readiness=readiness,
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise SnapshotError(message)


def load_document(
    document: object,
    *,
    path: Path,
    version: str | None = None,
    activation_id: str | None = None,
) -> Snapshot:
    """Validate a snapshot document strictly, then return it.

    Fails closed on every axis a rollback would otherwise trust blindly: schema,
    the version it claims to describe, the shape of the unit and policy tables,
    and the digest over its own body. A snapshot is the single input to a
    rollback that nothing else re-derives, so "looks about right" is not a bar
    it is allowed to clear.

    ``version`` and ``activation_id`` are the caller's *expectations*, and each
    is checked when supplied. A caller that looked a snapshot up by activation
    id has to be told when the file it found describes a different activation,
    or a record swapped for another valid record would replay unnoticed: both
    documents hash correctly, and only the expectation distinguishes them.
    """

    _require(isinstance(document, dict), f"{path}: snapshot is not an object")
    assert isinstance(document, dict)  # noqa: S101 - narrows for the type checker
    _require("digest" in document, f"{path}: snapshot carries no digest")
    claimed = document["digest"]
    body = {key: value for key, value in document.items() if key != "digest"}
    allowed_document = {
        "schema", "version", "profile", "python", "release_digest",
        "units", "policy", "readiness", "digest",
    }
    required_document = allowed_document - {"readiness"}
    _require(
        required_document <= set(document) <= allowed_document,
        f"{path}: snapshot has unknown or missing top-level fields",
    )

    _require(
        body.get("schema") == ACTIVATION_SNAPSHOT_VERSION,
        f"{path}: unknown snapshot schema {body.get('schema')!r}",
    )
    recorded_version = body.get("version")
    _require(
        isinstance(recorded_version, str) and bool(recorded_version),
        f"{path}: snapshot names no version",
    )
    if version is not None:
        _require(recorded_version == version, f"{path}: snapshot describes {recorded_version!r}")
    _require(
        isinstance(claimed, str) and len(claimed) == _DIGEST_HEX and claimed == body_digest(body),
        f"{path}: snapshot does not match its own digest",
    )
    if activation_id is not None:
        # The digest *is* the activation id, so this compares identity against
        # the name the caller resolved it by rather than re-deriving it.
        _require(claimed == activation_id, f"{path}: snapshot is not activation {activation_id}")

    profile = body.get("profile")
    python = body.get("python")
    release = body.get("release_digest")
    units = body.get("units")
    policy = body.get("policy")
    readiness = body.get("readiness", [])
    _require(isinstance(profile, str) and bool(profile), f"{path}: snapshot has no profile")
    _require(isinstance(python, str) and bool(python), f"{path}: snapshot has no interpreter")
    _require(isinstance(release, str) and len(release) == _DIGEST_HEX,
             f"{path}: snapshot has no release digest")
    _require(isinstance(units, dict) and bool(units), f"{path}: snapshot declares no units")
    _require(isinstance(policy, dict), f"{path}: snapshot declares no unit policy")
    _require(isinstance(readiness, list), f"{path}: snapshot readiness is not a list")
    assert isinstance(units, dict) and isinstance(policy, dict)  # noqa: S101

    total = 0
    for name, text in units.items():
        _require(isinstance(name, str) and bool(name), f"{path}: snapshot has an unnamed unit")
        _require(isinstance(text, str), f"{path}: {name} is not unit text")
        total += len(text.encode("utf-8"))
        flags = policy.get(name)
        _require(isinstance(flags, dict), f"{path}: {name} has no policy")
        assert isinstance(flags, dict)  # noqa: S101
        for key in _POLICY_KEYS:
            _require(isinstance(flags.get(key), bool), f"{path}: {name}.{key} is not a boolean")
        allowed = set(_POLICY_KEYS)
        if "condition" in flags:
            allowed.add("condition")
            _snapshot_condition(flags["condition"], f"{path}: {name}")
        _require(set(flags) == allowed, f"{path}: {name} has unknown or missing policy fields")
    _require(set(policy) == set(units), f"{path}: snapshot policy does not match its units")
    _require(total <= MAX_SNAPSHOT_BYTES, f"{path}: snapshot exceeds {MAX_SNAPSHOT_BYTES} bytes")

    for check in readiness:
        _require(isinstance(check, dict), f"{path}: readiness entry is not an object")
        assert isinstance(check, dict)  # noqa: S101
        for key in ("id", "kind", "target"):
            _require(isinstance(check.get(key), str) and bool(check[key]),
                     f"{path}: readiness entry has no {key}")

    return Snapshot(
        version=str(recorded_version),
        profile=str(profile),
        python=str(python),
        release_digest=str(release),
        units={str(name): str(text) for name, text in units.items()},
        policy={
            str(name): {
                **{key: bool(flags[key]) for key in _POLICY_KEYS},
                **({"condition": dict(flags["condition"])} if "condition" in flags else {}),
            }
            for name, flags in policy.items()
        },
        readiness=tuple(dict(check) for check in readiness),
    )
