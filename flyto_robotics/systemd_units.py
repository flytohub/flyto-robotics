"""Semantic systemd unit parsing and validation.

This module exists because a substring check against unit *text* proves nothing.
systemd reads a directive only from the section it is defined for: a
``StartLimitBurst=`` placed in ``[Service]`` is parsed, accepted, and then
ignored, which silently restores the unbounded restart loop that
``handoffs/2026-08-07-bringup-boot-race-and-silent-hang.md`` describes. Every
assertion this project makes about a unit therefore goes through
:func:`parse_unit` and names a section, a key, and a value.

configparser is deliberately not used. systemd allows a key to repeat inside a
section with cumulative meaning (several ``ExecStartPre=``/``Environment=``
lines), and configparser collapses repeats to the last one.

Continuation semantics follow systemd's ``config_parse`` rather than "the line
ends with a backslash":

* a line is continued only when it ends with an **odd** number of backslashes;
  an even number is escaped backslashes and terminates the directive. Treating
  ``ExecStart=/bin/echo c:\\\\`` as a continuation swallows the next directive
  whole, and the swallowed directive then reads as absent to every test.
* a continuation dangling at end of file is not silently discarded. systemd
  warns and still applies the accumulated directive, so the parser applies it
  and reports it as a defect instead of dropping the directive on the floor.
* a comment line continues as a *comment*, so a trailing backslash in a comment
  cannot capture the directive underneath it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

__all__ = [
    "START_LIMIT_KEYS",
    "UnitDefect",
    "UnitFile",
    "parse_unit",
    "parse_unit_file",
    "validate_unit",
]

# systemd reads these from [Unit] only. In [Service] they are accepted and
# ignored -- the exact failure mode that let a unit reach NRestarts=193.
START_LIMIT_KEYS = ("StartLimitIntervalSec", "StartLimitInterval", "StartLimitBurst")

_COMMENT_PREFIXES = ("#", ";")


def _trailing_backslashes(line: str) -> int:
    count = 0
    for character in reversed(line):
        if character != "\\":
            break
        count += 1
    return count


def _is_continued(line: str) -> bool:
    return _trailing_backslashes(line) % 2 == 1


def _drop_continuation(line: str) -> str:
    """Remove the single continuation backslash, keeping escaped ones intact."""

    return line[:-1]


@dataclass(frozen=True)
class UnitDefect:
    """A machine-readable reason a unit would not behave as written."""

    code: str
    section: str
    key: str
    detail: str

    def as_dict(self) -> dict[str, str]:
        return {
            "code": self.code,
            "section": self.section,
            "key": self.key,
            "detail": self.detail,
        }


@dataclass(frozen=True)
class UnitFile:
    """A parsed unit: ``{section: [(key, value), ...]}`` in file order."""

    sections: dict[str, list[tuple[str, str]]] = field(default_factory=dict)
    defects: tuple[UnitDefect, ...] = ()

    def values(self, section: str, key: str) -> list[str]:
        return [value for name, value in self.sections.get(section, []) if name == key]

    def only(self, section: str, key: str) -> str:
        values = self.values(section, key)
        if len(values) != 1:
            raise KeyError(f"expected exactly one {section}/{key}, got {values!r}")
        return values[0]

    def keys(self, section: str) -> set[str]:
        return {key for key, _ in self.sections.get(section, [])}

    def has(self, section: str, key: str) -> bool:
        return bool(self.values(section, key))


def parse_unit(text: str) -> UnitFile:
    """Parse systemd unit text into sections, preserving repeated keys.

    Only the first ``=`` splits a directive, so ``Environment=KEY=VALUE`` keeps
    its own ``=``. Values are stripped of surrounding whitespace; the body of an
    ``ExecStart=`` shell line is otherwise left intact.
    """

    sections: dict[str, list[tuple[str, str]]] = {}
    defects: list[UnitDefect] = []
    current: str | None = None
    pending: str | None = None
    pending_is_comment = False

    for raw in text.splitlines():
        line = raw.strip()

        if pending is None:
            if not line:
                continue
            if line.startswith(_COMMENT_PREFIXES):
                # A comment may itself be continued. Consuming it as a comment
                # is what stops `# note \` from capturing the directive below.
                if _is_continued(line):
                    pending = ""
                    pending_is_comment = True
                continue
            if line.startswith("[") and line.endswith("]"):
                current = line[1:-1].strip()
                sections.setdefault(current, [])
                continue
            if _is_continued(line):
                pending = _drop_continuation(line)
                pending_is_comment = False
                continue
            joined = line
        else:
            if pending_is_comment:
                if not _is_continued(line):
                    pending = None
                    pending_is_comment = False
                continue
            joined = pending + line
            if _is_continued(joined):
                pending = _drop_continuation(joined)
                continue
            pending = None

        _record(sections, current, joined, defects)

    if pending is not None and not pending_is_comment:
        # systemd warns and still applies the accumulated directive. Dropping it
        # here would make a real directive read as absent to every caller.
        defects.append(
            UnitDefect(
                code="dangling_continuation",
                section=current or "",
                key=pending.partition("=")[0].strip(),
                detail="file ends inside a line continuation",
            )
        )
        _record(sections, current, pending, defects)

    return UnitFile(sections=sections, defects=tuple(defects))


def _record(
    sections: dict[str, list[tuple[str, str]]],
    current: str | None,
    line: str,
    defects: list[UnitDefect],
) -> None:
    if "=" not in line:
        return
    key, _, value = line.partition("=")
    key = key.strip()
    if current is None:
        defects.append(
            UnitDefect(
                code="directive_outside_section",
                section="",
                key=key,
                detail="directive appears before any [Section] header",
            )
        )
        return
    sections[current].append((key, value.strip()))


def parse_unit_file(path: Path) -> UnitFile:
    return parse_unit(Path(path).read_text(encoding="utf-8"))


def validate_unit(
    text: str,
    *,
    name: str = "",
    allow_home_paths: bool = False,
) -> tuple[UnitDefect, ...]:
    """Return every reason ``text`` would not behave the way it reads.

    An empty tuple means the unit is safe to install. This runs before a release
    is switched into ``current``, so a unit that would flap forever or silently
    ignore its own rate limit never becomes the running configuration.
    """

    unit = parse_unit(text)
    defects: list[UnitDefect] = list(unit.defects)

    if "Unit" not in unit.sections:
        defects.append(
            UnitDefect("missing_section", "Unit", "", f"{name or 'unit'} has no [Unit] section")
        )
    if not unit.values("Unit", "Description"):
        defects.append(UnitDefect("missing_description", "Unit", "Description", "no Description="))

    is_timer = name.endswith(".timer")

    for key in START_LIMIT_KEYS:
        for section in unit.sections:
            if section == "Unit":
                continue
            if key in unit.keys(section):
                defects.append(
                    UnitDefect(
                        "start_limit_wrong_section",
                        section,
                        key,
                        f"systemd reads {key}= from [Unit]; in [{section}] it is ignored",
                    )
                )

    if not is_timer and "Service" in unit.sections:
        exec_start = unit.values("Service", "ExecStart")
        service_type = (unit.values("Service", "Type") or ["simple"])[-1]
        if not exec_start and service_type != "oneshot":
            defects.append(
                UnitDefect("missing_exec_start", "Service", "ExecStart", "no ExecStart=")
            )
        if len(exec_start) > 1 and service_type != "oneshot":
            defects.append(
                UnitDefect(
                    "multiple_exec_start",
                    "Service",
                    "ExecStart",
                    "only Type=oneshot may repeat ExecStart=",
                )
            )
        restart = (unit.values("Service", "Restart") or ["no"])[-1]
        if restart in {"always", "on-failure", "on-abnormal"}:
            interval = unit.values("Unit", "StartLimitIntervalSec") or unit.values(
                "Unit", "StartLimitInterval"
            )
            burst = unit.values("Unit", "StartLimitBurst")
            if not interval or not burst:
                defects.append(
                    UnitDefect(
                        "unbounded_restart",
                        "Unit",
                        "StartLimitBurst",
                        f"Restart={restart} without a [Unit] start rate limit retries forever",
                    )
                )
            elif burst[-1].isdigit() and int(burst[-1]) > 5:
                # A burst that cannot be reached inside one interval is the same
                # as no limit at all: one failure cycle is far longer than
                # interval/burst, so the counter resets before it ever trips.
                defects.append(
                    UnitDefect(
                        "start_limit_unreachable",
                        "Unit",
                        "StartLimitBurst",
                        f"StartLimitBurst={burst[-1]} is too high to trip on a slow failure cycle",
                    )
                )

    if not allow_home_paths:
        for section, entries in unit.sections.items():
            for key, value in entries:
                if key not in {
                    "ExecStart",
                    "ExecStartPre",
                    "ExecStartPost",
                    "ExecStop",
                    "WorkingDirectory",
                    "EnvironmentFile",
                    "StateDirectory",
                }:
                    continue
                if "/home/" in value or "~/" in value:
                    defects.append(
                        UnitDefect(
                            "home_path_hardcoded",
                            section,
                            key,
                            "a product unit must not depend on a login user's home directory",
                        )
                    )

    return tuple(defects)
