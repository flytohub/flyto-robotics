"""Deterministic natural-language goal screening and destination resolution."""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass, field
from typing import Any

from .capabilities import SAFE_TEXT
from .semantic_map import SemanticLocation, SemanticLocationMap, SemanticLocationStore

GOAL_MATCHER_ID = "flyto.robotics.goal-matcher.deterministic.v1"
MAX_GOAL_LENGTH = 2000
MAX_CANDIDATES = 8
MAX_RESOLVED_LOCATIONS = 512
AMBIGUITY_RATIO = 0.6

LABEL_PHRASE_SCORE = 60.0
LOCATION_ID_SCORE = 80.0
CJK_FRAGMENT_SCORE = 30.0
WORD_TOKEN_SCORE = 20.0

# Phrases that ask the robot to bypass its own safety envelope. Screened before
# any destination lookup so a dangerous request is refused even when the place
# name is perfectly valid.
SAFETY_OVERRIDE_PATTERNS: tuple[str, ...] = (
    "忽略障礙",
    "忽視障礙",
    "無視障礙",
    "不要停",
    "不用停",
    "別停",
    "不要煞車",
    "全速",
    "最快速度",
    "衝過去",
    "衝去",
    "直接撞",
    "撞開",
    "關閉安全",
    "停用安全",
    "略過確認",
    "跳過確認",
    "不用確認",
    "免確認",
    "ignore obstacle",
    "ignore obstacles",
    "ignore people",
    "ignore the human",
    "do not stop",
    "don't stop",
    "dont stop",
    "no braking",
    "disable safety",
    "bypass safety",
    "override safety",
    "full speed",
    "max speed",
    "maximum speed",
    "skip confirmation",
    "skip the confirmation",
    "no confirmation",
    "ram ",
    "crash through",
)

# Raw actuator vocabulary: a goal is a destination and an intent, never a
# velocity or a duty cycle.
ACTUATOR_PATTERNS: tuple[str, ...] = (
    "cmd_vel",
    "linear_x",
    "angular_z",
    "pwm",
    "duty cycle",
    "motor command",
    "raw motor",
    "馬達指令",
    "直接控制馬達",
    "轉速",
    "占空比",
)

# Deliveries are the only supported intent family in this phase. A goal that
# clearly asks for something else is refused instead of silently becoming a
# delivery.
DELIVERY_INTENT_PATTERNS: tuple[str, ...] = (
    "送",
    "配送",
    "運",
    "拿",
    "帶",
    "交付",
    "遞",
    "去",
    "前往",
    "到",
    "移動",
    "deliver",
    "delivery",
    "bring",
    "take",
    "carry",
    "transport",
    "drop off",
    "dropoff",
    "go to",
    "goto",
    "navigate",
    "move to",
    "head to",
    "fetch",
)

HAN_DIGITS = {
    "零": "0",
    "一": "1",
    "二": "2",
    "兩": "2",
    "三": "3",
    "四": "4",
    "五": "5",
    "六": "6",
    "七": "7",
    "八": "8",
    "九": "9",
    "十": "10",
}

WORD_RE = re.compile(r"[a-z0-9]+")
CJK_RE = re.compile(r"[㐀-䶿一-鿿豈-﫿]")
MIN_CJK_FRAGMENT = 2
MIN_WORD_TOKEN = 3


class GoalResolutionError(ValueError):
    """Raised when a goal cannot be screened or resolved."""


@dataclass(frozen=True)
class LocationCandidate:
    """One scored destination hypothesis with the rules that produced it."""

    location_id: str
    label: str
    score: float
    match_rules: tuple[str, ...]
    shared_fragments: tuple[str, ...] = ()
    selected: bool = False

    def to_dict(self) -> dict[str, Any]:
        return {
            "location_id": self.location_id,
            "label": self.label,
            "score": round(self.score, 3),
            "match_rules": list(self.match_rules),
            "shared_fragments": list(self.shared_fragments),
            "selected": self.selected,
        }


@dataclass(frozen=True)
class GoalResolution:
    """Screening plus destination outcome for one operator goal."""

    goal: str
    normalized_goal: str
    map_id: str
    map_revision: int
    destination: LocationCandidate | None = None
    candidates: tuple[LocationCandidate, ...] = ()
    shared_fragments: tuple[str, ...] = ()
    reason_code: str = ""
    stage: str = ""
    detail: str = ""
    operator_action: str = ""

    @property
    def resolved(self) -> bool:
        return self.destination is not None and not self.reason_code


def _fold_han_digits(text: str) -> str:
    return "".join(HAN_DIGITS.get(char, char) for char in text)


def normalize_goal_text(value: str) -> str:
    """Case-fold, NFKC-normalize, and fold Han digits for stable matching."""
    folded = unicodedata.normalize("NFKC", value).casefold()
    return _fold_han_digits(folded)


def _cjk_fragments(text: str, *, minimum: int = MIN_CJK_FRAGMENT) -> tuple[str, ...]:
    runs = re.findall(r"[^\W_]+", text, flags=re.UNICODE)
    fragments: list[str] = []
    for run in runs:
        if not CJK_RE.search(run):
            continue
        for size in range(minimum, min(len(run), 8) + 1):
            for start in range(0, len(run) - size + 1):
                fragments.append(run[start : start + size])
    return tuple(dict.fromkeys(fragments))


def _word_tokens(text: str, *, minimum: int = MIN_WORD_TOKEN) -> tuple[str, ...]:
    return tuple(
        dict.fromkeys(token for token in WORD_RE.findall(text) if len(token) >= minimum)
    )


def _screen_goal(normalized: str) -> tuple[str, str, str]:
    """Return (reason_code, detail, operator_action); empty code means clean."""
    for pattern in ACTUATOR_PATTERNS:
        if normalize_goal_text(pattern) in normalized:
            return (
                "safety_override_refused",
                "raw actuator control is not accepted from a goal",
                "restate_goal",
            )
    for pattern in SAFETY_OVERRIDE_PATTERNS:
        if normalize_goal_text(pattern) in normalized:
            return (
                "safety_override_refused",
                "goal asks the robot to bypass its safety envelope",
                "restate_goal",
            )
    if not any(
        normalize_goal_text(pattern) in normalized
        for pattern in DELIVERY_INTENT_PATTERNS
    ):
        return (
            "intent_unsupported",
            "goal does not request a delivery or navigation action",
            "restate_goal",
        )
    return ("", "", "")


def _location_labels(location: SemanticLocation) -> tuple[str, ...]:
    return tuple(location.labels)


def _score_location(
    location: SemanticLocation,
    *,
    normalized_goal: str,
    goal_fragments: frozenset[str],
    goal_tokens: frozenset[str],
) -> LocationCandidate:
    score = 0.0
    rules: list[str] = []
    matched_keys: list[str] = []

    if normalize_goal_text(location.location_id) in normalized_goal:
        score += LOCATION_ID_SCORE
        rules.append("location_id")
        matched_keys.append(location.location_id)

    for label in _location_labels(location):
        normalized_label = normalize_goal_text(label)
        if not normalized_label:
            continue
        if normalized_label in normalized_goal:
            score += LABEL_PHRASE_SCORE + len(normalized_label)
            rules.append("label_phrase")
            matched_keys.append(label)
            continue
        label_fragments = frozenset(_cjk_fragments(normalized_label))
        overlap = label_fragments & goal_fragments
        if overlap:
            longest = max(overlap, key=len)
            score += CJK_FRAGMENT_SCORE + len(longest)
            rules.append("cjk_fragment")
            matched_keys.append(longest)
            continue
        label_tokens = frozenset(_word_tokens(normalized_label))
        token_overlap = label_tokens & goal_tokens
        if token_overlap:
            score += WORD_TOKEN_SCORE * len(token_overlap)
            rules.append("word_token")
            matched_keys.extend(sorted(token_overlap))

    return LocationCandidate(
        location_id=location.location_id,
        label=_location_labels(location)[0],
        score=score,
        match_rules=tuple(dict.fromkeys(rules)),
        shared_fragments=tuple(dict.fromkeys(matched_keys))[:4],
    )


def _discriminative_keys(locations: tuple[SemanticLocation, ...]) -> frozenset[str]:
    """Keys owned by exactly one location; shared keys cannot select a target."""
    counts: dict[str, int] = {}
    for location in locations:
        keys: set[str] = {normalize_goal_text(location.location_id)}
        for label in _location_labels(location):
            keys.add(normalize_goal_text(label))
        for key in keys:
            counts[key] = counts.get(key, 0) + 1
    return frozenset(key for key, count in counts.items() if count == 1)


def resolve_delivery_goal(
    goal: str,
    *,
    semantic_map: SemanticLocationMap | SemanticLocationStore,
) -> GoalResolution:
    """Screen one goal and resolve exactly one destination, or explain why not."""
    if not isinstance(goal, str):
        raise GoalResolutionError("goal must be a string")
    trimmed = goal.strip()[:MAX_GOAL_LENGTH]
    normalized = normalize_goal_text(trimmed)

    if isinstance(semantic_map, SemanticLocationStore):
        location_map = semantic_map.load()
    else:
        location_map = semantic_map
    locations = tuple(location_map.locations)
    base = {
        "goal": trimmed,
        "normalized_goal": normalized,
        "map_id": location_map.map_id,
        "map_revision": location_map.revision,
    }

    if not trimmed:
        return GoalResolution(
            **base,
            reason_code="intent_unsupported",
            stage="goal_screening",
            detail="goal is empty",
            operator_action="restate_goal",
        )
    if not locations or len(locations) > MAX_RESOLVED_LOCATIONS:
        return GoalResolution(
            **base,
            reason_code="semantic_map_unavailable",
            stage="goal_resolution",
            detail=f"semantic map holds {len(locations)} usable locations",
            operator_action="contact_operator",
        )

    reason_code, detail, operator_action = _screen_goal(normalized)
    if reason_code:
        return GoalResolution(
            **base,
            reason_code=reason_code,
            stage="goal_screening",
            detail=detail,
            operator_action=operator_action,
        )

    goal_fragments = frozenset(_cjk_fragments(normalized))
    goal_tokens = frozenset(_word_tokens(normalized))
    discriminative = _discriminative_keys(locations)
    scored = sorted(
        (
            _score_location(
                location,
                normalized_goal=normalized,
                goal_fragments=goal_fragments,
                goal_tokens=goal_tokens,
            )
            for location in locations
        ),
        key=lambda candidate: (-candidate.score, candidate.location_id),
    )
    # A match only counts when at least one matched key belongs to a single
    # location; shared suffixes such as "號病房" must never pick a winner.
    contenders = [
        candidate
        for candidate in scored
        if candidate.score > 0.0
        and any(
            normalize_goal_text(key) in discriminative
            for key in candidate.shared_fragments
        )
    ]
    near_misses = tuple(
        dict.fromkeys(
            fragment
            for candidate in scored
            if candidate.score > 0.0
            for fragment in candidate.shared_fragments
            if normalize_goal_text(fragment) not in discriminative
        )
    )[:4]

    if not contenders:
        return GoalResolution(
            **base,
            candidates=tuple(
                candidate for candidate in scored if candidate.score > 0.0
            )[:MAX_CANDIDATES],
            shared_fragments=near_misses,
            reason_code="location_unresolved",
            stage="goal_resolution",
            detail=f"no location in map {location_map.map_id} matches the goal",
            operator_action=(
                "name_one_of_the_listed_locations"
                if near_misses
                else "teach_location_or_restate_goal"
            ),
        )

    best = contenders[0]
    rivals = [
        candidate
        for candidate in contenders[1:]
        if candidate.score >= best.score * AMBIGUITY_RATIO
    ]
    if rivals:
        return GoalResolution(
            **base,
            candidates=tuple([best, *rivals])[:MAX_CANDIDATES],
            reason_code="location_ambiguous",
            stage="goal_resolution",
            detail=f"{len(rivals) + 1} locations match the goal equally well",
            operator_action="restate_goal",
        )

    selected = LocationCandidate(
        location_id=best.location_id,
        label=best.label,
        score=best.score,
        match_rules=best.match_rules,
        shared_fragments=best.shared_fragments,
        selected=True,
    )
    return GoalResolution(
        **base,
        destination=selected,
        candidates=tuple([selected, *contenders[1:]])[:MAX_CANDIDATES],
    )


def rejection_payload(resolution: GoalResolution) -> dict[str, Any]:
    """Build the relay-safe structured rejection the UI renders."""
    assert resolution.reason_code, "rejection_payload requires a rejected resolution"
    assert SAFE_TEXT.fullmatch(resolution.reason_code)
    return {
        "contract_version": "flyto.robotics.delivery-rejection.v1",
        "reason_code": resolution.reason_code,
        "stage": resolution.stage,
        "message_key": f"robotics.delivery.rejected.{resolution.reason_code}",
        "detail": resolution.detail[:160],
        "goal_excerpt": resolution.goal[:120],
        "map_id": resolution.map_id,
        "map_revision": resolution.map_revision,
        "matcher_id": GOAL_MATCHER_ID,
        "shared_fragments": list(resolution.shared_fragments),
        "candidates": [candidate.to_dict() for candidate in resolution.candidates],
        "recoverable": True,
        "operator_action": resolution.operator_action or "restate_goal",
    }


@dataclass(frozen=True)
class DecisionTimeline:
    """Bounded, attributed decision trail rendered by the operator UI."""

    entries: list[dict[str, Any]] = field(default_factory=list)
    limit: int = 16

    def record(self, *, stage: str, actor: str, detail: str) -> None:
        if len(self.entries) >= self.limit:
            return
        self.entries.append(
            {
                "sequence": len(self.entries) + 1,
                "stage": stage,
                "actor": actor,
                "detail": detail[:160],
            }
        )

    def to_list(self) -> list[dict[str, Any]]:
        return list(self.entries)
