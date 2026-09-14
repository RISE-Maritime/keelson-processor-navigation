"""Operational authority: the level the vessel may act at, and what each level lets conduct do.

Acts on ``OperationalAuthority.level`` only, never ``composite_score``. UNKNOWN is non-authorising,
and so is a level nobody has restated recently: a ceiling is a claim about now.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..world.latch import Sample, fresh

UNKNOWN = "AUTHORITY_LEVEL_UNKNOWN"
LEVELS = (
    UNKNOWN,
    "AUTHORITY_LEVEL_MINIMAL_SAFE_MODE",
    "AUTHORITY_LEVEL_SUPERVISED_REMOTE",
    "AUTHORITY_LEVEL_REMOTE_CONTROLLED",
    "AUTHORITY_LEVEL_ASSISTED_AUTONOMOUS",
    "AUTHORITY_LEVEL_FULL_AUTONOMOUS",
)
REMOTE_CONTROLLED = LEVELS[3]
ASSISTED_AUTONOMOUS = LEVELS[4]
FULL_AUTONOMOUS = LEVELS[5]
DANGER = "RISK_DANGER"


def rank(level: str) -> int:
    """0 for UNKNOWN and for any name keelson does not define."""
    try:
        return LEVELS.index(level)
    except ValueError:
        return 0


def may_drive(level: str) -> bool:
    """Guidance may command the vessel only at REMOTE_CONTROLLED or above."""
    return rank(level) >= rank(REMOTE_CONTROLLED)


def may_auto_accept(level: str, worst_risk: str | None) -> bool:
    """FULL_AUTONOMOUS accepts any advice; ASSISTED_AUTONOMOUS only below DANGER; lower never.

    docs/PLAN.md §7 default: assisted auto-accepts below DANGER, otherwise an operator decides."""
    if rank(level) >= rank(FULL_AUTONOMOUS):
        return True
    if rank(level) >= rank(ASSISTED_AUTONOMOUS):
        return worst_risk != DANGER
    return False


@dataclass(frozen=True)
class InForce:
    level: str
    note: str


def in_force(sample: Sample | None, now_ns: int, max_age_s: float, fixed: str | None) -> InForce:
    """The level to act under now, and why. ``fixed`` comes from configuration and is meant for
    software-in-the-loop runs with no authority aggregator; it is stated in the note every time."""
    if fixed is not None:
        return InForce(fixed, f"authority level fixed by configuration at {fixed}")
    if sample is None:
        return InForce(UNKNOWN, "no operational_authority heard")
    if fresh(sample, now_ns, max_age_s) is None:
        return InForce(UNKNOWN, "operational_authority is stale")
    level = sample.value if sample.value in LEVELS else UNKNOWN
    return InForce(level, f"operational_authority says {level}")
