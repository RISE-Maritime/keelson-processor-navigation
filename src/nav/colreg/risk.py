"""Collision risk grade from CPA and TCPA, with thresholds per operational context.

Rule 7 asks whether risk of collision exists; this answers in four grades so an advisor can watch
before it advises and advise before it is urgent. Entering a grade takes the threshold, leaving it
takes the threshold times ``clear_margin``: without that, a target sitting on a boundary raises and
clears on every sample (Crowsnest measured 9 re-raises in 15 minutes on live traffic without it).
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum

NM = 1852.0


class Risk(IntEnum):
    NONE = 0
    WATCH = 1
    RISK = 2
    DANGER = 3

    @property
    def proto_name(self) -> str:
        return f"RISK_{self.name}"


@dataclass(frozen=True)
class Thresholds:
    cpa_m: float
    tcpa_s: float
    danger_cpa_m: float
    danger_tcpa_s: float
    watch_factor: float = 2.0


# RISK at cpa_m / tcpa_s; DANGER inside the tighter pair; WATCH inside watch_factor times RISK.
# Open sea matches Crowsnest's default alert (1 NM / 12 min); the rest step down to harbour scale.
DEFAULT_THRESHOLDS: dict[str, Thresholds] = {
    "OPERATIONAL_CONTEXT_OPEN_SEA": Thresholds(1.0 * NM, 12 * 60, 0.5 * NM, 6 * 60),
    "OPERATIONAL_CONTEXT_COASTAL": Thresholds(0.5 * NM, 8 * 60, 0.25 * NM, 4 * 60),
    "OPERATIONAL_CONTEXT_ARCHIPELAGO": Thresholds(0.25 * NM, 6 * 60, 0.12 * NM, 3 * 60),
    "OPERATIONAL_CONTEXT_PORT_APPROACH": Thresholds(0.15 * NM, 5 * 60, 0.08 * NM, 150),
    "OPERATIONAL_CONTEXT_PORT_AREA": Thresholds(0.1 * NM, 4 * 60, 0.05 * NM, 120),
    "OPERATIONAL_CONTEXT_RIVER_WATERS": Thresholds(0.1 * NM, 4 * 60, 0.05 * NM, 120),
    "OPERATIONAL_CONTEXT_LOCK": Thresholds(0.05 * NM, 3 * 60, 0.03 * NM, 90),
}
DEFAULT_CONTEXT = "OPERATIONAL_CONTEXT_COASTAL"


def thresholds_for(context: str | None, table: dict[str, Thresholds] | None = None) -> Thresholds:
    table = table or DEFAULT_THRESHOLDS
    return table.get(context or "", table[DEFAULT_CONTEXT])


def _inside(cpa_m: float, tcpa_s: float, cpa_limit: float, tcpa_limit: float) -> bool:
    return cpa_m < cpa_limit and 0.0 < tcpa_s < tcpa_limit


def grade(
    cpa_m: float | None,
    tcpa_s: float | None,
    th: Thresholds,
    previous: Risk = Risk.NONE,
    clear_margin: float = 1.2,
) -> Risk:
    """The risk grade now, given the grade held before.

    No CPA or TCPA (velocity unknown, no relative motion) grades NONE: this module does not invent
    risk, and a target it cannot assess is surfaced elsewhere as degraded, not as safe.
    A closest approach already past (TCPA <= 0) is never a risk: the range is opening.
    """
    if cpa_m is None or tcpa_s is None:
        return Risk.NONE
    levels = (
        (Risk.DANGER, th.danger_cpa_m, th.danger_tcpa_s),
        (Risk.RISK, th.cpa_m, th.tcpa_s),
        (Risk.WATCH, th.cpa_m * th.watch_factor, th.tcpa_s * th.watch_factor),
    )
    for level, cpa_limit, tcpa_limit in levels:
        margin = clear_margin if previous >= level else 1.0
        if _inside(cpa_m, tcpa_s, cpa_limit * margin, tcpa_limit * margin):
            return level
    return Risk.NONE
