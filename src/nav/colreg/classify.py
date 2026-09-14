"""Encounter classification under COLREG Part B, from own ship's point of view.

Pure geometry and rules; no zenoh, no clock.

Sectors (relative to a vessel's heading, degrees clockwise from the bow):

    ahead      within +-6 deg of the bow           head-on test (Rule 14)
    starboard  (6, 112.5)                           crossing from starboard (Rule 15)
    abaft      [112.5, 247.5]                       more than 22.5 deg abaft the beam (Rule 13)
    port       (247.5, 354)                         crossing from port (Rule 15)

Heading is used where it is known and course otherwise: Rules 13-15 turn on what a vessel
presents, and AIS heading is often unavailable (511).

What this does not model, stated so nobody reads it in: narrow channels and fairways (Rule 9),
traffic separation schemes (Rule 10), sound and light signals, vessel type beyond AIS navigational
status, and anything a chart would say. Rule 18 is applied from ``nav_status`` only.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum

HEAD_ON_SECTOR_DEG = 6.0
ABAFT_BEAM_DEG = 112.5
STOPPED_KNOTS = 0.5

# AIS navigational status that obliges a power-driven vessel to keep out of the way (Rule 18(a)),
# or that means the target is not under way at all.
_KEEP_CLEAR_OF = {
    "NOT_UNDER_COMMAND": "18(a)(i)",
    "RESTRICTED_MANEUVERABILITY": "18(a)(ii)",
    "CONSTRAINED_BY_DRAFT": "18(d)",
    "FISHING": "18(a)(iii)",
    "UNDER_WAY_SAILING": "18(a)(iv)",
}
_NOT_UNDER_WAY = {"AT_ANCHOR", "MOORING", "AGROUND"}


class Situation(StrEnum):
    """ColregSituation from keelson Route.proto, plus the two it lacks for a live encounter."""

    UNSPECIFIED = "COLREG_SITUATION_UNSPECIFIED"
    OVERTAKING = "COLREG_SITUATION_OVERTAKING"  # own ship is overtaking the target
    CROSSING_STBD = "COLREG_SITUATION_CROSSING_STBD"  # target crossing from own starboard
    CROSSING_PORT = "COLREG_SITUATION_CROSSING_PORT"  # target crossing from own port
    HEAD_ON = "COLREG_SITUATION_HEAD_ON"
    OVERTAKEN = "COLREG_SITUATION_OVERTAKEN"  # the target is overtaking own ship
    NONE = "COLREG_SITUATION_NONE"  # no vessel-to-vessel rule applies (stopped, restricted vis.)


class Role(StrEnum):
    UNSPECIFIED = "ROLE_UNSPECIFIED"
    GIVE_WAY = "ROLE_GIVE_WAY"
    STAND_ON = "ROLE_STAND_ON"
    NONE = "ROLE_NONE"  # Rule 19: neither stands on, both avoid


@dataclass(frozen=True)
class Vessel:
    """A vessel's presented state: position is not needed, only the relative bearing to it."""

    heading_deg: float | None
    cog_deg: float | None
    sog_knots: float | None
    nav_status: str | None = None

    @property
    def presented_deg(self) -> float | None:
        return self.heading_deg if self.heading_deg is not None else self.cog_deg

    @property
    def stopped(self) -> bool:
        return self.nav_status in _NOT_UNDER_WAY or (
            self.sog_knots is not None and self.sog_knots < STOPPED_KNOTS
        )


@dataclass(frozen=True)
class Classification:
    situation: Situation
    role: Role
    rules: tuple[str, ...]
    reason: str


def relative_deg(bearing_deg: float, heading_deg: float) -> float:
    return (bearing_deg - heading_deg) % 360.0


def _sector(rel: float) -> str:
    if rel <= HEAD_ON_SECTOR_DEG or rel >= 360.0 - HEAD_ON_SECTOR_DEG:
        return "ahead"
    if rel < ABAFT_BEAM_DEG:
        return "starboard"
    if rel <= 360.0 - ABAFT_BEAM_DEG:
        return "abaft"
    return "port"


def classify(
    own: Vessel,
    target: Vessel,
    bearing_to_target_deg: float,
    closing: bool,
    restricted_visibility: bool = False,
) -> Classification:
    """Classify one encounter.

    ``bearing_to_target_deg`` is true bearing from own ship to the target. ``closing`` is whether
    range is decreasing (TCPA > 0); overtaking only exists while closing.
    """
    own_dir = own.presented_deg
    if own_dir is None:
        return Classification(
            Situation.UNSPECIFIED, Role.UNSPECIFIED, (), "own heading and course unknown"
        )

    rel_target = relative_deg(bearing_to_target_deg, own_dir)
    target_sector = _sector(rel_target)
    target_side = (
        "port"
        if target_sector == "port" or (target_sector == "abaft" and rel_target > 180)
        else "starboard"
    )

    if restricted_visibility:
        forward = target_sector in ("ahead", "starboard", "port")
        return Classification(
            Situation.NONE,
            Role.NONE,
            ("19(d)",),
            "restricted visibility: no stand-on vessel; "
            + (
                f"target forward of the beam on the {target_side} side, avoid altering to port"
                if forward
                else f"target abeam or abaft on the {target_side} side, avoid altering towards it"
            ),
        )

    if target.stopped:
        return Classification(
            Situation.NONE,
            Role.GIVE_WAY,
            ("8",),
            "target is not under way: keep clear as of an obstruction",
        )

    tgt_dir = target.presented_deg
    if tgt_dir is None:
        return Classification(
            Situation.UNSPECIFIED,
            Role.GIVE_WAY,
            ("7(c)", "8"),
            "target heading and course unknown: assume the worst and keep clear",
        )

    # Bearing of own ship as seen from the target, relative to the target's heading.
    rel_own_from_target = relative_deg((bearing_to_target_deg + 180.0) % 360.0, tgt_dir)
    own_sector_from_target = _sector(rel_own_from_target)

    if own_sector_from_target == "abaft" and closing and target_sector != "abaft":
        return Classification(
            Situation.OVERTAKING,
            Role.GIVE_WAY,
            ("13",),
            "own ship is overtaking: keep out of the way",
        )
    if target_sector == "abaft":
        if closing:
            return Classification(
                Situation.OVERTAKEN,
                Role.STAND_ON,
                ("13", "17"),
                "being overtaken: keep course and speed",
            )
        return Classification(Situation.NONE, Role.NONE, (), "target abaft the beam and opening")

    status_rule = _KEEP_CLEAR_OF.get(target.nav_status or "")
    # Rule 14: the other vessel ahead or nearly ahead on a reciprocal or nearly reciprocal course.
    # Tested on the course difference, not on where own ship sits off the target's bow, which
    # would stack both vessels' offsets and call a 5-degree-off meeting a crossing.
    reciprocal_off = abs((tgt_dir - own_dir - 180.0 + 180.0) % 360.0 - 180.0)
    if target_sector == "ahead" and reciprocal_off <= HEAD_ON_SECTOR_DEG:
        situation, role, rules = Situation.HEAD_ON, Role.GIVE_WAY, ("14",)
        reason = "head-on: both alter course to starboard"
    elif target_sector in ("starboard", "ahead"):
        situation, role, rules = Situation.CROSSING_STBD, Role.GIVE_WAY, ("15", "16")
        reason = "target crossing from starboard: give way, avoid crossing ahead"
    else:
        situation, role, rules = Situation.CROSSING_PORT, Role.STAND_ON, ("15", "17")
        reason = "target crossing from port: stand on"

    if status_rule is not None:
        return Classification(
            situation,
            Role.GIVE_WAY,
            (*rules, status_rule),
            f"{reason}; target {target.nav_status.lower().replace('_', ' ')}: keep out of her way",
        )
    return Classification(situation, role, rules, reason)
