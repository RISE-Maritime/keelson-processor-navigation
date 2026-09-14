"""COLREG classification truth table.

Each case states the geometry a watchkeeper would recognise.
"""

from __future__ import annotations

import pytest

from nav.colreg.classify import Role, Situation, Vessel, classify

pytestmark = pytest.mark.unit


def v(heading, sog=10.0, status=None, cog=None):
    return Vessel(heading, cog, sog, status)


OWN_NORTH = v(0.0)


@pytest.mark.parametrize(
    "case, own, target, bearing, closing, situation, role, rule",
    [
        (
            "head-on dead ahead",
            OWN_NORTH,
            v(180.0),
            0.0,
            True,
            Situation.HEAD_ON,
            Role.GIVE_WAY,
            "14",
        ),
        (
            "head-on 5 deg off",
            OWN_NORTH,
            v(183.0),
            355.0,
            True,
            Situation.HEAD_ON,
            Role.GIVE_WAY,
            "14",
        ),
        (
            "nearly reciprocal but broad: crossing",
            OWN_NORTH,
            v(200.0),
            10.0,
            True,
            Situation.CROSSING_STBD,
            Role.GIVE_WAY,
            "15",
        ),
        (
            "crossing from starboard",
            OWN_NORTH,
            v(270.0),
            45.0,
            True,
            Situation.CROSSING_STBD,
            Role.GIVE_WAY,
            "15",
        ),
        (
            "crossing from port",
            OWN_NORTH,
            v(90.0),
            315.0,
            True,
            Situation.CROSSING_PORT,
            Role.STAND_ON,
            "17",
        ),
        (
            "overtaking a slower ship ahead",
            v(0.0, 12),
            v(10.0, 5),
            2.0,
            True,
            Situation.OVERTAKING,
            Role.GIVE_WAY,
            "13",
        ),
        (
            "being overtaken from astern",
            v(0.0, 5),
            v(0.0, 12),
            180.0,
            True,
            Situation.OVERTAKEN,
            Role.STAND_ON,
            "13",
        ),
        (
            "overtaken from the starboard quarter",
            v(0.0, 5),
            v(20.0, 12),
            150.0,
            True,
            Situation.OVERTAKEN,
            Role.STAND_ON,
            "13",
        ),
        (
            "anchored ship ahead",
            OWN_NORTH,
            v(90.0, 0.0, "AT_ANCHOR"),
            0.0,
            True,
            Situation.NONE,
            Role.GIVE_WAY,
            "8",
        ),
        (
            "drifting target counts as stopped",
            OWN_NORTH,
            v(90.0, 0.2),
            30.0,
            True,
            Situation.NONE,
            Role.GIVE_WAY,
            "8",
        ),
        (
            "fishing vessel on port bow: keep clear",
            OWN_NORTH,
            v(90.0, 4, "FISHING"),
            315.0,
            True,
            Situation.CROSSING_PORT,
            Role.GIVE_WAY,
            "18(a)(iii)",
        ),
        (
            "not under command crossing from port",
            OWN_NORTH,
            v(90.0, 3, "NOT_UNDER_COMMAND"),
            320.0,
            True,
            Situation.CROSSING_PORT,
            Role.GIVE_WAY,
            "18(a)(i)",
        ),
    ],
)
def test_truth_table(case, own, target, bearing, closing, situation, role, rule):
    c = classify(own, target, bearing, closing)
    assert (c.situation, c.role) == (situation, role), case
    assert rule in c.rules, (case, c.rules)


def test_heading_preferred_over_course_for_aspect():
    # Target steering 270 but set to make good 180: she presents her side, so it is a crossing.
    crabbing = Vessel(heading_deg=270.0, cog_deg=180.0, sog_knots=8.0)
    assert classify(OWN_NORTH, crabbing, 30.0, True).situation == Situation.CROSSING_STBD


def test_course_used_when_heading_unknown():
    no_heading = Vessel(heading_deg=None, cog_deg=180.0, sog_knots=10.0)
    assert classify(OWN_NORTH, no_heading, 0.0, True).situation == Situation.HEAD_ON


def test_restricted_visibility_has_no_stand_on_vessel():
    fwd = classify(OWN_NORTH, v(90.0), 315.0, True, restricted_visibility=True)
    aft = classify(OWN_NORTH, v(0.0, 12), 160.0, True, restricted_visibility=True)
    assert (fwd.situation, fwd.role, fwd.rules) == (Situation.NONE, Role.NONE, ("19(d)",))
    assert "avoid altering to port" in fwd.reason
    assert "avoid altering towards it" in aft.reason and "starboard" in aft.reason


def test_unknown_target_direction_assumes_keep_clear():
    c = classify(OWN_NORTH, Vessel(None, None, 10.0), 20.0, True)
    assert (c.situation, c.role) == (Situation.UNSPECIFIED, Role.GIVE_WAY)


def test_unknown_own_direction_classifies_nothing():
    c = classify(Vessel(None, None, 10.0), v(180.0), 0.0, True)
    assert c.role == Role.UNSPECIFIED


def test_opening_target_abaft_is_no_encounter():
    c = classify(v(0.0, 12), v(0.0, 5), 180.0, closing=False)
    assert (c.situation, c.role) == (Situation.NONE, Role.NONE)


@pytest.mark.parametrize(
    "target_heading, head_on", [(174.0, True), (186.0, True), (172.0, False), (190.0, False)]
)
def test_head_on_is_bounded_by_course_difference(target_heading, head_on):
    c = classify(OWN_NORTH, v(target_heading), 2.0, True)
    assert (c.situation == Situation.HEAD_ON) is head_on
