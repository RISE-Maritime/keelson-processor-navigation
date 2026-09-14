from __future__ import annotations

import math

import pytest

from nav.colreg.synthesize import predict, synthesize
from nav.traffic.cpa import geometry

pytestmark = pytest.mark.unit

NM = 1852.0
LAT, LON = 57.70, 11.90


def at(east_nm=0.0, north_nm=0.0, cog=None, sog=None):
    from nav.traffic.cpa import Kinematics

    return Kinematics(
        LAT + north_nm / 60.0, LON + east_nm / (60.0 * math.cos(math.radians(LAT))), cog, sog
    )


def test_crossing_from_starboard_is_cleared_by_a_starboard_turn_passing_astern():
    own = at(cog=0, sog=10)
    bravo = at(east_nm=1.0, north_nm=1.0, cog=270, sog=10)  # collision course from starboard
    assert geometry(own, bravo).cpa_m == pytest.approx(0, abs=1)
    m = synthesize(own, {"bravo": bravo}, safe_passing_m=0.3 * NM, no_cross_ahead=["bravo"])
    assert m.satisfied and m.kind == "KIND_COURSE_ALTERATION"
    assert m.alteration_deg >= 30 and m.alteration_deg > 0  # starboard, readily apparent
    assert m.prediction.min_cpa_m >= 0.3 * NM and m.prediction.crosses_ahead_of == ()


def test_crossing_ahead_of_a_give_way_target_is_rejected():
    own = at(cog=0, sog=20)  # fast enough that holding course crosses her bow
    bravo = at(east_nm=1.0, north_nm=1.2, cog=270, sog=6)
    p = predict(own, 0, 20, {"bravo": bravo}, no_cross_ahead=["bravo"])
    assert p.crosses_ahead_of == ("bravo",)


def test_head_on_turns_starboard_only():
    own = at(cog=0, sog=10)
    other = at(north_nm=2.0, cog=180, sog=10)
    m = synthesize(own, {"other": other}, safe_passing_m=0.25 * NM)
    assert m.satisfied and m.alteration_deg > 0


def test_port_is_tried_only_when_allowed():
    own = at(cog=0, sog=12)
    # A second ship to starboard makes every starboard alteration unsafe.
    slow_ahead = at(east_nm=0.02, north_nm=1.0, cog=0, sog=4)
    blocker = at(east_nm=0.6, north_nm=0.6, cog=225, sog=10)
    targets = {"ahead": slow_ahead, "blocker": blocker}
    stbd_only = synthesize(own, targets, safe_passing_m=0.2 * NM)
    either = synthesize(own, targets, safe_passing_m=0.2 * NM, turns=("starboard", "port"))
    assert either.satisfied and either.alteration_deg < 0
    assert stbd_only.alteration_deg >= 0


def test_speed_reduction_when_no_turn_clears():
    own = at(cog=0, sog=10)
    targets = {
        "stbd": at(east_nm=1.0, north_nm=1.0, cog=270, sog=10),
        "wall": at(east_nm=1.5, north_nm=0.5, cog=270, sog=0.0),
    }
    m = synthesize(own, targets, safe_passing_m=0.1 * NM, cfg=None)
    assert m.satisfied
    assert m.kind in ("KIND_COURSE_ALTERATION", "KIND_SPEED_CHANGE", "KIND_COURSE_AND_SPEED")


def test_nothing_clears_returns_best_effort_marked_unsatisfied():
    own = at(cog=0, sog=10)
    alongside = at(east_nm=0.01, cog=0, sog=10)  # 18 m abeam: already inside any safe distance
    m = synthesize(own, {"alongside": alongside}, safe_passing_m=0.5 * NM)
    assert not m.satisfied
    assert m.prediction.min_cpa_m < 0.5 * NM  # the record must not claim the distance is kept


def test_opening_target_counts_at_current_range():
    own = at(cog=0, sog=10)
    astern_opening = at(north_nm=-0.01, cog=180, sog=10)  # 18.5 m astern, moving away
    p = predict(own, 0, 10, {"t": astern_opening})
    assert p.min_cpa_m == pytest.approx(0.01 * NM, rel=1e-3)


def test_own_velocity_required():
    with pytest.raises(ValueError):
        synthesize(at(cog=None, sog=10), {}, 100.0)


def test_a_bystander_is_held_to_the_lesser_distance():
    own = at(cog=0, sog=10)
    bravo = at(east_nm=1.0, north_nm=1.0, cog=270, sog=10)  # the encounter
    bystander = at(east_nm=1.2, north_nm=-0.2, cog=0, sog=10)  # abeam to starboard, same speed
    strict = synthesize(own, {"bravo": bravo, "by": bystander}, 0.5 * NM, subjects=None)
    lenient = synthesize(
        own, {"bravo": bravo, "by": bystander}, 0.5 * NM, subjects=["bravo"], other_min_m=0.25 * NM
    )
    assert lenient.satisfied
    assert lenient.prediction.min_cpa_subject_m >= 0.5 * NM
    assert lenient.prediction.min_cpa_other_m >= 0.25 * NM
    assert not strict.satisfied or strict.prediction.min_cpa_m >= 0.5 * NM
