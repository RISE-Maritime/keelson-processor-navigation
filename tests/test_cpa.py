from __future__ import annotations

import math

import pytest

from nav.traffic.cpa import KNOT_MPS, Kinematics, dead_reckon, geometry, relative_bearing_deg

pytestmark = pytest.mark.unit

NM = 1852.0
LAT, LON = 57.70, 11.90


def at(east_nm=0.0, north_nm=0.0, cog=None, sog=None):
    """A position offset from (LAT, LON) in the same flat frame the module uses."""
    return Kinematics(
        LAT + north_nm / 60.0, LON + east_nm / (60.0 * math.cos(math.radians(LAT))), cog, sog
    )


def crowsnest_compute_cpa(own_lat, own_lon, own_cog, own_sog, t_lat, t_lon, t_cog, t_sog):
    """crowsnest-dev/src/utils/cpaCalculation.js computeCPA, transcribed line for line (nm, min)."""
    d2r = math.pi / 180
    cos_lat = math.cos(own_lat * d2r)
    dx = (t_lon - own_lon) * 60 * cos_lat
    dy = (t_lat - own_lat) * 60
    rng = math.sqrt(dx * dx + dy * dy)
    brg = (math.atan2(dx, dy) / d2r + 360) % 360
    ovx, ovy = own_sog * math.sin(own_cog * d2r), own_sog * math.cos(own_cog * d2r)
    tvx, tvy = t_sog * math.sin(t_cog * d2r), t_sog * math.cos(t_cog * d2r)
    rvx, rvy = tvx - ovx, tvy - ovy
    rel = rvx * rvx + rvy * rvy
    if rel < 0.0001:
        return rng, math.inf, rng, brg
    tcpa_h = -(dx * rvx + dy * rvy) / rel
    cpa = math.hypot(dx + rvx * tcpa_h, dy + rvy * tcpa_h)
    return cpa, tcpa_h * 60, rng, brg


def test_head_on_meets_at_zero_cpa():
    g = geometry(at(cog=0, sog=10), at(north_nm=2, cog=180, sog=10))
    assert g.cpa_m == pytest.approx(0.0, abs=1e-6)
    assert g.tcpa_s == pytest.approx(2 * NM / (20 * KNOT_MPS))
    assert (g.range_m, g.bearing_deg) == (pytest.approx(2 * NM), pytest.approx(0.0))


def test_crossing_from_starboard_passes_ahead():
    own = at(cog=0, sog=10)
    target = at(east_nm=2, north_nm=2, cog=270, sog=10)
    g = geometry(own, target, own_heading_deg=0)
    assert g.cpa_m == pytest.approx(0.0, abs=1e-6) and g.tcpa_s == pytest.approx(720.0)
    # A collision course crosses own heading line at own position: reported, at range zero.
    assert g.bct_s == pytest.approx(720.0)
    assert g.bcr_m == pytest.approx(0.0, abs=1e-6)


def test_bow_crossing_ahead_is_reported_and_astern_is_not():
    own = at(cog=0, sog=5)
    ahead = geometry(own, at(east_nm=1, north_nm=2, cog=270, sog=10), own_heading_deg=0)
    assert ahead.bct_s == pytest.approx(NM / (10 * KNOT_MPS))
    assert ahead.bcr_m == pytest.approx(2 * NM - 5 * KNOT_MPS * ahead.bct_s)
    astern = geometry(own, at(east_nm=1, north_nm=-0.2, cog=270, sog=10), own_heading_deg=0)
    assert astern.bcr_m is None and astern.bct_s is None


def test_overtaking_and_diverging():
    overtaking = geometry(at(cog=0, sog=12), at(east_nm=0.1, north_nm=1, cog=0, sog=6))
    assert overtaking.cpa_m == pytest.approx(0.1 * NM, rel=1e-6) and overtaking.tcpa_s > 0
    diverging = geometry(at(cog=0, sog=10), at(north_nm=-1, cog=180, sog=10))
    assert diverging.tcpa_s < 0


def test_stationary_target_has_a_cpa_whatever_its_course():
    g = geometry(at(cog=90, sog=10), at(east_nm=2, north_nm=0.5, cog=None, sog=0.0))
    assert g.cpa_m == pytest.approx(0.5 * NM, rel=1e-6)


@pytest.mark.parametrize(
    "target", [at(north_nm=1, cog=None, sog=8), at(north_nm=1, cog=90, sog=None)]
)
def test_unknown_velocity_gives_range_and_bearing_only(target):
    g = geometry(at(cog=0, sog=10), target)
    assert g.range_m == pytest.approx(NM) and g.cpa_m is None and g.tcpa_s is None


def test_no_relative_motion_has_cpa_equal_range_and_no_tcpa():
    g = geometry(at(cog=45, sog=10), at(east_nm=1, cog=45, sog=10))
    assert g.cpa_m == pytest.approx(g.range_m) and g.tcpa_s is None


@pytest.mark.parametrize(
    "own, target",
    [
        ((10, 12.0), (2.3, -1.7, 315, 14.0)),
        ((200, 4.0), (-0.4, 3.1, 90, 9.5)),
        ((0, 0.0), (1.0, 1.0, 180, 20.0)),
    ],
)
def test_matches_crowsnest_to_the_float(own, target):
    ocog, osog = own
    e, n, tcog, tsog = target
    o, t = at(cog=ocog, sog=osog), at(e, n, tcog, tsog)
    g = geometry(o, t)
    cpa_nm, tcpa_min, rng_nm, brg = crowsnest_compute_cpa(
        o.lat, o.lon, ocog, osog, t.lat, t.lon, tcog, tsog
    )
    assert g.cpa_m / NM == pytest.approx(cpa_nm, rel=1e-9)
    assert g.tcpa_s / 60 == pytest.approx(tcpa_min, rel=1e-9)
    assert g.range_m / NM == pytest.approx(rng_nm, rel=1e-9)
    assert g.bearing_deg == pytest.approx(brg, abs=1e-9)


def test_dead_reckon_moves_along_velocity_only_when_known():
    t = at(cog=90, sog=10)
    moved = dead_reckon(t, 60.0)
    g = geometry(Kinematics(t.lat, t.lon), moved)
    assert g.range_m == pytest.approx(
        10 * KNOT_MPS * 60, rel=1e-3
    ) and g.bearing_deg == pytest.approx(90, abs=0.01)
    unknown = at(cog=None, sog=10)
    assert dead_reckon(unknown, 60.0) == unknown


def test_relative_bearing():
    assert relative_bearing_deg(10.0, 350.0) == pytest.approx(20.0)
    assert relative_bearing_deg(350.0, 10.0) == pytest.approx(340.0)
