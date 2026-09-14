from __future__ import annotations

import pytest

from nav.route_monitor import geodesy as g
from nav.route_monitor.progress import ProgressConfig, ProgressTracker
from nav.world.route import LegLimits, RoutePlan
from tests.routes import square_route

pytestmark = pytest.mark.unit


def point_on_leg(plan: RoutePlan, i: int, along_m: float, offset_m: float = 0.0):
    a, b = plan.waypoints[i], plan.waypoints[i + 1]
    brg = g.bearing_deg(a.lat, a.lon, b.lat, b.lon)
    p = g.destination(a.lat, a.lon, brg, along_m)
    return g.destination(*p, (brg + 90.0) % 360.0, offset_m) if offset_m else p


def test_before_the_first_waypoint_is_leg_zero_not_the_last_leg():
    plan = square_route()
    behind = g.destination(plan.waypoints[0].lat, plan.waypoints[0].lon, 180.0, 300.0)
    p = ProgressTracker().update(plan, *behind, 10.0)
    assert (p.leg_index, p.from_waypoint_id, p.next_waypoint_id) == (0, "wp0", "wp1")


def test_xtd_sign_and_side_specific_threshold():
    plan = square_route(leg=LegLimits(xtd_port_m=20.0, xtd_starboard_m=60.0))
    stbd = ProgressTracker().update(plan, *point_on_leg(plan, 0, 400.0, 40.0), 10.0)
    port = ProgressTracker().update(plan, *point_on_leg(plan, 0, 400.0, -40.0), 10.0)
    assert stbd.xtd_m == pytest.approx(40.0, abs=0.5)
    assert (stbd.deviation_threshold_m, stbd.deviation_active) == (60.0, False)
    assert port.xtd_m == pytest.approx(-40.0, abs=0.5)
    assert (port.deviation_threshold_m, port.deviation_active) == (20.0, True)


def test_threshold_falls_back_to_config_when_not_authored():
    plan = square_route(leg=None)
    p = ProgressTracker(ProgressConfig(xtd_limit_fallback_m=75.0)).update(
        plan, *point_on_leg(plan, 0, 400.0, 10.0), 10.0
    )
    assert p.deviation_threshold_m == 75.0


def test_advances_at_the_capture_radius_and_never_goes_back():
    plan = square_route(radius=30.0)
    t = ProgressTracker()
    assert t.update(plan, *point_on_leg(plan, 0, 900.0), 10.0).leg_index == 0
    assert t.update(plan, *point_on_leg(plan, 0, 975.0), 10.0).leg_index == 1
    # Drifting back onto leg 0 does not move progress backwards.
    assert t.update(plan, *point_on_leg(plan, 0, 800.0), 10.0).leg_index == 1


def test_reacquires_forward_when_a_leg_is_skipped():
    plan = square_route()
    t = ProgressTracker()
    t.update(plan, *point_on_leg(plan, 0, 100.0), 10.0)
    p = t.update(plan, *point_on_leg(plan, 2, 500.0), 10.0)
    assert p.leg_index == 2


def test_arrival_unsets_next_waypoint_and_etas():
    plan = square_route()
    t = ProgressTracker()
    t.update(plan, *point_on_leg(plan, 2, 500.0), 10.0)
    p = t.update(plan, *point_on_leg(plan, 2, 995.0), 10.0)
    assert p.arrived and p.next_waypoint_id is None
    assert p.eta_next_s is None and p.eta_arrival_s is None
    assert p.deviation_active is False


def test_etas_use_sog_now_and_planned_speed_ahead():
    plan = square_route(leg_m=1852.0, sog=6.0)
    p = ProgressTracker().update(plan, *point_on_leg(plan, 0, 926.0), 12.0)
    assert p.eta_next_s == pytest.approx(926.0 / (12.0 * 1852 / 3600), rel=1e-3)
    assert p.eta_arrival_s == pytest.approx(p.eta_next_s + 2 * 3600 / 6.0, rel=1e-3)


def test_no_eta_without_usable_speed():
    plan = square_route(sog=None)
    p = ProgressTracker().update(plan, *point_on_leg(plan, 0, 500.0), 0.1)
    assert p.eta_next_s is None and p.eta_arrival_s is None


def test_new_edition_keeps_its_place_by_waypoint_id():
    plan = square_route(edition=1)
    t = ProgressTracker()
    t.update(plan, *point_on_leg(plan, 1, 200.0), 10.0)
    p = t.update(square_route(edition=2), *point_on_leg(plan, 1, 250.0), 10.0)
    assert p.from_waypoint_id == "wp1"
