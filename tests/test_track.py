"""Track keeping: wheel-over, line-of-sight course, helm sign, and a closed loop on a square."""

from __future__ import annotations

import math
from dataclasses import replace

import pytest

from nav.guidance.track import KNOT_MPS, Own, TrackConfig, TrackGuide, wrap180
from nav.route_monitor import geodesy as g
from tests.routes import square_route

pytestmark = pytest.mark.unit


def test_wheel_over_is_radius_times_tan_half_the_turn():
    plan = square_route(radius=30.0)
    assert TrackGuide().wheel_over_m(plan, 1) == pytest.approx(30.0, rel=1e-3)


def test_an_authored_wheel_over_distance_wins():
    plan = square_route(radius=30.0)
    wps = list(plan.waypoints)
    wps[1] = replace(wps[1], wheel_over_distance_m=120.0)
    assert TrackGuide().wheel_over_m(replace(plan, waypoints=tuple(wps)), 1) == 120.0


def test_wheel_over_never_exceeds_half_the_incoming_leg():
    plan = square_route(leg_m=200.0, radius=1000.0)
    assert TrackGuide().wheel_over_m(plan, 1) == pytest.approx(100.0, rel=1e-3)


def test_turns_onto_the_next_leg_at_the_wheel_over_point_not_the_waypoint():
    plan = square_route(radius=30.0)
    guide = TrackGuide()
    wp1 = plan.waypoints[1]
    before = g.destination(wp1.lat, wp1.lon, 180.0, 40.0)
    after = g.destination(wp1.lat, wp1.lon, 180.0, 25.0)
    assert guide.follow(plan, Own(*before, heading_deg=0.0)).to_waypoint_id == "wp1"
    assert guide.follow(plan, Own(*after, heading_deg=0.0)).to_waypoint_id == "wp2"


def test_arrives_inside_the_capture_radius_of_the_final_waypoint():
    plan = square_route(radius=30.0)
    guide = TrackGuide()
    last = plan.waypoints[-1]
    near = g.destination(last.lat, last.lon, 0.0, 20.0)
    state = guide.follow(plan, Own(*near, heading_deg=180.0))
    assert state.arrived and state.to_waypoint_id == "wp3"


def test_line_of_sight_turns_back_towards_the_track_and_caps_the_intercept():
    plan = square_route()
    guide = TrackGuide()
    start = plan.waypoints[0]
    starboard = g.destination(*g.destination(start.lat, start.lon, 0.0, 300.0), 90.0, 50.0)
    state = guide.follow(plan, Own(*starboard, heading_deg=0.0))
    assert state.xte_m > 0
    course = guide.course(state, Own(*starboard, heading_deg=0.0, sog_knots=0.0))
    assert wrap180(course - 0.0) == pytest.approx(-math.degrees(math.atan2(state.xte_m, 100.0)))
    far = g.destination(*g.destination(start.lat, start.lon, 0.0, 300.0), 270.0, 900.0)
    far_state = TrackGuide().follow(plan, Own(*far, heading_deg=0.0))
    assert far_state.to_waypoint_id == "wp1" and far_state.xte_m < 0
    assert wrap180(guide.course(far_state, Own(*far, heading_deg=0.0))) == pytest.approx(45.0)


def test_helm_is_positive_to_starboard_and_damped_by_yaw_rate():
    guide = TrackGuide()
    assert guide.helm(20.0, Own(0, 0, heading_deg=0.0)).steering_pct == pytest.approx(40.0)
    assert guide.helm(340.0, Own(0, 0, heading_deg=0.0)).steering_pct == pytest.approx(-40.0)
    damped = guide.helm(20.0, Own(0, 0, heading_deg=0.0, yaw_rate_degps=5.0))
    assert damped.steering_pct == pytest.approx(30.0)
    assert guide.helm(179.0, Own(0, 0, heading_deg=0.0)).steering_pct == 100.0


def test_drift_is_crabbed_out():
    guide = TrackGuide()
    # Set to starboard: making good 010 while heading 000. To make good 000, head 350.
    order = guide.helm(0.0, Own(0, 0, heading_deg=0.0, cog_deg=10.0, sog_knots=8.0))
    assert order.heading_deg == pytest.approx(350.0)


def _sail(plan, cfg: TrackConfig, sog_knots=8.0, turn_rate=12.0, dt=0.1, seconds=2000.0):
    """An L0-like hull: steering percent is a share of the turn rate, no inertia, no current."""
    guide = TrackGuide(cfg)
    wp0 = plan.waypoints[0]
    lat, lon, heading = wp0.lat, wp0.lon, 0.0
    worst = 0.0
    for _ in range(int(seconds / dt)):
        own = Own(lat, lon, heading, heading, sog_knots, None)
        state = guide.follow(plan, own)
        if state.arrived:
            return worst, True
        order = guide.helm(guide.course(state, own), own)
        yaw = order.steering_pct / 100.0 * turn_rate
        heading = (heading + yaw * dt) % 360.0
        lat, lon = g.destination(lat, lon, heading, sog_knots * KNOT_MPS * dt)
        a, b = plan.waypoints[state.leg_index], plan.waypoints[state.leg_index + 1]
        worst = max(worst, abs(g.cross_track_m(a.lat, a.lon, b.lat, b.lon, lat, lon)))
    return worst, False


def test_a_closed_loop_holds_the_track_and_arrives():
    worst, arrived = _sail(square_route(radius=30.0), TrackConfig())
    assert arrived
    assert worst < 40.0
