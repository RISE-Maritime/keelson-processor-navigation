from __future__ import annotations

from dataclasses import replace

import pytest

from nav.guidance.speed import SpeedConfig, target_sog, throttle_pct
from nav.world.route import LegLimits, RouteLimits
from tests.routes import square_route

pytestmark = pytest.mark.unit

CFG = SpeedConfig(full_ahead_sog_knots=20.0, throttle_kp_pct_per_knot=5.0)


def test_planned_speed_of_the_leg_else_the_default():
    assert target_sog(CFG, square_route(sog=9.0), 0, 5000, False, None)[0] == 9.0
    sog, why = target_sog(CFG, square_route(sog=None), 0, 5000, False, None)
    assert sog == 6.0 and "default" in why


def test_accepted_advice_replaces_the_plan_and_visibility_scales_it():
    assert target_sog(CFG, square_route(sog=9.0), 0, 5000, False, 4.0)[0] == 4.0
    assert target_sog(CFG, square_route(sog=9.0), 0, 5000, True, None)[0] == 4.5


def test_tapers_to_the_arrival_speed_near_the_final_waypoint():
    plan = square_route(sog=10.0)
    assert target_sog(CFG, plan, 2, 150, False, None)[0] == pytest.approx(6.0)
    assert target_sog(CFG, plan, 2, 0, False, None)[0] == pytest.approx(2.0)


def test_leg_route_and_hull_limits_cap_it_and_the_leg_minimum_floors_it():
    plan = square_route(sog=15.0, leg=LegLimits(sog_max_knots=12.0))
    assert target_sog(CFG, plan, 0, 5000, False, None)[0] == 12.0
    plan = replace(square_route(sog=15.0), limits=RouteLimits(max_speed_knots=11.0))
    assert target_sog(CFG, plan, 0, 5000, False, None)[0] == 11.0
    assert target_sog(CFG, square_route(sog=30.0), 0, 5000, False, None)[0] == 20.0
    plan = square_route(sog=3.0, leg=LegLimits(sog_min_knots=5.0))
    assert target_sog(CFG, plan, 0, 5000, False, None)[0] == 5.0
    assert target_sog(CFG, plan, 0, 5000, True, None)[0] == 1.5  # asked for less: not floored


def test_throttle_is_feed_forward_plus_correction_and_never_astern():
    assert throttle_pct(CFG, 10.0, 10.0) == pytest.approx(50.0)
    assert throttle_pct(CFG, 10.0, 8.0) == pytest.approx(60.0)
    assert throttle_pct(CFG, 10.0, None) == pytest.approx(50.0)
    assert throttle_pct(CFG, 1.0, 10.0) == 0.0
    assert throttle_pct(CFG, 0.0, 0.0) == 0.0
