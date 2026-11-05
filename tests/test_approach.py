"""The rate across the track and the time to a limit (crowsnest-dev#253).

The numbers match Crowsnest's passageKpis.js lateralRate / timeToLimit, so PASSAGE's "time to
limit" and the navigator's approach CAUTION cannot disagree about the same fix.
"""

from __future__ import annotations

import pytest

from nav.route_monitor.approach import RATE_FLOOR_MPS, LateralRate, time_to_limit

pytestmark = pytest.mark.unit

S = 1_000_000_000


def fed(points, key="leg0"):
    r = LateralRate()
    for t_s, x in points:
        r.add(key, int(t_s * S), x)
    return r


def test_rate_is_the_slope_toward_starboard_positive():
    assert fed([(t, 2.0 + 0.5 * t) for t in range(6)]).rate_mps() == pytest.approx(0.5)
    assert fed([(t, -0.25 * t) for t in range(6)]).rate_mps() == pytest.approx(-0.25)


def test_no_rate_from_too_few_fixes_or_too_short_a_span():
    assert LateralRate().rate_mps() is None
    assert fed([(0, 0.0), (5, 1.0)]).rate_mps() is None  # two fixes
    assert fed([(0, 0.0), (1, 1.0), (2, 2.0)]).rate_mps() is None  # 2 s span
    assert fed([(0, 0.0), (1.5, 1.0), (3, 2.0)]).rate_mps() is not None  # 3 s span


def test_window_keeps_the_last_ten_seconds():
    # 20 s drifting to port, then 10 s drifting to starboard: only the last 10 s count.
    pts = [(t, -1.0 * t) for t in range(21)] + [(20 + t, -20.0 + 1.0 * t) for t in range(1, 11)]
    assert fed(pts).rate_mps() == pytest.approx(1.0)


def test_a_repeated_fix_is_not_counted_twice():
    r = fed([(0, 0.0), (1, 1.0), (2, 2.0), (3, 3.0)])
    before = r.rate_mps()
    for _ in range(20):
        r.add("leg0", 3 * S, 3.0)
    assert r.rate_mps() == pytest.approx(before)


def test_a_new_leg_starts_the_rate_over():
    r = fed([(t, 0.5 * t) for t in range(6)])
    # The next leg's cross-track jumps; a slope across the jump would be a phantom rate.
    r.add("leg1", 6 * S, -40.0)
    assert r.rate_mps() is None


def test_time_to_limit_on_the_side_she_is_moving_toward():
    # 10 m to starboard of the line, 30 m starboard limit, closing at 0.5 m/s: 40 s.
    assert time_to_limit(10.0, 0.5, 20.0, 30.0) == pytest.approx(40.0)
    # Same fix moving to port toward a 20 m port limit at 1 m/s: 30 s (10 + 20).
    assert time_to_limit(10.0, -1.0, 20.0, 30.0) == pytest.approx(30.0)


def test_time_to_limit_unknowns_are_none_and_outside_is_zero():
    assert time_to_limit(None, 0.5, 20.0, 20.0) is None
    assert time_to_limit(5.0, None, 20.0, 20.0) is None
    assert time_to_limit(5.0, RATE_FLOOR_MPS / 2, 20.0, 20.0) is None  # not moving across
    assert time_to_limit(5.0, 0.5, 20.0, None) is None  # no limit on that side
    assert time_to_limit(25.0, -0.5, 20.0, 20.0) == 0.0  # already outside, starboard
    assert time_to_limit(-25.0, 0.5, 20.0, 20.0) == 0.0  # already outside, port
