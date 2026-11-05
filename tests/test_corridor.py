"""The XTD limits where the ship is, on a VARIABLE corridor (crowsnest-dev#287).

The literal corridor is Crowsnest's scripts/checks/routeDeviationAlerts.mjs, so the two monitors are
pinned to the same numbers: a 2.2 km leg due north at 11.60 E, the PORT boundary 30 m -> 8 m abeam
57.610 N -> 40 m at 57.620 N, the STARBOARD a plain 20 m."""

from __future__ import annotations

import math

import pytest

from nav.route_monitor.corridor import limits_at_ship, limits_where_ship_is
from nav.route_monitor.progress import ProgressTracker
from nav.world.route import LegLimits, RoutePlan, WaypointPlan

pytestmark = pytest.mark.unit

M = 6371008.8 * math.pi / 180.0
A, B = (57.6, 11.6), (57.62, 11.6)


def lon_at(lat: float, east_m: float) -> float:
    return 11.6 + east_m / (M * math.cos(math.radians(lat)))


PORT = ((57.6, lon_at(57.6, -30)), (57.61, lon_at(57.61, -8)), (57.62, lon_at(57.62, -40)))
STBD = ((57.6, lon_at(57.6, 20)), (57.62, lon_at(57.62, 20)))


def near(got, want, tol=0.2):
    return got is not None and abs(got - want) <= tol


# ---------------------------------------------------------------------------- the geometry


def test_abeam_the_narrows_8_port_20_starboard():
    port, stbd = limits_at_ship(A, B, 57.61, 11.6, PORT, STBD)
    assert near(port, 8) and near(stbd, 20)


def test_near_the_end_33_6_port_not_the_legs_narrowest():
    port, stbd = limits_at_ship(A, B, 57.618, 11.6, PORT, STBD)
    assert near(port, 33.6) and near(stbd, 20)  # 8 + 0.8 x 32


def test_the_ships_own_offset_does_not_move_the_limit():
    """Measured from the TRACK at her abeam point, not from her: 12 m to port she still has 8 m."""
    port, _ = limits_at_ship(A, B, 57.61, lon_at(57.61, -12), PORT, STBD)
    assert near(port, 8)


def test_a_boundary_that_does_not_reach_her_abeam_line_is_none_and_filled_from_the_leg():
    short = ((57.6, lon_at(57.6, -30)), (57.605, lon_at(57.605, -19)))
    here = limits_at_ship(A, B, 57.615, 11.6, short, STBD)
    assert here[0] is None and near(here[1], 20)
    assert limits_where_ship_is(here, (8.0, 20.0))[0] == 8.0


def test_no_boundary_and_a_single_vertex_are_none():
    assert limits_at_ship(A, B, 57.61, 11.6, (), ((57.61, lon_at(57.61, 20)),)) == (None, None)


def test_a_side_crossing_on_the_wrong_side_of_the_track_is_not_its_limit():
    """A 'port' line drawn to starboard is no port limit at all."""
    wrong = ((57.6, lon_at(57.6, 10)), (57.62, lon_at(57.62, 10)))
    assert limits_at_ship(A, B, 57.61, 11.6, wrong, STBD)[0] is None


# ---------------------------------------------------------------------------- the monitor


def plan(variable=True, port=PORT, stbd=STBD):
    leg = LegLimits(xtd_port_m=8.0, xtd_starboard_m=20.0)
    return RoutePlan(
        "r",
        1,
        (
            WaypointPlan(
                "a",
                *A,
                leg=leg,
                xtd_variable=variable,
                xtd_port_boundary=port,
                xtd_starboard_boundary=stbd,
            ),
            WaypointPlan("b", *B),
        ),
    )


def progress(p, lat, east_m):
    return ProgressTracker().update(p, lat, lon_at(lat, east_m), 6.0)


def test_twelve_metres_to_port_is_a_deviation_at_the_narrows_and_not_where_it_is_wide():
    narrow = progress(plan(), 57.61, -12)
    assert narrow.deviation_active and near(narrow.deviation_threshold_m, 8)
    wide = progress(plan(), 57.618, -12)
    assert not wide.deviation_active and near(wide.deviation_threshold_m, 33.6)


def test_the_approach_reads_the_limits_where_she_is():
    p = progress(plan(), 57.618, -12)
    assert near(p.xtd_port_limit_m, 33.6) and near(p.xtd_stbd_limit_m, 20)


def test_a_fixed_leg_never_sails_a_stale_boundary():
    p = progress(plan(variable=False), 57.618, -12)
    assert p.deviation_active and p.deviation_threshold_m == 8.0  # the leg's own 8 m


def test_a_variable_leg_without_a_boundary_is_the_legs_limit():
    p = progress(plan(port=(), stbd=()), 57.618, -12)
    assert p.deviation_threshold_m == 8.0 and p.xtd_stbd_limit_m == 20.0


def test_reversed_the_boundaries_move_with_their_leg_and_swap_sides():
    r = plan().reversed()
    first = r.waypoints[0]
    assert (
        first.xtd_variable
        and first.xtd_port_boundary == STBD
        and first.xtd_starboard_boundary == PORT
    )
    assert r.waypoints[-1].xtd_port_boundary == () and not r.waypoints[-1].xtd_variable
    # Sailed south, 12 m to the vessel's STARBOARD (west, the old port side) at the narrows: 8 m.
    p = ProgressTracker().update(r, 57.61, lon_at(57.61, -12), 6.0)
    assert p.xtd_m > 0 and p.deviation_active and near(p.deviation_threshold_m, 8)


# ---------------------------------------------------------------------------- from the wire


def test_the_route_translation_reads_mode_and_boundaries():
    from keelson.payloads.Route_pb2 import Route, XtdCorridorMode

    from nav.bus.translate import route_plan

    route = Route(route_id="r", route_edition_number=1)
    for wid, (lat, lon) in (("a", A), ("b", B)):
        w = route.waypoints.waypoint.add(id=wid)
        w.position.latitude_deg, w.position.longitude_deg = lat, lon
    leg = route.waypoints.waypoint[0].leg
    leg.xtd_corridor_mode = XtdCorridorMode.XTD_CORRIDOR_MODE_VARIABLE
    for lat, lon in PORT:
        v = leg.xtd_port_boundary.add()
        v.position.latitude_deg, v.position.longitude_deg = lat, lon
    leg.xtd_port_boundary.add()  # no position: dropped
    p = route_plan(route)
    assert p.waypoints[0].xtd_variable and len(p.waypoints[0].xtd_port_boundary) == 3
    leg.xtd_corridor_mode = XtdCorridorMode.XTD_CORRIDOR_MODE_FIXED
    assert not route_plan(route).waypoints[0].xtd_variable


def test_a_boundary_crossing_her_abeam_line_twice_is_its_nearer_crossing():
    """A boundary that doubles back (a pocket in the corridor) crosses the abeam line twice: the
    limit is the crossing nearer the track, never the far one."""
    pocket = (
        (57.609, lon_at(57.609, -25)),
        (57.611, lon_at(57.611, -25)),
        (57.611, lon_at(57.611, -8)),
        (57.609, lon_at(57.609, -8)),
    )
    port, _ = limits_at_ship(A, B, 57.61, 11.6, pocket, STBD)
    assert near(port, 8)
