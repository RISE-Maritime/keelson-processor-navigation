"""keelson#324: a cross-track offset, reverse travel and a speed plan factor on the track order."""

from __future__ import annotations

import pytest

from nav.control.orders import GuidanceOrder, OrderError
from nav.guidance.track import TrackState
from nav.world.route import LegLimits, RoutePlan, WaypointPlan
from tests.routes import square_route

pytestmark = pytest.mark.unit


@pytest.mark.parametrize(
    "payload",
    [
        {"track": {"cross_track_offset_m": 25.0}},
        {"track": {"cross_track_offset_fraction": -0.5, "reverse": True}},
        {"track": {"resume_from_waypoint_id": "wp1", "reverse": True}, "speed_plan_factor": 1.1},
    ],
)
def test_the_new_track_fields_round_trip(payload):
    from google.protobuf import json_format
    from keelson.payloads.NavigationState_pb2 import GuidanceOrder as Proto

    order = GuidanceOrder.from_payload(payload)
    assert order.to_payload() == payload
    back = json_format.MessageToDict(
        json_format.ParseDict(order.to_payload(), Proto()), preserving_proto_field_name=True
    )
    assert GuidanceOrder.from_payload(back) == order


@pytest.mark.parametrize(
    "payload",
    [
        {"track": {"cross_track_offset_m": 10.0, "cross_track_offset_fraction": 0.1}},
        {"track": {"cross_track_offset_fraction": 1.5}},
        {"track": {}, "speed_knots": 5.0, "speed_plan_factor": 1.1},
        {"track": {}, "speed_plan_factor": 0.0},
    ],
)
def test_the_new_fields_are_refused_when_they_contradict_or_overflow(payload):
    with pytest.raises(OrderError):
        GuidanceOrder.from_payload(payload)


def test_describe_says_direction_offset_and_trim():
    order = GuidanceOrder.from_payload(
        {"track": {"reverse": True, "cross_track_offset_m": 25.0}, "speed_plan_factor": 1.1}
    )
    assert order.describe() == "track reversed, STBD 25 m, plan +10 %"


def test_reversed_plan_sails_the_legs_backwards_with_port_and_starboard_swapped():
    wps = (
        WaypointPlan(
            "a",
            57.0,
            11.0,
            planned_sog_knots=5.0,
            leg=LegLimits(xtd_port_m=10.0, xtd_starboard_m=40.0),
        ),
        WaypointPlan(
            "b",
            57.01,
            11.0,
            planned_sog_knots=8.0,
            leg=LegLimits(xtd_port_m=20.0, xtd_starboard_m=30.0),
        ),
        WaypointPlan("c", 57.02, 11.0),
    )
    rev = RoutePlan("r", 3, wps).reversed()
    assert [w.id for w in rev.waypoints] == ["c", "b", "a"]
    assert (rev.route_id, rev.edition) == ("r", 3)  # the same edition, not a new route
    # c -> b is b -> c backwards: its limits, swapped, and its planned speed.
    assert (rev.leg(0).xtd_port_m, rev.leg(0).xtd_starboard_m) == (30.0, 20.0)
    assert rev.planned_sog_knots(0) == 8.0
    assert (rev.leg(1).xtd_port_m, rev.leg(1).xtd_starboard_m) == (40.0, 10.0)
    assert rev.planned_sog_knots(1) == 5.0


def _state(xte, leg_index=0):
    return TrackState(
        leg_index=leg_index,
        from_waypoint_id="wp0",
        to_waypoint_id="wp1",
        leg_bearing_deg=0.0,
        xte_m=xte,
        distance_to_final_m=1000.0,
        arrived=False,
    )


def test_an_offset_makes_the_line_parallel_to_the_leg(guidance_conf_like):
    from nav.guidance.guide import Guidance
    from nav.guidance.track import Own

    g = Guidance(guidance_conf_like)
    own = Own(lat=57.0, lon=11.0, heading_deg=0.0, cog_deg=0.0, sog_knots=8.0, yaw_rate_degps=0.0)
    # On the offset line: no intercept, she steers the leg's bearing.
    assert g.track.course(_state(25.0), own, 25.0) == pytest.approx(0.0)
    # On the centreline with a starboard offset: she comes right to reach it.
    assert 0.0 < g.track.course(_state(0.0), own, 25.0) < 90.0


def test_a_corridor_share_is_resolved_per_leg_and_kept_inside(guidance_conf_like):
    from nav.guidance.guide import OFFSET_CAP, Guidance

    g = Guidance(guidance_conf_like)
    g.state = _state(0.0, leg_index=0)
    plan = square_route(leg=LegLimits(xtd_port_m=20.0, xtd_starboard_m=60.0))
    half = GuidanceOrder.from_payload({"track": {"cross_track_offset_fraction": 0.5}})
    assert g.track_offset_m(half, plan) == pytest.approx(30.0)
    port = GuidanceOrder.from_payload({"track": {"cross_track_offset_fraction": -1.0}})
    assert g.track_offset_m(port, plan) == pytest.approx(-OFFSET_CAP * 20.0)
    # No corridor authored: the monitor's fallback limit.
    fallback = guidance_conf_like["route_monitor"]["xtd_limit_fallback_m"]
    assert g.track_offset_m(half, square_route()) == pytest.approx(0.5 * fallback)
    assert (
        g.track_offset_m(
            GuidanceOrder.from_payload({"track": {"cross_track_offset_m": -12.0}}), plan
        )
        == -12.0
    )
