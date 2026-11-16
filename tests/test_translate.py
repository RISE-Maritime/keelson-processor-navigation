from __future__ import annotations

import keelson
import pytest

from nav.bus import translate
from nav.world.route import LegLimits

pytestmark = pytest.mark.unit


def cls(name):
    return keelson.get_protobuf_message_class_from_type_name(name)


def make_route():
    r = cls("keelson.Route")(route_id="r1", route_edition_number=4)
    r.info.max_wind_speed_mps = 12.0
    r.info.min_speed_knots = 0.0  # authored zero is a limit, not absence
    r.waypoints.defaults.planned_sog_knots = 8.0
    r.waypoints.defaults.leg.xtd_port_m = 25.0
    a = r.waypoints.waypoint.add(id="a", name="A")
    a.position.latitude_deg, a.position.longitude_deg = 57.7, 11.9
    a.leg.xtd_starboard_m = 40.0
    b = r.waypoints.waypoint.add(id="b", planned_sog_knots=5.0, radius_m=0.0)
    b.position.latitude_deg, b.position.longitude_deg = 57.71, 11.9
    return r


def test_route_plan_keeps_presence_and_defaults():
    plan = translate.route_plan(make_route())
    assert (plan.route_id, plan.edition) == ("r1", 4)
    assert plan.limits.authored() == {"max_wind_speed_mps": 12.0, "min_speed_knots": 0.0}
    assert plan.waypoints[0].planned_sog_knots is None
    assert plan.planned_sog_knots(0) == 8.0 and plan.planned_sog_knots(1) == 5.0
    assert plan.waypoints[1].radius_m == 0.0
    assert plan.leg(0) == LegLimits(xtd_port_m=25.0, xtd_starboard_m=40.0)
    assert plan.leg(1) == LegLimits(xtd_port_m=25.0)


def test_waypoint_without_position_is_rejected():
    r = cls("keelson.Route")(route_id="r1")
    r.waypoints.waypoint.add(id="a")
    with pytest.raises(translate.TranslateError, match="no position"):
        translate.route_plan(r)


def test_voyage_plan():
    v = cls("keelson.Voyage")(voyage_id="v1", status=2)
    v.route_ref.route_id, v.route_ref.route_edition_number = "r1", 4
    plan = translate.voyage_plan(v)
    assert plan.status == "VOYAGE_STATUS_ACTIVE"
    assert (plan.route_ref.route_id, plan.route_ref.edition) == ("r1", 4)
    assert translate.voyage_plan(cls("keelson.Voyage")(voyage_id="v2")).route_ref is None


def test_samples():
    fix = cls("foxglove.LocationFix")(latitude=57.7, longitude=11.9)
    fix.timestamp.FromNanoseconds(123)
    s = translate.sample(fix, received_ns=999, source_id="gnss")
    assert (s.value, s.measured_ns, s.received_ns, s.source_id) == ((57.7, 11.9), 123, 999, "gnss")

    sog = cls("keelson.TimestampedFloat")(value=6.5)
    assert translate.sample(sog, 1, "slipway").measured_ns is None
    gust = cls("keelson.TimestampedFloatPeriod")(value=14.0)
    assert translate.sample(gust, 1, "wx").value == 14.0

    with pytest.raises(translate.TranslateError):
        translate.sample(cls("keelson.TimestampedFloat")(value=float("nan")), 1, "slipway")


def test_string_and_nav_status_samples():
    name = cls("keelson.TimestampedString")(value="STENA DANICA")
    assert translate.sample(name, 1, "ais").value == "STENA DANICA"
    status = cls("keelson.VesselNavStatus")(navigation_status=2)
    assert translate.sample(status, 1, "ais").value == "AT_ANCHOR"
