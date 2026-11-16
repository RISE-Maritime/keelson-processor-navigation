"""Route monitoring on an isolated local zenoh peer session: a declared route and position fixes
go in on real keys, a decodable route_execution comes out with its liveliness token."""

from __future__ import annotations

import threading
import time

import keelson
import pytest

from nav import app as app_mod
from nav.route_monitor import geodesy as g
from tests.routes import square_route
from tests.test_e2e_bus import session  # noqa: F401  (fixture)

pytestmark = pytest.mark.e2e


def cls(name):
    return keelson.get_protobuf_message_class_from_type_name(name)


def route_proto(plan):
    r = cls("keelson.Route")(route_id=plan.route_id, route_edition_number=plan.edition)
    for w in plan.waypoints:
        wp = r.waypoints.waypoint.add(id=w.id, planned_sog_knots=8.0, radius_m=30.0)
        wp.position.latitude_deg, wp.position.longitude_deg = w.lat, w.lon
    return r


def test_declared_route_produces_route_execution(session, monitor_conf):  # noqa: F811
    realm, entity = monitor_conf["realm"], monitor_conf["entity_id"]
    key = lambda subject, source: keelson.construct_pubsub_key(realm, entity, subject, source)  # noqa: E731

    received = []
    sub = session.declare_subscriber(
        key("route_execution", "*"),
        lambda s: received.append((str(s.key_expr), bytes(s.payload.to_bytes()))),
    )
    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, monitor_conf, stop), daemon=True)
    worker.start()

    plan = square_route()
    start = plan.waypoints[0]
    route_bytes = keelson.enclose(route_proto(plan).SerializeToString())
    token_keys = []
    try:
        deadline = time.monotonic() + 20
        along = 100.0
        while len(received) < 2 and time.monotonic() < deadline:
            session.put(key("route", "slipway"), route_bytes)
            lat, lon = g.destination(start.lat, start.lon, 0.0, along)
            fix = cls("foxglove.LocationFix")(latitude=lat, longitude=lon)
            fix.timestamp.FromNanoseconds(time.time_ns())
            session.put(key("location_fix", "gnss"), keelson.enclose(fix.SerializeToString()))
            sog = cls("keelson.TimestampedFloat")(value=8.0)
            session.put(
                key("speed_over_ground_knots", "slipway"), keelson.enclose(sog.SerializeToString())
            )
            along += 2.0
            time.sleep(0.2)
        token_keys = [
            str(r.ok.key_expr)
            for r in session.liveliness().get(key("route_execution", "*"))
            if r.ok
        ]
    finally:
        stop.set()
        worker.join(timeout=10)
        sub.undeclare()

    assert not worker.is_alive()
    assert len(received) >= 2
    exec_key, data = received[-1]
    assert exec_key == key("route_execution", "declared-slipway")
    _, _, payload = keelson.uncover(data)
    msg = cls("keelson.RouteExecution").FromString(payload)
    assert msg.route_ref.route_id == "r1"
    assert msg.current_leg_from_waypoint_id == "wp0" and msg.next_waypoint_id == "wp1"
    assert abs(msg.xtd_m) < 1.0 and msg.speed_over_ground_knots == pytest.approx(8.0)
    assert token_keys == [key("route_execution", "declared-slipway")]
