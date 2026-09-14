from __future__ import annotations

import keelson
import pytest
from google.protobuf import json_format

from nav.route_monitor import geodesy as g
from nav.runtime import Processor
from nav.world.latch import Sample
from nav.world.model import World
from nav.world.route import LegLimits, RouteLimits, RouteRef, VoyagePlan
from tests.routes import square_route

pytestmark = pytest.mark.unit

S = 1_000_000_000
T0 = 1_800_000_000 * S


def on_leg0(plan, along, offset=0.0):
    a = plan.waypoints[0]
    p = g.destination(a.lat, a.lon, 0.0, along)
    return g.destination(*p, 90.0, offset) if offset else p


def feed(world, t, position, sog=8.0, wind=None):
    world.inputs.put("location_fix", Sample(position, t, t))
    if sog is not None:
        world.inputs.put("speed_over_ground_knots", Sample(sog, t, t))
    if wind is not None:
        world.inputs.put("true_wind_speed_mps", Sample(wind, t, t))


def by_subject(outputs, subject):
    return [o for o in outputs if o.subject == subject]


def parse(type_name, payload):
    cls = keelson.get_protobuf_message_class_from_type_name(type_name)
    return json_format.ParseDict(payload, cls())


@pytest.fixture
def declared(monitor_conf):
    world = World()
    plan = square_route(leg=LegLimits(xtd_port_m=50.0, xtd_starboard_m=50.0))
    world.put_declared("sim", plan, T0, live=True)
    return Processor(monitor_conf, world, id_factory=lambda: "ex-1"), world, plan


def test_declared_route_execution_deviation_alarm_and_state(declared):
    proc, world, plan = declared
    feed(world, T0, on_leg0(plan, 400.0, 80.0))
    out = proc.tick(T0)

    (execution,) = by_subject(out, "route_execution")
    assert execution.source_id == "declared-sim"
    msg = parse("keelson.RouteExecution", execution.payload)
    assert msg.route_ref.route_id == "r1" and msg.voyage_id == ""
    assert (msg.current_leg_from_waypoint_id, msg.next_waypoint_id) == ("wp0", "wp1")
    assert msg.xtd_m == pytest.approx(80.0, abs=0.5) and msg.deviation_active
    assert msg.eta_next_waypoint.ToNanoseconds() > T0

    (alarm,) = by_subject(out, "alarm")
    assert alarm.payload["identifier"] == "route_monitor/route_deviation/declared-sim"
    (state,) = by_subject(out, "navigation_state")
    assert state.payload["mode"] == "MODE_READY"
    assert state.payload["route_ref"] == {"route_id": "r1", "route_edition_number": 1}
    assert state.payload["current_leg_from_waypoint_id"] == "wp0"
    assert proc.dynamic_keys() == {("route_execution", "declared-sim")}

    # 10 Hz tick, 1 Hz execution.
    feed(world, T0 + S // 10, on_leg0(plan, 402.0, 45.0))
    assert by_subject(proc.tick(T0 + S // 10), "route_execution") == []

    # Back inside 0.8 x limit: the alarm clears by expiring.
    feed(world, T0 + S, on_leg0(plan, 410.0, 10.0))
    out = proc.tick(T0 + S)
    assert len(by_subject(out, "route_execution")) == 1
    (cleared,) = by_subject(out, "alarm")
    assert cleared.payload["expiration_time_utc"] == cleared.payload["timestamp"]


def test_hysteresis_keeps_the_alarm_between_clear_fraction_and_limit(declared):
    proc, world, plan = declared
    feed(world, T0, on_leg0(plan, 400.0, 80.0))
    proc.tick(T0)
    # A changing distance inside the hysteresis band must not republish the alarm every tick.
    for i in range(1, 10):
        feed(world, T0 + i * S // 10, on_leg0(plan, 400.0 + i, 45.0 + i))
        assert by_subject(proc.tick(T0 + i * S // 10), "alarm") == []
    assert proc.monitor.alarms.active_identifiers == {"route_monitor/route_deviation/declared-sim"}


def test_stale_position_stops_execution_and_degrades(declared):
    proc, world, plan = declared
    feed(world, T0, on_leg0(plan, 400.0))
    proc.tick(T0)
    out = proc.tick(T0 + 10 * S)
    assert by_subject(out, "route_execution") == []
    (alarm,) = by_subject(out, "alarm")
    assert alarm.payload["identifier"] == "route_monitor/no_position/declared-sim"
    (state,) = by_subject(out, "navigation_state")
    assert state.payload["degraded_inputs"] == ["location_fix", "speed_over_ground_knots"]
    assert proc.dynamic_keys() == set()


def test_voyage_envelope_breach_opens_and_closes_an_exceedance(monitor_conf):
    world = World()
    plan = square_route(limits=RouteLimits(max_wind_speed_mps=12.0))
    world.put_edition(plan)
    world.put_voyage(VoyagePlan("voy-1", "VOYAGE_STATUS_ACTIVE", RouteRef("r1", 1)))
    proc = Processor(monitor_conf, world, id_factory=lambda: "ex-1")

    feed(world, T0, on_leg0(plan, 300.0), wind=13.0)
    out = proc.tick(T0)
    (opened,) = by_subject(out, "envelope_exceedance")
    assert opened.source_id == "voy-1/ex-1"
    assert parse("keelson.EnvelopeExceedance", opened.payload).responses == [2]  # ALERTED
    (execution,) = by_subject(out, "route_execution")
    assert execution.source_id == "voy-1" and execution.payload["voyage_id"] == "voy-1"
    assert execution.payload["envelope_limits"][0]["state"] == "STATE_BREACHED"
    assert {a.payload["identifier"] for a in by_subject(out, "alarm")} == {
        "route_monitor/envelope/voy-1/max_wind_speed_mps"
    }

    feed(world, T0 + S, on_leg0(plan, 310.0), wind=10.0)
    (closed,) = by_subject(proc.tick(T0 + S), "envelope_exceedance")
    assert closed.payload["cleared_at"] and closed.payload["peak_value"] == 13.0


def test_declared_route_never_produces_exceedances(declared, monitor_conf):
    proc, world, plan = declared
    feed(world, T0, on_leg0(plan, 300.0), wind=99.0)
    assert by_subject(proc.tick(T0), "envelope_exceedance") == []


def test_dangling_voyage_is_idle_with_an_alarm(monitor_conf):
    world = World()
    world.put_edition(square_route(edition=1))
    world.put_voyage(VoyagePlan("voy-1", "VOYAGE_STATUS_ACTIVE", RouteRef("r1", 9)))
    out = Processor(monitor_conf, world).tick(T0)
    assert by_subject(out, "route_execution") == []
    (state,) = by_subject(out, "navigation_state")
    assert state.payload["mode"] == "MODE_IDLE" and "not falling back" in state.payload["reason"]
    (alarm,) = by_subject(out, "alarm")
    assert alarm.payload["identifier"] == "route_monitor/dangling_route/voy-1"


def test_alarms_can_be_disabled(declared, monitor_conf):
    proc, world, plan = declared
    monitor_conf["route_monitor"]["alarms"] = False
    proc = Processor(monitor_conf, world)
    feed(world, T0, on_leg0(plan, 400.0, 80.0))
    out = proc.tick(T0)
    assert by_subject(out, "alarm") == [] and "route_monitor" not in {
        m for m, subjects in proc.publications().items() if subjects
    }
