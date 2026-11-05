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


APPROACH_ID = "route_monitor/route_deviation_approach/declared-sim"


def drift(proc, world, plan, start_s, end_s, xtd_at):
    """Ticks once a second; returns the alarm outputs of each tick, by second."""
    out = {}
    for t in range(start_s, end_s + 1):
        # Re-declared every tick, as a live sim does: an unheard declaration goes stale in 35 s.
        world.put_declared("sim", plan, T0 + t * S, live=True)
        feed(world, T0 + t * S, on_leg0(plan, 100.0 + 4.0 * t, xtd_at(t)))
        out[t] = by_subject(proc.tick(T0 + t * S), "alarm")
    return out


def test_approach_caution_before_the_limit_then_the_warning_past_it(declared):
    """crowsnest-dev#253: closing on the 50 m starboard limit at 0.5 m/s raises a CAUTION once the
    limit is under 60 s away (xtd > 20 m), and the deviation WARNING replaces it past the limit."""
    proc, world, plan = declared
    alarms = drift(proc, world, plan, 0, 30, lambda t: 0.5 * t)
    # At 0.5 m/s, 60 s to the 50 m limit is 20 m off the line: second 40. Not yet at second 30.
    assert APPROACH_ID not in proc.monitor.alarms.active_identifiers
    alarms = drift(proc, world, plan, 31, 45, lambda t: 0.5 * t)
    assert APPROACH_ID in proc.monitor.alarms.active_identifiers
    raised = [a for t in alarms for a in alarms[t] if a.payload["identifier"] == APPROACH_ID]
    assert raised[0].payload["priority"] == "PRIORITY_CAUTION"
    assert "starboard cross-track limit (50 m)" in raised[0].payload["description"]
    # Past the limit: the WARNING stands and the CAUTION stands down.
    drift(proc, world, plan, 46, 105, lambda t: 0.5 * t)
    active = proc.monitor.alarms.active_identifiers
    assert "route_monitor/route_deviation/declared-sim" in active
    assert APPROACH_ID not in active


def test_approach_caution_clears_when_she_turns_away(declared):
    proc, world, plan = declared
    drift(proc, world, plan, 0, 45, lambda t: 0.5 * t)
    assert APPROACH_ID in proc.monitor.alarms.active_identifiers
    # Back toward the line: no time to a limit, the CAUTION clears.
    drift(proc, world, plan, 46, 60, lambda t: 22.5 - 0.5 * (t - 45))
    assert APPROACH_ID not in proc.monitor.alarms.active_identifiers


def test_approach_caution_hysteresis(declared):
    """It holds until the time to the limit is above 1.5 x 60 s, so a rate that eases a little does
    not flicker it."""
    proc, world, plan = declared
    drift(proc, world, plan, 0, 45, lambda t: 0.5 * t)  # 22.5 m, 55 s to go
    assert APPROACH_ID in proc.monitor.alarms.active_identifiers
    # Slowing to 0.3 m/s for 10 s, to 25.5 m: 24.5 / 0.3 = 82 s -- above 60, below 90: holds.
    drift(proc, world, plan, 46, 55, lambda t: 22.5 + 0.3 * (t - 45))
    assert APPROACH_ID in proc.monitor.alarms.active_identifiers
    # Slowing to 0.2 m/s for 15 s, to 28.5 m: 21.5 / 0.2 = 108 s -- above 90: clears.
    drift(proc, world, plan, 56, 70, lambda t: 25.5 + 0.2 * (t - 55))
    assert APPROACH_ID not in proc.monitor.alarms.active_identifiers


def test_no_approach_caution_beside_a_standing_warning(declared):
    """Back inside the limit but still within the WARNING's hysteresis (above 0.8 x 50 m), and
    heading out again: the WARNING stands, and a CAUTION beside it would only be noise."""
    proc, world, plan = declared
    drift(proc, world, plan, 0, 120, lambda t: 0.5 * t)  # out to 60 m
    drift(proc, world, plan, 121, 135, lambda t: 60.0 - 1.2 * (t - 120))  # back to 42 m
    drift(proc, world, plan, 136, 146, lambda t: 42.0 + 0.5 * (t - 135))  # out again, 45.5 m
    active = proc.monitor.alarms.active_identifiers
    assert "route_monitor/route_deviation/declared-sim" in active
    assert APPROACH_ID not in active


def test_a_leg_switch_is_not_a_rate(declared):
    """Holding 30 m to starboard of leg 0 into the turn, then on leg 1's line: the cross-track
    jumps 30 m -> 0 at the switch. A slope across that jump would read as 3 m/s toward port and
    raise a CAUTION about a limit she is nowhere near."""
    proc, world, plan = declared
    wp1 = plan.waypoints[1]
    for t in range(0, 31):
        world.put_declared("sim", plan, T0 + t * S, live=True)
        # Captured at 970 m (the 30 m radius), so from second 14 she is on leg 1's line.
        if t < 14:
            pos = on_leg0(plan, 900.0 + 5.0 * t, 30.0)
        else:
            pos = g.destination(wp1.lat, wp1.lon, 90.0, 5.0 * (t - 13))
        feed(world, T0 + t * S, pos)
        proc.tick(T0 + t * S)
        assert APPROACH_ID not in proc.monitor.alarms.active_identifiers, t
    assert proc.monitor.status.progress.leg_index == 1
