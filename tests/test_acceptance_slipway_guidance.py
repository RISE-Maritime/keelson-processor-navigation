"""Acceptance: guidance sails Slipway's sim-usv-alpha in COMMANDED mode, in-process.

Slipway's kernel runs two-vessel-harbor with alpha handed to vehicle_control; the processor gets
her operational-plane values each tick and its axes are posted to the kernel's command mailbox,
where the kernel's own dead-man judges them. Nothing touches a zenoh session.

The conn and the route arrive the way they do on the bus (navigation_control/v1): an operator
holds the conn, loads her route with its lease token, then grants the conn to this processor.
Calls are queued as ``Pending`` on the World, exactly as bus/control.py queues them.

Run with Slipway's environment, from the Slipway checkout:

    cd ../SLIPWAY
    NAV=../keelson-processor-navigation
    PYTHONPATH=$NAV/src:$NAV .venv/bin/python -m pytest -p no:cacheprovider \\
        ../keelson-processor-navigation/tests/test_acceptance_slipway_guidance.py

Skipped wherever slipway_kernel cannot be imported, which includes this repo's own venv.
"""

from __future__ import annotations

import copy
from pathlib import Path

import pytest

pytest.importorskip("slipway_kernel")

from slipway_kernel import model  # noqa: E402
from slipway_kernel.model import ControlMode  # noqa: E402
from slipway_kernel.publisher import operational_values  # noqa: E402
from slipway_kernel.runtime import KernelRuntime  # noqa: E402
from slipway_kernel.scenario import load_scenario  # noqa: E402

from nav import config as config_mod  # noqa: E402
from nav.conduct.state import Mode  # noqa: E402
from nav.control.commands import Pending  # noqa: E402
from nav.guidance.axes import wanted_mapping  # noqa: E402
from nav.runtime import Processor  # noqa: E402
from nav.world.latch import Sample  # noqa: E402
from nav.world.route import RoutePlan, WaypointPlan  # noqa: E402

SLIPWAY = Path(__file__).resolve().parents[2] / "SLIPWAY"
SCENARIO = SLIPWAY / "scenarios" / "two-vessel-harbor.yaml"
ALPHA = "sim-usv-alpha"
OPERATOR = {"controller_id": "roc@ROC-1", "authority_token": "roc-lease"}
EPOCH = 1_700_000_000 * 10**9
S = 10**9
# The kernel resolves a stop to its next 0.1 s step: she stopped 3.1 s after the pull against a
# 3.0 s neutral burst + dead-man on Slipway main e87b85e, measured and accepted 2026-09-28 (Ted).
STOP_MARGIN_S = 0.1
FLOATS = (
    "heading_true_north_deg",
    "course_over_ground_deg",
    "speed_over_ground_knots",
    "yaw_rate_degps",
)


def nav_conf() -> dict:
    c = copy.deepcopy(config_mod.DEFAULTS)
    c["entity_id"] = ALPHA
    c["modules"].update(route_monitor=True, guidance=True, decision_log=True, control=True)
    c["inputs"] = {"location_fix": "gnss", **{s: "slipway" for s in FLOATS}}
    c["conduct"].update(fixed_authority_level="AUTHORITY_LEVEL_REMOTE_CONTROLLED")
    c["guidance"].update(full_ahead_sog_knots=20.5)
    config_mod._validate(c)
    return c


class Harbour:
    def __init__(self, grant: bool = True):
        self.scenario = load_scenario(SCENARIO, vessel_dir=SLIPWAY / "vessels")
        self.rt = KernelRuntime(self.scenario, run_id="nav-acceptance", sim_epoch_ns=EPOCH)
        self.rt.prime()
        self.rt.set_all_running()
        with self.rt._lock:
            self.rt._world = model.set_control_mode(self.rt._world, ALPHA, ControlMode.COMMANDED)
        self.cfg = self.scenario.vessel_config(ALPHA)
        conf = nav_conf()
        self.p = Processor(conf)
        self.ours = wanted_mapping(config_mod.module_source_id(conf, "guidance"), 1.0)
        self.mapping: dict = {}
        self.plan = self._route()
        self.p.world.put_edition(self.plan)
        self.history: list[tuple] = []
        self.posted = 0
        self.grant = "grant-1"
        self.operator_heartbeat = False
        if grant:
            self._hand_over(conf)
        else:
            self._operator_keeps_the_conn()

    def _operator_keeps_the_conn(self) -> None:
        """The operator holds the conn, heartbeats it, and loads her route with its token. No
        grant: the process carries the operator's order out under the operator's conn."""
        self.grant = "roc-lease"
        self.operator_heartbeat = True
        self._heartbeat(EPOCH)
        loaded = self.call(
            "load_route",
            {"caller": OPERATOR, "route_ref": {"route_id": ALPHA, "route_edition_number": 1}},
        )
        assert loaded["result"] == "COMMAND_RESULT_ACCEPTED", loaded["detail"]

    def _heartbeat(self, now: int) -> None:
        self.p.world.put_lease(
            {"controller_id": "roc@ROC-1", "token": "roc-lease", "lease_ttl_seconds": 30}, now
        )

    def _hand_over(self, conf: dict) -> None:
        """The operator holds the conn, loads her route, then grants the conn to the processor.
        Loading first matters: holding no route, the processor hands a granted conn back."""
        w = self.p.world
        w.put_lease(
            {"controller_id": "roc@ROC-1", "token": "roc-lease", "lease_ttl_seconds": 30}, EPOCH
        )
        loaded = self.call(
            "load_route",
            {"caller": OPERATOR, "route_ref": {"route_id": ALPHA, "route_edition_number": 1}},
        )
        assert loaded["result"] == "COMMAND_RESULT_ACCEPTED", loaded["detail"]
        w.put_lease(
            {
                "controller_id": config_mod.controller_id(conf),
                "controller_site": "ROC-1",
                "token": self.grant,
                "lease_ttl_seconds": 30,
            },
            self.history[-1][0],
        )

    def call(self, procedure: str, request: dict) -> dict:
        """A navigation_control/v1 call, decided on the next tick."""
        pending = Pending(procedure, request, 0)
        self.p.world.put_command(pending)
        self.step()
        assert pending.done.is_set() and pending.error is None, pending.error
        return pending.response

    def order(self, order: dict) -> dict:
        """A guidance order from the operator, who knows the token it granted."""
        caller = {"controller_id": "roc@ROC-1", "authority_token": self.grant}
        return self.call("set_guidance_order", {"caller": caller, "order": order})

    def _route(self) -> RoutePlan:
        """Alpha's circuit as Slipway declares it, starting where she starts."""
        plane = self.scenario.plane
        points = [(self.cfg.initial_east_m, self.cfg.initial_north_m), *self.cfg.waypoints_enu]
        wps = []
        for i, (east, north) in enumerate(points):
            lat, lon = plane.wgs84_from_enu(east, north)
            wps.append(WaypointPlan(str(i), lat, lon, planned_sog_knots=8.0, radius_m=30.0))
        return RoutePlan(ALPHA, 1, tuple(wps))

    def step(self, *, processor_alive: bool = True) -> None:
        world, _ = self.rt.step()
        now = self.rt.sim_timestamp_ns(world)
        vessel = world.vessel(ALPHA)
        values = operational_values(vessel, self.cfg, self.scenario.conditions)
        if self.operator_heartbeat and (now - EPOCH) % (10 * S) < self.scenario.dt_ns:
            self._heartbeat(now)  # Crowsnest heartbeats every 10 s
        if processor_alive:
            w = self.p.world
            w.inputs.put(
                "location_fix",
                Sample(self.scenario.plane.wgs84_from_enu(vessel.east_m, vessel.north_m), now),
            )
            for subject in FLOATS:
                w.inputs.put(subject, Sample(values[subject], now))
            w.put_helm(self.mapping, now)
            self.p.tick(now)
        act = self.p.actuation
        if act.mapping == "install" and not self.mapping:
            self.mapping = self.ours
        elif act.mapping == "release" and self.mapping == self.ours:
            self.mapping = {}
        if act.axes is not None and now <= act.valid_until_ns and self.mapping == self.ours:
            self.rt._mailbox.post(ALPHA, "steering", act.axes[0] / 100.0)
            self.rt._mailbox.post(ALPHA, "throttle", act.axes[1] / 100.0)
            self.posted += 1
        progress = self.p.monitor.status.progress
        self.history.append(
            (now, self.p.conduct.mode, vessel, values["speed_over_ground_knots"], progress, values)
        )

    def run(self, seconds: float, **kw) -> None:
        for _ in range(int(seconds * S / self.scenario.dt_ns)):
            self.step(**kw)

    def run_until(self, mode: Mode, max_seconds: float) -> float:
        for _ in range(int(max_seconds * S / self.scenario.dt_ns)):
            self.step()
            if self.p.conduct.mode is mode:
                return (self.history[-1][0] - self.history[0][0]) / S
        raise AssertionError(f"never reached {mode}; last reason: {self.p.conduct.reason}")


def test_guidance_sails_alpha_round_the_harbour_inside_the_leg_limits_and_stops_on_arrival():
    h = Harbour()
    assert h.run_until(Mode.UNDERWAY, 2.0) < 1.0
    h.run_until(Mode.ENDED, 1800.0)
    underway = [r for r in h.history if r[1] is Mode.UNDERWAY]
    t0 = underway[0][0]
    settled = [r for r in underway if r[0] - t0 > 20 * S and r[4] is not None]
    worst = max(abs(r[4].xtd_m) for r in settled)
    print(f"underway {(underway[-1][0] - t0) / S:.0f} s, worst |xtd| {worst:.1f} m")
    assert worst < h.p.conf["route_monitor"]["xtd_limit_fallback_m"]
    # Steady control, not orders alternating hard over every tick: an unstable loop can still
    # average onto the track, so the track alone does not prove it.
    demands = [(r[2].steering_demand, r[2].throttle_demand) for r in settled]
    jumps = sum(
        1
        for (s0, t0_), (s1, t1) in zip(demands, demands[1:], strict=False)
        if abs(s1 - s0) > 0.5 or abs(t1 - t0_) > 0.3
    )
    print(f"large tick-to-tick demand changes: {jumps} of {len(demands)}")
    assert jumps < 0.01 * len(demands)
    assert not any(r[4].deviation_active for r in settled)
    assert underway[-1][4].from_waypoint_id == "3"
    h.run(10.0)
    assert h.history[-1][3] == 0.0, "alpha still making way after ENDED"
    assert h.p.lease.token is None and h.mapping == {}


def test_pulling_the_lease_stops_her_within_the_neutral_burst_and_dead_man():
    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(120.0)
    assert h.history[-1][3] > 7.0, "not up to speed before the pull"
    pulled = h.history[-1][0]
    h.p.world.put_lease({"token": h.p.lease.token, "released": True}, pulled)
    h.run(60.0)
    after = [r for r in h.history if r[0] > pulled]
    assert after[0][1] is Mode.FALLBACK
    stopped = next(r[0] for r in after if r[3] == 0.0)
    limit = h.p.conf["guidance"]["neutral_burst_s"] + h.cfg.actuators.max_axis_age_s + STOP_MARGIN_S
    print(f"stopped {(stopped - pulled) / S:.1f} s after the pull (limit {limit:.1f} s)")
    assert (stopped - pulled) / S <= limit
    assert all(r[3] == 0.0 for r in after if r[0] >= stopped)
    assert all(r[1] is Mode.FALLBACK for r in after)
    assert h.p.lease.token is None


def test_an_operator_who_requests_the_conn_gets_it_and_she_stops():
    """The way back from "Hand conn to navigator": an operator's command_request is granted, she
    stops within the neutral burst + dead-man, and the operator holds the conn."""
    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(120.0)
    assert h.history[-1][3] > 7.0, "not up to speed before the request"
    asked = h.history[-1][0]
    h.p.world.put_request(
        {"vessel_id": ALPHA, "requester_id": "roc@ROC-1", "requester_site": "ROC-1"}, asked
    )
    h.run(60.0)
    after = [r for r in h.history if r[0] > asked]
    stopped = next(r[0] for r in after if r[3] == 0.0)
    # The request is heard on the next tick: one kernel step before the grant goes out.
    heard_s = h.scenario.dt_ns / S
    limit = (
        h.p.conf["guidance"]["neutral_burst_s"]
        + h.cfg.actuators.max_axis_age_s
        + STOP_MARGIN_S
        + heard_s
    )
    print(f"stopped {(stopped - asked) / S:.1f} s after the request (limit {limit:.1f} s)")
    assert (stopped - asked) / S <= limit
    # The last lease heard is the grant to the operator (who, in this harness, never heartbeats
    # it, so it lapses later; the navigator does not take it back).
    last = h.p.lease_book.last
    assert last is not None and last.controller_id == "roc@ROC-1"
    assert h.p.lease.token is None
    assert h.mapping == {}, "the operator's helm is still mapped to guidance"


def test_a_processor_that_stops_ticking_goes_silent_and_the_kernel_dead_man_stops_her():
    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(60.0)
    died = h.history[-1][0]
    h.run(10.0, processor_alive=False)
    after = [r for r in h.history if r[0] > died]
    stopped = next(r[0] for r in after if r[2].throttle_demand == h.cfg.actuators.failsafe_throttle)
    limit = h.p.conf["guidance"]["command_hold_s"] + h.cfg.actuators.max_axis_age_s + 0.2
    print(
        f"failsafe {(stopped - died) / S:.1f} s after the processor stopped (limit {limit:.1f} s)"
    )
    assert (stopped - died) / S <= limit
    assert after[-1][3] == 0.0


def _heading_error(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def test_a_heading_order_mid_run_is_held_and_track_hands_her_back_onto_the_route():
    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(60.0)
    limit = h.p.conf["route_monitor"]["xtd_limit_fallback_m"]
    before = h.history[-1][5]["heading_true_north_deg"]
    ordered = round((before + 90.0) % 360.0, 1)

    r = h.order({"heading_deg": ordered, "speed_knots": 8.0})
    assert r["result"] == "COMMAND_RESULT_ACCEPTED", r["detail"]
    assert r["state"]["guidance_order"] == {"heading_deg": ordered, "speed_knots": 8.0}
    h.run(30.0)
    assert h.p.conduct.mode is Mode.UNDERWAY
    assert h.p.conduct.sub_mode.value == "SUB_MODE_STEERING"
    last = [row[5]["heading_true_north_deg"] for row in h.history[-50:]]  # the last 5 s
    worst = max(_heading_error(hd, ordered) for hd in last)
    print(f"ordered {ordered:05.1f}, worst heading error over the last 5 s {worst:.1f} deg")
    assert worst < 5.0
    off = abs(h.history[-1][4].xtd_m)
    assert off > limit, f"only {off:.0f} m off track after 30 s at 90 degrees to it"

    back = h.order({"track": {}})
    assert back["result"] == "COMMAND_RESULT_ACCEPTED", back["detail"]
    h.run(120.0)
    assert h.p.conduct.sub_mode.value == "SUB_MODE_TRACK"
    rejoined = [abs(row[4].xtd_m) for row in h.history[-200:] if row[4] is not None]
    print(f"worst |xtd| over the last 20 s after the hand-back: {max(rejoined):.1f} m")
    assert max(rejoined) < limit


def _distance_m(h: Harbour, vessel, target) -> float:
    from nav.route_monitor import geodesy as g

    lat, lon = h.scenario.plane.wgs84_from_enu(vessel.east_m, vessel.north_m)
    return g.haversine_m(lat, lon, *target)


def test_a_go_to_makes_for_the_position_and_holds_there():
    """keelson#308 end to end on the kernel: steer for a point off the route, hold on arrival with
    the order still in force, and stay by it (making back for it past twice the radius)."""
    from nav.route_monitor import geodesy as g

    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(30.0)
    vessel = h.history[-1][2]
    here = h.scenario.plane.wgs84_from_enu(vessel.east_m, vessel.north_m)
    heading = h.history[-1][5]["heading_true_north_deg"]
    target = g.destination(*here, (heading + 90.0) % 360.0, 300.0)
    radius = 20.0
    order = {
        "go_to": {
            "position": {"latitude_deg": target[0], "longitude_deg": target[1]},
            "arrival_radius_m": radius,
        },
        "speed_knots": 6.0,
    }
    r = h.order(order)
    assert r["result"] == "COMMAND_RESULT_ACCEPTED", r["detail"]
    assert r["state"]["guidance_order"]["go_to"]["arrival_radius_m"] == radius
    h.run(5.0)
    assert h.p.conduct.sub_mode.value == "SUB_MODE_STEERING"

    ordered_at = h.history[-1][0]
    for _ in range(int(300 * S / h.scenario.dt_ns)):
        h.step()
        if h.p.conduct.sub_mode.value == "SUB_MODE_HOLDING":
            break
    else:
        raise AssertionError(f"never arrived; {_distance_m(h, h.history[-1][2], target):.0f} m off")
    took = (h.history[-1][0] - ordered_at) / S
    print(f"arrived after {took:.0f} s")
    assert _distance_m(h, h.history[-1][2], target) <= radius

    arrived_at = h.history[-1][0]
    h.run(120.0)
    after = [row for row in h.history if row[0] > arrived_at]
    worst = max(_distance_m(h, row[2], target) for row in after)
    rest = _distance_m(h, after[-1][2], target)
    print(f"held: worst {worst:.0f} m from the position, {rest:.0f} m at rest, sog {after[-1][3]}")
    assert worst < 4 * radius, "overran the position and did not come back"
    assert rest <= 2 * radius
    assert h.p.conduct.mode is Mode.UNDERWAY
    state = h.call("get_navigation_state", {})
    assert state["guidance_order"] == order  # arrival ends the passage, never the order


def test_a_loiter_drifts_by_the_position_and_makes_back_for_it():
    """keelson#320's loiter on the kernel: make for a point off the route, go neutral near it, and
    stay within the radius band -- making back if carried out past it."""
    from nav.route_monitor import geodesy as g

    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(30.0)
    vessel = h.history[-1][2]
    here = h.scenario.plane.wgs84_from_enu(vessel.east_m, vessel.north_m)
    heading = h.history[-1][5]["heading_true_north_deg"]
    centre = g.destination(*here, (heading + 90.0) % 360.0, 300.0)
    radius = 100.0
    order = {
        "loiter": {
            "position": {"latitude_deg": centre[0], "longitude_deg": centre[1]},
            "radius_m": radius,
            "return_speed_knots": 3.0,
        },
        "speed_knots": 6.0,
    }
    r = h.order(order)
    assert r["result"] == "COMMAND_RESULT_ACCEPTED", r["detail"]
    for _ in range(int(300 * S / h.scenario.dt_ns)):
        h.step()
        if h.p.conduct.sub_mode.value == "SUB_MODE_HOLDING":
            break
    else:
        raise AssertionError(f"never drifting; {_distance_m(h, h.history[-1][2], centre):.0f} m")
    drifting_at = h.history[-1][0]
    print(f"drifting after {(drifting_at - h.history[0][0]) / S:.0f} s")
    h.run(180.0)
    after = [row for row in h.history if row[0] > drifting_at]
    worst = max(_distance_m(h, row[2], centre) for row in after)
    print(f"loiter: worst {worst:.0f} m from the position over 180 s (radius {radius:g} m)")
    # Coasting on after going neutral may carry her past the radius once; she then makes back.
    assert worst < 2 * radius
    assert _distance_m(h, after[-1][2], centre) <= radius
    assert h.p.conduct.mode is Mode.UNDERWAY
    assert h.call("get_navigation_state", {})["guidance_order"] == order


def test_a_starboard_offset_is_held_and_rejoin_returns_to_the_centreline():
    """keelson#324 Track.cross_track_offset_m on the kernel: she moves over and holds it, then
    rejoins the centreline."""
    h = Harbour()
    h.run_until(Mode.UNDERWAY, 2.0)
    h.run(60.0)
    r = h.order({"track": {"cross_track_offset_m": 25.0}})
    assert r["result"] == "COMMAND_RESULT_ACCEPTED", r["detail"]
    h.run(90.0)
    held = [row[4].xtd_m for row in h.history[-200:] if row[4] is not None]
    print(f"offset 25 m: xtd over the last 20 s {min(held):.1f} .. {max(held):.1f} m")
    assert all(20.0 <= x <= 30.0 for x in held), (min(held), max(held))
    back = h.order({"track": {}})
    assert back["result"] == "COMMAND_RESULT_ACCEPTED", back["detail"]
    h.run(90.0)
    rejoined = [abs(row[4].xtd_m) for row in h.history[-200:] if row[4] is not None]
    print(f"rejoined: worst |xtd| over the last 20 s {max(rejoined):.1f} m")
    assert max(rejoined) < 5.0


def test_the_operator_keeps_the_conn_and_she_holds_a_starboard_offset_on_the_order():
    """Ted, 2026-09-29: the operator keeps the conn; the navigator carries the order out. She
    sails the operator's track order STBD 25 m without the lease ever being the navigator's, and
    the operator's lease lapsing stops her (FALLBACK)."""
    h = Harbour(grant=False)
    assert h.run_until(Mode.UNDERWAY, 2.0) < 1.0
    assert "carrying out the order of roc@ROC-1" in h.p.conduct.reason
    h.run(60.0)
    r = h.order({"track": {"cross_track_offset_m": 25.0}})
    assert r["result"] == "COMMAND_RESULT_ACCEPTED", r["detail"]
    assert r["state"]["mode"] == "MODE_UNDERWAY" and not r["state"].get("conn_held", False)
    h.run(90.0)
    held = [row[4].xtd_m for row in h.history[-200:] if row[4] is not None]
    print(f"offset 25 m under the operator's conn: xtd {min(held):.1f} .. {max(held):.1f} m")
    assert all(20.0 <= x <= 30.0 for x in held), (min(held), max(held))
    assert h.p.lease.token is None and not h.p.conn_held
    assert h.p.lease_book.last.controller_id == "roc@ROC-1"
    assert h.history[-1][3] > 5.0, "not making way on the order"

    h.operator_heartbeat = False  # the operator's station goes quiet
    quiet = h.history[-1][0]
    h.run(45.0)
    after = [r for r in h.history if r[0] > quiet]
    lapsed = next(r[0] for r in after if r[1] is Mode.FALLBACK)
    print(f"FALLBACK {(lapsed - quiet) / S:.1f} s after the last heartbeat")
    assert "the conn holder's lease lapsed" in h.p.conduct.reason
    assert h.history[-1][3] == 0.0, "still making way after the operator's lease lapsed"
    assert h.p.conduct.mode is Mode.FALLBACK
    assert h.p.lease.token is None
    assert h.mapping == {}, "the operator's helm is still mapped to guidance"
