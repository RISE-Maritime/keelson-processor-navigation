"""Phase 4 acceptance: guidance sails Slipway's slipway-alpha in COMMANDED mode, in-process.

Slipway's kernel runs two-vessel-harbor with alpha handed to vehicle_control; the processor gets
her operational-plane values each tick and its axes are posted to the kernel's command mailbox,
where the kernel's own dead-man judges them. Nothing touches a zenoh session.

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
from nav.guidance.axes import wanted_mapping  # noqa: E402
from nav.runtime import Processor  # noqa: E402
from nav.world.latch import Sample  # noqa: E402
from nav.world.route import RoutePlan, WaypointPlan  # noqa: E402

SLIPWAY = Path(__file__).resolve().parents[2] / "SLIPWAY"
SCENARIO = SLIPWAY / "scenarios" / "two-vessel-harbor.yaml"
ALPHA = "slipway-alpha"
EPOCH = 1_700_000_000 * 10**9
S = 10**9
FLOATS = (
    "heading_true_north_deg",
    "course_over_ground_deg",
    "speed_over_ground_knots",
    "yaw_rate_degps",
)


def nav_conf() -> dict:
    c = copy.deepcopy(config_mod.DEFAULTS)
    c["entity_id"] = ALPHA
    c["modules"].update(route_monitor=True, guidance=True, decision_log=True)
    c["inputs"] = {"location_fix": "gnss", **{s: "sim" for s in FLOATS}}
    c["conduct"].update(
        take_conn_when_free=True, fixed_authority_level="AUTHORITY_LEVEL_REMOTE_CONTROLLED"
    )
    c["guidance"].update(full_ahead_sog_knots=20.5)
    config_mod._validate(c)
    return c


class Harbour:
    def __init__(self):
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
        self.history: list[tuple] = []
        self.posted = 0

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
        if processor_alive:
            w = self.p.world
            w.inputs.put(
                "location_fix",
                Sample(self.scenario.plane.wgs84_from_enu(vessel.east_m, vessel.north_m), now),
            )
            for subject in FLOATS:
                w.inputs.put(subject, Sample(values[subject], now))
            w.put_declared("sim", self.plan, now, live=True)
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
            (now, self.p.conduct.mode, vessel, values["speed_over_ground_knots"], progress)
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
    limit = h.p.conf["guidance"]["neutral_burst_s"] + h.cfg.actuators.max_axis_age_s
    print(f"stopped {(stopped - pulled) / S:.1f} s after the pull (limit {limit:.1f} s)")
    assert (stopped - pulled) / S <= limit
    assert all(r[3] == 0.0 for r in after if r[0] >= stopped)
    assert all(r[1] is Mode.FALLBACK for r in after)
    assert h.p.lease.token is None


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
