"""Conduct and guidance through the Processor: taking the conn, driving, and falling back."""

from __future__ import annotations

import pytest

from nav.bus.codec import Codec
from nav.conduct.state import Mode
from nav.guidance.axes import wanted_mapping
from nav.route_monitor import geodesy as g
from nav.runtime import Processor
from nav.world.latch import Sample
from tests.proto_draft import check_payload, parse
from tests.routes import square_route

pytestmark = pytest.mark.unit

S = 1_000_000_000
T0 = 1_700_000_000 * S
SRC = "navigation/0/guidance"


@pytest.fixture
def guidance_conf(conf):
    conf["modules"].update(route_monitor=True, guidance=True, decision_log=True)
    conf["inputs"] = {
        "location_fix": "gnss",
        "heading_true_north_deg": "sim",
        "course_over_ground_deg": "sim",
        "speed_over_ground_knots": "sim",
    }
    conf["conduct"].update(
        take_conn_when_free=True, fixed_authority_level="AUTHORITY_LEVEL_REMOTE_CONTROLLED"
    )
    conf["guidance"].update(full_ahead_sog_knots=20.0, neutral_burst_s=2.0)
    return conf


class Rig:
    def __init__(self, conf):
        tokens = iter(f"own-{i}" for i in range(100))
        ids = iter(range(10_000))
        self.p = Processor(
            conf, id_factory=lambda: f"id{next(ids)}", token_factory=lambda: next(tokens)
        )
        self.plan = square_route(radius=30.0)
        self.now = T0
        self.mapping: dict = {}
        wp0 = self.plan.waypoints[0]
        self.pos = g.destination(wp0.lat, wp0.lon, 0.0, 100.0)

    def tick(self, fresh_inputs=True):
        w = self.p.world
        w.put_declared("sim", self.plan, self.now, live=True)
        w.put_helm(self.mapping, self.now)
        if fresh_inputs:
            for subject, value in (
                ("location_fix", self.pos),
                ("heading_true_north_deg", 0.0),
                ("course_over_ground_deg", 0.0),
                ("speed_over_ground_knots", 8.0),
            ):
                w.inputs.put(subject, Sample(value, self.now))
        out = self.p.tick(self.now)
        if self.p.actuation.mapping == "install":
            self.mapping = wanted_mapping(SRC, 1.0)
        self.now += S // 10
        return out

    def run(self, seconds, **kw):
        out = []
        for _ in range(int(seconds * 10)):
            out += self.tick(**kw)
        return out


def subjects(outputs, subject):
    return [o for o in outputs if o.subject == subject]


def test_takes_the_free_conn_installs_the_mapping_and_drives(guidance_conf):
    rig = Rig(guidance_conf)
    out = rig.run(
        1.2
    )  # navigation_state is 1 Hz: the second one is the first after taking the conn
    assert rig.p.conduct.mode is Mode.UNDERWAY
    taken = subjects(out, "command_authority")
    assert taken and taken[0].source_id == "nav-test-vessel"
    assert taken[0].payload["controller_id"] == "navigation/0@nav-test-vessel"
    assert rig.mapping == wanted_mapping(SRC, 1.0)
    steering, throttle = rig.p.actuation.axes
    assert -100 <= steering <= 100 and throttle > 0
    assert rig.p.actuation.valid_until_ns > rig.now - S // 10
    state = subjects(out, "navigation_state")[-1].payload
    assert state["mode"] == "MODE_UNDERWAY" and state["conn_held"] is True
    assert state["authority_level_in_force"] == "AUTHORITY_LEVEL_REMOTE_CONTROLLED"


def test_setpoints_and_decisions_fit_their_drafts_and_ride_raw_json(guidance_conf):
    rig = Rig(guidance_conf)
    out = rig.run(2.0)
    setpoints = subjects(out, "navigation_setpoint")
    assert setpoints and setpoints[0].module == "guidance"
    draft = parse("NavigationSetpoint.proto", "NavigationSetpoint")
    for s in setpoints:
        assert check_payload(s.payload, draft) == []
    assert setpoints[-1].payload["purpose"] == "PURPOSE_TRACK"
    decisions = subjects(out, "navigation_decision")
    kinds = [d.payload["kind"] for d in decisions]
    assert "KIND_CONN_TAKEN" in kinds and "KIND_MODE_TRANSITION" in kinds
    draft = parse("NavigationDecision.proto", "NavigationDecision")
    for d in decisions:
        assert check_payload(d.payload, draft) == []
    codec = Codec("rider")
    assert codec.wire_subject("navigation_setpoint") == "raw_json"
    assert codec.wire_subject("navigation_decision") == "raw_json"


def test_pulling_the_lease_falls_back_bursts_neutral_then_goes_silent_and_does_not_retake(
    guidance_conf,
):
    rig = Rig(guidance_conf)
    rig.run(1.0)
    token = rig.p.lease.token
    rig.p.world.put_lease({"token": token, "released": True}, rig.now)
    out = rig.tick()
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert rig.p.actuation.axes == (0.0, 0.0)
    out += rig.run(1.5)
    assert rig.p.actuation.axes == (0.0, 0.0)
    out += rig.run(1.0)
    assert rig.p.actuation.axes is None
    out += rig.run(30.0)
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert not [o for o in subjects(out, "command_authority") if not o.payload["released"]]
    purposes = {s.payload["purpose"] for s in subjects(out, "navigation_setpoint")}
    assert purposes == {"PURPOSE_FAILSAFE_NEUTRAL"}


def test_an_operator_grant_resumes_after_fallback(guidance_conf):
    rig = Rig(guidance_conf)
    rig.run(1.0)
    rig.p.world.put_lease({"token": rig.p.lease.token, "released": True}, rig.now)
    rig.run(5.0)
    rig.p.world.put_lease(
        {
            "controller_id": "navigation/0@nav-test-vessel",
            "controller_site": "ROC-1",
            "token": "operator-grant",
            "lease_ttl_seconds": 30,
        },
        rig.now,
    )
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.UNDERWAY
    assert rig.p.lease.token == "operator-grant"
    assert rig.p.actuation.axes is not None


def test_without_a_driving_authority_it_never_takes_the_conn(guidance_conf):
    guidance_conf["conduct"]["fixed_authority_level"] = None
    rig = Rig(guidance_conf)
    out = rig.run(3.0)
    assert rig.p.conduct.mode is Mode.READY
    assert "no operational_authority heard" in rig.p.conduct.reason
    assert subjects(out, "command_authority") == [] and rig.p.actuation.axes is None
    assert rig.mapping == {}


def test_another_controllers_mapping_is_never_overwritten(guidance_conf):
    rig = Rig(guidance_conf)
    rig.mapping = {
        "axes": {"steering": {"subject": "wheel_position_pct", "source_id": "autopilot"}}
    }
    out = rig.run(3.0)
    assert rig.p.conduct.mode is Mode.READY
    assert "another controller" in rig.p.conduct.reason
    assert rig.p.actuation.mapping is None and rig.p.actuation.axes is None
    assert subjects(out, "command_authority") == []


def test_stale_ownship_while_underway_falls_back(guidance_conf):
    rig = Rig(guidance_conf)
    rig.run(1.0)
    rig.run(4.0, fresh_inputs=False)
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert "location_fix" in rig.p.conduct.reason


def test_arrival_releases_the_conn_and_the_mapping(guidance_conf):
    rig = Rig(guidance_conf)
    rig.run(1.0)
    last = rig.plan.waypoints[-1]
    rig.pos = g.destination(last.lat, last.lon, 0.0, 10.0)
    out = rig.run(5.0)
    assert rig.p.conduct.mode is Mode.ENDED
    released = [o for o in subjects(out, "command_authority") if o.payload["released"]]
    assert len(released) == 1
    assert rig.mapping == wanted_mapping(SRC, 1.0)  # the rig does not act on release
    kinds = [d.payload["kind"] for d in subjects(out, "navigation_decision")]
    assert "KIND_CONN_RELEASED" in kinds


def test_guidance_publications_are_declared(guidance_conf):
    pubs = Processor(guidance_conf).publications()
    assert pubs["guidance"] == ("joystick_x_pct", "joystick_y_pct", "navigation_setpoint")
    assert pubs["decision_log"] == ("navigation_decision",)
