"""navigation_control/v1 decided in the tick: who may call, load_route, guidance orders, voyages.

Driven through ``Processor.tick`` with ``Pending`` calls on the World, as bus/control.py queues
them, so the same decisions are tested that the RPC layer waits on.
"""

from __future__ import annotations

import pytest
from google.protobuf import json_format

from nav.bus import translate
from nav.bus.codec import Codec
from nav.bus.control import _encode
from nav.conduct.state import Mode
from nav.control.commands import Pending
from nav.control.orders import GuidanceOrder, Kind, OrderError
from nav.guidance.axes import wanted_mapping
from nav.route_monitor import geodesy as g
from nav.runtime import Processor
from nav.world.latch import Sample
from tests.proto_draft import check_payload, parse
from tests.routes import square_route
from tests.test_e2e_route import route_proto

pytestmark = pytest.mark.unit

S = 1_000_000_000
T0 = 1_700_000_000 * S
ME = "navigation/0@nav-test-vessel"
SRC = "navigation/0/guidance"


@pytest.fixture
def control_conf(conf):
    conf["modules"].update(route_monitor=True, guidance=True, decision_log=True, control=True)
    conf["inputs"] = {
        "location_fix": "gnss",
        "heading_true_north_deg": "sim",
        "course_over_ground_deg": "sim",
        "speed_over_ground_knots": "sim",
    }
    conf["conduct"]["fixed_authority_level"] = "AUTHORITY_LEVEL_REMOTE_CONTROLLED"
    conf["guidance"].update(full_ahead_sog_knots=20.0)
    return conf


@pytest.fixture
def monitor_only(control_conf):
    control_conf["modules"]["guidance"] = False
    return control_conf


class Rig:
    def __init__(self, conf):
        ids = iter(range(10_000))
        tokens = iter(f"own-{i}" for i in range(100))
        self.p = Processor(
            conf, id_factory=lambda: f"id{next(ids)}", token_factory=lambda: next(tokens)
        )
        self.plan = square_route()
        self.p.world.put_edition(self.plan)
        self.now = T0
        self.mapping: dict = {}
        wp0 = self.plan.waypoints[0]
        self.pos = g.destination(wp0.lat, wp0.lon, 0.0, 100.0)
        self.out: list = []

    def lease(self, controller: str, token: str) -> None:
        self.p.world.put_lease(
            {"controller_id": controller, "token": token, "lease_ttl_seconds": 30}, self.now
        )

    def call(self, procedure: str, request: dict, fetched=None) -> Pending:
        pending = Pending(procedure, request, self.now, fetched=fetched)
        self.p.world.put_command(pending)
        self.tick()
        assert pending.done.is_set()
        return pending

    def tick(self):
        w = self.p.world
        w.put_helm(self.mapping, self.now)
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
        self.out += out
        return out

    def run(self, seconds: float):
        for _ in range(int(seconds * 10)):
            self.tick()

    def published(self, subject: str):
        return [o for o in self.out if o.subject == subject]


def caller(controller="roc@ROC-1", token=None) -> dict:
    c = {"controller_id": controller}
    if token is not None:
        c["authority_token"] = token
    return c


def load(route_id="r1", edition=1, **kw) -> dict:
    return {"route_ref": {"route_id": route_id, "route_edition_number": edition}, **kw}


# -- orders ------------------------------------------------------------------------------------


@pytest.mark.parametrize(
    "payload",
    [
        {"track": {}},
        {"track": {"resume_from_waypoint_id": "wp2"}, "speed_knots": 6.0},
        {"course_over_ground_deg": 90.0},
        {"heading_deg": 359.5, "speed_knots": 4.0},
        {"hold": {}},
    ],
)
def test_orders_round_trip_through_keelson_guidance_order(payload):
    from keelson.payloads.NavigationState_pb2 import GuidanceOrder as Proto

    order = GuidanceOrder.from_payload(payload)
    assert order.to_payload() == payload
    msg = json_format.ParseDict(order.to_payload(), Proto())
    back = json_format.MessageToDict(msg, preserving_proto_field_name=True)
    assert GuidanceOrder.from_payload(back) == order


@pytest.mark.parametrize(
    "payload",
    [
        {},
        {"course_over_ground_deg": 90.0, "heading_deg": 90.0},
        {"heading_deg": 360.0},
        {"course_over_ground_deg": -1.0},
        {"track": {}, "speed_knots": -2.0},
    ],
)
def test_an_order_is_one_frame_with_a_usable_value(payload):
    with pytest.raises(OrderError):
        GuidanceOrder.from_payload(payload)


# -- who may call ------------------------------------------------------------------------------


def test_nobody_may_direct_it_while_nobody_holds_the_conn(control_conf):
    rig = Rig(control_conf)
    p = rig.call("load_route", load(caller=caller()))
    assert p.response["result"] == "COMMAND_RESULT_DENIED"
    assert "nobody holds the conn" in p.response["detail"]
    assert not rig.published("voyage")


def test_a_caller_without_the_holders_token_is_denied(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    p = rig.call("load_route", load(caller=caller("other@ROC-2", token="guess")))
    assert p.response["result"] == "COMMAND_RESULT_DENIED"


def test_a_permitted_controller_may_direct_it_while_it_holds_the_conn(control_conf):
    """With no route the process hands a granted conn straight back, so a route is loaded by the
    conn holder first and the conn granted after. From then on a permitted controller (a scenario
    runner) may direct it without a token; anyone else still needs the conn."""
    control_conf["control"]["controllers"] = ["slipway"]
    rig = _underway(control_conf)
    p = rig.call("set_guidance_order", {"caller": caller("slipway"), "order": {"hold": {}}})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    q = rig.call("set_guidance_order", {"caller": caller("stranger"), "order": {"track": {}}})
    assert q.response["result"] == "COMMAND_RESULT_DENIED"


# -- load_route --------------------------------------------------------------------------------


def test_load_route_activates_a_voyage_and_the_monitor_follows_it(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    p = rig.call("load_route", load(caller=caller(token="T")))
    r = p.response
    assert r["result"] == "COMMAND_RESULT_ACCEPTED"
    assert r["voyage_id"] == "voy-id0"
    assert r["state"]["voyage_id"] == "voy-id0"
    assert r["state"]["route_ref"] == {"route_id": "r1", "route_edition_number": 1}
    assert r["state"]["guidance_order"] == {"track": {}}
    assert rig.p.selection.voyage.voyage_id == "voy-id0"

    (voyage,) = rig.published("voyage")
    assert voyage.source_id == "voy-id0"
    doc = voyage.payload
    assert doc["status"] == "VOYAGE_STATUS_ACTIVE"
    assert doc["audit_trail"][-1]["action"] == "VOYAGE_AUDIT_ACTION_ACTIVATED"
    assert doc["audit_trail"][-1]["actor_id"] == "roc@ROC-1"
    # What goes on the wire is keelson's Voyage, and the reply is keelson's response message.
    Codec("rider").encode("voyage", doc)
    _encode("load_route", r)

    rig.run(1.2)
    assert any(o.source_id == "voy-id0" for o in rig.published("route_execution"))


def test_an_edition_that_is_not_there_is_refused_never_replaced_by_latest(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    p = rig.call("load_route", load(edition=7, caller=caller(token="T")))
    assert p.response["result"] == "COMMAND_RESULT_FAILED"
    assert "edition 7" in p.response["detail"]
    assert not rig.published("voyage")


def test_a_route_from_another_vessel_is_copied_into_this_tree_first(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    other = square_route(route_id="r9", edition=3)
    msg = route_proto(other)
    fetched = (
        json_format.MessageToDict(msg, preserving_proto_field_name=True),
        translate.route_plan(msg),
    )
    p = rig.call(
        "load_route",
        load("r9", 3, source_entity_id="sf18", caller=caller(token="T")),
        fetched=fetched,
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    assert "copied from sf18" in p.response["detail"]
    tails = sorted(o.source_id for o in rig.published("route"))
    assert tails == ["r9/edition/3", "r9/latest"]
    for o in rig.published("route"):
        Codec("rider").encode("route", o.payload)
    assert ("r9", 3) in rig.p.world.snapshot().editions
    assert rig.p.selection.plan.route_id == "r9"


def test_a_missing_foreign_edition_is_refused(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    pending = Pending(
        "load_route", load("r9", 3, source_entity_id="sf18", caller=caller(token="T")), 0
    )
    pending.fetch_error = "no stored copy"
    rig.p.world.put_command(pending)
    rig.tick()
    assert pending.response["result"] == "COMMAND_RESULT_FAILED"
    assert "no stored copy" in pending.response["detail"]


def test_a_new_voyage_supersedes_the_active_one_which_is_aborted(control_conf):
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    rig.call("load_route", load(caller=caller(token="T")))
    rig.p.world.put_edition(square_route(route_id="r2"))
    p = rig.call("load_route", load("r2", caller=caller(token="T")))
    second = p.response["voyage_id"]
    assert second.startswith("voy-") and second != "voy-id0"
    status = {o.source_id: o.payload["status"] for o in rig.published("voyage")}
    assert status == {"voy-id0": "VOYAGE_STATUS_ABORTED", second: "VOYAGE_STATUS_ACTIVE"}
    assert rig.p.selection.voyage.voyage_id == second


def test_a_republished_voyage_keeps_the_fields_it_does_not_read(control_conf):
    from nav.world.route import RouteRef, VoyagePlan

    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    doc = {
        "voyage_id": "v-roc",
        "vessel_name": "SLIPWAY ALPHA",
        "arrival_port_name": "Gothenburg",
        "status": "VOYAGE_STATUS_PLANNED",
        "route_ref": {"route_id": "r1", "route_edition_number": 1},
    }
    rig.p.world.put_voyage(VoyagePlan("v-roc", "VOYAGE_STATUS_PLANNED", RouteRef("r1", 1)), doc)
    p = rig.call(
        "set_voyage_status",
        {"caller": caller(token="T"), "voyage_id": "v-roc", "status": "VOYAGE_STATUS_ACTIVE"},
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED"
    (published,) = rig.published("voyage")
    assert published.payload["vessel_name"] == "SLIPWAY ALPHA"
    assert published.payload["arrival_port_name"] == "Gothenburg"
    assert published.payload["audit_trail"][-1]["action"] == "VOYAGE_AUDIT_ACTION_ACTIVATED"


# -- guidance orders ---------------------------------------------------------------------------


def _underway(conf) -> Rig:
    """The scenario flow: the ROC holds the conn, loads the route, then grants the conn."""
    rig = Rig(conf)
    rig.lease("roc@ROC-1", "T")
    rig.call("load_route", load(caller=caller(token="T")))
    rig.lease(ME, "grant-1")
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    return rig


def test_a_heading_order_steers_by_heading_and_reports_steering(control_conf):
    rig = _underway(control_conf)
    p = rig.call(
        "set_guidance_order",
        {"caller": caller(token="grant-1"), "order": {"heading_deg": 90.0, "speed_knots": 5.0}},
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"
    setpoint = rig.published("navigation_setpoint")[-1].payload
    assert setpoint["purpose"] == "PURPOSE_STEERING"
    assert setpoint["steering"] == {"heading_deg": 90.0}
    assert setpoint["actuation"]["steering_pct"] > 0  # heading 000 -> 090 is a turn to starboard
    assert check_payload(setpoint, parse("NavigationSetpoint.proto", "NavigationSetpoint")) == []
    state = rig.published("navigation_state")[-1].payload
    assert state["guidance_order"] == {"heading_deg": 90.0, "speed_knots": 5.0}
    assert check_payload(state, parse("NavigationState.proto", "NavigationState")) == []


def test_hold_drives_neutral_and_track_rejoins(control_conf):
    rig = _underway(control_conf)
    token = {"caller": caller(token="grant-1")}
    rig.call("set_guidance_order", {**token, "order": {"hold": {}}})
    rig.run(0.5)
    assert rig.p.actuation.axes == (0.0, 0.0)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_HOLDING"
    assert rig.published("navigation_setpoint")[-1].payload["purpose"] == "PURPOSE_HOLDING"
    rig.call(
        "set_guidance_order", {**token, "order": {"track": {"resume_from_waypoint_id": "wp2"}}}
    )
    rig.run(0.5)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_TRACK"
    assert rig.p.guidance.state.to_waypoint_id == "wp2"


def test_a_waypoint_not_on_the_route_is_refused(control_conf):
    rig = _underway(control_conf)
    p = rig.call(
        "set_guidance_order",
        {"caller": caller(token="grant-1"), "order": {"track": {"resume_from_waypoint_id": "x"}}},
    )
    assert p.response["result"] == "COMMAND_RESULT_FAILED"


def test_a_malformed_order_is_an_error_not_a_result(control_conf):
    rig = _underway(control_conf)
    p = rig.call(
        "set_guidance_order", {"caller": caller(token="grant-1"), "order": {"heading_deg": 400.0}}
    )
    assert p.response is None and "[0, 360)" in p.error


def test_without_guidance_only_track_is_held(monitor_only):
    rig = Rig(monitor_only)
    rig.lease("roc@ROC-1", "T")
    rig.call("load_route", load(caller=caller(token="T")))
    p = rig.call(
        "set_guidance_order", {"caller": caller(token="T"), "order": {"heading_deg": 90.0}}
    )
    assert p.response["result"] == "COMMAND_RESULT_UNSUPPORTED"
    q = rig.call("set_guidance_order", {"caller": caller(token="T"), "order": {"track": {}}})
    assert q.response["result"] == "COMMAND_RESULT_ACCEPTED"


def test_an_order_lapses_with_its_voyage(control_conf):
    rig = _underway(control_conf)
    rig.call(
        "set_guidance_order", {"caller": caller(token="grant-1"), "order": {"heading_deg": 45.0}}
    )
    assert rig.p.order.kind is Kind.HEADING
    p = rig.call(
        "set_voyage_status",
        {
            "caller": caller(token="grant-1"),
            "voyage_id": "voy-id0",
            "status": "VOYAGE_STATUS_PAUSED",
        },
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED"
    assert rig.p.order.kind is Kind.TRACK
    assert rig.p.selection.plan is None and "PAUSED" in rig.p.selection.reason
    (paused,) = [
        o for o in rig.published("voyage") if o.payload["status"] == "VOYAGE_STATUS_PAUSED"
    ]
    assert paused.payload["audit_trail"][-1]["action"] == "VOYAGE_AUDIT_ACTION_PAUSED"


def test_get_navigation_state_needs_no_authority_and_is_keelsons_message(control_conf):
    rig = Rig(control_conf)
    p = rig.call("get_navigation_state", {})
    assert p.response["mode"] == "MODE_IDLE"
    _encode("get_navigation_state", p.response)
