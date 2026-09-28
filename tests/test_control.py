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
from nav.guidance.track import KNOT_MPS
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
        self.sog = 8.0
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
            ("speed_over_ground_knots", self.sog),
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
        {"go_to": {"position": {"latitude_deg": 57.7, "longitude_deg": 11.9}}},
        {
            "go_to": {
                "position": {"latitude_deg": -33.9, "longitude_deg": 151.2},
                "arrival_radius_m": 25.0,
            },
            "speed_knots": 5.0,
        },
        {"dynamic_positioning": {"position": {"latitude_deg": 57.7, "longitude_deg": 11.9}}},
        {
            "dynamic_positioning": {
                "position": {"latitude_deg": 57.7, "longitude_deg": 11.9},
                "heading_deg": 90.0,
                "walk_speed_knots": 1.0,
            }
        },
        {
            "loiter": {
                "position": {"latitude_deg": 57.7, "longitude_deg": 11.9},
                "radius_m": 150.0,
            }
        },
        {
            "loiter": {
                "position": {"latitude_deg": 57.7, "longitude_deg": 11.9},
                "radius_m": 150.0,
                "return_speed_knots": 2.5,
            },
            "speed_knots": 6.0,
        },
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
        {"go_to": {}},
        {"go_to": {"position": {"latitude_deg": 91.0, "longitude_deg": 0.0}}},
        {"go_to": {"position": {"latitude_deg": 0.0, "longitude_deg": 181.0}}},
        {
            "go_to": {
                "position": {"latitude_deg": 0.0, "longitude_deg": 0.0},
                "arrival_radius_m": -1.0,
            }
        },
        {"go_to": {"position": {"latitude_deg": 0.0, "longitude_deg": 0.0}}, "hold": {}},
        {"loiter": {"position": {"latitude_deg": 0.0, "longitude_deg": 0.0}}},  # no radius
        {"loiter": {"position": {"latitude_deg": 0.0, "longitude_deg": 0.0}, "radius_m": 0.0}},
        {
            "loiter": {
                "position": {"latitude_deg": 0.0, "longitude_deg": 0.0},
                "radius_m": 100.0,
                "return_speed_knots": 0.0,
            }
        },
        {"loiter": {"radius_m": 100.0}},
        {
            "dynamic_positioning": {
                "position": {"latitude_deg": 0.0, "longitude_deg": 0.0},
                "heading_deg": 360.0,
            }
        },
        {
            "dynamic_positioning": {
                "position": {"latitude_deg": 0.0, "longitude_deg": 0.0},
                "walk_speed_knots": -1.0,
            }
        },
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


def test_a_permitted_controller_may_load_a_route_while_nobody_holds_the_conn(control_conf):
    """Slipway's tick-0 handover: nobody holds the conn, the scenario loads the route, and a
    processor with take_conn_when_free takes the free conn and sails it. Slipway never grants."""
    control_conf["control"]["controllers"] = ["slipway"]
    control_conf["conduct"]["take_conn_when_free"] = True
    rig = Rig(control_conf)
    p = rig.call("load_route", load(caller=caller("slipway"), initial_order={"heading_deg": 45.0}))
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"


def test_an_operator_holding_the_conn_outranks_the_permitted_list(control_conf):
    control_conf["control"]["controllers"] = ["slipway"]
    rig = Rig(control_conf)
    rig.lease("roc@ROC-1", "T")
    p = rig.call("load_route", load(caller=caller("slipway")))
    assert p.response["result"] == "COMMAND_RESULT_DENIED"


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


def _go_to(lat, lon, radius=None) -> dict:
    go_to: dict = {"position": {"latitude_deg": lat, "longitude_deg": lon}}
    if radius is not None:
        go_to["arrival_radius_m"] = radius
    return {"go_to": go_to}


def test_a_go_to_steers_for_the_position_then_holds_and_makes_back_for_it(control_conf):
    rig = _underway(control_conf)
    target = g.destination(*rig.pos, 90.0, 500.0)
    order = _go_to(*target, radius=20.0)
    p = rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": order})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"
    setpoint = rig.published("navigation_setpoint")[-1].payload
    assert setpoint["purpose"] == "PURPOSE_STEERING"
    assert abs(setpoint["steering"]["course_over_ground_deg"] - 90.0) < 0.5
    assert setpoint["actuation"]["steering_pct"] > 0  # heading 000 -> 090: starboard
    assert setpoint["propulsion"]["sog_mps"] > 0
    assert abs(rig.published("heading_setpoint_deg")[-1].payload["value"] - 90.0) < 0.5
    state = rig.published("navigation_state")[-1].payload
    assert state["guidance_order"] == order
    assert check_payload(state, parse("NavigationState.proto", "NavigationState")) == []

    for distance, sub_mode in ((10.0, "HOLDING"), (35.0, "HOLDING"), (45.0, "STEERING")):
        rig.pos = g.destination(*target, 270.0, distance)
        rig.run(0.5)
        assert rig.p.conduct.sub_mode.value == f"SUB_MODE_{sub_mode}", distance
        if sub_mode == "HOLDING":
            assert rig.p.actuation.axes == (0.0, 0.0)
            assert rig.published("navigation_setpoint")[-1].payload["purpose"] == (
                "PURPOSE_HOLDING"
            )
    rig.run(1.0)
    # Arrival ends the passage, never the order: go_to is still the one in force.
    assert rig.published("navigation_state")[-1].payload["guidance_order"] == order
    assert rig.p.conduct.mode is Mode.UNDERWAY


def test_a_go_to_without_a_radius_arrives_at_the_configured_one(control_conf):
    control_conf["guidance"]["goto_arrival_radius_m"] = 60.0
    rig = _underway(control_conf)
    target = g.destination(*rig.pos, 90.0, 500.0)
    rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": _go_to(*target)})
    rig.pos = g.destination(*target, 270.0, 50.0)
    rig.run(0.5)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_HOLDING"


def test_a_go_to_keeps_her_speed_but_makes_way_from_a_standstill(control_conf):
    rig = _underway(control_conf)
    target = g.destination(*rig.pos, 90.0, 500.0)
    rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": _go_to(*target)})
    rig.run(1.0)
    kn = rig.published("navigation_setpoint")[-1].payload["propulsion"]["sog_mps"] / KNOT_MPS
    assert kn == pytest.approx(8.0)  # the speed she has (keelson#308)
    rig.sog = 0.0
    rig.run(1.0)
    kn = rig.published("navigation_setpoint")[-1].payload["propulsion"]["sog_mps"] / KNOT_MPS
    assert kn == pytest.approx(control_conf["guidance"]["default_sog_knots"])
    assert rig.p.actuation.axes[1] > 0


def test_setpoint_subjects_name_the_frame_the_helm_steers_by(control_conf):
    rig = _underway(control_conf)
    rig.run(1.0)
    assert rig.published("course_over_ground_setpoint_deg")  # on the track: a course
    assert rig.published("heading_setpoint_deg")
    token = {"caller": caller(token="grant-1")}
    rig.call("set_guidance_order", {**token, "order": {"heading_deg": 45.0}})
    rig.out.clear()
    rig.run(1.0)
    assert rig.published("heading_setpoint_deg")[-1].payload["value"] == 45.0
    assert not rig.published("course_over_ground_setpoint_deg")  # a heading has no course
    rig.call("set_guidance_order", {**token, "order": {"hold": {}}})
    rig.out.clear()
    rig.run(1.0)
    assert not rig.published("heading_setpoint_deg")  # absent when nobody holds one
    assert not rig.published("course_over_ground_setpoint_deg")


def _loiter(lat, lon, radius, back=None) -> dict:
    loiter: dict = {"position": {"latitude_deg": lat, "longitude_deg": lon}, "radius_m": radius}
    if back is not None:
        loiter["return_speed_knots"] = back
    return {"loiter": loiter}


def test_dynamic_positioning_is_refused_not_approximated(control_conf):
    rig = _underway(control_conf)
    dp = {
        "dynamic_positioning": {
            "position": {"latitude_deg": rig.pos[0], "longitude_deg": rig.pos[1]},
            "heading_deg": 90.0,
        }
    }
    p = rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": dp})
    assert p.response["result"] == "COMMAND_RESULT_UNSUPPORTED"
    assert "lateral authority" in p.response["detail"]
    assert rig.p.order.kind is Kind.TRACK  # nothing taken up in its place


def test_a_loiter_makes_for_the_point_drifts_and_makes_back_past_the_radius(control_conf):
    rig = _underway(control_conf)
    centre = g.destination(*rig.pos, 90.0, 500.0)
    order = _loiter(*centre, 150.0, back=3.0)
    p = rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": order})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"  # on passage
    assert rig.published("navigation_state")[-1].payload["guidance_order"] == order

    # Near the point she goes neutral and drifts; drifting inside the radius stays neutral.
    for distance, sub_mode in ((10.0, "HOLDING"), (120.0, "HOLDING")):
        rig.pos = g.destination(*centre, 270.0, distance)
        rig.run(0.5)
        assert rig.p.conduct.sub_mode.value == f"SUB_MODE_{sub_mode}", distance
        assert rig.p.actuation.axes == (0.0, 0.0)
    rig.out.clear()
    rig.run(1.0)
    assert not rig.published("heading_setpoint_deg")  # no setpoint while drifting

    # Carried past the radius: back at the return speed, until near the point again.
    rig.pos = g.destination(*centre, 270.0, 160.0)
    rig.run(1.0)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"
    setpoint = rig.published("navigation_setpoint")[-1].payload
    assert setpoint["propulsion"]["sog_mps"] / KNOT_MPS == pytest.approx(3.0)
    assert abs(setpoint["steering"]["course_over_ground_deg"] - 90.0) < 0.5
    rig.pos = g.destination(*centre, 270.0, 60.0)  # coming back, not yet near: still steering
    rig.run(0.5)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_STEERING"
    rig.pos = g.destination(*centre, 270.0, 10.0)
    rig.run(0.5)
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_HOLDING"


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
    r = rig.call("set_guidance_order", {"caller": caller(token="T"), "order": _go_to(57.7, 11.9)})
    assert r.response["result"] == "COMMAND_RESULT_UNSUPPORTED"
    lo = rig.call(
        "set_guidance_order", {"caller": caller(token="T"), "order": _loiter(57.7, 11.9, 100.0)}
    )
    assert lo.response["result"] == "COMMAND_RESULT_UNSUPPORTED"
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


# -- settings (keelson#322) --------------------------------------------------------------------


def _settings(rig, settings: dict, token="grant-1"):
    return rig.call("set_settings", {"caller": caller(token=token), "settings": settings})


def test_get_settings_says_what_it_has_and_what_is_fixed(control_conf):
    rig = Rig(control_conf)
    got = rig.call("get_settings", {}).response
    s = got["settings"]
    assert s["default_speed_knots"] == control_conf["guidance"]["default_sog_knots"]
    assert s["go_to_arrival_radius_m"] == control_conf["guidance"]["goto_arrival_radius_m"]
    assert s["loiter_return_speed_knots"] == control_conf["guidance"]["loiter_return_knots"]
    assert s["route_alarms"] is True
    assert "traffic_range_m" not in s  # no traffic module here: not a setting it has
    # The advisor was not started, so it (and auto-accept) cannot change while it runs.
    assert s["colreg_advisor_active"] is False
    assert set(got["fixed"]) == {"colreg_advisor_active", "auto_accept_advice"}
    _encode("get_settings", got)


def test_set_settings_needs_the_conn(control_conf):
    rig = _underway(control_conf)
    p = _settings(rig, {"default_speed_knots": 9.0}, token="not-the-holder")
    assert p.response["result"] == "COMMAND_RESULT_DENIED"
    assert rig.p.guidance.speed.default_sog_knots == control_conf["guidance"]["default_sog_knots"]


def test_set_settings_applies_the_named_settings_and_restates_them(control_conf):
    rig = _underway(control_conf)
    p = _settings(rig, {"default_speed_knots": 9.0, "go_to_arrival_radius_m": 35.0})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    assert p.response["settings"]["default_speed_knots"] == 9.0
    assert p.response["settings"]["go_to_arrival_radius_m"] == 35.0
    assert rig.p.guidance.goto_radius_default_m == 35.0
    _encode("set_settings", p.response)
    # The new default speed is what a go_to from a standstill makes way at.
    rig.sog = 0.0
    target = g.destination(*rig.pos, 90.0, 500.0)
    rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": _go_to(*target)})
    rig.run(1.0)
    kn = rig.published("navigation_setpoint")[-1].payload["propulsion"]["sog_mps"] / KNOT_MPS
    assert kn == pytest.approx(9.0)
    kinds = [d.payload["kind"] for d in rig.published("navigation_decision")]
    assert "KIND_GUIDANCE_ORDER" in kinds


@pytest.mark.parametrize(
    "settings, result",
    [
        ({"colreg_advisor_active": True, "default_speed_knots": 9.0}, "COMMAND_RESULT_UNSUPPORTED"),
        ({"traffic_range_m": 5000.0}, "COMMAND_RESULT_UNSUPPORTED"),  # no traffic module
    ],
)
def test_a_fixed_or_absent_setting_refuses_the_whole_request(control_conf, settings, result):
    rig = _underway(control_conf)
    p = _settings(rig, settings)
    assert p.response["result"] == result
    # Whole or not at all: the valid half was not applied either.
    assert rig.p.guidance.speed.default_sog_knots == control_conf["guidance"]["default_sog_knots"]


def test_a_value_out_of_range_is_an_error_and_changes_nothing(control_conf):
    rig = _underway(control_conf)
    p = _settings(rig, {"default_speed_knots": 9.0, "go_to_arrival_radius_m": -1.0})
    assert p.response is None and "go_to_arrival_radius_m" in p.error
    assert rig.p.guidance.speed.default_sog_knots == control_conf["guidance"]["default_sog_knots"]


def test_every_procedure_is_decoded_and_encoded_on_the_bus():
    """The bus handler maps each procedure to its request and response types. Missing a mapping
    answered 'undecodable request' live (get_settings, 2026-09-28) while every Pending-level test
    passed."""
    from keelson.interfaces import NavigationControl_pb2 as nc

    from nav.bus import control as bus
    from nav.control.commands import PROCEDURES

    for p in PROCEDURES:
        assert hasattr(nc, bus._REQUESTS[p]), p
        if p != "get_navigation_state":
            assert hasattr(nc, bus._RESPONSES[p]), p


# -- track offset, reverse, speed plan factor (keelson#324) --------------------------------------


def test_reverse_sails_the_route_back_and_the_monitor_follows(control_conf):
    rig = _underway(control_conf)
    forward_to = rig.p.guidance.state.to_waypoint_id
    p = rig.call(
        "set_guidance_order",
        {"caller": caller(token="grant-1"), "order": {"track": {"reverse": True}}},
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    rev = rig.p.guidance.state
    assert rev.to_waypoint_id != forward_to
    # On the first leg reversed, she makes for WP0 -- the route's first mark.
    assert rev.to_waypoint_id == rig.plan.waypoints[0].id
    assert rig.p.conduct.sub_mode.value == "SUB_MODE_TRACK"
    state = rig.published("navigation_state")[-1].payload
    assert state["guidance_order"] == {"track": {"reverse": True}}
    assert state["route_ref"]["route_edition_number"] == rig.plan.edition  # the same edition
    rig.call("set_guidance_order", {"caller": caller(token="grant-1"), "order": {"track": {}}})
    rig.run(1.0)
    assert rig.p.guidance.state.to_waypoint_id == forward_to


def test_a_speed_plan_factor_trims_the_planned_speed(control_conf):
    rig = _underway(control_conf)
    rig.run(1.0)
    planned = rig.published("navigation_setpoint")[-1].payload["propulsion"]["sog_mps"]
    rig.call(
        "set_guidance_order",
        {"caller": caller(token="grant-1"), "order": {"track": {}, "speed_plan_factor": 1.2}},
    )
    rig.run(1.0)
    trimmed = rig.published("navigation_setpoint")[-1].payload
    assert trimmed["propulsion"]["sog_mps"] == pytest.approx(planned * 1.2)
    assert "+20 %" in trimmed["reason"]


def test_an_offset_steers_for_the_parallel_line(control_conf):
    rig = _underway(control_conf)
    rig.run(1.0)
    on_track = rig.published("navigation_setpoint")[-1].payload["steering"][
        "course_over_ground_deg"
    ]
    rig.call(
        "set_guidance_order",
        {"caller": caller(token="grant-1"), "order": {"track": {"cross_track_offset_m": 40.0}}},
    )
    rig.run(1.0)
    offset = rig.published("navigation_setpoint")[-1].payload["steering"]["course_over_ground_deg"]
    # A starboard offset from the centreline: she comes right of the leg's course.
    assert 0.0 < ((offset - on_track + 540.0) % 360.0 - 180.0) < 90.0
