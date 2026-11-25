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
        "heading_true_north_deg": "slipway",
        "course_over_ground_deg": "slipway",
        "speed_over_ground_knots": "slipway",
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
        elif self.p.actuation.mapping == "release" and self.mapping == wanted_mapping(SRC, 1.0):
            self.mapping = {}
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


# -- the operator keeps the conn; the process carries out the order (navigation_conduct) ------

OPERATOR = "ted@roc1"


def _directed(conf, order=None) -> Rig:
    """An operator holds the conn and loads the route with its token. No grant: it stays theirs."""
    rig = Rig(conf)
    rig.lease(OPERATOR, "op-1")
    extra = {"initial_order": order} if order is not None else {}
    p = rig.call("load_route", load(caller=caller(OPERATOR, token="op-1"), **extra))
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    return rig


def test_an_order_from_the_conn_holder_is_carried_out_under_the_holders_conn(control_conf):
    rig = _directed(control_conf, {"heading_deg": 90.0})
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    assert "carrying out the order of ted@roc1" in rig.p.conduct.reason
    assert rig.p.lease.token is None and not rig.p.conn_held
    assert not rig.published("command_authority"), "the conn is the operator's, never taken"
    state = rig.published("navigation_state")[-1].payload
    assert state["mode"] == "MODE_UNDERWAY" and state["conn_held"] is False
    assert rig.p.actuation.axes is not None and rig.p.actuation.axes[0] > 0  # turning to 090


def test_a_voyage_the_holder_did_not_order_does_not_drive(control_conf):
    """A voyage selected (here published by a station, not loaded by the holder) carries the
    default TRACK, and that is not an order from the conn holder."""
    rig = Rig(control_conf)
    rig.lease(OPERATOR, "op-1")
    from nav.world.route import RouteRef, VoyagePlan

    rig.p.world.put_voyage(
        VoyagePlan("voy-x", "VOYAGE_STATUS_ACTIVE", RouteRef("r1", 1)), {"voyage_id": "voy-x"}
    )
    rig.run(1.0)
    assert rig.p.selection.voyage.voyage_id == "voy-x"
    assert rig.p.conduct.mode is Mode.READY, rig.p.conduct.reason
    assert "no order" in rig.p.conduct.reason
    p = rig.call(
        "set_guidance_order", {"caller": caller(OPERATOR, token="op-1"), "order": {"track": {}}}
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED"
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason


def test_a_handover_between_operators_keeps_the_order_in_force(control_conf):
    rig = _directed(control_conf)
    rig.lease("anna@roc2", "op-2")
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.UNDERWAY
    assert "carrying out the order of anna@roc2" in rig.p.conduct.reason
    summaries = [o.payload.get("summary", "") for o in rig.published("navigation_decision")]
    assert any("conn passed from ted@roc1 to anna@roc2" in s for s in summaries), summaries


def test_the_holders_lease_lapsing_stops_her_until_a_new_order(control_conf):
    rig = _directed(control_conf)
    rig.p.world.put_lease({"token": "op-1", "released": True}, rig.now)
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert "the conn holder's lease lapsed" in rig.p.conduct.reason
    # The conn comes back to an operator: nothing moves until that holder orders her again.
    rig.lease(OPERATOR, "op-3")
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.FALLBACK
    p = rig.call(
        "set_guidance_order", {"caller": caller(OPERATOR, token="op-3"), "order": {"track": {}}}
    )
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED"
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason


def test_a_lease_left_to_lapse_by_ttl_stops_her_and_frees_the_helm(control_conf):
    rig = _directed(control_conf)
    assert rig.mapping, "guidance never installed its mapping"
    rig.run(31.0)  # no heartbeat: 30 s ttl on this receiver's clock
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert "lease lapsed" in rig.p.conduct.reason
    rig.run(5.0)  # past the neutral burst
    assert rig.mapping == {}, "the operator's helm is still mapped to guidance"


def test_a_station_taking_the_helm_stands_her_down(control_conf):
    rig = _directed(control_conf)
    rig.mapping = wanted_mapping("roc/helm", 1.0)
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.FALLBACK
    assert "a station took the helm" in rig.p.conduct.reason
    rig.mapping = {}  # the station lets go: still no order, so she stays stood down
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.FALLBACK


def test_no_advice_is_auto_accepted_under_an_operators_conn(control_conf):
    rig = _directed(control_conf)
    assert rig.p.conduct.mode is Mode.UNDERWAY

    class Advisor:
        def undecided_advice(self, now_ns):
            raise AssertionError("looked for advice to accept under an operator's conn")

    rig.p.advisor = Advisor()
    assert rig.p._auto_accept(rig.now) == []


# -- orders with no voyage (Ted, 2026-11-25) --------------------------------------------------
# A course, heading, hold, go_to or loiter order steers by her own sensors and needs no route:
# with no voyage it is taken alone, carried out under the conn holder's conn, and lapses when a
# voyage starts. A TRACK order still needs the one ACTIVE voyage that names its route.


def _alone(conf, order) -> tuple[Rig, object]:
    rig = Rig(conf)
    rig.lease(OPERATOR, "op-1")
    rig.run(0.5)
    assert rig.p.selection.voyage is None
    p = rig.call("set_guidance_order", {"caller": caller(OPERATOR, token="op-1"), "order": order})
    rig.run(1.0)
    return rig, p


def test_a_heading_order_with_no_voyage_is_taken_and_carried_out(control_conf):
    rig, p = _alone(control_conf, {"heading_deg": 90.0, "speed_knots": 5.0})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    assert "no voyage" in p.response["detail"]
    assert rig.p.order.kind is Kind.HEADING
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    assert "carrying out the order of ted@roc1" in rig.p.conduct.reason
    assert not rig.p.conn_held, "the conn stays the operator's"
    setpoint = rig.published("navigation_setpoint")[-1].payload
    assert setpoint["purpose"] == "PURPOSE_STEERING" and setpoint["steering"] == {"heading_deg": 90.0}
    assert rig.p.actuation.axes is not None and rig.p.actuation.axes[0] > 0  # turning to 090
    rig.run(5.0)
    assert rig.p.order.kind is Kind.HEADING, "an order with no voyage is kept while none starts"
    assert rig.p.conduct.mode is Mode.UNDERWAY


def test_an_order_with_no_voyage_and_no_speed_takes_the_default_not_a_route_speed(control_conf):
    rig, p = _alone(control_conf, {"course_over_ground_deg": 45.0})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED"
    sp = rig.published("navigation_setpoint")[-1].payload
    assert sp["purpose"] == "PURPOSE_STEERING"
    # A course order holds the speed she has (8 kn in the rig), as it does on a route.
    assert sp["reason"] == "holding the speed she has under a course or heading order"
    assert abs(sp["propulsion"]["sog_mps"] - 8.0 * 0.514444) < 0.01


def test_a_go_to_from_a_standstill_with_no_voyage_takes_the_default_speed(control_conf):
    rig = Rig(control_conf)
    rig.sog = 0.0
    rig.lease(OPERATOR, "op-1")
    rig.run(0.5)
    p = rig.call("set_guidance_order", {"caller": caller(OPERATOR, token="op-1"), "order": _go_to(57.71, 11.95)})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    sp = rig.published("navigation_setpoint")[-1].payload
    assert "from a standstill: the default speed" in sp["reason"]


def test_an_order_with_no_voyage_lapses_when_a_voyage_starts(control_conf):
    rig, _ = _alone(control_conf, {"heading_deg": 90.0})
    assert rig.p.order.kind is Kind.HEADING
    p = rig.call("load_route", load(caller=caller(OPERATOR, token="op-1")))
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(0.5)
    assert rig.p.order.kind is Kind.TRACK, "a heading given with no voyage never steers the next"


def test_an_order_with_no_voyage_stands_down_when_the_lease_lapses(control_conf):
    rig, _ = _alone(control_conf, {"heading_deg": 90.0})
    rig.p.world.put_lease({"token": "op-1", "released": True}, rig.now)
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.FALLBACK, rig.p.conduct.reason
    assert "lease lapsed" in rig.p.conduct.reason


def test_a_track_order_still_needs_a_voyage(control_conf):
    rig, p = _alone(control_conf, {"track": {}})
    assert p.response["result"] == "COMMAND_RESULT_TEMPORARILY_REJECTED"
    assert p.response["detail"] == "no single ACTIVE voyage to steer"
    assert rig.p.conduct.mode is not Mode.UNDERWAY


def test_two_active_voyages_refuse_every_order(control_conf):
    from nav.world.route import RouteRef, VoyagePlan

    rig = Rig(control_conf)
    rig.lease(OPERATOR, "op-1")
    for vid in ("voy-a", "voy-b"):
        rig.p.world.put_voyage(VoyagePlan(vid, "VOYAGE_STATUS_ACTIVE", RouteRef("r1", 1)), {"voyage_id": vid})
    rig.run(0.5)
    p = rig.call("set_guidance_order", {"caller": caller(OPERATOR, token="op-1"), "order": {"heading_deg": 90.0}})
    assert p.response["result"] == "COMMAND_RESULT_TEMPORARILY_REJECTED"


# -- the ANS is the conn's default holder; Slipway instructs (Ted, 2026-11-25) -----------------
# Whenever no station holds the conn the ANS does, from any mode. A station granted the conn on
# request carries the order in force on until it orders otherwise; releasing the conn, or letting
# it lapse, hands her back with that order still in force. Unattended past a limit, she holds
# position. An exercise instructor may change her order even under a station's conn, which stays
# the station's.

INSTRUCTOR = "slipway"


@pytest.fixture
def default_conf(control_conf):
    control_conf["conduct"]["default_holder"] = True
    control_conf["conduct"]["unattended_hold_s"] = 0.0  # the watchdog has its own tests
    control_conf["control"]["controllers"] = [INSTRUCTOR]
    control_conf["control"]["instructors"] = [INSTRUCTOR]
    control_conf["conduct"]["startup_grace_s"] = 0.0  # the grace has its own test
    return control_conf


def _own_lease(rig):
    held = [o.payload for o in rig.published("command_authority") if o.payload.get("controller_id") == ME]
    return held[-1] if held else None


def _sailing(conf) -> Rig:
    """Scenario start: nobody holds the conn, the simulator loads her route, the ANS sails it."""
    rig = Rig(conf)
    p = rig.call("load_route", load(caller=caller(INSTRUCTOR)))
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conn_held and rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    return rig


def _request(rig, requester=OPERATOR, site="roc1"):
    rig.p.world.put_request(
        {"vessel_id": rig.p.conf["entity_id"], "requester_id": requester, "requester_site": site},
        rig.now,
    )


def _granted_token(rig, to=OPERATOR):
    grants = [o.payload for o in rig.published("command_authority") if o.payload.get("controller_id") == to]
    assert grants, f"no grant to {to}"
    return grants[-1]["token"]


def test_the_default_holder_takes_a_free_conn_even_with_nothing_to_sail(default_conf):
    rig = Rig(default_conf)
    rig.run(1.0)
    assert rig.p.selection.voyage is None and rig.p.conduct.mode is Mode.IDLE
    assert rig.p.conn_held, "nobody holds the conn, so the ANS does"
    assert _own_lease(rig) is not None
    rig.run(40.0)  # past a TTL: it keeps the conn (heartbeats) though it has nothing to do
    assert rig.p.conn_held and not rig.published("command_authority")[-1].payload["released"]


def test_without_default_holder_an_idle_ans_takes_no_conn(control_conf):
    control_conf["conduct"]["take_conn_when_free"] = True
    rig = Rig(control_conf)
    rig.run(1.0)
    assert not rig.p.conn_held


def test_a_station_granted_the_conn_carries_the_order_in_force_on(default_conf):
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    assert not rig.p.conn_held, "an operator's request is answered with the conn"
    _granted_token(rig)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    assert "carrying out the order of ted@roc1" in rig.p.conduct.reason
    assert rig.p.order.kind is Kind.TRACK
    assert rig.mapping == wanted_mapping(SRC, 1.0), "the helm is not let go of on a handover"


def test_a_released_conn_comes_back_and_her_order_goes_on(default_conf):
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    token = _granted_token(rig)
    rig.lease(OPERATOR, token)  # the station adopts the grant and heartbeats it
    p = rig.call("set_guidance_order", {"caller": caller(OPERATOR, token=token), "order": {"heading_deg": 90.0}})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(0.5)
    assert "carrying out the order of ted@roc1" in rig.p.conduct.reason
    rig.p.world.put_lease({"token": token, "released": True}, rig.now)
    rig.run(0.5)
    assert rig.p.conn_held, "released: the conn comes back to the default holder"
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason
    assert rig.p.order.kind is Kind.HEADING and rig.p.order.value_deg == 90.0
    summaries = [o.payload.get("summary", "") for o in rig.published("navigation_decision")]
    assert any("stays in force" in s for s in summaries), summaries


def test_a_lapsed_station_lease_hands_her_back_without_stopping(default_conf):
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    rig.lease(OPERATOR, _granted_token(rig))
    rig.run(31.0)  # the station falls silent: no heartbeat for a TTL
    assert rig.p.conn_held
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason


def test_unattended_past_the_limit_she_holds_position(default_conf):
    default_conf["conduct"]["unattended_hold_s"] = 5.0
    rig = _sailing(default_conf)
    rig.run(3.0)
    assert rig.p.order.kind is Kind.TRACK, "not yet: within the limit"
    rig.run(3.0)
    order = rig.p.order
    assert order.kind is Kind.LOITER, order
    assert order.radius_m == default_conf["conduct"]["unattended_hold_radius_m"]
    assert abs(order.position[0] - rig.pos[0]) < 1e-6 and abs(order.position[1] - rig.pos[1]) < 1e-6
    summaries = [o.payload.get("summary", "") for o in rig.published("navigation_decision")]
    assert any("unattended for 5 s" in s and "holding position" in s for s in summaries), summaries
    assert rig.p.conduct.mode is Mode.UNDERWAY


def test_an_order_restarts_the_unattended_clock(default_conf):
    default_conf["conduct"]["unattended_hold_s"] = 5.0
    rig = _sailing(default_conf)
    rig.run(4.0)
    p = rig.call("set_guidance_order", {"caller": caller(INSTRUCTOR), "order": {"heading_deg": 45.0}})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(4.0)
    assert rig.p.order.kind is Kind.HEADING, "the instructor attended her 4 s ago"
    rig.run(2.0)
    assert rig.p.order.kind is Kind.LOITER


def test_a_station_holding_the_conn_is_attending_her(default_conf):
    default_conf["conduct"]["unattended_hold_s"] = 5.0
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    token = _granted_token(rig)
    for _ in range(4):  # 20 s under the station's conn, heartbeating
        rig.lease(OPERATOR, token)
        rig.run(5.0)
    assert rig.p.order.kind is Kind.TRACK


def test_a_zero_limit_never_holds_position(default_conf):
    rig = _sailing(default_conf)  # unattended_hold_s = 0 in this fixture
    rig.run(200.0)
    assert rig.p.order.kind is Kind.TRACK


def test_an_instructor_overrides_the_order_never_the_conn(default_conf):
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    token = _granted_token(rig)
    rig.lease(OPERATOR, token)
    rig.call("set_guidance_order", {"caller": caller(OPERATOR, token=token), "order": {"heading_deg": 90.0}})
    rig.run(0.5)
    p = rig.call("set_guidance_order", {"caller": caller(INSTRUCTOR), "order": {"heading_deg": 180.0}})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(0.5)
    assert rig.p.order.value_deg == 180.0
    assert not rig.p.conn_held, "the conn stays the operator's"
    assert rig.p.lease_book.holder(rig.now).controller_id == OPERATOR
    assert rig.p.conduct.mode is Mode.UNDERWAY and "carrying out the order of ted@roc1" in rig.p.conduct.reason
    state = rig.published("navigation_state")[-1].payload
    if "ordered_by" in state:  # on a keelson build that carries #94's fields
        assert state["ordered_by"] == INSTRUCTOR
    summaries = [o.payload.get("summary", "") for o in rig.published("navigation_decision")]
    assert any(s.startswith("instructor override:") and "ted@roc1's conn" in s for s in summaries), summaries
    # The operator's next order replaces the instructor's.
    rig.call("set_guidance_order", {"caller": caller(OPERATOR, token=token), "order": {"heading_deg": 270.0}})
    rig.run(0.5)
    assert rig.p.order.value_deg == 270.0


def test_a_permitted_controller_that_is_no_instructor_is_still_outranked(default_conf):
    default_conf["control"]["instructors"] = []
    rig = _sailing(default_conf)
    _request(rig)
    rig.run(0.5)
    rig.lease(OPERATOR, _granted_token(rig))
    p = rig.call("set_guidance_order", {"caller": caller(INSTRUCTOR), "order": {"heading_deg": 180.0}})
    assert p.response["result"] == "COMMAND_RESULT_DENIED"


def test_after_a_fallback_any_new_order_re_arms_the_default_holder(default_conf):
    rig = _sailing(default_conf)
    rig.mapping = wanted_mapping("roc/helm", 1.0)  # a station takes the helm
    rig.run(0.5)
    assert rig.p.conduct.mode is Mode.FALLBACK, rig.p.conduct.reason
    rig.mapping = {}
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.FALLBACK, "letting go of the helm is no order"
    p = rig.call("set_guidance_order", {"caller": caller(INSTRUCTOR), "order": {"track": {}}})
    assert p.response["result"] == "COMMAND_RESULT_ACCEPTED", p.response["detail"]
    rig.run(1.0)
    assert rig.p.conduct.mode is Mode.UNDERWAY, rig.p.conduct.reason


def test_default_holder_needs_yield_on_request(default_conf):
    from nav.config import ConfigError, _validate

    default_conf["conduct"]["yield_on_request"] = False
    with pytest.raises(ConfigError, match="yield_on_request"):
        _validate(default_conf)


def test_a_grant_names_the_station_as_operator_at_site(default_conf):
    """A station requests as operator + site in two fields (Crowsnest's CommandRequest); the grant must
    name it `operator@site`, its controller id, or the station never recognises the conn as its own."""
    rig = _sailing(default_conf)
    _request(rig, requester="ted-sup", site="ROC-TED")
    rig.run(0.5)
    grants = [o.payload for o in rig.published("command_authority") if o.payload.get("controller_site") == "ROC-TED"]
    assert grants and grants[-1]["controller_id"] == "ted-sup@ROC-TED", grants
    assert rig.p.lease_book.holder(rig.now).controller_id == "ted-sup@ROC-TED"


def test_a_restarted_ans_does_not_take_a_conn_a_station_still_holds(default_conf):
    """Measured 2026-11-25: every ANS restart took the conn from Ted's ROC. Started, it has heard no lease,
    so the conn looks free; it must wait out a heartbeat before believing that."""
    default_conf["conduct"]["startup_grace_s"] = 12.0
    rig = Rig(default_conf)
    rig.run(5.0)  # just started: the station's next heartbeat is still on its way
    assert not rig.p.conn_held, "took the conn before a holder's heartbeat could arrive"
    rig.lease(OPERATOR, "op-1")  # it arrives
    rig.run(10.0)
    assert not rig.p.conn_held and rig.p.lease_book.holder(rig.now).controller_id == OPERATOR


def test_with_nobody_holding_it_the_ans_takes_the_conn_after_the_grace(default_conf):
    default_conf["conduct"]["startup_grace_s"] = 12.0
    rig = Rig(default_conf)
    rig.run(11.0)
    assert not rig.p.conn_held
    rig.run(2.0)
    assert rig.p.conn_held
