"""Behind the motion gate (crowsnest-dev#94): which basis each DOF's candidate claims, the polite
yield, the stream, the human decision's id, and that this backend never installs a mapping."""

from __future__ import annotations

import pytest

from nav import config as config_mod
from nav.bus import gate_candidates
from nav.conduct import functions as F
from nav.guidance import candidates as C
from nav.guidance.axes import FREE, OTHER, OURS, UNKNOWN, Actuation
from nav.runtime import Processor
from nav.world.model import World

pytestmark = pytest.mark.unit

SRC = "navigation/0"
ME = "navigation/0@nav-test-vessel"
TED = "ted@ROC-1"
DOFS = {"rudder": "steering", "throttle_port": "throttle", "throttle_stbd": "throttle"}
EPOCH = ("t" * 32, 4)


def gmap(holder=TED, source=SRC, fence=EPOCH, dofs=DOFS):
    return C.GateMap(tuple(C.GateRow(d, f"m-{d}", holder, source) for d in dofs), fence)


def doc(*held, revision=5, holder=ME, method="CONTROL_METHOD_DELEGATED"):
    return F.read_document(
        {
            "navigation_authority": {
                "revision": str(revision),
                "functions": [
                    {
                        "function": f"NAVIGATION_FUNCTION_{f.upper()}",
                        "holder": holder,
                        "control_method": method,
                    }
                    for f in held
                ],
            }
        }
    )


def run(
    gm=None,
    d=None,
    holder=TED,
    epoch=EPOCH,
    avoiding=False,
    advice=None,
    order="track",
    order_id="o-7",
    axes=(12.0, 40.0),
):
    return C.candidates(
        axes,
        gm if gm is not None else gmap(),
        DOFS,
        SRC,
        ME,
        d,
        holder,
        epoch,
        avoiding,
        advice,
        order,
        order_id,
    )


def by_dof(found):
    return {c.dof: c for c in found}


# ---------------------------------------------------------------------------- the bases


def test_a_delegated_function_decides_and_a_human_conn_is_carried_out():
    found, why = run(d=doc("track_keeping", "speed_control"))
    c = by_dof(found)
    assert why == () and set(c) == set(DOFS)
    assert (c["rudder"].decision, c["rudder"].function, c["rudder"].nav_revision) == (
        C.NAV_FUNCTION,
        "track_keeping",
        5,
    )
    assert (c["rudder"].execution, c["rudder"].mapping_id) == (C.CARRY_OUT, "m-rudder")
    assert c["rudder"].value_pct == 12.0 and c["throttle_stbd"].value_pct == 40.0
    assert c["throttle_port"].function == "speed_control"


def test_without_a_delegation_the_humans_order_decides():
    found, _ = run(d=None)
    assert {(c.decision, c.order_id, c.decision_id) for c in found} == {
        (C.HUMAN_ORDER, "o-7", "o-7")
    }


def test_each_output_independently():
    """Steering delegated, speed not: the rudder under NAV_FUNCTION, the throttles under the
    order."""
    c = by_dof(run(d=doc("track_keeping"))[0])
    assert c["rudder"].decision == C.NAV_FUNCTION and c["throttle_port"].decision == C.HUMAN_ORDER


@pytest.mark.parametrize("method", ["CONTROL_METHOD_DIRECTED", "CONTROL_METHOD_OPERATED"])
def test_below_the_divisor_a_function_is_not_this_processs_to_decide(method):
    c = by_dof(run(d=doc("track_keeping", "speed_control", method=method))[0])
    assert {x.decision for x in c.values()} == {C.HUMAN_ORDER}


def test_a_heading_order_is_only_ever_a_humans_decision():
    c = by_dof(run(d=doc("track_keeping", "speed_control"), order="heading_deg")[0])
    assert c["rudder"].decision == C.HUMAN_ORDER and c["throttle_port"].decision == C.NAV_FUNCTION


def test_avoiding_is_collision_avoidance_with_the_advice_as_the_decision():
    c = by_dof(run(d=doc("collision_avoidance"), avoiding=True, advice="adv-1")[0])
    assert (c["rudder"].function, c["rudder"].decision_id) == ("collision_avoidance", "adv-1")


def test_full_automation_is_direct_and_never_a_human_order():
    found, why = run(gmap(holder=ME), d=doc("track_keeping", "speed_control"), holder=ME)
    assert {(c.execution, c.mapping_id) for c in found} == {(C.DIRECT, "")}
    found, why = run(gmap(holder=ME), d=None, holder=ME)
    assert found == () and all("not delegated to this process" in w for w in why)


def test_no_basis_no_candidate():
    found, why = run(d=None, order_id=None)
    assert found == () and len(why) == 3 and "no human order is in force" in why[0]


def test_only_its_own_rows_and_only_the_configured_dofs():
    rows = (
        *gmap().rows,
        C.GateRow("bow_thruster", "m-b", TED, SRC),
        C.GateRow("rudder2", "m-r2", TED, "ROC-1/helm-1/rudder2"),
    )
    found, _ = run(C.GateMap(rows, EPOCH))
    assert {c.dof for c in found} == set(DOFS)


def test_a_dof_a_station_holds_gets_no_candidate_from_this_process():
    """The rudder mapped to a human helm, the throttles to this process: throttle only."""
    rows = (
        C.GateRow("rudder", "m-rudder", TED, "ROC-1/helm-1/rudder"),
        *(r for r in gmap().rows if r.dof != "rudder"),
    )
    found, _ = run(C.GateMap(rows, EPOCH))
    assert {c.dof for c in found} == {"throttle_port", "throttle_stbd"}


def test_silence_is_silence():
    assert run(axes=None) == ((), ())


# ---------------------------------------------------------------------------- the polite yield


def test_a_new_conn_holder_silences_it_before_the_map_is_read_again():
    found, why = run(holder="anna@ROC-2")
    assert found == () and "the conn is anna@ROC-2's now" in why[0]
    assert run(holder=None)[0] == ()


def test_a_new_motion_epoch_silences_it_until_a_map_under_that_epoch_is_read():
    found, why = run(epoch=("t" * 32, 5))
    assert found == () and why == ("the gate's motion epoch changed; re-reading its map",)
    assert run(gm=gmap(fence=("t" * 32, 5)), epoch=("t" * 32, 5))[0] != ()
    assert run(epoch=None)[0] != ()  # no resolved_control heard yet: the map read stands


def test_an_unread_map_proposes_nothing():
    assert (
        C.candidates((1.0, 1.0), None, DOFS, SRC, ME, None, TED, None, False, None, "track", "o")[0]
        == ()
    )


@pytest.mark.parametrize(
    "gm,want",
    [
        (None, UNKNOWN),
        (C.GateMap((), None), FREE),
        (gmap(), OURS),
        (gmap(dofs={"rudder": "steering"}), OTHER),  # only part of the helm
        (gmap(source="ROC-1/helm-1"), OTHER),
    ],
)
def test_helm_status_from_the_gates_map(gm, want):
    assert C.gate_ownership(gm, SRC, DOFS) == want


# ---------------------------------------------------------------------------- the bus side


class _Pub:
    def __init__(self):
        self.data = []

    def put(self, data):
        self.data.append(data)


def backend(conf):
    conf["guidance"]["backend"] = "gate"
    conf["guidance"]["gate"] = {**config_mod.GATE_DEFAULTS, "dofs": DOFS}
    return gate_candidates.GateBackend(None, conf, World())


def test_one_stream_one_rising_sequence_across_every_dof(conf):
    from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate

    b = backend(conf)
    found, _ = run(d=doc("track_keeping"))
    msgs = [GuidanceCandidate.FromString(b.message(c, 10**18).SerializeToString()) for c in found]
    assert [m.stream.sequence for m in msgs] == [1, 2, 3]
    assert {m.stream.stream_id for m in msgs} == {b.stream_id} and len(b.stream_id) == 32
    assert all(m.source == SRC and m.stream.clock_quality == 2 for m in msgs)  # FREE_RUNNING
    r = next(m for m in msgs if m.dof == "rudder")
    assert r.WhichOneof("decision_basis") == "nav_function"
    assert r.nav_function.navigation_authority_revision == 5
    assert (r.WhichOneof("execution_basis"), r.carry_out.control_mapping_id) == (
        "carry_out",
        "m-rudder",
    )
    t = next(m for m in msgs if m.dof == "throttle_port")
    assert (t.WhichOneof("decision_basis"), t.human_order.order_id) == ("human_order", "o-7")
    assert backend(conf).stream_id != b.stream_id  # a new incarnation on every start


def test_direct_is_set_on_the_wire(conf):
    from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate

    (c, *_), _ = run(gmap(holder=ME), d=doc("track_keeping", "speed_control"), holder=ME)
    m = GuidanceCandidate.FromString(backend(conf).message(c, 1).SerializeToString())
    assert m.WhichOneof("execution_basis") == "direct"


def test_the_gate_backend_never_installs_a_mapping(conf):
    """Asked to install, it only ever READS the gate's map: its one RPC is get_gate_state."""

    class Session:
        def __init__(self):
            self.keys = []

        def get(self, key, **kw):
            self.keys.append(key)
            return []

    b = backend(conf)
    b.session = Session()
    b.apply(Actuation(axes=(1.0, 1.0), valid_until_ns=1, mapping="install"))
    b.poll()
    b.poll()
    assert b.session.keys and all("/control_gate/v1/get_gate_state/" in k for k in b.session.keys)


def test_the_map_is_read_from_get_gate_state(conf):
    from keelson.interfaces.ControlGate_pb2 import GateState
    from keelson.payloads.ControlMapping_pb2 import ControlMapping

    st = GateState(
        mappings=[
            ControlMapping(
                dof="rudder", mapping_id="m-r", authority_holder=TED, execution_source=SRC
            )
        ]
    )
    st.fence.term, st.fence.generation = "t" * 32, 9
    b = backend(conf)
    b._call = lambda procedure, payload: st.SerializeToString()
    assert b.poll() == C.GateMap((C.GateRow("rudder", "m-r", TED, SRC),), ("t" * 32, 9))
    b._call = lambda procedure, payload: None
    assert b.poll() is None


# ---------------------------------------------------------------------------- the runtime


def gate_proc(conf):
    conf["modules"].update(route_monitor=True, guidance=True, conduct=True, control=True)
    conf["guidance"].update(backend="gate", full_ahead_sog_knots=20.0)
    conf["guidance"]["gate"] = {**config_mod.GATE_DEFAULTS, "dofs": DOFS}
    ids = iter(f"id-{i}" for i in range(100))
    return Processor(conf, World(), id_factory=lambda: next(ids))


def test_every_order_taken_draws_a_new_id_and_names_who_gave_it(conf):
    from nav.control.orders import TRACK

    p = gate_proc(conf)
    p._take_order(TRACK, "v-1", TED)
    first = p._order_id
    p._take_order(TRACK, "v-1", TED)
    assert p._order_id != first and p._ordered_by == TED
    p.directed = True
    assert p._order_in_force(None) == (p._order_id, TED)
    p.directed = p.conn_held = False
    assert p._order_in_force(None) == (None, "")  # not carrying anyone's order out


def test_an_advice_a_human_accepted_is_the_decision_in_force(conf):
    from nav.colreg.records import DispositionRecord

    p = gate_proc(conf)
    p.directed = True
    p._take_order(__import__("nav.control.orders", fromlist=["TRACK"]).TRACK, "v-1", TED)

    class Adv:
        advice_id = "adv-1"

    class Advisor:
        dispositions = {
            "adv-1": DispositionRecord("t", "adv-1", "v-1", "STATE_ACCEPTED", decided_by=TED)
        }

    p.advisor = Advisor()
    assert p._order_in_force(Adv()) == ("advice/adv-1", TED)
    Advisor.dispositions["adv-1"] = DispositionRecord(
        "t", "adv-1", "v-1", "STATE_ACCEPTED", decided_by="navigation/0/conduct"
    )
    assert p._order_in_force(Adv())[0] == p._order_id  # its own acceptance is no human's order


def test_behind_a_gate_the_actuation_never_asks_for_a_mapping(conf):
    p = gate_proc(conf)
    p.guidance.actuation = Actuation(axes=(1.0, 2.0), valid_until_ns=10, mapping="install")
    act = p.actuation
    assert act.mapping is None and act.candidates == ()


def test_the_helm_status_behind_a_gate_comes_from_its_map(conf):
    p = gate_proc(conf)
    assert p._helm(10**9) == UNKNOWN
    p.world.put_gate(gmap(), 10**9)
    assert p._helm(10**9) == OURS
    assert p._helm(10**9 + int(conf["guidance"]["mapping_max_age_s"] * 1e9) + 1) == UNKNOWN


@pytest.mark.parametrize(
    "gate,why",
    [
        ({"dofs": {"rudder": "steering"}}, "both steering and throttle"),
        ({"dofs": {"rudder": "yaw"}}, "maps each DOF"),
        ({"dofs": DOFS, "clock_quality": "gps"}, "clock_quality"),
        ({"dofs": DOFS, "responder": "x"}, "unknown guidance.gate"),
    ],
)
def test_the_gate_config_is_validated(conf, gate, why):
    conf["guidance"].update(backend="gate")
    conf["guidance"]["gate"] = gate
    with pytest.raises(config_mod.ConfigError, match=why):
        config_mod._validate_guidance(conf)


def test_the_axes_backend_is_still_the_default(conf):
    assert config_mod.DEFAULTS["guidance"]["backend"] == "axes"
    p = Processor(conf, World())
    assert p._gate is False


def test_navigation_state_names_the_order_in_force_and_encodes(conf):
    from nav.bus import codec
    from nav.control.orders import TRACK

    p = gate_proc(conf)
    if not p._state_order:
        pytest.skip("this keelson build has no NavigationState.order_id")
    payload = p._state(10**18).to_payload()
    assert "order_id" not in payload  # no order taken: absence, never an empty id
    p._take_order(TRACK, "v-1", TED)
    p.directed = True
    payload = p._state(10**18).to_payload()
    assert (payload["order_id"], payload["ordered_by"]) == (p._order_id, TED)
    back = codec._from_proto_bytes(
        "navigation_state", codec._to_proto_bytes("navigation_state", payload)
    )
    assert back["order_id"] == p._order_id


def test_navigation_state_reports_the_functions_in_force_and_encodes(conf):
    from nav.bus import codec

    p = gate_proc(conf)
    if not p._state_functions:
        pytest.skip("this keelson build has no NavigationState.functions_in_force")
    p.nav_doc = doc("track_keeping", revision=8)
    payload = p._state(10**18).to_payload()
    assert payload["navigation_authority_revision"] == "8"
    (f,) = payload["functions_in_force"]
    assert (f["function"], f["control_method"]) == (
        "NAVIGATION_FUNCTION_TRACK_KEEPING",
        "CONTROL_METHOD_DELEGATED",
    )
    back = codec._from_proto_bytes(
        "navigation_state", codec._to_proto_bytes("navigation_state", payload)
    )
    assert back["functions_in_force"][0]["holder"] == ME
