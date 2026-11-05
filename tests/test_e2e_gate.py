"""Backend B on an isolated local zenoh peer session, against a fake control_gate/v1 responder
(crowsnest-dev#94). What only the bus shows: the map is read through get_gate_state, a candidate per
DOF goes out on guidance_candidate/{producer}/{dof} with one rising stream, a new motion epoch on
resolved_control is heard, and silence follows the actuation's validity."""

from __future__ import annotations

import time

import keelson
import pytest

from nav import config as config_mod
from nav.bus.gate_candidates import GateBackend
from nav.guidance import candidates as C
from nav.guidance.axes import Actuation
from nav.world.model import World
from tests.test_e2e_bus import session  # noqa: F401  (fixture)

pytestmark = pytest.mark.e2e

DOFS = {"rudder": "steering", "throttle": "throttle"}


def wait(pred, timeout=5.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred():
            return True
        time.sleep(0.02)
    return False


def test_candidates_flow_to_the_gate_and_the_map_is_only_read(session, conf):  # noqa: F811
    from keelson.interfaces.ControlGate_pb2 import GateState
    from keelson.payloads.ControlMapping_pb2 import ControlMapping
    from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate
    from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

    conf["guidance"].update(backend="gate", mapping_poll_s=0.2, publish_hz=20.0)
    conf["guidance"]["gate"] = {**config_mod.GATE_DEFAULTS, "dofs": DOFS}
    realm, entity = conf["realm"], conf["entity_id"]

    state = GateState(
        mappings=[
            ControlMapping(
                dof=d,
                mapping_id=f"m-{d}",
                authority_holder="ted@ROC-1",
                execution_source="navigation/0",
            )
            for d in DOFS
        ]
    )
    state.fence.term, state.fence.generation = "a" * 32, 3
    asked = []

    def on_get(query):
        asked.append(str(query.key_expr))
        query.reply(query.key_expr, state.SerializeToString())

    rpc = keelson.construct_rpc_key(
        realm, entity, "control_gate", "v1", "get_gate_state", "control-gate"
    )
    q = session.declare_queryable(rpc, on_get)
    heard = []
    sub = session.declare_subscriber(
        keelson.construct_pubsub_key(realm, entity, "guidance_candidate", "**"),
        lambda s: heard.append(
            (
                str(s.key_expr),
                GuidanceCandidate.FromString(keelson.uncover(bytes(s.payload.to_bytes()))[2]),
            )
        ),
    )
    world = World()
    backend = GateBackend(session, conf, world)
    backend.start()
    try:
        assert wait(lambda: world.gate() is not None), "the gate's map was never read"
        gm = world.gate()[0]
        assert gm.fence == ("a" * 32, 3) and {r.dof for r in gm.rows} == set(DOFS)

        found, _ = C.candidates(
            (10.0, 30.0),
            gm,
            DOFS,
            "navigation/0",
            "navigation/0@x",
            None,
            "ted@ROC-1",
            None,
            False,
            None,
            "track",
            "o-1",
        )
        backend.apply(
            Actuation(
                axes=(10.0, 30.0), valid_until_ns=time.time_ns() + int(0.6e9), candidates=found
            )
        )
        assert wait(lambda: {m.dof for _, m in heard} == set(DOFS))
        keys = {k for k, _ in heard}
        assert keys == {
            keelson.construct_pubsub_key(
                realm, entity, "guidance_candidate", f"navigation/0/guidance/{d}"
            )
            for d in DOFS
        }
        seqs = [m.stream.sequence for _, m in heard]
        assert seqs == sorted(seqs) and len(set(seqs)) == len(
            seqs
        )  # one stream, rising across DOFs
        assert {m.human_order.order_id for _, m in heard} == {"o-1"}
        assert {m.carry_out.control_mapping_id for _, m in heard} == {f"m-{d}" for d in DOFS}

        # Silence after the actuation's validity: nothing is held past it.
        time.sleep(0.8)
        n = len(heard)
        time.sleep(0.3)
        assert len(heard) == n

        # A new motion epoch is heard and wakes a re-read at once.
        before = len(asked)
        rc = ResolvedControl(vessel_id=entity)
        rc.motion_epoch.term, rc.motion_epoch.generation = "a" * 32, 4
        session.put(
            keelson.construct_pubsub_key(realm, entity, "resolved_control", entity),
            keelson.enclose(rc.SerializeToString()),
        )
        assert wait(lambda: world.gate_epoch() == ("a" * 32, 4))
        assert wait(lambda: len(asked) > before, timeout=0.15)
        # It only ever read the map: no vehicle_control call, no set of any kind.
        assert all("/control_gate/v1/get_gate_state/" in k for k in asked)
    finally:
        backend.close()
        sub.undeclare()
        q.undeclare()
