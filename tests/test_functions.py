"""Navigation authority per function (crowsnest-dev#90): reading the document, what the
process holds, and whether it may accept its own COLREG advice -- collision_avoidance's
control method."""

from __future__ import annotations

import pytest

from nav.colreg.records import AdviceRecord
from nav.conduct import functions as F
from nav.conduct.authority import FULL_AUTONOMOUS, InForce
from nav.conduct.state import Mode
from nav.runtime import Processor
from nav.world.model import World

pytestmark = pytest.mark.unit

ME = "navigation/0@nav-test-vessel"
S = 1_000_000_000
T0 = 1_800_000_000 * S


def record(*functions, revision="4"):
    return {
        "vessel_id": "nav-test-vessel",
        "navigation_authority": {
            "assignment_id": "a1",
            "revision": revision,
            "functions": list(functions),
        },
    }


def fn(name, holder=ME, method="CONTROL_METHOD_DELEGATED", **env):
    return {
        "function": f"NAVIGATION_FUNCTION_{name.upper()}",
        "holder": holder,
        "holder_lease_id": "lease",
        "control_method": method,
        "envelope": env,
    }


def ca(method="CONTROL_METHOD_DELEGATED", holder=ME, **env):
    return F.read_document(record(fn("collision_avoidance", holder, method, **env)))


# ---------------------------------------------------------------------------- the document


def test_the_document_is_read_with_its_revision_methods_and_envelope():
    doc = F.read_document(
        record(
            fn(
                "collision_avoidance",
                intervention_window_ms=30000,
                intent_delivery_required=True,
                comms_loss_behaviour="COMMS_LOSS_BEHAVIOUR_CONTINUE",
            ),
            fn("route_modification", holder="anna@ROC-1", method="CONTROL_METHOD_DIRECTED"),
            {
                "function": "NAVIGATION_FUNCTION_NOT_A_FUNCTION",
                "holder": ME,
                "control_method": "CONTROL_METHOD_AUTONOMOUS",
            },
        )
    )
    assert doc.revision == 4
    assert set(doc.functions) == {
        "collision_avoidance",
        "route_modification",
    }  # an unknown function grants nothing
    c = doc.functions["collision_avoidance"]
    assert (
        c.method,
        c.envelope.intervention_window_ms,
        c.envelope.intent_delivery_required,
        c.envelope.comms_loss_behaviour,
    ) == (3, 30000, True, "COMMS_LOSS_BEHAVIOUR_CONTINUE")
    assert doc.functions["route_modification"].method == 2


def test_a_record_without_the_document_is_none():
    assert F.read_document({"vessel_id": "v", "token": "t"}) is None


def test_held_is_ours_at_method_three_or_above():
    doc = F.read_document(
        record(
            fn("collision_avoidance"),
            fn("track_keeping", method="CONTROL_METHOD_MONITORED"),
            fn("speed_control", method="CONTROL_METHOD_DIRECTED"),
            fn("route_execution", holder="anna@ROC-1"),
        )
    )
    assert [f.name for f in F.held(doc, ME)] == ["track_keeping", "collision_avoidance"]
    assert F.held(None, ME) == ()


# ---------------------------------------------------------------------------- the decision


@pytest.mark.parametrize(
    "doc,why",
    [
        (None, "no navigation authority document"),
        (F.read_document(record(fn("track_keeping"))), "held by nobody"),
        (ca(holder="anna@ROC-1"), "held by anna@ROC-1"),
        (ca(method="CONTROL_METHOD_DIRECTED"), "at method 2: a human confirms each decision"),
        (ca(method="CONTROL_METHOD_OPERATED"), "at method 1"),
    ],
)
def test_below_the_divisor_or_not_ours_never_accepts(doc, why):
    v = F.advice_decision(doc, ME, T0, T0 - 600 * S, nav_session_live=True)
    assert v.kind == F.REFUSE and why in v.reason


def test_method_four_and_five_accept_at_once():
    for m in ("CONTROL_METHOD_MONITORED", "CONTROL_METHOD_AUTONOMOUS"):
        assert F.advice_decision(ca(method=m), ME, T0, T0, nav_session_live=False).kind == F.ACCEPT


def test_method_three_waits_until_the_intent_was_published():
    doc = ca(intervention_window_ms=10000)
    v = F.advice_decision(doc, ME, T0 + 60 * S, T0, True, delivered=False)
    assert v == F.Verdict(F.WAIT, "the intent has not been published yet")
    # Methods 4-5 inform rather than wait for a veto, so delivery does not hold them.
    four = ca(method="CONTROL_METHOD_MONITORED")
    assert F.advice_decision(four, ME, T0, T0, False, delivered=False).kind == F.ACCEPT


def test_an_advice_not_yet_published_is_held_by_the_runtime(proc, monkeypatch):
    underway(proc, monkeypatch, record(fn("collision_avoidance", intervention_window_ms=0)))
    proc._published_advice.clear()
    assert proc._auto_accept(T0 + 60 * S) == [] and proc.advisor.recorded == []


def test_method_three_waits_out_the_veto_window():
    doc = ca(intervention_window_ms=30000)
    assert F.advice_decision(doc, ME, T0 + 29 * S, T0, True) == F.Verdict(
        F.WAIT, "the veto window is open (1 s left)"
    )
    assert F.advice_decision(doc, ME, T0 + 30 * S, T0, True).kind == F.ACCEPT


@pytest.mark.parametrize(
    "required,behaviour,expect",
    [
        (
            True,
            "COMMS_LOSS_BEHAVIOUR_CONTINUE",
            F.REFUSE,
        ),  # delivery required: no veto could arrive
        (False, "COMMS_LOSS_BEHAVIOUR_CONTINUE", F.ACCEPT),
        (False, "COMMS_LOSS_BEHAVIOUR_HOLD", F.REFUSE),
        (False, "COMMS_LOSS_BEHAVIOUR_FALLBACK_TO_METHOD_2", F.REFUSE),
        (False, "COMMS_LOSS_BEHAVIOUR_SAFE_STATE", F.REFUSE),
        (False, "COMMS_LOSS_BEHAVIOUR_UNSPECIFIED", F.REFUSE),
    ],
)
def test_silence_with_no_live_commander_is_no_veto_that_could_arrive(required, behaviour, expect):
    doc = ca(
        intervention_window_ms=10000,
        intent_delivery_required=required,
        comms_loss_behaviour=behaviour,
    )
    assert F.advice_decision(doc, ME, T0 + 60 * S, T0, nav_session_live=False).kind == expect


def test_silence_with_a_live_commander_accepts_after_the_window():
    doc = ca(
        intervention_window_ms=10000,
        intent_delivery_required=True,
        comms_loss_behaviour="COMMS_LOSS_BEHAVIOUR_SAFE_STATE",
    )
    assert F.advice_decision(doc, ME, T0 + 11 * S, T0, nav_session_live=True).kind == F.ACCEPT


# ---------------------------------------------------------------------------- the runtime


@pytest.fixture
def proc(conf):
    return Processor(conf, World(), id_factory=lambda: "id-1")


def test_records_are_routed_by_slot_and_only_the_conn_feeds_the_lease_book(proc):
    proc.world.put_lease(record(fn("collision_avoidance")), T0, "nav-test-vessel/navigation")
    proc.world.put_lease(
        {"controller_id": "anna@ROC-1", "token": "sess", "lease_ttl_seconds": 30},
        T0,
        "nav-test-vessel/navigational/session",
    )
    proc.world.put_lease(
        {"controller_id": "ted@ROC-1", "token": "pay", "lease_ttl_seconds": 30},
        T0,
        "nav-test-vessel/payload/gimbal",
    )
    proc._hear_leases()
    assert proc.nav_doc.functions["collision_avoidance"].method == 3
    assert proc.nav_session.holder(T0).token == "sess"
    assert (
        proc.lease_book.holder(T0) is None
    )  # neither the document, the session nor a payload lease is the conn
    proc.world.put_lease(
        {"controller_id": "anna@ROC-1", "token": "conn", "lease_ttl_seconds": 30},
        T0,
        "nav-test-vessel",
    )
    proc._hear_leases()
    assert proc.lease_book.holder(T0).token == "conn"


class FakeAdvisor:
    def __init__(self, advice):
        self.advice, self.recorded = advice, []

    def undecided_advice(self, now_ns):
        return None if self.recorded else self.advice

    def worst_risk(self, advice):
        return "RISK_RISK"

    def record_disposition(self, record):
        self.recorded.append(record)


def advice(valid_from_ns):
    from nav.bus.codec import rfc3339

    return AdviceRecord(
        rfc3339(valid_from_ns),
        "adv-1",
        None,
        "KIND_COURSE_ALTERATION",
        ("enc-1",),
        90.0,
        None,
        None,
        None,
        rfc3339(valid_from_ns),
        rfc3339(valid_from_ns + 600 * S),
        None,
        None,
        ("15",),
        "give way",
    )


def underway(proc, monkeypatch, doc_record, session_live=True):
    proc.advisor = FakeAdvisor(advice(T0))
    proc._published_advice.add("adv-1")  # published on an earlier tick: delivered
    proc.authority = InForce(FULL_AUTONOMOUS, "test")
    monkeypatch.setattr(proc.conduct, "mode", Mode.UNDERWAY, raising=False)
    monkeypatch.setattr(proc.lease, "held", lambda *a, **k: True)
    if doc_record is not None:
        proc.world.put_lease(doc_record, T0, "nav-test-vessel/navigation")
    if session_live:
        proc.world.put_lease(
            {"controller_id": "anna@ROC-1", "token": "sess", "lease_ttl_seconds": 3600},
            T0,
            "nav-test-vessel/navigational/session",
        )
    proc._hear_leases()


def test_full_autonomy_alone_no_longer_accepts_without_a_delegation(proc, monkeypatch):
    underway(proc, monkeypatch, None)
    assert proc._auto_accept(T0 + 60 * S) == [] and proc.advisor.recorded == []


def test_a_delegated_collision_avoidance_accepts_after_its_window(proc, monkeypatch):
    underway(proc, monkeypatch, record(fn("collision_avoidance", intervention_window_ms=20000)))
    assert proc._auto_accept(T0 + 10 * S) == []  # inside the veto window
    out = proc._auto_accept(T0 + 21 * S)
    assert [o.subject for o in out] == ["advice_disposition"] and proc.advisor.recorded[
        0
    ].state == "STATE_ACCEPTED"
    assert "the veto window closed with no veto" in proc.advisor.recorded[0].reason


def test_a_method_two_collision_avoidance_waits_for_a_human(proc, monkeypatch):
    underway(proc, monkeypatch, record(fn("collision_avoidance", method="CONTROL_METHOD_DIRECTED")))
    assert proc._auto_accept(T0 + 3600 * S) == [] and proc.advisor.recorded == []
