"""The navigation state machine's guard matrix: every mode against every gate."""

from __future__ import annotations

from dataclasses import replace

import pytest

from nav.conduct.fsm import Conduct, Gates
from nav.conduct.state import Mode, SubMode

pytestmark = pytest.mark.unit

GOOD = Gates(
    route="declared-sim",
    route_reason="monitoring declared route from source sim",
    guidance=True,
    conn_held=True,
    conn_token="t1",
    conn_self_issued=True,
    may_drive=True,
    authority_note="authority level fixed by configuration",
    ownship_fresh=True,
    missing_inputs=(),
    helm="ours",
    arrived=False,
    avoiding=False,
)

BAD = {
    "route": dict(route=None, route_reason="no active voyage and no live declared route"),
    "conn": dict(conn_held=False, conn_token=None, conn_self_issued=False),
    "authority": dict(may_drive=False, authority_note="operational_authority is stale"),
    "ownship": dict(ownship_fresh=False, missing_inputs=("location_fix",)),
    "helm_other": dict(helm="other"),
    "helm_unknown": dict(helm="unknown"),
    "arrived": dict(arrived=True),
}


def machine_in(mode: Mode) -> Conduct:
    c = Conduct()
    if mode is Mode.IDLE:
        return c
    c.step(replace(GOOD, **BAD["conn"]))
    assert c.mode is Mode.READY
    if mode is Mode.READY:
        return c
    c.step(GOOD)
    assert c.mode is Mode.UNDERWAY
    if mode is Mode.UNDERWAY:
        return c
    if mode is Mode.FALLBACK:
        c.step(replace(GOOD, **BAD["conn"]))
        assert c.mode is Mode.FALLBACK
        return c
    c.step(replace(GOOD, **BAD["arrived"]))
    assert c.mode is Mode.ARRIVED
    if mode is Mode.ARRIVED:
        return c
    c.step(replace(GOOD, **BAD["arrived"], **BAD["conn"]))
    assert c.mode is Mode.ENDED
    return c


EXPECTED = {
    # mode before, the one bad gate -> mode after
    (Mode.IDLE, None): Mode.UNDERWAY,
    (Mode.IDLE, "route"): Mode.IDLE,
    (Mode.IDLE, "conn"): Mode.READY,
    (Mode.IDLE, "authority"): Mode.READY,
    (Mode.IDLE, "ownship"): Mode.READY,
    (Mode.IDLE, "helm_other"): Mode.READY,
    (Mode.IDLE, "helm_unknown"): Mode.READY,
    (Mode.IDLE, "arrived"): Mode.READY,
    (Mode.READY, None): Mode.UNDERWAY,
    (Mode.READY, "route"): Mode.IDLE,
    (Mode.READY, "conn"): Mode.READY,
    (Mode.READY, "authority"): Mode.READY,
    (Mode.READY, "ownship"): Mode.READY,
    (Mode.READY, "helm_other"): Mode.READY,
    (Mode.READY, "helm_unknown"): Mode.READY,
    (Mode.READY, "arrived"): Mode.READY,
    (Mode.UNDERWAY, None): Mode.UNDERWAY,
    (Mode.UNDERWAY, "route"): Mode.FALLBACK,
    (Mode.UNDERWAY, "conn"): Mode.FALLBACK,
    (Mode.UNDERWAY, "authority"): Mode.FALLBACK,
    (Mode.UNDERWAY, "ownship"): Mode.FALLBACK,
    (Mode.UNDERWAY, "helm_other"): Mode.FALLBACK,
    (Mode.UNDERWAY, "helm_unknown"): Mode.UNDERWAY,  # a slow poll is not a lost helm
    (Mode.UNDERWAY, "arrived"): Mode.ARRIVED,
    # Still holding the same token: no resume. The token at fallback was t1 or None.
    (Mode.FALLBACK, None): Mode.FALLBACK,
    (Mode.FALLBACK, "route"): Mode.FALLBACK,
    (Mode.FALLBACK, "conn"): Mode.FALLBACK,
    (Mode.FALLBACK, "authority"): Mode.FALLBACK,
    (Mode.FALLBACK, "ownship"): Mode.FALLBACK,
    (Mode.FALLBACK, "helm_other"): Mode.FALLBACK,
    (Mode.FALLBACK, "helm_unknown"): Mode.FALLBACK,
    (Mode.FALLBACK, "arrived"): Mode.FALLBACK,
    (Mode.ARRIVED, None): Mode.ARRIVED,
    (Mode.ARRIVED, "route"): Mode.ARRIVED,
    (Mode.ARRIVED, "conn"): Mode.ENDED,
    (Mode.ARRIVED, "authority"): Mode.ARRIVED,
    (Mode.ARRIVED, "ownship"): Mode.ARRIVED,
    (Mode.ARRIVED, "helm_other"): Mode.ARRIVED,
    (Mode.ARRIVED, "helm_unknown"): Mode.ARRIVED,
    (Mode.ARRIVED, "arrived"): Mode.ARRIVED,
    (Mode.ENDED, None): Mode.ENDED,
    (Mode.ENDED, "route"): Mode.IDLE,
    (Mode.ENDED, "conn"): Mode.ENDED,
    (Mode.ENDED, "authority"): Mode.ENDED,
    (Mode.ENDED, "ownship"): Mode.ENDED,
    (Mode.ENDED, "helm_other"): Mode.ENDED,
    (Mode.ENDED, "helm_unknown"): Mode.ENDED,
    (Mode.ENDED, "arrived"): Mode.ENDED,
}


@pytest.mark.parametrize(("before", "bad"), sorted(EXPECTED, key=lambda k: (k[0].value, str(k[1]))))
def test_guard_matrix(before, bad):
    c = machine_in(before)
    step = c.step(replace(GOOD, **BAD[bad]) if bad else GOOD)
    assert step.mode is EXPECTED[(before, bad)], step.reason
    assert (step.sub_mode is SubMode.TRACK) == (step.mode is Mode.UNDERWAY)


def test_the_matrix_covers_every_mode_and_gate():
    modes = {m for m in Mode if m is not Mode.UNSPECIFIED}
    assert {k for k in EXPECTED} == {(m, b) for m in modes for b in (None, *BAD)}


@pytest.mark.parametrize("bad", ["conn", "authority", "ownship", "helm_other", "helm_unknown"])
def test_ready_says_what_keeps_it_from_getting_underway(bad):
    c = machine_in(Mode.READY)
    step = c.step(replace(GOOD, **BAD[bad]))
    assert "not underway" in step.reason


def test_without_guidance_ready_is_monitoring_only():
    step = Conduct().step(replace(GOOD, guidance=False))
    assert step.mode is Mode.READY and step.reason == GOOD.route_reason


def test_accepted_advice_is_the_avoiding_sub_mode():
    c = machine_in(Mode.UNDERWAY)
    assert c.step(replace(GOOD, avoiding=True)).sub_mode is SubMode.AVOIDING


def test_fallback_resumes_only_on_a_grant_it_did_not_issue_itself():
    c = machine_in(Mode.FALLBACK)
    assert c.step(replace(GOOD, conn_token="t2", conn_self_issued=True)).mode is Mode.FALLBACK
    step = c.step(replace(GOOD, conn_token="granted", conn_self_issued=False))
    assert step.mode is Mode.READY and "granted" in step.reason
    assert c.step(GOOD).mode is Mode.UNDERWAY


def test_fallback_clears_to_idle_when_there_is_nothing_left_to_navigate():
    c = machine_in(Mode.FALLBACK)
    assert c.step(replace(GOOD, **BAD["route"], **BAD["conn"])).mode is Mode.IDLE


def test_fallback_keeps_the_reason_it_fell_back_for():
    c = machine_in(Mode.UNDERWAY)
    reason = c.step(replace(GOOD, **BAD["ownship"])).reason
    assert c.step(replace(GOOD, **BAD["ownship"])).reason == reason
    assert "location_fix" in reason


def test_a_step_reports_whether_the_mode_changed():
    c = machine_in(Mode.READY)
    first = c.step(GOOD)
    assert first.changed and first.before is Mode.READY
    assert not c.step(GOOD).changed


# -- carrying out the conn holder's order (navigation_conduct, 2026-09-29) ----------------------

DIRECTED = replace(
    GOOD,
    conn_held=False,
    conn_token=None,
    conn_self_issued=False,
    directed=True,
    directed_by="ted@roc1",
    order_serial=1,
    conn_holder="ted@roc1",
)


def _directed_underway() -> Conduct:
    c = machine_in(Mode.READY)
    step = c.step(DIRECTED)
    assert step.mode is Mode.UNDERWAY, step.reason
    return c


def test_an_order_from_the_conn_holder_gets_her_underway_without_the_conn():
    c = machine_in(Mode.READY)
    step = c.step(DIRECTED)
    assert step.mode is Mode.UNDERWAY and step.sub_mode is SubMode.TRACK
    assert "carrying out the order of ted@roc1" in step.reason
    assert "conn held" not in step.reason


def test_without_an_order_from_the_holder_she_stays_ready():
    c = machine_in(Mode.READY)
    step = c.step(replace(DIRECTED, directed=False, order_serial=0))
    assert step.mode is Mode.READY
    assert "no order" in step.reason


@pytest.mark.parametrize(
    ("bad", "reason"),
    [
        (dict(directed=False, conn_holder=None), "the conn holder's lease lapsed"),
        (dict(helm="other"), "a station took the helm"),
        (dict(directed=False, helm="other"), "a station took the helm"),
        (dict(directed=False, conn_holder="navigation/1@x"), "has given no order"),
    ],
)
def test_the_holders_lease_lapsing_or_the_helm_taken_is_fallback(bad, reason):
    c = _directed_underway()
    step = c.step(replace(DIRECTED, **bad))
    assert step.mode is Mode.FALLBACK
    assert reason in step.reason


def test_only_a_new_order_from_the_holder_re_arms_fallback():
    c = _directed_underway()
    c.step(replace(DIRECTED, directed=False, conn_holder=None))
    assert c.mode is Mode.FALLBACK
    # The same order again (the serial it fell back on) is not a new order.
    assert c.step(DIRECTED).mode is Mode.FALLBACK
    step = c.step(replace(DIRECTED, order_serial=2))
    assert step.mode is Mode.READY and "new order from the conn holder ted@roc1" in step.reason
    assert c.step(replace(DIRECTED, order_serial=2)).mode is Mode.UNDERWAY


def test_arrival_under_the_holders_order_ends_at_once():
    c = _directed_underway()
    assert c.step(replace(DIRECTED, arrived=True)).mode is Mode.ARRIVED
    step = c.step(replace(DIRECTED, arrived=True))
    assert step.mode is Mode.ENDED and "order is carried out" in step.reason


def test_a_handover_between_operators_keeps_her_underway_under_the_new_holder():
    c = _directed_underway()
    step = c.step(replace(DIRECTED, directed_by="anna@roc2", conn_holder="anna@roc2"))
    assert step.mode is Mode.UNDERWAY and not step.changed
    assert "carrying out the order of anna@roc2" in step.reason


# A finished route ends only a TRACK order: a course, heading, hold, go_to or loiter order is not
# about the route's end (live on slipway-usv-alpha: orders blocked as "already at the final
# waypoint").
OFF_TRACK = ("course_over_ground_deg", "heading_deg", "hold", "go_to", "loiter")


@pytest.mark.parametrize("order", OFF_TRACK)
def test_a_finished_route_does_not_block_an_off_track_order(order):
    c = machine_in(Mode.READY)
    step = c.step(replace(GOOD, arrived=True, order=order))
    assert step.mode is Mode.UNDERWAY, step.reason


def test_a_finished_route_still_blocks_a_track_order():
    c = machine_in(Mode.READY)
    step = c.step(replace(GOOD, arrived=True, order="track"))
    assert step.mode is Mode.READY and "already at the final waypoint" in step.reason


@pytest.mark.parametrize("order", OFF_TRACK)
def test_underway_on_an_off_track_order_stays_underway_past_the_routes_end(order):
    c = Conduct()
    c.step(replace(GOOD, order=order))
    assert c.mode is Mode.UNDERWAY
    step = c.step(replace(GOOD, arrived=True, order=order))
    assert step.mode is Mode.UNDERWAY and not step.changed


def test_directed_course_order_past_the_routes_end_goes_underway():
    c = Conduct()
    step = c.step(replace(DIRECTED, arrived=True, order="course_over_ground_deg"))
    assert step.mode is Mode.UNDERWAY, step.reason
