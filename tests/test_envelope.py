from __future__ import annotations

import keelson
import pytest
from google.protobuf import json_format

from nav.route_monitor.envelope import ExceedanceTracker, evaluate
from nav.world.latch import Sample
from nav.world.route import RouteLimits, RouteRef

pytestmark = pytest.mark.unit

S = 1_000_000_000
REF = RouteRef("r1", 3)
LIMITS = RouteLimits(max_wind_speed_mps=12.0, min_visibility_m=1000.0, max_roll_deg=20.0)


def samples(**values):
    return {k: Sample(v, received_ns=0, measured_ns=5 * S) for k, v in values.items()}


def by_id(standings):
    return {s.limit_id: s for s in standings}


def test_unmeasured_limits_are_not_reported_as_within():
    st = by_id(evaluate(LIMITS, samples(true_wind_speed_mps=8.0)))
    assert st["max_wind_speed_mps"].state == "STATE_WITHIN"
    assert st["min_visibility_m"].state == "STATE_UNMEASURED"
    assert "measured_value" not in st["min_visibility_m"].to_payload()
    assert set(st) == {"max_wind_speed_mps", "min_visibility_m", "max_roll_deg"}


def test_lower_bound_and_absolute_roll():
    st = by_id(evaluate(LIMITS, samples(visibility_range_m=800.0, roll_deg=-25.0)))
    assert st["min_visibility_m"].state == "STATE_BREACHED"
    assert st["max_roll_deg"].state == "STATE_BREACHED"
    assert st["max_roll_deg"].measured_value == 25.0


def test_standing_payload_fits_the_keelson_proto():
    payload = by_id(evaluate(LIMITS, samples(true_wind_speed_mps=8.0)))["max_wind_speed_mps"]
    msg = json_format.ParseDict(
        payload.to_payload(),
        keelson.get_protobuf_message_class_from_type_name("keelson.EnvelopeLimitState")(),
    )
    assert msg.measured_at.ToNanoseconds() == 5 * S


def run(tracker, wind, t):
    return tracker.step(
        evaluate(LIMITS, samples(true_wind_speed_mps=wind)),
        t * S,
        REF,
        "wp1",
        LIMITS,
        ("RESPONSE_ALERTED",),
    )


def test_breach_opens_restates_and_closes_with_margin():
    tracker = ExceedanceTracker("voy", clear_margin=0.05, id_factory=lambda: "ex-1")
    assert run(tracker, 11.0, 1) == []
    (opened,) = run(tracker, 13.0, 2)
    assert run(tracker, 15.0, 3) == []  # still open, no new record
    assert run(tracker, 11.8, 4) == []  # inside the limit but not by the margin
    (closed,) = run(tracker, 11.0, 5)

    assert opened["exceedance_id"] == closed["exceedance_id"] == "ex-1"
    assert "cleared_at" not in opened and "peak_value" not in opened
    for key in (
        "detected_at",
        "limit_value",
        "measured_value",
        "responses",
        "responded_at",
        "voyage_id",
    ):
        assert closed[key] == opened[key]
    assert closed["peak_value"] == 15.0
    assert tracker.open_limit_ids == set()

    cls = keelson.get_protobuf_message_class_from_type_name("keelson.EnvelopeExceedance")
    msg = json_format.ParseDict(closed, cls())
    assert msg.cleared_at.ToNanoseconds() == 5 * S and msg.detected_at.ToNanoseconds() == 2 * S
    assert len(msg.envelope_config_digest) == 32


def test_unmeasured_neither_opens_nor_closes():
    tracker = ExceedanceTracker("voy", id_factory=lambda: "ex")
    run(tracker, 13.0, 1)
    assert tracker.step(evaluate(LIMITS, {}), 2 * S, REF, None, LIMITS, ("RESPONSE_NONE",)) == []
    assert tracker.open_limit_ids == {"max_wind_speed_mps"}


def test_response_none_has_no_responded_at():
    tracker = ExceedanceTracker("voy", id_factory=lambda: "ex")
    (rec,) = tracker.step(
        evaluate(LIMITS, samples(true_wind_speed_mps=13.0)),
        S,
        REF,
        None,
        LIMITS,
        ("RESPONSE_NONE",),
    )
    assert rec["responses"] == ["RESPONSE_NONE"] and "responded_at" not in rec
