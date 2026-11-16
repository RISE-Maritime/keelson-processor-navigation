from __future__ import annotations

import pytest

from nav.colreg.records import AdviceRecord, DispositionRecord, EncounterRecord, TargetRef
from tests.proto_draft import check_payload, parse

pytestmark = pytest.mark.unit

TS = "2026-09-14T12:00:00Z"


def encounter(**kw):
    base = dict(
        timestamp=TS,
        encounter_id="e1",
        voyage_id="v1",
        target=TargetRef("sim-ais-traffic", "ais", "mmsi_999001001"),
        target_name="NORDIC TRADER",
        range_m=900.0,
        bearing_true_deg=45.0,
        bearing_relative_deg=45.0,
        cpa_m=50.0,
        tcpa_s=120.0,
        situation="COLREG_SITUATION_CROSSING_STBD",
        own_role="ROLE_GIVE_WAY",
        rules=("15", "16"),
        reasoning="target crossing from starboard",
        risk="RISK_DANGER",
        peak_risk="RISK_DANGER",
        opened_at=TS,
        closed_at=TS,
        threshold_context="OPERATIONAL_CONTEXT_COASTAL",
    )
    base.update(kw)
    return EncounterRecord(**base)


def advice(**kw):
    base = dict(
        timestamp=TS,
        advice_id="a1",
        voyage_id=None,
        kind="KIND_COURSE_AND_SPEED",
        encounter_ids=("e1",),
        course_deg=40.0,
        sog_knots=5.0,
        from_course_deg=0.0,
        from_sog_knots=10.0,
        valid_from=TS,
        valid_to=TS,
        expected_min_cpa_m=600.0,
        expected_min_range_m=600.0,
        rules=("15", "16", "8"),
        rationale="alter 40 degrees to starboard",
        policy_id="nav/colreg/v1",
        policy_config_digest="AAAA",
    )
    base.update(kw)
    return AdviceRecord(**base)


def test_encounter_payload_matches_the_draft():
    assert check_payload(encounter().to_payload(), parse("Encounter.proto", "Encounter")) == []


def test_advice_payload_matches_the_draft():
    assert (
        check_payload(advice().to_payload(), parse("NavigationAdvice.proto", "NavigationAdvice"))
        == []
    )
    stop = advice(kind="KIND_STOP", course_deg=None, sog_knots=0.0, expected_min_cpa_m=float("inf"))
    payload = stop.to_payload()
    assert "steering" not in payload and payload["propulsion"] == {"sog_mps": 0.0}
    assert "expected_min_cpa_m" not in payload


def test_disposition_payload_matches_the_draft_and_round_trips():
    d = DispositionRecord(
        TS, "a1", "v1", "STATE_SUPERSEDED", "navigation/0/colreg", superseded_by="a2"
    )
    assert (
        check_payload(d.to_payload(), parse("AdviceDisposition.proto", "AdviceDisposition")) == []
    )
    assert DispositionRecord.from_payload(d.to_payload()) == d


def test_checker_catches_drift():
    bad = encounter().to_payload() | {"own_role": "ROLE_KEEP_CLEAR", "surprise": 1}
    problems = check_payload(bad, parse("Encounter.proto", "Encounter"))
    assert "Encounter.surprise" in problems and "Encounter.own_role=ROLE_KEEP_CLEAR" in problems
