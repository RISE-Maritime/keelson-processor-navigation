from __future__ import annotations

import math
from itertools import count

import pytest

from nav.colreg.advisor import SUBJECTS, Advisor, AdvisorConfig, OwnState
from nav.colreg.records import DispositionRecord
from nav.traffic.cpa import Kinematics, geometry
from nav.traffic.monitor import Assessed
from nav.world.latch import Sample
from nav.world.targets import TargetView
from tests.proto_draft import check_payload, parse

pytestmark = pytest.mark.unit

S = 1_000_000_000
NM = 1852.0
LAT, LON = 57.70, 11.90
COASTAL = "OPERATIONAL_CONTEXT_COASTAL"  # RISK 0.5 NM / 8 min, DANGER 0.25 NM / 4 min


def pos(east_nm, north_nm):
    return LAT + north_nm / 60.0, LON + east_nm / (60.0 * math.cos(math.radians(LAT)))


def own(cog=0.0, sog=10.0, heading=None):
    return OwnState(Kinematics(LAT, LON, cog, sog), heading if heading is not None else cog)


def target(
    tid, east_nm, north_nm, cog, sog, own_state, name="BRAVO", nav_status=None, heading=None
):
    lat, lon = pos(east_nm, north_nm)
    k = Kinematics(lat, lon, cog, sog)
    view = TargetView(
        tid,
        Sample((lat, lon), 0, source_id="slipway-traffic/ais"),
        cog=Sample(cog, 0),
        sog=Sample(sog, 0),
        heading=Sample(heading, 0) if heading is not None else None,
        nav_status=Sample(nav_status, 0) if nav_status else None,
        name=Sample(name, 0),
    )
    return {tid: Assessed(view, k, geometry(own_state.kinematics, k, own_state.heading_deg))}


def advisor(**kw):
    ids = count(1)
    return Advisor(AdvisorConfig(**kw), id_factory=lambda: f"id{next(ids)}")


def by(subject, emits):
    return [e for e in emits if e.subject == subject]


def test_crossing_from_starboard_opens_give_way_and_advises_starboard():
    o = own()
    a = advisor()
    out = a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, "voy")
    (enc,) = by("encounter", out)
    p = enc.payload
    # Collision course 6 min out: inside RISK (8 min), not DANGER (4 min).
    assert (p["situation"], p["own_role"], p["risk"]) == (
        "COLREG_SITUATION_CROSSING_STBD",
        "ROLE_GIVE_WAY",
        "RISK_RISK",
    )
    assert (
        enc.source_id == f"voy/{p['encounter_id']}"
        and p["target"]["entity_id"] == "slipway-traffic"
    )
    assert "closed_at" not in p
    (adv,) = by("navigation_advice", out)
    q = adv.payload
    assert q["kind"] == "KIND_COURSE_ALTERATION" and q["encounter_ids"] == [p["encounter_id"]]
    assert 30 <= q["steering"]["course_over_ground_deg"] <= 90  # starboard of 000
    assert q["expected_min_cpa_m"] >= 0.5 * NM
    assert "Not checked against the chart" in q["rationale"]
    assert by("alarm", out) == []  # collision alarm is for DANGER only
    assert check_payload(p, parse("Encounter.proto", "Encounter")) == []
    assert check_payload(q, parse("NavigationAdvice.proto", "NavigationAdvice")) == []


def test_role_is_latched_while_the_geometry_changes():
    o = own()
    a = advisor()
    a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, None)
    # Same target now bearing on the port bow, still closing: still give-way, same encounter.
    out = a.step(S, o, target("mmsi_1", -0.3, 1.0, 100.0, 10.0, o), COASTAL, None)
    (enc,) = by("encounter", out)
    assert enc.payload["own_role"] == "ROLE_GIVE_WAY" and enc.payload["encounter_id"] == "id1"


def test_same_advice_is_restated_with_the_same_id():
    o = own()
    a = advisor(advice_restate_s=5)
    first = by(
        "navigation_advice", a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, None)
    )
    assert (
        by(
            "navigation_advice",
            a.step(S, o, target("mmsi_1", 0.98, 0.98, 270.0, 10.0, o), COASTAL, None),
        )
        == []
    )
    again = by(
        "navigation_advice",
        a.step(6 * S, o, target("mmsi_1", 0.9, 0.9, 270.0, 10.0, o), COASTAL, None),
    )
    assert again and again[0].payload["advice_id"] == first[0].payload["advice_id"]


def test_closing_encounter_resumes_track_and_supersedes_the_avoidance():
    o = own()
    a = advisor()
    a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, None)
    avoid_id = a.active_advice.advice_id
    out = a.step(
        S, o, target("mmsi_1", -1.0, 0.5, 270.0, 10.0, o), COASTAL, None
    )  # passed, opening
    (enc,) = by("encounter", out)
    assert "closed_at" in enc.payload and enc.payload["peak_risk"] == "RISK_RISK"
    (disp,) = by("advice_disposition", out)
    assert (disp.payload["advice_id"], disp.payload["state"]) == (avoid_id, "STATE_SUPERSEDED")
    (resume,) = by("navigation_advice", out)
    assert (
        resume.payload["kind"] == "KIND_RESUME_TRACK"
        and disp.payload["superseded_by"] == resume.payload["advice_id"]
    )
    assert by("alarm", out) == []


def test_stand_on_maintains_then_acts_when_give_way_does_not():
    o = own()
    a = advisor()
    # Crossing from port on a collision course, 6.9 min out: RISK, not yet DANGER.
    out = a.step(0, o, target("mmsi_1", -1.15, 1.15, 90.0, 10.0, o), COASTAL, None)
    (enc,) = by("encounter", out)
    assert (enc.payload["situation"], enc.payload["own_role"]) == (
        "COLREG_SITUATION_CROSSING_PORT",
        "ROLE_STAND_ON",
    )
    (maintain,) = by("navigation_advice", out)
    assert maintain.payload["kind"] == "KIND_MAINTAIN" and "17(a)(i)" in maintain.payload["rules"]
    # She holds on: two minutes out and still on a collision course.
    out = a.step(S, o, target("mmsi_1", -0.35, 0.35, 90.0, 10.0, o), COASTAL, None)
    (act,) = by("navigation_advice", out)
    assert act.payload["kind"] != "KIND_MAINTAIN" and "17(a)(ii)" in act.payload["rules"]
    course = act.payload.get("steering", {}).get("course_over_ground_deg", 0.0)
    assert course == 0.0 or 0 < course <= 180  # never to port for a vessel on the port side


def test_overtaking_may_use_either_side():
    o = own(sog=12)
    a = advisor()
    out = a.step(0, o, target("mmsi_1", 0.0, 0.5, 0.0, 3.0, o), COASTAL, None)
    (enc,) = by("encounter", out)
    assert enc.payload["situation"] == "COLREG_SITUATION_OVERTAKING"
    assert "starboard or port" in by("navigation_advice", out)[0].payload["rationale"]


def test_restricted_visibility_has_no_stand_on_and_says_so():
    o = own()
    a = advisor()
    out = a.step(
        0, o, target("mmsi_1", -1.15, 1.15, 90.0, 10.0, o), COASTAL, None, visibility_m=500.0
    )
    (enc,) = by("encounter", out)
    assert enc.payload["own_role"] == "ROLE_NONE" and enc.payload["rules"] == ["19(d)"]
    assert by("navigation_advice", out)[0].payload["kind"] != "KIND_MAINTAIN"


def test_lost_target_closes_its_encounter():
    o = own()
    a = advisor()
    a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, None)
    (enc,) = by("encounter", a.step(S, o, {}, COASTAL, None))
    assert "closed_at" in enc.payload and "no longer tracked" in enc.payload["reasoning"]


def test_unknown_own_velocity_holds_encounters_and_alarms():
    o = own()
    a = advisor()
    a.step(0, o, target("mmsi_1", 1.0, 1.0, 270.0, 10.0, o), COASTAL, None)
    out = a.step(S, OwnState(Kinematics(LAT, LON, None, None), None), {}, COASTAL, None)
    assert by("encounter", out) == [] and a.open_encounters
    assert "colreg/own_velocity_unknown" in {al.payload["identifier"] for al in by("alarm", out)}


def test_no_risk_no_output():
    o = own()
    assert advisor().step(0, o, target("mmsi_1", 3.0, 3.0, 0.0, 10.0, o), COASTAL, None) == []


def test_advisor_emits_only_advisory_subjects():
    o = own()
    a = advisor()
    emits = []
    for i, (e, n, cog) in enumerate([(1.0, 1.0, 270.0), (0.8, 0.9, 270.0), (-1.0, 0.5, 270.0)]):
        emits += a.step(i * 6 * S, o, target("mmsi_1", e, n, cog, 10.0, o), COASTAL, None)
    a.record_disposition(DispositionRecord("t", "id2", None, "STATE_ACCEPTED", "ted@ROC-1"))
    assert {e.subject for e in emits} <= set(SUBJECTS)
    assert not any(
        k in e.payload for e in emits for k in ("joystick_x_pct", "steering_pct", "rudder")
    )


def test_danger_raises_a_collision_alarm_and_clears_it_on_close():
    o = own()
    a = advisor()
    out = a.step(0, o, target("mmsi_1", 0.5, 0.5, 270.0, 10.0, o), COASTAL, None)  # 3 min out
    (enc,) = by("encounter", out)
    assert enc.payload["risk"] == "RISK_DANGER"
    alarms = {al.payload["identifier"]: al.payload for al in by("alarm", out)}
    collision = alarms[f"colreg/collision_risk/{enc.payload['encounter_id']}"]
    assert collision["category"] == "CATEGORY_SAFETY"
    # 0.7 NM off on a collision course cannot be opened to the 0.5 NM RISK distance: say so loudly.
    assert alarms["colreg/no_clear_manoeuvre"]["priority"] == "PRIORITY_EMERGENCY"
    out = a.step(S, o, target("mmsi_1", -1.0, 0.2, 270.0, 10.0, o), COASTAL, None)  # passed
    cleared = {al.payload["identifier"]: al.payload for al in by("alarm", out)}
    assert set(cleared) == set(alarms)
    assert all(c["expiration_time_utc"] == c["timestamp"] for c in cleared.values())
