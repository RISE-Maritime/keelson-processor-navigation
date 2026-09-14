"""Phase 3 acceptance: Slipway's rivo-crossing, replayed through the whole processor.

The fixture is the Slipway kernel run in-process (tests/fixtures/rivo_crossing.jsonl, one row per
second): Alpha on 079.9 at 11 kn, Bravo on 024.8 at 6 kn on Alpha's starboard bow, background
contact TARGET 1. Recorded from the kernel as it sails today, which is NOT what the scenario's
comments describe (a 100 m pass at 277 s); Slipway's own test_rivo_crossing fails the same way. The
recorded geometry is still a clean Rule 15 crossing: closest approach 394 m at t=420 s.

Alpha is own ship. Bravo and TARGET 1 are fed as targets, as an AIS receiver aboard Alpha would see
them. Nobody steers: this is the advisory phase, so the advice keeps coming until the pass.
"""

from __future__ import annotations

import copy
import json
from itertools import count
from pathlib import Path

import pytest

from nav import config as config_mod
from nav.colreg.advisor import SUBJECTS
from nav.runtime import Processor
from nav.world.latch import Sample
from nav.world.model import World

pytestmark = pytest.mark.unit

FIXTURE = Path(__file__).resolve().parent / "fixtures" / "rivo_crossing.jsonl"
S = 1_000_000_000
T0 = 1_800_000_000 * S
RISK_CPA_M = 0.5 * 1852.0  # coastal
DANGER_CPA_M = 0.25 * 1852.0


@pytest.fixture(scope="module")
def run():
    conf = copy.deepcopy(config_mod.DEFAULTS)
    conf["entity_id"] = "nav-test-alpha"
    conf["modules"].update(traffic=True, colreg=True, conduct=False)
    conf["inputs"] = {
        "location_fix": "gnss",
        "course_over_ground_deg": "sim",
        "speed_over_ground_knots": "sim",
        "heading_true_north_deg": "sim",
    }
    conf["traffic"]["feeds"] = [{"entity": "slipway-traffic", "source": "ais"}]
    ids = count(1)
    proc = Processor(conf, World(), id_factory=lambda: f"id{next(ids)}")

    outputs = []
    for line in FIXTURE.read_text().splitlines():
        row = json.loads(line)
        now = T0 + int(row["t_s"] * S)
        a = row["vessels"]["slipway-alpha"]
        w = proc.world
        w.inputs.put("location_fix", Sample((a["lat"], a["lon"]), now))
        w.inputs.put("course_over_ground_deg", Sample(a["heading_deg"], now))  # L0, no current
        w.inputs.put("speed_over_ground_knots", Sample(a["sog_knots"], now))
        w.inputs.put("heading_true_north_deg", Sample(a["heading_deg"], now))
        targets = {"mmsi_bravo": (row["vessels"]["slipway-bravo"], "VESSEL BRAVO")}
        targets |= {tid: (t, "TARGET 1") for tid, t in row["targets"].items()}
        for tid, (t, name) in targets.items():
            src = "slipway-traffic/ais"
            w.targets.put(tid, "location_fix", Sample((t["lat"], t["lon"]), now, source_id=src))
            w.targets.put(
                tid, "course_over_ground_deg", Sample(t["heading_deg"], now, source_id=src)
            )
            w.targets.put(
                tid, "speed_over_ground_knots", Sample(t["sog_knots"], now, source_id=src)
            )
            w.targets.put(
                tid, "heading_true_north_deg", Sample(t["heading_deg"], now, source_id=src)
            )
            w.targets.put(tid, "name", Sample(name, now, source_id=src))
        outputs += [(row["t_s"], o) for o in proc.tick(now)]
    return outputs


def colreg(outputs, subject):
    return [(t, o.payload) for t, o in outputs if o.module == "colreg" and o.subject == subject]


def test_one_crossing_from_starboard_with_alpha_give_way(run):
    bravo = [
        (t, p) for t, p in colreg(run, "encounter") if p["target"]["target_id"] == "mmsi_bravo"
    ]
    assert bravo, "no encounter with Bravo"
    assert {p["encounter_id"] for _, p in bravo} == {bravo[0][1]["encounter_id"]}
    assert {(p["situation"], p["own_role"]) for _, p in bravo} == {
        ("COLREG_SITUATION_CROSSING_STBD", "ROLE_GIVE_WAY")
    }
    assert "15" in bravo[0][1]["rules"]


def test_encounter_reaches_danger_and_closes_after_the_pass(run):
    bravo = [
        (t, p) for t, p in colreg(run, "encounter") if p["target"]["target_id"] == "mmsi_bravo"
    ]
    opened_t = bravo[0][0]
    closes = [(t, p) for t, p in bravo if "closed_at" in p]
    assert opened_t < 60, "a 395 m CPA 7 min out is inside coastal RISK from the start"
    assert len(closes) == 1 and closes[0][1]["peak_risk"] == "RISK_DANGER"
    assert 415 <= closes[0][0] <= 440, f"closed at {closes[0][0]} s; closest approach is at 420 s"


def test_first_advice_is_a_starboard_alteration_that_clears_bravo(run):
    from nav.traffic.cpa import Kinematics, geometry

    advice = colreg(run, "navigation_advice")
    t, first = advice[0]
    assert first["kind"] == "KIND_COURSE_ALTERATION"
    course = first["steering"]["course_over_ground_deg"]
    alteration = (course - first["from_course_over_ground_deg"]) % 360
    assert 30 <= alteration <= 90, f"alteration {alteration:.0f} is not a starboard turn of 30-90"
    assert "NO MANOEUVRE CLEARS" not in first["rationale"]
    assert "Not checked against the chart" in first["rationale"]
    # Recomputed independently from the fixture at the advice's own second: Bravo is cleared to
    # the RISK distance, and TARGET 1 (a bystander on the starboard quarter) stays outside DANGER.
    row = next(json.loads(x) for x in FIXTURE.read_text().splitlines() if json.loads(x)["t_s"] == t)
    a = row["vessels"]["slipway-alpha"]
    own_after = Kinematics(a["lat"], a["lon"], course, a["sog_knots"])
    b = row["vessels"]["slipway-bravo"]
    bravo = geometry(own_after, Kinematics(b["lat"], b["lon"], b["heading_deg"], b["sog_knots"]))
    assert bravo.cpa_m >= RISK_CPA_M
    for tgt in row["targets"].values():
        other = geometry(
            own_after, Kinematics(tgt["lat"], tgt["lon"], tgt["heading_deg"], tgt["sog_knots"])
        )
        assert (other.cpa_m if other.tcpa_s and other.tcpa_s > 0 else other.range_m) >= DANGER_CPA_M
    assert first["expected_min_cpa_m"] >= DANGER_CPA_M


def test_resume_track_follows_the_close_and_supersedes_the_avoidance(run):
    advice = colreg(run, "navigation_advice")
    resumes = [(t, p) for t, p in advice if p["kind"] == "KIND_RESUME_TRACK"]
    assert resumes and resumes[0][0] >= 415
    superseded = [
        p for _, p in colreg(run, "advice_disposition") if p["state"] == "STATE_SUPERSEDED"
    ]
    assert any(p["superseded_by"] == resumes[0][1]["advice_id"] for p in superseded)


def test_collision_alarm_raised_during_danger_and_cleared(run):
    alarms = [
        p for _, p in colreg(run, "alarm") if p["identifier"].startswith("colreg/collision_risk/")
    ]
    assert alarms and alarms[-1]["expiration_time_utc"] == alarms[-1]["timestamp"]


def test_colreg_publishes_only_advisory_subjects(run):
    assert {o.subject for _, o in run if o.module == "colreg"} <= set(SUBJECTS)
