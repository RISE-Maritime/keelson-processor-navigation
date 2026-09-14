from __future__ import annotations

import pytest

from nav.log.decision_log import DecisionLog
from tests.proto_draft import check_payload, parse

pytestmark = pytest.mark.unit

S = 1_000_000_000


def log(conf, snapshot_s=30.0):
    conf["decision_log"]["snapshot_s"] = snapshot_s
    ids = iter(range(1000))
    return DecisionLog(conf, id_factory=lambda: f"d{next(ids)}")


def test_a_decision_fits_the_draft_proto(conf):
    d = log(conf).record(
        0,
        "KIND_MODE_TRANSITION",
        "navigation/0/conduct",
        "MODE_READY -> MODE_UNDERWAY",
        reason="conn held",
        voyage_id="v1",
        mode_before="MODE_READY",
        mode_after="MODE_UNDERWAY",
        ownship={"latitude_deg": 57.7, "longitude_deg": 11.9, "sog_mps": 3.0},
        advice_id="a1",
        encounter_ids=("e1",),
        conn_held=True,
    )
    msg = parse("NavigationDecision.proto", "NavigationDecision")
    assert check_payload(d.to_payload(), msg) == []
    assert d.key_tail == "v1/d0"


def test_published_once_then_restated_with_the_same_id(conf):
    dl = log(conf, snapshot_s=30.0)
    dl.record(0, "KIND_CONN_TAKEN", "navigation/0/conduct", "taken")
    (first,) = dl.step(0)
    assert first.payload["restated"] is False and first.source_id == "novoyage/d0"
    assert dl.step(10 * S) == []
    (again,) = dl.step(30 * S)
    assert again.payload["restated"] is True
    assert again.payload["decision_id"] == first.payload["decision_id"]
