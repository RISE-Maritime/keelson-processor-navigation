from __future__ import annotations

import pytest

from nav.world.latch import Sample
from nav.world.targets import TargetTable

pytestmark = pytest.mark.unit

S = 1_000_000_000


def test_newest_receipt_wins_across_feeds():
    t = TargetTable()
    t.put(
        "mmsi_1", "location_fix", Sample((57.0, 11.0), 10 * S, source_id="maritimedb/digitraffic")
    )
    t.put(
        "mmsi_1",
        "location_fix",
        Sample((57.1, 11.0), 12 * S, source_id="maritimedb/sjofartsverket"),
    )
    t.put(
        "mmsi_1", "location_fix", Sample((57.2, 11.0), 11 * S, source_id="maritimedb/digitraffic")
    )
    view = t.snapshot(12 * S, 60, 600)["mmsi_1"]
    assert view.position.value == (57.1, 11.0)


def test_stale_dynamic_subjects_are_dropped_but_static_ones_kept():
    t = TargetTable()
    t.put("mmsi_1", "name", Sample("FERRY", 0))
    t.put("mmsi_1", "speed_over_ground_knots", Sample(10.0, 0))
    t.put("mmsi_1", "location_fix", Sample((57.0, 11.0), 100 * S))
    view = t.snapshot(100 * S, 60, 600)["mmsi_1"]
    assert view.sog is None and view.name.value == "FERRY"


def test_target_expires_when_its_position_ages_out():
    t = TargetTable()
    t.put("mmsi_1", "location_fix", Sample((57.0, 11.0), 0))
    t.put("mmsi_2", "name", Sample("NO POSITION YET", 0))
    assert set(t.snapshot(10 * S, 60, 600)) == {"mmsi_1"}
    assert (
        t.snapshot(61 * S, 60, 600) == {} and len(t) == 1
    )  # mmsi_2 kept until its static data ages
    assert t.snapshot(601 * S, 60, 600) == {} and len(t) == 0
