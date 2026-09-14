from __future__ import annotations

import math

import pytest

from nav.runtime import Processor
from nav.traffic.cpa import KNOT_MPS
from nav.world.latch import Sample
from nav.world.model import World

pytestmark = pytest.mark.unit

S = 1_000_000_000
T0 = 1_800_000_000 * S
NM = 1852.0
LAT, LON = 57.70, 11.90


def offset(east_nm, north_nm):
    return (LAT + north_nm / 60.0, LON + east_nm / (60.0 * math.cos(math.radians(LAT))))


def own(world, t, cog=0.0, sog=10.0, heading=0.0):
    world.inputs.put("location_fix", Sample((LAT, LON), t))
    world.inputs.put("course_over_ground_deg", Sample(cog, t))
    world.inputs.put("speed_over_ground_knots", Sample(sog, t))
    world.inputs.put("heading_true_north_deg", Sample(heading, t))


def target(world, tid, t, east_nm, north_nm, cog=None, sog=None):
    world.targets.put(tid, "location_fix", Sample(offset(east_nm, north_nm), t))
    if cog is not None:
        world.targets.put(tid, "course_over_ground_deg", Sample(cog, t))
    if sog is not None:
        world.targets.put(tid, "speed_over_ground_knots", Sample(sog, t))


def values(outputs):
    return {(o.target_id, o.subject): o.payload["value"] for o in outputs if o.module == "traffic"}


@pytest.fixture
def proc(traffic_conf):
    return Processor(traffic_conf, World())


def test_publishes_defined_values_per_target(proc):
    own(proc.world, T0)
    target(proc.world, "mmsi_1", T0, 0, 2, cog=180, sog=10)  # head-on
    target(proc.world, "mmsi_2", T0, 1, 1, cog=None, sog=8)  # velocity unknown
    v = values(proc.tick(T0))
    assert v[("mmsi_1", "target_cpa_m")] == pytest.approx(0.0, abs=1e-6)
    assert v[("mmsi_1", "target_tcpa_s")] == pytest.approx(2 * NM / (20 * KNOT_MPS))
    assert v[("mmsi_1", "target_bearing_relative_deg")] == pytest.approx(0.0)
    assert v[("mmsi_2", "target_bearing_north_deg")] == pytest.approx(45.0, abs=0.01)
    assert ("mmsi_2", "target_cpa_m") not in v and ("mmsi_2", "target_tcpa_s") not in v
    assert proc.traffic.status.degraded_inputs == ()


def test_one_publication_per_period(proc):
    own(proc.world, T0)
    target(proc.world, "mmsi_1", T0, 0, 2, cog=180, sog=10)
    assert values(proc.tick(T0))
    assert values(proc.tick(T0 + S // 10)) == {}
    own(proc.world, T0 + S)
    assert values(proc.tick(T0 + S))


def test_range_limit_with_exit_hysteresis(proc):
    limit_nm = proc.conf["traffic"]["range_limit_m"] / NM  # 12
    own(proc.world, T0)
    target(proc.world, "mmsi_far", T0, 0, limit_nm + 0.5, cog=0, sog=0)
    assert values(proc.tick(T0)) == {}
    target(proc.world, "mmsi_far", T0 + S, 0, limit_nm - 0.5, cog=0, sog=0)
    own(proc.world, T0 + S)
    assert values(proc.tick(T0 + S))
    target(proc.world, "mmsi_far", T0 + 2 * S, 0, limit_nm + 0.5, cog=0, sog=0)  # inside 1.1x
    own(proc.world, T0 + 2 * S)
    assert values(proc.tick(T0 + 2 * S))
    target(proc.world, "mmsi_far", T0 + 3 * S, 0, limit_nm * 1.2, cog=0, sog=0)
    own(proc.world, T0 + 3 * S)
    assert values(proc.tick(T0 + 3 * S)) == {}


def test_no_own_position_publishes_nothing(proc):
    target(proc.world, "mmsi_1", T0, 0, 2, cog=180, sog=10)
    assert values(proc.tick(T0)) == {}
    assert proc.traffic.status.degraded_inputs == ("location_fix",)


def test_unknown_own_velocity_keeps_bearing_but_no_cpa(proc):
    proc.world.inputs.put("location_fix", Sample((LAT, LON), T0))
    target(proc.world, "mmsi_1", T0, 1, 0, cog=270, sog=10)
    v = values(proc.tick(T0))
    assert set(v) == {("mmsi_1", "target_bearing_north_deg")}


def test_excluded_targets_are_skipped(traffic_conf):
    traffic_conf["traffic"]["exclude_targets"] = ["mmsi_own"]
    proc = Processor(traffic_conf, World())
    own(proc.world, T0)
    target(proc.world, "mmsi_own", T0, 0, 0.001, cog=0, sog=10)
    assert values(proc.tick(T0)) == {}


def test_dead_reckoning_moves_old_reports(traffic_conf):
    own_t = T0 + 60 * S
    for dr in (True, False):
        traffic_conf["traffic"]["dead_reckon"] = dr
        proc = Processor(traffic_conf, World())
        own(proc.world, own_t, sog=0.0)
        target(proc.world, "mmsi_1", T0, 0, 1, cog=90, sog=10)  # reported 60 s ago
        brg = values(proc.tick(own_t))[("mmsi_1", "target_bearing_north_deg")]
        east_m = 10 * KNOT_MPS * 60 if dr else 0.0
        assert brg == pytest.approx(math.degrees(math.atan2(east_m, NM)), abs=0.05)


def test_target_liveliness_publications(proc):
    assert "target_cpa_m" in proc.target_publications()["traffic"]
    assert proc.publications()["traffic"] == proc.target_publications()["traffic"]
