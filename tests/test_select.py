from __future__ import annotations

import pytest

from nav.route_monitor.select import select
from nav.world.model import Declared, WorldSnapshot
from nav.world.route import RoutePlan, RouteRef, VoyagePlan, WaypointPlan
from tests.routes import square_route

pytestmark = pytest.mark.unit

S = 1_000_000_000
NOW = 1_000 * S


def world(voyages=(), editions=(), declared=None):
    return WorldSnapshot(
        voyages={v.voyage_id: v for v in voyages},
        editions={(p.route_id, p.edition): p for p in editions},
        declared=declared or {},
        inputs={},
    )


def voyage(vid="v1", status="VOYAGE_STATUS_ACTIVE", ref=None):
    return VoyagePlan(vid, status, ref or RouteRef("r1", 1))


def test_active_voyage_resolves_its_pinned_edition():
    s = select(world([voyage()], [square_route(edition=1), square_route(edition=2)]), NOW, 35)
    assert s.plan.edition == 1 and s.execution_key_token == "v1"


def test_dangling_edition_is_reported_not_resolved_to_another():
    s = select(world([voyage(ref=RouteRef("r1", 7))], [square_route(edition=2)]), NOW, 35)
    assert s.plan is None and s.dangling == RouteRef("r1", 7)
    assert "not falling back" in s.reason


def test_several_active_voyages_are_refused():
    s = select(world([voyage("a"), voyage("b")], [square_route()]), NOW, 35)
    assert s.plan is None and "several ACTIVE" in s.reason


def test_voyage_wins_over_declared_route():
    declared = {"sim": Declared(square_route(route_id="decl"), NOW, True)}
    s = select(world([voyage()], [square_route()], declared), NOW, 35)
    assert s.plan.route_id == "r1"


def test_paused_voyage_blocks_declared_fallback():
    declared = {"sim": Declared(square_route(route_id="decl"), NOW, True)}
    s = select(world([voyage(status="VOYAGE_STATUS_PAUSED")], [], declared), NOW, 35)
    assert s.plan is None and "PAUSED" in s.reason


def test_live_declared_route_is_monitored_without_a_voyage():
    declared = {"sim": Declared(square_route(route_id="decl"), NOW - 5 * S, True)}
    s = select(world(declared=declared), NOW, 35)
    assert s.plan.route_id == "decl" and s.execution_key_token == "declared-sim"
    assert s.voyage is None


@pytest.mark.parametrize("received, live", [(NOW - 40 * S, True), (NOW, False)])
def test_stale_or_storage_only_declarations_are_ignored(received, live):
    declared = {"sim": Declared(square_route(), received, live)}
    assert select(world(declared=declared), NOW, 35).plan is None


def test_unmonitorable_route_is_refused_with_reason():
    bad = RoutePlan("r1", 1, (WaypointPlan("", 57.0, 11.0), WaypointPlan("x", 57.1, 11.0)))
    s = select(world([voyage()], [bad]), NOW, 35)
    assert s.plan is None and "without an id" in s.reason


def test_completed_voyages_are_ignored():
    declared = {"sim": Declared(square_route(route_id="decl"), NOW, True)}
    s = select(world([voyage(status="VOYAGE_STATUS_COMPLETED")], [], declared), NOW, 35)
    assert s.plan.route_id == "decl"
