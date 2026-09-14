"""Route builders shared by tests."""

from __future__ import annotations

from nav.route_monitor import geodesy as g
from nav.world.route import LegLimits, RouteLimits, RoutePlan, WaypointPlan

ORIGIN = (57.70, 11.90)


def square_route(
    leg_m: float = 1000.0,
    route_id: str = "r1",
    edition: int = 1,
    sog: float | None = 10.0,
    radius: float | None = 30.0,
    leg: LegLimits | None = None,
    limits: RouteLimits | None = None,
) -> RoutePlan:
    """WP1 -> north -> east -> south: three legs of ``leg_m``."""
    p0 = ORIGIN
    p1 = g.destination(*p0, 0.0, leg_m)
    p2 = g.destination(*p1, 90.0, leg_m)
    p3 = g.destination(*p2, 180.0, leg_m)
    wps = tuple(
        WaypointPlan(
            id=f"wp{i}", lat=p[0], lon=p[1], planned_sog_knots=sog, radius_m=radius, leg=leg
        )
        for i, p in enumerate((p0, p1, p2, p3))
    )
    return RoutePlan(route_id, edition, wps, limits=limits or RouteLimits())
