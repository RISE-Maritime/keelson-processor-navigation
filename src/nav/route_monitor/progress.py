"""Progress along a route: which leg, cross-track, distance, bearing and ETAs.

Improves on Crowsnest's computeRouteExecution in four ways that each change a reading:
  - progress is sticky and forward-only, so a route that doubles back does not jump legs;
  - before the first leg the vessel is on leg 0, not the last one;
  - the deviation limit is the one for the side the vessel is on;
  - with no planned speed and no usable SOG there is no arrival ETA, rather than an assumed 10 kn.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..world.route import RoutePlan
from . import geodesy as g

KNOT_MPS = 1852.0 / 3600.0


@dataclass(frozen=True)
class ProgressConfig:
    arrival_radius_m: float = 50.0
    reacquire_margin_m: float = 100.0
    min_sog_for_eta_knots: float = 0.5
    xtd_limit_fallback_m: float = 50.0


@dataclass(frozen=True)
class Progress:
    leg_index: int
    from_waypoint_id: str
    next_waypoint_id: str | None
    arrived: bool
    xtd_m: float
    distance_to_next_m: float
    bearing_to_next_deg: float
    eta_next_s: float | None
    eta_arrival_s: float | None
    deviation_active: bool
    deviation_threshold_m: float
    # The leg's limit on each side, the fallback applied: the side she is moving TOWARD is the one
    # a time to limit is measured to (approach.py), which need not be the side she is on.
    xtd_port_limit_m: float = 0.0
    xtd_stbd_limit_m: float = 0.0


class ProgressTracker:
    """Holds leg state between ticks, by waypoint id so a new edition keeps its place."""

    def __init__(self, cfg: ProgressConfig | None = None):
        self.cfg = cfg or ProgressConfig()
        self._route_id: str | None = None
        self._from_id: str | None = None
        self._arrived = False

    def reset(self) -> None:
        self._route_id, self._from_id, self._arrived = None, None, False

    def update(self, plan: RoutePlan, lat: float, lon: float, sog_knots: float | None) -> Progress:
        wps = plan.waypoints
        last_leg = len(wps) - 2
        if plan.route_id != self._route_id:
            self.reset()
            self._route_id = plan.route_id

        leg = plan.index_of(self._from_id) if self._from_id is not None else None
        if leg is None or leg > last_leg:
            leg = self._acquire(plan, lat, lon)
            self._arrived = False
        elif not self._arrived:
            leg = self._reacquire_forward(plan, leg, lat, lon)

        while not self._arrived:
            capture = self._capture_m(plan, leg + 1)
            a, b = wps[leg], wps[leg + 1]
            length = g.haversine_m(a.lat, a.lon, b.lat, b.lon)
            along = g.along_track_m(a.lat, a.lon, b.lat, b.lon, lat, lon)
            reached = along >= length - capture or g.haversine_m(lat, lon, b.lat, b.lon) <= capture
            if not reached:
                break
            if leg == last_leg:
                self._arrived = True
            else:
                leg += 1
        self._from_id = wps[leg].id

        a, b = wps[leg], wps[leg + 1]
        xtd = g.cross_track_m(a.lat, a.lon, b.lat, b.lon, lat, lon)
        dist = g.haversine_m(lat, lon, b.lat, b.lon)
        brg = g.bearing_deg(lat, lon, b.lat, b.lon)

        limits = plan.leg(leg)
        fallback = self.cfg.xtd_limit_fallback_m
        port_limit = limits.xtd_port_m if limits.xtd_port_m is not None else fallback
        stbd_limit = limits.xtd_starboard_m if limits.xtd_starboard_m is not None else fallback
        threshold = stbd_limit if xtd >= 0 else port_limit

        eta_next = eta_arrival = None
        usable_sog = (
            sog_knots
            if sog_knots is not None and sog_knots >= self.cfg.min_sog_for_eta_knots
            else None
        )
        if not self._arrived and usable_sog is not None:
            eta_next = dist / (usable_sog * KNOT_MPS)
            eta_arrival = eta_next + self._remaining_s(plan, leg + 1, usable_sog)

        return Progress(
            leg_index=leg,
            from_waypoint_id=a.id,
            next_waypoint_id=None if self._arrived else b.id,
            arrived=self._arrived,
            xtd_m=xtd,
            distance_to_next_m=dist,
            bearing_to_next_deg=brg,
            eta_next_s=eta_next,
            eta_arrival_s=eta_arrival,
            deviation_active=(not self._arrived) and abs(xtd) > threshold,
            deviation_threshold_m=threshold,
            xtd_port_limit_m=port_limit,
            xtd_stbd_limit_m=stbd_limit,
        )

    def _capture_m(self, plan: RoutePlan, waypoint_index: int) -> float:
        radius = plan.radius_m(waypoint_index)
        return radius if radius is not None and radius > 0 else self.cfg.arrival_radius_m

    @staticmethod
    def _segment_distance(plan: RoutePlan, i: int, lat: float, lon: float) -> float:
        a, b = plan.waypoints[i], plan.waypoints[i + 1]
        return g.distance_to_segment_m(a.lat, a.lon, b.lat, b.lon, lat, lon)

    def _acquire(self, plan: RoutePlan, lat: float, lon: float) -> int:
        distances = [
            self._segment_distance(plan, i, lat, lon) for i in range(len(plan.waypoints) - 1)
        ]
        return distances.index(min(distances))

    def _reacquire_forward(self, plan: RoutePlan, leg: int, lat: float, lon: float) -> int:
        current = self._segment_distance(plan, leg, lat, lon)
        best, best_d = leg, current
        for i in range(leg + 1, len(plan.waypoints) - 1):
            d = self._segment_distance(plan, i, lat, lon)
            if d + self.cfg.reacquire_margin_m < best_d:
                best, best_d = i, d
        return best

    @staticmethod
    def _remaining_s(plan: RoutePlan, from_index: int, fallback_sog: float) -> float:
        total = 0.0
        wps = plan.waypoints
        for i in range(from_index, len(wps) - 1):
            planned = plan.planned_sog_knots(i)
            sog = planned if planned is not None and planned > 0 else fallback_sog
            total += g.haversine_m(wps[i].lat, wps[i].lon, wps[i + 1].lat, wps[i + 1].lon) / (
                sog * KNOT_MPS
            )
        return total
