"""Track keeping: which leg to steer for, the course that returns to it, and the helm order.

Guidance keeps its own leg, by waypoint id, separate from route_monitor's. The monitor reports
progress and switches legs on arrival; guidance must put the wheel over before the waypoint, at the
authored ``Waypoint.wheel_over_distance_m`` or, when none is authored, ``R * tan(delta / 2)`` for a
turn of radius R through the course change delta.

The course is line-of-sight: the leg bearing, turned back towards the track by
``atan(xte / lookahead)``, with xte measured at a position dead-reckoned a few seconds ahead
(keelson-processor-autopilot dead-reckons before its cross-track PID for the same reason). The
intercept is capped, so a vessel far off track returns at a fixed angle instead of a right angle.

The helm order is a PD on heading, damped by measured yaw rate as Slipway's autopilot.py does, so a
leg change's step in setpoint gives no derivative kick. Positive is starboard, as keelson's
``joystick_x_pct`` and Slipway's steering demand both define it. The damping's loop gain is
``yaw_kd_pct_per_degps * turn rate at full helm / 100``: on a hull that turns at its demanded rate
within a tick (Slipway L0, 12 deg/s for alpha) a gain of 1 or more makes the helm alternate hard
over every tick, so the default is small and a hull with real yaw inertia is tuned per platform.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ..route_monitor import geodesy as g
from ..world.route import RoutePlan

KNOT_MPS = 1852.0 / 3600.0


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


@dataclass(frozen=True)
class TrackConfig:
    predict_s: float = 5.0
    lookahead_s: float = 20.0
    min_lookahead_m: float = 100.0
    max_intercept_deg: float = 45.0
    default_turn_radius_m: float = 200.0
    arrival_radius_m: float = 50.0
    reacquire_margin_m: float = 100.0
    heading_kp_pct_per_deg: float = 2.0
    yaw_kd_pct_per_degps: float = 2.0
    max_drift_deg: float = 30.0
    min_sog_for_drift_knots: float = 1.0


@dataclass(frozen=True)
class Own:
    lat: float
    lon: float
    heading_deg: float
    cog_deg: float | None = None
    sog_knots: float | None = None
    yaw_rate_degps: float | None = None


@dataclass(frozen=True)
class TrackState:
    leg_index: int  # index of the waypoint the leg leaves
    from_waypoint_id: str
    to_waypoint_id: str
    leg_bearing_deg: float
    xte_m: float  # at the predicted position; positive is starboard of the track
    distance_to_final_m: float
    arrived: bool


@dataclass(frozen=True)
class HelmOrder:
    course_deg: float
    heading_deg: float
    steering_pct: float


class TrackGuide:
    def __init__(self, cfg: TrackConfig | None = None):
        self.cfg = cfg or TrackConfig()
        self.reset()

    def reset(self) -> None:
        self._route: tuple[str, int] | None = None
        self._to_id: str | None = None
        self._arrived = False

    def follow(self, plan: RoutePlan, own: Own) -> TrackState:
        wps = plan.waypoints
        last = len(wps) - 1
        if (plan.route_id, plan.edition) != self._route:
            keep = self._to_id if self._route and self._route[0] == plan.route_id else None
            self.reset()
            self._route, self._to_id = (plan.route_id, plan.edition), keep

        to = plan.index_of(self._to_id) if self._to_id is not None else None
        if to is None or to < 1:
            to = self._acquire(plan, own) + 1
            self._arrived = False
        elif not self._arrived:
            to = self._reacquire_forward(plan, to - 1, own) + 1

        while not self._arrived:
            a, b = wps[to - 1], wps[to]
            length = g.haversine_m(a.lat, a.lon, b.lat, b.lon)
            along = g.along_track_m(a.lat, a.lon, b.lat, b.lon, own.lat, own.lon)
            if to == last:
                capture = self._capture_m(plan, to)
                if (
                    along >= length - capture
                    or g.haversine_m(own.lat, own.lon, b.lat, b.lon) <= capture
                ):
                    self._arrived = True
                break
            if along >= length - self.wheel_over_m(plan, to):
                to += 1
                continue
            break
        self._to_id = wps[to].id

        a, b = wps[to - 1], wps[to]
        plat, plon = self._predicted(own)
        remaining = sum(
            g.haversine_m(wps[i].lat, wps[i].lon, wps[i + 1].lat, wps[i + 1].lon)
            for i in range(to, last)
        )
        return TrackState(
            leg_index=to - 1,
            from_waypoint_id=a.id,
            to_waypoint_id=b.id,
            leg_bearing_deg=g.bearing_deg(a.lat, a.lon, b.lat, b.lon),
            xte_m=g.cross_track_m(a.lat, a.lon, b.lat, b.lon, plat, plon),
            distance_to_final_m=g.haversine_m(own.lat, own.lon, b.lat, b.lon) + remaining,
            arrived=self._arrived,
        )

    def wheel_over_m(self, plan: RoutePlan, i: int) -> float:
        """Distance before waypoint ``i`` at which guidance turns onto the next leg."""
        wps = plan.waypoints
        a, b, c = wps[i - 1], wps[i], wps[i + 1]
        leg_in = g.haversine_m(a.lat, a.lon, b.lat, b.lon)
        authored = b.wheel_over_distance_m
        if authored is not None and authored >= 0:
            return min(authored, leg_in)
        leg_out_bearing = g.bearing_deg(b.lat, b.lon, c.lat, c.lon)
        leg_in_bearing = g.bearing_deg(a.lat, a.lon, b.lat, b.lon)
        delta = abs(wrap180(leg_out_bearing - leg_in_bearing))
        radius = plan.radius_m(i)
        radius = radius if radius is not None and radius > 0 else self.cfg.default_turn_radius_m
        return min(radius * math.tan(math.radians(min(delta, 170.0)) / 2.0), 0.5 * leg_in)

    def course(self, state: TrackState, own: Own, offset_m: float = 0.0) -> float:
        """Line-of-sight course back onto the leg -- or onto a line parallel to it, ``offset_m``
        off its centreline (+ starboard; keelson#324 Track.cross_track_offset)."""
        sog_mps = (own.sog_knots or 0.0) * KNOT_MPS
        lookahead = max(self.cfg.min_lookahead_m, sog_mps * self.cfg.lookahead_s)
        intercept = math.degrees(math.atan2(state.xte_m - offset_m, lookahead))
        intercept = max(-self.cfg.max_intercept_deg, min(self.cfg.max_intercept_deg, intercept))
        return (state.leg_bearing_deg - intercept) % 360.0

    def rejoin(self, plan: RoutePlan | None, waypoint_id: str | None) -> None:
        """Return to the track after an off-track order. At ``waypoint_id``: steer for it, on the
        leg that arrives there (the first leg for the first waypoint). Without one: forget the leg,
        so the next ``follow`` acquires the nearest, not the one she left the track on."""
        self.reset()
        if plan is None or waypoint_id is None:
            return
        i = plan.index_of(waypoint_id)
        if i is None:
            return
        self._route = (plan.route_id, plan.edition)
        self._to_id = plan.waypoints[max(i, 1)].id

    def helm_heading(self, heading_deg: float, own: Own) -> HelmOrder:
        """The order for a heading, with no drift correction: an ordered heading is where she
        points, and what she makes good is whatever the water leaves her."""
        return self._order(
            own.cog_deg if own.cog_deg is not None else heading_deg, heading_deg, own
        )

    def helm(self, course_deg: float, own: Own) -> HelmOrder:
        """Heading that makes good ``course_deg`` given her present drift, and the order for it."""
        drift = 0.0
        if (
            own.cog_deg is not None
            and own.sog_knots is not None
            and own.sog_knots >= self.cfg.min_sog_for_drift_knots
        ):
            d = wrap180(own.heading_deg - own.cog_deg)
            if abs(d) <= self.cfg.max_drift_deg:
                drift = d
        return self._order(course_deg, (course_deg + drift) % 360.0, own)

    def _order(self, course_deg: float, heading: float, own: Own) -> HelmOrder:
        error = wrap180(heading - own.heading_deg)
        pct = self.cfg.heading_kp_pct_per_deg * error - self.cfg.yaw_kd_pct_per_degps * (
            own.yaw_rate_degps or 0.0
        )
        return HelmOrder(course_deg, heading, max(-100.0, min(100.0, pct)))

    def _predicted(self, own: Own) -> tuple[float, float]:
        if own.cog_deg is None or not own.sog_knots:
            return own.lat, own.lon
        return g.destination(
            own.lat, own.lon, own.cog_deg, own.sog_knots * KNOT_MPS * self.cfg.predict_s
        )

    def _capture_m(self, plan: RoutePlan, i: int) -> float:
        radius = plan.radius_m(i)
        return radius if radius is not None and radius > 0 else self.cfg.arrival_radius_m

    @staticmethod
    def _segment_m(plan: RoutePlan, i: int, own: Own) -> float:
        a, b = plan.waypoints[i], plan.waypoints[i + 1]
        return g.distance_to_segment_m(a.lat, a.lon, b.lat, b.lon, own.lat, own.lon)

    def _acquire(self, plan: RoutePlan, own: Own) -> int:
        distances = [self._segment_m(plan, i, own) for i in range(len(plan.waypoints) - 1)]
        return distances.index(min(distances))

    def _reacquire_forward(self, plan: RoutePlan, leg: int, own: Own) -> int:
        """A later leg takes over only when it is closer by the margin, as route_monitor does, so
        a vessel moved along the route (by an operator, or after a gap) is steered from there."""
        best, best_m = leg, self._segment_m(plan, leg, own)
        for i in range(leg + 1, len(plan.waypoints) - 1):
            d = self._segment_m(plan, i, own)
            if d + self.cfg.reacquire_margin_m < best_m:
                best, best_m = i, d
        return best
