"""Route and voyage plans as the monitor reads them: plain frozen data, built by bus/translate.py.

Field-level defaults follow keelson.Route: a waypoint's own value wins, then Waypoints.defaults,
then nothing. ``None`` means "not authored", never zero.
"""

from __future__ import annotations

from dataclasses import dataclass, field, fields


@dataclass(frozen=True)
class LegLimits:
    xtd_port_m: float | None = None
    xtd_starboard_m: float | None = None
    sog_min_knots: float | None = None
    sog_max_knots: float | None = None

    def merged_over(self, default: LegLimits | None) -> LegLimits:
        if default is None:
            return self
        return LegLimits(
            **{
                f.name: getattr(self, f.name)
                if getattr(self, f.name) is not None
                else getattr(default, f.name)
                for f in fields(self)
            }
        )


@dataclass(frozen=True)
class WaypointPlan:
    id: str
    lat: float
    lon: float
    name: str = ""
    planned_sog_knots: float | None = None
    radius_m: float | None = None
    leg: LegLimits | None = None
    wheel_over_distance_m: float | None = None  # Waypoint.wheel_over_distance_m; None: derive it
    operational_context: str | None = None  # keelson.OperationalContext name; None when unspecified


@dataclass(frozen=True)
class RouteLimits:
    """RouteInfo operational limits. Unset means no limit is authored."""

    max_roll_deg: float | None = None
    max_wave_height_m: float | None = None
    max_wind_speed_mps: float | None = None
    min_speed_knots: float | None = None
    max_speed_knots: float | None = None
    min_visibility_m: float | None = None
    max_wind_gust_mps: float | None = None

    def authored(self) -> dict[str, float]:
        return {
            f.name: getattr(self, f.name) for f in fields(self) if getattr(self, f.name) is not None
        }


@dataclass(frozen=True)
class RoutePlan:
    route_id: str
    edition: int
    waypoints: tuple[WaypointPlan, ...]
    default_leg: LegLimits | None = None
    default_planned_sog_knots: float | None = None
    default_radius_m: float | None = None
    limits: RouteLimits = field(default_factory=RouteLimits)

    def problems(self) -> list[str]:
        """Reasons this plan cannot be monitored. Empty when it can."""
        out = []
        if len(self.waypoints) < 2:
            out.append("route has fewer than two waypoints")
        ids = [w.id for w in self.waypoints]
        if any(not i for i in ids):
            out.append("route has a waypoint without an id; progress is addressed by id")
        if len(set(ids)) != len(ids):
            out.append("route has duplicate waypoint ids")
        return out

    def index_of(self, waypoint_id: str) -> int | None:
        for i, w in enumerate(self.waypoints):
            if w.id == waypoint_id:
                return i
        return None

    def planned_sog_knots(self, i: int) -> float | None:
        own = self.waypoints[i].planned_sog_knots
        return own if own is not None else self.default_planned_sog_knots

    def radius_m(self, i: int) -> float | None:
        own = self.waypoints[i].radius_m
        return own if own is not None else self.default_radius_m

    def leg(self, i: int) -> LegLimits:
        own = self.waypoints[i].leg or LegLimits()
        return own.merged_over(self.default_leg)


@dataclass(frozen=True)
class RouteRef:
    route_id: str
    edition: int


@dataclass(frozen=True)
class VoyagePlan:
    voyage_id: str
    status: str  # keelson.VoyageStatus name, e.g. "VOYAGE_STATUS_ACTIVE"
    route_ref: RouteRef | None
