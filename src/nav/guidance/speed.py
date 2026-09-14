"""Speed policy: which speed over ground to make, and the throttle for it.

The planned speed of the leg (keelson Route: a waypoint's planned_sog_knots applies to the leg that
leaves it, as route_monitor's ETA reads it), else a configured default. Accepted advice replaces it.
Restricted visibility scales it down (Rule 6/19). Within ``arrival_slowdown_m`` of the final
waypoint it tapers to ``arrival_sog_knots``. The leg's ``sog_max_knots``, the route's
``max_speed_knots`` and the hull's full-ahead speed cap it; ``sog_min_knots`` floors it only while
nothing asked for less.

Throttle is feed-forward on the hull's full-ahead speed plus a proportional correction. No astern:
stopping by reversing the shaft is a manoeuvre, not speed keeping. The correction's loop gain is
``throttle_kp_pct_per_knot * full_ahead_sog_knots / 100``; a hull whose speed follows the throttle
within a tick (Slipway L0) oscillates at a gain of 1 or more, so configuration refuses that.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..world.route import RoutePlan


@dataclass(frozen=True)
class SpeedConfig:
    full_ahead_sog_knots: float
    default_sog_knots: float = 6.0
    arrival_sog_knots: float = 2.0
    arrival_slowdown_m: float = 300.0
    restricted_visibility_factor: float = 0.5
    throttle_kp_pct_per_knot: float = 1.0


def target_sog(
    cfg: SpeedConfig,
    plan: RoutePlan,
    leg_index: int,
    distance_to_final_m: float,
    restricted_visibility: bool,
    advice_sog_knots: float | None,
) -> tuple[float, str]:
    planned = plan.planned_sog_knots(leg_index)
    if advice_sog_knots is not None:
        sog, why = advice_sog_knots, "accepted advice"
    elif planned is not None and planned > 0:
        sog, why = planned, "planned speed of the leg"
    else:
        sog, why = cfg.default_sog_knots, "default speed (no planned speed)"
    slowed = advice_sog_knots is not None
    if restricted_visibility:
        sog *= cfg.restricted_visibility_factor
        why += ", reduced for restricted visibility"
        slowed = True
    if distance_to_final_m < cfg.arrival_slowdown_m:
        taper = cfg.arrival_sog_knots + (sog - cfg.arrival_sog_knots) * max(
            0.0, distance_to_final_m / cfg.arrival_slowdown_m
        )
        if taper < sog:
            sog, why, slowed = taper, why + ", slowing for the final waypoint", True
    limits = plan.leg(leg_index)
    caps = [cfg.full_ahead_sog_knots, limits.sog_max_knots, plan.limits.max_speed_knots]
    cap = min(c for c in caps if c is not None)
    if sog > cap:
        sog, why = cap, why + f", capped at {cap:g} kn"
    if not slowed and limits.sog_min_knots is not None and sog < limits.sog_min_knots:
        sog, why = limits.sog_min_knots, why + f", raised to the leg minimum {sog:g} kn"
    return max(0.0, sog), why


def throttle_pct(cfg: SpeedConfig, target_knots: float, sog_knots: float | None) -> float:
    feed_forward = 100.0 * target_knots / cfg.full_ahead_sog_knots
    correction = (
        cfg.throttle_kp_pct_per_knot * (target_knots - sog_knots) if sog_knots is not None else 0.0
    )
    if target_knots <= 0.0:
        return 0.0
    return max(0.0, min(100.0, feed_forward + correction))
