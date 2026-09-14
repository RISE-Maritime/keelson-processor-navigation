"""CPA, TCPA, bow crossing range and time, bearing and range between own ship and a target.

Same local flat-earth frame as Crowsnest's cpaCalculation.js (60 NM per degree of latitude,
longitude scaled by cos of own latitude), in metres and seconds instead of nautical miles and
minutes, so a vessel-side value and a station's display can be compared directly. Accurate to well
under 1 % inside the 12 NM this is used for.

A velocity that was not reported is unknown, never zero: a target with speed but no course is not
heading 000°, and one with no speed is not stopped. A target known to be stopped has a defined CPA
whatever its course.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

KNOT_MPS = 1852.0 / 3600.0
M_PER_DEG_LAT = 60.0 * 1852.0
_REL_SPEED_EPS_SQ = (0.01 * KNOT_MPS) ** 2


@dataclass(frozen=True)
class Kinematics:
    lat: float
    lon: float
    cog_deg: float | None = None
    sog_knots: float | None = None

    @property
    def velocity_known(self) -> bool:
        if self.sog_knots is None:
            return False
        return self.sog_knots == 0.0 or self.cog_deg is not None

    def velocity_mps(self) -> tuple[float, float]:
        """(east, north). Only meaningful when velocity_known."""
        if not self.sog_knots:
            return 0.0, 0.0
        v = self.sog_knots * KNOT_MPS
        c = math.radians(self.cog_deg)
        return v * math.sin(c), v * math.cos(c)


@dataclass(frozen=True)
class Geometry:
    range_m: float
    bearing_deg: float
    cpa_m: float | None = None
    tcpa_s: float | None = None  # negative: closest approach is in the past, range opening
    bcr_m: float | None = None
    bct_s: float | None = None


def offset_m(own_lat: float, own_lon: float, lat: float, lon: float) -> tuple[float, float]:
    """(east, north) of a point relative to own ship in the local frame."""
    return (
        (lon - own_lon) * M_PER_DEG_LAT * math.cos(math.radians(own_lat)),
        (lat - own_lat) * M_PER_DEG_LAT,
    )


def dead_reckon(t: Kinematics, dt_s: float) -> Kinematics:
    """The target moved along its reported velocity for ``dt_s``. Unknown velocity: unmoved."""
    if dt_s <= 0 or not t.velocity_known or not t.sog_knots:
        return t
    ve, vn = t.velocity_mps()
    lat = t.lat + vn * dt_s / M_PER_DEG_LAT
    lon = t.lon + ve * dt_s / (M_PER_DEG_LAT * math.cos(math.radians(t.lat)))
    return Kinematics(lat, lon, t.cog_deg, t.sog_knots)


def geometry(own: Kinematics, target: Kinematics, own_heading_deg: float | None = None) -> Geometry:
    dx, dy = offset_m(own.lat, own.lon, target.lat, target.lon)
    rng = math.hypot(dx, dy)
    bearing = (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0
    if not (own.velocity_known and target.velocity_known):
        return Geometry(rng, bearing)

    oe, on = own.velocity_mps()
    te, tn = target.velocity_mps()
    rvx, rvy = te - oe, tn - on
    rel_sq = rvx * rvx + rvy * rvy
    if rel_sq < _REL_SPEED_EPS_SQ:
        # No relative motion: the range never changes, so it is the CPA, and there is no time to it.
        return Geometry(rng, bearing, cpa_m=rng)

    tcpa = -(dx * rvx + dy * rvy) / rel_sq
    cpa = math.hypot(dx + rvx * tcpa, dy + rvy * tcpa)

    bcr = bct = None
    line = (
        own_heading_deg if own_heading_deg is not None else (own.cog_deg if own.sog_knots else None)
    )
    if line is not None:
        hx, hy = math.sin(math.radians(line)), math.cos(math.radians(line))
        closing_across = hx * rvy - hy * rvx
        if abs(closing_across) > 1e-9:
            t = -(hx * dy - hy * dx) / closing_across
            ahead = hx * (dx + rvx * t) + hy * (dy + rvy * t)
            # Zero counts as ahead: a crossing at own position is a collision, the case that
            # most needs reporting. The tolerance absorbs rounding on that exact case.
            if t > 0 and ahead >= -1e-6:
                bcr, bct = max(ahead, 0.0), t
    return Geometry(rng, bearing, cpa, tcpa, bcr, bct)


def relative_bearing_deg(bearing_deg: float, heading_deg: float) -> float:
    """Bearing relative to the bow, in [0, 360)."""
    return (bearing_deg - heading_deg) % 360.0
