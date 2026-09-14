"""Great-circle geometry on a sphere, in metres and degrees true.

Same sphere and formulas as Crowsnest's routeSync.js, so a vessel-side snapshot and a station's
fallback computation agree. One deliberate difference: ``along_track_m`` is signed. Crowsnest's
``acos`` form is always positive, which places a vessel that has not yet reached a leg's start
as if it were already on the leg.

Legs are treated as great-circle segments. Loxodrome legs differ by well under a metre over the
leg lengths this processor monitors; revisit if ocean-crossing legs appear.
"""

from __future__ import annotations

import math

EARTH_RADIUS_M = 6371008.8


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = p2 - p1
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial great-circle bearing from point 1 to point 2, in [0, 360)."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def cross_track_m(lat1, lon1, lat2, lon2, lat3, lon3) -> float:
    """Distance of point 3 from the great circle through 1 -> 2. Positive is starboard of the
    direction of travel, negative is port (RouteExecution.xtd_m convention)."""
    d13 = haversine_m(lat1, lon1, lat3, lon3) / EARTH_RADIUS_M
    t13 = math.radians(bearing_deg(lat1, lon1, lat3, lon3))
    t12 = math.radians(bearing_deg(lat1, lon1, lat2, lon2))
    return math.asin(max(-1.0, min(1.0, math.sin(d13) * math.sin(t13 - t12)))) * EARTH_RADIUS_M


def along_track_m(lat1, lon1, lat2, lon2, lat3, lon3) -> float:
    """Signed distance from point 1 along 1 -> 2 to the projection of point 3. Negative when
    point 3 projects behind point 1."""
    d13 = haversine_m(lat1, lon1, lat3, lon3) / EARTH_RADIUS_M
    if d13 == 0.0:
        return 0.0
    xt = cross_track_m(lat1, lon1, lat2, lon2, lat3, lon3) / EARTH_RADIUS_M
    t13 = math.radians(bearing_deg(lat1, lon1, lat3, lon3))
    t12 = math.radians(bearing_deg(lat1, lon1, lat2, lon2))
    magnitude = math.acos(max(-1.0, min(1.0, math.cos(d13) / math.cos(xt)))) * EARTH_RADIUS_M
    return magnitude if math.cos(t13 - t12) >= 0 else -magnitude


def distance_to_segment_m(lat1, lon1, lat2, lon2, lat3, lon3) -> float:
    """Shortest distance from point 3 to the segment 1 -> 2 (not the infinite great circle)."""
    length = haversine_m(lat1, lon1, lat2, lon2)
    along = along_track_m(lat1, lon1, lat2, lon2, lat3, lon3)
    if along <= 0.0 or length == 0.0:
        return haversine_m(lat1, lon1, lat3, lon3)
    if along >= length:
        return haversine_m(lat2, lon2, lat3, lon3)
    return abs(cross_track_m(lat1, lon1, lat2, lon2, lat3, lon3))


def destination(lat: float, lon: float, bearing: float, distance_m: float) -> tuple[float, float]:
    """The point ``distance_m`` from (lat, lon) along initial ``bearing``."""
    p1, l1, t = math.radians(lat), math.radians(lon), math.radians(bearing)
    d = distance_m / EARTH_RADIUS_M
    p2 = math.asin(math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(t))
    l2 = l1 + math.atan2(
        math.sin(t) * math.sin(d) * math.cos(p1), math.cos(d) - math.sin(p1) * math.sin(p2)
    )
    return math.degrees(p2), (math.degrees(l2) + 540.0) % 360.0 - 180.0
