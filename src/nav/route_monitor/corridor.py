"""The XTD limits WHERE THE SHIP IS, from a VARIABLE corridor's boundaries (crowsnest-dev#287).

A drawn corridor can be far narrower at the ship than its widest on the leg, or far wider than its
narrowest. Judging the deviation and the approach against the leg's one number made the ANS
and Crowsnest disagree at exactly those places: PASSAGE showed her outside while this monitor was
silent. So the limits are the corridor at the ship's abeam point, the same rule as Crowsnest's
crossTrackPicture.limitsAtShip / crossingAbeam:

  - the ship's projection onto the leg is the origin of a local metre frame along the leg's bearing
    (+y along the track, +x to starboard);
  - each side's boundary polyline is crossed with the abeam line y = 0, and the crossing NEAREST the
    track on that side is the limit there (port: x < 0; starboard: x > 0), to 0.1 m;
  - a side whose line does not reach the abeam line is None, and is filled from the leg's limit,
    side by side (limits_where_ship_is); the leg's from the fallback.

Pure: no clock, no bus. Same sphere as geodesy.py and Crowsnest (6371008.8 m).
"""

from __future__ import annotations

import math

from . import geodesy as g

M_PER_DEG = g.EARTH_RADIUS_M * math.pi / 180.0


def _frame(a_lat: float, a_lon: float, b_lat: float, b_lon: float, lat: float, lon: float):
    """The track-up frame about the ship's projection onto the leg a -> b."""
    along = g.along_track_m(a_lat, a_lon, b_lat, b_lon, lat, lon)
    bearing = g.bearing_deg(a_lat, a_lon, b_lat, b_lon)
    o_lat, o_lon = g.destination(a_lat, a_lon, bearing, along)
    kx = M_PER_DEG * math.cos(math.radians(o_lat))
    brg = math.radians(bearing)
    cos_b, sin_b = math.cos(brg), math.sin(brg)

    def to_local(p_lat: float, p_lon: float) -> tuple[float, float]:
        e = (p_lon - o_lon) * kx
        n = (p_lat - o_lat) * M_PER_DEG
        # Rotate by -bearing: a point along the track lands on +y, one to its right on +x.
        return e * cos_b - n * sin_b, e * sin_b + n * cos_b

    return to_local


def _crossing_abeam(line: list[tuple[float, float]], sign: int) -> float | None:
    best = None
    for (ax, ay), (bx, by) in zip(line, line[1:], strict=False):
        if (ay > 0) == (by > 0) and ay != 0 and by != 0:
            continue  # this segment does not reach the abeam line
        t = 0.0 if ay == by else -ay / (by - ay)
        off = (ax + t * (bx - ax)) * sign
        if off > 0 and (best is None or off < best):
            best = off
    return None if best is None else round(best, 1)


def limits_at_ship(
    a: tuple[float, float],
    b: tuple[float, float],
    lat: float,
    lon: float,
    port_boundary,
    starboard_boundary,
) -> tuple[float | None, float | None]:
    """(port_m, starboard_m) where the boundaries cross the ship's abeam line; None for a side
    whose boundary does not reach it, or has fewer than two vertices."""
    to_local = _frame(a[0], a[1], b[0], b[1], lat, lon)

    def side(points, sign):
        pts = [to_local(p_lat, p_lon) for p_lat, p_lon in points]
        return _crossing_abeam(pts, sign) if len(pts) >= 2 else None

    return side(port_boundary, -1), side(starboard_boundary, 1)


def limits_where_ship_is(
    here: tuple[float | None, float | None], leg: tuple[float, float]
) -> tuple[float, float]:
    """Each side the corridor at the ship, or the leg's limit where it has none."""
    return tuple(h if h is not None else fallback for h, fallback in zip(here, leg, strict=True))
