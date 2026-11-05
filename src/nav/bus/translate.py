"""keelson protobuf messages -> the processor's plain plan and sample types.

The only place above the codec that touches protobuf objects on the way in. Optional proto3
fields are read with presence: an unset limit is not a zero limit.
"""

from __future__ import annotations

import math

from ..world.latch import Sample
from ..world.route import LegLimits, RouteLimits, RoutePlan, RouteRef, VoyagePlan, WaypointPlan

_LEG_FIELDS = {
    "xtd_port_m": "xtd_port_m",
    "xtd_starboard_m": "xtd_starboard_m",
    "sog_min_knots": "sog_min_knots",
    "sog_max_knots": "sog_max_knots",
}


class TranslateError(ValueError):
    """A message that decodes but cannot be used."""


def _opt(msg, name):
    return getattr(msg, name) if msg.HasField(name) else None


def _leg(leg_msg) -> LegLimits:
    return LegLimits(**{ours: _opt(leg_msg, theirs) for ours, theirs in _LEG_FIELDS.items()})


def _boundary(points) -> tuple[tuple[float, float], ...]:
    """An XTD boundary's WGS-84 vertices; one without a usable position is dropped."""
    out = []
    for p in points:
        if not p.HasField("position"):
            continue
        lat, lon = p.position.latitude_deg, p.position.longitude_deg
        if math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180:
            out.append((lat, lon))
    return tuple(out)


def _variable(leg_msg) -> bool:
    try:
        name = (
            type(leg_msg)
            .DESCRIPTOR.fields_by_name["xtd_corridor_mode"]
            .enum_type.values_by_number[leg_msg.xtd_corridor_mode]
            .name
        )
    except (KeyError, AttributeError):
        return False
    return name == "XTD_CORRIDOR_MODE_VARIABLE"


def route_plan(route) -> RoutePlan:
    wps = []
    for w in route.waypoints.waypoint:
        if not w.HasField("position"):
            raise TranslateError(f"waypoint {w.id!r} has no position")
        lat, lon = w.position.latitude_deg, w.position.longitude_deg
        if not (
            math.isfinite(lat) and math.isfinite(lon) and -90 <= lat <= 90 and -180 <= lon <= 180
        ):
            raise TranslateError(f"waypoint {w.id!r} has an invalid position")
        wps.append(
            WaypointPlan(
                id=w.id,
                lat=lat,
                lon=lon,
                name=w.name,
                planned_sog_knots=_opt(w, "planned_sog_knots"),
                radius_m=_opt(w, "radius_m"),
                wheel_over_distance_m=_opt(w, "wheel_over_distance_m"),
                leg=_leg(w.leg) if w.HasField("leg") else None,
                operational_context=_enum_name(w, "operational_context"),
                xtd_variable=w.HasField("leg") and _variable(w.leg),
                xtd_port_boundary=_boundary(w.leg.xtd_port_boundary) if w.HasField("leg") else (),
                xtd_starboard_boundary=(
                    _boundary(w.leg.xtd_starboard_boundary) if w.HasField("leg") else ()
                ),
            )
        )
    defaults = route.waypoints.defaults
    info = route.info
    return RoutePlan(
        route_id=route.route_id,
        edition=route.route_edition_number,
        waypoints=tuple(wps),
        default_leg=_leg(defaults.leg) if defaults.HasField("leg") else None,
        default_planned_sog_knots=_opt(defaults, "planned_sog_knots"),
        default_radius_m=_opt(defaults, "radius_m"),
        limits=RouteLimits(**{name: _opt(info, name) for name in RouteLimits.__dataclass_fields__}),
    )


def _enum_name(msg, field_name: str) -> str | None:
    number = getattr(msg, field_name)
    if not number:
        return None
    value = msg.DESCRIPTOR.fields_by_name[field_name].enum_type.values_by_number.get(number)
    return value.name if value else None


def voyage_plan(voyage) -> VoyagePlan:
    status = voyage.DESCRIPTOR.fields_by_name["status"].enum_type.values_by_number.get(
        voyage.status
    )
    ref = None
    if voyage.HasField("route_ref"):
        ref = RouteRef(voyage.route_ref.route_id, voyage.route_ref.route_edition_number)
    return VoyagePlan(voyage.voyage_id, status.name if status else "VOYAGE_STATUS_UNSPECIFIED", ref)


def _measured_ns(msg) -> int | None:
    return msg.timestamp.ToNanoseconds() if msg.HasField("timestamp") else None


def sample(msg, received_ns: int, source_id: str) -> Sample:
    """A measured input as a Sample.

    LocationFix -> (lat, lon); VesselNavStatus -> enum name; Timestamped* -> value.
    """
    name = msg.DESCRIPTOR.full_name
    fields = msg.DESCRIPTOR.fields_by_name
    if name == "foxglove.LocationFix":
        if not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            raise TranslateError("location fix is not finite")
        value = (msg.latitude, msg.longitude)
    elif name == "keelson.VesselNavStatus":
        value = fields["navigation_status"].enum_type.values_by_number[msg.navigation_status].name
    elif "value" in fields and fields["value"].type == fields["value"].TYPE_STRING:
        value = msg.value
    elif "value" in fields:
        value = float(msg.value)
        if not math.isfinite(value):
            raise TranslateError(f"{name} value is not finite")
    else:
        raise TranslateError(f"no sample translation for {name}")
    return Sample(value, received_ns, _measured_ns(msg), source_id)
