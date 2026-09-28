"""The guidance order in force: ``keelson.GuidanceOrder`` (NavigationState.proto, keelson#282,
go_to from keelson#308).

The frame is the field, as in vehicle_navigation's SteeringOrder: an order is track, a course over
ground, a heading, hold or a position to make for, never a number with a separate discriminator.
The same type is given by ``set_guidance_order`` and ``load_route`` and reported in
``navigation_state.guidance_order``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import StrEnum


class OrderError(ValueError):
    """A guidance order that cannot be taken up as given."""


class Kind(StrEnum):
    TRACK = "track"
    COURSE = "course_over_ground_deg"
    HEADING = "heading_deg"
    HOLD = "hold"
    GOTO = "go_to"


@dataclass(frozen=True)
class GuidanceOrder:
    kind: Kind = Kind.TRACK
    value_deg: float | None = None  # COURSE and HEADING only
    resume_from_waypoint_id: str | None = None  # TRACK only
    speed_knots: float | None = None
    position: tuple[float, float] | None = None  # GOTO only: (lat, lon)
    arrival_radius_m: float | None = None  # GOTO only; None = the configured default

    @property
    def steers_off_track(self) -> bool:
        return self.kind in (Kind.COURSE, Kind.HEADING, Kind.GOTO)

    def to_payload(self) -> dict:
        """Protobuf JSON mapping of keelson.GuidanceOrder."""
        if self.kind is Kind.TRACK:
            track = {}
            if self.resume_from_waypoint_id is not None:
                track["resume_from_waypoint_id"] = self.resume_from_waypoint_id
            payload: dict = {"track": track}
        elif self.kind is Kind.HOLD:
            payload = {"hold": {}}
        elif self.kind is Kind.GOTO:
            lat, lon = self.position
            go_to: dict = {"position": {"latitude_deg": lat, "longitude_deg": lon}}
            if self.arrival_radius_m is not None:
                go_to["arrival_radius_m"] = self.arrival_radius_m
            payload = {"go_to": go_to}
        else:
            payload = {self.kind.value: self.value_deg}
        if self.speed_knots is not None:
            payload["speed_knots"] = self.speed_knots
        return payload

    @classmethod
    def from_payload(cls, payload: dict) -> GuidanceOrder:
        """From the protobuf JSON mapping. Raises OrderError for anything that is not exactly one
        frame with a usable value: an order with no frame is almost always a caller that failed to
        set the field it meant to, so it is refused rather than read as TRACK."""
        frames = [k for k in Kind if k.value in payload]
        if len(frames) != 1:
            raise OrderError(
                "a guidance order needs exactly one of track, course, heading, hold, go_to"
            )
        kind = frames[0]
        speed = payload.get("speed_knots")
        if speed is not None:
            speed = float(speed)
            if not math.isfinite(speed) or speed < 0:
                raise OrderError(f"speed_knots must be a finite, non-negative number, not {speed}")
        if kind is Kind.TRACK:
            track = payload["track"] or {}
            resume = track.get("resume_from_waypoint_id")
            return cls(Kind.TRACK, None, resume or None, speed)
        if kind is Kind.HOLD:
            return cls(Kind.HOLD, None, None, speed)
        if kind is Kind.GOTO:
            return cls._go_to(payload["go_to"] or {}, speed)
        value = float(payload[kind.value])
        if not math.isfinite(value) or not 0.0 <= value < 360.0:
            raise OrderError(f"{kind.value} must be in [0, 360), not {value}")
        return cls(kind, value, None, speed)

    @classmethod
    def _go_to(cls, go_to: dict, speed: float | None) -> GuidanceOrder:
        position = go_to.get("position") or {}
        try:
            lat = float(position["latitude_deg"])
            lon = float(position["longitude_deg"])
        except (KeyError, TypeError, ValueError) as exc:
            raise OrderError("go_to needs a position with latitude_deg and longitude_deg") from exc
        if not (math.isfinite(lat) and -90.0 <= lat <= 90.0):
            raise OrderError(f"go_to latitude_deg must be in [-90, 90], not {lat}")
        if not (math.isfinite(lon) and -180.0 <= lon <= 180.0):
            raise OrderError(f"go_to longitude_deg must be in [-180, 180], not {lon}")
        radius = go_to.get("arrival_radius_m")
        if radius is not None:
            radius = float(radius)
            if not math.isfinite(radius) or radius < 0:
                raise OrderError(f"go_to arrival_radius_m must be non-negative, not {radius}")
        return cls(Kind.GOTO, None, None, speed, (lat, lon), radius)

    def describe(self) -> str:
        speed = f" at {self.speed_knots:g} kn" if self.speed_knots is not None else ""
        if self.kind is Kind.TRACK:
            resume = (
                f" from waypoint {self.resume_from_waypoint_id}"
                if self.resume_from_waypoint_id
                else ""
            )
            return f"track{resume}{speed}"
        if self.kind is Kind.HOLD:
            return "hold"
        if self.kind is Kind.GOTO:
            lat, lon = self.position
            within = (
                f" within {self.arrival_radius_m:g} m" if self.arrival_radius_m is not None else ""
            )
            return f"go to {lat:.5f}, {lon:.5f}{within}{speed}"
        frame = "course" if self.kind is Kind.COURSE else "heading"
        return f"{frame} {self.value_deg:05.1f}{speed}"


TRACK = GuidanceOrder()
