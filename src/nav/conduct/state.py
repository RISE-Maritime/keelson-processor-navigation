"""NavigationState: the draft ``keelson.NavigationState`` payload, field for field.

The draft proto is ``docs/upstream/NavigationState.proto``. ``tests/test_state.py`` fails if this
module emits a field the draft does not declare.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import StrEnum


class Mode(StrEnum):
    UNSPECIFIED = "MODE_UNSPECIFIED"
    IDLE = "MODE_IDLE"
    READY = "MODE_READY"
    UNDERWAY = "MODE_UNDERWAY"
    FALLBACK = "MODE_FALLBACK"
    ARRIVED = "MODE_ARRIVED"
    ENDED = "MODE_ENDED"


class SubMode(StrEnum):
    UNSPECIFIED = "SUB_MODE_UNSPECIFIED"
    TRACK = "SUB_MODE_TRACK"
    AVOIDING = "SUB_MODE_AVOIDING"
    HOLDING = "SUB_MODE_HOLDING"
    REROUTING = "SUB_MODE_REROUTING"
    STEERING = "SUB_MODE_STEERING"  # an ordered course, heading or go_to, off the track


# keelson.OperationalAuthority.AuthorityLevel names. UNKNOWN is non-authorising.
AUTHORITY_UNKNOWN = "AUTHORITY_LEVEL_UNKNOWN"


@dataclass(frozen=True)
class RouteRef:
    route_id: str
    route_edition_number: int


@dataclass(frozen=True)
class NavigationState:
    timestamp: str
    mode: Mode
    reason: str
    sub_mode: SubMode = SubMode.UNSPECIFIED
    voyage_id: str | None = None
    route_ref: RouteRef | None = None
    current_leg_from_waypoint_id: str | None = None
    authority_level_in_force: str = AUTHORITY_UNKNOWN
    conn_held: bool = False
    degraded_inputs: tuple[str, ...] = field(default_factory=tuple)
    guidance_order: dict | None = (
        None  # keelson.GuidanceOrder, protobuf JSON; absent with no voyage
    )
    # crowsnest-dev#90 / #94, on a keelson build that carries them (None: not on the wire).
    functions_in_force: tuple[dict, ...] | None = None  # keelson.FunctionAuthority, protobuf JSON
    navigation_authority_revision: int | None = None
    order_id: str | None = None  # the human decision in force; absent when there is none
    ordered_by: str = ""

    def to_payload(self) -> dict:
        """Protobuf JSON mapping. Defaults are emitted too: an explicit MODE_IDLE or
        conn_held=false is a statement, and a reader must not have to know proto3 defaults."""
        payload: dict = {
            "timestamp": self.timestamp,
            "mode": self.mode.value,
            "sub_mode": self.sub_mode.value,
            "authority_level_in_force": self.authority_level_in_force,
            "conn_held": self.conn_held,
            "reason": self.reason,
            "degraded_inputs": list(self.degraded_inputs),
        }
        if self.voyage_id is not None:
            payload["voyage_id"] = self.voyage_id
        if self.route_ref is not None:
            payload["route_ref"] = {
                "route_id": self.route_ref.route_id,
                "route_edition_number": self.route_ref.route_edition_number,
            }
        if self.current_leg_from_waypoint_id is not None:
            payload["current_leg_from_waypoint_id"] = self.current_leg_from_waypoint_id
        if self.guidance_order is not None:
            payload["guidance_order"] = self.guidance_order
        if self.functions_in_force is not None:
            payload["functions_in_force"] = list(self.functions_in_force)
        if self.navigation_authority_revision is not None:
            payload["navigation_authority_revision"] = str(self.navigation_authority_revision)
        if self.order_id is not None:
            payload["order_id"] = self.order_id
            payload["ordered_by"] = self.ordered_by
        return payload
