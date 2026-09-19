"""Guidance: the selected route, or accepted advice, into helm and throttle orders.

Acts only while conduct says UNDERWAY. Leaving UNDERWAY for any reason starts the failsafe: neutral
axes for ``neutral_burst_s``, then silence. Every order is also stated as a ``navigation_setpoint``
record at ``setpoint_hz``.
"""

from __future__ import annotations

from dataclasses import fields

from ..bus.codec import rfc3339
from ..control.orders import TRACK, GuidanceOrder, Kind
from ..outputs import Emit
from .axes import FREE, OURS, Actuation
from .failsafe import Failsafe
from .speed import SpeedConfig, target_sog, throttle_pct
from .track import KNOT_MPS, HelmOrder, Own, TrackConfig, TrackGuide, TrackState

MODULE = "guidance"
BACKEND = "BACKEND_VEHICLE_CONTROL_AXES"
SUBJECTS = ("joystick_x_pct", "joystick_y_pct", "navigation_setpoint")


def track_config(conf: dict) -> TrackConfig:
    gd = conf["guidance"]
    values = {f.name: gd[f.name] for f in fields(TrackConfig) if f.name in gd}
    values["arrival_radius_m"] = conf["route_monitor"]["arrival_radius_m"]
    values["reacquire_margin_m"] = conf["route_monitor"]["reacquire_margin_m"]
    return TrackConfig(**values)


def speed_config(conf: dict) -> SpeedConfig:
    gd = conf["guidance"]
    return SpeedConfig(**{f.name: gd[f.name] for f in fields(SpeedConfig)})


class Guidance:
    def __init__(self, conf: dict):
        gd = conf["guidance"]
        self.track = TrackGuide(track_config(conf))
        self.speed = speed_config(conf)
        self.failsafe = Failsafe(gd["neutral_burst_s"])
        self._hold_ns = int(gd["command_hold_s"] * 1e9)
        self._setpoint_period_ns = int(1e9 / gd["setpoint_hz"])
        self._next_setpoint_ns: int | None = None
        self._driving = False
        self._released = False
        self.state: TrackState | None = None
        self.actuation = Actuation()

    def take_order(self, order: GuidanceOrder, previous: GuidanceOrder, plan) -> None:
        """A new order is in force. Returning to the track re-acquires it: at the named waypoint, or
        the nearest leg, never the leg she left it on."""
        if order.kind is Kind.TRACK and (
            previous.kind is not Kind.TRACK or order.resume_from_waypoint_id is not None
        ):
            self.track.rejoin(plan, order.resume_from_waypoint_id)

    def observe(self, plan, own: Own | None) -> TrackState | None:
        """Where the vessel is along the route, for conduct's arrival gate. Keeps the leg across a
        transient loss of inputs rather than re-acquiring."""
        self.state = self.track.follow(plan, own) if plan is not None and own is not None else None
        return self.state

    def step(
        self,
        now_ns: int,
        *,
        driving: bool,
        ended: bool,
        helm: str,
        plan,
        own: Own | None,
        advice,
        restricted_visibility: bool,
        voyage_id: str | None,
        route_ref: dict | None,
        order: GuidanceOrder = TRACK,
    ) -> list[Emit]:
        if driving and plan is not None and own is not None and self.state is not None:
            if not self._driving:
                self._driving, self._released, self._next_setpoint_ns = True, False, None
            avoiding = advice is not None
            install = "install" if helm == FREE else None
            if order.kind is Kind.HOLD and not avoiding:
                # Neutral helm and no throttle, published continuously: an ordered hold is still
                # this process driving, not silence (silence is the failsafe's).
                self.actuation = Actuation((0.0, 0.0), now_ns + self._hold_ns, install)
                return self._setpoint(
                    now_ns,
                    "PURPOSE_HOLDING",
                    voyage_id,
                    route_ref,
                    None,
                    0.0,
                    0.0,
                    "ordered hold: neutral helm, no throttle",
                    None,
                )
            heading_frame = False
            if avoiding and advice.course_deg is not None:
                helm_order = self.track.helm(advice.course_deg, own)
            elif order.kind is Kind.COURSE and not avoiding:
                helm_order = self.track.helm(order.value_deg, own)
            elif order.kind is Kind.HEADING and not avoiding:
                helm_order, heading_frame = self.track.helm_heading(order.value_deg, own), True
            else:
                helm_order = self.track.helm(self.track.course(self.state, own), own)
            if avoiding:
                sog, why = target_sog(
                    self.speed,
                    plan,
                    self.state.leg_index,
                    self.state.distance_to_final_m,
                    restricted_visibility,
                    advice.sog_knots,
                )
            elif order.speed_knots is not None:
                sog, why = order.speed_knots, f"ordered speed {order.speed_knots:g} kn"
            elif order.steers_off_track and own.sog_knots is not None:
                sog, why = (
                    own.sog_knots,
                    "holding the speed she has under a course or heading order",
                )
            else:
                sog, why = target_sog(
                    self.speed,
                    plan,
                    self.state.leg_index,
                    self.state.distance_to_final_m,
                    restricted_visibility,
                    None,
                )
            throttle = throttle_pct(self.speed, sog, own.sog_knots)
            self.actuation = Actuation(
                (helm_order.steering_pct, throttle), now_ns + self._hold_ns, install
            )
            if avoiding:
                purpose = "PURPOSE_AVOIDING"
            elif order.steers_off_track:
                purpose = "PURPOSE_STEERING"
            else:
                purpose = "PURPOSE_TRACK"
            return self._setpoint(
                now_ns,
                purpose,
                voyage_id,
                route_ref,
                helm_order,
                sog,
                throttle,
                why,
                advice,
                heading_frame,
            )

        if self._driving:
            self._driving = False
            self._next_setpoint_ns = None
            self.failsafe.start(now_ns)
        if self.failsafe.active(now_ns):
            self.actuation = Actuation(
                (0.0, 0.0), min(now_ns + self._hold_ns, self.failsafe.until_ns)
            )
            return self._setpoint(
                now_ns,
                "PURPOSE_FAILSAFE_NEUTRAL",
                voyage_id,
                route_ref,
                None,
                0.0,
                0.0,
                "neutral axes before going silent",
                None,
            )
        mapping = None
        if ended and helm == OURS and not self._released:
            mapping, self._released = "release", True
        self.actuation = Actuation(None, 0, mapping)
        return []

    def _setpoint(
        self,
        now_ns: int,
        purpose: str,
        voyage_id,
        route_ref,
        order: HelmOrder | None,
        sog_knots: float,
        throttle: float,
        why: str,
        advice,
        heading_frame: bool = False,
    ) -> list[Emit]:
        if self._next_setpoint_ns is not None and now_ns < self._next_setpoint_ns:
            return []
        if self._next_setpoint_ns is None:
            self._next_setpoint_ns = now_ns
        while self._next_setpoint_ns <= now_ns:
            self._next_setpoint_ns += self._setpoint_period_ns
        actuation: dict = {
            "steering_pct": order.steering_pct if order else 0.0,
            "throttle_pct": throttle,
        }
        payload: dict = {
            "timestamp": rfc3339(now_ns),
            "purpose": purpose,
            "backend": BACKEND,
            "propulsion": {"sog_mps": sog_knots * KNOT_MPS},
            "actuation": actuation,
            "reason": why,
        }
        if order is not None:
            payload["steering"] = (
                {"heading_deg": order.heading_deg}
                if heading_frame
                else {"course_over_ground_deg": order.course_deg}
            )
            actuation["heading_deg"] = order.heading_deg
        if voyage_id is not None:
            payload["voyage_id"] = voyage_id
        if route_ref is not None:
            payload["route_ref"] = route_ref
        if self.state is not None:
            payload["from_waypoint_id"] = self.state.from_waypoint_id
            payload["to_waypoint_id"] = self.state.to_waypoint_id
            payload["cross_track_error_m"] = self.state.xte_m
        if advice is not None:
            payload["source_advice_id"] = advice.advice_id
        return [Emit("navigation_setpoint", payload)]
