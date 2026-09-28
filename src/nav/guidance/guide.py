"""Guidance: the selected route, or accepted advice, into helm and throttle orders.

Acts only while conduct says UNDERWAY. Leaving UNDERWAY for any reason starts the failsafe: neutral
axes for ``neutral_burst_s``, then silence. Every order is also stated as a ``navigation_setpoint``
record at ``setpoint_hz``.
"""

from __future__ import annotations

from dataclasses import fields, replace
from enum import StrEnum

from ..bus.codec import rfc3339
from ..control.orders import TRACK, GuidanceOrder, Kind
from ..outputs import Emit
from ..route_monitor import geodesy as g
from .axes import FREE, OURS, Actuation
from .failsafe import Failsafe
from .speed import SpeedConfig, target_sog, throttle_pct
from .track import KNOT_MPS, HelmOrder, Own, TrackConfig, TrackGuide, TrackState

MODULE = "guidance"
BACKEND = "BACKEND_VEHICLE_CONTROL_AXES"
# navigation_setpoint is this process's full record (a raw_json rider). The two *_setpoint_*
# subjects (keelson#316) state the same target in the form any display can read. There is no
# speed_through_water_setpoint_knots: the speed this process orders is over ground.
SUBJECTS = (
    "joystick_x_pct",
    "joystick_y_pct",
    "navigation_setpoint",
    "heading_setpoint_deg",
    "course_over_ground_setpoint_deg",
)

# A corridor-share offset stays inside the corridor, never on its edge (keelson#324).
OFFSET_CAP = 0.9

# An arrived go_to holds with neutral axes, which only drifts. She makes for the position again
# once she has drifted out past this multiple of the arrival radius, so she does not hunt at its
# edge.
REAPPROACH_FACTOR = 2.0


class LoiterPhase(StrEnum):
    TRANSIT = "transit"  # making for the position
    DRIFTING = "drifting"  # neutral, with wind and stream
    RETURNING = "returning"  # carried past the radius, making back


def loiter_phase(phase: LoiterPhase, distance_m: float, radius_m: float, near_m: float):
    """keelson#320's loiter: neutral once near the position, back past ``radius_m``, neutral again
    once near. ``near_m`` is the go_to arrival radius, never more than half the loiter radius, so
    she is not sent back the moment she goes neutral."""
    near = min(near_m, 0.5 * radius_m)
    if phase is LoiterPhase.DRIFTING:
        return LoiterPhase.RETURNING if distance_m > radius_m else phase
    return LoiterPhase.DRIFTING if distance_m <= near else phase


def go_to_arrived(was_arrived: bool, distance_m: float, radius_m: float) -> bool:
    """Arrival at a go_to position, with hysteresis: in at ``radius_m``, out past
    ``REAPPROACH_FACTOR`` times it."""
    if was_arrived:
        return distance_m <= REAPPROACH_FACTOR * radius_m
    return distance_m <= radius_m


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
        self._release_pending = False
        self._goto_radius_m = float(gd["goto_arrival_radius_m"])
        self._xtd_fallback_m = float(conf["route_monitor"]["xtd_limit_fallback_m"])
        self._loiter_return_knots = float(gd["loiter_return_knots"])
        self.goto_arrived = False
        self.loiter = LoiterPhase.TRANSIT
        self.state: TrackState | None = None
        self.actuation = Actuation()

    def take_order(self, order: GuidanceOrder, previous: GuidanceOrder, plan) -> None:
        """A new order is in force. Returning to the track re-acquires it: at the named waypoint, or
        the nearest leg, never the leg she left it on."""
        self.goto_arrived = False
        self.loiter = LoiterPhase.TRANSIT
        if order.kind is Kind.TRACK and (
            previous.kind is not Kind.TRACK
            or order.resume_from_waypoint_id is not None
            or order.reverse != previous.reverse  # the other direction: re-acquire on it
        ):
            self.track.rejoin(plan, order.resume_from_waypoint_id)

    @property
    def goto_radius_default_m(self) -> float:
        return self._goto_radius_m

    @property
    def loiter_return_default_knots(self) -> float:
        return self._loiter_return_knots

    def track_offset_m(self, order: GuidanceOrder, plan) -> float:
        """The lateral offset to sail the current leg at, metres (+ starboard). A fraction is
        resolved against the leg's corridor on that side and kept inside it (keelson#324)."""
        if order.kind is not Kind.TRACK:
            return 0.0
        if order.offset_m:
            return float(order.offset_m)
        f = order.offset_fraction
        if not f or self.state is None or plan is None:
            return 0.0
        leg = plan.leg(self.state.leg_index)
        side = leg.xtd_starboard_m if f > 0 else leg.xtd_port_m
        half = side if side is not None and side > 0 else self._xtd_fallback_m
        return max(-OFFSET_CAP, min(OFFSET_CAP, f)) * half

    def set_defaults(self, *, default_sog_knots=None, goto_radius_m=None, loiter_return_knots=None):
        """set_settings (keelson#322): the defaults an order that names none falls back to."""
        if default_sog_knots is not None:
            self.speed = replace(self.speed, default_sog_knots=float(default_sog_knots))
        if goto_radius_m is not None:
            self._goto_radius_m = float(goto_radius_m)
        if loiter_return_knots is not None:
            self._loiter_return_knots = float(loiter_return_knots)

    def release_helm(self) -> None:
        """The conn was handed to an operator: once guidance has gone silent, release the axis
        mapping so their helm is free, as at the end of a voyage."""
        self._release_pending = True

    def observe(self, plan, own: Own | None) -> TrackState | None:
        """Where the vessel is along the route, for conduct's arrival gate. Keeps the leg across a
        transient loss of inputs rather than re-acquiring."""
        self.state = self.track.follow(plan, own) if plan is not None and own is not None else None
        return self.state

    def goto_radius_m(self, order: GuidanceOrder) -> float:
        r = order.arrival_radius_m
        return r if r is not None else self._goto_radius_m

    def station(self, order: GuidanceOrder, own: Own | None) -> bool:
        """Whether a point order is holding on its point -- a go_to arrived (keelson#308), a loiter
        drifting (keelson#320) -- for conduct's sub-mode and for the helm. Kept across a transient
        loss of own-ship."""
        if order.kind not in (Kind.GOTO, Kind.LOITER):
            self.goto_arrived, self.loiter = False, LoiterPhase.TRANSIT
            return False
        if own is not None:
            lat, lon = order.position
            d = g.haversine_m(own.lat, own.lon, lat, lon)
            if order.kind is Kind.GOTO:
                self.goto_arrived = go_to_arrived(self.goto_arrived, d, self.goto_radius_m(order))
            else:
                self.loiter = loiter_phase(self.loiter, d, order.radius_m, self._goto_radius_m)
        if order.kind is Kind.GOTO:
            return self.goto_arrived
        return self.loiter is LoiterPhase.DRIFTING

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
                self._release_pending = False
            avoiding = advice is not None
            install = "install" if helm == FREE else None
            arrived = order.kind is Kind.GOTO and self.goto_arrived
            drifting = order.kind is Kind.LOITER and self.loiter is LoiterPhase.DRIFTING
            if (order.kind is Kind.HOLD or arrived or drifting) and not avoiding:
                # Neutral helm and no throttle, published continuously: an ordered hold is still
                # this process driving, not silence (silence is the failsafe's).
                self.actuation = Actuation((0.0, 0.0), now_ns + self._hold_ns, install)
                if arrived:
                    why = (
                        f"arrived within {self.goto_radius_m(order):g} m of the go_to position: "
                        "neutral helm, no throttle"
                    )
                elif drifting:
                    why = f"loiter: drifting inside {order.radius_m:g} m, neutral helm, no throttle"
                else:
                    why = "ordered hold: neutral helm, no throttle"
                return self._setpoint(
                    now_ns, "PURPOSE_HOLDING", voyage_id, route_ref, None, 0.0, 0.0, why, None
                )
            heading_frame = False
            if avoiding and advice.course_deg is not None:
                helm_order = self.track.helm(advice.course_deg, own)
            elif order.kind in (Kind.GOTO, Kind.LOITER) and not avoiding:
                lat, lon = order.position
                helm_order = self.track.helm(g.bearing_deg(own.lat, own.lon, lat, lon), own)
            elif order.kind is Kind.COURSE and not avoiding:
                helm_order = self.track.helm(order.value_deg, own)
            elif order.kind is Kind.HEADING and not avoiding:
                helm_order, heading_frame = self.track.helm_heading(order.value_deg, own), True
            else:
                offset = self.track_offset_m(order, plan)
                helm_order = self.track.helm(self.track.course(self.state, own, offset), own)
            if avoiding:
                sog, why = target_sog(
                    self.speed,
                    plan,
                    self.state.leg_index,
                    self.state.distance_to_final_m,
                    restricted_visibility,
                    advice.sog_knots,
                )
            elif order.kind is Kind.LOITER and self.loiter is LoiterPhase.RETURNING:
                # Not the passage speed: keelson#320 makes back at return_speed_knots.
                back = order.return_speed_knots or self._loiter_return_knots
                sog, why = back, f"loiter: carried past {order.radius_m:g} m, back at {back:g} kn"
            elif order.speed_knots is not None:
                sog, why = order.speed_knots, f"ordered speed {order.speed_knots:g} kn"
            elif (
                order.kind in (Kind.GOTO, Kind.LOITER)
                and (own.sog_knots or 0.0) < self.speed.arrival_sog_knots
            ):
                # keelson#308 says the speed she has; a go_to is often given from a hold, where
                # that is nothing and she would never leave. A loiter's passage likewise.
                sog, why = (
                    self.speed.default_sog_knots,
                    f"{order.kind.value} from a standstill: the default speed",
                )
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
                if order.speed_factor is not None and order.kind is Kind.TRACK:
                    sog = sog * order.speed_factor
                    why = f"{why}, trimmed {(order.speed_factor - 1) * 100:+g} % (keelson#324)"
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
        if (ended or self._release_pending) and helm == OURS and not self._released:
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
        out = [Emit("navigation_setpoint", payload)]
        if order is not None:
            # keelson#316: published by whoever holds the setpoint, absent when none is. The helm
            # always steers to a heading; a course is the setpoint too unless the frame is heading.
            stamp = payload["timestamp"]
            out.append(
                Emit("heading_setpoint_deg", {"timestamp": stamp, "value": order.heading_deg})
            )
            if not heading_frame:
                out.append(
                    Emit(
                        "course_over_ground_setpoint_deg",
                        {"timestamp": stamp, "value": order.course_deg},
                    )
                )
        return out
