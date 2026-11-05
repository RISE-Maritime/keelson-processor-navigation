"""One tick of route monitoring: progress, envelope, exceedances and alarms -> outputs."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field

from ..bus.codec import rfc3339
from ..outputs import Emit
from ..world.latch import fresh
from ..world.model import WorldSnapshot
from ..world.route import RouteRef
from .alarms import AlarmManager, AlarmSpec
from .approach import LateralRate, time_to_limit
from .envelope import LIMITS, ExceedanceTracker, evaluate
from .progress import Progress, ProgressConfig, ProgressTracker
from .select import Selection

MODULE = "route_monitor"


@dataclass(frozen=True)
class MonitorStatus:
    execution_key_token: str | None = None
    progress: Progress | None = None
    degraded_inputs: tuple[str, ...] = field(default_factory=tuple)


class RouteMonitor:
    def __init__(self, conf: dict, id_factory=None):
        rm = conf["route_monitor"]
        self.rm = rm
        self.inputs = conf["inputs"]
        self._new_id = id_factory or (lambda: str(uuid.uuid4()))
        self.progress = ProgressTracker(
            ProgressConfig(
                arrival_radius_m=rm["arrival_radius_m"],
                reacquire_margin_m=rm["reacquire_margin_m"],
                min_sog_for_eta_knots=rm["min_sog_for_eta_knots"],
                xtd_limit_fallback_m=rm["xtd_limit_fallback_m"],
            )
        )
        self.alarms = AlarmManager(rm["alarm_republish_s"], rm["alarm_ttl_s"])
        self._trackers: dict[str, ExceedanceTracker] = {}
        self._period_ns = int(1e9 / rm["execution_hz"])
        self._next_exec_ns: int | None = None
        self._token: str | None = None
        self._deviation_latched = False
        self._approach_latched = False
        self._rate = LateralRate()
        self.status = MonitorStatus()

    def step(self, world: WorldSnapshot, selection: Selection, now_ns: int) -> list[Emit]:
        out: list[Emit] = []
        wanted_alarms: dict[str, AlarmSpec] = {}

        if selection.dangling is not None and selection.voyage is not None:
            ref = selection.dangling
            wanted_alarms[f"route_monitor/dangling_route/{selection.voyage.voyage_id}"] = AlarmSpec(
                f"Voyage {selection.voyage.voyage_id} pins route {ref.route_id} edition "
                f"{ref.edition}, which is not available. Route is not monitored.",
                priority="PRIORITY_CAUTION",
            )

        token = selection.execution_key_token
        if token != self._token:
            self._token = token
            self._deviation_latched = False
            self._approach_latched = False
            self._rate.reset()
            self._next_exec_ns = None
            if selection.plan is None:
                self.progress.reset()

        plan = selection.plan
        if plan is None:
            self.status = MonitorStatus()
            out += self._alarm_emits(now_ns, wanted_alarms)
            return out

        position = fresh(world.inputs.get("location_fix"), now_ns, self.rm["max_state_age_s"])
        sog = fresh(world.inputs.get("speed_over_ground_knots"), now_ns, self.rm["max_state_age_s"])
        cog = fresh(world.inputs.get("course_over_ground_deg"), now_ns, self.rm["max_state_age_s"])
        degraded = tuple(
            s
            for s, sample in (("location_fix", position), ("speed_over_ground_knots", sog))
            if sample is None
        )

        if position is None:
            wanted_alarms[f"route_monitor/no_position/{token}"] = AlarmSpec(
                "No fresh own-ship position: route progress is not being monitored",
                priority="PRIORITY_CAUTION",
            )
            self.status = MonitorStatus(token, None, degraded)
            out += self._alarm_emits(now_ns, wanted_alarms)
            return out

        progress = self.progress.update(plan, *position.value, sog.value if sog else None)
        self.status = MonitorStatus(token, progress, degraded)

        if progress.deviation_active:
            self._deviation_latched = True
        elif (
            progress.arrived
            or abs(progress.xtd_m)
            < progress.deviation_threshold_m * self.rm["deviation_clear_fraction"]
        ):
            self._deviation_latched = False
        if self._deviation_latched:
            side = "starboard" if progress.xtd_m >= 0 else "port"
            # Stable text: the manager republishes a changed description at once, so a live
            # distance here would send an alarm every tick.
            wanted_alarms[f"route_monitor/route_deviation/{token}"] = AlarmSpec(
                f"Cross-track deviation to {side} exceeds {progress.deviation_threshold_m:.0f} m "
                f"on the leg from waypoint {progress.from_waypoint_id}"
            )

        # THE EARLY HALF (crowsnest-dev#253): a CAUTION while the limit she is moving toward is
        # under approach_time_s away. It stands down to the WARNING once she is past it, and clears
        # with hysteresis (above approach_clear_factor x the time) so a rate near the line does not
        # flicker it. The rate is per leg: a slope across a leg switch's jump is no rate at all.
        self._rate.add((plan.route_id, progress.leg_index), position.received_ns, progress.xtd_m)
        ttl = time_to_limit(
            progress.xtd_m,
            self._rate.rate_mps(),
            progress.xtd_port_limit_m,
            progress.xtd_stbd_limit_m,
        )
        approach_s = self.rm["approach_time_s"]
        if self._deviation_latched or progress.arrived or ttl is None or ttl <= 0:
            self._approach_latched = False
        elif ttl <= approach_s:
            self._approach_latched = True
        elif ttl > approach_s * self.rm["approach_clear_factor"]:
            self._approach_latched = False
        if self._approach_latched:
            toward_stbd = (self._rate.rate_mps() or 0) > 0
            side = "starboard" if toward_stbd else "port"
            limit = progress.xtd_stbd_limit_m if toward_stbd else progress.xtd_port_limit_m
            # Stable text, as above: no live seconds, or it republishes every tick.
            wanted_alarms[f"route_monitor/route_deviation_approach/{token}"] = AlarmSpec(
                f"Approaching the {side} cross-track limit ({limit:.0f} m): under "
                f"{approach_s:.0f} s at the present rate, on the leg from waypoint "
                f"{progress.from_waypoint_id}",
                priority="PRIORITY_CAUTION",
            )

        fresh_env = {
            d.subject: fresh(world.inputs.get(d.subject), now_ns, self.rm["measurement_max_age_s"])
            for d in LIMITS
        }
        standings = evaluate(plan.limits, fresh_env)
        ref = RouteRef(plan.route_id, plan.edition)

        if selection.voyage is not None:
            voyage_id = selection.voyage.voyage_id
            tracker = self._trackers.get(voyage_id)
            if tracker is None:
                tracker = self._trackers[voyage_id] = ExceedanceTracker(
                    voyage_id, self.rm["envelope_clear_margin"], self._new_id
                )
            responses = ("RESPONSE_ALERTED",) if self.rm["alarms"] else ("RESPONSE_NONE",)
            for record in tracker.step(
                standings, now_ns, ref, progress.from_waypoint_id, plan.limits, responses
            ):
                out.append(
                    Emit("envelope_exceedance", record, f"{voyage_id}/{record['exceedance_id']}")
                )
            for limit_id in sorted(tracker.open_limit_ids):
                standing = next((s for s in standings if s.limit_id == limit_id), None)
                if standing is None:
                    continue
                wanted_alarms[f"route_monitor/envelope/{voyage_id}/{limit_id}"] = AlarmSpec(
                    f"Operational limit {limit_id} breached: limit {standing.limit_value:g}"
                )

        if self._next_exec_ns is None or now_ns >= self._next_exec_ns:
            out.append(
                Emit(
                    "route_execution",
                    self._execution(selection, progress, standings, sog, cog, now_ns),
                    token,
                )
            )
            if self._next_exec_ns is None:
                self._next_exec_ns = now_ns
            while self._next_exec_ns <= now_ns:
                self._next_exec_ns += self._period_ns

        out += self._alarm_emits(now_ns, wanted_alarms)
        return out

    def _alarm_emits(self, now_ns: int, wanted: dict[str, AlarmSpec]) -> list[Emit]:
        if not self.rm["alarms"]:
            return []
        return [Emit("alarm", p) for p in self.alarms.step(now_ns, wanted)]

    @staticmethod
    def _execution(selection, progress: Progress, standings, sog, cog, now_ns) -> dict:
        plan = selection.plan
        payload = {
            "timestamp": rfc3339(now_ns),
            "route_ref": {"route_id": plan.route_id, "route_edition_number": plan.edition},
            "current_leg_from_waypoint_id": progress.from_waypoint_id,
            "distance_to_next_waypoint_m": progress.distance_to_next_m,
            "xtd_m": progress.xtd_m,
            "bearing_to_next_deg": progress.bearing_to_next_deg,
            "deviation_active": progress.deviation_active,
            "deviation_threshold_m": progress.deviation_threshold_m,
            "envelope_limits": [s.to_payload() for s in standings],
        }
        if selection.voyage is not None:
            payload["voyage_id"] = selection.voyage.voyage_id
        if progress.next_waypoint_id is not None:
            payload["next_waypoint_id"] = progress.next_waypoint_id
        if progress.eta_next_s is not None:
            payload["eta_next_waypoint"] = rfc3339(now_ns + int(progress.eta_next_s * 1e9))
        if progress.eta_arrival_s is not None:
            payload["eta_arrival"] = rfc3339(now_ns + int(progress.eta_arrival_s * 1e9))
        # Not optional in the proto: an absent echo reads as 0 on the wire. See TODO.md.
        if sog is not None:
            payload["speed_over_ground_knots"] = sog.value
        if cog is not None:
            payload["course_over_ground_deg"] = cog.value
        return payload
