"""The deterministic processing loop, free of zenoh.

``Processor.tick(now_ns)`` is the only entry point. It reads a World snapshot, returns the outputs
for that instant and does no I/O, so the same code runs on a wall-clock timer in production and on
Slipway's tick fence in software-in-the-loop (Phase 5).
"""

from __future__ import annotations

from dataclasses import dataclass

from .bus.codec import rfc3339
from .conduct.state import Mode, NavigationState, RouteRef
from .route_monitor.monitor import MODULE as ROUTE_MONITOR
from .route_monitor.monitor import RouteMonitor
from .route_monitor.select import Selection, select
from .traffic.monitor import MODULE as TRAFFIC
from .traffic.monitor import PUBLISHED_SUBJECTS as TRAFFIC_SUBJECTS
from .traffic.monitor import TrafficMonitor
from .world.model import World


@dataclass(frozen=True)
class Output:
    module: str
    subject: str
    payload: dict
    source_id: str | None = None  # None: the module's own source id
    target_id: str | None = None  # set: published under .../@target/{target_id}


class Processor:
    def __init__(self, conf: dict, world: World | None = None, id_factory=None):
        self.conf = conf
        self.world = world or World()
        self._state_period_ns = int(1e9 / conf["state_hz"])
        self._next_state_ns: int | None = None
        self.monitor = RouteMonitor(conf, id_factory) if self.enabled(ROUTE_MONITOR) else None
        self.traffic = TrafficMonitor(conf) if self.enabled(TRAFFIC) else None
        self.selection = Selection(None, "not started")

    def enabled(self, module: str) -> bool:
        return bool(self.conf["modules"].get(module))

    @property
    def needs_routes(self) -> bool:
        return self.enabled(ROUTE_MONITOR) or self.enabled("conduct")

    @property
    def needs_targets(self) -> bool:
        return self.traffic is not None

    def target_publications(self) -> dict[str, tuple[str, ...]]:
        """Subjects each module publishes under ``@target``; they also get a target-scoped token."""
        return {TRAFFIC: TRAFFIC_SUBJECTS} if self.traffic is not None else {}

    def publications(self) -> dict[str, tuple[str, ...]]:
        """Subjects each enabled module publishes under its own source id. Keys whose source slot
        carries an id (route_execution, envelope_exceedance) are reported by ``dynamic_keys``."""
        pubs: dict[str, tuple[str, ...]] = {}
        if self.enabled("conduct"):
            pubs["conduct"] = ("navigation_state",)
        if self.monitor is not None:
            pubs[ROUTE_MONITOR] = ("alarm",) if self.conf["route_monitor"]["alarms"] else ()
        pubs.update(self.target_publications())
        return pubs

    def dynamic_keys(self) -> set[tuple[str, str]]:
        """(subject, source_id) pairs that should hold a subject liveliness token right now."""
        if self.monitor is None or self.monitor.status.progress is None:
            return set()
        return {("route_execution", self.monitor.status.execution_key_token)}

    def tick(self, now_ns: int) -> list[Output]:
        snapshot = self.world.snapshot()
        self.selection = (
            select(snapshot, now_ns, self.conf["route_monitor"]["declared_stale_s"])
            if self.needs_routes
            else Selection(None, "no module needs routes")
        )

        outputs: list[Output] = []
        if self.monitor is not None:
            outputs += [
                Output(ROUTE_MONITOR, e.subject, e.payload, e.source_id)
                for e in self.monitor.step(snapshot, self.selection, now_ns)
            ]
        if self.traffic is not None:
            tr = self.conf["traffic"]
            targets = self.world.targets.snapshot(
                now_ns, tr["target_max_age_s"], tr["static_max_age_s"]
            )
            outputs += [
                Output(TRAFFIC, e.subject, e.payload, e.source_id, e.target_id)
                for e in self.traffic.step(snapshot, targets, now_ns)
            ]
        if self.enabled("conduct") and (
            self._next_state_ns is None or now_ns >= self._next_state_ns
        ):
            outputs.append(Output("conduct", "navigation_state", self._state(now_ns).to_payload()))
            # Anchor to the schedule so jitter does not accumulate; skip missed slots.
            if self._next_state_ns is None:
                self._next_state_ns = now_ns
            while self._next_state_ns <= now_ns:
                self._next_state_ns += self._state_period_ns
        return outputs

    def _state(self, now_ns: int) -> NavigationState:
        sel = self.selection
        if sel.plan is None:
            return NavigationState(timestamp=rfc3339(now_ns), mode=Mode.IDLE, reason=sel.reason)
        status = self.monitor.status if self.monitor is not None else None
        progress = status.progress if status else None
        reason = sel.reason if self.monitor is not None else f"{sel.reason}; route_monitor disabled"
        return NavigationState(
            timestamp=rfc3339(now_ns),
            mode=Mode.READY,
            reason=reason,
            voyage_id=sel.voyage.voyage_id if sel.voyage else None,
            route_ref=RouteRef(sel.plan.route_id, sel.plan.edition),
            current_leg_from_waypoint_id=progress.from_waypoint_id if progress else None,
            degraded_inputs=status.degraded_inputs if status else (),
        )
