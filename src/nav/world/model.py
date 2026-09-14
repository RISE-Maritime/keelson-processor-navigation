"""Everything the processor has heard, in one thread-safe place.

Subscriber callbacks write here; ``snapshot()`` hands the tick an immutable copy.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

from .latch import Latch, Sample
from .route import RoutePlan, VoyagePlan
from .targets import TargetTable


@dataclass(frozen=True)
class Declared:
    """A route a vessel says it is sailing (``route/{source_id}``)."""

    plan: RoutePlan
    received_ns: int
    live: bool  # False when it came from router storage and no live heartbeat has arrived yet


@dataclass(frozen=True)
class WorldSnapshot:
    voyages: dict[str, VoyagePlan]
    editions: dict[tuple[str, int], RoutePlan]
    declared: dict[str, Declared]
    inputs: dict[str, Sample]
    rejected: tuple[str, ...] = field(default_factory=tuple)


class World:
    def __init__(self):
        self._lock = threading.Lock()
        self._voyages: dict[str, VoyagePlan] = {}
        self._editions: dict[tuple[str, int], RoutePlan] = {}
        self._declared: dict[str, Declared] = {}
        self._rejected: dict[str, str] = {}
        self._dispositions: list[dict] = []
        self.inputs = Latch()
        self.targets = TargetTable()

    def put_voyage(self, voyage: VoyagePlan) -> None:
        with self._lock:
            self._voyages[voyage.voyage_id] = voyage

    def put_edition(self, plan: RoutePlan) -> None:
        # Editions are immutable (protocol-specification §6.3): the first copy wins.
        with self._lock:
            self._editions.setdefault((plan.route_id, plan.edition), plan)

    def put_declared(self, source_id: str, plan: RoutePlan, received_ns: int, live: bool) -> None:
        with self._lock:
            prev = self._declared.get(source_id)
            if not live and prev is not None and prev.live:
                return  # a storage reply never overrides a live heartbeat
            self._declared[source_id] = Declared(plan, received_ns, live)

    def put_disposition(self, payload: dict) -> None:
        """An advice_disposition from outside, as its payload dict. Drained by the tick."""
        with self._lock:
            self._dispositions.append(payload)

    def take_dispositions(self) -> list[dict]:
        with self._lock:
            out, self._dispositions = self._dispositions, []
        return out

    def reject(self, key: str, reason: str) -> None:
        """Record an input that arrived but could not be used, for navigation_state."""
        with self._lock:
            self._rejected[key] = reason

    def snapshot(self) -> WorldSnapshot:
        with self._lock:
            return WorldSnapshot(
                voyages=dict(self._voyages),
                editions=dict(self._editions),
                declared=dict(self._declared),
                inputs=self.inputs.snapshot(),
                rejected=tuple(f"{k}: {v}" for k, v in sorted(self._rejected.items())),
            )
