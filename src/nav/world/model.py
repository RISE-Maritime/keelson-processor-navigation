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
    # The full Voyage record (protobuf JSON) by id, kept so a republish drops nothing.
    voyage_docs: dict[str, dict] = field(default_factory=dict)


class World:
    def __init__(self):
        self._lock = threading.Lock()
        self._voyages: dict[str, VoyagePlan] = {}
        self._voyage_docs: dict[str, dict] = {}
        self._commands: list = []
        self._editions: dict[tuple[str, int], RoutePlan] = {}
        self._declared: dict[str, Declared] = {}
        self._rejected: dict[str, str] = {}
        self._dispositions: list[dict] = []
        self._leases: list[tuple[dict, int]] = []
        self._requests: list[tuple[dict, int]] = []
        self._helm: tuple[dict | None, int] | None = None
        self._helm_events: list[tuple[str, str]] = []
        self._gate = None  # (GateMap | None, received_ns): the motion gate's map, as last read
        self._gate_epoch = None  # (term, generation) on the gate's latest resolved_control
        self.inputs = Latch()
        self.targets = TargetTable()

    def put_voyage(self, voyage: VoyagePlan, doc: dict | None = None) -> None:
        """``doc`` is the whole Voyage as protobuf JSON: navigation_control republishes a voyage
        it transitions, and must not drop the fields it does not read (names, ports, policy)."""
        with self._lock:
            self._voyages[voyage.voyage_id] = voyage
            if doc is not None:
                self._voyage_docs[voyage.voyage_id] = doc

    def put_command(self, pending) -> None:
        """A navigation_control/v1 call waiting for the tick (control/commands.Pending)."""
        with self._lock:
            self._commands.append(pending)

    def take_commands(self) -> list:
        with self._lock:
            out, self._commands = self._commands, []
        return out

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

    def put_lease(self, payload: dict, received_ns: int, slot: str | None = None) -> None:
        """A command_authority message, as its payload dict, stamped with local receipt time. `slot`
        is the key after the subject (`{vessel}`, `{vessel}/navigation`, ...); None is the conn."""
        with self._lock:
            self._leases.append((payload, received_ns, slot))

    def take_leases(self) -> list[tuple[dict, int, str | None]]:
        with self._lock:
            out, self._leases = self._leases, []
        return out

    def put_request(self, payload: dict, received_ns: int) -> None:
        """A command_request message, as its payload dict, stamped with local receipt time."""
        with self._lock:
            self._requests.append((payload, received_ns))

    def take_requests(self) -> list[tuple[dict, int]]:
        with self._lock:
            out, self._requests = self._requests, []
        return out

    def put_helm(self, mapping: dict | None, received_ns: int) -> None:
        """The vehicle_control mapping as last read (protobuf JSON); None when nobody answered."""
        with self._lock:
            self._helm = (mapping, received_ns)

    def helm(self) -> tuple[dict | None, int] | None:
        with self._lock:
            return self._helm

    def put_gate(self, gate_map, received_ns: int) -> None:
        """The motion gate's map in force (guidance/candidates.GateMap); None: nobody answered."""
        with self._lock:
            self._gate = (gate_map, received_ns) if gate_map is not None else None

    def gate(self):
        with self._lock:
            return self._gate

    def put_gate_epoch(self, epoch: tuple[str, int] | None) -> None:
        with self._lock:
            self._gate_epoch = epoch

    def gate_epoch(self) -> tuple[str, int] | None:
        with self._lock:
            return self._gate_epoch

    def put_helm_event(self, kind: str, detail: str) -> None:
        with self._lock:
            self._helm_events.append((kind, detail))

    def take_helm_events(self) -> list[tuple[str, str]]:
        with self._lock:
            out, self._helm_events = self._helm_events, []
        return out

    def reject(self, key: str, reason: str) -> None:
        """Record an input that arrived but could not be used, for navigation_state."""
        with self._lock:
            self._rejected[key] = reason

    def snapshot(self) -> WorldSnapshot:
        with self._lock:
            return WorldSnapshot(
                voyages=dict(self._voyages),
                voyage_docs=dict(self._voyage_docs),
                editions=dict(self._editions),
                declared=dict(self._declared),
                inputs=self.inputs.snapshot(),
                rejected=tuple(f"{k}: {v}" for k, v in sorted(self._rejected.items())),
            )
