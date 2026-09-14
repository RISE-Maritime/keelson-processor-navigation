"""The decision log: an append-only record of what conduct and guidance decided, and why.

Each decision is published once when taken. Every ``snapshot_s`` the most recent
``snapshot_count`` are restated with ``restated = true`` and their original ids, so a station that
joins late sees the recent history without a router query (docs/PLAN.md §1, WarrantRecord pattern).
"""

from __future__ import annotations

import uuid
from collections import deque
from dataclasses import dataclass, field

from ..bus.codec import rfc3339
from ..outputs import Emit

MODULE = "decision_log"
POLICY_ID = "keelson-processor-navigation/conduct/v1"
SUBJECTS = ("navigation_decision",)


@dataclass(frozen=True)
class Decision:
    timestamp: str
    decision_id: str
    kind: str
    decided_by: str
    summary: str
    reason: str = ""
    voyage_id: str | None = None
    mode_before: str | None = None
    mode_after: str | None = None
    ownship: dict | None = None
    advice_id: str | None = None
    encounter_ids: tuple[str, ...] = field(default_factory=tuple)
    authority_level_in_force: str = "AUTHORITY_LEVEL_UNKNOWN"
    conn_held: bool = False

    def to_payload(self, restated: bool = False) -> dict:
        out: dict = {
            "timestamp": self.timestamp,
            "decision_id": self.decision_id,
            "kind": self.kind,
            "decided_by": self.decided_by,
            "summary": self.summary,
            "reason": self.reason,
            "encounter_ids": list(self.encounter_ids),
            "authority_level_in_force": self.authority_level_in_force,
            "conn_held": self.conn_held,
            "policy_id": POLICY_ID,
            "restated": restated,
        }
        for name in ("voyage_id", "mode_before", "mode_after", "ownship", "advice_id"):
            value = getattr(self, name)
            if value is not None:
                out[name] = value
        return out

    @property
    def key_tail(self) -> str:
        return f"{self.voyage_id or 'novoyage'}/{self.decision_id}"


class DecisionLog:
    def __init__(self, conf: dict, id_factory=None):
        dl = conf["decision_log"]
        self._new_id = id_factory or (lambda: str(uuid.uuid4()))
        self._period_ns = int(dl["snapshot_s"] * 1e9)
        self._recent: deque[Decision] = deque(maxlen=int(dl["snapshot_count"]))
        self._pending: list[Decision] = []
        self._next_snapshot_ns: int | None = None

    def record(self, now_ns: int, kind: str, decided_by: str, summary: str, **fields) -> Decision:
        d = Decision(rfc3339(now_ns), self._new_id(), kind, decided_by, summary, **fields)
        self._pending.append(d)
        return d

    def step(self, now_ns: int) -> list[Emit]:
        out = [Emit("navigation_decision", d.to_payload(), d.key_tail) for d in self._pending]
        fresh_ids = {d.decision_id for d in self._pending}
        self._recent.extend(self._pending)
        self._pending = []
        if self._next_snapshot_ns is None:
            self._next_snapshot_ns = now_ns + self._period_ns
        elif now_ns >= self._next_snapshot_ns:
            out += [
                Emit("navigation_decision", d.to_payload(restated=True), d.key_tail)
                for d in self._recent
                if d.decision_id not in fresh_ids
            ]
            while self._next_snapshot_ns <= now_ns:
                self._next_snapshot_ns += self._period_ns
        return out
