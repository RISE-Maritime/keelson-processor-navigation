"""keelson.Alarm lifecycle: raise, keep alive, clear.

keelson.Alarm has no active flag. Consumers (Crowsnest's alert list) key an alarm by entity and
``identifier`` and drop it once ``expiration_time_utc`` has passed. So an active alarm is
republished every ``republish_s`` with an expiry ``ttl_s`` ahead, which also makes the alarm of a
crashed publisher disappear on its own, and a cleared alarm is published once already expired.

``ack_scheme`` is ACK_NONE: this processor serves no alarm_ack/v1, so it must not ask for an
acknowledgement it cannot record.
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

from ..bus.codec import rfc3339


@dataclass(frozen=True)
class AlarmSpec:
    description: str
    priority: str = "PRIORITY_WARNING"
    severity: str = "SEVERITY_LEVEL_MEDIUM"
    category: str = "CATEGORY_NAVIGATION"


@dataclass
class _Active:
    spec: AlarmSpec
    activated_ns: int
    last_published_ns: int


class AlarmManager:
    def __init__(self, republish_s: float = 5.0, ttl_s: float = 15.0):
        if ttl_s <= republish_s:
            raise ValueError("ttl_s must exceed republish_s, or an active alarm flickers out")
        self.republish_ns = int(republish_s * 1e9)
        self.ttl_ns = int(ttl_s * 1e9)
        self._active: dict[str, _Active] = {}

    @property
    def active_identifiers(self) -> set[str]:
        return set(self._active)

    def step(self, now_ns: int, wanted: Mapping[str, AlarmSpec]) -> list[dict]:
        out = []
        for identifier, spec in sorted(wanted.items()):
            current = self._active.get(identifier)
            if current is None:
                current = self._active[identifier] = _Active(spec, now_ns, now_ns)
            elif current.spec == spec and now_ns - current.last_published_ns < self.republish_ns:
                continue
            current.spec = spec
            current.last_published_ns = now_ns
            out.append(self._payload(identifier, current, now_ns, now_ns + self.ttl_ns))
        for identifier in sorted(set(self._active) - set(wanted)):
            current = self._active.pop(identifier)
            out.append(self._payload(identifier, current, now_ns, now_ns, cleared=True))
        return out

    def clear_all(self, now_ns: int) -> list[dict]:
        return self.step(now_ns, {})

    @staticmethod
    def _payload(identifier, active: _Active, now_ns, expires_ns, cleared=False) -> dict:
        spec = active.spec
        return {
            "timestamp": rfc3339(now_ns),
            "identifier": identifier,
            "description": f"{spec.description} (cleared)" if cleared else spec.description,
            "category": spec.category,
            "priority": spec.priority,
            "severity": spec.severity,
            "activation_time_utc": rfc3339(active.activated_ns),
            "expiration_time_utc": rfc3339(expires_ns),
            "ack_scheme": "ACK_NONE",
            "acknowledged": False,
        }
