"""The deterministic processing loop, free of zenoh.

``Processor.tick(now_ns)`` is the only entry point. It returns the outputs for that instant and
does no I/O, so the same code runs on a wall-clock timer in production and on Slipway's tick
fence in software-in-the-loop (Phase 5).
"""

from __future__ import annotations

from dataclasses import dataclass

from .bus.codec import rfc3339
from .conduct.state import Mode, NavigationState


@dataclass(frozen=True)
class Output:
    module: str
    subject: str
    payload: dict


class Processor:
    def __init__(self, conf: dict):
        self.conf = conf
        self._state_period_ns = int(1e9 / conf["state_hz"])
        self._next_state_ns: int | None = None

    def enabled(self, module: str) -> bool:
        return bool(self.conf["modules"].get(module))

    def publications(self) -> dict[str, tuple[str, ...]]:
        """Subjects each enabled module publishes. Drives liveliness and publisher declaration."""
        pubs: dict[str, tuple[str, ...]] = {}
        if self.enabled("conduct"):
            pubs["conduct"] = ("navigation_state",)
        return pubs

    def tick(self, now_ns: int) -> list[Output]:
        outputs: list[Output] = []
        if self.enabled("conduct") and (
            self._next_state_ns is None or now_ns >= self._next_state_ns
        ):
            outputs.append(Output("conduct", "navigation_state", self._state(now_ns).to_payload()))
            # Anchor to the schedule, not to now, so jitter does not accumulate; skip missed
            # slots rather than bursting to catch up.
            if self._next_state_ns is None:
                self._next_state_ns = now_ns
            while self._next_state_ns <= now_ns:
                self._next_state_ns += self._state_period_ns
        return outputs

    def _state(self, now_ns: int) -> NavigationState:
        # Phase 0 subscribes to nothing, so nothing is known: no voyage, no authority, no conn.
        return NavigationState(
            timestamp=rfc3339(now_ns),
            mode=Mode.IDLE,
            reason="no inputs subscribed in this version; not navigating",
        )
