"""FALLBACK output: neutral axes for ``neutral_burst_s``, then silence (docs/PLAN.md §2).

The burst tells the vehicle "stop" in words before the silence tells it by dead-man; the silence
is what guarantees it, because a publisher that keeps talking can keep a vessel moving.
"""

from __future__ import annotations


class Failsafe:
    def __init__(self, burst_s: float):
        self.burst_ns = int(burst_s * 1e9)
        self._until_ns: int | None = None

    def start(self, now_ns: int) -> None:
        self._until_ns = now_ns + self.burst_ns

    def active(self, now_ns: int) -> bool:
        return self._until_ns is not None and now_ns < self._until_ns

    @property
    def until_ns(self) -> int:
        return self._until_ns or 0
