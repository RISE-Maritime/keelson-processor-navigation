"""Newest-wins latch for measured inputs, written from subscriber threads and read by the tick.

Freshness is judged on the LOCAL receipt time, never the payload's own timestamp: comparing a
remote clock with ours makes correctness a function of clock skew. The payload timestamp is kept
as ``measured_ns`` because RouteExecution and EnvelopeExceedance report it.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Sample:
    value: Any
    received_ns: int
    measured_ns: int | None = None
    source_id: str = ""


class Latch:
    def __init__(self):
        self._lock = threading.Lock()
        self._samples: dict[str, Sample] = {}

    def put(self, subject: str, sample: Sample) -> None:
        with self._lock:
            self._samples[subject] = sample

    def snapshot(self) -> dict[str, Sample]:
        with self._lock:
            return dict(self._samples)


def fresh(sample: Sample | None, now_ns: int, max_age_s: float) -> Sample | None:
    """The sample if it was received within ``max_age_s`` of ``now_ns``, else None."""
    if sample is None or now_ns - sample.received_ns > max_age_s * 1e9:
        return None
    return sample
