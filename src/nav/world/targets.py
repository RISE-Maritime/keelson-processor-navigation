"""Targets heard under ``@target``, fused across feeds by target id.

Per subject, the newest receipt wins whichever feed it came from: two AIS feeds reporting the same
MMSI are two copies of one transmission stream. A target exists while its position is fresher than
the configured age; its other subjects are used only while they are equally fresh.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

from .latch import Sample, fresh

TARGET_SUBJECTS = (
    "location_fix",
    "course_over_ground_deg",
    "speed_over_ground_knots",
    "heading_true_north_deg",
    "nav_status",
    "name",
    "length_over_all_m",
    "breadth_over_all_m",
)
_STATIC = frozenset({"name", "length_over_all_m", "breadth_over_all_m"})


@dataclass(frozen=True)
class TargetView:
    target_id: str
    position: Sample
    cog: Sample | None = None
    sog: Sample | None = None
    heading: Sample | None = None
    nav_status: Sample | None = None
    name: Sample | None = None
    length: Sample | None = None
    breadth: Sample | None = None


class TargetTable:
    def __init__(self):
        self._lock = threading.Lock()
        self._targets: dict[str, dict[str, Sample]] = {}

    def put(self, target_id: str, subject: str, sample: Sample) -> None:
        with self._lock:
            subjects = self._targets.setdefault(target_id, {})
            prev = subjects.get(subject)
            if prev is None or sample.received_ns >= prev.received_ns:
                subjects[subject] = sample

    def snapshot(
        self, now_ns: int, max_age_s: float, static_max_age_s: float
    ) -> dict[str, TargetView]:
        """Live targets. Targets whose position has aged out are dropped from the table."""
        out = {}
        with self._lock:
            for target_id in list(self._targets):
                s = self._targets[target_id]
                position = fresh(s.get("location_fix"), now_ns, max_age_s)
                if position is None:
                    if "location_fix" in s or all(
                        fresh(v, now_ns, static_max_age_s) is None for v in s.values()
                    ):
                        del self._targets[target_id]
                    continue

                def dyn(subject, s=s):
                    return fresh(s.get(subject), now_ns, max_age_s)

                def static(subject, s=s):
                    return fresh(s.get(subject), now_ns, static_max_age_s)

                out[target_id] = TargetView(
                    target_id,
                    position,
                    cog=dyn("course_over_ground_deg"),
                    sog=dyn("speed_over_ground_knots"),
                    heading=dyn("heading_true_north_deg"),
                    nav_status=dyn("nav_status"),
                    name=static("name"),
                    length=static("length_over_all_m"),
                    breadth=static("breadth_over_all_m"),
                )
        return out

    def __len__(self) -> int:
        with self._lock:
            return len(self._targets)
