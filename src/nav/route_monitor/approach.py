"""How fast the vessel is moving across the track, and how long until it reaches a limit.

The early half of the cross-track alarm (crowsnest-dev#253): a CAUTION while the limit on the side
she is moving toward is under ``approach_time_s`` away, before the deviation WARNING says she is
past it. The numbers are Crowsnest's own (passageKpis.js lateralRate / timeToLimit), so PASSAGE's
"time to limit" and this alarm cannot disagree about the same fix:

  - the rate is a least-squares slope over the last 10 s of fixes, and there is none with fewer
    than 3 fixes or less than 3 s of them -- a rate from two points is noise;
  - below 0.02 m/s she is not going anywhere across the track, and a time to limit would be noise;
  - unknown is None, never 0. 0 means already outside.
"""

from __future__ import annotations

from collections import deque

RATE_WINDOW_S = 10.0
RATE_MIN_SAMPLES = 3
RATE_MIN_SPAN_S = 3.0
RATE_FLOOR_MPS = 0.02


class LateralRate:
    """Cross-track samples of ONE leg; +m/s toward starboard. Reset on a leg or route change,
    where the cross-track jumps and a slope across the jump would be a phantom rate."""

    def __init__(self, window_s: float = RATE_WINDOW_S):
        self.window_ns = int(window_s * 1e9)
        self._samples: deque[tuple[int, float]] = deque()
        self._key: object = None

    def reset(self) -> None:
        self._samples.clear()
        self._key = None

    def add(self, key: object, t_ns: int, xtd_m: float) -> None:
        """``key`` names the leg; a different one starts over. A repeated ``t_ns`` (the tick runs
        faster than the fixes arrive) is the same fix and is not counted twice."""
        if key != self._key:
            self.reset()
            self._key = key
        if self._samples and t_ns <= self._samples[-1][0]:
            return
        self._samples.append((t_ns, xtd_m))
        while self._samples and self._samples[0][0] < t_ns - self.window_ns:
            self._samples.popleft()

    def rate_mps(self) -> float | None:
        pts = list(self._samples)
        if len(pts) < RATE_MIN_SAMPLES or (pts[-1][0] - pts[0][0]) < RATE_MIN_SPAN_S * 1e9:
            return None
        t0 = pts[0][0]
        ts = [(t - t0) / 1e9 for t, _ in pts]
        n = len(pts)
        mt = sum(ts) / n
        mx = sum(x for _, x in pts) / n
        num = sum((t - mt) * (x - mx) for t, (_, x) in zip(ts, pts, strict=True))
        den = sum((t - mt) ** 2 for t in ts)
        return num / den if den > 0 else None


def time_to_limit(
    xtd_m: float | None, rate_mps: float | None, port_m: float | None, stbd_m: float | None
) -> float | None:
    """Seconds until the limit on the side she is moving toward (xtd + starboard). 0 when already
    outside; None when moving away, not moving across, or there is no limit on that side."""
    if xtd_m is None:
        return None
    p = port_m if port_m is not None and port_m > 0 else None
    s = stbd_m if stbd_m is not None and stbd_m > 0 else None
    if (s is not None and xtd_m > s) or (p is not None and xtd_m < -p):
        return 0.0
    if rate_mps is None or abs(rate_mps) < RATE_FLOOR_MPS:
        return None
    if rate_mps > 0:
        return None if s is None else (s - xtd_m) / rate_mps
    return None if p is None else (xtd_m + p) / -rate_mps
