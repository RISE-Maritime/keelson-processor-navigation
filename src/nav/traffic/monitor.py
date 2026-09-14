"""One tick of traffic: CPA geometry for every live target in range, published per target."""

from __future__ import annotations

from dataclasses import dataclass, field

from ..bus.codec import rfc3339
from ..outputs import Emit
from ..world.latch import fresh
from ..world.model import WorldSnapshot
from ..world.targets import TargetView
from .cpa import Geometry, Kinematics, dead_reckon, geometry, relative_bearing_deg

MODULE = "traffic"
PUBLISHED_SUBJECTS = (
    "target_bearing_north_deg",
    "target_bearing_relative_deg",
    "target_cpa_m",
    "target_tcpa_s",
    "target_bcr_m",
    "target_bct_s",
)


@dataclass(frozen=True)
class Assessed:
    view: TargetView
    kinematics: Kinematics
    geometry: Geometry


@dataclass(frozen=True)
class TrafficStatus:
    tracked: int = 0
    in_range: dict[str, Assessed] = field(default_factory=dict)
    degraded_inputs: tuple[str, ...] = ()


def _value(sample):
    return sample.value if sample is not None else None


class TrafficMonitor:
    def __init__(self, conf: dict):
        self.tr = conf["traffic"]
        self.max_state_age_s = conf["route_monitor"]["max_state_age_s"]
        self.exclude = frozenset(self.tr["exclude_targets"])
        self._period_ns = int(1e9 / self.tr["publish_hz"])
        self._next_ns: int | None = None
        self._in_range: set[str] = set()
        self.status = TrafficStatus()

    def step(self, world: WorldSnapshot, targets: dict[str, TargetView], now_ns: int) -> list[Emit]:
        if self._next_ns is not None and now_ns < self._next_ns:
            return []
        if self._next_ns is None:
            self._next_ns = now_ns
        while self._next_ns <= now_ns:
            self._next_ns += self._period_ns

        age = self.max_state_age_s
        position = fresh(world.inputs.get("location_fix"), now_ns, age)
        if position is None:
            self._in_range.clear()
            self.status = TrafficStatus(len(targets), {}, ("location_fix",))
            return []
        cog = fresh(world.inputs.get("course_over_ground_deg"), now_ns, age)
        sog = fresh(world.inputs.get("speed_over_ground_knots"), now_ns, age)
        heading = fresh(world.inputs.get("heading_true_north_deg"), now_ns, age)
        degraded = tuple(
            s
            for s, v in (
                ("course_over_ground_deg", cog),
                ("speed_over_ground_knots", sog),
                ("heading_true_north_deg", heading),
            )
            if v is None
        )
        own = Kinematics(*position.value, _value(cog), _value(sog))
        own_heading = _value(heading)

        limit = self.tr["range_limit_m"]
        exit_limit = limit * self.tr["range_exit_factor"]
        in_range: dict[str, Assessed] = {}
        out: list[Emit] = []
        stamp = rfc3339(now_ns)
        for target_id, view in sorted(targets.items()):
            if target_id in self.exclude:
                continue
            k = Kinematics(*view.position.value, _value(view.cog), _value(view.sog))
            if self.tr["dead_reckon"]:
                k = dead_reckon(k, (now_ns - view.position.received_ns) / 1e9)
            g = geometry(own, k, own_heading)
            keep = g.range_m <= (exit_limit if target_id in self._in_range else limit)
            if not keep:
                continue
            in_range[target_id] = Assessed(view, k, g)
            values = {
                "target_bearing_north_deg": g.bearing_deg,
                "target_bearing_relative_deg": (
                    relative_bearing_deg(g.bearing_deg, own_heading)
                    if own_heading is not None
                    else None
                ),
                "target_cpa_m": g.cpa_m,
                "target_tcpa_s": g.tcpa_s,
                "target_bcr_m": g.bcr_m,
                "target_bct_s": g.bct_s,
            }
            for subject, value in values.items():
                if value is not None:
                    out.append(
                        Emit(subject, {"timestamp": stamp, "value": value}, target_id=target_id)
                    )

        self._in_range = set(in_range)
        self.status = TrafficStatus(len(targets), in_range, degraded)
        return out
