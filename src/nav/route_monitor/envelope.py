"""Operational envelope: live standing of every authored RouteInfo limit, and the durable
EnvelopeExceedance record of each breach.

Payloads are dicts in the protobuf JSON form of keelson.EnvelopeLimitState and
keelson.EnvelopeExceedance.
"""

from __future__ import annotations

import base64
import hashlib
import json
import uuid
from collections.abc import Callable, Mapping
from dataclasses import dataclass

from ..bus.codec import rfc3339
from ..world.latch import Sample
from ..world.route import RouteLimits, RouteRef

UPPER = "ENVELOPE_BOUND_UPPER"
LOWER = "ENVELOPE_BOUND_LOWER"


@dataclass(frozen=True)
class LimitDef:
    limit_id: str
    bound: str
    subject: str
    transform: Callable[[float], float] = float


# The measured subject for each RouteInfo limit (subjects.yaml; Route.proto comments).
LIMITS: tuple[LimitDef, ...] = (
    LimitDef("max_roll_deg", UPPER, "roll_deg", abs),
    LimitDef("max_wave_height_m", UPPER, "wave_height_significant_m"),
    LimitDef("max_wind_speed_mps", UPPER, "true_wind_speed_mps"),
    LimitDef("max_wind_gust_mps", UPPER, "true_wind_speed_max_mps"),
    LimitDef("min_speed_knots", LOWER, "speed_over_ground_knots"),
    LimitDef("max_speed_knots", UPPER, "speed_over_ground_knots"),
    LimitDef("min_visibility_m", LOWER, "visibility_range_m"),
)
ENVELOPE_SUBJECTS = frozenset(d.subject for d in LIMITS)


@dataclass(frozen=True)
class Standing:
    limit_id: str
    limit_value: float
    bound: str
    state: str
    measured_value: float | None = None
    measured_ns: int | None = None

    def to_payload(self) -> dict:
        out = {
            "limit_id": self.limit_id,
            "limit_value": self.limit_value,
            "state": self.state,
            "bound": self.bound,
        }
        if self.state != "STATE_UNMEASURED":
            out["measured_value"] = self.measured_value
            if self.measured_ns is not None:
                out["measured_at"] = rfc3339(self.measured_ns)
        return out


def breached(bound: str, value: float, limit: float) -> bool:
    return value > limit if bound == UPPER else value < limit


def evaluate(limits: RouteLimits, fresh_inputs: Mapping[str, Sample | None]) -> list[Standing]:
    """One standing per authored limit. ``fresh_inputs`` must already exclude stale samples."""
    authored = limits.authored()
    out = []
    for d in LIMITS:
        if d.limit_id not in authored:
            continue
        limit = authored[d.limit_id]
        sample = fresh_inputs.get(d.subject)
        if sample is None:
            out.append(Standing(d.limit_id, limit, d.bound, "STATE_UNMEASURED"))
            continue
        value = d.transform(sample.value)
        state = "STATE_BREACHED" if breached(d.bound, value, limit) else "STATE_WITHIN"
        out.append(Standing(d.limit_id, limit, d.bound, state, value, sample.measured_ns))
    return out


def envelope_digest(limits: RouteLimits) -> str:
    """base64 SHA-256 over the authored limits: the envelope_config_digest bytes, JSON-encoded."""
    canonical = json.dumps(limits.authored(), sort_keys=True, separators=(",", ":")).encode()
    return base64.b64encode(hashlib.sha256(canonical).digest()).decode()


@dataclass
class _Breach:
    exceedance_id: str
    standing: Standing
    detected_ns: int
    peak: float
    responses: tuple[str, ...]


class ExceedanceTracker:
    """Opens a record when a limit is breached and closes it when the measurement returns inside
    the limit by ``clear_margin`` (a fraction of the limit). Every record restates the whole breach.

    One tracker per voyage. A breach still open when the voyage ends stays open: no closing record
    is invented for a measurement that never came back inside the limit.
    """

    def __init__(self, voyage_id: str, clear_margin: float = 0.02, id_factory=None):
        self.voyage_id = voyage_id
        self.clear_margin = clear_margin
        self._open: dict[str, _Breach] = {}
        self._new_id = id_factory or (lambda: str(uuid.uuid4()))

    @property
    def open_limit_ids(self) -> set[str]:
        return set(self._open)

    def step(
        self,
        standings: list[Standing],
        now_ns: int,
        route_ref: RouteRef,
        leg_from_waypoint_id: str | None,
        limits: RouteLimits,
        responses: tuple[str, ...],
    ) -> list[dict]:
        records = []
        for s in standings:
            breach = self._open.get(s.limit_id)
            if s.state == "STATE_UNMEASURED":
                continue  # cannot open or close on no evidence
            value = s.measured_value
            if breach is None:
                if s.state == "STATE_BREACHED":
                    breach = _Breach(self._new_id(), s, now_ns, value, responses)
                    self._open[s.limit_id] = breach
                    records.append(
                        self._record(breach, now_ns, route_ref, leg_from_waypoint_id, limits)
                    )
                continue
            breach.peak = max(breach.peak, value) if s.bound == UPPER else min(breach.peak, value)
            margin = abs(s.limit_value) * self.clear_margin
            cleared = (
                value <= s.limit_value - margin
                if s.bound == UPPER
                else value >= s.limit_value + margin
            )
            if cleared:
                del self._open[s.limit_id]
                records.append(
                    self._record(
                        breach, now_ns, route_ref, leg_from_waypoint_id, limits, cleared_ns=now_ns
                    )
                )
        return records

    def _record(self, b: _Breach, now_ns, route_ref, leg_id, limits, cleared_ns=None) -> dict:
        s = b.standing
        out = {
            "timestamp": rfc3339(now_ns),
            "exceedance_id": b.exceedance_id,
            "limit_id": s.limit_id,
            "voyage_id": self.voyage_id,
            "route_ref": {
                "route_id": route_ref.route_id,
                "route_edition_number": route_ref.edition,
            },
            "envelope_id": f"route/{route_ref.route_id}/edition/{route_ref.edition}",
            "envelope_config_digest": envelope_digest(limits),
            "detected_at": rfc3339(b.detected_ns),
            "limit_value": s.limit_value,
            "measured_value": s.measured_value,
            "bound": s.bound,
            "responses": list(b.responses),
        }
        if leg_id is not None:
            out["leg_from_waypoint_id"] = leg_id
        if b.responses != ("RESPONSE_NONE",):
            out["responded_at"] = rfc3339(b.detected_ns)
        if cleared_ns is not None:
            out["peak_value"] = b.peak
            out["cleared_at"] = rfc3339(cleared_ns)
        return out
