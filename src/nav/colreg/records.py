"""Encounter, NavigationAdvice and AdviceDisposition payloads, field for field with the drafts in
docs/upstream/. tests/test_records.py fails if a payload emits a name the draft does not declare.
"""

from __future__ import annotations

from dataclasses import dataclass, field

KNOT_MPS = 1852.0 / 3600.0


def _put(out: dict, key: str, value) -> None:
    if value is not None:
        out[key] = value


@dataclass(frozen=True)
class TargetRef:
    entity_id: str
    source_id: str
    target_id: str


@dataclass(frozen=True)
class EncounterRecord:
    timestamp: str
    encounter_id: str
    voyage_id: str | None
    target: TargetRef
    target_name: str
    range_m: float
    bearing_true_deg: float
    bearing_relative_deg: float | None
    cpa_m: float | None
    tcpa_s: float | None
    situation: str
    own_role: str
    rules: tuple[str, ...]
    reasoning: str
    risk: str
    peak_risk: str
    opened_at: str
    closed_at: str | None
    threshold_context: str

    def to_payload(self) -> dict:
        out = {
            "timestamp": self.timestamp,
            "encounter_id": self.encounter_id,
            "target": {
                "entity_id": self.target.entity_id,
                "source_id": self.target.source_id,
                "target_id": self.target.target_id,
            },
            "target_name": self.target_name,
            "range_m": self.range_m,
            "bearing_true_deg": self.bearing_true_deg,
            "bearing_relative_deg": self.bearing_relative_deg
            if self.bearing_relative_deg is not None
            else 0.0,
            "situation": self.situation,
            "own_role": self.own_role,
            "rules": list(self.rules),
            "reasoning": self.reasoning,
            "risk": self.risk,
            "peak_risk": self.peak_risk,
            "opened_at": self.opened_at,
            "threshold_context": self.threshold_context,
        }
        _put(out, "voyage_id", self.voyage_id)
        _put(out, "cpa_m", self.cpa_m)
        _put(out, "tcpa_s", self.tcpa_s)
        _put(out, "closed_at", self.closed_at)
        return out


@dataclass(frozen=True)
class AdviceRecord:
    timestamp: str
    advice_id: str
    voyage_id: str | None
    kind: str
    encounter_ids: tuple[str, ...]
    course_deg: float | None
    sog_knots: float | None
    from_course_deg: float | None
    from_sog_knots: float | None
    valid_from: str
    valid_to: str
    expected_min_cpa_m: float | None
    expected_min_range_m: float | None
    rules: tuple[str, ...]
    rationale: str
    policy_id: str | None = None
    policy_config_digest: str | None = None  # base64, as protobuf JSON encodes bytes

    def to_payload(self) -> dict:
        out = {
            "timestamp": self.timestamp,
            "advice_id": self.advice_id,
            "kind": self.kind,
            "encounter_ids": list(self.encounter_ids),
            "valid_from": self.valid_from,
            "valid_to": self.valid_to,
            "rules": list(self.rules),
            "rationale": self.rationale,
        }
        _put(out, "voyage_id", self.voyage_id)
        if self.course_deg is not None:
            out["steering"] = {"course_over_ground_deg": self.course_deg}
        if self.sog_knots is not None:
            out["propulsion"] = {"sog_mps": self.sog_knots * KNOT_MPS}
        _put(out, "from_course_over_ground_deg", self.from_course_deg)
        _put(
            out,
            "from_sog_mps",
            None if self.from_sog_knots is None else self.from_sog_knots * KNOT_MPS,
        )
        _put(out, "expected_min_cpa_m", _finite(self.expected_min_cpa_m))
        _put(out, "expected_min_range_m", _finite(self.expected_min_range_m))
        _put(out, "policy_id", self.policy_id)
        _put(out, "policy_config_digest", self.policy_config_digest)
        return out


@dataclass(frozen=True)
class DispositionRecord:
    timestamp: str
    advice_id: str
    voyage_id: str | None
    state: str
    decided_by: str
    decided_site: str = ""
    superseded_by: str | None = None
    reason: str = ""

    def to_payload(self) -> dict:
        out = {
            "timestamp": self.timestamp,
            "advice_id": self.advice_id,
            "state": self.state,
            "decided_by": self.decided_by,
            "decided_site": self.decided_site,
            "reason": self.reason,
        }
        _put(out, "voyage_id", self.voyage_id)
        _put(out, "superseded_by", self.superseded_by)
        return out

    @classmethod
    def from_payload(cls, payload: dict) -> DispositionRecord:
        return cls(
            timestamp=payload.get("timestamp", ""),
            advice_id=payload["advice_id"],
            voyage_id=payload.get("voyage_id"),
            state=payload.get("state", "STATE_UNSPECIFIED"),
            decided_by=payload.get("decided_by", ""),
            decided_site=payload.get("decided_site", ""),
            superseded_by=payload.get("superseded_by"),
            reason=payload.get("reason", ""),
        )


def _finite(x: float | None) -> float | None:
    return x if x is not None and x != float("inf") else None


__all__ = ["AdviceRecord", "DispositionRecord", "EncounterRecord", "TargetRef", "field"]
