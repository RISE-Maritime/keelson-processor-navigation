"""The COLREG advisor: encounters, one advice for the vessel, and what became of it.

Advisory by construction. The advisor emits encounter, navigation_advice, advice_disposition and
alarm, and nothing else; tests/test_advisor.py holds it to that. Acting on advice is Phase 4's
guidance, after an ACCEPTED disposition from whoever holds the decision.
"""

from __future__ import annotations

import base64
import hashlib
import json
import uuid
from collections.abc import Mapping
from dataclasses import dataclass, field

from ..bus.codec import rfc3339
from ..outputs import Emit
from ..route_monitor.alarms import AlarmManager, AlarmSpec
from ..traffic.cpa import Kinematics
from ..traffic.monitor import Assessed
from .classify import Role, Situation, Vessel, classify, relative_deg
from .records import AdviceRecord, DispositionRecord, EncounterRecord, TargetRef
from .risk import Risk, Thresholds, grade, thresholds_for
from .synthesize import NOT_CONSIDERED, Manoeuvre, SynthesisConfig, synthesize

POLICY_ID = "keelson-processor-navigation/colreg/v1"
AVOIDING_KINDS = frozenset(
    {"KIND_COURSE_ALTERATION", "KIND_SPEED_CHANGE", "KIND_COURSE_AND_SPEED", "KIND_STOP"}
)
SUBJECTS = ("encounter", "navigation_advice", "advice_disposition", "alarm")

_SITUATION_WORDS = {
    Situation.OVERTAKING: "own ship overtaking",
    Situation.OVERTAKEN: "being overtaken",
    Situation.CROSSING_STBD: "crossing from starboard",
    Situation.CROSSING_PORT: "crossing from port",
    Situation.HEAD_ON: "head-on",
    Situation.NONE: "no vessel rule applies",
    Situation.UNSPECIFIED: "situation unknown",
}
_ROLE_WORDS = {
    Role.GIVE_WAY: "give-way",
    Role.STAND_ON: "stand-on",
    Role.NONE: "neither stands on",
    Role.UNSPECIFIED: "role unknown",
}


@dataclass(frozen=True)
class AdvisorConfig:
    default_context: str = "OPERATIONAL_CONTEXT_COASTAL"
    clear_margin: float = 1.2
    advice_validity_s: float = 30.0
    advice_restate_s: float = 5.0
    same_course_tolerance_deg: float = 10.0
    same_speed_tolerance_knots: float = 1.0
    restricted_visibility_m: float = 1852.0
    stand_on_not_improving_factor: float = 1.05
    thresholds: Mapping[str, Thresholds] | None = None
    synthesis: SynthesisConfig = field(default_factory=SynthesisConfig)
    alarms: bool = True
    alarm_republish_s: float = 5.0
    alarm_ttl_s: float = 15.0

    def digest(self) -> str:
        blob = json.dumps(
            {k: v for k, v in self.__dict__.items() if k not in ("thresholds", "synthesis")}
            | {
                "synthesis": self.synthesis.__dict__,
                "thresholds": {k: v.__dict__ for k, v in (self.thresholds or {}).items()},
            },
            sort_keys=True,
            default=str,
        ).encode()
        return base64.b64encode(hashlib.sha256(blob).digest()).decode()


@dataclass(frozen=True)
class OwnState:
    kinematics: Kinematics
    heading_deg: float | None


@dataclass
class _Encounter:
    encounter_id: str
    target_id: str
    target: TargetRef
    name: str
    situation: Situation
    role: Role
    rules: tuple[str, ...]
    reason: str
    opened_ns: int
    cpa_at_open_m: float | None
    risk: Risk
    peak: Risk
    last: Assessed


@dataclass
class _Advice:
    record: AdviceRecord
    last_restated_ns: int
    valid_to_ns: int


class Advisor:
    def __init__(self, cfg: AdvisorConfig | None = None, id_factory=None):
        self.cfg = cfg or AdvisorConfig()
        self._new_id = id_factory or (lambda: str(uuid.uuid4()))
        self._open: dict[str, _Encounter] = {}
        self._advice: _Advice | None = None
        self._digest = self.cfg.digest()
        self.alarms = AlarmManager(self.cfg.alarm_republish_s, self.cfg.alarm_ttl_s)
        self.dispositions: dict[str, DispositionRecord] = {}

    def _threshold_table(self) -> dict[str, Thresholds]:
        from .risk import DEFAULT_THRESHOLDS

        return {**DEFAULT_THRESHOLDS, **dict(self.cfg.thresholds or {})}

    @property
    def open_encounters(self) -> dict[str, _Encounter]:
        return dict(self._open)

    @property
    def active_advice(self) -> AdviceRecord | None:
        return self._advice.record if self._advice else None

    def record_disposition(self, record: DispositionRecord) -> None:
        self.dispositions[record.advice_id] = record

    def step(
        self,
        now_ns: int,
        own: OwnState | None,
        in_range: Mapping[str, Assessed],
        context: str | None,
        voyage_id: str | None,
        visibility_m: float | None = None,
    ) -> list[Emit]:
        out: list[Emit] = []
        token = voyage_id or "novoyage"
        th = thresholds_for(context or self.cfg.default_context, self._threshold_table())
        restricted = visibility_m is not None and visibility_m < self.cfg.restricted_visibility_m
        stamp = rfc3339(now_ns)

        if own is None or not own.kinematics.velocity_known:
            # Without own velocity there is no CPA and no advice. Open encounters are held, not
            # closed: the risk has not gone away because own ship stopped reporting.
            out += self._alarm_emits(now_ns, self._wanted_alarms(unassessable=bool(self._open)))
            return out

        own_vessel = Vessel(own.heading_deg, own.kinematics.cog_deg, own.kinematics.sog_knots)

        # -- encounters -------------------------------------------------------------------------
        for target_id, a in sorted(in_range.items()):
            g = a.geometry
            enc = self._open.get(target_id)
            previous = enc.risk if enc else Risk.NONE
            risk = grade(g.cpa_m, g.tcpa_s, th, previous, self.cfg.clear_margin)
            if enc is None:
                if risk < Risk.RISK:
                    continue
                view = a.view
                c = classify(
                    own_vessel,
                    Vessel(
                        view.heading.value if view.heading else None,
                        a.kinematics.cog_deg,
                        a.kinematics.sog_knots,
                        view.nav_status.value if view.nav_status else None,
                    ),
                    g.bearing_deg,
                    closing=g.tcpa_s is not None and g.tcpa_s > 0,
                    restricted_visibility=restricted,
                )
                entity, _, source = view.position.source_id.partition("/")
                enc = _Encounter(
                    self._new_id(),
                    target_id,
                    TargetRef(entity, source, target_id),
                    view.name.value if view.name else target_id,
                    c.situation,
                    c.role,
                    c.rules,
                    c.reason,
                    now_ns,
                    g.cpa_m,
                    risk,
                    risk,
                    a,
                )
                self._open[target_id] = enc
            else:
                enc.risk, enc.last = risk, a
                enc.peak = max(enc.peak, risk)
            closed = risk < Risk.RISK
            out.append(
                Emit(
                    "encounter",
                    self._encounter_record(enc, own, stamp, token, th, closed=closed).to_payload(),
                    source_id=f"{token}/{enc.encounter_id}",
                )
            )
            if closed:
                del self._open[target_id]

        for target_id in sorted(set(self._open) - set(in_range)):
            enc = self._open.pop(target_id)
            record = self._encounter_record(enc, own, stamp, token, th, closed=True, lost=True)
            out.append(
                Emit("encounter", record.to_payload(), source_id=f"{token}/{enc.encounter_id}")
            )

        # -- advice -----------------------------------------------------------------------------
        out += self._advise(now_ns, own, in_range, th, token, voyage_id, restricted)
        out += self._alarm_emits(now_ns, self._wanted_alarms())
        return out

    # ------------------------------------------------------------------------------------------

    def _encounter_record(
        self, enc, own, stamp, token, th, closed=False, lost=False
    ) -> EncounterRecord:
        g = enc.last.geometry
        rel = (
            relative_deg(g.bearing_deg, own.heading_deg)
            if own and own.heading_deg is not None
            else None
        )
        reasoning = enc.reason + ("; target no longer tracked in range" if lost else "")
        return EncounterRecord(
            timestamp=stamp,
            encounter_id=enc.encounter_id,
            voyage_id=None if token == "novoyage" else token,
            target=enc.target,
            target_name=enc.name,
            range_m=g.range_m,
            bearing_true_deg=g.bearing_deg,
            bearing_relative_deg=rel,
            cpa_m=g.cpa_m,
            tcpa_s=g.tcpa_s,
            situation=enc.situation.value,
            own_role=enc.role.value,
            rules=enc.rules,
            reasoning=reasoning,
            risk=enc.risk.proto_name,
            peak_risk=enc.peak.proto_name,
            opened_at=rfc3339(enc.opened_ns),
            closed_at=stamp if closed else None,
            threshold_context=_context_name(th, self.cfg),
        )

    def _advise(self, now_ns, own, in_range, th, token, voyage_id, restricted) -> list[Emit]:
        out: list[Emit] = []
        active = {tid: e for tid, e in self._open.items() if e.risk >= Risk.RISK}
        kin = own.kinematics

        if not active:
            if self._advice is None:
                return out
            current = self._advice.record
            if current.kind in AVOIDING_KINDS:
                resume = self._make_advice(
                    now_ns,
                    token,
                    voyage_id,
                    "KIND_RESUME_TRACK",
                    (),
                    None,
                    None,
                    kin,
                    None,
                    ("8",),
                    "Encounters this advice answered have closed: resume the planned track.",
                )
                return self._supersede(now_ns, token, resume)
            if now_ns >= self._advice.valid_to_ns or current.kind == "KIND_MAINTAIN":
                out.append(
                    self._disposition(
                        now_ns,
                        token,
                        current,
                        "STATE_EXPIRED",
                        "encounters closed"
                        if current.kind == "KIND_MAINTAIN"
                        else "validity ended",
                    )
                )
                self._advice = None
            return out

        targets = {tid: a.kinematics for tid, a in in_range.items()}
        give_way = {
            tid: e for tid, e in active.items() if e.role in (Role.GIVE_WAY, Role.UNSPECIFIED)
        }
        no_role = {tid: e for tid, e in active.items() if e.role == Role.NONE}
        stand_on = {tid: e for tid, e in active.items() if e.role == Role.STAND_ON}
        ids = tuple(sorted(e.encounter_id for e in active.values()))
        rules = tuple(dict.fromkeys(r for e in active.values() for r in e.rules))

        if give_way or no_role:
            turns = ("starboard",)
            if (
                give_way
                and all(e.situation == Situation.OVERTAKING for e in give_way.values())
                and not no_role
            ):
                turns = ("starboard", "port")  # Rule 13 does not prescribe a side
            if no_role and all(
                relative_deg(e.last.geometry.bearing_deg, own.heading_deg or kin.cog_deg) > 90
                and relative_deg(e.last.geometry.bearing_deg, own.heading_deg or kin.cog_deg) < 180
                for e in no_role.values()
            ):
                turns = (
                    "port",
                )  # 19(d)(ii): not towards a vessel abeam or abaft on the starboard side
            guarded = [tid for tid, e in give_way.items() if e.situation == Situation.CROSSING_STBD]
            subjects = [tid for tid in active]
            m = synthesize(
                kin,
                targets,
                th.cpa_m,
                turns,
                guarded,
                self.cfg.synthesis,
                subjects,
                th.danger_cpa_m,
            )
            return self._offer(
                now_ns, token, voyage_id, m, ids, (*rules, "8"), kin, self._why(active, m, turns)
            )

        # Only stand-on encounters.
        acting = [
            e
            for e in stand_on.values()
            if e.risk >= Risk.DANGER
            and e.last.geometry.tcpa_s is not None
            and e.last.geometry.tcpa_s < th.danger_tcpa_s
            and e.cpa_at_open_m is not None
            and e.last.geometry.cpa_m is not None
            and e.last.geometry.cpa_m <= e.cpa_at_open_m * self.cfg.stand_on_not_improving_factor
        ]
        if acting:
            m = synthesize(
                kin,
                targets,
                th.cpa_m,
                ("starboard",),
                (),
                self.cfg.synthesis,
                list(active),
                th.danger_cpa_m,
            )
            why = (
                "Stand-on, but the give-way vessel is not taking action and the approach is close: "
                "act to avoid collision under 17(a)(ii), not altering to port for a vessel "
                "on the port side "
                f"(17(c)). {self._describe(m)} {NOT_CONSIDERED}"
            )
            return self._offer(
                now_ns, token, voyage_id, m, ids, (*rules, "17(a)(ii)", "17(c)"), kin, why
            )
        maintain = Manoeuvre(
            "KIND_MAINTAIN", kin.cog_deg, kin.sog_knots, 0.0, _no_prediction(), True
        )
        why = (
            "Stand-on vessel: keep course and speed (17(a)(i)) and watch whether the give-way "
            "vessel acts."
        )
        return self._offer(now_ns, token, voyage_id, maintain, ids, (*rules, "17(a)(i)"), kin, why)

    def _offer(
        self, now_ns, token, voyage_id, m: Manoeuvre, ids, rules, kin, rationale
    ) -> list[Emit]:
        current = self._advice
        same = (
            current is not None
            and current.record.kind == m.kind
            and set(current.record.encounter_ids) == set(ids)
            and _close(current.record.course_deg, m.course_deg, self.cfg.same_course_tolerance_deg)
            and _close(current.record.sog_knots, m.sog_knots, self.cfg.same_speed_tolerance_knots)
        )
        if same:
            if now_ns - current.last_restated_ns < self.cfg.advice_restate_s * 1e9:
                return []
            restated = self._make_advice(
                now_ns,
                token,
                voyage_id,
                m.kind,
                ids,
                m.course_deg,
                m.sog_knots,
                kin,
                m,
                rules,
                rationale,
                advice_id=current.record.advice_id,
                from_course=current.record.from_course_deg,
                from_sog=current.record.from_sog_knots,
            )
            self._advice = _Advice(restated, now_ns, now_ns + int(self.cfg.advice_validity_s * 1e9))
            return [
                Emit(
                    "navigation_advice",
                    restated.to_payload(),
                    source_id=f"{token}/{restated.advice_id}",
                )
            ]
        new = self._make_advice(
            now_ns,
            token,
            voyage_id,
            m.kind,
            ids,
            m.course_deg,
            m.sog_knots,
            kin,
            m,
            rules,
            rationale,
        )
        return self._supersede(now_ns, token, new)

    def _supersede(self, now_ns, token, new: AdviceRecord) -> list[Emit]:
        out = []
        if self._advice is not None:
            out.append(
                self._disposition(
                    now_ns,
                    token,
                    self._advice.record,
                    "STATE_SUPERSEDED",
                    "a different manoeuvre is advised",
                    superseded_by=new.advice_id,
                )
            )
        self._advice = _Advice(new, now_ns, now_ns + int(self.cfg.advice_validity_s * 1e9))
        out.append(
            Emit("navigation_advice", new.to_payload(), source_id=f"{token}/{new.advice_id}")
        )
        return out

    def _make_advice(
        self,
        now_ns,
        token,
        voyage_id,
        kind,
        ids,
        course,
        sog,
        kin,
        m,
        rules,
        rationale,
        advice_id=None,
        from_course=None,
        from_sog=None,
    ) -> AdviceRecord:
        p = m.prediction if m is not None else None
        return AdviceRecord(
            timestamp=rfc3339(now_ns),
            advice_id=advice_id or self._new_id(),
            voyage_id=voyage_id,
            kind=kind,
            encounter_ids=tuple(ids),
            course_deg=course,
            sog_knots=sog,
            from_course_deg=from_course if from_course is not None else kin.cog_deg,
            from_sog_knots=from_sog if from_sog is not None else kin.sog_knots,
            valid_from=rfc3339(now_ns),
            valid_to=rfc3339(now_ns + int(self.cfg.advice_validity_s * 1e9)),
            expected_min_cpa_m=p.min_cpa_m if p else None,
            expected_min_range_m=p.min_range_m if p else None,
            rules=tuple(dict.fromkeys(rules)),
            rationale=rationale,
            policy_id=POLICY_ID,
            policy_config_digest=self._digest,
        )

    def _disposition(
        self, now_ns, token, advice: AdviceRecord, state, reason, superseded_by=None
    ) -> Emit:
        d = DispositionRecord(
            rfc3339(now_ns),
            advice.advice_id,
            advice.voyage_id,
            state,
            decided_by="navigation/colreg",
            superseded_by=superseded_by,
            reason=reason,
        )
        self.dispositions[advice.advice_id] = d
        return Emit("advice_disposition", d.to_payload(), source_id=f"{token}/{advice.advice_id}")

    def _why(self, active, m: Manoeuvre, turns) -> str:
        parts = [
            f"{e.name}: {_SITUATION_WORDS[e.situation]}, own vessel {_ROLE_WORDS[e.role]} "
            f"({', '.join(e.rules) or 'no rule'})"
            for e in sorted(active.values(), key=lambda e: e.name)
        ]
        side = " or ".join(turns)
        return (
            f"{'; '.join(parts)}. Alterations tried to {side}. {self._describe(m)} {NOT_CONSIDERED}"
        )

    @staticmethod
    def _describe(m: Manoeuvre) -> str:
        if m.kind in ("KIND_COURSE_ALTERATION", "KIND_COURSE_AND_SPEED"):
            side = "starboard" if m.alteration_deg > 0 else "port"
            text = f"Alter {abs(m.alteration_deg):.0f} degrees to {side}, to {m.course_deg:.0f}"
            if m.kind == "KIND_COURSE_AND_SPEED":
                text += f", and reduce to {m.sog_knots:.1f} kn"
        elif m.kind == "KIND_SPEED_CHANGE":
            text = f"Reduce speed to {m.sog_knots:.1f} kn on the present course"
        elif m.kind == "KIND_STOP":
            text = "Take all way off"
        else:
            text = "Keep course and speed"
        if m.kind == "KIND_MAINTAIN":
            return text + "."
        if not m.satisfied:
            return (
                f"{text}. NO MANOEUVRE CLEARS the safe passing distance: this is the best found, "
                f"expected minimum distance {m.prediction.min_cpa_m:.0f} m."
            )
        return (
            f"{text}. Expected minimum CPA {m.prediction.min_cpa_subject_m:.0f} m to the vessels "
            "this "
            f"answers and {m.prediction.min_cpa_other_m:.0f} m to any other target."
        )

    def _wanted_alarms(self, unassessable: bool = False) -> dict[str, AlarmSpec]:
        wanted = {}
        for e in self._open.values():
            if e.risk >= Risk.DANGER:
                wanted[f"colreg/collision_risk/{e.encounter_id}"] = AlarmSpec(
                    f"Collision risk with {e.name}: {_SITUATION_WORDS[e.situation]}, "
                    f"own vessel {_ROLE_WORDS[e.role]}",
                    priority="PRIORITY_WARNING",
                    severity="SEVERITY_LEVEL_HIGH",
                    category="CATEGORY_SAFETY",
                )
        advice = self.active_advice
        if advice is not None and advice.rationale and "NO MANOEUVRE CLEARS" in advice.rationale:
            wanted["colreg/no_clear_manoeuvre"] = AlarmSpec(
                "No manoeuvre found that clears all traffic: take action now",
                priority="PRIORITY_EMERGENCY",
                severity="SEVERITY_LEVEL_CRITICAL",
                category="CATEGORY_SAFETY",
            )
        if unassessable:
            wanted["colreg/own_velocity_unknown"] = AlarmSpec(
                "Open collision-risk encounters cannot be assessed: own course or speed unknown",
                priority="PRIORITY_WARNING",
                severity="SEVERITY_LEVEL_HIGH",
                category="CATEGORY_SAFETY",
            )
        return wanted

    def _alarm_emits(self, now_ns, wanted) -> list[Emit]:
        if not self.cfg.alarms:
            return []
        return [Emit("alarm", p) for p in self.alarms.step(now_ns, wanted)]


def _close(a, b, tol) -> bool:
    if a is None or b is None:
        return a is None and b is None
    diff = abs((a - b + 180.0) % 360.0 - 180.0) if tol >= 5 else abs(a - b)
    return diff <= tol


def _no_prediction():
    from .synthesize import Prediction

    return Prediction(float("inf"), float("inf"))


def _context_name(th: Thresholds, cfg: AdvisorConfig) -> str:
    from .risk import DEFAULT_THRESHOLDS

    table = {**DEFAULT_THRESHOLDS, **dict(cfg.thresholds or {})}
    return next(
        (name for name, value in table.items() if value == th), "OPERATIONAL_CONTEXT_UNSPECIFIED"
    )


def advisor_config(conf: dict) -> AdvisorConfig:
    cr = conf["colreg"]
    return AdvisorConfig(
        default_context=cr["default_context"],
        clear_margin=cr["clear_margin"],
        advice_validity_s=cr["advice_validity_s"],
        advice_restate_s=cr["advice_restate_s"],
        restricted_visibility_m=cr["restricted_visibility_m"],
        thresholds={k: Thresholds(**v) for k, v in cr["thresholds"].items()},
        synthesis=SynthesisConfig(
            min_alteration_deg=cr["min_alteration_deg"],
            max_alteration_deg=cr["max_alteration_deg"],
            step_deg=cr["alteration_step_deg"],
            speed_factors=tuple(cr["speed_factors"]),
            horizon_s=cr["horizon_s"],
        ),
        alarms=cr["alarms"],
        alarm_republish_s=conf["route_monitor"]["alarm_republish_s"],
        alarm_ttl_s=conf["route_monitor"]["alarm_ttl_s"],
    )
