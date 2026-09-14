"""Candidate manoeuvres, each checked against every target in range.

Constant-velocity prediction for the targets: the same assumption CPA already makes, and the reason
advice carries a validity window. Own ship is assumed to take up the new course and speed at once,
which a real hull does not; the rationale says so rather than the numbers pretending otherwise.

Not considered, and stated in every rationale: the chart (depth, land, hazards), traffic separation
schemes, narrow channels, and the route corridor.
"""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass

from ..traffic.cpa import KNOT_MPS, Kinematics, geometry

NOT_CONSIDERED = (
    "Not checked against the chart, traffic separation schemes, narrow channels or the route "
    "corridor. Targets assumed to hold course and speed; own ship assumed to take up the new "
    "course and speed immediately."
)


@dataclass(frozen=True)
class SynthesisConfig:
    min_alteration_deg: float = 30.0  # Rule 8(b): large enough to be readily apparent
    max_alteration_deg: float = 90.0
    step_deg: float = 10.0
    speed_factors: tuple[float, ...] = (0.5, 0.25)
    horizon_s: float = 1800.0  # approaches further out than this are not held against a manoeuvre


@dataclass(frozen=True)
class Prediction:
    min_cpa_m: float  # smallest future CPA over all targets (current range where it is opening)
    min_range_m: float  # same, also counting targets whose approach is past the horizon
    crosses_ahead_of: tuple[str, ...] = ()
    min_cpa_subject_m: float = float("inf")  # over the targets the advice answers
    min_cpa_other_m: float = float("inf")  # over every other target


@dataclass(frozen=True)
class Manoeuvre:
    kind: str  # NavigationAdvice.Kind name
    course_deg: float | None
    sog_knots: float | None
    alteration_deg: float  # signed, + starboard
    prediction: Prediction
    satisfied: bool


def predict(
    own: Kinematics,
    course_deg: float | None,
    sog_knots: float | None,
    targets: Mapping[str, Kinematics],
    no_cross_ahead: Iterable[str] = (),
    horizon_s: float = 1800.0,
    subjects: Iterable[str] | None = None,
) -> Prediction:
    trial = Kinematics(own.lat, own.lon, course_deg, sog_knots)
    guarded = set(no_cross_ahead)
    subject_ids = set(targets) if subjects is None else set(subjects)
    min_cpa = min_range = float("inf")
    min_subject = min_other = float("inf")
    crosses = []
    for tid, t in targets.items():
        g = geometry(trial, t)
        near = g.range_m
        if g.cpa_m is not None and g.tcpa_s is not None and 0 < g.tcpa_s <= horizon_s:
            near = g.cpa_m
        elif g.cpa_m is not None and g.tcpa_s is None:
            near = g.cpa_m  # no relative motion: the range holds
        min_range = min(min_range, near)
        if g.tcpa_s is None or g.tcpa_s > 0 or g.range_m < min_cpa:
            # An approach already past still counts at the current range: a target 18 m off and
            # opening is 18 m off now, not infinitely safe.
            min_cpa = min(min_cpa, near)
            if tid in subject_ids:
                min_subject = min(min_subject, near)
            else:
                min_other = min(min_other, near)
        if tid in guarded and t.cog_deg is not None and t.sog_knots:
            # Own ship crossing the target's heading line ahead of her, seen from her side.
            from_target = geometry(t, trial, t.cog_deg)
            if from_target.bcr_m is not None and from_target.bct_s is not None:
                crosses.append(tid)
    return Prediction(min_cpa, min_range, tuple(sorted(crosses)), min_subject, min_other)


def _course(base: float, alteration: float) -> float:
    return (base + alteration) % 360.0


def synthesize(
    own: Kinematics,
    targets: Mapping[str, Kinematics],
    safe_passing_m: float,
    turns: tuple[str, ...] = ("starboard",),
    no_cross_ahead: Iterable[str] = (),
    cfg: SynthesisConfig | None = None,
    subjects: Iterable[str] | None = None,
    other_min_m: float | None = None,
) -> Manoeuvre:
    """The smallest manoeuvre that clears the targets it answers by ``safe_passing_m`` and keeps
    every other target at least ``other_min_m`` off.

    ``subjects`` are the targets the advice answers (default: all). Holding bystanders to the full
    safe passing distance as well would let one ship passing at the edge of that distance veto
    every sound manoeuvre, so they are held only to ``other_min_m`` (default: ``safe_passing_m``):
    a manoeuvre may bring a bystander closer, never dangerously close.

    Order: course alterations on the allowed sides (smallest first, never below the readily
    apparent minimum), then speed reductions on the present course, then alteration and reduction
    together, then stopping. If nothing clears, the candidate that keeps the most distance is
    returned with satisfied=False (Rule 17(b), best aid to avoid collision): the caller must surface
    that as the danger it is, not as advice that works.
    """
    cfg = cfg or SynthesisConfig()
    if own.cog_deg is None or own.sog_knots is None:
        raise ValueError("own course and speed are needed to synthesise a manoeuvre")
    guarded = tuple(no_cross_ahead)
    subject_ids = tuple(targets) if subjects is None else tuple(subjects)
    other_limit = safe_passing_m if other_min_m is None else other_min_m
    base_course, base_sog = own.cog_deg, own.sog_knots

    tried: list[Manoeuvre] = []

    def attempt(kind, alteration, sog):
        course = _course(base_course, alteration)
        p = predict(own, course, sog, targets, guarded, cfg.horizon_s, subject_ids)
        ok = (
            p.min_cpa_subject_m >= safe_passing_m
            and p.min_cpa_other_m >= other_limit
            and not p.crosses_ahead_of
        )
        m = Manoeuvre(kind, course, sog, alteration, p, ok)
        tried.append(m)
        return m

    signs = [(+1.0 if side == "starboard" else -1.0) for side in turns]
    alterations = []
    a = cfg.min_alteration_deg
    while a <= cfg.max_alteration_deg + 1e-9:
        alterations.append(a)
        a += cfg.step_deg

    for alteration in alterations:
        for sign in signs:
            m = attempt("KIND_COURSE_ALTERATION", sign * alteration, base_sog)
            if m.satisfied:
                return m
    if base_sog > 0:
        for factor in cfg.speed_factors:
            m = attempt("KIND_SPEED_CHANGE", 0.0, base_sog * factor)
            if m.satisfied:
                return m
        for factor in cfg.speed_factors:
            for alteration in alterations:
                for sign in signs:
                    m = attempt("KIND_COURSE_AND_SPEED", sign * alteration, base_sog * factor)
                    if m.satisfied:
                        return m
    stop = attempt("KIND_STOP", 0.0, 0.0)
    if stop.satisfied:
        return stop
    # Nothing clears. Best effort: most distance kept, then no bow crossing, then smallest change.
    return max(
        tried,
        key=lambda m: (
            round(min(m.prediction.min_cpa_subject_m, m.prediction.min_cpa_other_m), 1),
            not m.prediction.crosses_ahead_of,
            -abs(m.alteration_deg),
        ),
    )


def eta_s(distance_m: float, sog_knots: float) -> float | None:
    return distance_m / (sog_knots * KNOT_MPS) if sog_knots and sog_knots > 0 else None
