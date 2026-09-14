"""Which route to monitor, if any.

An ACTIVE voyage wins and pins an exact edition. A dangling edition is reported, never resolved to
``latest``. Only with no voyage in play does a vessel's live declared route get monitored.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..world.model import WorldSnapshot
from ..world.route import RoutePlan, RouteRef, VoyagePlan

ACTIVE = "VOYAGE_STATUS_ACTIVE"
PAUSED = "VOYAGE_STATUS_PAUSED"


@dataclass(frozen=True)
class Selection:
    plan: RoutePlan | None
    reason: str
    voyage: VoyagePlan | None = None
    declared_source: str | None = None
    dangling: RouteRef | None = None

    @property
    def execution_key_token(self) -> str | None:
        """The token after ``route_execution/``: the voyage id, or ``declared-{source}``."""
        if self.plan is None:
            return None
        if self.voyage is not None:
            return self.voyage.voyage_id
        return f"declared-{self.declared_source}"


def select(world: WorldSnapshot, now_ns: int, declared_stale_s: float) -> Selection:
    voyages = world.voyages.values()
    active = sorted((v for v in voyages if v.status == ACTIVE), key=lambda v: v.voyage_id)
    if len(active) > 1:
        ids = ", ".join(v.voyage_id for v in active)
        return Selection(None, f"refusing to monitor: several ACTIVE voyages ({ids})")
    if active:
        voyage = active[0]
        ref = voyage.route_ref
        if ref is None or not ref.route_id:
            return Selection(None, f"voyage {voyage.voyage_id} is ACTIVE but pins no route", voyage)
        plan = world.editions.get((ref.route_id, ref.edition))
        if plan is None:
            return Selection(
                None,
                f"voyage {voyage.voyage_id} pins route {ref.route_id} edition {ref.edition}, "
                "which is not available; not falling back to latest",
                voyage,
                dangling=ref,
            )
        return _checked(Selection(plan, f"monitoring voyage {voyage.voyage_id}", voyage))

    paused = sorted(v.voyage_id for v in voyages if v.status == PAUSED)
    if paused:
        return Selection(None, f"voyage {paused[0]} is PAUSED")

    live = {
        src: d
        for src, d in world.declared.items()
        if d.live and now_ns - d.received_ns <= declared_stale_s * 1e9
    }
    if len(live) > 1:
        return Selection(None, f"refusing to monitor: several declared routes ({sorted(live)})")
    if live:
        source, declared = next(iter(live.items()))
        return _checked(
            Selection(
                declared.plan, f"monitoring declared route from source {source}", None, source
            )
        )
    return Selection(None, "no active voyage and no live declared route")


def _checked(selection: Selection) -> Selection:
    problems = selection.plan.problems()
    if not problems:
        return selection
    return Selection(
        None,
        f"cannot monitor route {selection.plan.route_id}: {'; '.join(problems)}",
        selection.voyage,
        selection.declared_source,
    )
