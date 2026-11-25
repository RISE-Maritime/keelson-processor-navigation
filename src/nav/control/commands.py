"""navigation_control/v1 requests, decided in the tick.

The RPC layer (bus/control.py) only decodes a request, fetches a route edition from another
vessel's tree when ``load_route`` names one, and waits. Everything that decides — who may call,
whether the edition exists, which voyage becomes ACTIVE, what the order is — happens here, in
``Processor.tick``, so it is deterministic and runs the same under Slipway's tick fence.

The voyage record is written by this process and nobody else while it is live: an RPC, not a
publish, is what keeps two stations from activating different routes on one vessel.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

from ..bus.codec import rfc3339
from ..world.route import RoutePlan, RouteRef, VoyagePlan
from . import settings as settings_mod
from .orders import TRACK, GuidanceOrder, Kind, OrderError

# keelson#320: DP holds a position AND a heading, which needs lateral authority. Guidance drives
# only vehicle_control's steering and throttle axes (config refuses any other backend), so it is
# refused, never approximated by a go_to or a loiter.
NO_DP = (
    "dynamic_positioning needs lateral authority: guidance here steers by vehicle_control "
    "steering and throttle only"
)

PROCEDURES = (
    "load_route",
    "set_guidance_order",
    "set_voyage_status",
    "get_navigation_state",
    "get_settings",  # keelson#322
    "set_settings",  # keelson#322
)

ACCEPTED = "COMMAND_RESULT_ACCEPTED"
TEMPORARILY_REJECTED = "COMMAND_RESULT_TEMPORARILY_REJECTED"
DENIED = "COMMAND_RESULT_DENIED"
UNSUPPORTED = "COMMAND_RESULT_UNSUPPORTED"
FAILED = "COMMAND_RESULT_FAILED"

ACTIVE = "VOYAGE_STATUS_ACTIVE"
PAUSED = "VOYAGE_STATUS_PAUSED"
PLANNED = "VOYAGE_STATUS_PLANNED"
COMPLETED = "VOYAGE_STATUS_COMPLETED"
ABORTED = "VOYAGE_STATUS_ABORTED"
_ENDED = (COMPLETED, ABORTED)


class InvalidRequest(ValueError):
    """A request that is malformed, as opposed to one the process declines. Answered with an
    ErrorResponse(INVALID_ARGUMENT) rather than a CommandResult."""


@dataclass
class Pending:
    """One call in flight: filled in by the tick, waited on by the RPC thread."""

    procedure: str
    request: dict  # protobuf JSON of the request message
    received_ns: int
    fetched: tuple[dict, RoutePlan] | None = None  # load_route from another vessel: (route, plan)
    fetch_error: str | None = None
    response: dict | None = None
    error: str | None = None  # set instead of response for an InvalidRequest
    done: threading.Event = field(default_factory=threading.Event)

    def answer(self, response: dict) -> None:
        self.response = response
        self.done.set()

    def refuse(self, error: str) -> None:
        self.error = error
        self.done.set()


@dataclass(frozen=True)
class Context:
    """What the tick knows when it decides one call."""

    now_ns: int
    entity_id: str
    voyages: dict[str, VoyagePlan]
    voyage_docs: dict[str, dict]
    editions: dict[tuple[str, int], RoutePlan]
    holder_token: str | None  # token on the live command_authority lease, as this receiver hears it
    conn_held: bool  # this process holds that lease
    guidance: bool  # guidance is enabled, so an off-track order can be held
    order: GuidanceOrder
    settings: dict = field(default_factory=dict)  # NavigationSettings in force (keelson#322)
    fixed: tuple = ()  # settings it has but cannot change while it runs


@dataclass
class Outcome:
    result: str
    detail: str = ""
    voyage_id: str | None = None
    order: GuidanceOrder | None = None  # set: the order in force from now
    voyages: list[tuple[VoyagePlan, dict]] = field(default_factory=list)  # to write and publish
    editions: list[tuple[RoutePlan, dict]] = field(default_factory=list)  # copies to write/publish
    publish_latest: list[tuple[str, dict]] = field(default_factory=list)  # (route_id, route dict)
    decisions: list[tuple[str, str]] = field(default_factory=list)  # (kind, summary)
    settings: dict | None = None  # set_settings: the changes to apply, validated whole
    # The order came from the live conn holder, presenting its token, while this process does not
    # hold the conn: the process carries it out under that holder's conn (navigation_conduct).
    from_holder: bool = False
    # An exercise instructor's order given while an operator holds the conn (Ted, 2026-11-25): it is
    # carried out under that operator's conn, which stays theirs, and reported as an override.
    instructed: bool = False

    @property
    def accepted(self) -> bool:
        return self.result == ACCEPTED


class Controller:
    def __init__(self, conf: dict, id_factory=None):
        import uuid

        self.site = conf["conduct"]["controller_site"]
        self.permitted = frozenset(conf["control"]["controllers"])
        self.instructors = frozenset(conf["control"].get("instructors") or ())
        self._new_id = id_factory or (lambda: uuid.uuid4().hex)

    # -- who may call --------------------------------------------------------------------------

    def refusal(self, request: dict, ctx: Context) -> str | None:
        """Why this caller may not direct the process, or None when it may (NavigationControl.proto,
        WHO MAY CALL)."""
        caller = request.get("caller") or {}
        controller = caller.get("controller_id") or ""
        if not controller:
            return "no caller.controller_id"
        # An exercise instructor (a simulator's station) may order her at any time: the
        # instructor overrides the order, never the conn (Ted, 2026-11-25).
        if controller in self.instructors:
            return None
        token = caller.get("authority_token")
        if token and ctx.holder_token is not None and token == ctx.holder_token:
            return None
        # A permitted controller (a scenario runner) may direct this process while it holds the
        # conn, or while nobody does: then it overrides nobody. Once anyone else holds the conn,
        # only that holder may, so an operator taking the conn always outranks the list.
        if controller in self.permitted and (ctx.conn_held or ctx.holder_token is None):
            return None
        if ctx.holder_token is None:
            return (
                f"nobody holds the conn, and {controller} is not a controller this process permits"
            )
        if ctx.conn_held:
            return f"{controller} holds no conn and is not a controller this process permits"
        return f"{controller} does not hold the conn"

    @staticmethod
    def _from_holder(request: dict, ctx: Context) -> bool:
        """The caller presented the live lease's token and that lease is not this process's."""
        token = (request.get("caller") or {}).get("authority_token")
        return bool(token) and token == ctx.holder_token and not ctx.conn_held

    # -- procedures ----------------------------------------------------------------------------

    def decide(self, pending: Pending, ctx: Context) -> Outcome:
        """Raises InvalidRequest for a malformed request; otherwise every refusal is an Outcome."""
        if pending.procedure not in PROCEDURES:
            raise InvalidRequest(f"unknown procedure {pending.procedure!r}")
        if pending.procedure in ("get_navigation_state", "get_settings"):
            return Outcome(ACCEPTED)
        req = pending.request
        denied = self.refusal(req, ctx)
        if denied is not None:
            return Outcome(DENIED, denied)
        controller = req["caller"]["controller_id"]
        if pending.procedure in ("load_route", "set_guidance_order"):
            outcome = (
                self._load_route(pending, ctx, controller)
                if pending.procedure == "load_route"
                else self._set_order(req, ctx, controller)
            )
            outcome.from_holder = outcome.accepted and self._from_holder(req, ctx)
            outcome.instructed = (
                outcome.accepted
                and not outcome.from_holder
                and controller in self.instructors
                and ctx.holder_token is not None
                and not ctx.conn_held
            )
            return outcome
        if pending.procedure == "set_settings":
            return self._set_settings(req, ctx, controller)
        return self._set_status(req, ctx, controller)

    def _set_settings(self, req: dict, ctx: Context, controller: str) -> Outcome:
        try:
            changes, unsupported = settings_mod.parse(
                req.get("settings") or {}, ctx.settings, list(ctx.fixed)
            )
        except (settings_mod.SettingsError, TypeError, ValueError) as exc:
            raise InvalidRequest(str(exc)) from exc
        if changes is None:
            return Outcome(UNSUPPORTED, unsupported)
        if not changes:
            return Outcome(ACCEPTED, "no setting named: nothing changed")
        summary = f"settings: {settings_mod.describe(ctx.settings, changes)} ({controller})"
        # No NavigationDecision kind for settings yet; logged under guidance order.
        return Outcome(
            ACCEPTED, summary, settings=changes, decisions=[("KIND_GUIDANCE_ORDER", summary)]
        )

    def _order(self, payload, ctx: Context) -> GuidanceOrder:
        try:
            order = GuidanceOrder.from_payload(payload)
        except OrderError as exc:
            raise InvalidRequest(str(exc)) from exc
        return order

    def _load_route(self, pending: Pending, ctx: Context, controller: str) -> Outcome:
        req = pending.request
        ref = req.get("route_ref") or {}
        route_id = ref.get("route_id") or ""
        edition = int(ref.get("route_edition_number", 0))
        if not route_id:
            raise InvalidRequest("route_ref.route_id is empty")
        order = self._order(req["initial_order"], ctx) if "initial_order" in req else TRACK
        if order.kind is Kind.DP:
            return Outcome(UNSUPPORTED, NO_DP)
        if order.kind is not Kind.TRACK and not ctx.guidance:
            return Outcome(UNSUPPORTED, "guidance is disabled here: only a track order is held")
        source = req.get("source_entity_id") or ctx.entity_id
        voyage_id = req.get("voyage_id") or None
        if voyage_id is not None and ("/" in voyage_id or "*" in voyage_id):
            raise InvalidRequest("voyage_id must be a single key token")

        out = Outcome(ACCEPTED)
        if source == ctx.entity_id:
            plan = ctx.editions.get((route_id, edition))
            if plan is None:
                return Outcome(
                    FAILED,
                    f"route {route_id} edition {edition} is not in this vessel's tree; not "
                    "falling back to latest",
                )
        else:
            if pending.fetched is None:
                why = pending.fetch_error or "not found"
                return Outcome(FAILED, f"route {route_id} edition {edition} from {source}: {why}")
            route_doc, plan = pending.fetched
            if (plan.route_id, plan.edition) != (route_id, edition):
                return Outcome(FAILED, f"{source} returned a different route or edition")
            if (route_id, edition) not in ctx.editions:
                out.editions.append((plan, route_doc))
                newer = [n for (rid, n) in ctx.editions if rid == route_id and n > edition]
                if not newer:
                    out.publish_latest.append((route_id, route_doc))
        problems = plan.problems()
        if problems:
            return Outcome(FAILED, f"route {route_id} cannot be sailed: {'; '.join(problems)}")

        at = rfc3339(ctx.now_ns)
        route_ref = {"route_id": route_id, "route_edition_number": edition}
        where = "" if source == ctx.entity_id else f" copied from {source}"
        existing = ctx.voyages.get(voyage_id) if voyage_id else None
        if existing is not None and existing.status in _ENDED:
            return Outcome(DENIED, f"voyage {voyage_id} has ended; start a new one")
        if existing is not None:
            doc = dict(ctx.voyage_docs.get(voyage_id) or {"voyage_id": voyage_id})
            same_route = existing.route_ref is not None and existing.route_ref.route_id == route_id
            action = (
                "VOYAGE_AUDIT_ACTION_EDITION_BUMP"
                if same_route
                else "VOYAGE_AUDIT_ACTION_REROUTE_ACCEPTED"
            )
            if existing.status == PLANNED:
                action = "VOYAGE_AUDIT_ACTION_ACTIVATED"
                doc.setdefault("started_at", at)
            summary = f"voyage {voyage_id} now sails route {route_id} edition {edition}{where}"
        else:
            voyage_id = voyage_id or f"voy-{self._new_id()}"
            doc = {"version": "1", "voyage_id": voyage_id, "started_at": at}
            action = "VOYAGE_AUDIT_ACTION_ACTIVATED"
            summary = f"voyage {voyage_id} activated on route {route_id} edition {edition}{where}"

        for other in sorted(ctx.voyages.values(), key=lambda v: v.voyage_id):
            if other.status == ACTIVE and other.voyage_id != voyage_id:
                out.voyages.append(
                    self._transition(
                        other,
                        ctx.voyage_docs.get(other.voyage_id),
                        ABORTED,
                        "VOYAGE_AUDIT_ACTION_ABORTED",
                        f"superseded by voyage {voyage_id} (load_route by {controller})",
                        controller,
                        at,
                    )
                )
                out.decisions.append(
                    ("KIND_VOYAGE_CHANGED", f"voyage {other.voyage_id} aborted: superseded")
                )
        doc.update(status=ACTIVE, route_ref=route_ref)
        doc.pop("completed_at", None)
        _audit(doc, at, controller, self.site, action, f"load_route by {controller}{where}")
        out.voyages.append((VoyagePlan(voyage_id, ACTIVE, RouteRef(route_id, edition)), doc))
        out.voyage_id = voyage_id
        out.order = order
        out.detail = summary
        out.decisions.append(("KIND_VOYAGE_CHANGED", summary))
        out.decisions.append(("KIND_GUIDANCE_ORDER", f"order {order.describe()} ({controller})"))
        return out

    def _set_order(self, req: dict, ctx: Context, controller: str) -> Outcome:
        if "order" not in req:
            raise InvalidRequest("no order")
        order = self._order(req["order"], ctx)
        if order.kind is Kind.DP:
            return Outcome(UNSUPPORTED, NO_DP)
        if order.kind is not Kind.TRACK and not ctx.guidance:
            return Outcome(UNSUPPORTED, "guidance is disabled here: only a track order is held")
        active = [v for v in ctx.voyages.values() if v.status == ACTIVE]
        # A TRACK order follows a route, so it needs the one ACTIVE voyage that names it. Any other
        # order -- course, heading, hold, go_to, loiter -- steers by her own sensors and needs no
        # route: with no voyage it is taken alone, and lapses when a voyage starts (runtime
        # _expire_order). Two ACTIVE voyages are ambiguous for every order (Ted, 2026-11-25).
        if len(active) > 1 or (order.kind is Kind.TRACK and len(active) != 1):
            return Outcome(TEMPORARILY_REJECTED, "no single ACTIVE voyage to steer")
        if order.resume_from_waypoint_id is not None:
            ref = active[0].route_ref if active else None
            plan = ctx.editions.get((ref.route_id, ref.edition)) if ref is not None else None
            if plan is None or plan.index_of(order.resume_from_waypoint_id) is None:
                return Outcome(
                    FAILED,
                    f"waypoint {order.resume_from_waypoint_id} is not on the active route",
                )
        summary = f"order {order.describe()} ({controller})"
        if not active:
            summary += "; no voyage: it lapses when one starts"
        return Outcome(
            ACCEPTED,
            summary,
            active[0].voyage_id if active else None,
            order,
            decisions=[("KIND_GUIDANCE_ORDER", summary)],
        )

    def _set_status(self, req: dict, ctx: Context, controller: str) -> Outcome:
        voyage_id = req.get("voyage_id") or ""
        status = req.get("status") or ""
        if status not in (ACTIVE, PAUSED, COMPLETED, ABORTED):
            raise InvalidRequest(f"status {status!r} cannot be set here")
        voyage = ctx.voyages.get(voyage_id)
        if voyage is None:
            return Outcome(FAILED, f"voyage {voyage_id!r} is not a voyage of this vessel")
        if voyage.status == status:
            return Outcome(ACCEPTED, f"voyage {voyage_id} is already {status}", voyage_id)
        if voyage.status in _ENDED:
            return Outcome(DENIED, f"voyage {voyage_id} has ended ({voyage.status})")
        if status == ACTIVE:
            others = [v.voyage_id for v in ctx.voyages.values() if v.status == ACTIVE]
            if others:
                return Outcome(
                    TEMPORARILY_REJECTED,
                    f"voyage {others[0]} is ACTIVE; pause or end it first, or use load_route",
                )
            if voyage.route_ref is None or not voyage.route_ref.route_id:
                return Outcome(FAILED, f"voyage {voyage_id} pins no route")
            action = (
                "VOYAGE_AUDIT_ACTION_RESUMED"
                if voyage.status == PAUSED
                else "VOYAGE_AUDIT_ACTION_ACTIVATED"
            )
        elif status == PAUSED:
            if voyage.status != ACTIVE:
                return Outcome(DENIED, f"only an ACTIVE voyage can be paused, not {voyage.status}")
            action = "VOYAGE_AUDIT_ACTION_PAUSED"
        elif status == COMPLETED:
            action = "VOYAGE_AUDIT_ACTION_COMPLETED"
        else:
            action = "VOYAGE_AUDIT_ACTION_ABORTED"
        reason = req.get("reason") or ""
        detail = f"set_voyage_status by {controller}" + (f": {reason}" if reason else "")
        entry = self._transition(
            voyage,
            ctx.voyage_docs.get(voyage_id),
            status,
            action,
            detail,
            controller,
            rfc3339(ctx.now_ns),
        )
        summary = f"voyage {voyage_id} {voyage.status} -> {status} ({controller})"
        # An order is given for a voyage while it sails. Any change of status returns guidance to
        # the track, so a heading ordered before a pause does not come back with the resume.
        order = TRACK
        return Outcome(
            ACCEPTED,
            summary,
            voyage_id,
            order,
            voyages=[entry],
            decisions=[("KIND_VOYAGE_CHANGED", summary)],
        )

    def _transition(
        self,
        voyage: VoyagePlan,
        doc: dict | None,
        status: str,
        action: str,
        detail: str,
        controller: str,
        at: str,
    ) -> tuple[VoyagePlan, dict]:
        doc = dict(doc or {"voyage_id": voyage.voyage_id})
        doc["status"] = status
        if voyage.route_ref is not None and "route_ref" not in doc:
            doc["route_ref"] = {
                "route_id": voyage.route_ref.route_id,
                "route_edition_number": voyage.route_ref.edition,
            }
        if status == ACTIVE:
            doc.setdefault("started_at", at)
        if status in _ENDED:
            doc["completed_at"] = at
        _audit(doc, at, controller, self.site, action, detail)
        return VoyagePlan(voyage.voyage_id, status, voyage.route_ref), doc


def _audit(doc: dict, at: str, actor: str, site: str, action: str, detail: str) -> None:
    trail = list(doc.get("audit_trail") or [])
    trail.append(
        {"at": at, "actor_id": actor, "actor_site": site, "action": action, "detail": detail}
    )
    doc["audit_trail"] = trail
