"""The deterministic processing loop, free of zenoh.

``Processor.tick(now_ns)`` is the only entry point. It reads a World snapshot, returns the outputs
for that instant and does no I/O, so the same code runs on a wall-clock timer in production and on
Slipway's tick fence in software-in-the-loop (Phase 5). What guidance wants on the helm is left in
``Processor.actuation`` for the bus layer to publish at its own rate.

Order within a tick: route selection, route_monitor, traffic, colreg, then conduct (lease,
authority, advice acceptance, the state machine), guidance, and the decision log.
"""

from __future__ import annotations

from dataclasses import dataclass

from . import config as config_mod
from .bus.codec import rfc3339
from .colreg.advisor import SUBJECTS as COLREG_SUBJECTS
from .colreg.advisor import Advisor, OwnState, advisor_config
from .colreg.records import DispositionRecord
from .conduct.authority import UNKNOWN, InForce, in_force, may_auto_accept, may_drive
from .conduct.fsm import Conduct, Gates
from .conduct.lease import LeaseBook, LeaseClient, LeaseConfig, is_navigation_process
from .conduct.state import Mode, NavigationState, RouteRef
from .control import settings as settings_mod
from .control.commands import Context, Controller, InvalidRequest, Outcome, Pending
from .control.orders import TRACK
from .guidance.axes import UNKNOWN as HELM_UNKNOWN
from .guidance.axes import Actuation, ownership
from .guidance.guide import MODULE as GUIDANCE
from .guidance.guide import SUBJECTS as GUIDANCE_SUBJECTS
from .guidance.guide import Guidance
from .guidance.track import KNOT_MPS, Own
from .log.decision_log import MODULE as DECISION_LOG
from .log.decision_log import SUBJECTS as DECISION_SUBJECTS
from .log.decision_log import DecisionLog
from .route_monitor.monitor import MODULE as ROUTE_MONITOR
from .route_monitor.monitor import RouteMonitor
from .route_monitor.select import Selection, select
from .traffic.cpa import Kinematics
from .traffic.monitor import MODULE as TRAFFIC
from .traffic.monitor import PUBLISHED_SUBJECTS as TRAFFIC_SUBJECTS
from .traffic.monitor import TrafficMonitor
from .world.latch import fresh
from .world.model import World

CONTROL = "control"
# keelson's navigation_decisions protocol: `encounter` and `navigation_advice` are INSTANCE keys,
# {producer}/{voyage_id or novoyage}/{id}, so two advisors on one vessel never share a key.
# `advice_disposition` is a SLOT, {voyage_id or novoyage}/{advice_id}, with no producer chunk:
# the advisor and the decision holder write one position, and `decided_by` says which.
_INSTANCE_KEYED = frozenset({"encounter", "navigation_advice"})
GUIDANCE_INPUTS = (
    "location_fix",
    "heading_true_north_deg",
    "course_over_ground_deg",
    "speed_over_ground_knots",
)
_LEASE_KINDS = {
    "taken": "KIND_CONN_TAKEN",
    "granted": "KIND_CONN_GRANTED",
    "handed": "KIND_CONN_LOST",  # granted on request: "went to another controller"
    "lost": "KIND_CONN_LOST",
    "released": "KIND_CONN_RELEASED",
    "requested": "KIND_CONN_REQUESTED",
}


@dataclass(frozen=True)
class Output:
    module: str
    subject: str
    payload: dict
    source_id: str | None = None  # None: the module's own source id
    target_id: str | None = None  # set: published under .../@target/{target_id}


class Processor:
    def __init__(self, conf: dict, world: World | None = None, id_factory=None, token_factory=None):
        self.conf = conf
        self.world = world or World()
        self._state_period_ns = int(1e9 / conf["state_hz"])
        self._next_state_ns: int | None = None
        self.monitor = RouteMonitor(conf, id_factory) if self.enabled(ROUTE_MONITOR) else None
        self.traffic = TrafficMonitor(conf) if self.enabled(TRAFFIC) else None
        self.advisor = (
            Advisor(advisor_config(conf), id_factory)
            if self.enabled("colreg") and self.traffic is not None
            else None
        )
        self.selection = Selection(None, "not started")

        cd = conf["conduct"]
        self.conduct = Conduct()
        self.lease_book = LeaseBook()
        self.lease = LeaseClient(
            LeaseConfig(
                vessel_id=conf["entity_id"],
                controller_id=config_mod.controller_id(conf),
                controller_site=cd["controller_site"],
                ttl_s=cd["lease_ttl_s"],
                heartbeat_s=cd["heartbeat_s"],
                request_interval_s=cd["request_interval_s"],
            ),
            token_factory,
        )
        self.authority = InForce(UNKNOWN, "not evaluated")
        self.guidance = Guidance(conf) if self.enabled(GUIDANCE) else None
        self.controller = Controller(conf, id_factory) if self.enabled(CONTROL) else None
        self.order = TRACK  # the guidance order in force (navigation_control/v1)
        self.colreg_active = True  # set_settings may pause the advisor (keelson#322)
        self._order_voyage: str | None = None  # the voyage it was given for
        self.decisions = DecisionLog(conf, id_factory) if self.enabled(DECISION_LOG) else None
        self.conn_held = False
        self._missing: tuple[str, ...] = ()
        self._own_payload: dict | None = None
        self._last_lease_event: str | None = None
        self._ignored_requesters: set[str] = set()

    def enabled(self, module: str) -> bool:
        return bool(self.conf["modules"].get(module))

    @property
    def needs_routes(self) -> bool:
        return self.enabled(ROUTE_MONITOR) or self.enabled("conduct") or self.enabled(CONTROL)

    @property
    def needs_targets(self) -> bool:
        return self.traffic is not None

    @property
    def needs_dispositions(self) -> bool:
        return self.advisor is not None

    @property
    def needs_leases(self) -> bool:
        return self.enabled("conduct")

    @property
    def actuation(self) -> Actuation:
        """What guidance wants on the helm now. Silence unless guidance is driving."""
        return self.guidance.actuation if self.guidance is not None else Actuation()

    def target_publications(self) -> dict[str, tuple[str, ...]]:
        """Subjects each module publishes under ``@target``; they also get a target-scoped token."""
        return {TRAFFIC: TRAFFIC_SUBJECTS} if self.traffic is not None else {}

    def publications(self) -> dict[str, tuple[str, ...]]:
        """Subjects each enabled module publishes under its own source id. Keys whose source slot
        carries an id (route_execution, envelope_exceedance) are reported by ``dynamic_keys``.
        command_authority and command_request are published under the vessel id, as keelson's
        other lease holders do, and are not listed here."""
        pubs: dict[str, tuple[str, ...]] = {}
        if self.enabled("conduct"):
            subjects = ["navigation_state"]
            if self.conf["conduct"]["auto_accept_advice"]:
                subjects.append("advice_disposition")
            pubs["conduct"] = tuple(subjects)
        if self.monitor is not None:
            pubs[ROUTE_MONITOR] = ("alarm",) if self.conf["route_monitor"]["alarms"] else ()
        pubs.update(self.target_publications())
        if self.advisor is not None:
            subjects = COLREG_SUBJECTS if self.conf["colreg"]["alarms"] else COLREG_SUBJECTS[:-1]
            pubs["colreg"] = tuple(subjects)
        if self.guidance is not None:
            pubs[GUIDANCE] = GUIDANCE_SUBJECTS
        if self.decisions is not None:
            pubs[DECISION_LOG] = DECISION_SUBJECTS
        return pubs

    def dynamic_keys(self) -> set[tuple[str, str]]:
        """(subject, source_id) pairs that should hold a subject liveliness token right now."""
        if self.monitor is None or self.monitor.status.progress is None:
            return set()
        return {("route_execution", self.monitor.status.execution_key_token)}

    def tick(self, now_ns: int) -> list[Output]:
        # navigation_control calls first, so this tick's route selection already sees them.
        answered: list[tuple[Pending, Outcome]] = []
        outputs: list[Output] = self._control_step(now_ns, answered) if self.controller else []

        snapshot = self.world.snapshot()
        self.selection = (
            select(snapshot, now_ns, self.conf["route_monitor"]["declared_stale_s"])
            if self.needs_routes
            else Selection(None, "no module needs routes")
        )
        self._expire_order(now_ns)
        if self.monitor is not None:
            outputs += [
                Output(ROUTE_MONITOR, e.subject, e.payload, e.source_id)
                for e in self.monitor.step(snapshot, self.selection, now_ns)
            ]
        if self.traffic is not None:
            tr = self.conf["traffic"]
            targets = self.world.targets.snapshot(
                now_ns, tr["target_max_age_s"], tr["static_max_age_s"]
            )
            outputs += [
                Output(TRAFFIC, e.subject, e.payload, e.source_id, e.target_id)
                for e in self.traffic.step(snapshot, targets, now_ns)
            ]
        if self.advisor is not None:
            for payload in self.world.take_dispositions():
                record = DispositionRecord.from_payload(payload)
                if record.decided_by != "navigation/colreg":
                    self.advisor.record_disposition(record)
            # Paused (set_settings colreg_advisor_active=false): no new encounters or advice.
            # Dispositions are still recorded above, so accepted advice is steered to its end.
            if self.colreg_active and self.traffic.evaluated_at_ns == now_ns:
                outputs += [
                    Output("colreg", e.subject, e.payload, self._colreg_key_tail(e))
                    for e in self.advisor.step(
                        now_ns,
                        self._own_state(snapshot, now_ns),
                        self.traffic.status.in_range,
                        self._leg_context(),
                        self.selection.voyage.voyage_id if self.selection.voyage else None,
                        self._visibility(snapshot, now_ns),
                    )
                ]
        if self.enabled("conduct"):
            outputs += self._conduct_step(snapshot, now_ns)
            if self._next_state_ns is None or now_ns >= self._next_state_ns:
                outputs.append(
                    Output("conduct", "navigation_state", self._state(now_ns).to_payload())
                )
                # Anchor to the schedule so jitter does not accumulate; skip missed slots.
                if self._next_state_ns is None:
                    self._next_state_ns = now_ns
                while self._next_state_ns <= now_ns:
                    self._next_state_ns += self._state_period_ns
        if answered:
            state = self._state(now_ns).to_payload()
            settings_now = self.settings()
            for pending, outcome in answered:
                pending.answer(_response(pending.procedure, outcome, state, settings_now))
        if self.decisions is not None:
            outputs += [
                Output(DECISION_LOG, e.subject, e.payload, e.source_id)
                for e in self.decisions.step(now_ns)
            ]
        return outputs

    # -- control -------------------------------------------------------------------------------

    def _control_step(self, now_ns: int, answered: list) -> list[Output]:
        pendings = self.world.take_commands()
        if not pendings:
            return []
        # Hear leases first: a scenario that grants the conn and calls straight after must be
        # judged against the grant, not the lease book of the previous tick.
        for payload, received_ns in self.world.take_leases():
            self.lease_book.hear(payload, received_ns)
        source = config_mod.module_source_id(self.conf, CONTROL)
        out: list[Output] = []
        for pending in pendings:
            snap = self.world.snapshot()  # fresh per call: a sequence sees its own effects
            settings_now, fixed_now = self.settings()
            holder = self.lease_book.holder(now_ns)
            ctx = Context(
                now_ns=now_ns,
                entity_id=self.conf["entity_id"],
                voyages=snap.voyages,
                voyage_docs=snap.voyage_docs,
                editions=snap.editions,
                holder_token=holder.token if holder is not None else None,
                conn_held=self.lease.held(self.lease_book, now_ns),
                guidance=self.guidance is not None,
                order=self.order,
                settings=settings_now,
                fixed=tuple(fixed_now),
            )
            try:
                outcome = self.controller.decide(pending, ctx)
            except InvalidRequest as exc:
                pending.refuse(str(exc))
                continue
            if outcome.accepted:
                for plan, doc in outcome.editions:
                    self.world.put_edition(plan)
                    tail = f"{plan.route_id}/edition/{plan.edition}"
                    out.append(Output(CONTROL, "route", doc, source_id=tail))
                for route_id, doc in outcome.publish_latest:
                    out.append(Output(CONTROL, "route", doc, source_id=f"{route_id}/latest"))
                for voyage, doc in outcome.voyages:
                    self.world.put_voyage(voyage, doc)
                    out.append(Output(CONTROL, "voyage", doc, source_id=voyage.voyage_id))
                for kind, summary in outcome.decisions:
                    self._decide(now_ns, kind, source, summary)
                if outcome.order is not None:
                    self._take_order(outcome.order, outcome.voyage_id)
                if outcome.settings:
                    self._apply_settings(outcome.settings)
            answered.append((pending, outcome))
        return out

    # -- settings (navigation_control/v1, keelson#322) ------------------------------------------

    def settings(self) -> tuple[dict, list[str]]:
        """The NavigationSettings in force and which are fixed (control/settings.py)."""
        g = self.guidance
        return settings_mod.current(
            conf=self.conf,
            advisor_built=self.advisor is not None,
            guidance_built=g is not None,
            traffic_built=self.traffic is not None,
            monitor_built=self.monitor is not None,
            colreg_active=self.colreg_active,
            default_speed_knots=g.speed.default_sog_knots if g else None,
            goto_radius_m=g.goto_radius_default_m if g else None,
            loiter_return_knots=g.loiter_return_default_knots if g else None,
        )

    def _apply_settings(self, changes: dict) -> None:
        """Validated whole by the controller; applied here in one tick."""
        if "colreg_advisor_active" in changes:
            self.colreg_active = changes["colreg_advisor_active"]
        if "auto_accept_advice" in changes:
            self.conf["conduct"]["auto_accept_advice"] = changes["auto_accept_advice"]
        g = self.guidance
        if g is not None:
            g.set_defaults(
                default_sog_knots=changes.get("default_speed_knots"),
                goto_radius_m=changes.get("go_to_arrival_radius_m"),
                loiter_return_knots=changes.get("loiter_return_speed_knots"),
            )
        if "traffic_range_m" in changes:
            self.conf["traffic"]["range_limit_m"] = changes["traffic_range_m"]
        if "route_alarms" in changes:
            self.conf["route_monitor"]["alarms"] = changes["route_alarms"]

    def _take_order(self, order, voyage_id: str | None) -> None:
        previous, self.order, self._order_voyage = self.order, order, voyage_id
        if self.guidance is not None:
            plan = None
            snap = self.world.snapshot()
            voyage = snap.voyages.get(voyage_id) if voyage_id else None
            if voyage is not None and voyage.route_ref is not None:
                plan = snap.editions.get((voyage.route_ref.route_id, voyage.route_ref.edition))
            self.guidance.take_order(order, previous, plan)

    def _expire_order(self, now_ns: int) -> None:
        """An order is given for a voyage. Once that voyage is no longer the one selected — ended,
        paused, replaced by another station's publish — guidance returns to the track, so a heading
        ordered for one voyage never steers the next."""
        if self.order == TRACK:
            return
        voyage = self.selection.voyage
        if voyage is not None and voyage.voyage_id == self._order_voyage:
            return
        was, given_for = self.order, self._order_voyage
        self._take_order(TRACK, voyage.voyage_id if voyage is not None else None)
        self._decide(
            now_ns,
            "KIND_GUIDANCE_ORDER",
            config_mod.module_source_id(self.conf, CONTROL),
            f"order {was.describe()} lapsed: voyage {given_for} is no longer the one selected",
        )

    # -- conduct -------------------------------------------------------------------------------

    def _request_to_answer(self, now_ns: int) -> dict | None:
        """The newest command_request to answer with a grant: an operator's, for this vessel,
        still fresh. Another navigation process's request is never granted -- logged once."""
        cd = self.conf["conduct"]
        me = self.lease.cfg.controller_id
        fresh_ns = int(cd["lease_ttl_s"] * 1e9)
        chosen = None
        for payload, received_ns in self.world.take_requests():
            requester = payload.get("requester_id") or ""
            if (
                not requester
                or requester == me
                or payload.get("vessel_id") != self.conf["entity_id"]
                or now_ns - received_ns > fresh_ns
            ):
                continue
            if is_navigation_process(requester):
                if requester not in self._ignored_requesters:
                    self._ignored_requesters.add(requester)
                    self._decide(
                        now_ns,
                        "KIND_CONN_REQUESTED",
                        config_mod.module_source_id(self.conf, "conduct"),
                        f"request from {requester} ignored: another navigation process",
                    )
                continue
            chosen = payload
        return chosen if cd["yield_on_request"] else None

    def _conduct_step(self, snapshot, now_ns: int) -> list[Output]:
        cd = self.conf["conduct"]
        out: list[Output] = []
        for payload, received_ns in self.world.take_leases():
            self.lease_book.hear(payload, received_ns)
        yield_to = self._request_to_answer(now_ns)
        for kind, detail in self.world.take_helm_events():
            self._decide(now_ns, kind, config_mod.module_source_id(self.conf, GUIDANCE), detail)

        self.authority = in_force(
            snapshot.inputs.get("operational_authority"),
            now_ns,
            cd["authority_max_age_s"],
            cd["fixed_authority_level"],
        )
        plan = self.selection.plan
        own = self._guidance_own(snapshot, now_ns)
        track = self.guidance.observe(plan, own) if self.guidance is not None else None
        station_held = (
            self.guidance.station(self.order, own) if self.guidance is not None else False
        )
        helm = self._helm(now_ns)

        if self.advisor is not None and cd["auto_accept_advice"]:
            out += self._auto_accept(now_ns)
        accepted = self.advisor.accepted_advice(now_ns) if self.advisor is not None else None

        progress = self.monitor.status.progress if self.monitor is not None else None
        arrived = track.arrived if track is not None else bool(progress and progress.arrived)
        self.conn_held = self.lease.held(self.lease_book, now_ns)
        route_reason = self.selection.reason
        if self.monitor is None and plan is not None:
            route_reason += "; route_monitor disabled"
        step = self.conduct.step(
            Gates(
                route=self.selection.execution_key_token if plan is not None else None,
                route_reason=route_reason,
                guidance=self.guidance is not None,
                conn_held=self.conn_held,
                conn_token=self.lease.token if self.conn_held else None,
                conn_self_issued=self.conn_held and self.lease.token in self.lease.self_issued,
                may_drive=may_drive(self.authority.level),
                authority_note=self.authority.note,
                ownship_fresh=not self._missing,
                missing_inputs=self._missing,
                helm=helm,
                arrived=arrived,
                avoiding=accepted is not None,
                order=self.order.kind.value,
                station_held=station_held,
            )
        )
        if step.changed:
            self._decide(
                now_ns,
                "KIND_MODE_TRANSITION",
                config_mod.module_source_id(self.conf, "conduct"),
                f"{step.before.value} -> {step.mode.value}"
                + (f" ({step.sub_mode.value})" if step.mode is Mode.UNDERWAY else ""),
                reason=step.reason,
                mode_before=step.before.value,
                mode_after=step.mode.value,
                advice_id=accepted.advice_id if accepted is not None else None,
            )

        mode = step.mode
        want = (
            self.guidance is not None
            and plan is not None
            and mode in (Mode.READY, Mode.UNDERWAY, Mode.FALLBACK)
        )
        may_take = (
            cd["take_conn_when_free"]
            and mode is Mode.READY
            and may_drive(self.authority.level)
            and not self._missing
            and helm in ("free", "ours")
        )
        publish, events = self.lease.step(
            self.lease_book,
            now_ns,
            want=want,
            may_take=may_take,
            may_request=cd["request_conn"],
            yield_to=yield_to,
        )
        for subject, payload in publish:
            if subject == "command_authority":
                self.lease_book.hear(payload, now_ns)
            out.append(Output("conduct", subject, payload, source_id=self.conf["entity_id"]))
        for event in events:
            if not (event.kind == "requested" and self._last_lease_event == "requested"):
                self._decide(
                    now_ns,
                    _LEASE_KINDS[event.kind],
                    config_mod.module_source_id(self.conf, "conduct"),
                    event.detail,
                )
            self._last_lease_event = event.kind
            if event.kind == "handed" and self.guidance is not None:
                # Free the operator's helm: the axis mapping is released once guidance goes
                # silent, by the same path as the end of a voyage.
                self.guidance.release_helm()
        self.conn_held = self.lease.held(self.lease_book, now_ns)

        if self.guidance is not None:
            ref = None
            if plan is not None:
                ref = {"route_id": plan.route_id, "route_edition_number": plan.edition}
            out += [
                Output(GUIDANCE, e.subject, e.payload, e.source_id)
                for e in self.guidance.step(
                    now_ns,
                    driving=mode is Mode.UNDERWAY,
                    ended=mode is Mode.ENDED,
                    helm=helm,
                    plan=plan,
                    own=own,
                    advice=accepted,
                    restricted_visibility=self._restricted(snapshot, now_ns),
                    voyage_id=self.selection.voyage.voyage_id if self.selection.voyage else None,
                    route_ref=ref,
                    order=self.order,
                )
            ]
        return out

    def _auto_accept(self, now_ns: int) -> list[Output]:
        """Accept the advisor's undecided advice when the conn is held, guidance is UNDERWAY and
        the authority level allows it for the advice's worst risk (docs/PLAN.md §1)."""
        if self.conduct.mode is not Mode.UNDERWAY or not self.lease.held(self.lease_book, now_ns):
            return []
        advice = self.advisor.undecided_advice(now_ns)
        if advice is None:
            return []
        risk = self.advisor.worst_risk(advice)
        level = self.authority.level
        if not may_auto_accept(level, risk):
            return []
        decided_by = config_mod.module_source_id(self.conf, "conduct")
        record = DispositionRecord(
            rfc3339(now_ns),
            advice.advice_id,
            advice.voyage_id,
            "STATE_ACCEPTED",
            decided_by=decided_by,
            reason=f"accepted by conduct at {level}; worst encounter risk {risk}",
        )
        self.advisor.record_disposition(record)
        self._decide(
            now_ns,
            "KIND_ADVICE_ACCEPTED",
            decided_by,
            f"accepted {advice.kind} advice {advice.advice_id}",
            reason=record.reason,
            advice_id=advice.advice_id,
            encounter_ids=tuple(advice.encounter_ids),
        )
        token = self.selection.execution_key_token or "novoyage"
        key_tail = f"{token}/{advice.advice_id}"
        return [Output("conduct", "advice_disposition", record.to_payload(), key_tail)]

    def _colreg_key_tail(self, e) -> str | None:
        """The advisor names `{voyage}/{id}`; the producer chunk is added here, where the module's
        own source id is known. A rider ignores the tail, so this changes proto mode only."""
        if e.subject in _INSTANCE_KEYED and e.source_id is not None:
            return f"{config_mod.module_source_id(self.conf, 'colreg')}/{e.source_id}"
        return e.source_id

    def _decide(self, now_ns: int, kind: str, decided_by: str, summary: str, **fields) -> None:
        if self.decisions is None:
            return
        self.decisions.record(
            now_ns,
            kind,
            decided_by,
            summary,
            voyage_id=self.selection.voyage.voyage_id if self.selection.voyage else None,
            ownship=self._own_payload,
            authority_level_in_force=self.authority.level,
            conn_held=self.conn_held,
            **fields,
        )

    def _guidance_own(self, snapshot, now_ns: int) -> Own | None:
        age = self.conf["route_monitor"]["max_state_age_s"]
        wanted = (*GUIDANCE_INPUTS, "yaw_rate_degps")
        samples = {s: fresh(snapshot.inputs.get(s), now_ns, age) for s in wanted}
        self._missing = tuple(s for s in GUIDANCE_INPUTS if samples[s] is None)
        position = samples["location_fix"]
        if position is not None:
            lat, lon = position.value
            self._own_payload = {"latitude_deg": lat, "longitude_deg": lon}
            for subject, name, scale in (
                ("heading_true_north_deg", "heading_deg", 1.0),
                ("course_over_ground_deg", "course_over_ground_deg", 1.0),
                ("speed_over_ground_knots", "sog_mps", KNOT_MPS),
            ):
                if samples[subject] is not None:
                    self._own_payload[name] = samples[subject].value * scale
        if self._missing:
            return None
        yaw = samples["yaw_rate_degps"]
        return Own(
            *position.value,
            heading_deg=samples["heading_true_north_deg"].value,
            cog_deg=samples["course_over_ground_deg"].value,
            sog_knots=samples["speed_over_ground_knots"].value,
            yaw_rate_degps=yaw.value if yaw is not None else None,
        )

    def _helm(self, now_ns: int) -> str:
        heard = self.world.helm()
        if heard is None or now_ns - heard[1] > self.conf["guidance"]["mapping_max_age_s"] * 1e9:
            return HELM_UNKNOWN
        return ownership(
            heard[0], self.conf["entity_id"], config_mod.module_source_id(self.conf, GUIDANCE)
        )

    def _restricted(self, snapshot, now_ns: int) -> bool:
        visibility = self._visibility(snapshot, now_ns)
        limit = self.conf["colreg"]["restricted_visibility_m"]
        return visibility is not None and visibility < limit

    # -- inputs for the other modules ----------------------------------------------------------

    def _own_state(self, snapshot, now_ns: int) -> OwnState | None:
        age = self.conf["route_monitor"]["max_state_age_s"]
        position = fresh(snapshot.inputs.get("location_fix"), now_ns, age)
        if position is None:
            return None
        cog = fresh(snapshot.inputs.get("course_over_ground_deg"), now_ns, age)
        sog = fresh(snapshot.inputs.get("speed_over_ground_knots"), now_ns, age)
        heading = fresh(snapshot.inputs.get("heading_true_north_deg"), now_ns, age)
        return OwnState(
            Kinematics(*position.value, cog.value if cog else None, sog.value if sog else None),
            heading.value if heading else None,
        )

    def _visibility(self, snapshot, now_ns: int) -> float | None:
        s = fresh(
            snapshot.inputs.get("visibility_range_m"),
            now_ns,
            self.conf["route_monitor"]["measurement_max_age_s"],
        )
        return s.value if s else None

    def _leg_context(self) -> str | None:
        plan = self.selection.plan
        progress = self.monitor.status.progress if self.monitor is not None else None
        if plan is None or progress is None:
            return None
        return plan.waypoints[progress.leg_index].operational_context

    def _state(self, now_ns: int) -> NavigationState:
        sel = self.selection
        plan = sel.plan
        status = self.monitor.status if self.monitor is not None else None
        progress = status.progress if status else None
        leg_from = progress.from_waypoint_id if progress else None
        if self.conduct.mode is Mode.UNDERWAY and self.guidance and self.guidance.state:
            leg_from = self.guidance.state.from_waypoint_id
        degraded = list(status.degraded_inputs) if status else []
        if self.guidance is not None and plan is not None:
            degraded += [s for s in self._missing if s not in degraded]
        return NavigationState(
            timestamp=rfc3339(now_ns),
            mode=self.conduct.mode,
            sub_mode=self.conduct.sub_mode,
            reason=self.conduct.reason,
            voyage_id=sel.voyage.voyage_id if sel.voyage and plan is not None else None,
            route_ref=RouteRef(plan.route_id, plan.edition) if plan is not None else None,
            current_leg_from_waypoint_id=leg_from if plan is not None else None,
            authority_level_in_force=self.authority.level,
            conn_held=self.conn_held,
            degraded_inputs=tuple(degraded),
            guidance_order=(
                self.order.to_payload() if sel.voyage is not None and plan is not None else None
            ),
        )


def _response(procedure: str, outcome: Outcome, state: dict, settings=None) -> dict:
    """The response message, as protobuf JSON, for one decided call."""
    if procedure == "get_navigation_state":
        return state
    if procedure in ("get_settings", "set_settings"):
        in_force, fixed = settings
        present = {k: v for k, v in in_force.items() if v is not None}
        if procedure == "get_settings":
            return {"settings": present, "fixed": fixed}
        return {"result": outcome.result, "detail": outcome.detail, "settings": present}
    response: dict = {"result": outcome.result, "detail": outcome.detail, "state": state}
    if procedure == "load_route" and outcome.accepted and outcome.voyage_id is not None:
        response["voyage_id"] = outcome.voyage_id
    return response
