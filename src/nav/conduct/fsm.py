"""The navigation state machine: IDLE -> READY -> UNDERWAY{TRACK, AVOIDING} -> ARRIVED -> ENDED,
with FALLBACK on loss of anything guidance needs (docs/PLAN.md §2).

Pure. ``Gates`` is everything the machine may look at for one tick; ``Conduct.step`` returns the
mode for that tick. Two rules carry the safety case:

- Leaving FALLBACK needs a conn grant with a token other than the one held when it fell back, and
  not one this process issued itself. Pulling the lease must stop the vessel, so a processor that
  may take a free conn must not simply take it back.
- UNDERWAY is entered only with a live conn, a driving authority level, fresh own-ship inputs and a
  vehicle_control mapping that is free or already ours.
"""

from __future__ import annotations

from dataclasses import dataclass

from .state import Mode, SubMode


@dataclass(frozen=True)
class Gates:
    route: str | None  # the selection's execution token when a plan is selected, else None
    route_reason: str
    guidance: bool
    conn_held: bool
    conn_token: str | None
    conn_self_issued: bool
    may_drive: bool
    authority_note: str
    ownship_fresh: bool
    missing_inputs: tuple[str, ...]
    helm: str  # free | ours | other | unknown
    arrived: bool
    avoiding: bool


@dataclass(frozen=True)
class Step:
    mode: Mode
    sub_mode: SubMode
    reason: str
    changed: bool
    before: Mode


class Conduct:
    def __init__(self):
        self.mode = Mode.IDLE
        self.sub_mode = SubMode.UNSPECIFIED
        self.reason = "not started"
        self._fallback_token: str | None = None
        self._ended_route: str | None = None

    def step(self, g: Gates) -> Step:
        before, before_sub = self.mode, self.sub_mode
        mode, sub, reason = self._next(g)
        if mode is Mode.FALLBACK and before is not Mode.FALLBACK:
            self._fallback_token = g.conn_token
        if mode is Mode.ENDED and before is not Mode.ENDED:
            self._ended_route = g.route
        self.mode, self.sub_mode, self.reason = mode, sub, reason
        return Step(mode, sub, reason, (mode, sub) != (before, before_sub), before)

    def _next(self, g: Gates) -> tuple[Mode, SubMode, str]:
        none = SubMode.UNSPECIFIED
        sub = SubMode.AVOIDING if g.avoiding else SubMode.TRACK
        m = self.mode
        if m in (Mode.IDLE, Mode.READY):
            if g.route is None:
                return Mode.IDLE, none, g.route_reason
            if not g.guidance:
                return Mode.READY, none, g.route_reason
            blocker = self._blocker(g)
            if blocker is not None:
                return Mode.READY, none, f"{g.route_reason}; not underway: {blocker}"
            return Mode.UNDERWAY, sub, f"{g.route_reason}; conn held, {g.authority_note}"
        if m is Mode.UNDERWAY:
            loss = self._loss(g)
            if loss is not None:
                return Mode.FALLBACK, none, loss
            if g.arrived:
                return Mode.ARRIVED, none, "final waypoint reached"
            return Mode.UNDERWAY, sub, f"{g.route_reason}; conn held, {g.authority_note}"
        if m is Mode.FALLBACK:
            if g.conn_held and g.conn_token != self._fallback_token and not g.conn_self_issued:
                return Mode.READY, none, "conn granted again after fallback"
            if g.route is None and not g.conn_held:
                return Mode.IDLE, none, "fallback cleared: no route selected and conn not held"
            return Mode.FALLBACK, none, self.reason
        if m is Mode.ARRIVED:
            if not g.conn_held:
                return Mode.ENDED, none, "arrived; conn released"
            return Mode.ARRIVED, none, "final waypoint reached; releasing the conn"
        if g.route != self._ended_route:
            return Mode.IDLE, none, "voyage ended; the route selection changed"
        return Mode.ENDED, none, self.reason

    @staticmethod
    def _blocker(g: Gates) -> str | None:
        if not g.may_drive:
            return g.authority_note
        if not g.ownship_fresh:
            return f"own-ship inputs missing or stale: {', '.join(g.missing_inputs)}"
        if g.helm == "other":
            return "the vehicle_control mapping belongs to another controller"
        if g.helm == "unknown":
            return "no vehicle_control/v1 responder has answered"
        if not g.conn_held:
            return "conn not held"
        if g.arrived:
            return "already at the final waypoint"
        return None

    @staticmethod
    def _loss(g: Gates) -> str | None:
        if g.route is None:
            return f"route no longer selected: {g.route_reason}"
        if not g.conn_held:
            return "conn lost"
        if not g.may_drive:
            return f"authority no longer allows driving: {g.authority_note}"
        if not g.ownship_fresh:
            return f"own-ship inputs missing or stale: {', '.join(g.missing_inputs)}"
        if g.helm == "other":
            return "another controller took the vehicle_control mapping"
        return None
