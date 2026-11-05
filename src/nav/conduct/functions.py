"""Navigation authority per function: which navigation functions this process holds, at which
control method, and what that lets it do with its own advice (crowsnest-dev#90,
COMMAND-ARCHITECTURE §1.3).

The document is `NavigationAuthority`, embedded in the `command_authority` record on the
`{vessel}/navigation` slot, written by the NAVIGATIONAL_COMMAND holder through the authority
manager.
It names a holder and a SARUMS control method per function: 1 Operated, 2 Directed, 3 Delegated,
4 Monitored, 5 Autonomous. The authority divisor lies between 2 and 3. Below it a human holds the
function and this process works for them; from 3 up this process holds it.

What it decides here is the one decision this process makes of its own today: accepting its COLREG
advice (`collision_avoidance`). keelson `protocols/navigation_decisions.yaml` states the rule:

- no document, or the function held by someone else, or held below method 3: never accept. A human
  decides; method 2 confirms each decision.
- method 3, Delegated: the advice is the intent. A REJECTED disposition is a veto, and the advisor
  never offers a decided advice again. Until the advice has been published it waits: an intent no
  station could see was not delivered (crowsnest-dev#94). Inside `valid_from +
  intervention_window_ms` it waits.
  After it, silence lets it accept only if a veto COULD have arrived: the NAVIGATIONAL_COMMAND
  session is live.
  If it is not, `intent_delivery_required` refuses; otherwise `comms_loss_behaviour` decides --
  CONTINUE accepts, HOLD, FALLBACK_TO_METHOD_2 and SAFE_STATE do not. "No veto arrived" is not "no
  veto could arrive".
- method 4 Monitored and 5 Autonomous: accept, and the disposition informs.

The other functions act on a human's orders (navigation_control/v1), which is method 2's carry-out
and unchanged here.

Pure: no clock, no Zenoh. The payloads are protobuf JSON dicts (MessageToDict, proto field names).
"""

from __future__ import annotations

from dataclasses import dataclass, field

FUNCTIONS = (
    "route_execution",
    "track_keeping",
    "speed_control",
    "collision_avoidance",
    "route_modification",
    "emergency_avoidance",
)
METHODS = {
    "CONTROL_METHOD_OPERATED": 1,
    "CONTROL_METHOD_DIRECTED": 2,
    "CONTROL_METHOD_DELEGATED": 3,
    "CONTROL_METHOD_MONITORED": 4,
    "CONTROL_METHOD_AUTONOMOUS": 5,
}
DELEGATED = 3

ACCEPT, WAIT, REFUSE = "accept", "wait", "refuse"


@dataclass(frozen=True)
class Envelope:
    intervention_window_ms: int = 0
    intent_delivery_required: bool = False
    comms_loss_behaviour: str = "COMMS_LOSS_BEHAVIOUR_UNSPECIFIED"
    reroute_allowed: bool = False
    min_speed_knots: float | None = None
    max_speed_knots: float | None = None
    colreg_auto_accept_up_to: str = ""


@dataclass(frozen=True)
class Function:
    name: str
    holder: str
    holder_lease_id: str
    method: int
    envelope: Envelope


@dataclass(frozen=True)
class Document:
    revision: int
    functions: dict = field(default_factory=dict)  # name -> Function


@dataclass(frozen=True)
class Verdict:
    kind: str
    reason: str


def _function_name(value: str) -> str | None:
    prefix = "NAVIGATION_FUNCTION_"
    if not isinstance(value, str) or not value.startswith(prefix):
        return None
    name = value[len(prefix) :].lower()
    return name if name in FUNCTIONS else None


def read_document(record: dict) -> Document | None:
    """The `navigation_authority` of a `{vessel}/navigation` record; None when it has none."""
    nav = (record or {}).get("navigation_authority")
    if not isinstance(nav, dict):
        return None
    functions = {}
    for f in nav.get("functions") or []:
        name = _function_name(f.get("function"))
        if name is None:
            continue  # an unknown function grants nothing here
        e = f.get("envelope") or {}
        env = Envelope(
            intervention_window_ms=int(e.get("intervention_window_ms", 0) or 0),
            intent_delivery_required=bool(e.get("intent_delivery_required", False)),
            comms_loss_behaviour=str(
                e.get("comms_loss_behaviour", "COMMS_LOSS_BEHAVIOUR_UNSPECIFIED")
            ),
            reroute_allowed=bool(e.get("reroute_allowed", False)),
            min_speed_knots=e.get("min_speed_knots"),
            max_speed_knots=e.get("max_speed_knots"),
            colreg_auto_accept_up_to=str(e.get("colreg_auto_accept_up_to", "")),
        )
        functions[name] = Function(
            name,
            str(f.get("holder", "")),
            str(f.get("holder_lease_id", "")),
            METHODS.get(f.get("control_method"), 0),
            env,
        )
    # protobuf JSON writes a uint64 as a string.
    return Document(int(nav.get("revision", 0) or 0), functions)


def held(doc: Document | None, principal: str) -> tuple[Function, ...]:
    """The functions this process holds: its principal is the holder, at method 3 or above."""
    if doc is None:
        return ()
    return tuple(
        f
        for name in FUNCTIONS
        if (f := doc.functions.get(name)) is not None
        and f.holder == principal
        and f.method >= DELEGATED
    )


def advice_decision(
    doc: Document | None,
    principal: str,
    now_ns: int,
    valid_from_ns: int,
    nav_session_live: bool,
    delivered: bool = True,
) -> Verdict:
    """May this process accept its own undecided COLREG advice now? collision_avoidance's method."""
    if doc is None:
        return Verdict(
            REFUSE, "no navigation authority document: collision_avoidance is not delegated"
        )
    f = doc.functions.get("collision_avoidance")
    if f is None or f.holder != principal:
        holder = f.holder if f is not None else "nobody"
        return Verdict(REFUSE, f"collision_avoidance is held by {holder}, not this process")
    if f.method < DELEGATED:
        return Verdict(
            REFUSE, f"collision_avoidance is at method {f.method}: a human confirms each decision"
        )
    if f.method > DELEGATED:
        return Verdict(ACCEPT, f"collision_avoidance at method {f.method}")
    if not delivered:
        return Verdict(WAIT, "the intent has not been published yet")
    deadline = valid_from_ns + f.envelope.intervention_window_ms * 1_000_000
    if now_ns < deadline:
        left = (deadline - now_ns) / 1e9
        return Verdict(WAIT, f"the veto window is open ({left:.0f} s left)")
    if nav_session_live:
        return Verdict(ACCEPT, "the veto window closed with no veto, and a veto could have arrived")
    if f.envelope.intent_delivery_required:
        return Verdict(REFUSE, "no veto could arrive: the navigational command session is not live")
    if f.envelope.comms_loss_behaviour == "COMMS_LOSS_BEHAVIOUR_CONTINUE":
        return Verdict(
            ACCEPT, "no veto could arrive, and collision_avoidance continues on comms loss"
        )
    return Verdict(
        REFUSE,
        "no veto could arrive, and collision_avoidance's comms loss is "
        f"{f.envelope.comms_loss_behaviour}",
    )
