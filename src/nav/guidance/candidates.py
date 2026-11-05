"""Backend B: behind the motion gate. What guidance wants, as one ``GuidanceCandidate`` per DOF,
each naming why this process may DECIDE it and why it may EXECUTE it (crowsnest-dev#94, keelson
protocols/control_gate.yaml, I15). The gate checks the two independently and either can refuse.

DECISION
  NAV_FUNCTION {function, navigation_authority_revision}: the function that decides this output is
  delegated to this process, at control method 3 or above (conduct/functions.held).
  HUMAN_ORDER {order_id}: otherwise, carrying out a human's decision -- the guidance order in force,
  or an avoidance a human accepted ("advice/{advice_id}") -- under the human's conn. The id is the
  one this process publishes as ``navigation_state.order_id``, which the gate checks it against.
  Neither: nothing is proposed for that DOF.
EXECUTION
  DIRECT: the gate's mapping row names this process as the authority holder (it holds the conn).
  CARRY_OUT {control_mapping_id}: the row names a human holder and this process as its source.

Which function decides an output: steering is ``collision_avoidance`` while avoiding,
``track_keeping`` on the track, and no function under a course, heading, hold or position order
(only a human's order decides those); throttle is ``speed_control``.

THE POLITE YIELD. Nothing is proposed from a row whose holder is not the conn holder heard now, or
from a map read under another motion epoch than the gate's latest: a take-control silences this
process at once, before the next read of the map. The gate's fence is what actually stops it; this
only means it never relies on being stopped.

THIS BACKEND NEVER INSTALLS A MAPPING. Pure: no clock, no Zenoh.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..conduct import functions as F
from .axes import FREE, OTHER, OURS, UNKNOWN

STEERING, THROTTLE = "steering", "throttle"
OUTPUTS = (STEERING, THROTTLE)
NAV_FUNCTION, HUMAN_ORDER = "nav_function", "human_order"
DIRECT, CARRY_OUT = "direct", "carry_out"


@dataclass(frozen=True)
class GateRow:
    """One ControlMapping row of the gate's map in force (control_gate/v1 get_gate_state)."""

    dof: str
    mapping_id: str
    authority_holder: str
    execution_source: str


@dataclass(frozen=True)
class GateMap:
    rows: tuple[GateRow, ...]
    fence: tuple[str, int] | None  # (term, generation) the map was read under


@dataclass(frozen=True)
class Candidate:
    dof: str
    value_pct: float
    decision: str
    decision_id: str
    execution: str
    function: str = ""  # NAV_FUNCTION
    nav_revision: int = 0  # NAV_FUNCTION
    order_id: str = ""  # HUMAN_ORDER
    mapping_id: str = ""  # CARRY_OUT


def gate_ownership(gm: GateMap | None, source: str, dofs: dict) -> str:
    """The helm status conduct reads, from the gate's map: unknown (no answer), free (none of this
    vessel's configured DOFs is mapped), ours (every one is mapped to this source), other."""
    if gm is None:
        return UNKNOWN
    mapped = {r.dof: r for r in gm.rows if r.dof in dofs}
    if not mapped:
        return FREE
    if set(mapped) == set(dofs) and all(r.execution_source == source for r in mapped.values()):
        return OURS
    return OTHER


def deciding_function(output: str, avoiding: bool, order_kind: str) -> str | None:
    if output == THROTTLE:
        return "speed_control"
    if avoiding:
        return "collision_avoidance"
    return "track_keeping" if order_kind == "track" else None


def candidates(
    axes: tuple[float, float] | None,
    gm: GateMap | None,
    dofs: dict,
    source: str,
    principal: str,
    doc: F.Document | None,
    conn_holder: str | None,
    epoch: tuple[str, int] | None,
    avoiding: bool,
    advice_id: str | None,
    order_kind: str,
    order_id: str | None,
) -> tuple[tuple[Candidate, ...], tuple[str, ...]]:
    """The candidates to propose now, and why any DOF this process is mapped to gets none."""
    if axes is None:
        return (), ()
    if gm is None:
        return (), ("the gate's map has not been read",)
    if epoch is not None and gm.fence != epoch:
        return (), ("the gate's motion epoch changed; re-reading its map",)
    held = {f.name for f in F.held(doc, principal)}
    out, why = [], []
    for row in gm.rows:
        if row.execution_source != source or row.dof not in dofs:
            continue
        if row.authority_holder != principal and row.authority_holder != conn_holder:
            why.append(
                f"{row.dof}: the map names {row.authority_holder}, and the conn is "
                f"{conn_holder or 'nobody'}'s now"
            )
            continue
        output = dofs[row.dof]
        value = axes[0] if output == STEERING else axes[1]
        direct = row.authority_holder == principal
        execution = (
            {"execution": DIRECT}
            if direct
            else {"execution": CARRY_OUT, "mapping_id": row.mapping_id}
        )
        function = deciding_function(output, avoiding, order_kind)
        if function is not None and function in held:
            decision_id = advice_id if function == "collision_avoidance" and advice_id else None
            out.append(
                Candidate(
                    row.dof,
                    float(value),
                    NAV_FUNCTION,
                    decision_id or f"{function}/{doc.revision}",
                    function=function,
                    nav_revision=doc.revision,
                    **execution,
                )
            )
        elif not direct and order_id:
            out.append(
                Candidate(
                    row.dof, float(value), HUMAN_ORDER, order_id, order_id=order_id, **execution
                )
            )
        else:
            needs = function or "this order"
            why.append(
                f"{row.dof}: no basis -- {needs} is not delegated to this process"
                + ("" if direct else ", and no human order is in force")
            )
    return tuple(out), tuple(why)
