"""Backend B on the bus: guidance candidates to the motion gate (crowsnest-dev#94).

Two daemon threads, apart from the tick, as the axes backend has:

- The candidate publisher repeats the tick's last candidates at ``publish_hz`` and stops at the
  actuation's ``valid_until_ns``, so a stalled tick goes silent. Every sample carries this process's
  one stream: a random id drawn at start and a sequence rising across every DOF it proposes -- the
  gate orders per source, so one counter, never one per DOF.
- The map thread reads ``control_gate/v1 get_gate_state`` every ``mapping_poll_s`` into the World.
  It NEVER calls ``set_control_mapping``: behind a gate, the authority manager and the gate decide
  the map. A new motion epoch on ``resolved_control`` wakes it at once, and the tick proposes
  nothing until a map read under that epoch is in (the polite yield; guidance/candidates.py).

Keys: ``guidance_candidate/{source_base}/guidance/{dof}`` (keyed {producer}/{dof}); the payload's
``source`` is ``{source_base}``, the execution source the gate's map spells.
"""

from __future__ import annotations

import logging
import secrets
import threading
import time

from .. import config as config_mod
from ..guidance.axes import Actuation
from ..guidance.candidates import CARRY_OUT, NAV_FUNCTION, GateMap, GateRow

logger = logging.getLogger(__name__)

CLOCK_QUALITY = {"synchronised": 1, "free_running": 2}  # keelson.ClockQuality


class GateBackend:
    def __init__(self, session, conf: dict, world, clock=time.time_ns):
        self.session = session
        self.world = world
        self.clock = clock
        self.gd = conf["guidance"]
        self.gate = self.gd["gate"]
        self.realm, self.entity_id = conf["realm"], conf["entity_id"]
        self.source = conf["source_base"]
        self.producer = config_mod.module_source_id(conf, "guidance")
        self.stream_id = secrets.token_hex(16)  # a new incarnation on every start
        self.sequence = 0
        self._lock = threading.Lock()
        self._actuation = Actuation()
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._threads: list[threading.Thread] = []
        self._publishers: dict[str, object] = {}
        self._subscriber = None
        self._warned_no_responder = False
        self.candidate_puts = 0

    def start(self) -> None:
        import keelson
        from keelson.scaffolding import declare_publisher_for_subject

        for dof in self.gate["dofs"]:
            key = keelson.construct_pubsub_key(
                self.realm, self.entity_id, "guidance_candidate", f"{self.producer}/{dof}"
            )
            self._publishers[dof] = declare_publisher_for_subject(
                self.session, key, "guidance_candidate"
            )
        resolved = keelson.construct_pubsub_key(
            self.realm, self.entity_id, "resolved_control", self.entity_id
        )
        self._subscriber = self.session.declare_subscriber(resolved, self._on_resolved)
        for target, name in ((self._publish_loop, "gate-candidates"), (self._map_loop, "gate-map")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def apply(self, actuation: Actuation) -> None:
        # A mapping request is ignored here, always: this backend never installs a mapping.
        with self._lock:
            self._actuation = actuation

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(timeout=5)
        if self._subscriber is not None:
            self._subscriber.undeclare()
        for pub in self._publishers.values():
            pub.undeclare()
        self._publishers.clear()

    # -- the candidates ------------------------------------------------------------------------

    def message(self, candidate, now_ns: int):
        """One keelson.GuidanceCandidate, with the next sequence of this process's stream."""
        from keelson.payloads.GuidanceCandidate_pb2 import GuidanceCandidate
        from keelson.payloads.NavigationAuthority_pb2 import NavigationFunction

        self.sequence += 1
        msg = GuidanceCandidate(
            source=self.source,
            decision_id=candidate.decision_id,
            dof=candidate.dof,
            value_pct=candidate.value_pct,
        )
        msg.timestamp.FromNanoseconds(now_ns)
        if candidate.decision == NAV_FUNCTION:
            msg.nav_function.function = NavigationFunction.Value(
                f"NAVIGATION_FUNCTION_{candidate.function.upper()}"
            )
            msg.nav_function.navigation_authority_revision = candidate.nav_revision
        else:
            msg.human_order.order_id = candidate.order_id
        if candidate.execution == CARRY_OUT:
            msg.carry_out.control_mapping_id = candidate.mapping_id
        else:
            msg.direct.SetInParent()
        msg.stream.stream_id = self.stream_id
        msg.stream.sequence = self.sequence
        msg.stream.clock_quality = CLOCK_QUALITY[self.gate["clock_quality"]]
        return msg

    def _publish_loop(self) -> None:
        import keelson

        period = 1.0 / self.gd["publish_hz"]
        deadline = time.monotonic()
        while not self._stop.is_set():
            now = self.clock()
            with self._lock:
                act = self._actuation
            if act.candidates and now <= act.valid_until_ns:
                try:
                    for candidate in act.candidates:
                        pub = self._publishers.get(candidate.dof)
                        if pub is None:
                            continue  # not a DOF this vessel's config names
                        data = self.message(candidate, now).SerializeToString()
                        pub.put(keelson.enclose(data, enclosed_at=now))
                    self.candidate_puts += 1
                except Exception:
                    logger.exception("Failed to publish guidance candidates")
            deadline += period
            delay = deadline - time.monotonic()
            if delay < 0:
                deadline, delay = time.monotonic(), 0.0
            self._stop.wait(delay)

    # -- the gate's map ------------------------------------------------------------------------

    def _on_resolved(self, sample) -> None:
        try:
            import keelson
            from keelson.payloads.ResolvedControl_pb2 import ResolvedControl

            rc = ResolvedControl.FromString(keelson.uncover(sample.payload.to_bytes())[2])
            epoch = (
                (rc.motion_epoch.term, int(rc.motion_epoch.generation))
                if rc.HasField("motion_epoch")
                else None
            )
            if epoch != self.world.gate_epoch():
                self.world.put_gate_epoch(epoch)
                self._wake.set()  # re-read the map under the new epoch now
        except Exception:
            logger.exception("Could not read resolved_control")

    def _map_loop(self) -> None:
        while not self._stop.is_set():
            self.world.put_gate(self.poll(), self.clock())
            self._wake.wait(self.gd["mapping_poll_s"])
            self._wake.clear()

    def poll(self) -> GateMap | None:
        """The gate's map in force, or None when no gate answered."""
        from keelson.interfaces.ControlGate_pb2 import GateState, GetGateStateRequest

        try:
            data = self._call("get_gate_state", GetGateStateRequest().SerializeToString())
        except Exception as exc:
            logger.warning("get_gate_state failed: %s", exc)
            return None
        if data is None:
            if not self._warned_no_responder:
                self._warned_no_responder = True
                logger.warning("No control_gate/v1 responder for %s", self.entity_id)
            return None
        self._warned_no_responder = False
        state = GateState.FromString(data)
        rows = tuple(
            GateRow(m.dof, m.mapping_id, m.authority_holder, m.execution_source)
            for m in state.mappings
        )
        fence = (state.fence.term, int(state.fence.generation)) if state.HasField("fence") else None
        return GateMap(rows, fence)

    def _call(self, procedure: str, payload: bytes) -> bytes | None:
        import keelson

        from .vehicle_control import RpcFailed, _error_text

        key = keelson.construct_rpc_key(
            self.realm, self.entity_id, "control_gate", "v1", procedure, self.gate["responder_id"]
        )
        for reply in self.session.get(key, payload=payload, timeout=self.gd["rpc_timeout_s"]):
            if reply.ok is not None:
                return bytes(reply.ok.payload.to_bytes())
            raise RpcFailed(_error_text(bytes(reply.err.payload.to_bytes())))
        return None
