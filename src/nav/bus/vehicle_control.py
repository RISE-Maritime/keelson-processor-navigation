"""Backend A on the bus: the vehicle_control/v1 mapping, and the axes at their own rate.

Two daemon threads, apart from the tick:

- The axis publisher repeats the tick's last ``Actuation`` at ``publish_hz`` (10-20 Hz, as the
  interface requires) and stops at its ``valid_until_ns``. A tick loop that stalls therefore goes
  silent, and silence is the vehicle's stop signal.
- The mapping thread reads ``get_control_mapping`` every ``mapping_poll_s`` into the World, and on
  the tick's request installs or releases this processor's mapping. It installs only when the
  vehicle reports an empty mapping, read immediately before: another controller's mapping is never
  overwritten, it is reported (VehicleControl has no token, so this is a read-then-write race that
  keelson cannot close; docs/PLAN.md §7).
"""

from __future__ import annotations

import logging
import threading
import time

from .. import config as config_mod
from ..guidance.axes import AXIS_SUBJECTS, FREE, OURS, Actuation, ownership, wanted_mapping

logger = logging.getLogger(__name__)


class RpcFailed(RuntimeError):
    pass


class HelmBackend:
    def __init__(self, session, conf: dict, world, clock=time.time_ns):
        self.session = session
        self.world = world
        self.clock = clock
        self.gd = conf["guidance"]
        self.realm, self.entity_id = conf["realm"], conf["entity_id"]
        self.source_id = config_mod.module_source_id(conf, "guidance")
        self._lock = threading.Lock()
        self._actuation = Actuation()
        self._want_install = False
        self._want_release = False
        self._stop = threading.Event()
        self._wake = threading.Event()
        self._threads: list[threading.Thread] = []
        self._publishers: dict[str, object] = {}
        self._refused: str | None = None
        self._warned_no_responder = False
        self.axis_puts = 0

    def start(self) -> None:
        import keelson
        from keelson.scaffolding import declare_publisher_for_subject

        for subject in AXIS_SUBJECTS.values():
            key = keelson.construct_pubsub_key(self.realm, self.entity_id, subject, self.source_id)
            self._publishers[subject] = declare_publisher_for_subject(self.session, key, subject)
        for target, name in ((self._publish_loop, "helm-axes"), (self._mapping_loop, "helm-map")):
            thread = threading.Thread(target=target, name=name, daemon=True)
            thread.start()
            self._threads.append(thread)

    def apply(self, actuation: Actuation) -> None:
        with self._lock:
            self._actuation = actuation
            if actuation.mapping == "install":
                self._want_install = True
            elif actuation.mapping == "release":
                self._want_release = True
        if actuation.mapping is not None:
            self._wake.set()

    def close(self) -> None:
        self._stop.set()
        self._wake.set()
        for thread in self._threads:
            thread.join(timeout=5)
        for pub in self._publishers.values():
            pub.undeclare()
        self._publishers.clear()

    def _publish_loop(self) -> None:
        import keelson

        float_cls = keelson.get_protobuf_message_class_from_type_name("keelson.TimestampedFloat")
        order = (AXIS_SUBJECTS["steering"], AXIS_SUBJECTS["throttle"])
        period = 1.0 / self.gd["publish_hz"]
        deadline = time.monotonic()
        while not self._stop.is_set():
            now = self.clock()
            with self._lock:
                act = self._actuation
            if act.axes is not None and now <= act.valid_until_ns:
                try:
                    for subject, value in zip(order, act.axes, strict=True):
                        msg = float_cls(value=float(value))
                        msg.timestamp.FromNanoseconds(now)
                        self._publishers[subject].put(
                            keelson.enclose(msg.SerializeToString(), enclosed_at=now)
                        )
                    self.axis_puts += 1
                except Exception:
                    logger.exception("Failed to publish axes")
            deadline += period
            delay = deadline - time.monotonic()
            if delay < 0:
                deadline, delay = time.monotonic(), 0.0
            self._stop.wait(delay)

    def _mapping_loop(self) -> None:
        while not self._stop.is_set():
            mapping = self._poll()
            with self._lock:
                install, release = self._want_install, self._want_release
                self._want_install = self._want_release = False
            status = ownership(mapping, self.entity_id, self.source_id)
            if install and status == FREE:
                if self._set(wanted_mapping(self.source_id, self.gd["max_axis_age_s"])):
                    self.world.put_helm_event(
                        "KIND_MAPPING_INSTALLED",
                        f"steering and throttle mapped to {self.source_id}",
                    )
                    self._refused = None
                    self._poll()
            elif install and status not in (FREE, OURS):
                signature = repr(mapping)
                if signature != self._refused:
                    self._refused = signature
                    self.world.put_helm_event(
                        "KIND_MAPPING_REFUSED",
                        f"vehicle_control mapping is {status}; not taking the helm",
                    )
                    logger.warning("vehicle_control mapping is %s; not taking the helm", status)
            if release and status == OURS and self._set({"axes": {}}):
                self.world.put_helm_event(
                    "KIND_MAPPING_RELEASED", "vehicle_control mapping cleared"
                )
                self._poll()
            self._wake.wait(self.gd["mapping_poll_s"])
            self._wake.clear()

    def _poll(self) -> dict | None:
        from google.protobuf import json_format
        from keelson.interfaces.VehicleControl_pb2 import ControlAxisMapping

        mapping = None
        try:
            data = self._call("get_control_mapping", None)
            if data is not None:
                mapping = json_format.MessageToDict(
                    ControlAxisMapping.FromString(data), preserving_proto_field_name=True
                )
                self._warned_no_responder = False
            elif not self._warned_no_responder:
                self._warned_no_responder = True
                logger.warning("No vehicle_control/v1 responder for %s", self.entity_id)
        except Exception as exc:
            logger.warning("get_control_mapping failed: %s", exc)
        self.world.put_helm(mapping, self.clock())
        return mapping

    def _set(self, mapping: dict) -> bool:
        from google.protobuf import json_format
        from keelson.interfaces.VehicleControl_pb2 import ControlAxisMapping

        request = json_format.ParseDict(mapping, ControlAxisMapping()).SerializeToString()
        try:
            if self._call("set_control_mapping", request) is None:
                logger.warning("set_control_mapping: no responder")
                return False
            return True
        except Exception as exc:
            logger.warning("set_control_mapping refused: %s", exc)
            self.world.put_helm_event("KIND_MAPPING_REFUSED", f"set_control_mapping failed: {exc}")
            return False

    def _call(self, procedure: str, payload: bytes | None) -> bytes | None:
        import keelson

        key = keelson.construct_rpc_key(
            self.realm, self.entity_id, "vehicle_control", "v1", procedure, self.gd["responder_id"]
        )
        kwargs = {"timeout": self.gd["rpc_timeout_s"]}
        if payload:
            kwargs["payload"] = payload
        for reply in self.session.get(key, **kwargs):
            if reply.ok is not None:
                return bytes(reply.ok.payload.to_bytes())
            raise RpcFailed(_error_text(bytes(reply.err.payload.to_bytes())))
        return None


def _error_text(data: bytes) -> str:
    try:
        from keelson.interfaces.ErrorResponse_pb2 import ErrorResponse

        err = ErrorResponse.FromString(data)
        return err.error_description or str(err.code)
    except Exception:
        return data.decode("utf-8", "replace")
