"""navigation_control/v1 on the bus: four queryables, decided in the tick.

Each call is decoded here, queued on the World as a ``Pending`` and answered once the tick has
decided it (control/commands.py). Nothing here decides anything. The one piece of I/O a call needs
before it can be decided happens here too: ``load_route`` naming another vessel reads that vessel's
route edition from router storage, because the tick does no I/O.

zenoh-python runs each queryable on its own thread (keelson.scaffolding.rpc), so a call waiting for
the tick blocks only calls to the same procedure, never telemetry.

Served under responder id ``{source_base}`` (e.g. ``navigation/0``): the process, not a module, is
what a caller directs.
"""

from __future__ import annotations

import logging
import time

from ..control.commands import PROCEDURES, Pending
from . import translate

logger = logging.getLogger(__name__)

INTERFACE, VERSION = "navigation_control", "v1"

_REQUESTS = {
    "load_route": "LoadRouteRequest",
    "set_guidance_order": "SetGuidanceOrderRequest",
    "set_voyage_status": "SetVoyageStatusRequest",
    "get_navigation_state": "GetNavigationStateRequest",
}
_RESPONSES = {
    "load_route": "LoadRouteResponse",
    "set_guidance_order": "SetGuidanceOrderResponse",
    "set_voyage_status": "SetVoyageStatusResponse",
    "get_settings": "GetSettingsResponse",
    "set_settings": "SetSettingsResponse",
}


class ControlServer:
    def __init__(self, session, conf: dict, world, clock=time.time_ns):
        self.session = session
        self.conf = conf
        self.world = world
        self.clock = clock
        self.ct = conf["control"]
        self.server = None

    @property
    def responder_id(self) -> str:
        return self.conf["source_base"]

    def start(self) -> None:
        from keelson.scaffolding.rpc import serve_rpc

        self.server = serve_rpc(
            self.session,
            base_path=self.conf["realm"],
            entity_id=self.conf["entity_id"],
            interface=INTERFACE,
            version=VERSION,
            responder_id=self.responder_id,
            handlers={p: self._handler(p) for p in PROCEDURES},
            log=logger,
        )

    def close(self) -> None:
        if self.server is None:
            return
        for queryable in self.server.queryables:
            queryable.undeclare()
        if self.server.liveliness_token is not None:
            self.server.liveliness_token.undeclare()
        self.server = None

    def _handler(self, procedure: str):
        def handle(op) -> None:
            from google.protobuf import json_format
            from keelson.interfaces import NavigationControl_pb2 as nc
            from keelson.interfaces.ErrorResponse_pb2 import ErrorResponse

            try:
                request = getattr(nc, _REQUESTS[procedure]).FromString(op.request_bytes)
            except Exception as exc:
                op.reply_err(f"undecodable request: {exc}", ErrorResponse.Code.INVALID_ARGUMENT)
                return
            pending = Pending(
                procedure,
                json_format.MessageToDict(request, preserving_proto_field_name=True),
                self.clock(),
            )
            if procedure == "load_route":
                self._fetch(pending)
            self.world.put_command(pending)
            if not pending.done.wait(self.ct["reply_timeout_s"]):
                op.reply_err(
                    "the navigation process did not decide the call in time",
                    ErrorResponse.Code.UNAVAILABLE,
                )
                return
            if pending.error is not None:
                op.reply_err(pending.error, ErrorResponse.Code.INVALID_ARGUMENT)
                return
            op.reply_ok(_encode(procedure, pending.response))

        return handle

    def _fetch(self, pending: Pending) -> None:
        """For a route on another vessel: read that exact edition from router storage. The tick
        refuses the call if nothing came back; it never falls back to ``latest``."""
        import keelson

        req = pending.request
        source = req.get("source_entity_id") or ""
        if not source or source == self.conf["entity_id"]:
            return
        ref = req.get("route_ref") or {}
        route_id = ref.get("route_id") or ""
        edition = int(ref.get("route_edition_number", 0))
        if not route_id or "/" in source or "*" in source:
            pending.fetch_error = "unusable source_entity_id or route_id"
            return
        key = keelson.construct_pubsub_key(
            self.conf["realm"], source, "route", f"{route_id}/edition/{edition}"
        )
        try:
            for reply in self.session.get(key, timeout=self.ct["fetch_timeout_s"]):
                if reply.ok is None:
                    continue
                route = _decode_route(bytes(reply.ok.payload.to_bytes()))
                pending.fetched = (_to_dict(route), translate.route_plan(route))
                return
            pending.fetch_error = f"no stored copy at {key}"
        except Exception as exc:  # a bad copy is a refusal, not a crashed handler
            pending.fetch_error = f"could not read {key}: {exc}"


def _decode_route(data: bytes):
    import keelson

    _, _, payload = keelson.uncover(data)
    return keelson.decode_protobuf_payload_from_type_name(
        payload, keelson.get_subject_schema("route")
    )


def _to_dict(message) -> dict:
    from google.protobuf import json_format

    return json_format.MessageToDict(message, preserving_proto_field_name=True)


def _encode(procedure: str, response: dict) -> bytes:
    from google.protobuf import json_format
    from keelson.interfaces import NavigationControl_pb2 as nc
    from keelson.payloads.NavigationState_pb2 import NavigationState

    cls = (
        NavigationState
        if procedure == "get_navigation_state"
        else getattr(nc, _RESPONSES[procedure])
    )
    return json_format.ParseDict(response, cls(), ignore_unknown_fields=False).SerializeToString()
