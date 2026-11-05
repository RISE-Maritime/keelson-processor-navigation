"""Subscriptions that fill the World: voyages, routes and measured inputs.

Voyages and route editions are documents the router stores, so a ``get`` at startup bootstraps
them. A declared route (``route/{source_id}``) is a heartbeat: a stored copy may be hours old, so
it enters not-live and only a live publication makes it monitorable.
"""

from __future__ import annotations

import logging
import time

from ..world.latch import Sample
from ..world.model import World
from ..world.targets import TARGET_SUBJECTS
from . import translate

logger = logging.getLogger(__name__)


def pubsub_tail(key: str) -> tuple[str, list[str]] | None:
    """``.../pubsub/{subject}/{rest...}`` -> (subject, rest chunks)."""
    parts = key.split("/")
    try:
        i = parts.index("pubsub")
    except ValueError:
        return None
    if len(parts) < i + 3:
        return None
    return parts[i + 1], parts[i + 2 :]


class Inputs:
    def __init__(
        self,
        session,
        conf: dict,
        world: World,
        *,
        need_routes: bool,
        need_targets: bool = False,
        need_dispositions: bool = False,
        need_leases: bool = False,
        clock=time.time_ns,
    ):
        self.session = session
        self.conf = conf
        self.world = world
        self.need_routes = need_routes
        self.need_targets = need_targets
        self.need_dispositions = need_dispositions
        self.need_leases = need_leases
        self.clock = clock
        self._subscribers = []
        self._warned: set[str] = set()
        # What this processor reads, by name, as it declares it: the answer to
        # subscriber/v1's get_subscriptions (see bus/subscriptions.py).
        self._subscriptions: dict[str, str] = {}

    def _key(self, subject: str, source: str) -> str:
        import keelson

        return keelson.construct_pubsub_key(
            self.conf["realm"], self.conf["entity_id"], subject, source
        )

    def start(self, bootstrap_timeout_s: float = 3.0) -> None:
        if self.need_routes:
            voyage_key = self._key("voyage", "*")
            route_key = self._key("route", "**")
            self._subscribe(voyage_key, lambda s: self._on_voyage(s))
            self._subscribe(route_key, lambda s: self._on_route(s, live=True))
        for subject, source in sorted(self.conf["inputs"].items()):
            self._subscribe(self._key(subject, source), self._input_handler(subject))
        if self.need_leases:
            # The lease rides source_id = the vessel id (Crowsnest's convention), but any source
            # that publishes one is heard: a lease this receiver ignores cannot lapse for it.
            key = self._key("command_authority", "**")
            self._subscribe(key, self._on_lease, label=key)
            # An operator's request for the conn, which the holder answers with a grant.
            key = self._key("command_request", "**")
            self._subscribe(key, self._on_request, label=key)
            source = self.conf["conduct"]["authority_source"]
            if source:
                self._subscribe(self._key("operational_authority", source), self._on_authority)
        if self.need_dispositions:
            # From the register, not the codec mode: keelson types advice_disposition now, so it is
            # typed in rider mode too, and a station's disposition arrives on its own subject.
            from .codec import Codec

            wire = Codec(self.conf["codec"]).wire_subject("advice_disposition")
            key = self._key(wire, "**")
            self._subscribe(key, self._disposition_handler(wire), label=key)
        if self.need_targets:
            for feed in self.conf["traffic"]["feeds"]:
                for subject in TARGET_SUBJECTS:
                    key = self._target_key(feed["entity"], subject, feed["source"])
                    self._subscribe(
                        key,
                        self._target_handler(feed, subject),
                        label=key,
                        name=f"{subject}@{feed['entity']}",
                    )

        if self.need_routes:
            for selector, handler in (
                (voyage_key, self._on_voyage),
                (route_key, lambda s: self._on_route(s, live=False)),
            ):
                try:
                    for reply in self.session.get(selector, timeout=bootstrap_timeout_s):
                        if reply.ok is not None:
                            handler(reply.ok)
                except Exception:
                    logger.exception("Bootstrap get failed for %s", selector)

    def close(self) -> None:
        for sub in self._subscribers:
            sub.undeclare()
        self._subscribers.clear()

    def _target_key(self, entity: str, subject: str, source: str) -> str:
        import keelson

        return keelson.construct_pubsub_key(
            self.conf["realm"], entity, subject, source, target_id="*"
        )

    @property
    def subscriptions(self) -> dict[str, str]:
        """Every key this processor reads, by name: its subject, or ``subject@entity`` for a
        traffic feed, where one subject is read from several entities. A copy."""
        return dict(self._subscriptions)

    def _subscribe(
        self, key: str, handler, label: str | None = None, name: str | None = None
    ) -> None:
        if name is None:
            tail = pubsub_tail(key)
            name = tail[0] if tail else key
        if name in self._subscriptions:
            # Two subscriptions under one name would make the map say less than was read.
            raise ValueError(
                f"two subscriptions named {name!r}: {self._subscriptions[name]}, {key}"
            )
        self._subscribers.append(self.session.declare_subscriber(key, self._guard(handler, label)))
        self._subscriptions[name] = key
        logger.info("Subscribed %s", key)

    def _guard(self, handler, label: str | None = None):
        """``label`` names rejections by subscription instead of by key, so a feed of hundreds of
        targets sending one bad subject records one rejection, not hundreds."""

        def wrapped(sample):
            try:
                handler(sample)
            except Exception as exc:  # a bad message must not kill the subscriber thread
                key = label or str(sample.key_expr)
                self.world.reject(key, str(exc))
                if key not in self._warned:
                    self._warned.add(key)
                    logger.warning("Rejected %s: %s", key, exc)

        return wrapped

    @staticmethod
    def _decode(sample, subject: str):
        import keelson

        _, _, payload = keelson.uncover(bytes(sample.payload.to_bytes()))
        return keelson.decode_protobuf_payload_from_type_name(
            payload, keelson.get_subject_schema(subject)
        )

    def _on_voyage(self, sample) -> None:
        tail = pubsub_tail(str(sample.key_expr))
        if tail is None or tail[0] != "voyage" or len(tail[1]) != 1:
            return
        from google.protobuf import json_format

        message = self._decode(sample, "voyage")
        voyage = translate.voyage_plan(message)
        if voyage.voyage_id != tail[1][0]:
            raise translate.TranslateError(
                f"voyage_id {voyage.voyage_id!r} does not match its key token {tail[1][0]!r}"
            )
        doc = json_format.MessageToDict(message, preserving_proto_field_name=True)
        self.world.put_voyage(voyage, doc)

    def _on_route(self, sample, live: bool) -> None:
        tail = pubsub_tail(str(sample.key_expr))
        if tail is None or tail[0] != "route":
            return
        rest = tail[1]
        if len(rest) == 3 and rest[1] == "edition":
            plan = translate.route_plan(self._decode(sample, "route"))
            if (plan.route_id, str(plan.edition)) != (rest[0], rest[2]):
                raise translate.TranslateError("route id or edition does not match its key")
            self.world.put_edition(plan)
        elif len(rest) == 1:
            if not self.conf["route_monitor"]["monitor_declared_route"]:
                return
            plan = translate.route_plan(self._decode(sample, "route"))
            self.world.put_declared(rest[0], plan, self.clock(), live)
        # route/{id}/latest is deliberately unused: a voyage pins an edition, never latest.

    def _disposition_handler(self, wire: str):
        from .codec import Codec, CodecError

        codec = Codec(self.conf["codec"])

        def handle(sample):
            try:
                decoded = codec.decode(wire, bytes(sample.payload.to_bytes()))
            except CodecError:
                return  # some other raw_json record on this vessel: not ours to judge
            if decoded.subject != "advice_disposition":
                return
            if "advice_id" not in decoded.payload:
                raise translate.TranslateError("advice_disposition without advice_id")
            self.world.put_disposition(decoded.payload)

        return handle

    def _target_handler(self, feed: dict, subject: str):
        source = f"{feed['entity']}/{feed['source']}"

        def handle(sample):
            key = str(sample.key_expr)
            _, sep, target_id = key.partition("/@target/")
            if not sep or not target_id or "/" in target_id:
                return
            self.world.targets.put(
                target_id,
                subject,
                translate.sample(self._decode(sample, subject), self.clock(), source),
            )

        return handle

    def _on_lease(self, sample) -> None:
        from google.protobuf import json_format

        message = self._decode(sample, "command_authority")
        payload = json_format.MessageToDict(message, preserving_proto_field_name=True)
        self.world.put_lease(payload, self.clock())

    def _on_request(self, sample) -> None:
        from google.protobuf import json_format

        message = self._decode(sample, "command_request")
        payload = json_format.MessageToDict(message, preserving_proto_field_name=True)
        self.world.put_request(payload, self.clock())

    def _on_authority(self, sample) -> None:
        message = self._decode(sample, "operational_authority")
        levels = message.DESCRIPTOR.fields_by_name["level"].enum_type.values_by_number
        level = levels.get(message.level)
        tail = pubsub_tail(str(sample.key_expr))
        self.world.inputs.put(
            "operational_authority",
            Sample(
                level.name if level else "AUTHORITY_LEVEL_UNKNOWN",
                self.clock(),
                translate._measured_ns(message),
                "/".join(tail[1]) if tail else "",
            ),
        )

    def _input_handler(self, subject: str):
        def handle(sample):
            tail = pubsub_tail(str(sample.key_expr))
            source = "/".join(tail[1]) if tail else ""
            self.world.inputs.put(
                subject, translate.sample(self._decode(sample, subject), self.clock(), source)
            )

        return handle
