"""Subscriptions that fill the World: voyages, routes and measured inputs.

Voyages and route editions are documents the router stores, so a ``get`` at startup bootstraps
them. A declared route (``route/{source_id}``) is a heartbeat: a stored copy may be hours old, so
it enters not-live and only a live publication makes it monitorable.
"""

from __future__ import annotations

import logging
import time

from ..world.model import World
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
    def __init__(self, session, conf: dict, world: World, *, need_routes: bool, clock=time.time_ns):
        self.session = session
        self.conf = conf
        self.world = world
        self.need_routes = need_routes
        self.clock = clock
        self._subscribers = []
        self._warned: set[str] = set()

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

    def _subscribe(self, key: str, handler) -> None:
        self._subscribers.append(self.session.declare_subscriber(key, self._guard(handler)))
        logger.info("Subscribed %s", key)

    def _guard(self, handler):
        def wrapped(sample):
            try:
                handler(sample)
            except Exception as exc:  # a bad message must not kill the subscriber thread
                key = str(sample.key_expr)
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
        voyage = translate.voyage_plan(self._decode(sample, "voyage"))
        if voyage.voyage_id != tail[1][0]:
            raise translate.TranslateError(
                f"voyage_id {voyage.voyage_id!r} does not match its key token {tail[1][0]!r}"
            )
        self.world.put_voyage(voyage)

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

    def _input_handler(self, subject: str):
        def handle(sample):
            tail = pubsub_tail(str(sample.key_expr))
            source = "/".join(tail[1]) if tail else ""
            self.world.inputs.put(
                subject, translate.sample(self._decode(sample, subject), self.clock(), source)
            )

        return handle
