"""Entry point: parse args, load config, open zenoh, run the tick loop until shutdown."""

from __future__ import annotations

import argparse
import logging
import time
from collections import OrderedDict
from contextlib import ExitStack

from . import config as config_mod

logger = logging.getLogger(__name__)

_MAX_CACHED_PUBLISHERS = 16


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    from keelson.scaffolding import add_common_arguments

    parser = argparse.ArgumentParser(
        prog="navigation",
        description=(
            "Keelson processor that owns navigation of an autonomous vessel under automation."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    add_common_arguments(parser)
    parser.add_argument("-c", "--config", help="Path to a YAML config file.")
    parser.add_argument("-r", "--realm", help="Keelson realm / base path (default: rise).")
    parser.add_argument("-e", "--entity-id", help="The vessel this processor navigates.")
    parser.add_argument(
        "--slipway-instance",
        help="Slipway ADR-0139: the run's instance, applied to the vessel and its traffic feeds.",
    )
    parser.add_argument("--source-base", help="Source id prefix (default: navigation/0).")
    parser.add_argument("--codec", choices=config_mod.CODECS, help="rider or proto.")
    return parser.parse_args(argv)


class _Publishers:
    """Declared publishers by key, with QoS from the subject. Keys with an id in the source slot
    change over a run, so the cache is bounded."""

    def __init__(self, session):
        self.session = session
        self._pubs: OrderedDict[str, object] = OrderedDict()

    def put(self, key: str, subject: str, data: bytes) -> None:
        from keelson.scaffolding import declare_publisher_for_subject

        pub = self._pubs.get(key)
        if pub is None:
            pub = self._pubs[key] = declare_publisher_for_subject(self.session, key, subject)
            logger.info("Publishing %s", key)
            while len(self._pubs) > _MAX_CACHED_PUBLISHERS:
                self._pubs.popitem(last=False)[1].undeclare()
        self._pubs.move_to_end(key)
        pub.put(data)

    def close(self) -> None:
        for pub in self._pubs.values():
            pub.undeclare()
        self._pubs.clear()


class _DynamicTokens:
    """Subject liveliness tokens for keys that come and go (route_execution/{voyage})."""

    def __init__(self, session):
        self.session = session
        self._tokens: dict[str, object] = {}

    def set(self, keys: set[str]) -> None:
        for key in sorted(set(self._tokens) - keys):
            self._tokens.pop(key).undeclare()
        for key in sorted(keys - set(self._tokens)):
            self._tokens[key] = self.session.liveliness().declare_token(key)

    def close(self) -> None:
        self.set(set())


def run(session, conf: dict, stop) -> None:
    """Declare liveliness, subscribe, then tick until ``stop.wait()`` returns True.

    ``stop`` is anything with ``wait(timeout) -> bool``: GracefulShutdown in production, a
    threading.Event in tests (signal handlers can only be installed on the main thread).
    """
    import keelson
    from keelson.scaffolding import declare_liveliness, put

    from .bus.codec import RAW_JSON, Codec
    from .bus.inputs import Inputs
    from .runtime import Processor

    realm, entity_id = conf["realm"], conf["entity_id"]
    codec = Codec(conf["codec"])
    processor = Processor(conf)

    def key(subject: str, source_id: str, target_id: str | None = None) -> str:
        return keelson.construct_pubsub_key(realm, entity_id, subject, source_id, target_id)

    with ExitStack() as stack:
        for module, subjects in processor.publications().items():
            source_id = config_mod.module_source_id(conf, module)
            wire = sorted({codec.wire_subject(s) for s in subjects})
            stack.enter_context(
                declare_liveliness(session, realm, entity_id, source_id, pubsub_subjects=wire)
            )

        # Target-scoped subject tokens (keelson-target-liveliness proposal): declared in addition
        # to the plain token, because no wildcard crosses @target.
        target_tokens = _DynamicTokens(session)
        stack.callback(target_tokens.close)
        target_tokens.set(
            {
                key(codec.wire_subject(s), config_mod.module_source_id(conf, module), "*")
                for module, subjects in processor.target_publications().items()
                for s in subjects
            }
        )

        publishers = _Publishers(session)
        stack.callback(publishers.close)
        tokens = _DynamicTokens(session)
        stack.callback(tokens.close)

        inputs = Inputs(
            session,
            conf,
            processor.world,
            need_routes=processor.needs_routes,
            need_targets=processor.needs_targets,
            need_dispositions=processor.needs_dispositions,
            need_leases=processor.needs_leases,
        )
        stack.callback(inputs.close)
        inputs.start()

        if processor.controller is not None:
            from .bus.control import ControlServer

            control = ControlServer(session, conf, processor.world)
            stack.callback(control.close)
            control.start()

        helm = None
        if processor.guidance is not None:
            from .bus.vehicle_control import HelmBackend

            helm = HelmBackend(session, conf, processor.world)
            stack.callback(helm.close)
            helm.start()

        period = 1.0 / conf["tick_hz"]
        deadline = time.monotonic()
        while True:
            for out in processor.tick(time.time_ns()):
                wire = codec.wire_subject(out.subject)
                module_source = config_mod.module_source_id(conf, out.module)
                # A rider stays on its module's own raw_json key and carries its ids in the payload:
                # putting encounter or advice ids in the source slot of raw_json would spread one
                # module's records across the vessel's whole raw_json namespace.
                source_id = module_source if wire == RAW_JSON else (out.source_id or module_source)
                try:
                    data = codec.encode(out.subject, out.payload)
                    if out.target_id is not None:
                        # One key per target: a one-shot put, not a cached publisher per target.
                        put(session, key(wire, source_id, out.target_id), data)
                    else:
                        publishers.put(key(wire, source_id), wire, data)
                except Exception:
                    logger.exception("Failed to publish %s from %s", out.subject, out.module)
            if helm is not None:
                helm.apply(processor.actuation)
            tokens.set({key(s, src) for s, src in processor.dynamic_keys()})

            deadline += period
            delay = deadline - time.monotonic()
            if delay < 0:
                if -delay > period:
                    logger.warning("Tick overran by %.3f s", -delay)
                deadline = time.monotonic()
                delay = 0.0
            if stop.wait(delay):
                break
    logger.info("Stopped")


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    from keelson.scaffolding import GracefulShutdown, create_zenoh_config, setup_logging

    setup_logging(level=args.log_level)
    try:
        conf = config_mod.load(args)
    except config_mod.ConfigError as exc:
        logger.error("Configuration error: %s", exc)
        return 2

    import zenoh

    zconf = create_zenoh_config(
        mode=args.mode, connect=args.connect, listen=args.listen, zenoh_config=args.zenoh_config
    )
    session = zenoh.open(zconf)
    try:
        with GracefulShutdown() as shutdown:
            run(session, conf, shutdown)
    finally:
        session.close()
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
