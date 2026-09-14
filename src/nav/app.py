"""Entry point: parse args, load config, open zenoh, run the tick loop until shutdown."""

from __future__ import annotations

import argparse
import logging
import time
from contextlib import ExitStack

from . import config as config_mod

logger = logging.getLogger(__name__)


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
    parser.add_argument("--source-base", help="Source id prefix (default: navigation/0).")
    parser.add_argument("--codec", choices=config_mod.CODECS, help="rider or proto.")
    return parser.parse_args(argv)


def run(session, conf: dict, stop) -> None:
    """Declare liveliness and publishers, then tick until ``stop.wait()`` returns True.

    ``stop`` is anything with ``wait(timeout) -> bool``: GracefulShutdown in production, a
    threading.Event in tests (signal handlers can only be installed on the main thread).
    """
    import keelson
    from keelson.scaffolding import declare_liveliness, declare_publisher_for_subject

    from .bus.codec import Codec
    from .runtime import Processor

    realm, entity_id = conf["realm"], conf["entity_id"]
    codec = Codec(conf["codec"])
    processor = Processor(conf)

    with ExitStack() as stack:
        publishers = {}
        for module, subjects in processor.publications().items():
            source_id = config_mod.module_source_id(conf, module)
            wire = sorted({codec.wire_subject(s) for s in subjects})
            stack.enter_context(
                declare_liveliness(session, realm, entity_id, source_id, pubsub_subjects=wire)
            )
            for subject in wire:
                key = keelson.construct_pubsub_key(realm, entity_id, subject, source_id)
                pub = declare_publisher_for_subject(session, key, subject)
                stack.callback(pub.undeclare)
                publishers[(module, subject)] = pub
                logger.info("Publishing %s", key)

        period = 1.0 / conf["tick_hz"]
        deadline = time.monotonic()
        while True:
            for out in processor.tick(time.time_ns()):
                wire = codec.wire_subject(out.subject)
                publishers[(out.module, wire)].put(codec.encode(out.subject, out.payload))
            deadline += period
            delay = deadline - time.monotonic()
            if delay < 0:
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
