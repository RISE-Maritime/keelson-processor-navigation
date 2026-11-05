"""subscriber/v1: what this processor reads, said over the bus.

Slipway records, in every run's manifest, what the software under test subscribed to. Reading
truth is unrestricted on its bus, so a run proves more or less depending on whether the consumer
read the simulator's truth or only what a real vessel would have. A client's subscriptions are
invisible on the bus, so the only honest source is the consumer itself, and keelson's registered
`subscriber/v1` is the interface for saying it.

`get_subscriptions` answers ``{name: key}`` from `Inputs.subscriptions`: the keys actually
declared, not the configuration they came from. `set_subscriptions` is refused: this processor's
inputs are fixed by its configuration at start, and re-pointing them mid-run is not something it
does. Saying so is better than a success that changed nothing.
"""

from __future__ import annotations

import logging

logger = logging.getLogger(__name__)

INTERFACE = "subscriber"
VERSION = "v1"


class SubscriberServer:
    def __init__(self, session, conf: dict, inputs):
        self.session = session
        self.conf = conf
        self.inputs = inputs
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
            handlers={
                "get_subscriptions": self._get_subscriptions,
                "set_subscriptions": self._set_subscriptions,
            },
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

    def _get_subscriptions(self, op) -> None:
        from keelson.interfaces.Subscriber_pb2 import Subscriptions

        op.reply_ok(Subscriptions(subscriptions=self.inputs.subscriptions))

    def _set_subscriptions(self, op) -> None:
        from keelson.interfaces.ErrorResponse_pb2 import ErrorResponse

        op.reply_err(
            "this processor's inputs are fixed by its configuration at start; "
            "set_subscriptions is not offered",
            ErrorResponse.Code.UNSUPPORTED,
        )
