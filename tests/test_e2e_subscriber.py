"""subscriber/v1 over an isolated local zenoh peer session: a running processor says what it
reads, refuses to be re-pointed, and is findable the way Slipway finds it (RPC liveliness)."""

from __future__ import annotations

import threading
import time

import keelson
import pytest
from keelson.interfaces.Subscriber_pb2 import Subscriptions

from nav import app as app_mod
from tests.test_e2e_bus import session  # noqa: F401  (fixture)

pytestmark = pytest.mark.e2e


def call(session, conf, procedure, payload=b"", timeout=5.0):  # noqa: F811
    key = keelson.construct_rpc_key(
        conf["realm"], conf["entity_id"], "subscriber", "v1", procedure, conf["source_base"]
    )
    for reply in session.get(key, payload=payload, timeout=timeout):
        if reply.ok is not None:
            return True, bytes(reply.ok.payload.to_bytes())
        return False, bytes(reply.err.payload.to_bytes())
    raise AssertionError(f"no reply on {key}")


def test_a_running_processor_says_what_it_reads(session, monitor_conf):  # noqa: F811
    conf = monitor_conf
    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, conf, stop), daemon=True)
    worker.start()
    try:
        deadline = time.monotonic() + 15
        answer = None
        while time.monotonic() < deadline:
            try:
                ok, payload = call(session, conf, "get_subscriptions")
            except AssertionError:  # not declared yet
                time.sleep(0.2)
                continue
            assert ok, payload
            answer = dict(Subscriptions.FromString(payload).subscriptions)
            break
        assert answer, "subscriber/v1 never answered"
        base = f"{conf['realm']}/@v0/{conf['entity_id']}/pubsub"
        for subject, source in conf["inputs"].items():
            assert answer[subject] == f"{base}/{subject}/{source}"

        ok, payload = call(session, conf, "set_subscriptions", Subscriptions().SerializeToString())
        assert not ok
        assert b"fixed by its configuration" in payload

        token = keelson.construct_rpc_interface_liveliness_key(
            conf["realm"], conf["entity_id"], "subscriber", "v1", conf["source_base"]
        )
        assert list(session.liveliness().get(token, timeout=2.0)), "no RPC liveliness token"
    finally:
        stop.set()
        worker.join(timeout=10)
