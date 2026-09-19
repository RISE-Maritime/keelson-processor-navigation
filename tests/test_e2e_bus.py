"""Run the processor on an isolated local zenoh peer session and observe it on the bus.

The session has multicast scouting off and connects to nothing, so it cannot reach the shared RISE
router or put a test vessel in front of other Crowsnest users.
"""

from __future__ import annotations

import threading
import time

import keelson
import pytest

from nav import app as app_mod
from nav.bus.codec import Codec

pytestmark = pytest.mark.e2e


@pytest.fixture
def session():
    import zenoh

    zconf = zenoh.Config()
    zconf.insert_json5("mode", '"peer"')
    zconf.insert_json5("scouting/multicast/enabled", "false")
    zconf.insert_json5("listen/endpoints", "[]")
    zconf.insert_json5("connect/endpoints", "[]")
    s = zenoh.open(zconf)
    yield s
    s.close()


def test_publishes_idle_state_and_declares_liveliness(session, conf):
    conf["state_hz"] = 5.0
    received = []
    # Typed since keelson registered navigation_state (0.6.0-pre.18); no longer a raw_json rider.
    key = keelson.construct_pubsub_key(
        conf["realm"], conf["entity_id"], "navigation_state", "navigation/0/conduct"
    )
    sub = session.declare_subscriber(key, lambda s: received.append(bytes(s.payload.to_bytes())))

    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, conf, stop), daemon=True)
    worker.start()
    try:
        deadline = time.monotonic() + 5
        while len(received) < 2 and time.monotonic() < deadline:
            time.sleep(0.05)

        source_token = f"{conf['realm']}/@v0/{conf['entity_id']}/*/navigation/0/conduct"
        tokens = [str(r.ok.key_expr) for r in session.liveliness().get(source_token) if r.ok]
        subject_tokens = [str(r.ok.key_expr) for r in session.liveliness().get(key) if r.ok]
    finally:
        stop.set()
        worker.join(timeout=5)
        sub.undeclare()

    assert not worker.is_alive()
    assert len(received) >= 2
    decoded = Codec("rider").decode("navigation_state", received[0])
    assert decoded.subject == "navigation_state"
    assert decoded.payload["mode"] == "MODE_IDLE"
    assert tokens == [source_token]
    assert subject_tokens == [key]
