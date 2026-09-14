"""COLREG wiring on an isolated local zenoh peer session.

A target on a collision course goes in on a real @target key; an encounter rider comes out on the
colreg module's own raw_json key; an operator's advice_disposition rider on the bus reaches the
advisor. No router, no multicast.
"""

from __future__ import annotations

import json
import threading
import time

import keelson
import pytest

from nav import app as app_mod
from nav.bus.codec import Codec
from tests.test_e2e_bus import session  # noqa: F401  (fixture)

pytestmark = pytest.mark.e2e


def enc(type_name, **fields):
    msg = keelson.get_protobuf_message_class_from_type_name(type_name)(**fields)
    return keelson.enclose(msg.SerializeToString())


def test_encounter_rider_and_inbound_disposition(session, traffic_conf, monkeypatch):  # noqa: F811
    conf = traffic_conf
    conf["modules"]["route_monitor"] = False
    conf["modules"]["colreg"] = True
    conf["colreg"]["alarms"] = False
    realm, vessel = conf["realm"], conf["entity_id"]
    own = lambda subject, source: keelson.construct_pubsub_key(realm, vessel, subject, source)  # noqa: E731
    ais = lambda subject: keelson.construct_pubsub_key(  # noqa: E731
        realm, "nav-test-ais", subject, "ais", target_id="mmsi_265000009"
    )

    # Capture the processor so the test can look at the advisor it built.
    built = {}
    from nav import runtime

    original = runtime.Processor.__init__

    def spy(self, *a, **kw):
        original(self, *a, **kw)
        built["proc"] = self

    monkeypatch.setattr(runtime.Processor, "__init__", spy)

    riders = []
    rider_key = own("raw_json", "navigation/0/colreg")
    wrong_keys = []
    sub = session.declare_subscriber(
        rider_key, lambda s: riders.append(bytes(s.payload.to_bytes()))
    )
    stray = session.declare_subscriber(
        own("raw_json", "*/**"), lambda s: wrong_keys.append(str(s.key_expr))
    )
    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, conf, stop), daemon=True)
    worker.start()
    codec = Codec("rider")
    disposition_sent = False
    try:
        deadline = time.monotonic() + 25
        lat, lon = 57.70, 11.90
        advice_id = None
        while time.monotonic() < deadline:
            session.put(
                own("location_fix", "gnss"),
                enc("foxglove.LocationFix", latitude=lat, longitude=lon),
            )
            session.put(
                own("course_over_ground_deg", "sim"), enc("keelson.TimestampedFloat", value=0.0)
            )
            session.put(
                own("speed_over_ground_knots", "sim"), enc("keelson.TimestampedFloat", value=10.0)
            )
            session.put(
                own("heading_true_north_deg", "sim"), enc("keelson.TimestampedFloat", value=0.0)
            )
            # 0.5 NM east, 0.5 NM north, heading west at 10 kn: collision course from starboard.
            session.put(
                ais("location_fix"),
                enc(
                    "foxglove.LocationFix",
                    latitude=lat + 0.5 / 60,
                    longitude=lon + 0.5 / (60 * 0.5348),
                ),
            )
            session.put(ais("course_over_ground_deg"), enc("keelson.TimestampedFloat", value=270.0))
            session.put(ais("speed_over_ground_knots"), enc("keelson.TimestampedFloat", value=10.0))

            records = [codec.decode("raw_json", r) for r in riders]
            advice = [d for d in records if d.subject == "navigation_advice"]
            if advice and not disposition_sent:
                advice_id = advice[0].payload["advice_id"]
                operator = {
                    "timestamp": "2026-09-14T12:00:00Z",
                    "advice_id": advice_id,
                    "state": "STATE_ACCEPTED",
                    "decided_by": "ted@ROC-1",
                    "decided_site": "ROC-1",
                    "reason": "e2e",
                }
                session.put(
                    own("raw_json", "crowsnest/roc-1"), codec.encode("advice_disposition", operator)
                )
                disposition_sent = True
            proc = built.get("proc")
            if disposition_sent and proc and advice_id in proc.advisor.dispositions:
                break
            time.sleep(0.2)
    finally:
        stop.set()
        worker.join(timeout=10)
        sub.undeclare()
        stray.undeclare()

    assert not worker.is_alive()
    records = [codec.decode("raw_json", r) for r in riders]
    encounters = [d.payload for d in records if d.subject == "encounter"]
    assert encounters and encounters[0]["own_role"] == "ROLE_GIVE_WAY"
    assert encounters[0]["target"] == {
        "entity_id": "nav-test-ais",
        "source_id": "ais",
        "target_id": "mmsi_265000009",
    }
    # Ids travel in the payload; nothing of ours landed on an id-shaped raw_json key.
    ours = own("raw_json", "navigation/0/")
    assert not [
        k
        for k in wrong_keys
        if k.startswith(ours) and k[len(ours) :].count("/") > 0  # an id below a module source
    ]
    assert disposition_sent, "no advice was published to accept"
    got = built["proc"].advisor.dispositions[advice_id]
    assert (got.state, got.decided_by) == ("STATE_ACCEPTED", "ted@ROC-1")
    assert json.dumps(encounters[0])  # payload is plain JSON
