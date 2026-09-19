"""COLREG wiring on an isolated local zenoh peer session.

A target on a collision course goes in on a real @target key; a typed encounter comes out on
keelson's instance key {producer}/{voyage}/{id} (protocols/navigation_decisions.yaml), and an
operator's typed advice_disposition on the slot {voyage}/{advice_id} reaches the advisor. No
router, no multicast.
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


def test_typed_encounter_and_inbound_disposition(session, traffic_conf, monkeypatch):  # noqa: F811
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

    # (subject, key, bytes) for every typed record the colreg module publishes.
    typed = []

    def capture(subject):
        return lambda s: typed.append((subject, str(s.key_expr), bytes(s.payload.to_bytes())))

    sub = session.declare_subscriber(own("encounter", "**"), capture("encounter"))
    sub_advice = session.declare_subscriber(
        own("navigation_advice", "**"), capture("navigation_advice")
    )
    riders = []
    stray = session.declare_subscriber(
        own("raw_json", "**"), lambda s: riders.append(str(s.key_expr))
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

            advice = [codec.decode(s, b) for s, _, b in list(typed) if s == "navigation_advice"]
            if advice and not disposition_sent:
                advice_id = advice[0].payload["advice_id"]
                operator = {
                    "timestamp": "2026-09-14T12:00:00Z",
                    "advice_id": advice_id,
                    "state": "STATE_ACCEPTED",
                    "decided_by": "ted@ROC-1",
                    "reason": "e2e",
                }
                # A slot: {voyage or novoyage}/{advice_id}, no producer chunk.
                session.put(
                    own("advice_disposition", f"novoyage/{advice_id}"),
                    codec.encode("advice_disposition", operator),
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
        sub_advice.undeclare()
        stray.undeclare()

    assert not worker.is_alive()
    encounters = [(k, codec.decode(s, b).payload) for s, k, b in typed if s == "encounter"]
    assert encounters
    key, first = encounters[0]
    assert first["own_role"] == "ROLE_GIVE_WAY"
    # The instance key keelson's protocol requires: the producer chunk, then voyage, then id.
    assert key == own("encounter", f"navigation/0/colreg/novoyage/{first['encounter_id']}")
    assert first["target"] == {
        "entity_id": "nav-test-ais",
        "source_id": "ais",
        "target_id": "mmsi_265000009",
    }
    advised = [k for s, k, _ in typed if s == "navigation_advice"]
    assert advised and all(
        k.startswith(own("navigation_advice", "navigation/0/colreg/novoyage/")) for k in advised
    )
    # keelson types all three now: none of them rides raw_json any more.
    assert not [k for k in riders if k.startswith(own("raw_json", "navigation/0/colreg"))]
    assert disposition_sent, "no advice was published to accept"
    got = built["proc"].advisor.dispositions[advice_id]
    assert (got.state, got.decided_by) == ("STATE_ACCEPTED", "ted@ROC-1")
    assert json.dumps(encounters[0])  # payload is plain JSON
