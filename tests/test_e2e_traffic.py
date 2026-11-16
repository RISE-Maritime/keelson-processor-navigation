"""Traffic on an isolated local zenoh peer session: a target on a real @target key goes in, CPA
comes out on the vessel's own @target keys, with a target-scoped liveliness token."""

from __future__ import annotations

import math
import threading
import time

import keelson
import pytest

from nav import app as app_mod
from tests.test_e2e_bus import session  # noqa: F401  (fixture)

pytestmark = pytest.mark.e2e


def enc(type_name, **fields):
    msg = keelson.get_protobuf_message_class_from_type_name(type_name)(**fields)
    return keelson.enclose(msg.SerializeToString())


def test_target_cpa_is_published_under_target(session, traffic_conf):  # noqa: F811
    traffic_conf["modules"]["route_monitor"] = False
    realm, vessel = traffic_conf["realm"], traffic_conf["entity_id"]
    own = lambda subject, source: keelson.construct_pubsub_key(realm, vessel, subject, source)  # noqa: E731
    ais = lambda subject: keelson.construct_pubsub_key(  # noqa: E731
        realm, "nav-test-ais", subject, "ais", target_id="mmsi_265000001"
    )

    received = {}
    cpa_key = keelson.construct_pubsub_key(
        realm, vessel, "target_cpa_m", "navigation/0/traffic", target_id="*"
    )
    sub = session.declare_subscriber(
        cpa_key, lambda s: received.setdefault(str(s.key_expr), bytes(s.payload.to_bytes()))
    )
    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, traffic_conf, stop), daemon=True)
    worker.start()
    tokens = []
    try:
        deadline = time.monotonic() + 20
        lat, lon = 57.70, 11.90
        while not received and time.monotonic() < deadline:
            session.put(
                own("location_fix", "gnss"),
                enc("foxglove.LocationFix", latitude=lat, longitude=lon),
            )
            session.put(
                own("course_over_ground_deg", "slipway"), enc("keelson.TimestampedFloat", value=0.0)
            )
            session.put(
                own("speed_over_ground_knots", "slipway"),
                enc("keelson.TimestampedFloat", value=10.0),
            )
            session.put(
                ais("location_fix"),
                enc("foxglove.LocationFix", latitude=lat + 2 / 60, longitude=lon),
            )
            session.put(ais("course_over_ground_deg"), enc("keelson.TimestampedFloat", value=180.0))
            session.put(ais("speed_over_ground_knots"), enc("keelson.TimestampedFloat", value=10.0))
            time.sleep(0.2)
        tokens = [str(r.ok.key_expr) for r in session.liveliness().get(cpa_key) if r.ok]
    finally:
        stop.set()
        worker.join(timeout=10)
        sub.undeclare()

    assert not worker.is_alive()
    ((key, data),) = received.items()
    assert key.endswith("/pubsub/target_cpa_m/navigation/0/traffic/@target/mmsi_265000001")
    _, _, payload = keelson.uncover(data)
    value = keelson.decode_protobuf_payload_from_type_name(
        payload, "keelson.TimestampedFloat"
    ).value
    assert value < 30.0  # head-on: CPA near zero, allowing for dead reckoning between puts
    assert tokens == [cpa_key]
    assert not math.isnan(value)
