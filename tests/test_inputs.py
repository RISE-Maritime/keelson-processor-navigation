from __future__ import annotations

from dataclasses import dataclass

import keelson
import pytest

from nav.bus.inputs import Inputs, pubsub_tail
from nav.world.model import World

pytestmark = pytest.mark.unit

BASE = "rise/@v0/nav-test-vessel/pubsub"


@dataclass
class _Payload:
    data: bytes

    def to_bytes(self):
        return self.data


@dataclass
class FakeSample:
    key_expr: str
    payload: _Payload


def enclosed(type_name, **fields):
    msg = keelson.get_protobuf_message_class_from_type_name(type_name)(**fields)
    return msg


def sample(key, msg):
    return FakeSample(key, _Payload(keelson.enclose(msg.SerializeToString())))


def route_msg(route_id="r1", edition=2):
    r = enclosed("keelson.Route", route_id=route_id, route_edition_number=edition)
    for i, lat in enumerate((57.70, 57.71)):
        w = r.waypoints.waypoint.add(id=f"wp{i}")
        w.position.latitude_deg, w.position.longitude_deg = lat, 11.9
    return r


@pytest.fixture
def inputs(monitor_conf):
    return Inputs(None, monitor_conf, World(), need_routes=True, clock=lambda: 42)


def test_pubsub_tail():
    assert pubsub_tail(f"{BASE}/route/r1/edition/2") == ("route", ["r1", "edition", "2"])
    assert pubsub_tail("rise/@v0/x/other") is None


def test_edition_key_is_stored(inputs):
    inputs._on_route(sample(f"{BASE}/route/r1/edition/2", route_msg()), live=True)
    assert set(inputs.world.snapshot().editions) == {("r1", 2)}


def test_edition_payload_must_match_its_key(inputs):
    handler = inputs._guard(lambda s: inputs._on_route(s, live=True))
    handler(sample(f"{BASE}/route/r1/edition/3", route_msg(edition=2)))
    snap = inputs.world.snapshot()
    assert snap.editions == {}
    assert "does not match" in snap.rejected[0]


def test_latest_is_ignored_and_declaration_is_kept(inputs):
    inputs._on_route(sample(f"{BASE}/route/r1/latest", route_msg()), live=True)
    inputs._on_route(sample(f"{BASE}/route/sim", route_msg("decl")), live=False)
    snap = inputs.world.snapshot()
    assert snap.editions == {}
    assert snap.declared["sim"].plan.route_id == "decl" and snap.declared["sim"].live is False


def test_storage_reply_never_overrides_a_live_declaration(inputs):
    inputs._on_route(sample(f"{BASE}/route/sim", route_msg("live")), live=True)
    inputs._on_route(sample(f"{BASE}/route/sim", route_msg("stored")), live=False)
    assert inputs.world.snapshot().declared["sim"].plan.route_id == "live"


def test_declared_routes_can_be_turned_off(inputs):
    inputs.conf["route_monitor"]["monitor_declared_route"] = False
    inputs._on_route(sample(f"{BASE}/route/sim", route_msg()), live=True)
    assert inputs.world.snapshot().declared == {}


def test_voyage_id_must_match_its_key(inputs):
    handler = inputs._guard(inputs._on_voyage)
    v = enclosed("keelson.Voyage", voyage_id="other", status=2)
    handler(sample(f"{BASE}/voyage/v1", v))
    assert inputs.world.snapshot().voyages == {}
    handler(sample(f"{BASE}/voyage/other", v))
    assert inputs.world.snapshot().voyages["other"].status == "VOYAGE_STATUS_ACTIVE"


def test_input_sample_records_source_and_receipt(inputs):
    fix = enclosed("foxglove.LocationFix", latitude=57.7, longitude=11.9)
    inputs._input_handler("location_fix")(sample(f"{BASE}/location_fix/gnss", fix))
    s = inputs.world.snapshot().inputs["location_fix"]
    assert (s.value, s.source_id, s.received_ns) == ((57.7, 11.9), "gnss", 42)


def test_garbage_payload_is_rejected_not_raised(inputs):
    handler = inputs._guard(inputs._input_handler("location_fix"))
    handler(FakeSample(f"{BASE}/location_fix/gnss", _Payload(b"\xff\x00garbage")))
    assert inputs.world.snapshot().rejected
