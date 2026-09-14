from __future__ import annotations

import json

import keelson
import pytest

from nav.bus.codec import RAW_JSON, Codec, CodecError

pytestmark = pytest.mark.unit

# route_status is a real keelson subject, so it can be sent both ways. Treating it as a rider
# proves the property the codec exists for: a rider decodes to exactly the dict the proto does.
STATUS = {
    "timestamp": "2026-09-14T08:00:00Z",
    "route_id": "r-1",
    "route_edition_number": 3,
    "status": "ROUTE_STATUS_PLANNED",
    "changed_by": "test",
}


def test_rider_and_proto_decode_to_the_same_payload():
    rider = Codec("rider", riders=frozenset({"route_status"}))
    proto = Codec("proto")

    via_rider = rider.decode(
        rider.wire_subject("route_status"), rider.encode("route_status", STATUS)
    )
    via_proto = proto.decode("route_status", proto.encode("route_status", STATUS))

    assert rider.wire_subject("route_status") == RAW_JSON
    assert proto.wire_subject("route_status") == "route_status"
    assert via_rider.subject == via_proto.subject == "route_status"
    assert via_rider.payload == via_proto.payload == STATUS


def test_rider_wire_format():
    codec = Codec("rider", riders=frozenset({"navigation_state"}))
    data = codec.encode("navigation_state", {"mode": "MODE_IDLE"}, enclosed_at_ns=1_000_000_000)
    _, enclosed_at, inner = keelson.uncover(data)
    msg = keelson.decode_protobuf_payload_from_type_name(
        inner, keelson.get_subject_schema(RAW_JSON)
    )
    body = json.loads(msg.value)
    assert enclosed_at == 1_000_000_000
    assert msg.timestamp.ToNanoseconds() == 1_000_000_000
    assert body == {
        "record": "navigation_state",
        "rider_version": 1,
        "payload": {"mode": "MODE_IDLE"},
    }


def test_non_rider_subjects_stay_proto_in_rider_mode():
    codec = Codec("rider", riders=frozenset({"navigation_state"}))
    assert codec.wire_subject("route_status") == "route_status"


def test_proto_mode_refuses_unregistered_subject():
    with pytest.raises(CodecError, match="not registered"):
        Codec("proto").encode("navigation_state_not_in_keelson", {})


def test_payload_that_does_not_fit_the_proto_is_refused():
    with pytest.raises(CodecError, match="does not fit"):
        Codec("proto").encode("route_status", {"no_such_field": 1})


@pytest.mark.parametrize(
    "value, match",
    [
        ("not json", "not JSON"),
        ('{"payload": {}}', "not a rider"),
        ('{"record": "x", "rider_version": 99}', "rider_version"),
    ],
)
def test_malformed_riders_are_refused(value, match):
    inner = keelson.get_protobuf_message_class_from_type_name(keelson.get_subject_schema(RAW_JSON))(
        value=value
    ).SerializeToString()
    with pytest.raises(CodecError, match=match):
        Codec("rider").decode(RAW_JSON, keelson.enclose(inner))
