from __future__ import annotations

import keelson
import pytest
from google.protobuf import json_format

from nav.route_monitor.alarms import AlarmManager, AlarmSpec

pytestmark = pytest.mark.unit

S = 1_000_000_000
DEV = AlarmSpec("Cross-track deviation 80 m exceeds 50 m")


def test_raise_republish_and_clear():
    m = AlarmManager(republish_s=5, ttl_s=15)
    (raised,) = m.step(0, {"dev": DEV})
    assert m.step(1 * S, {"dev": DEV}) == []
    (again,) = m.step(5 * S, {"dev": DEV})
    (cleared,) = m.step(6 * S, {})

    assert raised["expiration_time_utc"] == "1970-01-01T00:00:15Z"
    assert again["expiration_time_utc"] == "1970-01-01T00:00:20Z"
    assert again["activation_time_utc"] == raised["activation_time_utc"]
    assert cleared["expiration_time_utc"] == cleared["timestamp"]
    assert cleared["description"].endswith("(cleared)")
    assert m.active_identifiers == set()
    assert m.step(7 * S, {}) == []


def test_changed_description_is_published_immediately():
    m = AlarmManager()
    m.step(0, {"dev": DEV})
    (updated,) = m.step(1 * S, {"dev": AlarmSpec("Cross-track deviation 90 m exceeds 50 m")})
    assert "90 m" in updated["description"]


def test_payload_fits_keelson_alarm():
    (payload,) = AlarmManager().step(0, {"dev": DEV})
    msg = json_format.ParseDict(
        payload, keelson.get_protobuf_message_class_from_type_name("keelson.Alarm")()
    )
    assert msg.identifier == "dev" and msg.expiration_time_utc.seconds == 15


def test_ttl_must_exceed_republish():
    with pytest.raises(ValueError):
        AlarmManager(republish_s=5, ttl_s=5)
