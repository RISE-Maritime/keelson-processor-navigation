from __future__ import annotations

import pytest

from nav.bus.codec import Codec
from nav.runtime import Processor

pytestmark = pytest.mark.unit

S = 1_000_000_000


def test_idle_state_published_at_state_rate(conf):
    p = Processor(conf)  # tick 10 Hz, state 1 Hz
    emitted = [t for t in range(0, 30 * S // 10 + 1, S // 10) if p.tick(1_700_000_000 * S + t)]
    assert emitted == [0, S, 2 * S, 3 * S]


def test_idle_state_says_nothing_is_known(conf):
    (out,) = Processor(conf).tick(1_700_000_000 * S)
    assert (out.module, out.subject) == ("conduct", "navigation_state")
    assert out.payload["mode"] == "MODE_IDLE"
    assert out.payload["conn_held"] is False
    assert out.payload["authority_level_in_force"] == "AUTHORITY_LEVEL_UNKNOWN"
    assert out.payload["timestamp"] == "2023-11-14T22:13:20Z"


def test_missed_slots_are_skipped_not_burst(conf):
    p = Processor(conf)
    p.tick(0)
    assert len(p.tick(int(5.5 * S))) == 1
    assert p.tick(int(5.9 * S)) == []
    assert len(p.tick(6 * S)) == 1


def test_disabled_conduct_publishes_nothing(conf):
    conf["modules"]["conduct"] = False
    p = Processor(conf)
    assert p.publications() == {} and p.tick(0) == []


def test_state_payload_encodes_typed(conf):
    # keelson registers navigation_state, so even rider mode puts it on its own typed subject.
    (out,) = Processor(conf).tick(0)
    codec = Codec("rider")
    assert codec.wire_subject(out.subject) == "navigation_state"
    decoded = codec.decode("navigation_state", codec.encode(out.subject, out.payload))
    assert decoded.subject == "navigation_state"
    # proto3 does not put a default on the wire (False, 0, UNKNOWN, []); everything else survives.
    assert decoded.payload == {k: v for k, v in out.payload.items() if k in decoded.payload}
    assert decoded.payload["mode"] == out.payload["mode"]
