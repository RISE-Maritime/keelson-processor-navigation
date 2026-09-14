"""The conn as a lease on the receiver's own clock (keelson protocol-specification §6.5.1)."""

from __future__ import annotations

import pytest

from nav.conduct.lease import LeaseBook, LeaseClient, LeaseConfig

pytestmark = pytest.mark.unit

S = 1_000_000_000
ME = "navigation/0@nav-test-vessel"


def grant(controller=ME, token="t-op", ttl=30, **extra):
    return {
        "controller_id": controller,
        "controller_site": "ROC-1",
        "token": token,
        "lease_ttl_seconds": ttl,
        **extra,
    }


def client(tokens=("t1", "t2", "t3")):
    it = iter(tokens)
    return LeaseClient(LeaseConfig("nav-test-vessel", ME), token_factory=lambda: next(it))


def test_lapse_is_armed_on_local_receipt_and_ignores_expires_at():
    book = LeaseBook()
    # A sender whose clock is a year behind stamps an expiry long past; one a year ahead, long
    # future. Neither changes when this receiver lapses the lease.
    for skewed in ("2025-01-01T00:00:00Z", "2027-12-31T00:00:00Z"):
        book.hear(grant(expires_at=skewed), received_ns=100 * S)
        assert book.holder(129 * S) is not None
        assert book.holder(130 * S) is not None
        assert book.holder(130 * S + 1) is None


def test_a_release_ends_only_the_lease_it_names():
    book = LeaseBook()
    book.hear(grant(token="a"), 0)
    book.hear({"token": "b", "released": True}, S)
    assert book.holder(2 * S).token == "a"
    book.hear({"token": "a", "released": True}, 3 * S)
    assert book.holder(3 * S) is None


def test_a_lease_without_ttl_is_not_a_lease():
    book = LeaseBook()
    book.hear(grant(ttl=0), 0)
    assert book.holder(0) is None


def test_takes_a_free_conn_heartbeats_and_releases_with_the_same_token():
    book, c = LeaseBook(), client()
    out, events = c.step(book, 0, want=True, may_take=True, may_request=False)
    assert [e.kind for e in events] == ["taken"]
    ((subject, payload),) = out
    assert subject == "command_authority" and payload["token"] == "t1"
    assert payload["lease_ttl_seconds"] == 30 and payload["released"] is False
    book.hear(payload, 0)
    assert c.held(book, 0)

    assert c.step(book, 9 * S, want=True, may_take=True, may_request=False)[0] == []
    out, _ = c.step(book, 10 * S, want=True, may_take=True, may_request=False)
    assert out[0][1]["token"] == "t1"
    book.hear(out[0][1], 10 * S)

    out, events = c.step(book, 11 * S, want=False, may_take=True, may_request=False)
    assert [e.kind for e in events] == ["released"]
    assert out[0][1] | {} == out[0][1] and out[0][1]["released"] is True
    assert out[0][1]["token"] == "t1"
    book.hear(out[0][1], 11 * S)
    assert book.holder(11 * S) is None and not c.held(book, 11 * S)


def test_never_takes_a_conn_someone_else_holds_and_asks_instead():
    book, c = LeaseBook(), client()
    book.hear(grant(controller="ted@ROC-1", token="op"), 0)
    out, events = c.step(book, S, want=True, may_take=True, may_request=True)
    assert [s for s, _ in out] == ["command_request"] and [e.kind for e in events] == ["requested"]
    assert c.step(book, 5 * S, want=True, may_take=True, may_request=True)[0] == []
    assert [s for s, _ in c.step(book, 11 * S, want=True, may_take=True, may_request=True)[0]] == [
        "command_request"
    ]


def test_adopts_a_grant_naming_it_and_confirms_at_once():
    book, c = LeaseBook(), client()
    book.hear(grant(token="granted"), 0)
    out, events = c.step(book, S, want=True, may_take=False, may_request=False)
    assert [e.kind for e in events] == ["granted"]
    assert out[0][1]["token"] == "granted" and c.token == "granted"
    book.hear(out[0][1], S)
    assert c.held(book, S)


def test_loses_the_conn_when_another_controller_takes_it():
    book, c = LeaseBook(), client()
    book.hear(c.step(book, 0, want=True, may_take=True, may_request=False)[0][0][1], 0)
    book.hear(grant(controller="ted@ROC-1", token="op"), S)
    out, events = c.step(book, 2 * S, want=True, may_take=True, may_request=False)
    assert [e.kind for e in events] == ["lost"] and "ted@ROC-1" in events[0].detail
    assert out == [] or out[0][0] == "command_request"
    assert c.token is None


def test_its_own_lease_heard_back_is_not_a_grant():
    book, c = LeaseBook(), client()
    ((subject, payload),) = c.step(book, 0, want=True, may_take=True, may_request=False)[0]
    book.hear(payload, 0)
    _, events = c.step(book, S, want=True, may_take=True, may_request=False)
    assert events == []
