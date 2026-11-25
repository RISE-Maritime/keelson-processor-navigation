"""The conn: keelson.CommandAuthority, held as a lease judged on this receiver's own clock.

keelson's protocol is pub/sub only (CommandAuthority.proto; protocol-specification §6.5, §6.5.1):

- A holder publishes CommandAuthority with its token, republishes it every heartbeat interval, and
  releases it with ``released=true`` and the same token.
- A requester publishes CommandRequest. The holder grants by publishing a CommandAuthority that
  names the requester with a fresh token; the new holder then heartbeats that token.
- Every receiver lapses a lease ``lease_ttl_seconds`` after the last message it heard for it, on
  its own clock. ``expires_at`` is never compared with a local clock.

Both subjects are published with source_id = the vessel id. Crowsnest (commandAuthority.js) does
the same, and Slipway subscribes ``command_authority/*``: a multi-chunk source such as
``navigation/0/conduct`` would not match that single-chunk wildcard, and the kernel would never
hear the lease.
"""

from __future__ import annotations

import re
import secrets
from dataclasses import dataclass, replace

from ..bus.codec import rfc3339


@dataclass(frozen=True)
class Heard:
    controller_id: str
    controller_site: str
    token: str
    ttl_s: float
    received_ns: int
    released: bool = False

    def live(self, now_ns: int) -> bool:
        return not self.released and now_ns <= self.received_ns + int(self.ttl_s * 1e9)


class LeaseBook:
    """The newest lease message heard, by local receipt."""

    def __init__(self):
        self.last: Heard | None = None

    def hear(self, payload: dict, received_ns: int) -> None:
        token = str(payload.get("token", ""))
        if payload.get("released", False):
            # A release ends only the lease it names: the same token as the latest grant.
            if self.last is not None and token and token == self.last.token:
                self.last = replace(self.last, released=True, received_ns=received_ns)
            return
        ttl = float(payload.get("lease_ttl_seconds", 0) or 0)
        if not token or ttl <= 0:
            return  # nothing a receiver could ever lapse; Slipway ignores these too
        self.last = Heard(
            str(payload.get("controller_id", "")),
            str(payload.get("controller_site", "")),
            token,
            ttl,
            received_ns,
        )

    def holder(self, now_ns: int) -> Heard | None:
        return self.last if self.last is not None and self.last.live(now_ns) else None


_NAVIGATION_PROCESS = re.compile(r"^navigation/[^@]+@")


def is_navigation_process(controller_id: str) -> bool:
    """A navigation process's controller id, ``{source_base}@{vessel}``: automation, never an
    operator. The conn is yielded to operators only."""
    return bool(_NAVIGATION_PROCESS.match(controller_id or ""))


@dataclass(frozen=True)
class LeaseConfig:
    vessel_id: str
    controller_id: str
    controller_site: str = "onboard"
    ttl_s: float = 30.0
    heartbeat_s: float = 10.0
    request_interval_s: float = 10.0


@dataclass(frozen=True)
class LeaseEvent:
    kind: str  # taken | granted | handed | lost | released | requested
    detail: str


def grantee_id(request: dict) -> str:
    """The controller id a directed grant names: ``{requester_id}@{requester_site}``.

    A station's controller id is ``operator@site``, but its CommandRequest carries the operator and the
    site in two fields (Crowsnest ``buildCommandRequest``), and Crowsnest's own directed grant joins them
    (``useCommandAuthority`` grant). Copying ``requester_id`` alone named a holder no station is: measured
    on sim-ted-bulk-alfa 2026-11-25, ROC-TED (ted-sup) requested, was granted "ted-sup", did not recognise
    it as its own, never heartbeated it, and the conn lapsed back to this process every 30 s. An id that
    already carries a site (``ted@roc1``, ``navigation/0@vessel``) is used as it is.
    """
    requester = request.get("requester_id") or ""
    site = request.get("requester_site") or ""
    return requester if "@" in requester or not site else f"{requester}@{site}"


class LeaseClient:
    """Takes, adopts, heartbeats and releases the conn for one controller id."""

    def __init__(self, cfg: LeaseConfig, token_factory=None):
        self.cfg = cfg
        self._new_token = token_factory or (lambda: secrets.token_hex(16))
        self.token: str | None = None
        self.self_issued: set[str] = set()
        self._granted_ns: int | None = None
        self._last_sent_ns: int | None = None
        self._last_request_ns: int | None = None

    def held(self, book: LeaseBook, now_ns: int) -> bool:
        h = book.holder(now_ns)
        return (
            self.token is not None
            and h is not None
            and h.token == self.token
            and h.controller_id == self.cfg.controller_id
        )

    def step(
        self,
        book: LeaseBook,
        now_ns: int,
        *,
        want: bool,
        may_take: bool,
        may_request: bool,
        yield_to: dict | None = None,
    ) -> tuple[list[tuple[str, dict]], list[LeaseEvent]]:
        """(subject, payload) pairs to publish, and what happened. Publish the command_authority
        payloads back into ``book`` too: a holder hears its own lease.

        ``yield_to`` is a command_request to answer: while holding, the conn is granted to its
        requester -- keelson's ``grant``, a fresh token naming the requester and its site, so the
        holder changes with no lapse in between."""
        out: list[tuple[str, dict]] = []
        events: list[LeaseEvent] = []
        me = self.cfg.controller_id
        h = book.holder(now_ns)

        if (
            h is not None
            and h.controller_id == me
            and h.token != self.token
            and h.token not in self.self_issued
        ):
            self.token, self._granted_ns, self._last_sent_ns = h.token, h.received_ns, None
            events.append(LeaseEvent("granted", f"conn granted to {me} from {h.controller_site}"))

        if self.token is not None and not self.held(book, now_ns):
            why = (
                "lease lapsed or was released"
                if h is None
                else f"lease is now held by {h.controller_id}"
            )
            events.append(LeaseEvent("lost", why))
            self.token = None

        if self.token is not None and yield_to is not None:
            out.append(("command_authority", self._grant(now_ns, yield_to)))
            events.append(
                LeaseEvent("handed", f"conn granted to {yield_to['requester_id']} on request")
            )
            self.token = None
            return out, events

        if self.token is not None:
            if not want:
                out.append(("command_authority", self._authority(now_ns, released=True)))
                events.append(LeaseEvent("released", "conn released"))
                self.token = None
            elif (
                self._last_sent_ns is None
                or now_ns - self._last_sent_ns >= self.cfg.heartbeat_s * 1e9
            ):
                out.append(("command_authority", self._authority(now_ns)))
                self._last_sent_ns = now_ns
        elif want:
            if h is None and may_take:
                self.token = self._new_token()
                self.self_issued.add(self.token)
                self._granted_ns = self._last_sent_ns = now_ns
                out.append(("command_authority", self._authority(now_ns)))
                events.append(LeaseEvent("taken", f"conn taken by {me}: no live lease was held"))
            elif may_request and (
                self._last_request_ns is None
                or now_ns - self._last_request_ns >= self.cfg.request_interval_s * 1e9
            ):
                self._last_request_ns = now_ns
                out.append(("command_request", self._request(now_ns)))
                holder = h.controller_id if h is not None else "nobody"
                events.append(LeaseEvent("requested", f"conn requested from {holder}"))
        return out, events

    def _authority(self, now_ns: int, released: bool = False) -> dict:
        c = self.cfg
        return {
            "vessel_id": c.vessel_id,
            "controller_id": c.controller_id,
            "controller_site": c.controller_site,
            "token": self.token,
            "granted_at": rfc3339(self._granted_ns or now_ns),
            "expires_at": rfc3339(now_ns + int(c.ttl_s * 1e9)),
            "last_heartbeat": rfc3339(now_ns),
            "lease_ttl_seconds": int(c.ttl_s),
            "heartbeat_interval_seconds": int(c.heartbeat_s),
            "released": released,
        }

    def _grant(self, now_ns: int, request: dict) -> dict:
        c = self.cfg
        token = self._new_token()
        self.self_issued.add(token)  # never adopted back as a grant to this process
        return {
            "vessel_id": c.vessel_id,
            "controller_id": grantee_id(request),
            # Exactly as requested: a station adopts a grant only when id AND site are its own.
            "controller_site": request.get("requester_site", ""),
            "token": token,
            "granted_at": rfc3339(now_ns),
            "expires_at": rfc3339(now_ns + int(c.ttl_s * 1e9)),
            "last_heartbeat": rfc3339(now_ns),
            "lease_ttl_seconds": int(c.ttl_s),
            "heartbeat_interval_seconds": int(c.heartbeat_s),
            "released": False,
        }

    def _request(self, now_ns: int) -> dict:
        c = self.cfg
        return {
            "vessel_id": c.vessel_id,
            "requester_id": c.controller_id,
            "requester_site": c.controller_site,
            "requested_at": rfc3339(now_ns),
            "reason": "navigation guidance needs the conn to sail the selected route",
        }
