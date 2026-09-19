"""navigation_control/v1 over an isolated local zenoh peer session: real queryables, a real voyage
publication, and the cross-vessel route fetch reading another entity's stored edition."""

from __future__ import annotations

import threading
import time

import keelson
import pytest
from keelson.interfaces import NavigationControl_pb2 as nc

from nav import app as app_mod
from tests.routes import square_route
from tests.test_e2e_bus import session  # noqa: F401  (fixture)
from tests.test_e2e_route import route_proto

pytestmark = pytest.mark.e2e


def call(session, conf, procedure, request, timeout=5.0):  # noqa: F811
    key = keelson.construct_rpc_key(
        conf["realm"], conf["entity_id"], "navigation_control", "v1", procedure, "navigation/0"
    )
    for reply in session.get(key, payload=request.SerializeToString(), timeout=timeout):
        if reply.ok is not None:
            return bytes(reply.ok.payload.to_bytes())
        raise AssertionError(bytes(reply.err.payload.to_bytes()))
    raise AssertionError(f"no reply on {key}")


def test_load_route_over_the_bus_copies_a_foreign_edition_and_publishes_the_voyage(
    session,  # noqa: F811
    conf,
):
    conf["modules"].update(control=True)
    realm, entity = conf["realm"], conf["entity_id"]

    # Another vessel's edition, answered from "storage": a queryable standing in for the router.
    other = route_proto(square_route(route_id="r9", edition=3))
    foreign_key = keelson.construct_pubsub_key(realm, "nav-test-other", "route", "r9/edition/3")
    stored = keelson.enclose(other.SerializeToString())
    storage = session.declare_queryable(foreign_key, lambda q: q.reply(foreign_key, stored))

    voyages = []
    sub = session.declare_subscriber(
        keelson.construct_pubsub_key(realm, entity, "voyage", "*"),
        lambda s: voyages.append(str(s.key_expr)),
    )
    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, conf, stop), daemon=True)
    worker.start()
    try:
        lease = keelson.get_protobuf_message_class_from_type_name("keelson.CommandAuthority")(
            vessel_id=entity, controller_id="roc@ROC-1", token="T", lease_ttl_seconds=30
        )
        lease_key = keelson.construct_pubsub_key(realm, entity, "command_authority", entity)
        request = nc.LoadRouteRequest(source_entity_id="nav-test-other")
        request.caller.controller_id = "roc@ROC-1"
        request.caller.authority_token = "T"
        request.route_ref.route_id, request.route_ref.route_edition_number = "r9", 3

        deadline = time.monotonic() + 15
        response = None
        while time.monotonic() < deadline:
            session.put(lease_key, keelson.enclose(lease.SerializeToString()))
            try:
                response = nc.LoadRouteResponse.FromString(
                    call(session, conf, "load_route", request)
                )
            except AssertionError:  # queryables not declared yet
                time.sleep(0.2)
                continue
            if response.result == 1:  # COMMAND_RESULT_ACCEPTED
                break
            time.sleep(0.2)  # the lease may not be heard yet
        assert response is not None and response.result == 1, response
        assert response.voyage_id.startswith("voy-")
        assert response.state.route_ref.route_id == "r9"
        assert response.state.HasField("guidance_order")

        state = keelson.get_protobuf_message_class_from_type_name("keelson.NavigationState")
        got = state.FromString(
            call(session, conf, "get_navigation_state", nc.GetNavigationStateRequest())
        )
        assert got.voyage_id == response.voyage_id

        deadline = time.monotonic() + 5
        while not voyages and time.monotonic() < deadline:
            time.sleep(0.1)
        assert voyages and voyages[-1].endswith(f"/voyage/{response.voyage_id}")
    finally:
        stop.set()
        worker.join(timeout=10)
        sub.undeclare()
        storage.undeclare()
