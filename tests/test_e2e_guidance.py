"""Guidance on an isolated local zenoh peer session, against a fake vehicle_control/v1 responder.

Checks what only the bus can show: the mapping is installed through the RPC, the lease is published
on the vessel-id key, both axes flow continuously at the interface's rate, and releasing the lease
gives neutral axes and then silence.
"""

from __future__ import annotations

import threading
import time

import keelson
import pytest

from nav import app as app_mod
from nav.route_monitor import geodesy as g
from tests.routes import square_route
from tests.test_e2e_bus import session  # noqa: F401  (fixture)
from tests.test_e2e_route import cls, route_proto

pytestmark = pytest.mark.e2e


def test_guidance_maps_the_helm_drives_and_goes_silent_when_the_lease_is_pulled(session, conf):  # noqa: F811
    from keelson.interfaces.VehicleControl_pb2 import ControlAxisMapping, ControlAxisMappingAck

    conf["modules"].update(route_monitor=True, guidance=True, decision_log=True)
    conf["inputs"] = {
        "location_fix": "gnss",
        "heading_true_north_deg": "sim",
        "course_over_ground_deg": "sim",
        "speed_over_ground_knots": "sim",
    }
    conf["conduct"].update(
        take_conn_when_free=True, fixed_authority_level="AUTHORITY_LEVEL_REMOTE_CONTROLLED"
    )
    conf["guidance"].update(
        full_ahead_sog_knots=20.0, neutral_burst_s=0.5, mapping_poll_s=0.5, mapping_max_age_s=3.0
    )
    realm, entity = conf["realm"], conf["entity_id"]

    def key(subject, source):
        return keelson.construct_pubsub_key(realm, entity, subject, source)

    installed = {"mapping": ControlAxisMapping()}
    sets = []

    def on_get(query):
        query.reply(query.key_expr, installed["mapping"].SerializeToString())

    def on_set(query):
        request = ControlAxisMapping.FromString(bytes(query.payload.to_bytes()))
        installed["mapping"] = request
        sets.append(request)
        query.reply(query.key_expr, ControlAxisMappingAck().SerializeToString())

    def rpc(procedure):
        return keelson.construct_rpc_key(realm, entity, "vehicle_control", "v1", procedure, "sim")

    queryables = [
        session.declare_queryable(rpc("get_control_mapping"), on_get),
        session.declare_queryable(rpc("set_control_mapping"), on_set),
    ]
    axes: list[tuple[str, float, float]] = []
    leases: list[tuple[str, bytes]] = []
    float_cls = cls("keelson.TimestampedFloat")

    def on_axis(sample):
        _, _, payload = keelson.uncover(bytes(sample.payload.to_bytes()))
        axes.append((str(sample.key_expr), float_cls.FromString(payload).value, time.monotonic()))

    setpoints: list[tuple[str, float]] = []

    def on_setpoint(sample):
        _, _, payload = keelson.uncover(bytes(sample.payload.to_bytes()))
        subject = str(sample.key_expr).split("/pubsub/")[1].split("/")[0]
        setpoints.append((subject, float_cls.FromString(payload).value))

    subs = [
        session.declare_subscriber(
            key("heading_setpoint_deg", "navigation/0/guidance"), on_setpoint
        ),
        session.declare_subscriber(
            key("course_over_ground_setpoint_deg", "navigation/0/guidance"), on_setpoint
        ),
        session.declare_subscriber(key("joystick_x_pct", "navigation/0/guidance"), on_axis),
        session.declare_subscriber(key("joystick_y_pct", "navigation/0/guidance"), on_axis),
        session.declare_subscriber(
            key("command_authority", "*"),
            lambda s: leases.append((str(s.key_expr), bytes(s.payload.to_bytes()))),
        ),
    ]

    stop = threading.Event()
    worker = threading.Thread(target=app_mod.run, args=(session, conf, stop), daemon=True)
    worker.start()

    plan = square_route()
    start = plan.waypoints[0]
    route_bytes = keelson.enclose(route_proto(plan).SerializeToString())
    along = 100.0

    def feed():
        nonlocal along
        session.put(key("route", "sim"), route_bytes)
        lat, lon = g.destination(start.lat, start.lon, 0.0, along)
        fix = cls("foxglove.LocationFix")(latitude=lat, longitude=lon)
        fix.timestamp.FromNanoseconds(time.time_ns())
        session.put(key("location_fix", "gnss"), keelson.enclose(fix.SerializeToString()))
        for subject, value in (
            ("heading_true_north_deg", 0.0),
            ("course_over_ground_deg", 0.0),
            ("speed_over_ground_knots", 8.0),
        ):
            msg = float_cls(value=value)
            session.put(key(subject, "sim"), keelson.enclose(msg.SerializeToString()))
        along += 1.0

    pulled_at = None
    try:
        deadline = time.monotonic() + 20
        while len(axes) < 40 and time.monotonic() < deadline:
            feed()
            time.sleep(0.2)
        window_start = time.monotonic()
        while time.monotonic() < window_start + 2.0:
            feed()
            time.sleep(0.2)
        rate = [a for a in axes if window_start + 0.5 <= a[2] < window_start + 1.5]

        _, _, payload = keelson.uncover(leases[-1][1])
        lease = cls("keelson.CommandAuthority").FromString(payload)
        released = cls("keelson.CommandAuthority")(
            vessel_id=entity, controller_id=lease.controller_id, token=lease.token, released=True
        )
        pulled_at = time.monotonic()
        session.put(key("command_authority", entity), keelson.enclose(released.SerializeToString()))
        while time.monotonic() < pulled_at + 3.0:
            feed()
            time.sleep(0.2)
    finally:
        stop.set()
        worker.join(timeout=10)
        for s in subs:
            s.undeclare()
        for q in queryables:
            q.undeclare()

    assert not worker.is_alive()
    assert sets, "guidance never called set_control_mapping"
    assert sets[0].axes["steering"].subject == "joystick_x_pct"
    assert sets[0].axes["throttle"].source_id == "navigation/0/guidance"
    assert leases and leases[0][0] == key("command_authority", entity)
    assert len(axes) >= 40, "axes did not flow"
    per_subject = {k: sum(1 for a in rate if a[0] == k) for k in {a[0] for a in rate}}
    assert len(per_subject) == 2, per_subject
    assert all(8 <= n <= 13 for n in per_subject.values()), per_subject

    # keelson#316 setpoints, typed on their own subjects: on the track the helm steers a course.
    named = {s for s, _ in setpoints}
    assert named == {"heading_setpoint_deg", "course_over_ground_setpoint_deg"}, named
    assert all(0.0 <= v < 360.0 for _, v in setpoints)

    after = [a for a in axes if a[2] > pulled_at + 0.3]
    assert after, "no neutral burst after the lease was pulled"
    assert {a[1] for a in after} == {0.0}
    assert not [a for a in axes if a[2] > pulled_at + 1.5], "axes did not go silent"
