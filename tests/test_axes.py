from __future__ import annotations

import pytest
from google.protobuf import json_format

from nav.guidance.axes import FREE, OTHER, OURS, UNKNOWN, ownership, wanted_mapping

pytestmark = pytest.mark.unit

SRC = "navigation/0/guidance"


def test_the_mapping_is_a_valid_control_axis_mapping():
    from keelson.interfaces.VehicleControl_pb2 import ControlAxisMapping

    msg = json_format.ParseDict(wanted_mapping(SRC, 1.0), ControlAxisMapping())
    assert msg.axes["steering"].subject == "joystick_x_pct"
    assert msg.axes["throttle"].subject == "joystick_y_pct"
    assert msg.axes["steering"].source_id == SRC and msg.max_axis_age_s == 1.0


@pytest.mark.parametrize(
    ("mapping", "expected"),
    [
        (None, UNKNOWN),
        ({}, FREE),
        ({"axes": {}}, FREE),
        (wanted_mapping(SRC, 1.0), OURS),
        (
            {"axes": {"steering": {"subject": "wheel_position_pct", "source_id": "autopilot"}}},
            OTHER,
        ),
        ({"axes": {"steering": {"subject": "joystick_x_pct", "source_id": SRC}}}, OTHER),
        (
            {
                "axes": {
                    a: v | {"entity_id": "someone-else"}
                    for a, v in wanted_mapping(SRC, 1.0)["axes"].items()
                }
            },
            OTHER,
        ),
        (
            {
                "axes": {
                    a: v | {"entity_id": "nav-test-vessel"}
                    for a, v in wanted_mapping(SRC, 1.0)["axes"].items()
                }
            },
            OURS,
        ),
    ],
)
def test_ownership(mapping, expected):
    assert ownership(mapping, "nav-test-vessel", SRC) == expected
