from __future__ import annotations

import pytest

from nav.route_monitor import geodesy as g

pytestmark = pytest.mark.unit

# A leg due north from Gothenburg harbour, about 1.1 km long.
A = (57.70, 11.95)
B = (57.71, 11.95)


def test_haversine_one_minute_of_latitude_is_about_a_nautical_mile():
    assert g.haversine_m(57.0, 11.0, 57.0 + 1 / 60, 11.0) == pytest.approx(1853.2, abs=1.0)


@pytest.mark.parametrize("to, expected", [((57.71, 11.95), 0.0), ((57.70, 11.97), 90.0)])
def test_bearing(to, expected):
    assert g.bearing_deg(*A, *to) == pytest.approx(expected, abs=0.02)


def test_cross_track_is_positive_to_starboard():
    east = g.destination(57.705, 11.95, 90.0, 40.0)
    west = g.destination(57.705, 11.95, 270.0, 40.0)
    assert g.cross_track_m(*A, *B, *east) == pytest.approx(40.0, abs=0.1)
    assert g.cross_track_m(*A, *B, *west) == pytest.approx(-40.0, abs=0.1)


def test_along_track_is_negative_behind_the_leg_start():
    behind = g.destination(*A, 180.0, 100.0)
    ahead = g.destination(*A, 0.0, 300.0)
    assert g.along_track_m(*A, *B, *behind) == pytest.approx(-100.0, abs=0.1)
    assert g.along_track_m(*A, *B, *ahead) == pytest.approx(300.0, abs=0.1)
    assert g.along_track_m(*A, *B, *A) == 0.0


def test_distance_to_segment_clamps_to_the_ends():
    length = g.haversine_m(*A, *B)
    beyond = g.destination(*B, 0.0, 50.0)
    beside = g.destination(57.705, 11.95, 90.0, 25.0)
    assert g.distance_to_segment_m(*A, *B, *beyond) == pytest.approx(50.0, abs=0.1)
    assert g.distance_to_segment_m(*A, *B, *beside) == pytest.approx(25.0, abs=0.1)
    assert g.distance_to_segment_m(*A, *B, *g.destination(*A, 180.0, 10.0)) == pytest.approx(10.0)
    assert length == pytest.approx(1112.0, abs=2.0)


def test_destination_round_trips_distance_and_bearing():
    lat, lon = g.destination(*A, 37.0, 1234.0)
    assert g.haversine_m(*A, lat, lon) == pytest.approx(1234.0, abs=0.01)
    assert g.bearing_deg(*A, lat, lon) == pytest.approx(37.0, abs=0.01)
