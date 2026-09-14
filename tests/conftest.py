from __future__ import annotations

import copy

import pytest

from nav import config as config_mod


@pytest.fixture
def conf() -> dict:
    """A valid config for the test vessel. Never a real entity id."""
    c = copy.deepcopy(config_mod.DEFAULTS)
    c["entity_id"] = "nav-test-vessel"
    return c


@pytest.fixture
def monitor_conf(conf) -> dict:
    conf["modules"]["route_monitor"] = True
    conf["inputs"] = {
        "location_fix": "gnss",
        "speed_over_ground_knots": "sim",
        "course_over_ground_deg": "sim",
        "true_wind_speed_mps": "wx",
    }
    return conf
