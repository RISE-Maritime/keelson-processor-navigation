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


@pytest.fixture
def traffic_conf(monitor_conf) -> dict:
    monitor_conf["modules"]["traffic"] = True
    monitor_conf["inputs"]["heading_true_north_deg"] = "sim"
    monitor_conf["traffic"]["feeds"] = [{"entity": "nav-test-ais", "source": "ais"}]
    return monitor_conf
