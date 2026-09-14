from __future__ import annotations

import pytest

from nav import config as config_mod

pytestmark = pytest.mark.unit


@pytest.fixture
def gconf(conf):
    conf["modules"].update(guidance=True, route_monitor=True, decision_log=True)
    conf["inputs"] = {
        "location_fix": "gnss",
        "heading_true_north_deg": "sim",
        "course_over_ground_deg": "sim",
        "speed_over_ground_knots": "sim",
    }
    conf["guidance"]["full_ahead_sog_knots"] = 20.5
    return conf


def test_a_complete_guidance_config_is_valid(gconf):
    config_mod._validate(gconf)
    assert config_mod.controller_id(gconf) == "navigation/0@nav-test-vessel"


@pytest.mark.parametrize(
    ("change", "match"),
    [
        (lambda c: c["guidance"].update(full_ahead_sog_knots=None), "full_ahead_sog_knots"),
        (lambda c: c["guidance"].update(publish_hz=5.0), "10-20"),
        (lambda c: c["guidance"].update(publish_hz=25.0), "10-20"),
        (lambda c: c["guidance"].update(backend="goto"), "Phase 6"),
        (lambda c: c["guidance"].update(command_hold_s=1.5), "max_axis_age_s"),
        (lambda c: c["guidance"].update(command_hold_s=0.1), "two axis publications"),
        (lambda c: c["guidance"].update(bogus=1), "unknown guidance"),
        (lambda c: c["guidance"].update(throttle_kp_pct_per_knot=5.0), "must be below 1"),
        (
            lambda c: c["modules"].update(conduct=False, decision_log=False),
            "needs module 'conduct'",
        ),
        (lambda c: c["inputs"].pop("heading_true_north_deg"), "heading_true_north_deg"),
        (lambda c: c["conduct"].update(heartbeat_s=30.0), "shorter than"),
        (lambda c: c["conduct"].update(fixed_authority_level="LEVEL_9"), "fixed_authority_level"),
        (
            lambda c: c["conduct"].update(
                authority_source="aggregator",
                fixed_authority_level="AUTHORITY_LEVEL_FULL_AUTONOMOUS",
            ),
            "not both",
        ),
        (lambda c: c["conduct"].update(authority_source="truth"), "ground truth"),
        (lambda c: c["conduct"].update(auto_accept_advice=True), "colreg"),
    ],
)
def test_invalid_guidance_configs_are_refused(gconf, change, match):
    change(gconf)
    with pytest.raises(config_mod.ConfigError, match=match):
        config_mod._validate(gconf)
