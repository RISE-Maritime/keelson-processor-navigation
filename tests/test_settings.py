"""navigation_control/v1 settings (keelson#322), the pure half: what is in force, what is fixed, and
how a request is refused whole."""

from __future__ import annotations

import copy

import pytest

from nav import config as config_mod
from nav.control import settings as s

pytestmark = pytest.mark.unit


def _current(**built):
    conf = copy.deepcopy(config_mod.DEFAULTS)
    args = dict(
        advisor_built=True,
        guidance_built=True,
        traffic_built=True,
        monitor_built=True,
        colreg_active=True,
        default_speed_knots=6.0,
        goto_radius_m=20.0,
        loiter_return_knots=2.0,
    )
    args.update(built)
    return s.current(conf=conf, **args)


def test_everything_built_nothing_fixed():
    settings, fixed = _current()
    assert set(settings) == set(s.FIELDS)
    assert fixed == []


def test_the_advisor_and_auto_accept_exist_only_when_started_with_them():
    _, fixed = _current(advisor_built=False)
    assert fixed == ["colreg_advisor_active", "auto_accept_advice"]
    _, fixed = _current(guidance_built=False)
    assert fixed == ["auto_accept_advice"]


def test_a_paused_advisor_reads_inactive():
    settings, _ = _current(colreg_active=False)
    assert settings["colreg_advisor_active"] is False


def test_parse_names_only_what_changes_and_refuses_whole():
    settings, fixed = _current(advisor_built=False)
    assert s.parse({"default_speed_knots": 8}, settings, fixed) == (
        {"default_speed_knots": 8.0},
        "",
    )
    changes, why = s.parse(
        {"default_speed_knots": 8, "colreg_advisor_active": True}, settings, fixed
    )
    assert changes is None and "set at start" in why
    with pytest.raises(s.SettingsError):
        s.parse({"traffic_range_m": 0}, settings, fixed)
    with pytest.raises(s.SettingsError):
        s.parse({"route_alarms": "yes"}, settings, fixed)
    assert s.parse({"go_to_arrival_radius_m": 0}, settings, fixed)[0] == {
        "go_to_arrival_radius_m": 0.0
    }


def test_describe_for_the_decision_log():
    assert (
        s.describe({"colreg_advisor_active": True}, {"colreg_advisor_active": False})
        == "colreg_advisor_active true -> false"
    )
