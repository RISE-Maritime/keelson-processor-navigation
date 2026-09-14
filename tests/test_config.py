from __future__ import annotations

import types
from pathlib import Path

import pytest

from nav import app as app_mod
from nav import config as config_mod

pytestmark = pytest.mark.unit

ROOT = Path(__file__).resolve().parent.parent


def ns(**kw):
    base = dict(config=None, realm=None, entity_id=None, source_base=None, codec=None)
    base.update(kw)
    return types.SimpleNamespace(**base)


def test_help_exits_zero():
    with pytest.raises(SystemExit) as exc:
        app_mod.parse_args(["--help"])
    assert exc.value.code == 0


def test_entity_id_is_required():
    with pytest.raises(config_mod.ConfigError, match="entity_id is required"):
        config_mod.load(ns())


def test_main_refuses_to_start_without_entity(caplog):
    assert app_mod.main([]) == 2


def test_cli_overrides_defaults():
    conf = config_mod.load(ns(entity_id="v1", realm="other", source_base="nav/9", codec="proto"))
    assert (conf["realm"], conf["entity_id"], conf["source_base"], conf["codec"]) == (
        "other",
        "v1",
        "nav/9",
        "proto",
    )


def test_example_file_loads():
    conf = config_mod.load(ns(config=str(ROOT / "config" / "processor.example.yaml")))
    assert conf["entity_id"] == "slipway-alpha"
    assert conf["modules"]["conduct"] is True


def test_file_then_cli_precedence(tmp_path):
    f = tmp_path / "c.yaml"
    f.write_text("entity_id: from-file\nrealm: file-realm\n")
    conf = config_mod.load(ns(config=str(f), entity_id="from-cli"))
    assert conf["entity_id"] == "from-cli"
    assert conf["realm"] == "file-realm"


def test_unimplemented_module_is_refused(tmp_path):
    f = tmp_path / "c.yaml"
    f.write_text("entity_id: v\nmodules:\n  colreg: true\n")
    with pytest.raises(config_mod.ConfigError, match="not implemented"):
        config_mod.load(ns(config=str(f)))


def test_unknown_module_is_refused(tmp_path):
    f = tmp_path / "c.yaml"
    f.write_text("entity_id: v\nmodules:\n  autopilot: false\n")
    with pytest.raises(config_mod.ConfigError, match="unknown module"):
        config_mod.load(ns(config=str(f)))


@pytest.mark.parametrize(
    "yaml_text, match",
    [
        ("entity_id: a/b\n", "single literal key chunk"),
        ("entity_id: v\ncodec: json\n", "codec"),
        ("entity_id: v\ntick_hz: 0\n", "positive"),
        ("entity_id: v\nstate_hz: 20\ntick_hz: 10\n", "cannot exceed"),
    ],
)
def test_invalid_values_are_refused(tmp_path, yaml_text, match):
    f = tmp_path / "c.yaml"
    f.write_text(yaml_text)
    with pytest.raises(config_mod.ConfigError, match=match):
        config_mod.load(ns(config=str(f)))


def test_module_source_id(conf):
    assert config_mod.module_source_id(conf, "conduct") == "navigation/0/conduct"


def _write(tmp_path, text):
    f = tmp_path / "c.yaml"
    f.write_text(text)
    return str(f)


def test_example_file_reads_slipway_operational_sources():
    conf = config_mod.load(ns(config=str(ROOT / "config" / "processor.example.yaml")))
    assert conf["modules"]["route_monitor"] is True
    assert conf["inputs"]["location_fix"] == "gnss"
    assert conf["route_monitor"]["execution_hz"] == 1


@pytest.mark.parametrize("source", ["truth", "kernel/truth"])
def test_truth_sources_are_refused(tmp_path, source):
    path = _write(tmp_path, f"entity_id: v\ninputs:\n  location_fix: {source}\n")
    with pytest.raises(config_mod.ConfigError, match="ground truth"):
        config_mod.load(ns(config=path))


def test_route_monitor_needs_a_position_source(tmp_path):
    path = _write(tmp_path, "entity_id: v\nmodules:\n  route_monitor: true\n")
    with pytest.raises(config_mod.ConfigError, match="inputs.location_fix"):
        config_mod.load(ns(config=path))


@pytest.mark.parametrize(
    "text, match",
    [
        ("inputs:\n  rudder_angle_deg: n2k\n", "not an input"),
        ("inputs:\n  location_fix: '**'\n", "not a key pattern"),
        ("route_monitor:\n  bogus: 1\n", "unknown route_monitor"),
        ("route_monitor:\n  alarm_ttl_s: 2\n", "alarm_ttl_s"),
        ("route_monitor:\n  deviation_clear_fraction: 1.2\n", "below 1"),
    ],
)
def test_input_and_monitor_settings_are_validated(tmp_path, text, match):
    with pytest.raises(config_mod.ConfigError, match=match):
        config_mod.load(ns(config=_write(tmp_path, "entity_id: v\n" + text)))


def test_route_monitor_section_merges_over_defaults(tmp_path):
    path = _write(tmp_path, "entity_id: v\nroute_monitor:\n  execution_hz: 2\n")
    conf = config_mod.load(ns(config=path))
    assert conf["route_monitor"]["execution_hz"] == 2
    assert conf["route_monitor"]["alarms"] is True


def test_traffic_needs_feeds(tmp_path):
    path = _write(
        tmp_path, "entity_id: v\nmodules:\n  traffic: true\ninputs:\n  location_fix: gnss\n"
    )
    with pytest.raises(config_mod.ConfigError, match="traffic.feeds"):
        config_mod.load(ns(config=path))


@pytest.mark.parametrize(
    "feed, match",
    [
        ("{entity: '*', source: ais}", "single literal key chunk"),
        ("{entity: maritimedb, source: 'ais/*'}", "not a pattern"),
        ("{entity: slipway-traffic, source: truth}", "ground truth"),
        ("{entity: maritimedb}", "exactly entity and source"),
    ],
)
def test_traffic_feeds_are_validated(tmp_path, feed, match):
    path = _write(tmp_path, f"entity_id: v\ntraffic:\n  feeds:\n    - {feed}\n")
    with pytest.raises(config_mod.ConfigError, match=match):
        config_mod.load(ns(config=path))


def test_example_traffic_feeds_are_maritimedb():
    conf = config_mod.load(ns(config=str(ROOT / "config" / "processor.example.yaml")))
    assert {f["entity"] for f in conf["traffic"]["feeds"]} == {"maritimedb"}
    assert conf["modules"]["traffic"] is True
