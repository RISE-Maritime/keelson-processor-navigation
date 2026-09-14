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
        ("entity_id: a/b\n", "single key chunk"),
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
