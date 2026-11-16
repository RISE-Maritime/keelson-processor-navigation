"""The run's instance (Slipway ADR-0139): one rule, applied to the vessel and its traffic feeds."""

from __future__ import annotations

import types
from pathlib import Path

import pytest
import yaml

from nav import app as app_mod
from nav import config as config_mod
from nav.instance import bus_entity, validate

pytestmark = pytest.mark.unit


@pytest.mark.parametrize(
    ("authored", "bus"),
    [
        ("sim-slipway", "sim-ted-slipway"),
        ("sim-usv-alpha", "sim-ted-usv-alpha"),
        ("sim-ais-traffic", "sim-ted-ais-traffic"),
    ],
)
def test_sim_entities_carry_the_instance(authored, bus):
    assert bus_entity(authored, "ted") == bus


# The old `slipway-` names are a hard cut (crowsnest-dev#420): they pass through like any other.
@pytest.mark.parametrize(
    "entity", ["roc1", "sf18", "sim", "simx", "my-sim-boat", "slipway", "slipway-usv-alpha"]
)
def test_an_entity_that_is_not_slipways_is_not_renamed(entity):
    assert bus_entity(entity, "ted") == entity


def test_the_empty_instance_is_the_plain_names():
    assert bus_entity("sim-usv-alpha", "") == "sim-usv-alpha"


@pytest.mark.parametrize("bad", ["Ted", "te-d", "te d", "ted/1", "*"])
def test_an_instance_is_one_chunk_of_letters_and_digits(bad):
    with pytest.raises(ValueError, match="lowercase letters and digits"):
        validate(bad)


def _ns(**kw):
    base = dict(config=None, realm=None, entity_id=None, slipway_instance=None)
    base.update(source_base=None, codec=None)
    base.update(kw)
    return types.SimpleNamespace(**base)


def _config_file(tmp_path: Path) -> Path:
    path = tmp_path / "navigation.yaml"
    path.write_text(
        yaml.safe_dump(
            {
                "entity_id": "sim-usv-alpha",
                "modules": {"traffic": True},
                "inputs": {"location_fix": "gnss"},
                "traffic": {
                    "feeds": [
                        {"entity": "sim-ais-traffic", "source": "ais"},
                        {"entity": "sf18", "source": "ais"},
                    ]
                },
            }
        ),
        encoding="utf-8",
    )
    return path


def test_the_flag_renames_the_vessel_and_slipways_feeds_but_not_the_file(tmp_path):
    path = _config_file(tmp_path)
    before = path.read_text(encoding="utf-8")
    conf = config_mod.load(_ns(config=str(path), slipway_instance="ted"))
    assert conf["entity_id"] == "sim-ted-usv-alpha"
    assert [f["entity"] for f in conf["traffic"]["feeds"]] == ["sim-ted-ais-traffic", "sf18"]
    # Derived after the rename, so the conn is granted to the name on the bus.
    assert config_mod.controller_id(conf) == "navigation/0@sim-ted-usv-alpha"
    assert path.read_text(encoding="utf-8") == before


def test_the_empty_flag_is_the_plain_names(tmp_path):
    conf = config_mod.load(_ns(config=str(_config_file(tmp_path)), slipway_instance=""))
    assert conf["entity_id"] == "sim-usv-alpha"
    assert conf["traffic"]["feeds"][0]["entity"] == "sim-ais-traffic"


def test_a_bad_instance_refuses_to_start(tmp_path):
    with pytest.raises(config_mod.ConfigError, match="slipway_instance"):
        config_mod.load(_ns(config=str(_config_file(tmp_path)), slipway_instance="te-d"))


def test_the_cli_takes_the_flag():
    args = app_mod.parse_args(["-c", "x.yaml", "--slipway-instance", "ted"])
    assert args.slipway_instance == "ted"
