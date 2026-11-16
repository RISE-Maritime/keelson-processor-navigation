from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import pytest

from nav import config as config_mod

pytestmark = pytest.mark.unit

CONFIGS = sorted((Path(__file__).resolve().parent.parent / "config").glob("*.yaml"))


@pytest.mark.parametrize("path", CONFIGS, ids=lambda p: p.name)
def test_every_shipped_config_loads(path):
    conf = config_mod.load(SimpleNamespace(config=str(path)))
    assert conf["entity_id"] == "sim-usv-alpha"


def test_guidance_is_off_unless_a_config_says_it_is_for_the_simulator():
    for path in CONFIGS:
        conf = config_mod.load(SimpleNamespace(config=str(path)))
        if conf["modules"]["guidance"]:
            assert "Software in the loop ONLY" in path.read_text(), path.name
