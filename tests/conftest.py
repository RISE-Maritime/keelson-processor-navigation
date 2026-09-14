from __future__ import annotations

import pytest

from nav import config as config_mod


@pytest.fixture
def conf() -> dict:
    """A valid config for the test vessel. Never a real entity id."""
    c = dict(config_mod.DEFAULTS)
    c["modules"] = dict(config_mod.DEFAULTS["modules"])
    c["entity_id"] = "nav-test-vessel"
    return c
