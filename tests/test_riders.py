"""The rider register is a debt list. These tests make it shrink on its own."""

from __future__ import annotations

from pathlib import Path

import keelson
import pytest

from nav.bus.riders import load_register
from nav.runtime import Processor

pytestmark = pytest.mark.unit

ROOT = Path(__file__).resolve().parent.parent


def test_awaiting_upstream_riders_are_not_yet_in_keelson():
    # Fails when the installed keelson registers a rider's subject: switch that subject to the
    # proto codec and delete its entry, rather than leaving JSON on the wire.
    arrived = [
        r.record
        for r in load_register().values()
        if r.status == "awaiting_upstream" and keelson.is_subject_well_known(r.record)
    ]
    assert not arrived, f"keelson now types {arrived}; retire these riders"


def test_every_rider_has_its_draft_proto():
    for rider in load_register().values():
        text = (ROOT / rider.draft_proto).read_text()
        assert f"message {rider.message.split('.')[-1]} " in text, rider.record


def test_every_published_subject_is_typed_or_declared(conf):
    conf["modules"] = {m: True for m in conf["modules"]}
    for subjects in Processor(conf).publications().values():
        for subject in subjects:
            assert keelson.is_subject_well_known(subject) or subject in load_register(), subject
