from __future__ import annotations

import re
from pathlib import Path

import pytest

from nav.conduct.state import Mode, NavigationState, RouteRef, SubMode

pytestmark = pytest.mark.unit

DRAFT = Path(__file__).resolve().parent.parent / "docs" / "upstream" / "NavigationState.proto"


def _draft_fields_and_enums():
    text = DRAFT.read_text()
    body = text[text.index("message NavigationState {") :]
    fields = set(re.findall(r"^\s*(?:optional |repeated )?[\w.]+\s+(\w+)\s*=\s*\d+;", body, re.M))
    enum_values = set(re.findall(r"^\s*([A-Z][A-Z_]+)\s*=\s*\d+;", body, re.M))
    return fields, enum_values


def _full_state():
    return NavigationState(
        timestamp="2026-09-14T08:00:00Z",
        mode=Mode.UNDERWAY,
        sub_mode=SubMode.TRACK,
        reason="r",
        voyage_id="v",
        route_ref=RouteRef("r1", 2),
        current_leg_from_waypoint_id="wp-1",
        conn_held=True,
        degraded_inputs=("location_fix",),
    )


def test_payload_fields_exist_in_the_draft_proto():
    fields, _ = _draft_fields_and_enums()
    assert set(_full_state().to_payload()) <= fields


def test_enum_names_exist_in_the_draft_proto():
    _, values = _draft_fields_and_enums()
    assert {m.value for m in Mode} | {s.value for s in SubMode} <= values


def test_optional_fields_are_omitted_when_unknown():
    payload = NavigationState(timestamp="t", mode=Mode.IDLE, reason="r").to_payload()
    assert "voyage_id" not in payload and "route_ref" not in payload
    assert payload["authority_level_in_force"] == "AUTHORITY_LEVEL_UNKNOWN"
    assert payload["conn_held"] is False
