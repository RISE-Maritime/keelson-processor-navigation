from __future__ import annotations

import pytest

from nav.colreg.risk import NM, Risk, Thresholds, grade, thresholds_for

pytestmark = pytest.mark.unit

TH = Thresholds(cpa_m=0.5 * NM, tcpa_s=480, danger_cpa_m=0.25 * NM, danger_tcpa_s=240)


@pytest.mark.parametrize(
    "cpa_nm, tcpa_s, expected",
    [
        (0.1, 120, Risk.DANGER),
        (0.1, 400, Risk.RISK),  # close but not soon
        (0.4, 400, Risk.RISK),
        (0.8, 400, Risk.WATCH),
        (0.4, 900, Risk.WATCH),
        (1.2, 400, Risk.NONE),
        (0.1, -30, Risk.NONE),  # closest approach already past
        (0.1, 0.0, Risk.NONE),
    ],
)
def test_grades(cpa_nm, tcpa_s, expected):
    assert grade(cpa_nm * NM, tcpa_s, TH) == expected


def test_unknown_geometry_is_not_risk():
    assert grade(None, 60, TH) == Risk.NONE and grade(100.0, None, TH) == Risk.NONE


def test_hysteresis_holds_a_grade_until_the_clear_margin():
    # Just outside the RISK CPA: not raised from NONE, but held when already at RISK.
    cpa = 0.55 * NM
    assert grade(cpa, 400, TH, previous=Risk.NONE) == Risk.WATCH
    assert grade(cpa, 400, TH, previous=Risk.RISK) == Risk.RISK
    assert grade(0.65 * NM, 400, TH, previous=Risk.RISK) == Risk.WATCH


def test_hysteresis_never_raises_a_grade():
    assert grade(0.3 * NM, 400, TH, previous=Risk.DANGER) == Risk.RISK


def test_context_lookup_falls_back_to_coastal():
    assert thresholds_for("OPERATIONAL_CONTEXT_PORT_AREA").cpa_m < thresholds_for(None).cpa_m
    assert thresholds_for("OPERATIONAL_CONTEXT_UNSPECIFIED") == thresholds_for(
        "OPERATIONAL_CONTEXT_COASTAL"
    )
    assert Risk.DANGER.proto_name == "RISK_DANGER"
