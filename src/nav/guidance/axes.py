"""Backend A: vehicle_control/v1 axes. The mapping this processor installs, whose mapping is
installed now, and what the tick asks the bus layer to do.

keelson VehicleControl.proto: ``steering`` and ``throttle`` are the only axes; a publisher MUST
publish continuously at 10-20 Hz and silence is the stop signal. Slipway releases a vessel to her
failsafe when either axis goes quiet, so both are always published together.
"""

from __future__ import annotations

from dataclasses import dataclass

AXIS_SUBJECTS = {"steering": "joystick_x_pct", "throttle": "joystick_y_pct"}

FREE = "free"
OURS = "ours"
OTHER = "other"
UNKNOWN = "unknown"


def wanted_mapping(source_id: str, max_axis_age_s: float) -> dict:
    """ControlAxisMapping in protobuf JSON form. entity_id is left empty: the responder's own."""
    return {
        "axes": {
            axis: {"subject": subject, "source_id": source_id}
            for axis, subject in AXIS_SUBJECTS.items()
        },
        "max_axis_age_s": max_axis_age_s,
    }


def ownership(mapping: dict | None, entity_id: str, source_id: str) -> str:
    """unknown: no responder answered. free: no axes mapped. ours: exactly our two axes."""
    if mapping is None:
        return UNKNOWN
    axes = mapping.get("axes") or {}
    if not axes:
        return FREE
    if set(axes) != set(AXIS_SUBJECTS):
        return OTHER
    for axis, subject in AXIS_SUBJECTS.items():
        a = axes[axis]
        if (
            a.get("subject") != subject
            or a.get("source_id") != source_id
            or a.get("entity_id", "") not in ("", entity_id)
        ):
            return OTHER
    return OURS


@dataclass(frozen=True)
class Actuation:
    """What the tick wants on the helm until ``valid_until_ns``. ``axes`` None means silence.

    The bus publisher repeats ``axes`` at its own rate and stops at ``valid_until_ns``, so a tick
    loop that stalls goes silent instead of holding the last order."""

    axes: tuple[float, float] | None = None  # (steering_pct, throttle_pct)
    valid_until_ns: int = 0
    mapping: str | None = None  # "install" | "release"
