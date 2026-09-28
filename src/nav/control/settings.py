"""navigation_control/v1 get_settings / set_settings (keelson#322): what an operator may tune while
the process runs.

Pure. ``current(...)`` states the settings in force and which are ``fixed``; ``parse(...)`` turns a
request's partial NavigationSettings into the changes to apply, or refuses it WHOLE: a request is
applied whole or not at all, never partly.
"""

from __future__ import annotations

import math

BOOLS = ("colreg_advisor_active", "auto_accept_advice", "route_alarms")
# name -> (lowest allowed, whether the lowest itself is allowed)
NUMBERS = {
    "default_speed_knots": (0.0, False),
    "go_to_arrival_radius_m": (0.0, True),
    "loiter_return_speed_knots": (0.0, False),
    "traffic_range_m": (0.0, False),
}
FIELDS = BOOLS + tuple(NUMBERS)


class SettingsError(ValueError):
    """A value out of range: INVALID_ARGUMENT."""


def current(
    *,
    conf: dict,
    advisor_built: bool,
    guidance_built: bool,
    traffic_built: bool,
    monitor_built: bool,
    colreg_active: bool,
    default_speed_knots: float | None,
    goto_radius_m: float | None,
    loiter_return_knots: float | None,
) -> tuple[dict, list[str]]:
    """``(settings, fixed)``: a setting is present when this process has it; ``fixed`` names the
    ones it has but cannot change while it runs (the advisor, and so auto-accept, only exist when
    started with them)."""
    s: dict = {}
    fixed: list[str] = []
    s["colreg_advisor_active"] = bool(advisor_built and colreg_active)
    if not advisor_built:
        fixed.append("colreg_advisor_active")
    s["auto_accept_advice"] = bool(conf["conduct"].get("auto_accept_advice", False))
    if not (advisor_built and guidance_built):
        fixed.append("auto_accept_advice")
    if guidance_built:
        s["default_speed_knots"] = default_speed_knots
        s["go_to_arrival_radius_m"] = goto_radius_m
        s["loiter_return_speed_knots"] = loiter_return_knots
    if traffic_built:
        s["traffic_range_m"] = float(conf["traffic"]["range_limit_m"])
    if monitor_built:
        s["route_alarms"] = bool(conf["route_monitor"]["alarms"])
    return s, fixed


def parse(request: dict, settings: dict, fixed: list[str]) -> tuple[dict | None, str]:
    """``(changes, why_unsupported)``. Raises SettingsError for a value out of range. Returns
    ``(None, reason)`` when a named setting is fixed or absent (UNSUPPORTED) -- nothing applied."""
    changes: dict = {}
    for name, value in (request or {}).items():
        if name not in FIELDS:
            raise SettingsError(f"unknown setting {name!r}")
        if name in fixed:
            return None, f"{name} cannot change while this process runs (set at start)"
        if name not in settings:
            return None, f"this process has no {name}"
        if name in BOOLS:
            if not isinstance(value, bool):
                raise SettingsError(f"{name} must be true or false")
            changes[name] = value
            continue
        low, inclusive = NUMBERS[name]
        v = float(value)
        if not math.isfinite(v) or v < low or (v == low and not inclusive):
            bound = f">= {low:g}" if inclusive else f"> {low:g}"
            raise SettingsError(f"{name} must be {bound}, not {v}")
        changes[name] = v
    return changes, ""


def describe(before: dict, changes: dict) -> str:
    """'colreg_advisor_active true -> false, default_speed_knots 6 -> 8' for the decision log."""

    def fmt(v):
        return str(v).lower() if isinstance(v, bool) else f"{v:g}"

    return ", ".join(f"{k} {fmt(before.get(k))} -> {fmt(v)}" for k, v in changes.items())
