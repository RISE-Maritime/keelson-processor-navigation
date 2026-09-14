"""Runtime configuration: built-in DEFAULTS < YAML file (--config) < CLI flags.

``entity_id`` has no default: it names the vessel this processor navigates, and a wrong one
navigates the wrong ship. Input sources have no wildcard default for the same kind of reason: a
vessel can publish one subject from several sources (a GNSS and an EKF, or Slipway's ``sim`` and
``truth``), and silently reading the wrong one is worse than refusing to start.
"""

from __future__ import annotations

import copy
import logging
from pathlib import Path

logger = logging.getLogger(__name__)

MODULES = ("route_monitor", "traffic", "colreg", "guidance", "conduct", "decision_log")

# Enabling any other module is refused, never ignored.
IMPLEMENTED_MODULES = frozenset({"conduct", "route_monitor"})

CODECS = ("rider", "proto")

OWNSHIP_INPUTS = (
    "location_fix",
    "heading_true_north_deg",
    "course_over_ground_deg",
    "speed_over_ground_knots",
)
REQUIRED_INPUTS = {"route_monitor": ("location_fix",)}

ROUTE_MONITOR_DEFAULTS = {
    "execution_hz": 1.0,
    "max_state_age_s": 3.0,
    "measurement_max_age_s": 30.0,
    "declared_stale_s": 35.0,
    "monitor_declared_route": True,
    "arrival_radius_m": 50.0,
    "reacquire_margin_m": 100.0,
    "min_sog_for_eta_knots": 0.5,
    "xtd_limit_fallback_m": 50.0,
    "deviation_clear_fraction": 0.8,
    "envelope_clear_margin": 0.02,
    "alarms": True,
    "alarm_republish_s": 5.0,
    "alarm_ttl_s": 15.0,
}

DEFAULTS: dict = {
    "realm": "rise",
    "entity_id": None,
    "source_base": "navigation/0",
    "codec": "rider",
    "tick_hz": 10.0,
    "state_hz": 1.0,
    "modules": {m: m == "conduct" for m in MODULES},
    "inputs": {},
    "route_monitor": dict(ROUTE_MONITOR_DEFAULTS),
}

_CLI_OVERRIDES = ("realm", "entity_id", "source_base", "codec")
_MERGED_SECTIONS = ("modules", "route_monitor")


class ConfigError(ValueError):
    """Raised when a config fails validation."""


def allowed_inputs() -> tuple[str, ...]:
    from .route_monitor.envelope import ENVELOPE_SUBJECTS

    return tuple(dict.fromkeys((*OWNSHIP_INPUTS, *sorted(ENVELOPE_SUBJECTS))))


def _positive(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and value > 0


def _validate(conf: dict) -> None:
    for key in ("realm", "entity_id", "source_base"):
        if not isinstance(conf.get(key), str) or not conf[key]:
            if key == "entity_id":
                raise ConfigError(
                    "entity_id is required: it names the vessel this processor navigates"
                )
            raise ConfigError(f"config field {key!r} must be a non-empty string")
    for key in ("realm", "entity_id"):
        if "/" in conf[key] or "*" in conf[key]:
            raise ConfigError(f"config field {key!r} must be a single literal key chunk")
    if conf["codec"] not in CODECS:
        raise ConfigError(f"codec must be one of {CODECS}, got {conf['codec']!r}")
    for key in ("tick_hz", "state_hz"):
        if not _positive(conf.get(key)):
            raise ConfigError(f"config field {key!r} must be a positive number")
    if conf["state_hz"] > conf["tick_hz"]:
        raise ConfigError("state_hz cannot exceed tick_hz: state is published from the tick")

    modules = conf.get("modules")
    if not isinstance(modules, dict):
        raise ConfigError("modules must be a mapping of module name to bool")
    unknown = set(modules) - set(MODULES)
    if unknown:
        raise ConfigError(f"unknown module(s): {sorted(unknown)}; known: {list(MODULES)}")
    for name, enabled in modules.items():
        if not isinstance(enabled, bool):
            raise ConfigError(f"modules.{name} must be true or false")
        if enabled and name not in IMPLEMENTED_MODULES:
            raise ConfigError(
                f"module {name!r} is enabled but not implemented in this version; disable it"
            )

    _validate_inputs(conf)
    _validate_route_monitor(conf)


def _validate_inputs(conf: dict) -> None:
    inputs = conf.get("inputs")
    if not isinstance(inputs, dict):
        raise ConfigError("inputs must be a mapping of subject to source_id")
    allowed = allowed_inputs()
    for subject, source in inputs.items():
        if subject not in allowed:
            raise ConfigError(
                f"inputs.{subject} is not an input this version reads: {list(allowed)}"
            )
        if not isinstance(source, str) or not source:
            raise ConfigError(f"inputs.{subject} must be a non-empty source_id")
        if "**" in source or "@" in source:
            raise ConfigError(f"inputs.{subject} must name a source, not a key pattern: {source!r}")
        if source == "truth" or source.endswith("/truth"):
            raise ConfigError(
                f"inputs.{subject} reads {source!r}: simulator ground truth is not an input "
                "for software under test; use the operational source"
            )
    for module, required in REQUIRED_INPUTS.items():
        if conf["modules"].get(module):
            missing = [s for s in required if s not in inputs]
            if missing:
                raise ConfigError(f"module {module!r} needs inputs.{missing[0]} set to a source_id")


def _validate_route_monitor(conf: dict) -> None:
    rm = conf.get("route_monitor")
    if not isinstance(rm, dict):
        raise ConfigError("route_monitor must be a mapping")
    unknown = set(rm) - set(ROUTE_MONITOR_DEFAULTS)
    if unknown:
        raise ConfigError(f"unknown route_monitor setting(s): {sorted(unknown)}")
    for key, default in ROUTE_MONITOR_DEFAULTS.items():
        value = rm[key]
        if isinstance(default, bool):
            if not isinstance(value, bool):
                raise ConfigError(f"route_monitor.{key} must be true or false")
        elif key in ("envelope_clear_margin",):
            if not isinstance(value, (int, float)) or isinstance(value, bool) or not 0 <= value < 1:
                raise ConfigError(f"route_monitor.{key} must be in [0, 1)")
        elif not _positive(value):
            raise ConfigError(f"route_monitor.{key} must be a positive number")
    if not rm["deviation_clear_fraction"] < 1:
        raise ConfigError("route_monitor.deviation_clear_fraction must be below 1")
    if rm["alarm_ttl_s"] <= rm["alarm_republish_s"]:
        raise ConfigError("route_monitor.alarm_ttl_s must exceed alarm_republish_s")
    if rm["execution_hz"] > conf["tick_hz"]:
        raise ConfigError("route_monitor.execution_hz cannot exceed tick_hz")


def load(args=None) -> dict:
    """Assemble the runtime config from DEFAULTS, an optional YAML file and CLI args."""
    conf = copy.deepcopy(DEFAULTS)

    config_path = getattr(args, "config", None)
    if config_path:
        loaded = yaml_safe_load(config_path)
        for key, value in loaded.items():
            if key in _MERGED_SECTIONS and isinstance(value, dict):
                conf[key].update(value)
            elif key == "inputs" and isinstance(value, dict):
                conf["inputs"] = {k: v for k, v in value.items() if v is not None}
            elif value is not None:
                conf[key] = value

    for key in _CLI_OVERRIDES:
        value = getattr(args, key, None) if args is not None else None
        if value is not None:
            conf[key] = value

    _validate(conf)
    enabled = [m for m, on in conf["modules"].items() if on]
    logger.info(
        "Config: realm=%s entity_id=%s source_base=%s codec=%s modules=%s inputs=%s",
        conf["realm"],
        conf["entity_id"],
        conf["source_base"],
        conf["codec"],
        enabled,
        conf["inputs"],
    )
    return conf


def module_source_id(conf: dict, module: str) -> str:
    """The source_id a module publishes under: one decision, one source."""
    return f"{conf['source_base']}/{module}"


def yaml_safe_load(path: str | Path) -> dict:
    import yaml

    path = Path(path)
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, dict):
        raise ConfigError(f"config {path} did not parse to a mapping")
    return loaded
