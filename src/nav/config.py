"""Runtime configuration: built-in DEFAULTS < YAML file (--config) < CLI flags.

``entity_id`` has no default. It names the vessel this processor navigates, and a wrong one
navigates the wrong ship, so it must be stated explicitly.
"""

from __future__ import annotations

import logging
from pathlib import Path

logger = logging.getLogger(__name__)

MODULES = ("route_monitor", "traffic", "colreg", "guidance", "conduct", "decision_log")

# Modules that exist in this version. Enabling any other is refused, never ignored: an operator
# who turned a safety function on must not be left believing it runs.
IMPLEMENTED_MODULES = frozenset({"conduct"})

CODECS = ("rider", "proto")

DEFAULTS: dict = {
    "realm": "rise",
    "entity_id": None,
    "source_base": "navigation/0",
    "codec": "rider",
    "tick_hz": 10.0,
    "state_hz": 1.0,
    "modules": {m: m == "conduct" for m in MODULES},
}

_CLI_OVERRIDES = ("realm", "entity_id", "source_base", "codec")


class ConfigError(ValueError):
    """Raised when a config fails validation."""


def _validate(conf: dict) -> None:
    for key in ("realm", "entity_id", "source_base"):
        if not isinstance(conf.get(key), str) or not conf[key]:
            if key == "entity_id":
                raise ConfigError(
                    "entity_id is required: it names the vessel this processor navigates"
                )
            raise ConfigError(f"config field {key!r} must be a non-empty string")
    for key in ("realm", "entity_id"):
        if "/" in conf[key]:
            raise ConfigError(f"config field {key!r} must be a single key chunk (no '/')")
    if conf["codec"] not in CODECS:
        raise ConfigError(f"codec must be one of {CODECS}, got {conf['codec']!r}")
    for key in ("tick_hz", "state_hz"):
        value = conf.get(key)
        if not isinstance(value, (int, float)) or isinstance(value, bool) or value <= 0:
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


def load(args=None) -> dict:
    """Assemble the runtime config from DEFAULTS, an optional YAML file and CLI args."""
    conf = dict(DEFAULTS)
    conf["modules"] = dict(DEFAULTS["modules"])

    config_path = getattr(args, "config", None)
    if config_path:
        loaded = yaml_safe_load(config_path)
        for key, value in loaded.items():
            if key == "modules" and isinstance(value, dict):
                conf["modules"].update(value)
            elif value is not None:
                conf[key] = value

    for key in _CLI_OVERRIDES:
        value = getattr(args, key, None) if args is not None else None
        if value is not None:
            conf[key] = value

    _validate(conf)
    enabled = [m for m, on in conf["modules"].items() if on]
    logger.info(
        "Config: realm=%s entity_id=%s source_base=%s codec=%s modules=%s",
        conf["realm"],
        conf["entity_id"],
        conf["source_base"],
        conf["codec"],
        enabled,
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
