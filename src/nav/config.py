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

MODULES = ("route_monitor", "traffic", "colreg", "guidance", "conduct", "decision_log", "control")

# Enabling any other module is refused, never ignored.
IMPLEMENTED_MODULES = frozenset(
    {"conduct", "route_monitor", "traffic", "colreg", "guidance", "decision_log", "control"}
)

CODECS = ("rider", "proto")

OWNSHIP_INPUTS = (
    "location_fix",
    "heading_true_north_deg",
    "course_over_ground_deg",
    "speed_over_ground_knots",
    "yaw_rate_degps",
)
REQUIRED_INPUTS = {
    "route_monitor": ("location_fix",),
    "traffic": ("location_fix",),
    "colreg": ("location_fix", "course_over_ground_deg", "speed_over_ground_knots"),
    "guidance": (
        "location_fix",
        "heading_true_north_deg",
        "course_over_ground_deg",
        "speed_over_ground_knots",
    ),
}

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

TRAFFIC_DEFAULTS = {
    "feeds": [],
    "range_limit_m": 12 * 1852.0,
    "range_exit_factor": 1.1,
    "target_max_age_s": 180.0,
    "static_max_age_s": 1800.0,
    "publish_hz": 1.0,
    "dead_reckon": True,
    "exclude_targets": [],
}

COLREG_DEFAULTS = {
    "default_context": "OPERATIONAL_CONTEXT_COASTAL",
    "thresholds": {},  # per OPERATIONAL_CONTEXT_*: {cpa_m, tcpa_s, danger_cpa_m, danger_tcpa_s}
    "clear_margin": 1.2,
    "advice_validity_s": 30.0,
    "advice_restate_s": 5.0,
    "restricted_visibility_m": 1852.0,
    "min_alteration_deg": 30.0,
    "max_alteration_deg": 90.0,
    "alteration_step_deg": 10.0,
    "speed_factors": [0.5, 0.25],
    "horizon_s": 1800.0,
    "alarms": True,
}

CONDUCT_DEFAULTS = {
    "controller_id": None,  # None: "{source_base}@{entity_id}"
    "controller_site": "onboard",
    "lease_ttl_s": 30.0,
    "heartbeat_s": 10.0,
    "request_conn": False,
    "request_interval_s": 10.0,
    "take_conn_when_free": False,
    "authority_source": None,
    "authority_max_age_s": 10.0,
    "fixed_authority_level": None,
    "auto_accept_advice": False,
}

GUIDANCE_DEFAULTS = {
    "backend": "axes",
    "responder_id": "*",
    "publish_hz": 10.0,
    "command_hold_s": 0.5,
    "neutral_burst_s": 2.0,
    "max_axis_age_s": 1.0,
    "mapping_poll_s": 2.0,
    "mapping_max_age_s": 6.0,
    "rpc_timeout_s": 2.0,
    "setpoint_hz": 2.0,
    "full_ahead_sog_knots": None,
    "default_sog_knots": 6.0,
    "arrival_sog_knots": 2.0,
    "arrival_slowdown_m": 300.0,
    "restricted_visibility_factor": 0.5,
    "throttle_kp_pct_per_knot": 1.0,
    "predict_s": 5.0,
    "lookahead_s": 20.0,
    "min_lookahead_m": 100.0,
    "max_intercept_deg": 45.0,
    "default_turn_radius_m": 200.0,
    "heading_kp_pct_per_deg": 2.0,
    "yaw_kd_pct_per_degps": 2.0,
    "max_drift_deg": 30.0,
    "min_sog_for_drift_knots": 1.0,
    # A go_to order without arrival_radius_m arrives here (keelson#308 leaves it to the process).
    "goto_arrival_radius_m": 20.0,
    # A loiter without return_speed_knots makes back at this (keelson#320; Slipway's default).
    "loiter_return_knots": 2.0,
}

CONTROL_DEFAULTS = {
    # Controllers that may direct this process while it holds the conn itself, without presenting
    # a lease token: a scenario runner in simulation, a conduct service once one exists. Anyone
    # holding the conn may always direct it. Empty: only the conn holder.
    "controllers": [],
    "reply_timeout_s": 3.0,  # how long a call waits for the tick to decide it
    "fetch_timeout_s": 3.0,  # load_route: reading another vessel's route edition from storage
}

DECISION_LOG_DEFAULTS = {
    "snapshot_s": 30.0,
    "snapshot_count": 20,
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
    "traffic": copy.deepcopy(TRAFFIC_DEFAULTS),
    "colreg": copy.deepcopy(COLREG_DEFAULTS),
    "conduct": dict(CONDUCT_DEFAULTS),
    "guidance": dict(GUIDANCE_DEFAULTS),
    "decision_log": dict(DECISION_LOG_DEFAULTS),
    "control": copy.deepcopy(CONTROL_DEFAULTS),
}

_CLI_OVERRIDES = ("realm", "entity_id", "source_base", "codec")
_MERGED_SECTIONS = (
    "modules",
    "route_monitor",
    "traffic",
    "colreg",
    "conduct",
    "guidance",
    "decision_log",
    "control",
)


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
    _validate_traffic(conf)
    _validate_colreg(conf)
    _validate_conduct(conf)
    _validate_guidance(conf)
    _validate_decision_log(conf)
    _validate_control(conf)


def _section(conf: dict, name: str, defaults: dict) -> dict:
    section = conf.get(name)
    if not isinstance(section, dict):
        raise ConfigError(f"{name} must be a mapping")
    unknown = set(section) - set(defaults)
    if unknown:
        raise ConfigError(f"unknown {name} setting(s): {sorted(unknown)}")
    return section


def _validate_conduct(conf: dict) -> None:
    from .conduct.authority import LEVELS

    cd = _section(conf, "conduct", CONDUCT_DEFAULTS)
    if cd["controller_id"] is not None and (
        not isinstance(cd["controller_id"], str) or not cd["controller_id"]
    ):
        raise ConfigError("conduct.controller_id must be a non-empty string or unset")
    if not isinstance(cd["controller_site"], str):
        raise ConfigError("conduct.controller_site must be a string")
    for key in ("lease_ttl_s", "heartbeat_s", "request_interval_s", "authority_max_age_s"):
        if not _positive(cd[key]):
            raise ConfigError(f"conduct.{key} must be a positive number")
    if cd["heartbeat_s"] >= cd["lease_ttl_s"]:
        raise ConfigError("conduct.heartbeat_s must be shorter than conduct.lease_ttl_s")
    for key in ("request_conn", "take_conn_when_free", "auto_accept_advice"):
        if not isinstance(cd[key], bool):
            raise ConfigError(f"conduct.{key} must be true or false")
    source = cd["authority_source"]
    if source is not None:
        if not isinstance(source, str) or not source or "*" in source or "@" in source:
            raise ConfigError("conduct.authority_source must name one source_id")
        if _is_truth(source):
            raise ConfigError("conduct.authority_source must not be simulator ground truth")
    if cd["fixed_authority_level"] is not None and cd["fixed_authority_level"] not in LEVELS:
        raise ConfigError(f"conduct.fixed_authority_level must be one of {list(LEVELS)}")
    if cd["fixed_authority_level"] is not None and source is not None:
        raise ConfigError("set conduct.authority_source or conduct.fixed_authority_level, not both")
    if cd["auto_accept_advice"] and not (
        conf["modules"].get("colreg") and conf["modules"].get("guidance")
    ):
        raise ConfigError("conduct.auto_accept_advice needs modules 'colreg' and 'guidance'")


def _validate_guidance(conf: dict) -> None:
    gd = _section(conf, "guidance", GUIDANCE_DEFAULTS)
    if gd["backend"] != "axes":
        raise ConfigError(
            "guidance.backend must be 'axes' (vehicle_control/v1); vehicle_navigation is Phase 6"
        )
    if not isinstance(gd["responder_id"], str) or not gd["responder_id"]:
        raise ConfigError("guidance.responder_id must be a non-empty string")
    if not _positive(gd["publish_hz"]) or not 10 <= gd["publish_hz"] <= 20:
        raise ConfigError("guidance.publish_hz must be 10-20: vehicle_control/v1 requires it")
    for key in sorted(set(GUIDANCE_DEFAULTS) - {"backend", "responder_id", "full_ahead_sog_knots"}):
        if not _positive(gd[key]):
            raise ConfigError(f"guidance.{key} must be a positive number")
    if gd["command_hold_s"] * gd["publish_hz"] < 2:
        raise ConfigError("guidance.command_hold_s must span at least two axis publications")
    if gd["command_hold_s"] >= gd["max_axis_age_s"]:
        raise ConfigError("guidance.command_hold_s must be shorter than guidance.max_axis_age_s")
    if gd["setpoint_hz"] > conf["tick_hz"]:
        raise ConfigError("guidance.setpoint_hz cannot exceed tick_hz")
    if gd["mapping_max_age_s"] <= gd["mapping_poll_s"]:
        raise ConfigError("guidance.mapping_max_age_s must be longer than guidance.mapping_poll_s")
    if gd["max_intercept_deg"] >= 90:
        raise ConfigError("guidance.max_intercept_deg must be below 90")
    if conf["modules"].get("guidance"):
        if not _positive(gd["full_ahead_sog_knots"]):
            raise ConfigError(
                "module 'guidance' needs guidance.full_ahead_sog_knots: the hull's speed at 100 %"
            )
        if not conf["modules"].get("conduct"):
            raise ConfigError("module 'guidance' needs module 'conduct': it drives only UNDERWAY")
        if gd["throttle_kp_pct_per_knot"] * gd["full_ahead_sog_knots"] / 100.0 >= 1.0:
            raise ConfigError(
                "guidance.throttle_kp_pct_per_knot * full_ahead_sog_knots / 100 must be below 1: "
                "a speed loop at gain 1 oscillates on a hull without inertia"
            )


def _validate_decision_log(conf: dict) -> None:
    dl = _section(conf, "decision_log", DECISION_LOG_DEFAULTS)
    if not _positive(dl["snapshot_s"]):
        raise ConfigError("decision_log.snapshot_s must be a positive number")
    if not isinstance(dl["snapshot_count"], int) or dl["snapshot_count"] < 1:
        raise ConfigError("decision_log.snapshot_count must be a positive integer")
    if conf["modules"].get("decision_log") and not conf["modules"].get("conduct"):
        raise ConfigError("module 'decision_log' needs module 'conduct'")


def _validate_control(conf: dict) -> None:
    ct = _section(conf, "control", CONTROL_DEFAULTS)
    controllers = ct["controllers"]
    if not isinstance(controllers, list) or not all(isinstance(c, str) and c for c in controllers):
        raise ConfigError("control.controllers must be a list of non-empty controller ids")
    for key in ("reply_timeout_s", "fetch_timeout_s"):
        if not _positive(ct[key]):
            raise ConfigError(f"control.{key} must be a positive number")
    if conf["modules"].get("control") and not conf["modules"].get("conduct"):
        raise ConfigError(
            "module 'control' needs module 'conduct': a caller is judged by the conn it holds"
        )


def controller_id(conf: dict) -> str:
    """The id this processor holds the conn under."""
    return conf["conduct"]["controller_id"] or f"{conf['source_base']}@{conf['entity_id']}"


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
        if _is_truth(source):
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


def _is_truth(source: str) -> bool:
    return source == "truth" or source.endswith("/truth")


def _validate_traffic(conf: dict) -> None:
    tr = conf.get("traffic")
    if not isinstance(tr, dict):
        raise ConfigError("traffic must be a mapping")
    unknown = set(tr) - set(TRAFFIC_DEFAULTS)
    if unknown:
        raise ConfigError(f"unknown traffic setting(s): {sorted(unknown)}")
    feeds = tr["feeds"]
    if not isinstance(feeds, list):
        raise ConfigError("traffic.feeds must be a list of {entity, source}")
    for i, feed in enumerate(feeds):
        if not isinstance(feed, dict) or set(feed) != {"entity", "source"}:
            raise ConfigError(f"traffic.feeds[{i}] must have exactly entity and source")
        entity, source = feed["entity"], feed["source"]
        if not isinstance(entity, str) or not entity or "/" in entity or "*" in entity:
            raise ConfigError(f"traffic.feeds[{i}].entity must be a single literal key chunk")
        if not isinstance(source, str) or not source or "*" in source or "@" in source:
            raise ConfigError(f"traffic.feeds[{i}].source must name a source, not a pattern")
        if _is_truth(source):
            raise ConfigError(f"traffic.feeds[{i}] reads simulator ground truth ({source!r})")
    for key in ("range_limit_m", "target_max_age_s", "static_max_age_s", "publish_hz"):
        if not _positive(tr[key]):
            raise ConfigError(f"traffic.{key} must be a positive number")
    if not isinstance(tr["range_exit_factor"], (int, float)) or tr["range_exit_factor"] < 1:
        raise ConfigError("traffic.range_exit_factor must be at least 1")
    if not isinstance(tr["dead_reckon"], bool):
        raise ConfigError("traffic.dead_reckon must be true or false")
    if not isinstance(tr["exclude_targets"], list) or not all(
        isinstance(x, str) and x for x in tr["exclude_targets"]
    ):
        raise ConfigError("traffic.exclude_targets must be a list of target ids")
    if tr["publish_hz"] > conf["tick_hz"]:
        raise ConfigError("traffic.publish_hz cannot exceed tick_hz")
    if conf["modules"].get("traffic") and not feeds:
        raise ConfigError("module 'traffic' needs at least one traffic.feeds entry")


def _validate_colreg(conf: dict) -> None:
    from .colreg.risk import DEFAULT_THRESHOLDS

    cr = conf.get("colreg")
    if not isinstance(cr, dict):
        raise ConfigError("colreg must be a mapping")
    unknown = set(cr) - set(COLREG_DEFAULTS)
    if unknown:
        raise ConfigError(f"unknown colreg setting(s): {sorted(unknown)}")
    if cr["default_context"] not in DEFAULT_THRESHOLDS:
        raise ConfigError(f"colreg.default_context must be one of {sorted(DEFAULT_THRESHOLDS)}")
    if not isinstance(cr["thresholds"], dict):
        raise ConfigError("colreg.thresholds must be a mapping of context to thresholds")
    for context, th in cr["thresholds"].items():
        if context not in DEFAULT_THRESHOLDS:
            raise ConfigError(f"colreg.thresholds: unknown context {context!r}")
        if not isinstance(th, dict) or set(th) != {
            "cpa_m",
            "tcpa_s",
            "danger_cpa_m",
            "danger_tcpa_s",
        }:
            raise ConfigError(
                f"colreg.thresholds.{context} needs cpa_m, tcpa_s, danger_cpa_m, danger_tcpa_s"
            )
        if not all(_positive(v) for v in th.values()):
            raise ConfigError(f"colreg.thresholds.{context} values must be positive")
        if th["danger_cpa_m"] > th["cpa_m"] or th["danger_tcpa_s"] > th["tcpa_s"]:
            raise ConfigError(f"colreg.thresholds.{context}: danger must be inside risk")
    for key in (
        "advice_validity_s",
        "advice_restate_s",
        "restricted_visibility_m",
        "horizon_s",
        "min_alteration_deg",
        "max_alteration_deg",
        "alteration_step_deg",
    ):
        if not _positive(cr[key]):
            raise ConfigError(f"colreg.{key} must be a positive number")
    if not isinstance(cr["clear_margin"], (int, float)) or cr["clear_margin"] < 1:
        raise ConfigError("colreg.clear_margin must be at least 1")
    if cr["advice_restate_s"] >= cr["advice_validity_s"]:
        raise ConfigError("colreg.advice_restate_s must be shorter than advice_validity_s")
    if cr["min_alteration_deg"] > cr["max_alteration_deg"]:
        raise ConfigError("colreg.min_alteration_deg cannot exceed max_alteration_deg")
    if not cr["speed_factors"] or not all(
        isinstance(f, (int, float)) and 0 < f < 1 for f in cr["speed_factors"]
    ):
        raise ConfigError("colreg.speed_factors must be fractions between 0 and 1")
    if not isinstance(cr["alarms"], bool):
        raise ConfigError("colreg.alarms must be true or false")
    if conf["modules"].get("colreg") and not conf["modules"].get("traffic"):
        raise ConfigError(
            "module 'colreg' needs module 'traffic': encounters are assessed from its targets"
        )


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
