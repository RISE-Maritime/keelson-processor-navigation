# keelson-processor-navigation — TODO

Single source of truth for **pending** work, as a checklist. Completed items are pruned; `git log`
has the record. The full design is [docs/PLAN.md](docs/PLAN.md); this file is its phase checklist.

**Status:** Phases 0 and 1 are done. Route monitoring publishes `route_execution`,
`envelope_exceedance` and alarms, verified live against Slipway's `slipway-alpha` on 2026-09-14
(1.00 s interval, `route_execution/declared-sim`, `MODE_READY`). Phase 2 is next.

## Invariants (read before touching anything)

- Advice is a record, never a command. `colreg` publishes; `guidance` acts only on `ACCEPTED` advice.
- Leases lapse on the **receiver's own clock**: `local_receipt + lease_ttl_seconds`. Never compare `expires_at`.
- Anything driving `vehicle_control/v1` axes publishes **continuously at 10–20 Hz**. Silence is the stop signal.
- The `route` subject carries two key shapes: `route/{source_id}` (declaration) and `route/{id}/latest` + `route/{id}/edition/{N}` (lineage). Discriminate by chunk count.
- Act on `OperationalAuthority.level`, never `composite_score`. `UNKNOWN` is non-authorising.
- A dangling `RouteRef` is an alarm, never resolved to `latest`.
- Waypoints are addressed by stable `Waypoint.id`, never by index.
- `@target` is a verbatim chunk: `**` does not cross it; subscribe to the target tree explicitly.
- New payloads ride `raw_json` until the keelson PR lands; field sets mirror the proto drafts exactly.
  A rider is published on `raw_json/{source_base}/{module}` and named by its `record` field, as Slipway does.
- Every new draft subject needs a proto in `docs/upstream/` and an entry in `src/nav/contracts/raw_json_riders.yaml`.
- `entity_id` has no default, and enabling an unimplemented module is a config error.
- Inputs name their `source_id` explicitly; `truth` sources are refused. Freshness is judged on local receipt time.
- Alarm descriptions must be stable while the condition holds: a changed description republishes immediately.
- `envelope_exceedance` is published only with a voyage; a declared route has none.

## Phase 2 — traffic + CPA

- [ ] `world/targets.py`: `TargetTable` from `.../{subject}/*/@target/**` for `location_fix`, `course_over_ground_deg`, `speed_over_ground_knots`, `heading_true_north_deg`, `nav_status`, `vessel_type`, `length_over_all_m`, `breadth_over_all_m`; staleness expiry by AIS class / nav_status
- [ ] `traffic/cpa.py`: port `crowsnest-dev/src/utils/cpaCalculation.js` (relative motion in local frame); add BCR/BCT
- [ ] Publish `target_cpa_m`, `target_tcpa_s`, `target_bcr_m`, `target_bct_s`, `target_bearing_north_deg`, `target_bearing_relative_deg` under `@target/{id}`
- [ ] Unit tests against Crowsnest fixtures; **acceptance**: numbers match the bearing-rate table

## Phase 3 — COLREG advisor (advisory only)

- [ ] `colreg/geometry.py`: aspect, relative bearing, sectors (head-on ±6°, overtaking >22.5° abaft beam)
- [ ] `colreg/classify.py`: Rules 13/14/15, 17 stand-on, 19 restricted visibility (`visibility_range_m` below `RouteInfo.min_visibility_m`); reuse `ColregSituation`
- [ ] `colreg/risk.py`: NONE/WATCH/RISK/DANGER from CPA/TCPA thresholds per `OperationalContext`; hysteresis clear margin 1.2 (`cpaAlerts.js`)
- [ ] `colreg/synthesize.py`: starboard-preference course alteration, speed reduction, hold; validity window; expected CPA after manoeuvre
- [ ] Publish `encounter`, `navigation_advice`; consume/publish `advice_disposition` (riders)
- [ ] `config/colreg.default.yaml`
- [ ] `docs/UPSTREAM-ASKS.md`: paste-ready proto drafts + `subjects.yaml`/`qos.yaml` rows for `encounter`, `navigation_advice`, `advice_disposition`, `navigation_setpoint`, `navigation_state`, `navigation_decision`, `task_allocation`
- [ ] keelson feature branch with the protos; alpha build pin
- [ ] Add `encounter`, `navigation_advice`, `advice_disposition` to `src/nav/contracts/raw_json_riders.yaml` with the upstream check recorded
- [ ] Unit truth table for classification; **acceptance**: `rivo-crossing.yaml` yields one CROSSING_STBD / GIVE_WAY encounter and one advice

## Phase 4 — guidance backend A + conduct FSM

- [ ] `bus/lease.py`: `CommandAuthority` client — request, take (token), heartbeat, release with same token; receiver-clock lapse; `command_request` publishing
- [ ] `guidance/backends/axes.py`: `vehicle_control/v1 set_control_mapping` (`steering`/`throttle`), publish `joystick_x_pct`/`joystick_y_pct` at 10–20 Hz on a dedicated timer repeating last setpoint
- [ ] `guidance/track.py`: XTE + heading controller (start from `keelson-processor-autopilot` dead-reckoned PID), wheel-over from `Waypoint.wheel_over_distance_m`, fly-by turn per `Waypoint.radius_m`, return-to-track
- [ ] `guidance/speed.py`: planned SOG per leg with defaults, `Leg.sog_min/max`, slow-down for UKC / visibility / accepted advice, ETA hold
- [ ] `guidance/failsafe.py`: neutral burst for `neutral_burst_s` then silence
- [ ] `conduct/fsm.py`: IDLE → READY → UNDERWAY{TRACK, AVOIDING, HOLDING, REROUTING} → ARRIVED/ENDED; FALLBACK on lease lapse / ceiling drop / stale inputs
- [ ] `conduct/authority.py`: gates on `OperationalAuthority.level` and conn; auto-accept policy per level
- [ ] `log/decision_log.py`: `navigation_decision/{voyage_id}/{decision_id}` append + 30 s snapshot
- [ ] Publish `navigation_state` (1 Hz), `navigation_setpoint`
- [ ] Unit: FSM guard matrix (every state × every input), lease clock rule with skewed `expires_at`
- [ ] Integration: backend goes silent on FALLBACK; heartbeat cadence
- [ ] **Acceptance**: Slipway vessel in COMMANDED mode sails `two-vessel-harbor.yaml` with XTD inside leg limits; pulling the lease stops her within the dead-man

## Phase 5 — SIL barrier participant

- [ ] `sil/tick_barrier.py`: subscribe Slipway fence / `scenario_status`, run `tick()`, publish `scenario_tick_ack{COMPLETED}` with `run_id`, `tick`
- [ ] Deterministic tests driven by `scenario_control/v1 step()`; recordings via `SLIPWAY/tools/tick_collector.py`
- [ ] Document reproducibility class (R2-conditional)

## Phase 6 — backend B + reroute

- [ ] `guidance/backends/goto.py`: `vehicle_navigation/v1 set_navigation_target` + `set_cruise_speed`; handle `NOT_OBSERVABLE` by polling `navigation_target`; `set_mode("HOLD")` failsafe
- [ ] `config/platforms/ghost-drone.yaml`; test against ghost-drone SITL
- [ ] `route_planner/v1` client (`consolidation=NONE`, bare protobuf) for REROUTING within `DynamicReroutingPolicy`; `select_route` → §6.4 bump; `Voyage` audit `REROUTE_ACCEPTED`

## Phase 7 — conduct / task allocation + proto swap

- [ ] Settle `task_allocation` contract in writing (ROC framework §17/§21 open questions) before code
- [ ] `conduct/allocation.py`: publish `task_allocation/{function_id}` for NAV-01…NAV-08
- [ ] Crowsnest Conduct panel reads it (crowsnest-dev change, own PR)
- [ ] Flip codec riders → protos once keelson release carries them; retire riders

## Needs a person

- [ ] With the processor running against `slipway-alpha`, confirm Crowsnest's Route tab chip reads *From vessel* (it reads `route_execution/*`, keyed by vessel, stale after 5 s, and rejects a different route id). Checked on the wire, not in a browser
- [ ] Decide whether `route_execution` for a declared route should be keyed `declared-{source}` or wait for a voyage (see Open questions)

## Open questions

- [ ] A declared route has no voyage, so its execution is keyed `route_execution/declared-{source}` with an empty `voyage_id`. Raise upstream alongside the two-key-shapes ask (crowsnest-dev NAVIGATION-DECISION-SERVICES §6.2)
- [ ] `RouteExecution.speed_over_ground_knots` and `course_over_ground_deg` are not `optional`, so an unknown SOG reads as 0 kn. Ask keelson for presence
- [ ] `keelson-processor-autopilot` v0.2.0 already steers `slipway-alpha` via `wheel_position_pct` + `--install-control-mapping` (keelson-platforms). Phase 4 must decide: replace it, or feed it

- [ ] CPA/TCPA thresholds and safe-passing distance per `OperationalContext`
- [ ] Auto-accept advice at `ASSISTED_AUTONOMOUS` (below DANGER) or only at `FULL_AUTONOMOUS`?
- [ ] keelson issue: `ControlAxisMapping` carries no authority token, so a receiver can observe but not enforce the conn
- [ ] Shore-side fallback advisor and its link-loss policy for hulls without onboard compute
