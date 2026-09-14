# keelson-processor-navigation — TODO

Single source of truth for **pending** work, as a checklist. Completed items are pruned; `git log`
has the record. The full design is [docs/PLAN.md](docs/PLAN.md); this file is its phase checklist.

**Status:** Phases 0, 1 and 2 are done. Route monitoring publishes `route_execution`,
`envelope_exceedance` and alarms (live against Slipway `slipway-alpha`: 1.00 s interval,
`MODE_READY`). Traffic publishes per-target CPA under `@target` (live with real AIS from
`maritimedb`: 110 targets in 12 NM, zero errors; 2,550 of 2,554 published CPAs within the
dead-reckoning bound of Crowsnest's formula recomputed from the raw inputs, median difference 0 m).
Phase 3 is next.

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
- `maritimedb` AIS stamps each report at publish time, not transmission time: the timestamp carries no report age. Dead-reckon targets from local receipt.
- Traffic publishes only defined values: no CPA from an unreported course or speed, no bow crossing astern.

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

- [x] Crowsnest Route tab reads *From vessel* with the processor running against `slipway-alpha` (checked in the browser 2026-09-14: declared route, 4 WPs, leg 2/3, XTD 0 m of 50 m). It first showed *No active route* because Crowsnest's `remote_control_target_override_v1` still named `drone-sim-01`, which outranks the selected platform in `resolveVesselId`
- [ ] Decide whether `route_execution` for a declared route should be keyed `declared-{source}` or wait for a voyage (see Open questions)
- [x] Route chip provenance in the browser: *From vessel* with the processor running, *Computed here* after it stopped and the page reloaded. *Vessel quiet* (stop without reload) not captured: the Crowsnest tab froze
- [ ] CPA/TCPA side by side with Crowsnest's Bearing app target list, time-aligned. Partial on 2026-09-14: one sample 8 s apart agreed (ANCHORED BULKER CPA 0.48/0.48 NM, rel. bearing 54°/50°, TCPA 1.0/1.2 min; NORDIC TRADER CPA 0.31/0.30 NM). A clean aligned run is still to do; the tab froze before it
- [ ] Crowsnest state changed by the agent on 2026-09-14 with permission: own ship set to *Slipway Alpha (SIM)* and `remote_control_target_override_v1` set to `slipway-alpha` (it was `drone-sim-01`). Previous values are in localStorage key `claude_backup_before_slipway_alpha_20260914`; restore or delete it

## Open questions

- [ ] crowsnest-dev: `remote_control_target_override_v1` outranks the selected platform in `resolveVesselId`, silently. A stale `drone-sim-01` made a selected `slipway-alpha` show *No active route* with the vessel declaring one
- [ ] crowsnest-dev: the Bearing app listed a circling Slipway contact (NORDIC TRADER, `mmsi_999001001`) with COG 20° while its reported course swept 270°→204° over 16 s
- [ ] Signalling a `timeout` wrapper delivered SIGINT twice to the processor ("forcing exit"). Run live checks without the wrapper and signal the Python pid

- [ ] keelson upstream: `scaffolding.GracefulShutdown` logs from inside its signal handler. Two signals at once (a `timeout` wrapper forwarding one) raised `RuntimeError: reentrant call inside BufferedWriter` on stderr during a live run's shutdown on 2026-09-14. Shutdown still completed. Signal-safe fix: set the event in the handler and log from the main loop

- [ ] Live CPA cross-check: 4 of 2,554 values exceeded the dead-reckoning bound (up to 455 m). Most likely a target course update between the processor's snapshot and the checker's; not confirmed

- [ ] A declared route has no voyage, so its execution is keyed `route_execution/declared-{source}` with an empty `voyage_id`. Raise upstream alongside the two-key-shapes ask (crowsnest-dev NAVIGATION-DECISION-SERVICES §6.2)
- [ ] `RouteExecution.speed_over_ground_knots` and `course_over_ground_deg` are not `optional`, so an unknown SOG reads as 0 kn. Ask keelson for presence
- [ ] `keelson-processor-autopilot` v0.2.0 already steers `slipway-alpha` via `wheel_position_pct` + `--install-control-mapping` (keelson-platforms). Phase 4 must decide: replace it, or feed it

- [ ] CPA/TCPA thresholds and safe-passing distance per `OperationalContext`
- [ ] Auto-accept advice at `ASSISTED_AUTONOMOUS` (below DANGER) or only at `FULL_AUTONOMOUS`?
- [ ] keelson issue: `ControlAxisMapping` carries no authority token, so a receiver can observe but not enforce the conn
- [ ] Shore-side fallback advisor and its link-loss policy for hulls without onboard compute
