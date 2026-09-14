# keelson-processor-navigation — TODO

Single source of truth for **pending** work, as a checklist. Completed items are pruned; `git log`
has the record. The full design is [docs/PLAN.md](docs/PLAN.md); this file is its phase checklist.

**Status:** Phases 0, 1 and 2 are done; Phase 3 is in progress. Route monitoring publishes `route_execution`,
`envelope_exceedance` and alarms (live against Slipway `slipway-alpha`: 1.00 s interval,
`MODE_READY`). Traffic publishes per-target CPA under `@target` (live with real AIS from
`maritimedb`: 110 targets in 12 NM, zero errors; 2,550 of 2,554 published CPAs within the
dead-reckoning bound of Crowsnest's formula recomputed from the raw inputs, median difference 0 m).


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

In progress. Design decisions:
- Classification uses heading where known, course otherwise (Rules 13-15 turn on aspect; Slipway ADR-0063). Sectors: ±6° head-on, 112.5° abaft beam. Rule 18 from AIS `nav_status` only; stopped or anchored targets are obstructions (Rule 8); Rule 19 in restricted visibility gives no stand-on vessel.
- Not modelled, and said so in every advice rationale: narrow channels (9), traffic separation schemes (10), chart and depth. Advice is never checked against the chart in this phase.
- An encounter opens at RISK, latches its situation and role for its life (roles do not flip as geometry changes), and closes past the clear margin (1.2×) or when range is opening beyond it.
- Thresholds per `OperationalContext` of the current leg (open sea 1 NM / 12 min, down to port area); unspecified uses coastal.
- Safe passing: the vessels an advice answers must be cleared to the RISK distance; every other target must stay outside DANGER. Holding bystanders to the RISK distance let one ship passing at its edge veto every sound manoeuvre (found on `rivo-crossing`: TARGET 1 on Alpha's starboard quarter).
- One active advice for the vessel, covering every open encounter, restated with the same id while it holds and superseded when the manoeuvre changes. Give-way: smallest starboard alteration ≥ 30° (Rule 8 "readily apparent") clearing every in-range target, then speed reduction, then stop. Stand-on: maintain; act under 17(a)(ii) only when the give-way vessel is not acting and TCPA is short, never to port for a target on the port side (17(c)).
- `advice_disposition` from outside (Crowsnest, later `conduct`) is recorded, never acted on in this phase.
- Acceptance runs Slipway's kernel in-process on `rivo-crossing` to a recorded fixture, not on the shared bus, so the running `slipway-alpha` exercise is not disturbed.

- [x] `colreg/classify.py`: sectors, Rules 13/14/15/17/18/19, stopped targets, unknown directions; truth-table tests
- [x] `colreg/risk.py`: NONE/WATCH/RISK/DANGER per operational context, hysteresis clear margin 1.2
- [x] `colreg/synthesize.py`: candidate manoeuvres evaluated against every in-range target; expected CPA and minimum range
- [x] `colreg/advisor.py`: encounter lifecycle with latched roles, advice restatement/supersession/expiry, 17(a)(ii) trigger, RESUME_TRACK when avoiding encounters close
- [x] Draft protos `Encounter`, `NavigationAdvice`, `AdviceDisposition` in `docs/upstream/`, rider register entries, payload classes checked against the drafts
- [x] Wiring: `WaypointPlan.operational_context`, visibility input for Rule 19, `advice_disposition` rider subscription, publish `encounter/{voyage|novoyage}/{id}`, `navigation_advice/...`, collision alarms
- [x] Advisory-by-construction test: the colreg module can publish only encounter, advice, disposition and alarm subjects
- [ ] e2e on an isolated session: encounter rider on the module's own `raw_json` key, inbound `advice_disposition` rider reaches the advisor
- [ ] keelson worktree `feature/navigation-decision-payloads`: drafts + `subjects.yaml` / `qos.yaml` rows, compiled by `generate_python.sh`, committed locally (not pushed)
- [x] `docs/UPSTREAM-ASKS.md`: paste-ready protos + `subjects.yaml` / `qos.yaml` rows
- [x] Fixture from Slipway `rivo-crossing`, kernel run in-process for 900 s (`tests/fixtures/rivo_crossing.jsonl`)
- [x] **Acceptance** (`tests/test_acceptance_rivo.py`, green): one CROSSING_STBD / GIVE_WAY encounter for Alpha against Bravo, DANGER at peak, closed at the pass (~420 s); first advice a 30-90° starboard alteration that clears Bravo to the RISK distance and keeps TARGET 1 outside DANGER (recomputed from the fixture); RESUME_TRACK supersedes it; collision alarm raised and cleared; only advisory subjects published

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

- [ ] SLIPWAY: `rivo-crossing` no longer sails as its comments say. The kernel runs Bravo at 6.0 kn (the order says 14.7) and the closest approach is 394 m at 420 s, not 100 m at 277 s; Slipway's own `tests/deterministic/test_rivo_crossing.py` fails (484 m within its 360 s). Last touched by `1dfae03` on 2026-09-13. The fixture here records what the kernel does now

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
