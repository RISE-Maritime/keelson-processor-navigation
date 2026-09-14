# keelson-processor-navigation — TODO

Single source of truth for **pending** work, as a checklist. Completed items are pruned; `git log`
has the record. The full design is [docs/PLAN.md](docs/PLAN.md); this file is its phase checklist.

**Status:** Phases 0 to 3 are done. Route monitoring publishes `route_execution`,
`envelope_exceedance` and alarms (live against Slipway `slipway-alpha`, and *From vessel* in
Crowsnest). Traffic publishes per-target CPA under `@target` (live with real AIS; matches Crowsnest's
formula). The COLREG advisor publishes encounters, advice and dispositions and commands nothing
(accepted against Slipway's `rivo-crossing` replayed in-process). Draft keelson payloads compile on a
local keelson branch and the processor's payloads parse into them. Phase 4, guidance, is next.

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
- COLREG advice is a record. The colreg module emits only encounter, navigation_advice, advice_disposition and alarm; a test holds it to that.
- Encounter situation and own role are latched when an encounter opens.
- Safe passing: encounter vessels to the RISK distance, every other target outside DANGER.
- A rider is published on its module's own `raw_json` key; ids travel in the payload.
- `entity_id` has no default, and enabling an unimplemented module is a config error.
- Inputs name their `source_id` explicitly; `truth` sources are refused. Freshness is judged on local receipt time.
- Alarm descriptions must be stable while the condition holds: a changed description republishes immediately.
- `envelope_exceedance` is published only with a voyage; a declared route has none.
- `maritimedb` AIS stamps each report at publish time, not transmission time: the timestamp carries no report age. Dead-reckon targets from local receipt.
- Traffic publishes only defined values: no CPA from an unreported course or speed, no bow crossing astern.

## Phase 4 — guidance backend A + conduct FSM

Guidance replaces `keelson-processor-autopilot` on any vessel it drives (decision 2026-09-14, see Needs a person). Consequences for this phase:

- [ ] Never share a vessel with a running `keelson-processor-autopilot`: Slipway keeps one control mapping per vessel, last writer wins, and the autopilot re-installs itself whenever the mapping is empty. Documented under README Deployment
- [ ] On start, `get_control_mapping`: if axes are mapped to another source, do not take the helm; publish `navigation_state` with the reason and raise an alarm. Never overwrite another controller's mapping silently
- [ ] Publish axes at 10–20 Hz on a dedicated timer (the contract's rate; the autopilot's 5 Hz is below it)
- [ ] Slipway scenario `alpha-navigation`, modelled on `alpha-autopilot.yaml`, requiring `slipway-alpha` / `navigation/0/guidance` / `joystick_x_pct` instead of `autopilot` / `wheel_position_pct`. Run in-process first; on the bus only with the autopilot container stopped
- [ ] Open: backend B (`vehicle_navigation/v1`) for real ArduPilot hulls has no Slipway counterpart to test against

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
- [x] Keelson PR opened: [RISE-Maritime/keelson#260](https://github.com/RISE-Maritime/keelson/pull/260), branch `feature/navigation-decision-payloads` (`bebc8e6` payloads, `e096c3a` tests), against `dev`. Verified locally: Python generation clean, SDK tests 259 passed, ruff and black clean, processor payloads parse into the generated classes. JavaScript generation fails locally (foxglove-sdk build on Python 3.14) and passed in CI: all eight checks green on 2026-09-14, including docker-build and connectors e2e. Next: review, by a person; switch `codec: proto` and retire the riders once it is released
- [x] Phase 4 replaces `keelson-processor-autopilot` on vessels guidance drives (decided 2026-09-14). Feeding it is not possible: v0.2.0 reads a GeoJSON LineString once at startup, has no Route, setpoint, speed or `command_authority` input, publishes at 5 Hz, and re-takes the helm within ~5 s of the control mapping going empty. The autopilot stays as-is for simple track demos; its v0.2.0 image is a local build of its open PR #5
- [ ] Crowsnest state changed by the agent on 2026-09-14 with permission: own ship set to *Slipway Alpha (SIM)* and `remote_control_target_override_v1` set to `slipway-alpha` (it was `drone-sim-01`). Previous values are in localStorage key `claude_backup_before_slipway_alpha_20260914`; restore or delete it

## Open questions

- [x] SLIPWAY drift landed on Slipway's local `main` on 2026-09-14, not pushed (local `main` is far ahead of GitHub's; pushing is for a person). Deterministic suite: 15 failures before, 1790 passed and 0 failed after.
  - `8fa3b2c` rivo-crossing sails its documented exercise again (`1dfae03` had moved both starts and ordered Bravo past her cruise).
  - `711e09a` alpha's 150 m / 20.5 kn hull is `alpha@2.0.0`, not an edit to 1.0.0. The OSC elaborator now carries `draught_m`. `workboat-set-and-drift` is restated at 20.5 kn (course made good 005.572, 20.597 kn), because an `.osc` twin cannot carry a pilot order.
  - `4e10508` and `a84b167` re-pin the three trajectory digests and regenerate the extent fixture. Proof in the commit: the cause was alpha plus seven unpinned L2 fields.
  - `1ce1d92` and `4cd2370` record the pilot shortfall (ADR-0049 clause 4) on vessel and contact state and on the truth record as `pilot_shortfall_mps` and `pilot_course_unreachable`. All six digests re-pinned with a stripped-key proof.
  - keelson-platforms `ab38eee` (local `main`, not pushed): `slipway-alpha` advertises 150 m / 30 m / 20.5 kn, and Slipway's package contract test passes against it.
  - The fixture here, regenerated from the final Slipway `main` with `tools/rivo_crossing_fixture.py`, is byte-identical to the committed one, so nothing moved rivo-crossing's trajectory.
- [ ] Slipway branch `docs/slipway-drift-todo` (`f94e2af`) replaces the three resolved Slipway TODO entries with two left open: alpha 2.0.0's render assets still describe the 6 m boat, and an `.osc` scripted step cannot address a vessel. Not merged, because another session has uncommitted edits to `TODO.md` in Slipway's main checkout. Merge when that is committed
- [ ] Slipway `./scripts/ci-local.sh --fast` stops at lint on four errors older than this work: `library_rpc.py`, `test_crowsnest_registry_contract.py`, `test_world_vocabulary.py` (last changed 2026-09-12/13). The deterministic tests it would then run pass

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
