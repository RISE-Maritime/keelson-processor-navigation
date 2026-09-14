# keelson-processor-navigation — design & implementation plan

## Context

`/Users/ted/Documents/CODE/keelson-processor-navigation` is empty. Everything *around* it already exists:

- **Plan side** (keelson `messages/payloads`): `Route` (RTZ/S-421 superset, `Waypoint.leg` with XTD/speed/UKC limits, `RouteInfo` envelope limits, `ColregSituation` enum on the plan), `Voyage` (pins a `RouteRef`, `DynamicReroutingPolicy`, audit trail), `RouteExecution` (1 Hz snapshot, **defined but has no producer**), `EnvelopeExceedance` (durable breach record, three timestamps, `responses[]`), `RouteChangeEvent`, `RoutePlanner` RPC (P1 skeleton in `keelson-processor-route-planner`), spec §6 choreography (edition store, three-publish bump, voyage activation, lease rules §6.5.1).
- **Authority**: `CommandAuthority` (the conn, single-writer lease, receiver-clock expiry), `OperationalAuthority` (vessel's ceiling, IMO MASS levels 1–5, hysteresis), `RouteEditAuthority`.
- **Vehicle side**: `vehicle_control/v1` (axis mapping onto `joystick_x_pct`/`joystick_y_pct`, 10–20 Hz, dead-man `max_axis_age_s` not disableable), `vehicle_navigation/v1` (`set_navigation_target`, `set_cruise_speed`), `vehicle_mission/v1`, `vehicle_lifecycle/v1`, `vehicle_geofence/v1`. Served by `mavlink2keelson` (real hull) and the **Slipway kernel** (`command_ingress.py`, ADR-0025) for simulation.
- **Targets**: AIS connector fans each target out on atomic subjects under `.../{source_id}/@target/mmsi_{n}` (`location_fix`, `course_over_ground_deg`, `speed_over_ground_knots`, `heading_true_north_deg`, `yaw_rate_degps`, `nav_status`, `vessel_type`, `length_over_all_m`, …). `@target` is a verbatim chunk: `**` does not cross it. Subjects `target_cpa_m`, `target_tcpa_s`, `target_bcr_m`, `target_bct_s`, `target_bearing_north_deg`, `target_bearing_relative_deg` exist with **zero producers**.
- **Reference implementations to port**: `crowsnest-dev/src/services/routeSync.js:1723 computeRouteExecution`, `crowsnest-dev/src/utils/cpaCalculation.js`, `crowsnest-dev/src/services/cpaAlerts.js` (hysteresis 1.2), `crowsnest-dev/src/services/declaredRoute.js` (two `route` key shapes), `RISE-Maritime/keelson-processor-autopilot` (GitHub only, 2025-06: dead-reckoned XTE+heading PID → rudder %, `nvector`, `simple-pid`), Slipway `autopilot.py`/`pilot.py` (PD heading, frame resolution).
- **The gap, in Crowsnest's own words** (`crowsnest-dev/docs/NAVIGATION-DECISION-SERVICES.md`, 2026-09-13): route monitoring "exist, no producer yet"; collision-risk assessment/advice "missing subject family"; conduct/task allocation "missing". Nobody owns the loop between *Voyage ACTIVE* and *setpoints on the actuator*.

**Decisions taken with the user (2026-09-14):**
1. **One repo, modular.** One image, internally split into functions, each with its own `source_id` and enable flag, so the design note's *one decision per source* rule (P6) and the *one owner of navigation under automation* goal both hold.
2. **Dual guidance backend, selectable per platform**: (a) `vehicle_control/v1` + joystick axes at 10–20 Hz (Slipway + mavlink2keelson today); (b) `vehicle_navigation/v1` goto targets + cruise speed (ArduPilot GUIDED).
3. **Draft keelson protos upstream; develop on `raw_json` riders until merged.** Local payload classes mirror the proto drafts field-for-field so the swap is a codec change only.

Boundaries (not owned): route authoring/persistence (Crowsnest, router storage), strategic planning algorithm (route-planner, called via RPC), target detection/tracking (AIS/radar connectors), joystick shaping/multi-station arbitration (`keelson-processor-supervisor`), arming and low-level actuator control (connector / Slipway kernel), alarm presentation (Crowsnest).

---

## 1. Functional decomposition and ownership

| Module (`source_id` suffix) | Role (§6.2) | Owns | Publishes | Consumes |
|---|---|---|---|---|
| `route_monitor` | MON | active Voyage + pinned Route edition; leg/waypoint progress by stable id; XTD; ETAs; envelope-limit standing; exceedance records | `route_execution/{voyage_id}` 1 Hz, `envelope_exceedance/{voyage_id}/{id}`, `alarm` (deviation) | `voyage/*`, `route/**` (both key shapes), ownship, environment subjects |
| `traffic` | MON | fused target table from `@target` trees; staleness expiry; CPA/TCPA/BCR/BCT; bearings | `target_cpa_m`, `target_tcpa_s`, `target_bcr_m`, `target_bct_s`, `target_bearing_north_deg`, `target_bearing_relative_deg` under `@target/{id}` | `**/pubsub/{location_fix,course_over_ground_deg,speed_over_ground_knots,heading_true_north_deg,nav_status,vessel_type,length_over_all_m}/*/@target/**` |
| `colreg` | DEC (advice) | encounter classification (Rules 13/14/15, 17 stand-on, 19 restricted vis), give-way/stand-on, risk level, manoeuvre synthesis (course/speed alteration, hold), advice validity | `encounter`, `navigation_advice`, `advice_disposition` (new) | traffic table, ownship, route corridor, visibility |
| `guidance` | EXEC/CTRL | track-keeping (wheel-over, fly-by turns per `Waypoint.radius_m`), speed policy (planned SOG, leg envelope, UKC/visibility slow-down, ETA hold), execution of *accepted* advice, return-to-track | backend A: `joystick_x_pct`/`joystick_y_pct` @ 10–20 Hz + `set_control_mapping`; backend B: `vehicle_navigation/v1` RPCs. Always `navigation_setpoint` (new, record of what was ordered) | route_monitor state, accepted advice, `command_authority`, `operational_authority` |
| `conduct` | state machine | navigation mode FSM, authority gating, fallbacks, handover requests, `task_allocation` (NAV-01…NAV-08) | `navigation_state` (new), `task_allocation` (new), `command_request`, `command_authority` heartbeat when holding, `alarm` | `operational_authority`, `command_authority`, `vehicle_mode`, `voyage`, all module health |
| `decision_log` | audit | append-only record of every decision with inputs snapshot, rule, outcome; periodic snapshot for late joiners (WarrantRecord pattern) | `navigation_decision/{voyage_id}/{decision_id}` (new) | every module's decision events |

**Advisory-by-construction rule:** `colreg` never commands. `guidance` acts only on advice whose `advice_disposition == ACCEPTED`, where acceptance is either (a) an operator via Crowsnest, or (b) `conduct` when `task_allocation[NAV-05].DEC` names this processor **and** `operational_authority.level >= ASSISTED_AUTONOMOUS` **and** the processor holds the conn. Every auto-acceptance is a `navigation_decision`.

---

## 2. Navigation state machine (`conduct`)

```
                 ┌──────────┐  voyage ACTIVE & route resolvable
   start ──────► │  IDLE    │ ───────────────────────────────────┐
                 └──────────┘                                    ▼
      ▲   lease lost / authority < REMOTE_CONTROLLED       ┌──────────┐
      │   / voyage PAUSED                                  │  READY   │ (monitoring only; no conn)
      │                                                    └────┬─────┘
      │        conn granted (command_authority.controller_id==me)│ & level >= REMOTE_CONTROLLED
      │                                                         ▼
┌─────┴──────┐                                          ┌──────────────┐
│  FALLBACK  │ ◄─── link/authority/sensor loss ──────── │  UNDERWAY    │
│ (HOLD_POS /│                                          │  ├ TRACK     │ track-keeping
│  MIN_SAFE) │                                          │  ├ AVOIDING  │ executing accepted advice
└────────────┘                                          │  ├ HOLDING   │ speed 0 / loiter (envelope, action point, wait for accept)
      │  operator resumes                               │  └ REROUTING │ awaiting route_planner reply / edition bump
      └──────────────────────────────────────────────►  └──────┬───────┘
                                                                │ final waypoint reached / voyage COMPLETED|ABORTED
                                                                ▼
                                                        ┌──────────────┐
                                                        │ ARRIVED/ENDED│ → release lease → IDLE
                                                        └──────────────┘
```

Guards, all evaluated on the receiver's own clock (§6.5.1):
- **Conn**: `command_authority` accepted messages arm `local_receipt + lease_ttl_seconds`; lapse ⇒ FALLBACK + alarm. Holding: heartbeat every `heartbeat_interval_seconds`, release with same token on exit.
- **Ceiling**: act on `OperationalAuthority.level` only (never `composite_score`). `UNKNOWN` is non-authorising. Mapping: `< REMOTE_CONTROLLED` ⇒ never drive; `REMOTE_CONTROLLED` ⇒ track-keeping only, advice operator-accepted; `ASSISTED_AUTONOMOUS` ⇒ auto-accept advice below a configured severity; `FULL_AUTONOMOUS` ⇒ auto-accept all, reroute within `DynamicReroutingPolicy`.
- **Sensor freshness**: ownship fix/heading/SOG older than `max_state_age_s` ⇒ FALLBACK. Absent subjects (Slipway L0 hulls publish no rudder, no operational `location_fix` without `keelson-sil-gnss`) degrade gracefully: only *required* inputs gate.
- **Failsafe output**: in FALLBACK backend A publishes neutral axes for `neutral_burst_s` then goes silent (dead-man stops the hull); backend B issues `set_cruise_speed(0)` and, if configured, `set_mode("HOLD")`.
- **Route edition bump mid-voyage**: re-pin only when `Voyage.route_ref` changes; a dangling `RouteRef` is surfaced as alarm, never resolved to `latest` (§6.2).

---

## 3. New bus vocabulary (keelson feature branch, developed on `raw_json` riders)

All pass §2.2.1: none is derivable from an existing subject by a fixed mapping. Reuse `ColregSituation`, `SeverityLevel`, `RouteRef`, `Coordinate`, `EnvelopeBound`. `steering`/`propulsion` orders reuse PR #255's `SteeringOrder`/`PropulsionOrder` oneof-as-frame pattern if merged, else mirror it.

| Subject | Payload | Key tail | Persist | Notes |
|---|---|---|---|---|
| `encounter` | `keelson.Encounter` {timestamp, encounter_id, voyage_id, target_ref{entity,source,target_id}, cpa_m, tcpa_s, range_m, bearing_true_deg, bearing_relative_deg, situation (ColregSituation + OVERTAKEN, NONE), own_role {GIVE_WAY, STAND_ON, NONE}, risk {NONE, WATCH, RISK, DANGER}, rule_refs[], opened_at, closed_at?} | `/{voyage_id}/{encounter_id}` | yes | full restatement per record (EnvelopeExceedance rule) |
| `navigation_advice` | `keelson.NavigationAdvice` {advice_id, encounter_ids[], kind {COURSE_ALTERATION, SPEED_CHANGE, HOLD, RESUME_TRACK, REROUTE}, proposed_course_deg?, proposed_sog_knots?, rationale, valid_from, valid_to, expected_cpa_m, expected_min_range_m} | `/{voyage_id}/{advice_id}` | yes | record, not command |
| `advice_disposition` | `keelson.AdviceDisposition` {advice_id, state {ACCEPTED, REJECTED, SUPERSEDED, EXPIRED}, by_id, by_site, at, reason} | `/{voyage_id}/{advice_id}` | yes | Crowsnest or `conduct` publishes |
| `navigation_setpoint` | `keelson.NavigationSetpoint` {timestamp, voyage_id, steering oneof {heading_deg, course_deg, rudder_pct}, propulsion oneof {sog_knots, stw_knots, throttle_pct}, source_advice_id?, backend} | `/{source}` | no | 1–10 Hz; what guidance ordered, for evidence |
| `navigation_state` | `keelson.NavigationState` {timestamp, mode enum (FSM above), sub_mode, voyage_id, route_ref, current_leg_from_waypoint_id, authority_level_in_force, conn_held, reason, degraded_inputs[]} | `/{source}` | no | 1 Hz |
| `navigation_decision` | `keelson.NavigationDecision` {decision_id, timestamp, voyage_id, kind, inputs (ownship snapshot, encounter_ids, limit_ids), rule_refs[], action (setpoint / advice / mode transition), outcome, policy_id, policy_config_digest} | `/{voyage_id}/{decision_id}` | yes | append-only + 30 s snapshot |
| `task_allocation` | `keelson.TaskAllocation` {timestamp, vessel_id, function_id (NAV-01…NAV-08), roles {EXEC, MON, DEC, CTRL, OVR} → agent{id, kind HUMAN/SOFTWARE, site}, valid_until, grant_ref (command_authority token / voyage_id)} | `/{function_id}` | yes | contract must be settled in writing first (design note §5.3) |

Development riders: publish the same field set as JSON on `raw_json` under the module source id `{source_base}/{module}`, named by a `record` field (Slipway's convention) and register them in the rider manifest format Slipway uses (`SLIPWAY/contracts/raw-json-riders.yaml`) with retirement condition "merged in keelson ≥ X". Codec layer `nav/bus/codec.py` has `encode(subject, obj) -> bytes` with two implementations; switching is one config flag.

Also claim (no keelson change needed): `target_cpa_m`, `target_tcpa_s`, `target_bcr_m`, `target_bct_s`, `target_bearing_north_deg`, `target_bearing_relative_deg`, `route_execution`, `envelope_exceedance`, `alarm`, `command_request`, `command_authority`.

---

## 4. Repository layout (mirrors `keelson-processor-route-planner`)

```
keelson-processor-navigation/
  README.md  CLAUDE.md  TODO.md  CONTRIBUTING.md  LICENSE
  pyproject.toml  uv.lock            # py>=3.13, deps: keelson (monorepo SDK), eclipse-zenoh>=1.2.1,
                                     #   nvector or pyproj, shapely, numpy, pyyaml, environs, pydantic
  Dockerfile  docker-compose.yml  docker-compose.dev.yml   # copy route-planner's uv layering
  bin/navigation                     # entrypoint shim
  config/
    processor.example.yaml           # modules enabled, realm/entity/source ids, backend, thresholds
    platforms/slipway-alpha.yaml     # backend A, axis mapping, dead-man, hull limits
    platforms/ghost-drone.yaml       # backend B
    colreg.default.yaml              # CPA/TCPA thresholds by situation, safe-passing distance, hysteresis
  src/nav/
    app.py                           # CLI (keelson.scaffolding.cli.add_common_arguments), wiring, GracefulShutdown
    bus/                             # subscriptions, publishers (declare_publisher_for_subject), keys, codec (proto|rider), lease.py
    world/                           # OwnshipState, TargetTable (@target ingest, staleness), RouteModel (both key shapes), Environment
    route_monitor/                   # progress.py (port of computeRouteExecution), envelope.py (EnvelopeExceedance lifecycle)
    traffic/                         # cpa.py (port of cpaCalculation.js), bearings, bcr/bct
    colreg/                          # geometry.py (aspect/relative bearing), classify.py (Rules 13-15,19), risk.py, synthesize.py, advice.py
    guidance/                        # track.py (XTE+heading, wheel-over, fly-by), speed.py, backends/{axes.py, goto.py, base.py}, failsafe.py
    conduct/                         # fsm.py, authority.py (lease + ceiling gates), allocation.py
    log/                             # decision_log.py (append + snapshot), evidence.py
    sil/                             # tick_barrier.py: Slipway BLOCKING participant (scenario_status fence → compute → scenario_tick_ack)
  tests/
    unit/  (pure: cpa, colreg classify, fsm guards, xte, envelope lifecycle, lease clock rule, codec round-trip)
    integration/ (zenoh in-proc: two-publisher route key shapes, lease lapse, dead-man silence)
    sil/  (Slipway scenarios: rivo-crossing, two-vessel-harbor, ferry-set-and-drift; marked @pytest.mark.e2e)
  docs/
    ARCHITECTURE.md  STATE-MACHINE.md  COLREG-POLICY.md  UPSTREAM-ASKS.md (proto drafts, paste-ready)
```

Key SDK helpers to use (all in `keelson/sdks/python/keelson`): `construct_pubsub_key`, `construct_rpc_key`, `parse_pubsub_key`, `enclose`/`uncover`, `get_subject_schema`, `decode_protobuf_payload_from_type_name`; `scaffolding.cli.add_common_arguments`/`create_zenoh_config`, `scaffolding.liveliness.declare_liveliness` (source + per-subject tokens, one per module `source_id`), `scaffolding.qos_zenoh.declare_publisher_for_subject`, `scaffolding.rpc.serve_rpc`/`reply_err` (only if the processor ever serves an RPC; v1 serves none), `scaffolding.signals.GracefulShutdown`, `scaffolding.logging.setup_logging`, `scaffolding.configurable.make_configurable` (expose thresholds via `configurable/v1` like the warrant aggregator).

Threading model: one zenoh session; subscriber callbacks only stamp into the `world` model (latched, newest wins, like Slipway's mailbox); a single deterministic `tick()` at 10 Hz runs monitor → traffic → colreg → conduct → guidance → log; backend A has its own 10–20 Hz output timer that repeats the last setpoint (continuous publication is mandatory). In SIL mode the tick is driven by the Slipway fence instead of a timer.

---

## 5. Phased delivery

**Phase 0 — repo bootstrap (day 1).** Copy route-planner scaffolding; write `TODO.md` = this plan's phase checklist; `CLAUDE.md` with the invariants (advice never commands; receiver-clock leases; publish continuously; both `route` key shapes; never act on `composite_score`). Skeleton `app.py` that connects, declares liveliness, publishes `navigation_state{IDLE}`.

**Phase 1 — route monitor (design note §5 step 1).** Port `computeRouteExecution`; subscribe `voyage/*` and `route/**` (discriminate 6-chunk declaration vs `{id}/latest` vs `{id}/edition/{N}`); publish `route_execution/{voyage_id}` at 1 Hz with envelope standings; `envelope_exceedance` open/restate/close with `responses[]`. **Acceptance:** Crowsnest Route tab provenance chip flips *Computed here → From vessel* against Slipway with zero Crowsnest changes.

**Phase 2 — traffic + CPA.** `TargetTable` from `@target` trees (two subscriptions per subject because `@target` is verbatim), staleness expiry (AIS class-dependent), port `cpaCalculation.js`; publish the six unclaimed `target_*` subjects. Verify numbers match Crowsnest's bearing-rate table.

**Phase 3 — COLREG advisor (advisory only).** Encounter classification, risk with hysteresis (1.2 clear margin), advice synthesis (starboard-preference, speed reduction when stand-on cannot hold, Rule 19 in `visibility_range_m` below limit), disposition handling. Publish on riders. Draft protos + `subjects.yaml`/`qos.yaml` in a keelson feature branch, `docs/UPSTREAM-ASKS.md` paste-ready. Run against `rivo-crossing.yaml`.

**Phase 4 — guidance backend A + conduct FSM.** *Decision 2026-09-14: guidance replaces `keelson-processor-autopilot` on vessels it drives, because v0.2.0 cannot be fed (static GeoJSON track, no setpoint/Route/speed/lease input, 5 Hz, re-takes an empty mapping) and Slipway allows one control mapping per vessel. The autopilot remains for simple track demos.* Lease client (take/heartbeat/release, receiver-clock lapse), `set_control_mapping` on Slipway kernel, XTE+heading controller (start from autopilot repo's dead-reckoned PID, gains per platform), speed policy, failsafe neutral burst → silence. FSM with authority gates. `navigation_state`, `navigation_setpoint`, decision log. **Acceptance:** Slipway vessel in COMMANDED mode sails `two-vessel-harbor.yaml` on route with XTD inside leg limits; pulling the lease stops her within dead-man.

**Phase 5 — SIL barrier participant.** `sil/tick_barrier.py`: subscribe `scenario_status`/fence, compute, `scenario_tick_ack{COMPLETED}`; deterministic tests under `step()`; recordings via `tools/tick_collector.py`. Reproducibility class R2-conditional.

**Phase 6 — backend B + reroute.** `vehicle_navigation/v1` goto backend against ghost-drone (`NOT_OBSERVABLE` handling, poll `navigation_target`); `route_planner/v1` client (`consolidation=NONE`) for REROUTING within `DynamicReroutingPolicy`, then §6.4 edition bump via `select_route` and voyage audit `REROUTE_ACCEPTED`.

**Phase 7 — conduct/task allocation + proto swap.** `task_allocation` contract written first (design note §5.3), Crowsnest Conduct panel reads it; switch codec from riders to protos when the keelson PR is released (alpha pin → pre → stable).

---

## 6. Verification

- **Unit** (`uv run pytest tests/unit`): CPA against Crowsnest fixtures; COLREG classification truth table (head-on ±6°, crossing sectors, overtaking >22.5° abaft beam); FSM guard matrix (every state × every input); lease clock rule (skewed `expires_at` must not affect lapse); envelope restatement; codec round-trip proto⇄rider.
- **Integration** (in-proc zenoh, `@pytest.mark.e2e`): two `route` key shapes resolve; backend A goes silent on FALLBACK; heartbeat cadence; `route_execution` at 1 Hz ± jitter.
- **SIL against Slipway** (`docker compose -f docker-compose.dev.yml up` with Slipway `core` profile on the shared router at `tcp/localhost:7447`, isolated entity `slipway-alpha`): rivo-crossing produces one `encounter` (CROSSING_STBD, GIVE_WAY) and one accepted advice with recorded near-miss absent in Slipway's `detect.py` flags; ferry-set-and-drift holds XTD; lease lapse test.
- **Crowsnest acceptance**: Route tab chip *From vessel*; bearing-rate CPA matches; Conduct panel shows the processor's rows.
- **Evidence**: every run recorded to MCAP via keelson mcap connector; decision log reconstructable from recording alone.
- **Lint**: `ruff check`, `black --check` per keelson style.

## 7. Open items to settle before Phase 3/7 code
- Safe-passing distances and CPA/TCPA thresholds per `OperationalContext` (open sea vs archipelago) — config, defaults from Crowsnest `cpaAlerts.js`.
- Whether `conduct` may auto-accept advice at `ASSISTED_AUTONOMOUS` or only at `FULL_AUTONOMOUS` (default: assisted ⇒ auto-accept below DANGER, otherwise operator).
- `task_allocation` precedence questions the ROC framework §21 leaves open.
- keelson issue: `ControlAxisMapping` carries no authority token (Slipway observes, cannot enforce).
