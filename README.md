# keelson-processor-navigation

A [keelson](https://github.com/RISE-Maritime/keelson) processor that owns navigation of an
autonomous vessel (MASS, USV) while it is under automation.

> **Status: Phase 4, guidance and conduct.** It monitors the vessel's route, tracks traffic with
> CPA, and classifies encounters under COLREG Rules 13 to 19. With `guidance` enabled it also
> **commands the vessel** over `vehicle_control/v1`, but only on a live conn -- an operator's
> whose order it carries out, or its own under full automation -- while the operational authority
> level allows driving, own-ship inputs are fresh and the control mapping is free or its own. Accepted on Slipway's kernel in-process; not yet run on a shared bus. Advice
> stays a record: guidance acts on it only once accepted. See [TODO.md](TODO.md) and
> [docs/PLAN.md](docs/PLAN.md).

## What it does

| Module | Role | Output |
|---|---|---|
| `route_monitor` | Tracks the active voyage against its pinned route edition | `route_execution`, `envelope_exceedance` |
| `traffic` | Target table from `@target` trees, CPA and TCPA | `target_cpa_m`, `target_tcpa_s`, … |
| `colreg` | Classifies encounters and proposes manoeuvres. Advice only, never a command | `encounter`, `navigation_advice` |
| `guidance` | Steers along the track and executes accepted advice | `vehicle_control/v1` axes, `navigation_setpoint` |
| `conduct` | Navigation state machine gated on the conn and operational authority | `navigation_state`, `command_authority`, `command_request` |
| `decision_log` | Append-only record of every navigation decision | `navigation_decision` |

Each module publishes under its own source id, `{source_base}/{module}`.

## What it does not do

It does not author or store routes (Crowsnest, router storage), plan routes (it calls
`keelson-processor-route-planner`), detect or track targets (AIS and radar connectors), shape
joystick input (`keelson-processor-supervisor`), or arm the vehicle.

## Run

```bash
cp config/processor.example.yaml config/processor.yaml   # set entity_id to your vessel
navigation --config config/processor.yaml --connect tcp/localhost:7447
```

There is no default `entity_id`, and each input names its `source_id` explicitly. The
processor refuses to start without them, and refuses simulator `truth` sources.

**Under a Slipway instance** (Slipway ADR-0139), keep the authored names in the file and pass
`--slipway-instance <i>` (or `slipway_instance:`): `sim-usv-alpha` is then navigated as
`sim-<i>-usv-alpha` and a `sim-ais-traffic` feed is read from `sim-<i>-ais-traffic`.
Other entities pass through, and an empty instance is the plain names (`src/nav/instance.py`).

## What route monitoring reads and writes

| Direction | Key under `{realm}/@v0/{entity_id}/pubsub/` | Notes |
|---|---|---|
| in | `voyage/*` | Bootstrapped from router storage at start |
| in | `route/{id}/edition/{N}` | The edition an ACTIVE voyage pins. `latest` is never used |
| in | `route/{source_id}` | A vessel's declared route. Monitored only with no voyage in play, and only while its heartbeat is live |
| in | `location_fix/{source}`, `speed_over_ground_knots/{source}`, … | Sources from `inputs` |
| out | `route_execution/{voyage_id}` | 1 Hz. For a declared route the token is `declared-{source}` and `voyage_id` is empty |
| out | `envelope_exceedance/{voyage_id}/{exceedance_id}` | Voyages only. Opened on breach, restated in full on close |
| out | `alarm/navigation/0/route_monitor` | Deviation (WARNING), approaching the XTD limit (CAUTION: under `approach_time_s` at the present rate), envelope, no position, dangling route. Cleared by publishing an expired alarm |

**Which limit.** On a leg with a VARIABLE corridor (keelson#275 `xtd_corridor_mode`), the deviation
and the approach are both judged against the corridor **where she is**: each side's
`xtd_*_boundary` is crossed with her abeam line, and the crossing nearest the track is used. A side
the boundary does not reach falls back to the leg's `xtd_*_m`, and that to `xtd_limit_fallback_m`.
This is the same rule as Crowsnest's `limitsAtShip`, pinned on the same literal corridor
(`tests/test_corridor.py`, crowsnest-dev#287). A FIXED leg uses its own limits and never reads a
boundary left over from a switch back.

## What traffic reads and writes

| Direction | Key | Notes |
|---|---|---|
| in | `{feed entity}/pubsub/{subject}/{feed source}/@target/*` | Position, COG, SOG, heading, nav status, name, dimensions. Newest receipt wins across feeds |
| out | `{entity_id}/pubsub/target_cpa_m/navigation/0/traffic/@target/{target_id}` | Also `target_tcpa_s`, `target_bcr_m`, `target_bct_s`, `target_bearing_north_deg`, `target_bearing_relative_deg`. 1 Hz, targets in range only, and only values that are defined |

CPA uses the same local frame as Crowsnest's `computeCPA`, so values compare directly. A course or
speed that was never reported gives no CPA rather than an assumed one. Target positions are
dead-reckoned from receipt to now; set `traffic.dead_reckon: false` to match Crowsnest exactly.

Watch it:

```bash
z_sub -e tcp/localhost:7447 -k 'rise/@v0/sim-usv-alpha/pubsub/raw_json/navigation/**'
```

## What the COLREG advisor publishes

| Subject (typed, keelson 0.6.0-pre.18; keys per `protocols/navigation_decisions.yaml`) | Content |
|---|---|
| `encounter` | Target, range, bearing, CPA, TCPA, situation, own role, rules, risk grade, opened and closed times. Restated each second while open |
| `navigation_advice` | One advice for the vessel: course and/or speed, the encounters it answers, validity window, expected CPA, rules, rationale |
| `advice_disposition` | Accepted or rejected (from an operator or a conduct service), superseded or expired (from the advisor) |

Keys, under `{realm}/@v0/{entity_id}/pubsub/`: `encounter/navigation/0/colreg/{voyage_id or novoyage}/{encounter_id}`
and `navigation_advice/navigation/0/colreg/{voyage_id or novoyage}/{advice_id}` are instance keys;
`advice_disposition/{voyage_id or novoyage}/{advice_id}` is a slot written by the advisor and the
decision holder alike, and it is where an operator's accept or reject arrives. `navigation_state` is
`navigation_state/navigation/0/conduct`.
| `alarm` | Collision risk at DANGER, no manoeuvre clears, own velocity unknown with encounters open |

How it decides:
- Situation and own role are fixed when an encounter opens and held until it closes.
- Give-way: the smallest starboard alteration of at least 30° that clears the encounter vessels to
  the RISK distance and keeps every other target outside DANGER; then speed reduction; then stop.
  If nothing clears, the best found is advised and an emergency alarm is raised.
- Stand-on: keep course and speed, and act under 17(a)(ii) only when the other vessel is not acting
  and the approach is close, never to port for a vessel on the port side.
- Thresholds follow the current leg's operational context, coastal when unknown.
- Not considered, and said in every rationale: the chart, traffic separation schemes, narrow
  channels and the route corridor.

Paste-ready keelson protos for all draft subjects are in [docs/UPSTREAM-ASKS.md](docs/UPSTREAM-ASKS.md).

## Draft subjects

Subjects keelson does not type yet ride `raw_json` as
`{"record": "<subject>", "rider_version": 1, "payload": {...}}`, with the payload in the
protobuf JSON form of a draft proto in [docs/upstream/](docs/upstream/). The register is
[src/nav/contracts/raw_json_riders.yaml](src/nav/contracts/raw_json_riders.yaml). Once keelson
registers a subject, `codec: proto` publishes it typed with no other change.

## What conduct and guidance read and write

| Direction | Key under `{realm}/@v0/{entity_id}/` | Notes |
|---|---|---|
| in | `pubsub/command_authority/**` | The conn. Lapses `lease_ttl_seconds` after the last message *this* receiver heard; `expires_at` is never compared with a local clock |
| in | `pubsub/operational_authority/{conduct.authority_source}` | Only `level` is read, never `composite_score`. None heard, or stale, is `UNKNOWN`, which never drives |
| rpc | `@rpc/vehicle_control/v1/get_control_mapping/{guidance.responder_id}` | Polled every `mapping_poll_s` |
| rpc | `@rpc/vehicle_control/v1/set_control_mapping/{guidance.responder_id}` | Installs `steering` from `joystick_x_pct` and `throttle` from `joystick_y_pct`, only when the vehicle reports an empty mapping |
| out | `pubsub/command_authority/{entity_id}`, `pubsub/command_request/{entity_id}` | Take, heartbeat, release; request. Under the vessel id as source, the convention Crowsnest and Slipway use |
| out | `pubsub/joystick_x_pct/navigation/0/guidance`, `pubsub/joystick_y_pct/navigation/0/guidance` | Both axes together at `publish_hz` (10-20 Hz) from their own timer. Positive steering is starboard |
| out | `pubsub/raw_json/navigation/0/guidance`, record `navigation_setpoint` | Course, speed, leg, cross-track error and the axis values behind them |
| out | `pubsub/raw_json/navigation/0/decision_log`, record `navigation_decision` | Mode changes, conn taken, granted, lost or released, mapping installed or refused, advice accepted. Recent decisions restated every 30 s |

The rpc rows and the joystick row are the default backend, `guidance.backend: axes`, for a vessel
with no motion gate.

**Behind a motion gate,** with `guidance.backend: gate` (crowsnest-dev#94), the backend never
installs a mapping:

| Direction | Key under `{realm}/@v0/{entity_id}/` | Notes |
|---|---|---|
| rpc | `@rpc/control_gate/v1/get_gate_state/{guidance.gate.responder_id}` | The gate's map in force, read every `mapping_poll_s` and at once on a new motion epoch. Only ever read |
| in | `pubsub/resolved_control/{entity_id}` | The gate's motion epoch. A new one silences the candidates until a map read under it is in |
| out | `pubsub/guidance_candidate/navigation/0/guidance/{dof}` | One `GuidanceCandidate` per DOF in `guidance.gate.dofs`, at `publish_hz`. `source` is `navigation/0`, and one stream (a random id per start) rises across every DOF |

Each candidate claims two bases, and the gate checks them independently (I15):
- **Decision.** It claims NAV_FUNCTION {function, revision} when the function that decides that
  output is delegated to this process at method 3 or above:
  - steering is `collision_avoidance` while avoiding and `track_keeping` on the track;
  - under a course, heading, hold or position order, steering has no function;
  - throttle is `speed_control`.

  Otherwise it claims HUMAN_ORDER {order_id}, under a human's conn only, with the id that
  `navigation_state.order_id` publishes (a new id per order taken, or `advice/{id}` for an advice a
  human accepted). Otherwise it claims nothing for that DOF.
- **Execution.** It claims DIRECT when the gate's row names this process as the holder, and
  CARRY_OUT {mapping_id} under a human's conn.

A row whose holder is not the conn holder heard now proposes nothing. That is the polite yield on
a take-control; the gate's fence is what stops it.

**On the pinned build** (`0.6.0.dev21`), `NavigationState` has neither `order_id` (keelson's
`feat/navigation-state-order-id`) nor `functions_in_force` (#376). Both are published only when the
installed SDK has them. Without `order_id` the gate refuses every HUMAN_ORDER, so only a delegated
function drives through it until the pin moves.

What the state machine guarantees:

- **UNDERWAY** needs a live conn -- the operator's, who has ordered it, or its own -- a level of
  `REMOTE_CONTROLLED` or above, fresh position, heading, course and speed, and a control mapping
  that is free or already this processor's. Another controller's mapping is reported, never
  overwritten. A route must be selected either way.
- **Losing any of them** is FALLBACK: neutral axes for `neutral_burst_s`, then silence, so the
  vehicle's dead-man stops the hull. Arrival does the same, then releases the conn (under an
  operator's conn there is none to release, and it ends at once).
- **Pulling the lease stops the vessel.** After a FALLBACK the processor does not take the conn
  back, even with `take_conn_when_free`; only a grant naming it, or a new order from the live conn
  holder, resumes.
- **A stalled tick loop goes silent.** Axes are repeated only until `command_hold_s` after the
  last tick.

## What navigation_control/v1 does

With `modules.control: true` the processor serves keelson's `navigation_control/v1`
(keelson#282, pinned through keelson's experimental build `0.6.0.dev11`) under responder id `{source_base}`:
`{realm}/@v0/{entity_id}/@rpc/navigation_control/v1/{procedure}/navigation/0`. It is then the only
writer of its vessel's `voyage` records. Each call is queued and decided in the tick
(`src/nav/control/`), so it is deterministic and runs the same under Slipway's tick fence.

| Procedure | Does |
|---|---|
| `load_route` | Makes a route edition the ACTIVE voyage (minted, or the `voyage_id` given). A route from another vessel (`source_entity_id`) is read from router storage and republished unchanged under this vessel, edition and `latest`, first. Any other ACTIVE voyage is ABORTED as superseded. A missing edition is refused, never replaced by `latest` |
| `set_guidance_order` | `track` (optionally rejoining at a waypoint id), `course_over_ground_deg`, `heading_deg`, `hold`, `go_to` (keelson#308: a position and an optional `arrival_radius_m`, default `guidance.goto_arrival_radius_m`) or `loiter` (keelson#320: a position, `radius_m` and an optional `return_speed_knots`, default `guidance.loiter_return_knots`), with an optional `speed_knots`. `dynamic_positioning` (keelson#320) is refused UNSUPPORTED: guidance drives steering and throttle only, with no lateral authority to hold a position and a heading. A `track` may carry (keelson#324) a `cross_track_offset_m` (+ starboard) or a `cross_track_offset_fraction` of the current leg's corridor on that side (capped at 0.9 of it; the monitor's `xtd_limit_fallback_m` where a leg names none), and `reverse`, which sails the same edition backwards -- guidance and the route monitor both follow the reversed legs, with port and starboard limits swapped. `speed_plan_factor` trims the legs' planned speed (at most one of it and `speed_knots`). Course and heading keep their frames: a course is corrected for drift, a heading is not. Without guidance, only `track` is accepted |
| `set_voyage_status` | Pause, resume, complete or abort. Every change returns the order to `track` |
| `get_navigation_state` | The `keelson.NavigationState` it would broadcast now. Needs no authority |
| `get_settings` | keelson#322: the `NavigationSettings` in force -- COLREG advisor active, advice auto-accept, default speed, go-to arrival radius, loiter return speed, traffic range, route alarms -- and `fixed`, the ones it cannot change while it runs (the advisor and auto-accept exist only when started with them). Needs no authority |
| `set_settings` | Tune it while it runs. The same who-may-call rule as the other directing calls; applied whole or refused whole (UNSUPPORTED for a fixed or absent setting, an error for a value out of range). Pausing the advisor stops new encounters and advice; accepted advice is steered to its end. Logged as `KIND_GUIDANCE_ORDER` "settings: …" until keelson has a settings kind |

Who may call: the holder of the live `command_authority` lease, by presenting its `token`; or a
controller listed in `control.controllers` (a scenario runner), while this processor holds the conn
itself **or nobody does**. Once anyone else holds the conn only that holder may, so an operator
taking the conn outranks the list. Everyone else is `DENIED`. Holding no route, the processor hands
a granted conn straight back, so an operator loads the route while it holds the conn and grants it
after; a permitted scenario runner loads it while the conn is free, and `take_conn_when_free` does
the rest.

**The operator keeps the conn; the process carries out the order** (keelson#282
`navigation_conduct`, Ted 2026-09-29). The human operator holding the conn carries the captain's
responsibility and keeps it. Once the live conn holder has ordered this process -- `load_route` or
`set_guidance_order` presenting that holder's token -- it sails the order under the holder's conn:
`MODE_UNDERWAY` with `conn_held=false`, reason "…; carrying out the order of ted@roc1, …". It never
takes the lease for this. A voyage it merely finds selected, with the default `track`, is not such
an order, and neither is `set_voyage_status`: it sits READY with "conn not held, and the conn holder
has given no order". The conn passing to another operator keeps the order in force under the new
holder (logged). It stands down to FALLBACK when the holder's lease lapses ("the conn holder's lease
lapsed"), when a station takes the helm mapping ("a station took the helm …"), or when the voyage it
was given for is no longer selected; arrival ends it. Standing down from an operator's order also
releases its axis mapping once it has gone silent, so the operator's helm is free. Only a **new** order from the live holder
re-arms it. Holding the conn itself is full automation: only then does it take navigation decisions
of its own (`auto_accept_advice`); under an operator's conn advice stays advice until the holder
accepts it.

**Navigation authority per function** (crowsnest-dev#90). Accepting its own COLREG advice also needs
`collision_avoidance` delegated to it, in the navigation authority document the
NAVIGATIONAL_COMMAND holder writes. The document is the `command_authority` record on
`{vessel}/navigation`, and the process is its `conduct.controller_id`. Both the operational-authority
ceiling and the function's control method must allow the accept:
- **no document, held by another, or method 1-2:** never; a human confirms each decision;
- **method 3:** only after the advice's veto window (`valid_from` + `intervention_window_ms`) has
  passed with no REJECTED disposition. Even then, only if a veto could have arrived: the
  navigational commander's session (`{vessel}/navigational/session`) is live, or, without
  `intent_delivery_required`, collision_avoidance continues on comms loss;
- **method 4-5:** at once.

Why it did not accept is logged as `KIND_ADVICE_HELD`. **Behaviour change:** `auto_accept_advice`
on its own no longer accepts anything. Records on other `command_authority` slots (the document,
sessions, payload leases) never reach the conn's lease book.

**Getting the conn back.** A live lease cannot be seized, and this process heartbeats its lease
for as long as it has a route, so it answers `command_request` (`conduct.yield_on_request`, on by
default): an operator's request is granted with keelson's directed `grant` -- a fresh token naming
the requester and its site -- and she stops as on a pulled lease (neutral burst, then silence), with
the axis mapping released so the operator's helm is free. It resumes only on a new grant. A request
from another navigation process (`navigation/…@…`) is never granted.

The order in force is reported as `navigation_state.guidance_order`, and the UNDERWAY sub-mode
follows it: `SUB_MODE_STEERING` for a course or heading, `SUB_MODE_HOLDING` for hold. A `go_to`
steers for the position (`SUB_MODE_STEERING`), at the speed she has or `default_sog_knots` from a
standstill; inside the radius it holds with neutral axes (`SUB_MODE_HOLDING`, the order still in
force), and makes for the position again once she drifts out past twice the radius. While it steers,
guidance also publishes keelson#316's `heading_setpoint_deg` and, unless the frame is a heading,
`course_over_ground_setpoint_deg` (TimestampedFloat, `navigation/0/guidance`); neither is published
while it holds or goes neutral. A `loiter` makes for its position like a go_to, lies neutral once
near it (`SUB_MODE_HOLDING`), and past `radius_m` makes back at the return speed (`SUB_MODE_STEERING`)
until near it again. An order is
given for a voyage: once that voyage is no longer the one selected, guidance returns to the track.
Every call that changes something is in the decision log (`KIND_VOYAGE_CHANGED`,
`KIND_GUIDANCE_ORDER`), with the voyage's own `audit_trail` naming the caller.

## What subscriber/v1 says

Every running processor serves keelson's `subscriber/v1`, whatever modules are on, at
`{realm}/@v0/{entity_id}/@rpc/subscriber/v1/{procedure}/navigation/0`.

- **`get_subscriptions`** answers `{name: key}` for every key the processor actually declared.
  The name is the subject, or `subject@entity` for a traffic feed, where one subject is read
  from several entities.
- **`set_subscriptions`** is refused (`UNSUPPORTED`). The inputs are fixed by configuration at
  start, and a success that changed nothing would be worse than saying so.

Slipway asks this at the start of every run and records the answer in its manifest. Reading the
simulator's truth is unrestricted on its bus, so a run proves more or less depending on whether
this processor read `…/truth` or only what a real vessel publishes. The manifest is how whoever
cites the run can tell.

## Deployment

A vessel has one control mapping and the last writer wins, so with `guidance` enabled this processor
must not share a vessel with `keelson-processor-autopilot`, which re-installs its mapping whenever
the helm is free. Stop the autopilot container on any vessel this processor steers. Monitoring,
traffic, advice and conduct without guidance are safe beside it: they command nothing.

[config/sim-usv-alpha-guidance.yaml](config/sim-usv-alpha-guidance.yaml) sails Slipway's
`sim-usv-alpha` on scenario `alpha-navigation`. It fixes the authority level because Slipway runs no
authority aggregator, which is acceptable in simulation only.

## Acceptance against Slipway

`tests/test_acceptance_slipway_guidance.py` runs Slipway's kernel in-process on `two-vessel-harbor`
with alpha in COMMANDED mode: guidance sails her circuit inside the leg's cross-track limit, pulling
the lease stops her within the neutral burst and dead-man, and a stalled processor is caught by the
kernel's dead-man. It needs Slipway's environment, so it is skipped in this repo's own:

```bash
cd ../SLIPWAY
NAV=../keelson-processor-navigation
PYTHONPATH=$NAV/src:$NAV .venv/bin/python -m pytest -p no:cacheprovider \
    $NAV/tests/test_acceptance_slipway_guidance.py
```

## Container

```bash
docker compose up --build
```

keelson is the exact version pinned in `pyproject.toml`, installed from PyPI; nothing is built from a
keelson checkout.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md).
