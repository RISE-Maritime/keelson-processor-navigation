# keelson-processor-navigation

A [keelson](https://github.com/RISE-Maritime/keelson) processor that owns navigation of an
autonomous vessel (MASS, USV) while it is under automation.

> **Status: Phase 4, guidance and conduct.** It monitors the vessel's route, tracks traffic with
> CPA, and classifies encounters under COLREG Rules 13 to 19. With `guidance` enabled it also
> **commands the vessel** over `vehicle_control/v1`, but only while it holds the conn, the
> operational authority level allows driving, own-ship inputs are fresh and the control mapping is
> free or its own. Accepted on Slipway's kernel in-process; not yet run on a shared bus. Advice
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

## What route monitoring reads and writes

| Direction | Key under `{realm}/@v0/{entity_id}/pubsub/` | Notes |
|---|---|---|
| in | `voyage/*` | Bootstrapped from router storage at start |
| in | `route/{id}/edition/{N}` | The edition an ACTIVE voyage pins. `latest` is never used |
| in | `route/{source_id}` | A vessel's declared route. Monitored only with no voyage in play, and only while its heartbeat is live |
| in | `location_fix/{source}`, `speed_over_ground_knots/{source}`, … | Sources from `inputs` |
| out | `route_execution/{voyage_id}` | 1 Hz. For a declared route the token is `declared-{source}` and `voyage_id` is empty |
| out | `envelope_exceedance/{voyage_id}/{exceedance_id}` | Voyages only. Opened on breach, restated in full on close |
| out | `alarm/navigation/0/route_monitor` | Deviation, envelope, no position, dangling route. Cleared by publishing an expired alarm |

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
z_sub -e tcp/localhost:7447 -k 'rise/@v0/slipway-usv-alpha/pubsub/raw_json/navigation/**'
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

What the state machine guarantees:

- **UNDERWAY** needs the conn, a level of `REMOTE_CONTROLLED` or above, fresh position, heading,
  course and speed, and a control mapping that is free or already this processor's. Another
  controller's mapping is reported, never overwritten.
- **Losing any of them** is FALLBACK: neutral axes for `neutral_burst_s`, then silence, so the
  vehicle's dead-man stops the hull. Arrival does the same, then releases the conn.
- **Pulling the lease stops the vessel.** After a FALLBACK the processor does not take the conn
  back, even with `take_conn_when_free`; only a grant naming it resumes.
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
| `set_guidance_order` | `track` (optionally rejoining at a waypoint id), `course_over_ground_deg`, `heading_deg`, `hold` or `go_to` (keelson#308: a position and an optional `arrival_radius_m`, default `guidance.goto_arrival_radius_m`), with an optional `speed_knots`. Course and heading keep their frames: a course is corrected for drift, a heading is not. Without guidance, only `track` is accepted |
| `set_voyage_status` | Pause, resume, complete or abort. Every change returns the order to `track` |
| `get_navigation_state` | The `keelson.NavigationState` it would broadcast now. Needs no authority |

Who may call: the holder of the live `command_authority` lease, by presenting its `token`; or a
controller listed in `control.controllers` (a scenario runner), while this processor holds the conn
itself **or nobody does**. Once anyone else holds the conn only that holder may, so an operator
taking the conn outranks the list. Everyone else is `DENIED`. Holding no route, the processor hands
a granted conn straight back, so an operator loads the route while it holds the conn and grants it
after; a permitted scenario runner loads it while the conn is free, and `take_conn_when_free` does
the rest.

The order in force is reported as `navigation_state.guidance_order`, and the UNDERWAY sub-mode
follows it: `SUB_MODE_STEERING` for a course or heading, `SUB_MODE_HOLDING` for hold. A `go_to`
steers for the position (`SUB_MODE_STEERING`), at the speed she has or `default_sog_knots` from a
standstill; inside the radius it holds with neutral axes (`SUB_MODE_HOLDING`, the order still in
force), and makes for the position again once she drifts out past twice the radius. While it steers,
guidance also publishes keelson#316's `heading_setpoint_deg` and, unless the frame is a heading,
`course_over_ground_setpoint_deg` (TimestampedFloat, `navigation/0/guidance`); neither is published
while it holds or goes neutral. An order is
given for a voyage: once that voyage is no longer the one selected, guidance returns to the track.
Every call that changes something is in the decision log (`KIND_VOYAGE_CHANGED`,
`KIND_GUIDANCE_ORDER`), with the voyage's own `audit_trail` naming the caller.

## Deployment

A vessel has one control mapping and the last writer wins, so with `guidance` enabled this processor
must not share a vessel with `keelson-processor-autopilot`, which re-installs its mapping whenever
the helm is free. Stop the autopilot container on any vessel this processor steers. Monitoring,
traffic, advice and conduct without guidance are safe beside it: they command nothing.

[config/slipway-usv-alpha-guidance.yaml](config/slipway-usv-alpha-guidance.yaml) sails Slipway's
`slipway-usv-alpha` on scenario `alpha-navigation`. It fixes the authority level because Slipway runs no
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
