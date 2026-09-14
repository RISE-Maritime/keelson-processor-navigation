# keelson-processor-navigation

A [keelson](https://github.com/RISE-Maritime/keelson) processor that owns navigation of an
autonomous vessel (MASS, USV) while it is under automation.

> **Status: Phase 3, COLREG advisor.** It monitors the vessel's route, tracks traffic with CPA,
> and classifies encounters under COLREG Rules 13 to 19, publishing encounters, advice and what
> became of each advice. **It commands nothing**: advice is a record, and acting on it is Phase 4,
> after someone accepts it. See [TODO.md](TODO.md) and [docs/PLAN.md](docs/PLAN.md).

## What it will do

| Module | Role | Output |
|---|---|---|
| `route_monitor` | Tracks the active voyage against its pinned route edition | `route_execution`, `envelope_exceedance` |
| `traffic` | Target table from `@target` trees, CPA and TCPA | `target_cpa_m`, `target_tcpa_s`, … |
| `colreg` | Classifies encounters and proposes manoeuvres. Advice only, never a command | `encounter`, `navigation_advice` |
| `guidance` | Steers along the track and executes accepted advice | `vehicle_control/v1` axes or `vehicle_navigation/v1` |
| `conduct` | Navigation state machine gated on the conn and operational authority | `navigation_state` |
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
z_sub -e tcp/localhost:7447 -k 'rise/@v0/slipway-alpha/pubsub/raw_json/navigation/**'
```

## What the COLREG advisor publishes

| Subject (rider on `raw_json/navigation/0/colreg` until keelson types it) | Content |
|---|---|
| `encounter` | Target, range, bearing, CPA, TCPA, situation, own role, rules, risk grade, opened and closed times. Restated each second while open |
| `navigation_advice` | One advice for the vessel: course and/or speed, the encounters it answers, validity window, expected CPA, rules, rationale |
| `advice_disposition` | Accepted or rejected (from an operator or a conduct service), superseded or expired (from the advisor) |
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

## Container

```bash
docker compose up --build
```

The build takes keelson from `../keelson/sdks/python` as a named build context. Run
`./generate_python.sh` in that checkout first.

## Development

See [CONTRIBUTING.md](CONTRIBUTING.md).
