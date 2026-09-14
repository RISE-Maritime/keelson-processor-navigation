# keelson-processor-navigation

A [keelson](https://github.com/RISE-Maritime/keelson) processor that owns navigation of an
autonomous vessel (MASS, USV) while it is under automation.

> **Status: Phase 0 skeleton.** It connects, declares liveliness and publishes
> `navigation_state` as `MODE_IDLE` at 1 Hz. It subscribes to nothing and commands nothing. See
> [TODO.md](TODO.md) for what comes next and [docs/PLAN.md](docs/PLAN.md) for the design.

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

There is no default `entity_id`. The processor refuses to start without one.

Watch it:

```bash
z_sub -e tcp/localhost:7447 -k 'rise/@v0/slipway-alpha/pubsub/raw_json/navigation/**'
```

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
