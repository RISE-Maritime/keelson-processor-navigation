# CLAUDE.md — keelson-processor-navigation

## What this is

A keelson processor that owns navigation of an autonomous vessel under automation: route
monitoring, traffic and CPA, COLREG advice, guidance, the navigation state machine and the
decision log. One repo and image, one `source_id` per module (`{source_base}/{module}`).

**Status:** Phases 1 and 2 done (verified live against Slipway `slipway-alpha` and in Crowsnest).
Phase 3, the advisory COLREG advisor, is code complete and accepted against Slipway's
`rivo-crossing` replayed from a fixture. Commands nothing.

## Where to look

- [docs/PLAN.md](docs/PLAN.md) — the approved design.
- [TODO.md](TODO.md) — invariants and the phase checklist. Read the invariants first.
- [docs/upstream/](docs/upstream/) — draft keelson protos.
- `/Users/ted/Documents/CODE/crowsnest-dev/docs/NAVIGATION-DECISION-SERVICES.md` — the design note
  this processor answers.

## Layout

```
src/nav/
  app.py            CLI, zenoh session, liveliness, publishers, tick loop. The only zenoh user
                    besides bus/.
  config.py         DEFAULTS < YAML < CLI; entity_id required; unimplemented modules refused
  runtime.py        Processor.tick(now_ns) -> [Output]; pure and deterministic
  bus/codec.py      payload dict (protobuf JSON form) -> proto or raw_json rider
  bus/riders.py     reads contracts/raw_json_riders.yaml
  bus/inputs.py     subscriptions + storage bootstrap -> World
  bus/translate.py  keelson protobuf -> plan and sample types (proto3 presence kept)
  world/            RoutePlan/VoyagePlan, newest-wins Latch (local receipt time), World snapshot
  route_monitor/    geodesy, select (which route), progress, envelope, alarms, monitor (per tick)
  traffic/          cpa (flat frame, matches Crowsnest), monitor (range filter, per-target outputs)
  world/targets.py  TargetTable: newest receipt per subject per target, expiry by position age
  outputs.py        Emit: subject, payload, optional source_id and target_id
  colreg/           classify (Rules 13-19), risk (grades per context), synthesize (manoeuvres vs all
                    targets), records (payloads), advisor (encounter lifecycle, advice, alarms)
  conduct/state.py  NavigationState, mirrors docs/upstream/NavigationState.proto
```

## Commands

```bash
.venv/bin/pytest                               # all; -m unit or -m e2e
.venv/bin/ruff check bin/ src/ tests/ && .venv/bin/ruff format --check bin/ src/ tests/
.venv/bin/navigation --config config/processor.yaml --connect tcp/localhost:7447
docker compose up --build
```

## Conventions

- Payloads are dicts in protobuf JSON form (snake_case, enum names, RFC 3339). Never build
  protobuf objects above `bus/`.
- New logic goes behind `Processor.tick`, so Phase 5 can drive it from Slipway's tick fence.
- Tests never touch the shared router. e2e sessions disable multicast and connect nowhere.
- The colreg module may emit only encounter, navigation_advice, advice_disposition and alarm. A test
  enforces it. Nothing in colreg/ imports a vehicle interface.
- A rider is published on its module's own `raw_json` key; ids travel in the payload.
- `tests/fixtures/rivo_crossing.jsonl` is regenerated from the Slipway kernel in-process, never
  from the shared bus.
- keelson SDK is the monorepo one (`../keelson/sdks/python`), installed editable.
- ruff, line length 100.
- Scripted edits must assert the old text exists. An unasserted replace that silently missed once
  published targets on the wrong keys; only the e2e test caught it.
