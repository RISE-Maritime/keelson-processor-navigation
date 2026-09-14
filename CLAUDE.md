# CLAUDE.md — keelson-processor-navigation

## What this is

A keelson processor that owns navigation of an autonomous vessel under automation: route
monitoring, traffic and CPA, COLREG advice, guidance, the navigation state machine and the
decision log. One repo and image, one `source_id` per module (`{source_base}/{module}`).

**Status:** Phase 0 done. Publishes `navigation_state{MODE_IDLE}` as a raw_json rider, subscribes
to nothing, commands nothing. Next is Phase 1, the `route_execution` producer.

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
- keelson SDK is the monorepo one (`../keelson/sdks/python`), installed editable.
- ruff, line length 100.
