# CLAUDE.md — keelson-processor-navigation

## What this is

A keelson processor that owns navigation of an autonomous vessel under automation: route
monitoring, traffic and CPA, COLREG advice, guidance, the navigation state machine and the
decision log. One repo and image, one `source_id` per module (`{source_base}/{module}`).

**Status:** Phases 1 and 2 done (verified live against Slipway `slipway-alpha` and in Crowsnest).
Phase 3, the advisory COLREG advisor, is accepted against Slipway's `rivo-crossing` replayed from a
fixture. Phase 4, guidance and the conduct state machine, commands a vessel over
`vehicle_control/v1`; accepted in-process on Slipway's kernel, not yet run on a shared bus.

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
  conduct/lease.py  command_authority lease: book (receiver clock) and client (take/grant/heartbeat/release)
  conduct/authority.py  OperationalAuthority.level gates
  conduct/fsm.py    IDLE/READY/UNDERWAY/FALLBACK/ARRIVED/ENDED; pure, Gates in, Step out
  guidance/         track (wheel-over, line of sight, PD helm), speed, axes (mapping ownership,
                    Actuation), failsafe, guide (orders + navigation_setpoint)
  log/decision_log.py  navigation_decision append + periodic restatement
  bus/vehicle_control.py  HelmBackend: axis timer and mapping RPCs, the only thing that drives
  bus/gate_candidates.py  GateBackend (guidance.backend: gate): guidance_candidate per DOF and a
                    read of the gate's map; it never installs a mapping (crowsnest-dev#94)
  guidance/candidates.py  each DOF's decision and execution basis behind the gate; pure
  control/          navigation_control/v1: orders (GuidanceOrder), commands (who may call,
                    load_route, voyages), decided in the tick
  bus/control.py    the four queryables: decode, cross-vessel route fetch, queue, wait, encode
```

## Commands

```bash
.venv/bin/pytest                               # all; -m unit or -m e2e
.venv/bin/ruff check bin/ src/ tests/ && .venv/bin/ruff format --check bin/ src/ tests/
.venv/bin/navigation --config config/processor.yaml --connect tcp/localhost:7447
docker compose up --build
# Slipway acceptance (Slipway's env; skipped in this venv):
cd ../SLIPWAY && PYTHONPATH=../keelson-processor-navigation/src:../keelson-processor-navigation \
  .venv/bin/python -m pytest ../keelson-processor-navigation/tests/test_acceptance_slipway_guidance.py
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
  from the shared bus, with `tools/rivo_crossing_fixture.py` (the command is in its docstring).
  A regeneration against an unchanged trajectory must be byte-identical.
- keelson is an exact published EXPERIMENTAL version pinned in `pyproject.toml` (Ted, 2026-11-25:
  experimental releases only; now `0.6.0.dev25`, the newest carrying the #316 setpoint subjects,
  ControlGate and NavigationState.order_id/ordered_by), never the monorepo checkout: that checkout is
  on whatever branch another session left it.
- ruff, line length 100.
- The tick never publishes axes. It leaves `Processor.actuation`; `bus/vehicle_control.py` repeats
  it at 10-20 Hz and stops at `valid_until_ns`.
- Pulling the lease must stop the vessel: after FALLBACK the processor never takes the conn back on
  its own, only a grant naming it resumes. `tests/test_fsm.py` holds the matrix. **Except as the
  conn's default holder** (`conduct.default_holder`, Ted 2026-11-25): a released or lapsed station
  conn comes back to it with the order in force; stopping is an order or the E-stop (README).
- `command_authority` and `command_request` go out under source_id = entity_id: Slipway subscribes
  `command_authority/*`, one chunk.
- A control loop on a Slipway L0 hull has no inertia to hide behind: keep throttle and yaw-damping
  loop gains below 1 (config refuses the throttle one).
- Scripted edits must assert the old text exists. An unasserted replace that silently missed once
  published targets on the wrong keys; only the e2e test caught it.
