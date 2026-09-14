"""Regenerate tests/fixtures/rivo_crossing.jsonl from Slipway's kernel, in-process.

Never from the shared bus (CLAUDE.md). Run with Slipway's own environment, so its kernel is the one
imported:

    cd ../SLIPWAY && uv run python ../keelson-processor-navigation/tools/rivo_crossing_fixture.py \
        ../keelson-processor-navigation/tests/fixtures/rivo_crossing.jsonl

One row per second for 600 s (6000 ticks at 100 ms), longer than the scenario's own duration so the
pass and the opening afterwards are both in it. Every vessel is started at tick 0, as Slipway's
trajectory pins do; the scenario's events are marks and are not fired. Knots are m/s * 3600 / 1852,
which is what the committed fixture was written with: another constant changes the last digit of
some speeds and the file stops being byte-identical, which is the check a regeneration should pass.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from slipway_kernel import model
from slipway_kernel.model import SimState
from slipway_kernel.scenario import load_scenario

SLIPWAY = Path(__file__).resolve().parents[2] / "SLIPWAY"
TICKS = 6000
EVERY = 10


def _row(world, scenario, dt_s: float) -> dict:
    vessels = {}
    for v in world.vessels:
        lat, lon = scenario.plane.wgs84_from_enu(v.east_m, v.north_m)
        vessels[v.entity_id] = {
            "lat": lat,
            "lon": lon,
            "heading_deg": v.heading_deg,
            "sog_knots": v.sog_mps * 3600.0 / 1852.0,
            "east_m": v.east_m,
            "north_m": v.north_m,
        }
    return {
        "tick": world.tick,
        "t_s": round(world.tick * dt_s, 6),
        "vessels": vessels,
        "targets": {},
    }


def main(out: Path) -> None:
    scenario = load_scenario(
        SLIPWAY / "scenarios" / "rivo-crossing.yaml", vessel_dir=SLIPWAY / "vessels"
    )
    world = model.initial_world(scenario, run_id="fixture")
    if world.targets:
        raise SystemExit("rivo-crossing now has contacts; extend _row before regenerating")
    for cfg in scenario.vessels:
        world = model.set_sim_state(world, cfg.entity_id, SimState.RUNNING)
    dt_s = world.dt_ns / 1e9
    lines = []
    for _ in range(TICKS):
        world = model.step(scenario, world)
        if world.tick % EVERY == 0:
            lines.append(json.dumps(_row(world, scenario, dt_s), separators=(",", ":")) + "\n")
    out.write_text("".join(lines))
    print(f"wrote {len(lines)} rows to {out}")


if __name__ == "__main__":
    main(Path(sys.argv[1]))
