"""The raw_json rider register: draft subjects that keelson does not type yet.

The register itself is ``nav/contracts/raw_json_riders.yaml``. This module only reads it.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import cache
from importlib import resources

import yaml

RIDER_VERSION = 1


@dataclass(frozen=True)
class Rider:
    record: str
    draft_proto: str
    message: str
    status: str


@cache
def load_register() -> dict[str, Rider]:
    text = resources.files("nav.contracts").joinpath("raw_json_riders.yaml").read_text("utf-8")
    doc = yaml.safe_load(text)
    riders = {}
    for entry in doc["riders"]:
        rider = Rider(
            record=entry["record"],
            draft_proto=entry["draft_proto"],
            message=entry["message"],
            status=entry["retire_when"]["status"],
        )
        if rider.record in riders:
            raise ValueError(f"duplicate rider record {rider.record!r}")
        riders[rider.record] = rider
    return riders


def rider_subjects() -> frozenset[str]:
    return frozenset(load_register())
