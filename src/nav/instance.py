"""Which Slipway run a vessel belongs to, when more than one shares the bus (Slipway ADR-0139).

Slipway puts the run's *instance* -- one chunk of ``[a-z0-9]+``, by default the person's login --
into every simulated entity, immediately after ``sim-`` (crowsnest-dev#420), and only in keys:

    sim-slipway          ->  sim-ted-slipway
    sim-usv-alpha        ->  sim-ted-usv-alpha
    sim-ais-traffic      ->  sim-ted-ais-traffic

A config file keeps the authored names; ``--slipway-instance`` applies this rule to the vessel
this processor navigates and to every traffic feed, once, when the config is loaded. Any other
entity -- a real platform, ``roc1`` -- is not Slipway's to rename and passes through. The empty
instance renames nothing, which is the plain names.

A copy of Slipway's ``services/kernel/slipway_kernel/instance.py`` rule, not an import: this
processor does not depend on Slipway. Pure: no I/O, no zenoh.
"""

from __future__ import annotations

import re

# The entity prefix every simulated entity carries (RISE-Maritime/crowsnest-dev#420). Shared with
# every simulator on the bus, so it marks *simulated*, not *Slipway*.
PREFIX = "sim"
_VALID = re.compile(r"[a-z0-9]*")


def validate(instance: str) -> str:
    """The instance itself, or ValueError unless it is empty or one chunk of [a-z0-9]+.

    No hyphen, so that ``sim-<instance>-…`` can never be another run's vessel.
    """
    if not isinstance(instance, str) or not _VALID.fullmatch(instance):
        raise ValueError(
            f"slipway_instance {instance!r} must be lowercase letters and digits only, or empty"
        )
    return instance


def bus_entity(entity: str, instance: str) -> str:
    """An authored entity chunk as it appears on the bus under ``instance``."""
    if not instance:
        return entity
    if entity.startswith(f"{PREFIX}-"):
        return f"{PREFIX}-{instance}-{entity[len(PREFIX) + 1 :]}"
    return entity
