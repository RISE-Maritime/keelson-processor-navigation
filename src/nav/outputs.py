"""What a module asks the app to publish."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Emit:
    subject: str
    payload: dict
    source_id: str | None = None  # None: the module's own source id
    target_id: str | None = None  # set: published under .../@target/{target_id}
