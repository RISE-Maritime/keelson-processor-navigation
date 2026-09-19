# Contributing

## Setup

Python 3.11 or newer and [uv](https://docs.astral.sh/uv/). keelson is an exact published version
pinned in `pyproject.toml` (an alpha from a keelson PR while that PR is open), never a local
checkout: a pin names one keelson generation, and a checkout names whatever branch it is on.

```bash
uv venv --python 3.13
uv pip install -e ".[dev]"
```

## Checks

```bash
.venv/bin/ruff check bin/ src/ tests/
.venv/bin/ruff format --check bin/ src/ tests/
.venv/bin/pytest -m unit      # pure, no zenoh session
.venv/bin/pytest -m e2e       # isolated local peer session, no router
```

The e2e tests disable multicast scouting and connect to nothing. Keep it that way: the shared
RISE router would put a test vessel in front of every Crowsnest user.

## Rules

- Read the invariants in [TODO.md](TODO.md) before changing behaviour.
- A new draft subject needs a draft proto in `docs/upstream/`, an entry in
  `src/nav/contracts/raw_json_riders.yaml` with the upstream check recorded, and a payload class
  whose fields the tests check against the draft.
- Enabling a module that is not implemented must stay a configuration error.
- Keep zenoh out of everything except `app.py` and `bus/`, so logic is testable without a session.
