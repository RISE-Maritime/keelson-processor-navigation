# syntax=docker/dockerfile:1
FROM python:3.13-slim-bookworm

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

# keelson is pinned exactly in pyproject.toml and installed from PyPI with the other dependencies.
COPY pyproject.toml README.md ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system -r pyproject.toml

COPY src src
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system --no-deps .

COPY --chmod=555 ./bin/* /usr/local/bin/

RUN useradd --uid 10001 --no-create-home --shell /usr/sbin/nologin app
USER app

ENTRYPOINT ["navigation"]
CMD ["--config", "/config/processor.yaml", "--connect", "tcp/host.docker.internal:7447"]
