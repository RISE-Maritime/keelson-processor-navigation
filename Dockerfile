# syntax=docker/dockerfile:1
FROM python:3.13-slim-bookworm

COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

WORKDIR /app

# keelson from the monorepo SDK, passed as the named build context `keelson_sdk`
# (docker-compose.yml wires it to ../keelson/sdks/python). The PyPI wheel lacks payloads.Route_pb2,
# which later phases need. The SDK's generated *_pb2 files are gitignored upstream, so run
# `./generate_python.sh` in the keelson checkout before building.
COPY --from=keelson_sdk . /opt/keelson-sdk
RUN --mount=type=cache,target=/root/.cache/uv \
    uv pip install --system /opt/keelson-sdk

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
