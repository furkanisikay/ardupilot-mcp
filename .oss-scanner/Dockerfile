# OSS Scanner build for ardupilot-mcp (https://github.com/anthropics/oss-scanner).
# Network is available only while this file builds; everything the audit needs is installed here.
FROM python:3.12-slim-bookworm

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1

# scanner contract: the checkout lives inside the image, at /src
COPY . /src
WORKDIR /src

# Install the package in editable mode with its dev extras (pytest, ruff, mypy),
# exactly as CI does, so tests and examples run fully offline afterwards.
RUN pip install -e ".[dev]"

# Pre-generate a realistic demo crash log so the agent has a ready .bin input.
RUN python examples/generate_demo_log.py /src/.oss-scanner/demo.bin

# Run the test suite as a smoke check; a failing test must not abort the build.
RUN pytest -q || echo "WARNING: pytest reported failures"
