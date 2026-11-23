#!/usr/bin/env bash
# Docker 이미지를 만든다 (Ubuntu 26.04 + GCC + CMake + Qt6).
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker compose -f "$here/docker-compose.yml" build
