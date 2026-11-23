#!/usr/bin/env bash
# 컨테이너 셸에 들어간다. 소스는 /workspace/krosc에 그대로 마운트되어 있다.
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
docker compose -f "$here/docker-compose.yml" run --rm krosc bash
