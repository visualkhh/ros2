#!/usr/bin/env bash
# 웹 Viz를 연다.
#   ./docker/web.sh
#   브라우저: http://localhost:8080/
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
echo "web: http://localhost:8080/"
exec docker compose -f "$here/docker-compose.yml" up krosc-web
