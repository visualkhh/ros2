#!/usr/bin/env bash
# 리눅스 컨테이너의 데스크톱을 noVNC로 연다.
#   ./docker/vnc.sh
#   브라우저: http://localhost:6080/vnc.html?autoconnect=1&resize=scale
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"
echo "noVNC: http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
exec docker compose -f "$here/docker-compose.yml" up srf-vnc
