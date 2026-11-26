#!/usr/bin/env bash
# macOS 전용: krosc_ws 시뮬레이터(Qt 진실 뷰) + 웹 Viz 게이트웨이를 함께 실행한다.
#   ./run-mac.sh
#   ./run-mac.sh --pucks 4 --no-noise
#   KROSC_VIZ_WEB_PORT=8081 ./run-mac.sh
set -euo pipefail
here="$(cd "$(dirname "$0")" && pwd)"

if [ "$(uname -s)" != "Darwin" ]; then
  echo "[run-mac] macOS 전용 스크립트다. Linux는 run-simulator-vizweb.sh를 쓴다." >&2
  exit 2
fi

source "$here/paths.sh"
exec "$here/run-simulator-vizweb.sh" "$@"
