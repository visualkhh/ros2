#!/usr/bin/env bash
# Shared build/run paths for srf_recycling_ws scripts.
# Linux(Pi/PC) keeps the existing build/ directories; macOS uses build-mac/
# because the repo's srf/build may be a Linux/Docker CMake cache.
SRF_RECYCLING_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRF_RECYCLING_WS_ROOT="$(cd "$SRF_RECYCLING_WS_DIR/.." && pwd)"
if [ "$(uname -s)" = "Darwin" ]; then
  SRF_BUILD_DIR="${SRF_BUILD_DIR:-$SRF_RECYCLING_WS_ROOT/srf/build-mac}"
  SRF_RECYCLING_WS_BUILD_DIR="${SRF_RECYCLING_WS_BUILD_DIR:-$SRF_RECYCLING_WS_DIR/build-mac}"
else
  SRF_BUILD_DIR="${SRF_BUILD_DIR:-$SRF_RECYCLING_WS_ROOT/srf/build}"
  SRF_RECYCLING_WS_BUILD_DIR="${SRF_RECYCLING_WS_BUILD_DIR:-$SRF_RECYCLING_WS_DIR/build}"
fi
export SRF_RECYCLING_WS_ROOT SRF_RECYCLING_WS_DIR SRF_BUILD_DIR SRF_RECYCLING_WS_BUILD_DIR

# Wait until a local TCP port accepts connections. macOS bash 3.2 has no /dev/tcp.
wait_tcp() {
  local port="$1"
  local attempts="${2:-30}"
  local i=0
  while [ "$i" -lt "$attempts" ]; do
    if command -v nc >/dev/null 2>&1 && nc -z 127.0.0.1 "$port" >/dev/null 2>&1; then
      return 0
    fi
    i=$((i + 1))
    sleep 0.2
  done
  return 1
}
