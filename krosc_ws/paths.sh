#!/usr/bin/env bash
# Shared build/run paths for krosc_ws scripts.
# Linux(Pi/PC) keeps the existing build/ directories; macOS uses build-mac/
# because the repo's krosc/build may be a Linux/Docker CMake cache.
KROSC_WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KROSC_ROOT="$(cd "$KROSC_WS_DIR/.." && pwd)"
if [ "$(uname -s)" = "Darwin" ]; then
  KROSC_BUILD_DIR="${KROSC_BUILD_DIR:-$KROSC_ROOT/krosc/build-mac}"
  KROSC_WS_BUILD_DIR="${KROSC_WS_BUILD_DIR:-$KROSC_WS_DIR/build-mac}"
else
  KROSC_BUILD_DIR="${KROSC_BUILD_DIR:-$KROSC_ROOT/krosc/build}"
  KROSC_WS_BUILD_DIR="${KROSC_WS_BUILD_DIR:-$KROSC_WS_DIR/build}"
fi
export KROSC_ROOT KROSC_WS_DIR KROSC_BUILD_DIR KROSC_WS_BUILD_DIR

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
