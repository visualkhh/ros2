#!/bin/bash
# kos_ws 환경 켜기 — underlay(ros2_ws) + overlay(kos_ws)를 순서대로 source한다.
#   source ./setup.sh
set -eo pipefail
WS="$(cd "$(dirname "$0")" && pwd)"
UNDERLAY="/home/pank/workspaces/visualkhh/source/ros2/ros2_ws"

set +u
if [[ -f "$UNDERLAY/install/setup.bash" ]]; then
  source "$UNDERLAY/install/setup.bash"
fi
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash"
fi
set -u
echo "kos_ws ready (underlay: $UNDERLAY)"
