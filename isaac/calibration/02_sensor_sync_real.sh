#!/bin/bash
# 02 — 센서 싱크 (real, PC). ROS/peers 자동 source.
#   ./02_sensor_sync_real.sh --check-only
#   ./02_sensor_sync_real.sh --label real_stand
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=25          # real 로봇 고정 (PC 환경에 99가 있어도 무시)
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
source "$REPO/tool/peers.sh"
exec python3 "$HERE/02_sensor_sync_real.py" "$@"
