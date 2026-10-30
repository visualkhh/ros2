#!/bin/bash
# URDF → USD만 다시 생성 (Isaac headless)
#   ./convert_usd.sh
# 결과: isaac/usd/tb3_waffle_pi/tb3_waffle_pi.usda
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
cd /home/pank/pa-course/IsaacLab-6.1
exec ./isaaclab.sh -p "$HERE/scripts/convert_urdf.py" "$@"
