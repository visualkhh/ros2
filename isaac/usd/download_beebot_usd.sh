#!/bin/bash
# beebot USDA (학습용 풀모델) 내려받기 — git 제외, 용량 큼
#   출처: visualkhh/Lv2_bee_Assignment@feature/isaac-pick lv2_module6/isaac/sim/usda/beebot
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)/beebot
BASE="https://raw.githubusercontent.com/visualkhh/Lv2_bee_Assignment/feature/isaac-pick/lv2_module6/isaac/sim/usda/beebot"
mkdir -p "$HERE/payloads/Physics"
for f in beebot.usda payloads/base.usda payloads/robot.usda payloads/instances.usda \
         payloads/Physics/mujoco.usda payloads/Physics/physics.usda payloads/Physics/physx.usda \
         payloads/geometries.usd; do
  [ -f "$HERE/$f" ] || curl -sL "$BASE/$f" -o "$HERE/$f" &
done
wait
echo "OK: $(du -sh "$HERE" | cut -f1) in $HERE"
