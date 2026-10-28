#!/bin/bash
# beebot 학습용 URDF 메시 내려받기 (git 제외, 용량 큼)
#   출처: visualkhh/Lv2_bee_Assignment@feature/isaac-pick lv2_module6/urdf/beebot_mesh/meshes
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
BASE="https://raw.githubusercontent.com/visualkhh/Lv2_bee_Assignment/feature/isaac-pick/lv2_module6/urdf/beebot_mesh/meshes"
mkdir -p "$HERE/meshes/storage_collision"
for m in arm_base_1 base_link camera_rear_link camera_wrist_link finger_L_1 finger_R_1 \
         gripper_body_1 link1_1 link2_1 shoulder_1 wheel_left_link wheel_right_link wrist_1; do
  [ -f "$HERE/meshes/$m.stl" ] || curl -sL "$BASE/$m.stl" -o "$HERE/meshes/$m.stl" &
done
for i in 0 1 2 3 4 5 6 7 8; do
  [ -f "$HERE/meshes/storage_collision/storage_$i.stl" ] || \
    curl -sL "$BASE/storage_collision/storage_$i.stl" -o "$HERE/meshes/storage_collision/storage_$i.stl" &
done
wait
echo "OK: $(du -sh "$HERE/meshes" | cut -f1) in $HERE/meshes"
