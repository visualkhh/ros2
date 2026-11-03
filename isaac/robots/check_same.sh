#!/bin/bash
# 학습용(isaac)과 ROS용(ros2_ws) 로봇 URDF·메시가 같은지 — 의도적으로 파일을 나눴고 내용은 같아야 한다.
#   ./check_same.sh   (다르면 diff 출력 + 실패)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
ROS=$HERE/../../ros2_ws/src/tb3_bringup
rc=0
diff -u "$ROS/urdf/robot.urdf.xacro" "$HERE/beebot/urdf.xacro" || rc=1
for m in body.stl wheel_left.stl wheel_right.stl; do
  if ! cmp -s "$ROS/meshes/$m" "$HERE/beebot/meshes/$m"; then
    echo "메시 다름/없음: $m (두 meshes/ 폴더에 같은 파일이어야 함)"; rc=1
  fi
done
[ $rc = 0 ] && echo "OK: urdf.xacro · 메시 3개 같음"
exit $rc
