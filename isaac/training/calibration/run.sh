#!/bin/bash
# Isaac Lab venv로 calibration/ 스크립트 실행 (cd 없이)
#   ./run.sh 01_random_world_sim.py [스크립트 옵션...]
# 기본은 창 표시(--viz kit). 창 없이 돌리려면 --viz none 을 직접 넘긴다.
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
if [ $# -eq 0 ]; then
  echo "사용법: $0 <스크립트> [옵션...]   예: $0 01_random_world_sim.py" >&2
  exit 2
fi
ARGS=("$@")
if [[ " ${ARGS[*]} " != *" --viz "* && " ${ARGS[*]} " != *" --visualizer "* ]]; then
  ARGS+=(--viz kit)
fi
cd /home/pank/pa-course/IsaacLab-6.1
exec ./isaaclab.sh -p "$HERE/${ARGS[0]}" "${ARGS[@]:1}"
