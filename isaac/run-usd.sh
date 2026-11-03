#!/bin/bash
# 로봇 URDF → USD 변환만 (Isaac Sim을 창 없이 잠깐 켜서 — 변환기가 Isaac Sim 안에 있다).
#
#   ./run-usd.sh [robot]      robot = robots/ 아래 이름 또는 로봇 디렉토리 경로 (기본 beebot)
#
# 입력: <로봇 폴더>/urdf/robot.urdf.xacro + meshes/
# 출력: <로봇 폴더>/usd/<이름>/<이름>.usda (robots/ 안이면 git 제외)
# URDF · 메시가 USD보다 새것일 때만 다시 만든다 — 강제로 새로: rm -rf robots/<robot>/usd 후 실행.
# 평소엔 안 돌려도 된다: 학습 · calibration · run-world가 시작할 때 같은 함수(robots/urdf_to_usd.py)를 부른다.
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
robot="${1:-}"
[[ "$robot" == */* ]] && robot=$(realpath -m "$robot")   # 경로는 지금 폴더 기준 → 절대경로 (아래에서 isaac/로 cd)
dir="$robot"; [[ "$robot" != /* ]] && dir="$ISAAC/robots/$robot"
if [[ -n "$robot" && ! -f "$dir/urdf/robot.urdf.xacro" ]]; then
  echo "로봇 없음: $dir/urdf/robot.urdf.xacro (있는 것: $(cd "$ISAAC/robots" && ls -d */ | tr -d / | grep -v __ | tr "\n" " "))" >&2
  exit 2
fi
cd "$ISAAC"
"$ISAACLAB/isaaclab.sh" -p robots/convert_urdf.py ${robot:+"$robot"} 2>&1 | grep -E "^\[usd\]|Error|Traceback" | grep -v "omni.ext\|carb.scripting" || true
