#!/bin/bash
# task 하나를 창으로 띄운다 (envs/drive.py).
#
#   ./run-world.sh <task> [--calibration] [--command=NAME] [--speed 0.15] [--turn 1.0]
#
#   <task>          training/ 아래 task 폴더 이름 — 로봇도 그 폴더 (robot_cfg.py · usda/robot.usda)
#                   기본: task env.py + command.py — 노란 점(랜덤 웨이포인트)을 기본 제어기가 따라감 (정책 보정 0) · R 새 에피소드
#   --calibration   task의 calibration/ — CalibrationEnv + CalibrationCommand (네모 54 s, 노란 점 = 목표 경로)
#   --command=NAME  명령만 바꿈 (task의 CalibrationEnv = 빈 바닥에서): keyboard (commands/keyboard.py 직접 운전) · square · MOD:CLASS
#                   (--command keyboard처럼 띄어 써도 된다)
#   --speed --turn  기준 전진 [m/s] · 회전 [rad/s] (keyboard: 키 하나 · 네모: 기준 속도)
#
# 키: W/A/S/D(화살표) 주행(keyboard) · Shift 느리게 · X 정지 · R 처음부터 · F 카메라를 로봇 쪽으로 · 창 닫으면 끝
# 카메라: 마우스 휠 줌 · 오른쪽 드래그 회전 · 가운데 드래그 이동 (고정 안 함)
# 화면: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 터미널: 0.5 s마다 명령 / 엔코더 / 자이로 (직접 운전)
#
#   ./run-world.sh rl_correction_waypoint
#   ./run-world.sh rl_correction_waypoint --calibration
#   ./run-world.sh rl_correction_waypoint --command=keyboard
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  echo "task: $(cd "$ISAAC/training" && ls -d */ | tr -d / | grep -v __ | tr "\n" " ")"
  exit 0
fi
cd "$ISAAC"   # 상대경로 기준 = isaac/
exec "$ISAACLAB/isaaclab.sh" -p -m envs.drive "$@"
