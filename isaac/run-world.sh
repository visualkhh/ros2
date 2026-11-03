#!/bin/bash
# 로봇을 직접 몰아보는 창 — 로봇(USD)과 환경(env · env cfg)을 골라서 띄운다 (envs/drive.py).
#
#   ./run-world.sh [--robot NAME|DIR] [--env NAME|MOD:CLASS] [--env-cfg MOD:CLASS] [--cmd keyboard|square|MOD:CLASS] [--speed 0.15] [--turn 1.0]
#
#   --robot     로봇 = robot_cfg.py가 있는 폴더 (robots/ 아래면 이름만) — 모양: robot_cfg.py + usda/robot.usda (그대로)
#               또는 robot_cfg.py + urdf/robot.urdf.xacro · meshes/ (usd/로 자동 변환)
#               기본: robots.DEFAULT = training/rl_correction_waypoint. 바퀴 관절 이름은 wheel_left/right_joint여야 함
#   --env       환경: envs/ 아래 이름 (기본 default — 아무것도 없는 평평한 바닥) · 없는 이름이면 MOD:CLASS로 직접
#               이름 규칙: envs/<name>/env.py <Name>Env + envs/<name>/env_cfg.py <Name>EnvCfg
#   --env-cfg   환경 설정 MOD:CLASS — 안 주면 --env 규칙으로 (MOD:CLASS면 <mod>_cfg:<Class>Cfg로 추측)
#               행동이 [v, w] 명령인 env면 된다 (BaseEnv 계열: envs/default · envs/rooms)
#   --cmd       명령 (commands/): keyboard = 직접 운전 (기본) · square = 네모 54 s (노란 점 = 목표 경로, calibration 측정과 같은 명령)
#               · MOD:CLASS = commands.base_command.BaseCommand 상속 클래스 (생성자 speed=, turn=)
#   --speed --turn   기준 전진 [m/s] · 회전 [rad/s] (keyboard: 키 하나 · square: 사각형 기준 속도)
#
# 키: W/A/S/D(화살표) 주행(keyboard) · Shift 느리게 · X 정지 · R 원점 + 명령 처음부터 · N 다음 방(rooms) · F 카메라 · 창 닫으면 끝
# 카메라: 마우스 휠 줌 · 오른쪽 드래그 회전 · 가운데 드래그 이동 (고정 안 함)
# 화면: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 (cfg.debug_vis) · 터미널: 0.5 s마다 명령 / 엔코더 / 자이로
#
#   ./run-world.sh
#   ./run-world.sh --robot training/rl_correction_waypoint
#   ./run-world.sh --robot ~/my_robots/other               # 다른 로봇 폴더 (robot_cfg.py + usda/robot.usda)
#   ./run-world.sh --cmd square                                  # 로봇이 혼자 네모 — 드리프트를 노란 점과 비교
#   ./run-world.sh --env rooms                                   # 방 + 장애물 (N = 다음 방)
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
args=()   # --robot 경로는 지금 폴더 기준 → 절대경로로 (아래에서 isaac/로 cd 하므로)
while [[ $# -gt 0 ]]; do
  if [[ "$1" == "--robot" && "${2:-}" == */* ]]; then args+=(--robot "$(realpath -m "$2")"); shift 2
  else args+=("$1"); shift; fi
done
cd "$ISAAC"   # 상대경로 기준 = isaac/
exec "$ISAACLAB/isaaclab.sh" -p -m envs.drive ${args[@]+"${args[@]}"}
