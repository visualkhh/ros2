#!/bin/bash
# 로봇을 직접 몰아보는 창 — 환경을 골라서.
#
#   ./run-world.sh [train|random|empty] [옵션...]       (기본 train)
#
#   train   학습 env 그대로 (Isaac Lab 지형: 평평·경사·요철 칸 · 바퀴 마찰/gain/τ/지연 랜덤 · 자이로 영점 오차), 로봇 1대
#           = ./run-training.sh --drive --watch [옵션...]
#     --policy ONNX     보정 정책 (C 키로 켬/끔) — 없으면 바이패스
#     --fit F,G         학습 때와 같은 fit (바퀴 랜덤 범위 중심)
#     --uneven          요철 켜기 · --slope DEG (기본 3, 0 = 평평) · --no-friction · --gyro-bias RAD_S
#     --speed 0.15 --turn 1.0   키 하나의 속도 [m/s] · 회전 [rad/s]
#   random  랜덤 방 + 평평한 바닥 (calibration) = ./run-calibration.sh random-world [옵션...]
#   empty   기본 환경 (envs/default DefaultEnv: 로봇 1대 · 평평한 바닥, 아무것도 없음) = envs/drive.py · R = 원점
#     --fit F[,G] · --seed 7 · --rooms 8 · N 다음 방 · C 보정값 창
#
# 키: W/A/S/D(화살표) 주행 · Shift 느리게 · X 정지 · R 리셋 (train: 제자리 · 바퀴 gain/τ 새로) · 창 닫으면 끝
# 화면: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 · 노란 점 = 지금 명령대로 갈 길 → 벗어나는 만큼 드리프트
# 터미널(train): 0.5 s마다 명령 / 실제 v·w / 보정 Δv·Δw / 정책 켬·끔
#
#   FIT=artifacts/calibration/20261102_140931-02-real/fit.json,artifacts/calibration/20261102_144928-sim_480/fit.json
#   ./run-world.sh --fit $FIT --uneven --policy artifacts/real/policy_30.onnx
#   ./run-world.sh random --fit $FIT
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
kind=train
if [[ "${1:-}" == "train" || "${1:-}" == "random" || "${1:-}" == "empty" ]]; then
  kind=$1; shift
fi
if [[ $kind == empty ]]; then
  cd "$ISAAC"
  exec "${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}/isaaclab.sh" -p -m envs.drive "$@"
fi
if [[ $kind == random ]]; then
  exec "$ISAAC/run-calibration.sh" random-world "$@"
fi
exec "$ISAAC/run-training.sh" --drive --watch --exp-name drive "$@"
