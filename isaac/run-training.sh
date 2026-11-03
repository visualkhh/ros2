#!/bin/bash
# 학습 · 재생 — Isaac Lab 공식 스크립트(scripts/reinforcement_learning/train.py · play.py)를 우리 task로 돌린다.
#
#   ./run-training.sh [train] [옵션...]            학습 (기본)
#   ./run-training.sh play --checkpoint <model_N.pt> [옵션...]   재생 + 내보내기 (exported/policy.onnx · policy.pt) — Ctrl+C로 끝
#   ./run-training.sh train --help                  공식 옵션 전체
#
# task: training/<task>/__init__.py의 gym.register 이름 (기본 RlCorrectionWaypoint — 로봇은 env.robot= · 기본 robots.DEFAULT). --external_callback training.register로 등록.
# 결과: isaac/logs/rsl_rl/<experiment_name>/<시각>/  (model_<N>.pt · params/ · TensorBoard · play 하면 exported/)
#
# ─── 자주 쓰는 공식 옵션 ───────────────────────────────────────────────────────
#   --num_envs N          동시에 학습하는 로봇 수 (기본: env cfg 512 · 창 볼 땐 16~32)
#   --max_iterations N    학습 반복 (기본: agent.py 500). 1회 = envs × 64스텝 수집 + PPO 업데이트
#   --viz [kit|...]       창으로 보기 (기본은 창 없음 · --viz만 쓰면 kit)
#   --video               영상 녹화 (logs/…/videos)
#   --resume --checkpoint <model_N.pt>   이어서 학습
# ─── cfg 덮어쓰기 (Hydra: env.<필드>=값 · agent.<필드>=값) ──────────────────────
#   env.robot=<이름|폴더>  로봇만 바꿔 실험 (실측 중심은 task cfg 그대로 — 제대로는 로봇별 변형 등록)
#   env.slope_deg=5       경사 칸 0~5° (기본 3, 0 = 없음)
#   env.uneven=true       요철 칸 (턱 1~3 cm)
#   env.events=null       바퀴 마찰 랜덤 끔
#   env.gyro_bias=0.01    자이로 영점 오차 ±0.01 rad/s (기본 0.005, 0 = 끔)
#   env.debug_vis=true    창에 명령(초록)/실제(파랑) 막대 + 목표 경로(노란 점)
#   agent.algorithm.entropy_coef=0.0 · agent.experiment_name=이름
#
# ─── 예 ────────────────────────────────────────────────────────────────────────
#   ./run-training.sh --num_envs 16 --max_iterations 5 --viz env.debug_vis=true          # 짧게 확인 (실기기에 올리지 말 것)
#   ./run-training.sh --max_iterations 300 env.uneven=true                                # 본 학습
#   ./run-training.sh play --checkpoint logs/rsl_rl/rl_waypoint/<시각>/model_499.pt --num_envs 1   # → <시각>/exported/policy.onnx
#   tensorboard --logdir logs/rsl_rl
#   policy.onnx 출력은 신경망 그대로 [-1, 1] — 로봇 corrector가 clamp × action_scale(0.10 m/s · 0.60 rad/s)을 곱한다.
#   exported/policy.onnx.data(가중치)도 같이 복사할 것.
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"

# 창은 기본 끔 (headless) — --viz만 쓰면 --viz kit, --viz X · --viz=X는 그대로
viz_args() {
  VIZ_ARGS=()
  while [[ $# -gt 0 ]]; do
    if [[ "$1" == "--viz" || "$1" == "--visualizer" ]] && [[ $# -eq 1 || "$2" == -* ]]; then VIZ_ARGS+=("$1" kit)
    else VIZ_ARGS+=("$1"); fi
    shift
  done
}

if [[ "${1:-}" == "-h" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
action=train
if [[ "${1:-}" == "train" || "${1:-}" == "play" ]]; then
  action=$1; shift
fi
viz_args "$@"; args=(${VIZ_ARGS[@]+"${VIZ_ARGS[@]}"})
[[ " $* " == *" --task "* ]] || args=(--task RlCorrectionWaypoint "${args[@]}")
cd "$ISAAC"   # logs/ · 상대경로 기준 = isaac/
export PYTHONPATH="$ISAAC${PYTHONPATH:+:$PYTHONPATH}"   # training.register · robots · envs (ROS 경로는 유지 — xacro)
exec "$ISAACLAB/isaaclab.sh" -p "$ISAACLAB/scripts/reinforcement_learning/$action.py" \
  --rl_library rsl_rl --external_callback training.register "${args[@]}"
