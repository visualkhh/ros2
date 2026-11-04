#!/bin/bash
# 학습 체크포인트 → ONNX (tools/onnx_tool.py · isaac venv python — onnx · onnxruntime).
#
#   ./run-make-onnx.sh <model_N.pt> [--quantize]       → <pt 폴더>/exported/policy.onnx (+ .onnx.data · policy.pt)
#   ./run-make-onnx.sh --quantize-only <policy.onnx>    → 같은 폴더 policy.int8.onnx
#
#   task는 경로에서: artifacts/<task>/training/<시각>_<label>/model_N.pt — 공식 play.py(창 없음, 1대)로 내보내고 스스로 끈다.
#   --quantize   동적 INT8(가중치) + fp32와 최대 차이 출력. 작은 정책은 이득이 거의 없어 비교용 — 로봇에는 fp32가 기본.
#   로봇에 올릴 때는 policy.onnx와 policy.onnx.data를 둘 다 (corrector · rl_nav_controller model_path = policy.onnx).
#
#   ./run-make-onnx.sh artifacts/rl_maze/training/<시각>_v1/model_499.pt
#   ./run-make-onnx.sh artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --quantize
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
PY="$ISAACLAB/.venv/bin/python"

if [[ $# -eq 0 || "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
args=()   # 경로는 지금 폴더 기준 → 절대경로로 (아래에서 isaac/로 cd)
for a in "$@"; do
  if [[ "$a" != --* && -e "$a" ]]; then args+=("$(realpath "$a")"); else args+=("$a"); fi
done
cd "$ISAAC"
exec "$PY" tools/onnx_tool.py "${args[@]}"
