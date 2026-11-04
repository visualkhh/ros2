#!/bin/bash
# 실기기 A/B — 같은 명령(task의 calibration 명령 = 네모)을 A(보정 없음) · B(정책 보정)로 한 번씩 달리고 비교.
#
#   ./run-real-ab.sh <task> --onnx <policy.onnx> [--label 이름] [--speed 0.15 --turn 1.0 --side 0.5 …]
#     → artifacts/<task>/ab/<시각>_<label>/{A,B}/raw.csv · compare.png (training/<task>/report.py)
#
# 준비 (calibration real과 같음 — training/<task>/README.md):
#   Pi: 베이스 켜기 · MANUAL · route rl   PC: ros2_ws/rl-only.sh (corrector — A/B는 RealRobot이 toggle · model_path를 직접 바꾼다)
# 순서: A (corrector 바이패스) → "로봇을 출발 자리 · 방향으로" Enter → B (--onnx 정책) → 보고서.
# 같은 배터리 · 바닥에서 연달아 — A와 B는 시작을 (0, 0, 0)으로 맞춰 겹쳐 그린다.
# 주의: goal 모드(웨이포인트) 정책은 속도 입력(네모)을 보정하지 않는다 — B 보정량이 0이면 보고서가 경고한다.
#
#   ./run-real-ab.sh rl_correction_waypoint --onnx artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
PY="$ISAAC/.venv/bin/python"
[ -x "$PY" ] || PY=python3

if [[ $# -eq 0 || "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi
TASK="$1"; shift
[[ -f "$ISAAC/training/$TASK/report.py" ]] || { echo "A/B 보고서 없음: training/$TASK/report.py" >&2; exit 2; }
ONNX="" LABEL="ab" REST=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --onnx=*) ONNX="${1#--onnx=}"; shift ;;
    --onnx) ONNX="${2:?--onnx 경로}"; shift 2 ;;
    --label=*) LABEL="${1#--label=}"; shift ;;
    --label) LABEL="${2:?--label 이름}"; shift 2 ;;
    *) REST+=("$1"); shift ;;
  esac
done
[[ -n "$ONNX" ]] || { echo "--onnx <policy.onnx> 필요 (B 정책)" >&2; exit 2; }
ONNX=$(realpath "$ONNX")
[[ -f "$ONNX" ]] || { echo "ONNX 없음: $ONNX" >&2; exit 2; }
RUN="$ISAAC/artifacts/$TASK/ab/$(date +%Y-%m-%d_%H-%M-%S)_$LABEL"
mkdir -p "$RUN"
cd "$ISAAC"
echo "[ab] $RUN · B 정책 $ONNX"

echo "[ab] A — 보정 없음 (corrector 바이패스)"
CALIB_NO_REPORT=1 ./run-calibration.sh "$TASK" --robot real --label A --out "$RUN/A/raw.csv" ${REST[@]+"${REST[@]}"}
read -r -p "[ab] 로봇을 A의 출발 자리 · 방향으로 돌려놓고 Enter (취소 Ctrl+C) " _
echo "[ab] B — 정책 보정"
CALIB_NO_REPORT=1 ./run-calibration.sh "$TASK" --robot real --label B --onnx "$ONNX" --out "$RUN/B/raw.csv" ${REST[@]+"${REST[@]}"}

"$PY" "training/$TASK/report.py" "$RUN/A" "$RUN/B" 2>&1 | grep -v "UserWarning\|warnings.warn" || true
echo "[ab] → $RUN"
