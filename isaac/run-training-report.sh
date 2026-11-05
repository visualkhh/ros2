#!/bin/bash
# 학습 run → report.html — scalar plot · 요약표 · snapshot gallery · config (이미지 base64 embed).
#
#   ./run-training-report.sh artifacts/<task>/training/<시각>_<label>
#   ./run-training-report.sh <task>          # artifacts/<task>/training/의 가장 최근 run
#   ./run-training-report.sh <task> --max-snaps 40
#
#   → <run>/report.html  (브라우저로 열면 됨 — 파일 하나로 self-contained)
#
# 학습 중에도 만들 수 있음 (마지막으로 기록된 iteration까지). 다시 만들면 갱신됨.
set -euo pipefail

ISAAC=$(cd "$(dirname "$0")" && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
PY="$ISAACLAB/.venv/bin/python"
[ -x "$PY" ] || PY=python3

if [[ $# -eq 0 || "${1:-}" == "-h" || "${1:-}" == "--help" || "${1:-}" == "help" ]]; then
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
  exit 0
fi

exec "$PY" "$ISAAC/tools/train_report.py" "$@"
