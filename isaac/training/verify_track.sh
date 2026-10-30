#!/bin/bash
# track RL 검증 — 보정 때와 같은 (v,w) 명령 재생 + before/after 비교를 한 번에.
#
#   ./training/verify_track.sh [options]
#
# options:
#   --profile PATH   02가 보낼 profile (.py 또는 디렉토리)
#                    기본: training/rl_correction_wheel_drift/calib_profile.py
#   --label NAME     real CSV label (기본 real_track_rl)
#   --before PATH    비교 전 CSV (기본 training/artifacts/real_floor.csv)
#   --after PATH     비교 후 CSV (기본 training/artifacts/<label>.csv)
#   --out PATH       비교 PNG (기본 training/artifacts/compare_track_rl.png)
#   --show           그래프 창 열기
#   -h, --help
#
# 사전:
#   - PC: `training/real_rl.sh <track.onnx> track` 실행 중
#   - Pi: `./run-cli route rl`, state MANUAL
#   - 보정 결과 `training/artifacts/real_floor.csv` 존재
#
# 결과 보고서: training/artifacts/verify_track_<시각>/
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
CALIB="$HERE/calibration"
PROFILE="$HERE/rl_correction_wheel_drift/calib_profile.py"
LABEL=real_track_rl
BEFORE="$HERE/artifacts/real_floor.csv"
AFTER=""
OUT="$HERE/artifacts/compare_track_rl.png"
SHOW=0

usage() { sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }
while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile) PROFILE=${2:?--profile 경로}; shift 2 ;;
    --label) LABEL=${2:?--label 이름}; shift 2 ;;
    --before) BEFORE=${2:?--before 경로}; shift 2 ;;
    --after) AFTER=${2:?--after 경로}; shift 2 ;;
    --out) OUT=${2:?--out 경로}; shift 2 ;;
    --show) SHOW=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "모르는 옵션: $1 (--help)" >&2; exit 2 ;;
  esac
done

[[ "$PROFILE" != /* ]] && PROFILE="$PWD/$PROFILE"
[[ -d "$PROFILE" ]] && PROFILE="$PROFILE/calib_profile.py"
[[ -f "$PROFILE" ]] || { echo "profile 없음: $PROFILE" >&2; exit 2; }
PROFILE=$(realpath "$PROFILE")
[[ -f "$BEFORE" ]] || { echo "before CSV 없음: $BEFORE (먼저 calibrate.sh)" >&2; exit 2; }
[[ -n "$AFTER" ]] || AFTER="$HERE/artifacts/${LABEL}.csv"

STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$HERE/artifacts/verify_track_${STAMP}"
mkdir -p "$RUN"
{
  echo "date: $(date -Is)"
  echo "profile: $PROFILE"
  echo "label: $LABEL"
  echo "before: $BEFORE"
  echo "after: $AFTER"
  echo "out: $OUT"
} > "$RUN/command.txt"

echo "[verify] Pi는 route rl / MANUAL, PC는 real_rl track 이어야 함"
echo "[verify] replay: $PROFILE"
"$CALIB/02_sensor_sync_real.sh" --label "$LABEL" --profile "$PROFILE" \
  2>&1 | tee "$RUN/replay.log"

[[ -f "$AFTER" ]] || { echo "after CSV 없음: $AFTER" >&2; exit 2; }

PLOT_ARGS=()
[[ $SHOW -eq 1 ]] && PLOT_ARGS+=(--show)
"$CALIB/plot_compare.sh" "$BEFORE" "$AFTER" \
  --out "$OUT" --calib-out "$RUN/sim_calib_track_rl.json" "${PLOT_ARGS[@]}" \
  2>&1 | tee "$RUN/compare.log"

cp "$AFTER" "$RUN/"
cp "$OUT" "$RUN/" 2>/dev/null || true
echo "[verify] report → $RUN"
