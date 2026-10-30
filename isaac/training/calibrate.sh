#!/bin/bash
# calibration sim + real 동시 실행 → 비교 → training/artifacts/<run>/ 보고서
#
#   ./training/calibrate.sh [options]
#
# options:
#   --profile PATH   보정 profile .py 경로 (또는 그 디렉토리)
#                    기본: training/calibration/calib_default_profile.py
#   --label NAME     real CSV 이름/조건 (기본 floor; --stand면 기본 stand)
#   --stand          공중(지지 박스) 실험
#   --no-calib       sim에 이전 보정 JSON 무시 (raw)
#   --side M         사각형 한 변 (기본 0.5)
#   --speed M/S      기준 전진 속도 (기본 0.15)
#   --turn RAD/S     회전 속도 (기본 1.0)
#   --show           끝나고 비교 그래프 창 열기
#   -h, --help
#
# 사전:
#   - Pi: base + MANUAL
#   - PC: ROS/peers (02_sensor_sync_real.sh가 자동 source)
#   - training/calibration/artifacts에 sim/real CSV가 생성됨
#   - training/artifacts/<run>/에 로그·CSV·PNG·JSON 복사본이 쌓임
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
ISAAC=$(cd "$HERE/.." && pwd)
CALIB="$HERE/calibration"
PROFILE="$CALIB/calib_default_profile.py"
LABEL=""
STAND=0
NOCAL=0
SHOW=0
SIDE=0.5
SPEED=0.15
TURN=1.0

usage() { sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'; }

while [[ $# -gt 0 ]]; do
  case "$1" in
    --profile) PROFILE=${2:?--profile 경로}; shift 2 ;;
    --label) LABEL=${2:?--label 이름}; shift 2 ;;
    --stand) STAND=1; shift ;;
    --no-calib) NOCAL=1; shift ;;
    --show) SHOW=1; shift ;;
    --side) SIDE=${2:?--side 값}; shift 2 ;;
    --speed) SPEED=${2:?--speed 값}; shift 2 ;;
    --turn) TURN=${2:?--turn 값}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "모르는 옵션: $1 (--help)" >&2; exit 2 ;;
  esac
done

[[ "$PROFILE" != /* ]] && PROFILE="$PWD/$PROFILE"
if [[ -d "$PROFILE" ]]; then PROFILE="$PROFILE/calib_profile.py"; fi
[[ -f "$PROFILE" ]] || { echo "profile 없음: $PROFILE" >&2; exit 2; }
PROFILE=$(realpath "$PROFILE")

if [[ $STAND -eq 1 ]]; then
  TAG="stand"
  LABEL=${LABEL:-stand}
  REAL_ARGS=(--label "real_$LABEL")
  SIM_ARGS=(--stand)
else
  TAG="floor"
  LABEL=${LABEL:-floor}
  REAL_ARGS=(--label "real_$LABEL")
  SIM_ARGS=()
fi
[[ $NOCAL -eq 1 ]] && SIM_ARGS+=(--no-calib)

STAMP=$(date +%Y%m%d_%H%M%S)
RUN="$HERE/artifacts/calibrate_${STAMP}_${LABEL}"
mkdir -p "$RUN"
{
  echo "date: $(date -Is)"
  echo "profile: $PROFILE"
  echo "label: $LABEL"
  echo "stand: $STAND no_calib: $NOCAL side: $SIDE speed: $SPEED turn: $TURN"
  echo "sim:  $CALIB/02_sensor_sync_sim.sh --viz none --profile $PROFILE --side $SIDE --speed $SPEED --turn $TURN ${SIM_ARGS[*]}"
  echo "real: $CALIB/02_sensor_sync_real.sh --profile $PROFILE --side $SIDE --speed $SPEED --turn $TURN ${REAL_ARGS[*]}"
} > "$RUN/command.txt"

echo "[calibrate] profile: $PROFILE"
echo "[calibrate] sim + real 동시 시작 → $RUN"
"$CALIB/02_sensor_sync_sim.sh" --viz none --profile "$PROFILE" \
  --side "$SIDE" --speed "$SPEED" --turn "$TURN" "${SIM_ARGS[@]}" \
  > "$RUN/sim.log" 2>&1 &
SIM_PID=$!

set +e
"$CALIB/02_sensor_sync_real.sh" --profile "$PROFILE" \
  --side "$SIDE" --speed "$SPEED" --turn "$TURN" "${REAL_ARGS[@]}" \
  > "$RUN/real.log" 2>&1
REAL_RC=$?
wait "$SIM_PID"; SIM_RC=$?
set -e

if [[ $REAL_RC -ne 0 || $SIM_RC -ne 0 ]]; then
  echo "[calibrate] 실패 sim=$SIM_RC real=$REAL_RC" >&2
  tail -30 "$RUN/sim.log" >&2 || true
  tail -30 "$RUN/real.log" >&2 || true
  exit 1
fi

SIM_CSV="$CALIB/artifacts/sim_sensor_${TAG}.csv"
REAL_CSV="$CALIB/artifacts/real_${LABEL}.csv"
[[ -f "$SIM_CSV" && -f "$REAL_CSV" ]] || { echo "CSV 없음" >&2; exit 1; }

PLOT_ARGS=()
[[ $SHOW -eq 1 ]] && PLOT_ARGS+=(--show)
"$CALIB/plot_compare.sh" "$SIM_CSV" "$REAL_CSV" "${PLOT_ARGS[@]}" | tee "$RUN/plot.log"

cp "$SIM_CSV" "$RUN/"
cp "$REAL_CSV" "$RUN/"
cp "$CALIB/artifacts/compare_${TAG}.png" "$RUN/" 2>/dev/null || true
cp "$CALIB/artifacts/sim_calib_${TAG}.json" "$RUN/" 2>/dev/null || true
echo "[calibrate] report → $RUN"
