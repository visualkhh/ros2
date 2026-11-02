#!/bin/bash
# calibration 실행기 — python -m calibration 을 런타임(IsaacLab / ROS / .venv)에 맞춰 돌린다.
#
#   ./run-calibration.sh <command> [옵션...]      (경로는 isaac/ 기준 상대경로 또는 절대경로)
#
# commands (sim은 IsaacLab, real·dummy·plot은 isaac/.venv):
#   random-world       랜덤 방 + 로봇 주행 (W/A/S/D, N 다음 방, C 보정값 창)
#     ./run-calibration.sh random-world [--seed 7] [--fit <real>/fit.json[,<sim>/fit.json]]
#   sensor-sync-sim    sim 센서 기록
#     ./run-calibration.sh sensor-sync-sim --viz none                         # raw plant (보정 없음)
#     ./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json[,<sim>/fit.json]
#     ./run-calibration.sh sensor-sync-sim --watch
#     ./run-calibration.sh sensor-sync-sim --viz none --scene path/to/MyScene.py   # 장면 교체 (random-world도 같음)
#     ./run-calibration.sh sensor-sync-sim --viz none --excitation path/to/MyPath.py   # 측정 경로 교체 (real·dummy도 같음)
#   sensor-sync-real   real 센서 기록 (PC, Pi base+MANUAL 필요, ROS/peers 자동)
#     ./run-calibration.sh sensor-sync-real --check-only
#     ./run-calibration.sh sensor-sync-real --label real_floor
#   plot-compare       sim/real CSV 비교 + fit (CSV 1개면 단건 리포트)
#     ./run-calibration.sh plot-compare <sim.csv> <real.csv> [--show]
#   calibrate          sim + real 같은 프로파일 기록 → 비교 → artifacts/calibration/<시각>-calibrate/
#     ./run-calibration.sh calibrate [--label real_floor] [--side 0.5] [--speed 0.15] [--turn 1.0] [--fit F] [--excitation E] [--show]
#     (sim은 --fit으로 준 보정만, 없으면 raw — 그대로 real,sim 차이 계산의 sim 기준이 된다)
#   <센서>             그 밖은 python -m calibration 으로 그대로 (pkg.mod.Class | path/to/file.py[:Class])
#     ./run-calibration.sh calibration/sensor/DummySensor.py --out artifacts/calibration/dummy_real/raw.csv --gain-l 0.96
#     ./run-calibration.sh calibration/sensor/DummySensor.py --fit artifacts/calibration/dummy_real/fit.json,artifacts/calibration/dummy_sim/fit.json
#
# common:
#   -h, --help, help   이 도움말
#   sim 명령은 --viz/--visualizer 없으면 --viz kit 자동 (창 표시)
#   ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
set -euo pipefail
ISAAC=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$ISAAC/.." && pwd)
ISAACLAB="${ISAACLAB_DIR:-/home/pank/pa-course/IsaacLab-6.1}"
PY="$ISAAC/.venv/bin/python"
[ -x "$PY" ] || PY=python3

usage() {
  sed -n '2,/^set -euo/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

# sim: 창 옵션이 없으면 --viz kit
sim() {
  local args=("$@")
  if [[ " ${args[*]} " != *" --viz "* && " ${args[*]} " != *" --visualizer "* ]]; then
    args+=(--viz kit)
  fi
  exec "$ISAACLAB/isaaclab.sh" -p "${args[@]}"
}

ros_env() {
  set +u   # ROS setup.bash는 unset 변수를 참조한다
  source /opt/ros/lyrical/setup.bash
  export ROS_DOMAIN_ID=25
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
  source "$REPO/tool/peers.sh"
  set -u
}

if [[ $# -eq 0 ]] || [[ "$1" == "-h" || "$1" == "--help" || "$1" == "help" ]]; then
  usage
  exit 0
fi

cd "$ISAAC"   # python -m calibration + 상대경로 기준
CMD="$1"; shift
case "$CMD" in
  random-world)
    sim calibration/world/DefaultWorld.py "$@"
    ;;
  sensor-sync-sim)
    sim -m calibration sensor-sync-sim "$@"
    ;;
  sensor-sync-real)
    ros_env
    exec "$PY" -m calibration sensor-sync-real "$@"
    ;;
  plot-compare)
    exec "$PY" -m calibration.report.report "$@"
    ;;
  calibrate)
    LABEL=real_floor SIDE=0.5 SPEED=0.15 TURN=1.0
    SIM_EXTRA=() SHOW=() COMMON_EXTRA=()
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --label) LABEL=${2:?--label 이름}; shift 2 ;;
        --side) SIDE=${2:?--side 값}; shift 2 ;;
        --speed) SPEED=${2:?--speed 값}; shift 2 ;;
        --turn) TURN=${2:?--turn 값}; shift 2 ;;
        --fit) SIM_EXTRA+=(--fit "${2:?--fit 경로}"); shift 2 ;;
        --excitation) COMMON_EXTRA+=(--excitation "${2:?--excitation 경로}"); shift 2 ;;
        --show) SHOW=(--show); shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "모르는 옵션: $1" >&2; exit 2 ;;
      esac
    done
    RUN="artifacts/calibration/$(date +%Y%m%d_%H%M%S)-calibrate"
    mkdir -p "$RUN"
    COMMON=(--side "$SIDE" --speed "$SPEED" --turn "$TURN" ${COMMON_EXTRA[@]+"${COMMON_EXTRA[@]}"})
    echo "[calibrate] sim + real 시작 → $RUN"
    "$0" sensor-sync-sim --viz none "${COMMON[@]}" ${SIM_EXTRA[@]+"${SIM_EXTRA[@]}"} \
      --out "$RUN/sim/raw.csv" > "$RUN/sim.log" 2>&1 &
    SIM_PID=$!
    set +e
    "$0" sensor-sync-real "${COMMON[@]}" --label "$LABEL" \
      --output "$RUN/real/raw.csv" > "$RUN/real.log" 2>&1
    REAL_RC=$?
    wait "$SIM_PID"; SIM_RC=$?
    set -e
    if [[ $REAL_RC -ne 0 || $SIM_RC -ne 0 ]]; then
      echo "[calibrate] 실패 sim=$SIM_RC real=$REAL_RC" >&2
      tail -30 "$RUN/sim.log" >&2 || true
      tail -30 "$RUN/real.log" >&2 || true
      exit 1
    fi
    "$0" plot-compare "$RUN/sim/raw.csv" "$RUN/real/raw.csv" \
      --out "$RUN/compare.png" --calib-out "$RUN/sim_calib.json" ${SHOW[@]+"${SHOW[@]}"} \
      | tee "$RUN/compare.log"
    echo "[calibrate] → $RUN  (sim/fit.json · real/fit.json · compare.png)"
    ;;
  *)
    exec "$PY" -m calibration "$CMD" "$@"
    ;;
esac
