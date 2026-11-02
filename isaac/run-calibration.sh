#!/bin/bash
# calibration 실행기 — python -m calibration 을 런타임(IsaacLab / ROS / .venv)에 맞춰 돌린다.
#
#   ./run-calibration.sh <command> [옵션...]      (경로는 isaac/ 기준 상대경로 또는 절대경로)
#   ./run-calibration.sh <command> --help        그 명령의 옵션 전체 (python argparse)
#
# 무엇을 하나: 같은 명령(기본: 사각형 → 180° 턴 → 역주행, 54 s)을 real과 sim에 넣고 응답을 재서
#   fit.json(바퀴 gain 좌/우 · 지연 · τ · 데드밴드)으로 만든다. real − sim 차이를 sim 입력에 넣으면(--fit)
#   sim이 real처럼 움직이고, 그 차이로 학습한다(run-training.sh --fit). 자세히: README.md 3장.
#
# 결과 폴더: artifacts/calibration/<시각>-<label>/  →  raw.csv · fit.json · report.png (--fit이면 base-fit.json)
#   raw.csv  = 20 Hz 기록 (명령 · 보정 출력 · odom · 엔코더 · 자이로 · 바퀴 rad/s · 메시지 시각)
#   fit.json = 응답 fit (gain 좌/우 · 지연 · τ · 데드밴드 · 구간별 speed_fit) · base-fit.json = 실제로 sim 입력에 넣은 보정값
#              (base-fit.json은 같은 {source, fit} 모양이라 다시 --fit에 넣을 수 있다)
#
# ─── commands ──────────────────────────────────────────────────────────────────
#
#   sensor-sync-sim    sim에서 측정 (IsaacLab)
#     --fit F[,G]        입력 보정. F 1개 = 그 fit 그대로 · F,G 2개(쉼표, 공백 없이, real 먼저) = real − sim 차이
#                        (gain 비율 · 지연/τ 차 · 데드밴드 = real 문턱 ÷ sim gain). 없으면 raw plant (숨은 보정 파일 안 읽음)
#     --policy ONNX      학습한 정책으로 보정하며 달림 = sim play. ros2 corrector와 같은 규칙
#                        (관측 6개 → 보정량 더하고 ±0.26 m/s · ±1.82 rad/s clip · 정지 명령엔 0). 기록 lin/ang_out = 보정 출력
#     --label NAME       폴더 이름 <시각>-NAME · CSV label (기본 02-sim / sim_sensor_floor)
#     --out PATH         정확한 raw.csv 경로 (label보다 우선)
#     --side 0.5 --speed 0.15 --turn 1.0    사각형 한 변 [m] · 기준 전진 [m/s] (변마다 1.0/0.67/1.33/0.8배) · 회전 [rad/s]
#     --room 0           방 크기 [m] (0 = 장애물 없는 빈 바닥, 측정은 0 권장) · --seed 0
#     --scene CLS        장면 교체 — calibration.scene.Scene 서브클래스 (보통 DefaultScene 상속), 생성자 (condition=, actuator=)
#     --excitation CLS   측정 경로 교체 — CalibExcitation 서브클래스 (보통 DefaultCalibExcitation 상속해 make_profile만)
#                        CLS 형식: path/to/file.py · path/to/file.py:Class · pkg.mod.Class (파일에 서브클래스 1개면 이름 생략)
#     --watch            기록 후 창을 닫을 때까지 같은 경로 반복 재생 (창 필요)
#     --viz none|kit     창 끔/켬. 안 주면 kit (창 표시) — 기록만 할 땐 --viz none
#     창에서 보이는 것: 초록 막대 = 명령 방향 · 파랑 막대 = 실제 방향 (길이 ∝ 속도) · 노란 점 = 목표 경로(사각형)
#                       로봇이 노란 점에서 벗어나는 만큼이 드리프트
#
#     ./run-calibration.sh sensor-sync-sim --viz none --label sim                                   # raw 측정 (3.2)
#     ./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json,<sim>/fit.json --label check   # 맞춰졌나 (3.3)
#     ./run-calibration.sh sensor-sync-sim --viz none --fit <real>,<sim> --label A                    # sim A/B — A (보정 없음)
#     ./run-calibration.sh sensor-sync-sim --watch --fit <real>,<sim> --policy artifacts/real/policy.onnx --label B   # B (정책)
#     ../ros2_ws/report.py artifacts/calibration/<A>/raw.csv artifacts/calibration/<B>/raw.csv       # A/B 그림
#
#   sensor-sync-real   실기기에서 측정 (PC · .venv · ROS 환경/peers 자동, ROS_DOMAIN_ID=25)
#     준비: 로봇 베이스 켜기 · MANUAL · 경로 direct (rl-only 안 띄움) · OpenCR는 가만히 둔 채 켜기 · 주변 1 m 비우기
#     --check-only       움직이지 않고 상태만 확인 (state=MANUAL · odom 수신)
#     --label NAME · --output PATH · --side --speed --turn (sim과 같은 기본값 → 같은 명령) · --rate 20 · --excitation CLS
#     ./run-calibration.sh sensor-sync-real --check-only
#     ./run-calibration.sh sensor-sync-real --label real_floor
#
#   calibrate          sim(raw) + real을 같은 경로로 한 번에 기록 → 비교 그림
#     --label real_floor --side --speed --turn --fit F --excitation CLS --show
#     → artifacts/calibration/<시각>-calibrate/{sim,real}/{raw.csv,fit.json,report.png} · compare.png · sim_calib.json
#     ./run-calibration.sh calibrate --side 0.5
#
#   plot-compare       CSV 그림. 1개 = 단건 리포트(report.png) · 2개(sim real) = 비교 그림 + fit
#     --out PNG · --show(창) · --calib-out JSON
#     ./run-calibration.sh plot-compare artifacts/calibration/<sim>/raw.csv artifacts/calibration/<real>/raw.csv
#
#   random-world       랜덤 방 + 로봇 키보드 주행 (IsaacLab, 창)
#     W/A/S/D 주행 · Shift 느리게 · X 정지 · R 리셋 · N 다음 방 · C 보정값 창
#     --fit F[,G] · --scene CLS · --seed 7 · --rooms 8 · --size-min 3.5 --size-max 6.0 · --speed --turn · --seconds 0(창 닫을 때까지)
#
#   <센서 파일>        그 밖은 python -m calibration 으로 그대로 (Sensor 서브클래스 · main(argv) 필요)
#     ./run-calibration.sh calibration/sensor/DummySensor.py --gain-l 0.96 --gain-r 1.02 --out artifacts/calibration/dummy_real/raw.csv
#     ./run-calibration.sh calibration/sensor/DummySensor.py --fit artifacts/calibration/dummy_real/fit.json,artifacts/calibration/dummy_sim/fit.json
#     (DummySensor = Isaac/ROS 없이 흐름 확인용 가짜 로봇: --gain/--gain-l/--gain-r --deadband --slip --noise --seed)
#
# common:
#   -h, --help, help   이 도움말
#   ISAACLAB_DIR 환경변수로 IsaacLab 경로 변경 (기본 /home/pank/pa-course/IsaacLab-6.1)
#   어느 폴더에서 실행해도 isaac/로 이동해서 돈다 (경로 인자는 isaac/ 기준)
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
