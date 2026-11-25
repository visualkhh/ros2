#!/usr/bin/env bash
# 실기 Pi에서 실행 중인 krosc_real을 정지한다.
#
#   ./stop.sh
#
# SLAM/주행/팔 토크를 포함한 프로세스를 정상 종료(SIGINT)하고,
# 5초 안에 끝나지 않으면 강제 종료한다.
set -euo pipefail

pids=$(pgrep -x krosc_real || true)
if [ -z "$pids" ]; then
  echo "[stop] krosc_real 실행 중 아님"
  exit 0
fi

echo "[stop] krosc_real 종료 요청: $pids"
# shellcheck disable=SC2086
kill -INT $pids

for _ in $(seq 1 50); do
  if ! pgrep -x krosc_real >/dev/null 2>&1; then
    echo "[stop] 정지 완료"
    exit 0
  fi
  sleep 0.1
done

echo "[stop] 5초 내 종료 안 됨 → 강제 종료"
pkill -KILL -x krosc_real || true
sleep 0.5
if pgrep -x krosc_real >/dev/null 2>&1; then
  echo "[stop] FAIL: 아직 실행 중"
  exit 1
fi
echo "[stop] 정지 완료 (강제)"
