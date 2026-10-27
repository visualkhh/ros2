#!/bin/bash
# OpenCR udev 규칙 설치 — /dev/opencr 고정 이름 + 누구나 읽기/쓰기(0666) (최초 1회, 로봇 Pi·PC 둘 다)
#
#   ./install-udev.sh            # 설치 + 지금 꽂힌 OpenCR에 바로 적용
#   ./install-udev.sh --remove   # 제거
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

RULE=99-opencr.rules
DST=/etc/udev/rules.d/$RULE

if [ "${1:-}" = --remove ]; then
  $SUDO rm -f "$DST"
  $SUDO udevadm control --reload
  echo "OK: 제거 ($DST)"
  exit 0
fi

$SUDO install -m 644 "$HERE/$RULE" "$DST"
$SUDO udevadm control --reload
$SUDO udevadm trigger --subsystem-match=tty
sleep 1
echo "OK: 설치 ($DST)"
if [ -e /dev/opencr ]; then
  ls -l /dev/opencr
else
  echo "/dev/opencr 아직 없음 — OpenCR USB를 꽂으면 생깁니다 (ls -l /dev/ttyACM*)"
fi
