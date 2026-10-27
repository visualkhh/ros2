#!/bin/bash
# udev 규칙 설치 — /dev/opencr (OpenCR) · /dev/tb3_lidar (LDS-03) 고정 이름 + 0666 (최초 1회, 로봇 Pi·PC 둘 다)
#
#   ./install-udev.sh            # 설치 + 지금 꽂힌 OpenCR에 바로 적용
#   ./install-udev.sh --remove   # 제거
set -euo pipefail
source "$(cd "$(dirname "$0")" && pwd)/common.sh"

RULES=(99-opencr.rules 99-tb3-lidar.rules)

if [ "${1:-}" = --remove ]; then
  for r in "${RULES[@]}"; do $SUDO rm -f "/etc/udev/rules.d/$r"; done
  $SUDO udevadm control --reload
  echo "OK: 제거 (${RULES[*]})"
  exit 0
fi

for r in "${RULES[@]}"; do
  $SUDO install -m 644 "$HERE/$r" "/etc/udev/rules.d/$r"
done
$SUDO udevadm control --reload
$SUDO udevadm trigger --subsystem-match=tty
sleep 1
echo "OK: 설치 (${RULES[*]})"
for dev in /dev/opencr /dev/tb3_lidar; do
  if [ -e "$dev" ]; then ls -l "$dev"; else echo "$dev 아직 없음 — 장치를 꽂으면 생깁니다"; fi
done
