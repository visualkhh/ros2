#!/bin/bash
# isaac real-side 공용 venv (안쪽 폴더에는 venv 없음)
#   ./setup_venv.sh
# → isaac/.venv (system-site-packages, ROS lyrical 상속) + requirements.txt 설치
# sim/학습/play/export는 isaaclab.sh venv 그대로 (여기 아님)
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
VENV="$HERE/.venv"
PY=${PYTHON:-python3}

if [ ! -x "$VENV/bin/python" ]; then
  echo "[setup_venv] create $VENV (--system-site-packages: ROS rclpy 상속용)"
  "$PY" -m venv --system-site-packages "$VENV"
fi
"$VENV/bin/pip" install --upgrade pip
"$VENV/bin/pip" install -r "$HERE/requirements.txt"
echo "[setup_venv] done → $VENV/bin/python"
echo "[setup_venv] real-side .sh들은 .venv 있으면 자동 사용, 없으면 python3"
