#!/usr/bin/env bash
# 실기용 venv: kros의 core 프로필(의존성 없음)을 쓴다. 라즈베리파이에서 최초 1회.
#   ./kos_ws/setting-venv.sh
# 실행은 ./kos_ws/run.sh (kos_ws/.venv가 없으면 kros/.venv를 쓴다).
set -e
here="$(cd "$(dirname "$0")" && pwd)"
exec "$here/../kros/setting-venv.sh" core
