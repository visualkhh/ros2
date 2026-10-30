#!/bin/bash
# sim/real CSV 파형 비교. --show로 창, 없으면 artifacts/compare.png 저장
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
exec python3 "$HERE/plot_compare.py" "$@"
