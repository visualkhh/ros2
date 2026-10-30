#!/bin/bash
# 01 — 랜덤 환경 + 우리 로봇
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
exec "$HERE/run.sh" 01_random_world_sim.py "$@"
