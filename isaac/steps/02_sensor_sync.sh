#!/bin/bash
# 02 — 센서 싱크 (sim). 공중: --stand --watch / 바닥: --viz none
set -e
HERE=$(cd "$(dirname "$0")" && pwd)
exec "$HERE/run.sh" 02_sensor_sync.py "$@"
