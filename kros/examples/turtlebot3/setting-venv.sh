#!/usr/bin/env bash
# kros/examples/turtlebot3/setting-venv.sh — kros 루트의 setting-venv.sh를 그대로 쓴다 (IDE에서 venv를 준비할 때 쓴다).
here="$(cd "$(dirname "$0")/../.." && pwd)"   # kros 루트
exec "$here/setting-venv.sh" "$@"
