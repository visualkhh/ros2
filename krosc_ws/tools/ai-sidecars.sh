#!/usr/bin/env bash
# krosc AI sidecar(LLM gateway 8767 / VLM 8766 / VLA 8765)를 PC에서 실행/정지한다.
# 로봇(Pi)은 Ollama 11434가 아니라 LLM gateway 8767에 붙는다.
# 로봇(Pi)은 이 PC의 LAN IP로 접속하므로 기본 바인딩은 0.0.0.0이다.
#
#   ./krosc_ws/tools/ai-sidecars.sh start
#   ./krosc_ws/tools/ai-sidecars.sh status
#   ./krosc_ws/tools/ai-sidecars.sh stop
#
# 환경변수:
#   KROSC_AI_BIND=0.0.0.0   바인딩 주소 (로봇이 붙으려면 0.0.0.0)
#   KROSC_AI_THREADS=8      torch CPU 스레드 수
#   KROSC_VLM_DEVICE=cpu    vlm_server device (cuda는 CUDA torch 설치 시)
set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
venv="${KROSC_AI_VENV:-$here/vla-venv}"
bind="${KROSC_AI_BIND:-0.0.0.0}"
threads="${KROSC_AI_THREADS:-8}"
device="${KROSC_VLM_DEVICE:-cpu}"
log_dir="${KROSC_AI_LOG_DIR:-$root/krosc_ws/artifacts/logs}"
run_dir="${KROSC_AI_RUN_DIR:-$root/krosc_ws/artifacts/run}"
mkdir -p "$log_dir" "$run_dir"

usage() {
  sed -n '2,/^set -/p' "$0" | sed '$d' | sed 's/^# \{0,1\}//'
}

ensure_venv() {
  if [ ! -x "$venv/bin/python" ]; then
    echo "[ai-sidecars] venv 생성: $venv"
    python3 -m venv --system-site-packages "$venv"
  fi
  if ! "$venv/bin/python" -c 'import torch, torchvision, transformers, num2words, PIL, numpy' 2>/dev/null; then
    echo "[ai-sidecars] 의존성 설치: transformers==5.10.4 num2words"
    "$venv/bin/python" -m pip install -q --upgrade pip
    "$venv/bin/python" -m pip install -q 'transformers==5.10.4' num2words
  fi
}

ensure_cache() {
  # vlm_server/vla_server는 artifacts 아래 HF_HOME을 기본으로 쓴다.
  # 이미 받아둔 ~/.cache/huggingface가 있으면 symlink로 재사용한다.
  local kind="$1"
  local cache="$root/krosc_ws/artifacts/models/$kind/hf-cache"
  if [ ! -e "$cache" ] && [ -d "$HOME/.cache/huggingface" ]; then
    mkdir -p "$(dirname "$cache")"
    ln -s "$HOME/.cache/huggingface" "$cache"
  fi
}

health() {
  local port="$1"
  curl -fsS --max-time 2 "http://127.0.0.1:$port/health" >/dev/null 2>&1
}

start_one() {
  local name="$1" script="$2" port="$3"
  local pid_file="$run_dir/$name.pid"
  local log="$log_dir/$name.log"
  if health "$port"; then
    echo "[ai-sidecars] $name 이미 실행 중 (:$port)"
    return 0
  fi
  if [ -f "$pid_file" ] && kill -0 "$(cat "$pid_file")" 2>/dev/null; then
    echo "[ai-sidecars] $name PID $(cat "$pid_file") 재사용"
    return 0
  fi
  local extra=()
  if [ "$name" = "vlm" ]; then
    extra=(--device "$device")
  fi
  echo "[ai-sidecars] $name 시작 (:$port, log=$log)"
  setsid "$venv/bin/python" "$here/$script" --host "$bind" --port "$port" \
    --threads "$threads" "${extra[@]}" >"$log" 2>&1 </dev/null &
  echo $! >"$pid_file"
}

wait_ready() {
  local name="$1" port="$2"
  for _ in $(seq 1 90); do
    if health "$port"; then
      echo "[ai-sidecars] $name ready (:$port)"
      return 0
    fi
    sleep 1
  done
  echo "[ai-sidecars] $name FAIL (: $port) — $log_dir/$name.log 확인" >&2
  return 1
}

stop_one() {
  local name="$1"
  local pid_file="$run_dir/$name.pid"
  if [ ! -f "$pid_file" ]; then
    echo "[ai-sidecars] $name PID 파일 없음"
    return 0
  fi
  local pid
  pid="$(cat "$pid_file")"
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT "$pid" 2>/dev/null || true
    for _ in $(seq 1 30); do
      kill -0 "$pid" 2>/dev/null || break
      sleep 0.2
    done
    kill -KILL "$pid" 2>/dev/null || true
    echo "[ai-sidecars] $name 정지 (PID $pid)"
  else
    echo "[ai-sidecars] $name 이미 정지"
  fi
  rm -f "$pid_file"
}

lan_ip() {
  hostname -I 2>/dev/null | awk '{print $1}'
}

case "${1:-start}" in
  start)
    ensure_venv
    ensure_cache vlm
    ensure_cache vla
    start_one llm llm_server.py 8767
    start_one vlm vlm_server.py 8766
    start_one vla vla_server.py 8765
    wait_ready llm 8767
    wait_ready vlm 8766
    wait_ready vla 8765
    ip="$(lan_ip)"
    echo
    echo "로봇(Pi)에서 PC sidecar를 보게 하려면:"
    echo "  export KROSC_OLLAMA_URL=http://${ip:-<PC-IP>}:8767"
    echo "  export KROSC_VLM_URL=http://${ip:-<PC-IP>}:8766/query"
    echo "  export KROSC_VLA_URL=http://${ip:-<PC-IP>}:8765/plan"
    echo "방화벽(UFW)에서 8765/8766/8767이 막혀 있으면:"
    echo "  sudo ufw allow from 10.2.150.0/23 to any port 8765 proto tcp"
    echo "  sudo ufw allow from 10.2.150.0/23 to any port 8766 proto tcp"
    echo "  sudo ufw allow from 10.2.150.0/23 to any port 8767 proto tcp"
    ;;
  stop)
    stop_one llm
    stop_one vlm
    stop_one vla
    ;;
  status)
    for item in "llm 8767" "vlm 8766" "vla 8765"; do
      set -- $item
      if health "$2"; then
        echo "$1: ready (:$2)"
      else
        echo "$1: down (:$2)"
      fi
    done
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "usage: $0 [start|stop|status]" >&2
    exit 2
    ;;
esac
