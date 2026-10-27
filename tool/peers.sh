# tool/peers.sh — cli/start.sh · rviz/start.sh가 source. PC에서 로봇 Pi를 유니캐스트로 직접 찾게 ROS_STATIC_PEERS를 정한다.
# 사내 WiFi는 멀티캐스트(DDS 자동 탐색)를 자주 버려서 노드가 보였다 안 보였다 함 (2026-10-27 실측). 한쪽만 지정해도 양방향.
# 이름(pak.local)을 그대로 넘기면 DDS가 시작 순간 mDNS 조회에 실패할 때 노드 생성이 죽음
# ("Unable to resolve peer pak.local") → 여기서 IP로 바꿔 넘긴다: 재시도 → 마지막 성공 IP(.run/robot_ip.<이름>) → 없으면 경고만.
# 로봇 이름이 다르면 ROBOT_HOST=다른이름.local. Pi 자신에서 실행하면 아무것도 안 함.
_robot="${ROBOT_HOST:-pak.local}"
_cache="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.run/robot_ip.${_robot}"
if [ -z "${ROS_STATIC_PEERS:-}" ] && [ "$(hostname)" != "${_robot%.local}" ]; then
  _ip=""
  for _ in 1 2 3; do
    _ip="$(getent ahostsv4 "$_robot" 2>/dev/null | awk 'NR==1{print $1}' || true)"  # start.sh는 set -e·pipefail
    [ -n "$_ip" ] && break
    sleep 1
  done
  if [ -n "$_ip" ]; then
    mkdir -p "$(dirname "$_cache")" && echo "$_ip" > "$_cache"
  elif [ -s "$_cache" ]; then
    _ip="$(cat "$_cache")"
    echo "WARN: $_robot 이름 조회 실패 → 마지막으로 찾은 IP $_ip 사용" >&2
  fi
  if [ -n "$_ip" ]; then
    export ROS_STATIC_PEERS="$_ip"
    echo "ROS_STATIC_PEERS=$ROS_STATIC_PEERS ($_robot)"
  else
    echo "WARN: $_robot 을 찾을 수 없음 — 멀티캐스트 탐색만 사용 (Pi 꺼짐? 다른 WiFi?)" >&2
  fi
fi
unset _robot _cache _ip
