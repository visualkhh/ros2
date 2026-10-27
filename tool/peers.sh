# tool/peers.sh — cli/start.sh · rviz/start.sh가 source. PC에서 로봇 Pi의 DDS 노드를 유니캐스트로 직접 찾게 한다.
#
# 왜: 사내 WiFi는 멀티캐스트(DDS 자동 탐색)를 자주 버려서 노드가 보였다 안 보였다 함 (2026-10-27 실측).
# 처음엔 ROS_STATIC_PEERS=pak.local을 썼는데 두 가지가 터짐:
#   1) 시작 순간 mDNS 조회 실패 → "Unable to resolve peer pak.local"로 노드 생성이 죽음 → 여기서 IP로 바꿔 넘김
#   2) STATIC_PEERS는 상대 컴퓨터의 참가자 슬롯 0~3번만 찾아감 (Fast DDS maxInitialPeersRange 기본 4).
#      Pi에는 드라이버·라이다·필터·SLAM·Nav… 참가자가 8개 넘게 떠서, 재시작해 뒤 슬롯으로 밀린 노드는 안 보임
#      (텔레오퍼가 안 먹고 RViz에 지도가 안 뜸) → Fast DDS 프로필로 슬롯 0~31번까지 찾게 한다.
# IP: 이름 조회 재시도 → 마지막 성공 IP(.run/robot_ip.<이름>) → 없으면 경고 후 멀티캐스트만.
# 로봇 이름이 다르면 ROBOT_HOST=다른이름.local. Pi 자신에서 실행하면 아무것도 안 함.
_robot="${ROBOT_HOST:-pak.local}"
_run="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/.run"
_cache="$_run/robot_ip.${_robot}"
if [ -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ] && [ "$(hostname)" != "${_robot%.local}" ]; then
  _ip=""
  for _ in 1 2 3; do
    _ip="$(getent ahostsv4 "$_robot" 2>/dev/null | awk 'NR==1{print $1}' || true)"  # start.sh는 set -e·pipefail
    [ -n "$_ip" ] && break
    sleep 1
  done
  mkdir -p "$_run"
  if [ -n "$_ip" ]; then
    echo "$_ip" > "$_cache"
  elif [ -s "$_cache" ]; then
    _ip="$(cat "$_cache")"
    echo "WARN: $_robot 이름 조회 실패 → 마지막으로 찾은 IP $_ip 사용" >&2
  fi
  if [ -n "$_ip" ]; then
    cat > "$_run/fastdds_peers.xml" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<!-- tool/peers.sh가 생성 — 로봇 $_robot ($_ip)의 참가자 슬롯 0~31을 유니캐스트로 탐색 (멀티캐스트도 계속 씀) -->
<profiles xmlns="http://www.eprosima.com">
  <transport_descriptors>
    <transport_descriptor>
      <transport_id>udp_peers</transport_id>
      <type>UDPv4</type>
      <maxInitialPeersRange>32</maxInitialPeersRange>
    </transport_descriptor>
    <transport_descriptor>
      <transport_id>shm</transport_id>
      <type>SHM</type>
    </transport_descriptor>
  </transport_descriptors>
  <participant profile_name="robot_peers" is_default_profile="true">
    <rtps>
      <userTransports>
        <transport_id>udp_peers</transport_id>
        <transport_id>shm</transport_id>
      </userTransports>
      <useBuiltinTransports>false</useBuiltinTransports>
      <builtin>
        <initialPeersList>
          <locator><udpv4><address>$_ip</address></udpv4></locator>
        </initialPeersList>
      </builtin>
    </rtps>
  </participant>
</profiles>
EOF
    export FASTDDS_DEFAULT_PROFILES_FILE="$_run/fastdds_peers.xml"
    unset ROS_STATIC_PEERS
    echo "DDS 피어: $_robot ($_ip, 슬롯 0~31) — $FASTDDS_DEFAULT_PROFILES_FILE"
  else
    echo "WARN: $_robot 을 찾을 수 없음 — 멀티캐스트 탐색만 사용 (Pi 꺼짐? 다른 WiFi?)" >&2
  fi
fi
unset _robot _run _cache _ip
