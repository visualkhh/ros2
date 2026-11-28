#!/usr/bin/env bash
# 컨테이너 안에서 Xvfb + openbox + x11vnc + noVNC를 띄우고, 그 위에서 TurtleBot3 데모를 실행한다.
# 호스트 브라우저: http://localhost:6080/vnc.html?autoconnect=1&resize=scale
set -euo pipefail
cd /workspace/srf

export DISPLAY=:99
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

Xvfb :99 -screen 0 1920x1080x24 -ac -nolisten tcp >/tmp/xvfb.log 2>&1 &
sleep 0.8
openbox >/tmp/openbox.log 2>&1 &
x11vnc -display :99 -forever -shared -nopw -rfbport 5900 -noxdamage -quiet >/tmp/x11vnc.log 2>&1 &
websockify --web=/usr/share/novnc 0.0.0.0:6080 localhost:5900 >/tmp/websockify.log 2>&1 &

echo "[srf] noVNC: http://localhost:6080/vnc.html?autoconnect=1&resize=scale"

if [ ! -x build/simulator/srf_simulator ]; then
  echo "[srf] 빌드가 없어서 먼저 빌드한다..."
  cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER=g++
  cmake --build build -j"$(nproc)"
fi

exec ./build/simulator/srf_simulator "$@"
