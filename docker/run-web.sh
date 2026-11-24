#!/usr/bin/env bash
# 로봇 앱(krosc_real, 가짜 하드웨어)과 웹 게이트웨이를 한 컨테이너에서 띄운다.
# 브라우저: http://localhost:8080/
set -euo pipefail
cd /workspace/krosc

echo "[web] build krosc_web..."
cmake -S . -B /tmp/krosc-web-build -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_BUILD_WEB=ON \
      -DKROSC_BUILD_TESTS=OFF \
      -DKROSC_BUILD_BENCH=OFF \
      -DKROSC_BUILD_VIZ=OFF \
      -DKROSC_BUILD_EXAMPLES=OFF >/tmp/krosc-web-cmake.log 2>&1
cmake --build /tmp/krosc-web-build --target krosc_web -j"$(nproc)" >/tmp/krosc-web-build.log 2>&1

echo "[web] build krosc_real..."
cmake -S /workspace/krosc_ws -B /tmp/krosc-ws-web-build -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DKROSC_WS_BUILD_TESTS=OFF \
      -DKROSC_WS_BUILD_RL_NAV=ON >/tmp/krosc-ws-web-cmake.log 2>&1
cmake --build /tmp/krosc-ws-web-build --target krosc_real -j"$(nproc)" >/tmp/krosc-ws-web-build.log 2>&1

echo "[web] robot + gateway"
cd /workspace/krosc_ws
/tmp/krosc-ws-web-build/krosc_real --opencr /dev/null --no-lidar --port 9000 >/tmp/krosc_real.log 2>&1 &
sleep 2
exec /tmp/krosc-web-build/web/krosc_web --robot 127.0.0.1:9000 --port 8080
