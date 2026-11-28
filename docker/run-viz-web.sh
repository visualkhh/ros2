#!/usr/bin/env bash
# 로봇 앱(srf_default_ws_real, 가짜 하드웨어)과 웹 게이트웨이를 한 컨테이너에서 띄운다.
# 브라우저: http://localhost:8080/
set -euo pipefail
cd /workspace/srf

echo "[viz-web] build srf_default_ws_viz_web..."
cmake -S . -B /tmp/srf-viz-web-build -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DSRF_BUILD_VIZ_WEB=ON \
      -DSRF_BUILD_TESTS=OFF \
      -DSRF_BUILD_BENCH=OFF \
      -DSRF_BUILD_VIZ=OFF \
      -DSRF_BUILD_EXAMPLES=OFF \
      -DSRF_VIZ_WEB_PROJECT_DIR=/workspace/srf-default-ws/viz-web >/tmp/srf-viz-web-cmake.log 2>&1
cmake --build /tmp/srf-viz-web-build --target srf_default_ws_viz_web -j"$(nproc)" >/tmp/srf-viz-web-build.log 2>&1

echo "[viz-web] build srf_default_ws_real..."
cmake -S /workspace/srf-default-ws -B /tmp/srf-default-ws-web-build -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DSRF_DEFAULT_WS_BUILD_TESTS=OFF \
      -DSRF_DEFAULT_WS_BUILD_RL_NAV=ON >/tmp/srf-default-ws-web-cmake.log 2>&1
cmake --build /tmp/srf-default-ws-web-build --target srf_default_ws_real -j"$(nproc)" >/tmp/srf-default-ws-web-build.log 2>&1

echo "[viz-web] robot + gateway"
cd /workspace/srf-default-ws
/tmp/srf-default-ws-web-build/srf_default_ws_real --opencr /dev/null --no-lidar --port 9000 >/tmp/srf_default_ws_real.log 2>&1 &
sleep 2
exec /tmp/srf-viz-web-build/viz-web/srf_default_ws_viz_web --robot 127.0.0.1:9000 --port 8080
