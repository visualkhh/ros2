# docker

`krosc`(C++ kros)를 빌드/테스트하고, **리눅스 안의 Qt sim+viz 창을 맥 브라우저로 보기 위한** Ubuntu 26.04 컨테이너.

- GCC 15 / G++ 15, CMake 4, Ninja
- Qt6 Core/Gui/Widgets (QPA offscreen, xcb)
- Xvfb + openbox + x11vnc + noVNC (브라우저 데스크톱)
- scrot (VNC 데스크톱 스크린샷)

소스는 이미지에 넣지 않고 호스트의 `ros2/` 저장소를 컨테이너 `/workspace`에 볼륨 마운트한다.

## 1. 이미지

```bash
# ros2/ 에서
./docker/build.sh
```

`docker/docker-compose.yml`의 `krosc`/`krosc-vnc` 서비스가 `krosc:ubuntu-26.04` 이미지를 만든다.

## 2. 컴파일 + 테스트 (화면 없이)

```bash
# ros2/ 에서
./docker/compile.sh RelWithDebInfo --clean
```

내부에서 실행하는 것 (`/workspace/krosc`):

```bash
cmake -S . -B build -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER=g++
cmake --build build -j"$(nproc)"
ctest --test-dir build --output-on-failure
```

테스트: `krosc_core_tests` 29개 + `krosc_sim_tests` 3개 = 32개. `krosc_ws`는 별도 빌드/테스트(4개)를 가진다.

## 3. 맥에서 UI 보기 (noVNC) — 추천

```bash
# ros2/ 에서
./docker/vnc.sh
```

그 다음 브라우저:

```
http://localhost:6080/vnc.html?autoconnect=1&resize=scale
```

- 컨테이너 안에서 `Xvfb :99` → `openbox` → `x11vnc` → `websockify(noVNC)`를 띄우고,
  그 X 디스플레이 위에서 `krosc_turtlebot3`가 **SimWindow(진실)** 와 **VizWindow(토픽)** 두 창을 연다.
- 맥은 WebSocket(6080)으로 화면만 받는다. 시뮬레이션/토픽/데이터는 전부 리눅스 컨테이너 안에 있다.
- 왼쪽 sim 창에서 `W/A/S/D`로 직접 몰거나, 오른쪽 viz 창에서 조종하면 된다.
- `Ctrl+C`로 끝내면 컨테이너도 정리된다.

VNC 데스크톱 스크린샷을 남기려면 (컨테이너가 떠 있는 동안):

```bash
docker compose -f docker/docker-compose.yml exec -T krosc-vnc \
  scrot -o /workspace/krosc/docs/vnc.png
```

> 보안: noVNC는 로컬 개발용으로 비밀번호 없이(`-nopw`) 띄운다. `6080`을 외부에 공개하지 마라.

## 4. 대안: XQuartz (macOS X11)

XQuartz를 설치하고 TCP 접속을 허용한 뒤, 컨테이너에 `DISPLAY=host.docker.internal:0`과
`QT_QPA_PLATFORM=xcb`를 주면 네이티브 X11 창으로 볼 수도 있다. 다만 Docker Desktop 설정에 따라
자주 막히므로 **noVNC를 권장**한다.

## 5. 로컬 Qt6 GUI

화면을 보며 조종하는 가장 간단한 방법은 Docker를 쓰지 않고 로컬에서 빌드하는 것이다.

```bash
cd krosc
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
./build/examples/turtlebot3/krosc_turtlebot3
```

## 셸

```bash
# ros2/ 에서
./docker/shell.sh
# inside: /workspace/krosc
```
