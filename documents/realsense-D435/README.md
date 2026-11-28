# RealSense D435 — Raspberry Pi 연결 · 확인

로봇 Pi(`pak@pak.local`)에 연결한 **Intel RealSense D435**를 ROS 2 Lyrical로 확인하고,
`srf-default-ws`의 **Hardware + Sensor** 구조로 연동하는 방법을 정리한다.

- 측정일: 2026-11-26
- 대상: Raspberry Pi 4 (`pak`, Ubuntu 26.04 / kernel 7.0.0-1024-raspi)
- 카메라: Intel RealSense D435 (`8086:0b07`)

## 1. 연결 상태 (실측)

```text
Bus 002 Device 002: ID 8086:0b07 Intel Corp. RealSense D435
```

USB 트리에서 **5 Gbps(USB 3.2)** 로 붙어 있는지 확인한다. 480M로 보이면 USB 2 포트/케이블이다.

```bash
lsusb
lsusb -t
```

정상 예:

```text
/:  Bus 002.Port 001: Dev 001, Class=root_hub, Driver=xhci_hcd/4p, 5000M
    |__ Port 002: Dev 002, If 0, Class=Video, Driver=uvcvideo, 5000M
    |__ Port 002: Dev 002, If 1, Class=Video, Driver=uvcvideo, 5000M
    ...
```

udev 정보:

```bash
udevadm info --query=property --name=/dev/video0 |
  grep -E 'ID_MODEL|ID_VENDOR|ID_SERIAL|ID_USB|ID_PATH'
```

실측 값:

| 항목 | 값 |
|---|---|
| 모델 | `Intel(R) RealSense(TM) Depth Camera 435` |
| USB ID | `8086:0b07` |
| 시리얼 | `261822074109` |
| USB 속도 | 5000M (USB 3.2) |
| 비디오 노드 | `/dev/video0` ~ `/dev/video5` (`uvcvideo`) |
| ROS 드라이버(확인용) | `ros-lyrical-realsense2-camera` 4.58.4 |
| librealsense2 | 2.58.4 (현재 Pi는 ROS 패키지로 설치됨) |

## 2. 권한 — `video` 그룹

`/dev/video*`는 기본적으로 `root:video 660`이다. 사용자가 `video` 그룹에 없으면
RealSense 노드가 아래처럼 실패한다.

```text
get_dev_capabilities: Cannot open '/dev/video3' Last Error: Permission denied
Mipi device capability could not be grabbed Last Error: Permission denied
```

확인:

```bash
id
ls -l /dev/video0
# crw-rw---- 1 root video 81, 0 ... /dev/video0
```

해결:

```bash
sudo usermod -aG video pak
```

> 그룹 변경은 **새 로그인 세션**부터 적용된다. SSH 세션이라면 다시 접속하거나
> `newgrp video`로 새 셸을 연다. `id`에 `44(video)`가 보여야 한다.

## 3. ROS 2 드라이버로 확인

드라이버 패키지는 이미 설치되어 있다.

```bash
source /opt/ros/lyrical/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_depth:=true enable_color:=true \
  enable_infra1:=false enable_infra2:=false pointcloud:=false
```

다른 터미널에서:

```bash
source /opt/ros/lyrical/setup.bash
ros2 topic list | grep camera
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

정상 실측:

```text
/camera/camera/color/camera_info
/camera/camera/color/image_raw
/camera/camera/color/metadata
/camera/camera/depth/camera_info
/camera/camera/depth/image_rect_raw
/camera/camera/depth/metadata
/camera/camera/extrinsics/depth_to_color

color: average rate: 30.5 Hz
depth: average rate: 30.5 Hz
```

`rs-enumerate-devices`, `v4l2-ctl`은 필수가 아니다. V4L2 장치 목록만 보고 싶으면:

```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

`rs-enumerate-devices`는 `librealsense2-utils` 계열 패키지가 있을 때만 사용할 수 있다.
이 Pi에는 설치되어 있지 않아도 위 `ros2 launch` 경로로 충분하다.

## 4. librealsense2 standalone 설치 (ROS 없이)

`srf-default-ws`는 ROS를 쓰지 않는다. `librealsense2`는 Intel의 독립 라이브러리이므로
ROS 패키지(`ros-lyrical-librealsense2`)와 무관하게 `/usr/local`에 설치할 수 있다.

### 4.1 패키지가 있으면

```bash
sudo apt install librealsense2-dev librealsense2-utils
pkg-config --modversion realsense2
```

> Ubuntu 26.04/Pi에서는 apt에 `librealsense2-dev`가 없고 ROS 패키지만 있는 경우가 있다.
> 그때는 아래 소스 빌드를 쓴다.

### 4.2 소스 빌드 (Pi, 권장)

의존성:

```bash
sudo apt install -y git cmake build-essential pkgconf \
  libssl-dev libusb-1.0-0-dev libudev-dev
# 그래픽 예제를 켤 때만: libgtk-3-dev libglfw3-dev
```

소스:

```bash
git clone --depth 1 --branch v2.58.4 \
  https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

빌드/설치:

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF \
  -DBUILD_UNIT_TESTS=OFF \
  -DBUILD_ROSBAG2=OFF
cmake --build build -j2
sudo cmake --install build
sudo ldconfig
```

`BUILD_ROSBAG2`는 librealsense의 rosbag 재생 기능이라 `srf-default-ws`에는 필요 없다.
빌드 시간을 줄이려면 위처럼 끈다.

udev 규칙:

```bash
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

보통 `udevadm trigger`로 바로 적용되고, 안 되면 카메라를 다시 꽂는다.
적용 후 USB 장치가 `crw-rw-rw- root plugdev`인지 확인한다.

확인:

```bash
pkg-config --modversion realsense2
ls /usr/local/lib/cmake/realsense2
ls -l /usr/local/lib/librealsense2.so*
ls -l /dev/bus/usb/002/002   # udev 적용 후 crw-rw-rw- root plugdev
```

`2.58.4`가 나오면 된다. `rs-enumerate-devices`는 `-DBUILD_EXAMPLES=ON`일 때
빌드되거나 별도 `librealsense2-utils` 패키지가 필요하다. SRF에는 필수가 아니다.

### 4.3 srf-default-ws가 찾게 하기

- `/usr/local`에 설치하면 CMake가 자동으로 찾는다 (`/usr/local`은 기본 prefix).
- 다른 prefix면:

```bash
SRF_DEFAULT_WS_REALSENSE_ROOT=/opt/librealsense ./srf-default-ws/build.sh robot
# 또는
cmake -S srf-default-ws -B srf-default-ws/build -DSRF_DEFAULT_WS_REALSENSE_ROOT=/opt/librealsense
```

- `CMAKE_PREFIX_PATH`/`PKG_CONFIG_PATH`로 지정해도 된다.
- CMake는 `/opt/ros/...`를 자동으로 뒤지지 않는다.

빌드 로그:

```text
-- srf_default_ws: RealSense D435 활성 (librealsense2 + libjpeg)
```

## 5. `srf-default-ws` 연동 구조

> 3장의 `ros2 launch`는 **장치 자체를 확인하는 용도**다. `srf-default-ws`는 ROS 노드를 쓰지 않고,
> `librealsense2`만 직접 링크한다. CMake는 ROS 경로를 하드코딩하지 않는다.

ROS 노드(`realsense2_camera_node`)를 쓰지 않고, 같은 카메라를 `srf-default-ws` 엔진에 직접 붙인다.
역할은 기존 OpenCR/LDS-03과 같다.

```text
engine/main.cpp
  └─ RealSenseDevice 생성 (Hardware, 물리 장치)
       └─ Engine (engine/engine.cpp)가 Sensor를 조립
            └─ RealSenseSensor (Sensor, 논리 레이어)
                 ├─ camera/color/image_raw   (Image, JPEG)
                 ├─ camera/depth/image_rect_raw (Image, 16UC1 raw)
                 └─ diagnostics (RealSense D435 상태)
```

| 파일 | 역할 |
|---|---|
| `srf-default-ws/engine/hardwares/realsense/realsense_device.hpp/.cpp` | librealsense2 pipeline, 최신 color/depth 프레임 보관, color JPEG 인코딩 |
| `srf-default-ws/engine/sensors/realsense/realsense_sensor.hpp/.cpp` | `Image` 토픽 발행, 발행 주기/깊이 주기, diagnostics |
| `srf-default-ws/engine/engine.hpp/.cpp` | Hardware 주입 → SensorManager 등록 |
| `srf-default-ws/engine/main.cpp` | `--camera-*` 옵션, 장치 생성/주입 |
| `srf-default-ws/viz-web/www/plugins/camera.js` | 웹 Viz의 **Camera** 탭 (color/depth) |

### 빌드

`librealsense2`와 `libjpeg`가 있으면 자동으로 포함된다. 없으면 카메라 없이 빌드된다.

```bash
./srf-default-ws/build.sh robot
# configure 로그에 아래가 보이면 RealSense 포함:
#   -- srf_default_ws: RealSense D435 활성 (librealsense2 + libjpeg)

# standalone prefix를 직접 지정할 때
SRF_DEFAULT_WS_REALSENSE_ROOT=/usr/local ./srf-default-ws/build.sh robot
```

CMake는 `find_package(realsense2)` → `pkg-config realsense2` 순으로 찾는다.
ROS 패키지 경로(`/opt/ros/...`)를 자동으로 뒤지지 않는다.
Pi의 현재 `librealsense2`는 ROS 패키지로 설치되어 있으므로, ROS 없이 쓰려면
standalone으로 설치하거나 `SRF_DEFAULT_WS_REALSENSE_ROOT`/`PKG_CONFIG_PATH`를 직접 지정한다.

Pi에서는 `srf_default_ws_real`만 빌드한다. Qt Viz/시뮬레이터/viz-web은 PC 전용이다.

### 실행

```bash
# 기본: 카메라 자동 사용 (color 320x240 5Hz, depth 160x120 2Hz)
./srf-default-ws/run.sh

# 카메라 끄기
./srf-default-ws/run.sh --no-camera

# 여러 대 중 특정 시리얼 선택
./srf-default-ws/run.sh --camera-serial 261822074109

# 발행 주기 조정
./srf-default-ws/run.sh --camera-rate 15 --camera-depth-rate 5
```

환경변수:

| 변수 | 기본 | 의미 |
|---|---|---|
| `SRF_DEFAULT_WS_CAMERA` | `1` | `0`이면 카메라 비활성 |
| `SRF_DEFAULT_WS_CAMERA_SERIAL` | (빈 값) | 특정 D435 시리얼 |
| `SRF_DEFAULT_WS_CAMERA_RATE` | `10` | color Image 발행 Hz |
| `SRF_DEFAULT_WS_CAMERA_DEPTH_RATE` | `5` | depth Image 발행 Hz |

### 토픽/메시지

| 토픽 | 실기 encoding | 시뮬레이터 encoding | 내용 |
|---|---|---|---|
| `camera/color/image_raw` | `jpeg` | `rgb8` | 실기 320×240 → JPEG, 시뮬 160×120 RGB |
| `camera/depth/image_rect_raw` | `16UC1` | `16UC1` | 실기 160×120(4× decimation), little-endian mm, `step = width*2` |

실기는 color 5Hz / depth 2Hz, 시뮬레이터도 color 5Hz(렌더)/depth 2Hz(발행)다.

`srf::msgs::Image` 필드:

```text
header(stamp, frame_id), width, height, encoding, step, data
```

depth는 color에 정렬(align)하지 않은 **depth 센서 시점**이다. 필요하면 후속으로
`rs2::align(RS2_STREAM_COLOR)`를 추가한다.

### 웹 Viz에서 보기

PC에서:

```bash
./srf-default-ws/run-viz-web.sh
# http://localhost:8080/  → Camera 탭
```

- 왼쪽: Color (실기 JPEG / 시뮬 RGB8)
- 오른쪽: Depth (0.2 m 파랑 → 3.0 m 빨강, 기본 particles 2×2 샘플)
- 상단: color/depth fps, 연결 상태
- Diagnostics 패널에도 `RealSense D435 [ok]`와 serial/firmware/fps/age가 표시된다.

영상은 `Image` 메시지로 PeerNetwork를 통해 **binary WebSocket frame(opcode 0x2)** 으로
브라우저에 전달된다. JSON/base64로 바꾸지 않는다.

raw depth(160×120×2 ≈ 38 KB)를 2Hz로 보내므로 대역폭이 작다.
더 줄이려면 `--camera-depth-rate 1` 또는 `--camera-depth-decimation 4`를 쓴다.

### 시뮬레이터 (실기 없이)

PC에서 Qt 시뮬레이터를 띄우면 오른쪽에 **REALSENSE D435 (SIM)** 패널이 붙는다.
같은 `Engine`/`RealSenseSensor`를 타므로 웹 Viz의 Camera 탭도 동시에 볼 수 있다.

```bash
./srf-default-ws/run-simulator-vizweb.sh
# Qt 창: REALSENSE D435 (SIM) color/depth
# 웹:   http://localhost:8080/ → Camera
```

시뮬레이터 카메라는 `SimulatorRealSense`가 월드 벽/퍽을 2.5D 레이캐스트해서 160×120으로 만든다.
실기 `librealsense2` 없이 PC에서 전체 카메라 경로(Engine → Sensor → PeerNetwork → Web)를
검증할 수 있다. Qt 패널은 시뮬레이션 스레드가 만든 최신 프레임을 복사해서 그린다.

## 6. 트러블슈팅

### `Permission denied` (`/dev/video*`)

`video` 그룹에 사용자를 추가하고 **재로그인**한다.

```bash
sudo usermod -aG video pak
id    # 44(video) 확인
```

### `xioctl(VIDIOC_S_FMT) failed, errno=5`

카메라를 이미 다른 프로세스가 잡고 있을 때 발생한다. stale `realsense2_camera_node`를 정리한다.

```bash
pgrep -af realsense2_camera
pkill -f realsense2_camera_node
pkill -f 'ros2 launch realsense2_camera'
sleep 2
```

그래도 안 되면 USB를 뺐다 꽂거나 Pi를 재부팅한다.

### USB 2(480M)로 인식

- USB 3 포트(파란색)에 직결
- USB 3 지원 케이블 사용
- 허브를 거치면 5Gbps가 안 나올 수 있다

```bash
lsusb -t | grep -A2 8086
```

### 프레임이 안 보임

```bash
# SRF 실행 로그
./srf-default-ws/run.sh
# [srf_default_ws] RealSense: ... serial=... color=640x480@30 depth=640x480@30

# 웹
# Camera 탭 상단이 "연결"인지, fps가 0인지 확인
```

`RealSense D435 [disconnected]`이면 USB/전원을 확인한다. 카메라를 뽑았다 다시 꽂으면
프로세스를 재시작해야 한다.

## 7. 참고

- RealSense ROS 2: <https://github.com/IntelRealSense/realsense-ros>
- librealsense: <https://github.com/IntelRealSense/librealsense>
- 관련 문서: [../raspberry/README.md](../raspberry/README.md) · [../README.md](../README.md)
