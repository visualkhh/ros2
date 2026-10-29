# ROS 2 Lyrical 설치 — Raspberry Pi 4 (Ubuntu 26.04 Server) 기준

이 프로젝트 로봇 SBC(`pak`) 기준입니다. 로봇에서 도는 것만 설치합니다:
**베이스(OpenCR) · cartographer(지도 만들기) · Nav2(지도 주행) · tool**.
화면(RViz)은 PC에서 보므로 Pi에는 GUI를 깔지 않습니다.

- 대상: Raspberry Pi 4 Model B · 4GB RAM · 4코어 · arm64 · Ubuntu 26.04 LTS Server (resolute) · Python 3.14
- ROS: Lyrical Luth (LTS) — 26.04 Tier 1, 공식 apt에 arm64 deb 있음 (Jetson용 Isaac 저장소 같은 우회 불필요)
- 전체 세팅 순서(목차): [Pi 세팅 순서](../../raspberry/README.md)
- OS 설치·WiFi·SSH: [SD카드 OS 설치](../../raspberry/raspberry-pi-ubuntu-sdcard-os-install.md) · [SSH 접속](../../raspberry/ssh-pak-local-troubleshooting.md)
- OpenCR 펌웨어 툴: [OpenCR 세팅](../../opencr/opencr-setup-ubuntu26.04.md)
- PC(개발·RViz)도 같은 방법으로 깔고, 4장의 PC 추가분만 더 깝니다.

> 이전 판은 Lv2 과제(Jetson Orin Nano, MoveIt·RTAB-Map·YOLO·pick_ik)를 옮긴 것이었습니다.
> 그 패키지들은 이 로봇에 없고 Pi 4에는 무거워서 뺐습니다.

## 0. 시작 전

```bash
ssh pak                       # PC에서
# 첫 부팅 직후면 자동 업데이트(unattended-upgrades)가 apt 잠금을 잡고 있음 → 끝날 때까지 대기
pgrep -a unattended-upgrade   # 아무것도 안 나와야 함 (상주용 --wait-for-signal 하나는 정상)
[ -f /var/run/reboot-required ] && sudo reboot   # 커널 업데이트가 있었으면 먼저 재부팅
```

메모리 4GB에 swap이 없으면 `colcon build`·대형 apt 설치 중 멈출 수 있어 swap 2GB를 만듭니다 (최초 1회):

```bash
sudo fallocate -l 2G /swapfile && sudo chmod 600 /swapfile
sudo mkswap /swapfile && sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
free -h
```

## 1. ROS 2 apt 저장소 + ros-base

```bash
sudo apt update && sudo apt install -y locales software-properties-common curl gnupg lsb-release
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
sudo add-apt-repository universe -y

# ROS 2 apt source — codename은 lsb_release로 (noble 등 하드코딩 금지)
export ROS_APT_SOURCE_VERSION=1.3.0
curl -fsSL -o /tmp/ros2-apt-source.deb \
  https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(lsb_release -cs)_all.deb
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update

# ros-base (desktop 아님 — Pi에 RViz·Gazebo 불필요)
sudo apt install -y ros-lyrical-ros-base ros-dev-tools python3-colcon-common-extensions
```

## 2. 이 프로젝트 패키지

`resolute arm64` 공식 apt 목록에서 있는 것을 확인했습니다 (2026-10-27).

```bash
sudo apt install -y \
  ros-lyrical-laser-filters \
  ros-lyrical-xacro \
  ros-lyrical-nav2-bringup \
  ros-lyrical-nav2-amcl \
  ros-lyrical-nav2-map-server \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-tf2-tools \
  ros-lyrical-image-geometry \
  espeak-ng \
  ros-lyrical-turtlebot3-msgs \
  ros-lyrical-dynamixel-sdk \
  python3-numpy socat
```

`ros-lyrical-image-geometry`는 `m-explore-ros2`에 딸려 오는 `multirobot_map_merge` 빌드에 필요.
빠진 시스템 의존성은 `rosdep install --from-paths src --ignore-src -r -y`로 한 번에 받으면 됨 (4장).

| 패키지 | 쓰는 곳 |
|---|---|
| `cartographer` (코어+ROS 래퍼, 소스 빌드) | `tool/run-cli slam start` (지도 만들기) — apt 없음, 아래 "apt에 없는 것" 참조 |
| `nav2-bringup` · `nav2-amcl` · `nav2-map-server` | `tool/run-cli nav start` · `explore start` · `map save` (bringup의 launch 파일을 그대로 씀 → 빼면 안 됨) |
| `robot-state-publisher` | 로봇 TF (`base_link → 라이다`) |
| `teleop-twist-keyboard` · `tf2-tools` | 수동 조종 · `view_frames` 디버그 |
| `turtlebot3-msgs` · `dynamixel-sdk` | 베이스 드라이버(OpenCR) 작업용 |
| `python3-numpy` | `ros2_ws` (rl 노드) |
| `socat` | 가상 시리얼 시험 |

**apt에 없는 것**

- 라이다 드라이버: 이 로봇은 **LDS-03** → `coin_d4_driver` (ROBOTIS, apt 없음). `ros2_ws/deps.repos`로 받아 빌드 (4장).
  포트 `/dev/tb3_lidar`는 udev 규칙으로 만든다 → [udev 세팅](../../raspberry/udev-setup.md).
  (`hls-lfcd-lds-driver`(LDS-01) · `ld08-driver`(LDS-02)도 Lyrical arm64 apt에 없음)
- 자동 맵핑 frontier 탐색: `explore_lite` (m-explore-ros2, BSD, apt 없음). `ros2_ws/deps.repos`로 받아 빌드 (4장).
  같은 저장소에 `multirobot_map_merge`도 딸려 오는데, 얘는 `image_geometry` apt가 필요 (2장 목록에 포함).
  없으면 `Could not find ... image_geometryConfig.cmake`로 깨지고 `cartographer_ros`까지 중단되므로,
  빌드 전 `rosdep install --from-paths src --ignore-src -r -y`로 받거나 `ros-lyrical-image-geometry`를 직접 깜.
  (우리는 `explore_lite`만 쓰고 `multirobot_map_merge`는 안 씀 — 급하면 `--packages-skip multirobot_map_merge`로 제외 가능)
- Google Cartographer (코어 + `cartographer_ros` + `cartographer_rviz`): Lyrical apt에 없음 → `ros2_ws/deps.repos`로 소스 빌드 (아래 4-1절).
  Pi 빌드 전제: `sudo apt install -y libceres-dev libprotobuf-dev libeigen3-dev liblua5.2-dev libcairo2-dev libgoogle-glog-dev libgflags-dev`
  (Ceres 2.x 대응 패치 `Manifold` 전환 + 신형 abseil `ABSL_` 접두어 패치 — `ros2_ws/patches/*.patch`를 `src/cartographer`,
  `src/cartographer_ros`에 각각 `patch -p1`로 적용. `src/`는 `.gitignore`라 패치 파일이 정본임. 4장 빌드 전 적용법 참조)
  Ubuntu 26.04의 CMake 4.x는 `cmake_minimum_required(<3.5)`를 거부 → cartographer 코어가 그대로는 configure 실패
  (`Compatibility with CMake < 3.5 has been removed`). 빌드 시 `--cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5` 필수 (4장 참조).
- (선택) RL 모델 추론: `pip install --break-system-packages onnxruntime` — torch는 Pi에 깔지 않음.

## 3. 셸 설정 (`~/.bashrc`)

```bash
cat >> ~/.bashrc <<'EOF'

# ROS 2 Lyrical
source /opt/ros/lyrical/setup.bash
[ -f ~/ros2/ros2_ws/install/setup.bash ] && source ~/ros2/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=25                         # 프로젝트 통일값 (PC·docker와 같게)
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET     # PC RViz가 Pi 토픽을 보려면 LOCALHOST 금지
EOF
source ~/.bashrc
```

`ROS_DOMAIN_ID=25`는 이미 들어가 있으면 한 번만 남기세요 (`grep -n ROS_DOMAIN_ID ~/.bashrc`).

## 4. 저장소 + 워크스페이스 빌드

권장은 `setup/pi/build.sh` 한 줄 (vcs import + rosdep install + 패치 + colcon build까지, 몇 번 돌려도 같은 결과):

```bash
git clone https://github.com/visualkhh/ros2.git ~/ros2   # 이미 있으면 cd ~/ros2 && git pull
~/ros2/setup/pi/build.sh
```

수동으로 할 때만 아래 순서 (build.sh와 같은 내용):

```bash
cd ~/ros2/ros2_ws
source /opt/ros/lyrical/setup.bash
vcs import src < deps.repos                               # apt에 없는 소스 (LDS-03 coin_d4_driver · 자동 맵핑 explore_lite · cartographer 코어+래퍼)
# cartographer 패치 적용 (최초 1회 + patches/*.patch가 바뀔 때마다 — src/는 gitignore라 pull로 안 넘어옴)
# 주의: 반드시 ~/ros2/ros2_ws에서 실행 (src/cartographer 안에서 cd src/... 하면 없음)
(cd src/cartographer && patch -p1 --forward < ../../patches/cartographer-ceres2.patch)
(cd src/cartographer_ros && patch -p1 --forward < ../../patches/cartographer_ros-ceres2.patch)
# 패치가 이미 적용된 상태에서 또 치면 `1 hunk ignored ... .rej`가 남음 → 그때는 각 폴더에서
# git checkout -- . && git clean -f 후 위 두 줄을 다시 (이력: issue-tracking 2026-10-28 §7-8)
rosdep install --from-paths src --ignore-src -r -y   # m-explore-ros2의 multirobot_map_merge용 image_geometry 등 (없으면 §9 에러)
# (rosdep 최초 1회는 sudo rosdep init && rosdep update 필요 — setup/pi/build.sh는 자동 처리, 수동 빌드는 직접)
colcon build --symlink-install --parallel-workers 2 \  # Pi 4: 2개로 제한 (메모리)
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5      # CMake 4.x가 <3.5를 거부하므로 필수
source install/setup.bash
```

갱신 흐름: PC에서 고치고 push → Pi에서 `cd ~/ros2 && git pull` →
`ros2_ws`를 바꿨으면 다시 `colcon build`, `tool/`만 바꿨으면 빌드 불필요,
`firmware/`를 바꿨으면 [OpenCR 문서](../../opencr/opencr-setup-ubuntu26.04.md)의 compile·upload.

### 4-1. 소스로 받거나 직접 빌드하는 것 (전체 목록)

apt로 안 되는 것만 모음. 받는 위치·버전은 저장소 파일에 고정돼 있고, 받은 소스 폴더는 `.gitignore` (저장소엔 목록만).

| 무엇 | 왜 | 어디서 받나 / 버전 | 빌드 | 문서 |
|---|---|---|---|---|
| `coin_d4_driver` | LDS-03 라이다 드라이버 (Lyrical apt 없음) | `ros2_ws/deps.repos` — ROBOTIS, 커밋 `79d1833` | `vcs import` → `colcon build` | [ros2_ws README](../../../ros2_ws/README.md#depsrepos--apt에-없는-외부-소스) |
| `explore_lite` · `explore_lite_msgs` (+ 딸림 `multirobot_map_merge`) | 자동 맵핑 frontier 탐색 (Lyrical apt 없음) | `ros2_ws/deps.repos` — robo-friends/m-explore-ros2, 커밋 `326cf8a` | `vcs import` → 패치(`m-explore-ros2-ament-target-deps`) → `rosdep install` (image_geometry) → `colcon build` (Pi 약 2분, 경고만) | `ros2_ws/patches/m-explore-ros2-ament-target-deps.patch` (Lyrical `ament_target_dependencies` 분리 대응) |
| `cartographer` (코어) | 2D SLAM 코어 (Lyrical apt 없음) | `ros2_ws/deps.repos` — 커밋 `877157a` | `vcs import` → 패치 → `colcon build --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5` (Pi 약 20분, CMake 4.x 필수) | `ros2_ws/patches/cartographer-ceres2.patch` (Ceres 2.x `Manifold` + abseil `ABSL_` 접두어) |
| `cartographer_ros` (+`cartographer_ros_msgs`·`cartographer_rviz`) | SLAM ROS 래퍼 — `cartographer_node`·`occupancy_grid_node` (Lyrical apt 없음) | `ros2_ws/deps.repos` — 커밋 `7433452` | `vcs import` → 패치 4종 → `colcon build` (같은 `--cmake-args` 공유) | `patches/cartographer_ros-ceres2.patch` (abseil `ABSL_` 7파일) + `patches/cartographer_ros-gflags-unique.patch` (gflags 중복 정의 해소 §11) + `patches/cartographer_ros-absl-deadlock.patch` (§14) |
| 우리 패키지 (`tb3_bringup` · `rl_correction` · `rl_eval` · `nav2_rl_controller`) | 이 프로젝트 코드 | 저장소 `ros2_ws/src/` | `colcon build` | [ros2_ws README](../../../ros2_ws/README.md) |
| OpenCR 펌웨어 `tb3_wheels` | 바퀴·부저 펌웨어 | 저장소 `firmware/opencr/` | `compile.sh` → `upload.sh` (Pi에서) | [OpenCR 세팅](../../opencr/opencr-setup-ubuntu26.04.md) |
| arduino-cli + OpenCR 보드 패키지 | 펌웨어 빌드 도구 (arm64는 armhf 툴체인·core min/max 패치 필요) | `firmware/opencr/setup.sh`가 설치·패치 | `setup.sh` 1회 | 같음 |

**빌드 주의**

- Lyrical `ament_cmake` 2.8.8은 `ament_target_dependencies()`를 딸려 주지 않음 (별도 `ament_cmake_target_dependencies`
  패키지로 분리). 그 매크로를 쓰는 외부 소스(`multirobot_map_merge`)는 `CMakeLists.txt`에
  `find_package(ament_cmake_target_dependencies REQUIRED)`를 넣는 패치가 필요 — `patches/m-explore-ros2-ament-target-deps.patch`.
  "Unknown CMake command ament_*"가 뜨면 이 분리를 의심 (이력: issue-tracking 2026-10-28 §10).

- `multirobot_map_merge` (m-explore-ros2 딸림): `image_geometry` 없이 빌드하면
  `Could not find a package configuration file provided by "image_geometry"`로 실패하고
  `cartographer_ros`까지 중단됨. `ros-lyrical-image-geometry` 설치 또는
  `rosdep install --from-paths src --ignore-src -r -y`로 해결. 안 쓰는 패키지라
  `--packages-skip multirobot_map_merge`로 제외해도 됨 (이력: issue-tracking 2026-10-28 §9).
- CMake 4.x (Ubuntu 26.04): `cartographer`의 `cmake_minimum_required`가 3.5 미만이라 그대로는 configure 실패
  (`Compatibility with CMake < 3.5 has been removed from CMake`). `--cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5`를 붙임.
  소스를 고치지 않으므로 `deps.repos` 갱신·재빌드 때마다 그대로 씀.
- 노드·실행 파일을 지웠거나 이름을 바꿨으면 colcon이 옛 것을 install에 남긴다 → 그 패키지만 지우고 다시:
  `rm -rf build/tb3_bringup install/tb3_bringup && colcon build --packages-select tb3_bringup`
  (확인: `ros2 pkg executables tb3_bringup`)
- 외부 소스 버전을 올릴 땐 `deps.repos`의 커밋만 바꾸고 `vcs import src < deps.repos` → 다시 빌드.
- PC는 `vcs import`를 안 해도 됨 (라이다·자동 맵핑은 Pi에서 돔). PC의 nav2_bringup엔 Python 모듈이 없어
  `explore-nav.launch.py`는 PC에서 안 뜸 — Pi 전용.

**PC 추가분** (RViz로 보기·시험): `ros-lyrical-desktop` 또는 `ros-lyrical-rviz2` + 위 2장 목록.
PC 셸도 `ROS_DOMAIN_ID=25`여야 Pi가 보입니다 (`tool/run-rviz`는 자동으로 25).

## 5. 검증

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID           # lyrical 25
ros2 pkg list | grep -E 'cartographer_ros|nav2_bringup|tb3_bringup'
cd ~/ros2/tool && ./run-cli status          # 프로세스·토픽 표
./run-cli base start && ./run-cli status    # tb-only: 로봇 ok · 모드 MANUAL · odom 20Hz · scan 10Hz
./run-cli stop-all
```

PC ↔ Pi 통신 확인:

```bash
# Pi
ros2 run demo_nodes_cpp talker     # demo_nodes_cpp 없으면: ros2 topic pub /chatter std_msgs/msg/String "{data: hi}"
# PC (ROS_DOMAIN_ID=25)
ros2 topic echo /chatter
```

**사내 WiFi는 멀티캐스트를 버려서 자동 탐색이 불안정** (ping은 되는데 노드가 안 보이거나 보였다 사라짐 — 2026-10-27 실측).
PC 쪽에서 Pi를 유니캐스트로 직접 찾게 하면 해결 (한쪽만 지정해도 양방향): `tool/peers.sh`가 Fast DDS 프로필을 만들어
Pi IP의 참가자 슬롯 0~31을 찾게 함 — `run-cli`·`run-rviz`는 자동, 터미널에선 `export ROS_DOMAIN_ID=25 && source tool/peers.sh`.
(`ROS_STATIC_PEERS`는 슬롯 0~3만 찾아서 Pi에 노드가 많으면 일부가 안 보임)

그래도 안 보이면: 같은 WiFi인지 · 양쪽 `ROS_DOMAIN_ID=25` · `ROS_AUTOMATIC_DISCOVERY_RANGE`가 `LOCALHOST`가 아닌지 ·
사내/게스트 WiFi의 기기 간 차단(AP isolation) — 이 경우 폰 핫스팟 등으로 시험.

## 6. Docker (PC 시험대 전용)

Pi에서는 Docker를 쓰지 않고 위처럼 네이티브로 설치합니다. PC에서 격리된 시험대가 필요할 때만
[docker/README.md](../../docker/README.md)를 봅니다 (`ROS_DOMAIN_ID=25`, `network_mode: host`, `shm_size`).

---
기준: Raspberry Pi 4 Model B Rev 1.5 · Ubuntu 26.04.1 LTS · 커널 7.0.0-raspi · 2026-10-27
