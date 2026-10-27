# ROS 2 Lyrical 설치 — Raspberry Pi 4 (Ubuntu 26.04 Server) 기준

이 프로젝트 로봇 SBC(`pak`) 기준입니다. 로봇에서 도는 것만 설치합니다:
**베이스(OpenCR) · slam_toolbox(지도 만들기) · Nav2(지도 주행) · tool**.
화면(RViz)은 PC에서 보므로 Pi에는 GUI를 깔지 않습니다.

- 대상: Raspberry Pi 4 Model B · 4GB RAM · 4코어 · arm64 · Ubuntu 26.04 LTS Server (resolute) · Python 3.14
- ROS: Lyrical Luth (LTS) — 26.04 Tier 1, 공식 apt에 arm64 deb 있음 (Jetson용 Isaac 저장소 같은 우회 불필요)
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
  ros-lyrical-slam-toolbox \
  ros-lyrical-nav2-bringup \
  ros-lyrical-nav2-amcl \
  ros-lyrical-nav2-map-server \
  ros-lyrical-robot-state-publisher \
  ros-lyrical-teleop-twist-keyboard \
  ros-lyrical-tf2-tools \
  ros-lyrical-turtlebot3-msgs \
  ros-lyrical-dynamixel-sdk \
  python3-numpy socat
```

| 패키지 | 쓰는 곳 |
|---|---|
| `slam-toolbox` | `tool/run-cli slam start` (지도 만들기) |
| `nav2-bringup` · `nav2-amcl` · `nav2-map-server` | `tool/run-cli nav start` · `explore start` · `map save` (bringup의 launch 파일을 그대로 씀 → 빼면 안 됨) |
| `robot-state-publisher` | 로봇 TF (`base_link → 라이다`) |
| `teleop-twist-keyboard` · `tf2-tools` | 수동 조종 · `view_frames` 디버그 |
| `turtlebot3-msgs` · `dynamixel-sdk` | 베이스 드라이버(OpenCR) 작업용 |
| `python3-numpy` | `ros2_ws` (mock_base·rl 노드) |
| `socat` | 가상 시리얼 시험 |

**apt에 없는 것**

- 라이다 드라이버: 이 로봇은 **LDS-03** → `coin_d4_driver` (ROBOTIS, apt 없음). `ros2_ws/deps.repos`로 받아 빌드 (4장).
  포트 `/dev/tb3_lidar`는 udev 규칙으로 만든다 → [udev 세팅](../../raspberry/udev-setup.md).
  (`hls-lfcd-lds-driver`(LDS-01) · `ld08-driver`(LDS-02)도 Lyrical arm64 apt에 없음)
- `cartographer_ros`: 없음 → SLAM은 slam_toolbox 사용.
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

```bash
git clone https://github.com/visualkhh/ros2.git ~/ros2   # 이미 있으면 cd ~/ros2 && git pull
cd ~/ros2/ros2_ws
source /opt/ros/lyrical/setup.bash
vcs import src < deps.repos                               # apt에 없는 소스 (LDS-03 coin_d4_driver)
colcon build --symlink-install --parallel-workers 2       # Pi 4: 2개로 제한 (메모리)
source install/setup.bash
```

갱신 흐름: PC에서 고치고 push → Pi에서 `cd ~/ros2 && git pull` →
`ros2_ws`를 바꿨으면 다시 `colcon build`, `tool/`만 바꿨으면 빌드 불필요,
`firmware/`를 바꿨으면 [OpenCR 문서](../../opencr/opencr-setup-ubuntu26.04.md)의 compile·upload.

**PC 추가분** (RViz로 보기·시험): `ros-lyrical-desktop` 또는 `ros-lyrical-rviz2` + 위 2장 목록.
PC 셸도 `ROS_DOMAIN_ID=25`여야 Pi가 보입니다 (`tool/run-rviz`는 자동으로 25).

## 5. 검증

```bash
printenv ROS_DISTRO ROS_DOMAIN_ID           # lyrical 25
ros2 pkg list | grep -E 'slam_toolbox|nav2_bringup|tb3_bringup'
cd ~/ros2/tool && ./run-cli status          # 프로세스·토픽 표
./run-cli base start && ./run-cli status    # mock_base면 odom 20Hz
./run-cli stop-all
```

PC ↔ Pi 통신 확인:

```bash
# Pi
ros2 run demo_nodes_cpp talker     # demo_nodes_cpp 없으면: ros2 topic pub /chatter std_msgs/msg/String "{data: hi}"
# PC (ROS_DOMAIN_ID=25)
ros2 topic echo /chatter
```

안 보이면: 같은 WiFi인지 · 양쪽 `ROS_DOMAIN_ID=25` · `ROS_AUTOMATIC_DISCOVERY_RANGE`가 `LOCALHOST`가 아닌지 ·
사내/게스트 WiFi의 기기 간 차단(AP isolation) — 이 경우 폰 핫스팟 등으로 시험.

## 6. Docker (PC 시험대 전용)

Pi에서는 Docker를 쓰지 않고 위처럼 네이티브로 설치합니다. PC에서 격리된 시험대가 필요할 때만
[docker/README.md](../../docker/README.md)를 봅니다 (`ROS_DOMAIN_ID=25`, `network_mode: host`, `shm_size`).

---
기준: Raspberry Pi 4 Model B Rev 1.5 · Ubuntu 26.04.1 LTS · 커널 7.0.0-raspi · 2026-10-27
