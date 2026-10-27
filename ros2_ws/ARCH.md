# 시스템 구성 (노드 계통도)

로봇 = TurtleBot3 Waffle Pi 개조형 (맨 위 판 제거, LDS-03 라이다 1층 앞).
**Pi(`pak`)는 움직이는 것**, **PC는 RL 계산·화면**. 둘은 같은 WiFi + `ROS_DOMAIN_ID=25`로 토픽을 주고받습니다.

- 사용법: [README.md](README.md) · 운영 도구: [../tool/README.md](../tool/README.md) · 펌웨어 프로토콜: [../firmware/opencr/README.md](../firmware/opencr/README.md)

## 1. 어디서 무엇이 도나

```mermaid
flowchart LR
  subgraph HW["하드웨어 (로봇)"]
    OPENCR["OpenCR<br/>tb3_wheels 펌웨어"]
    DXL["DYNAMIXEL XM430 ×2<br/>(ID 1 좌 · 2 우)"]
    LDS["LDS-03 라이다<br/>(USB2LDS · CP2102)"]
    OPENCR <-->|"1 Mbps"| DXL
  end

  subgraph PI["Raspberry Pi 4 — pak (tb3_bringup + tool)"]
    DRV["opencr_driver"]
    RSP["robot_state_publisher<br/>(URDF)"]
    LIDAR["coin_d4_driver"]
    FILT["scan_filter<br/>(laser_filters 몸통 상자)"]
    SLAM["slam_toolbox<br/>(tool: slam start)"]
    NAV["Nav2<br/>(tool: nav / explore start)"]
    TELE["teleop<br/>(tool: teleop)"]
  end

  subgraph PC["PC (개발 · RL · 화면)"]
    RL["rl_corrector<br/>(rl-only)"]
    EVAL["eval_recorder<br/>(rl-only)"]
    RVIZ["RViz<br/>(tool/run-rviz)"]
  end

  OPENCR <-->|"USB CDC /dev/opencr<br/>C·T·R ↓ · S·E ↑"| DRV
  LDS -->|"/dev/tb3_lidar 230400"| LIDAR
  LIDAR -->|"/scan_raw"| FILT
  FILT -->|"/scan"| SLAM
  FILT -->|"/scan"| NAV
  SLAM -->|"/map · TF map→odom"| NAV
  DRV -->|"/odom · TF odom→base_footprint"| SLAM
  DRV -->|"/odom"| NAV
  RSP -->|"TF base_footprint→base_link→base_scan"| SLAM
  SLAM -.->|"/map /scan TF (WiFi)"| RVIZ
  NAV -.->|"경로 · 코스트맵"| RVIZ
```

## 2. 바퀴 명령 경로 — 3가지 bringup

| launch | 어디서 | 띄우는 것 | 드라이버가 받는 토픽 |
|---|---|---|---|
| `tb-only.launch.py` | Pi | opencr_driver · robot_state_publisher · coin_d4_driver · scan_filter | `cmd_vel` |
| `tb-for-rl.launch.py` | Pi | tb-only와 같음 (`rl:=true`만 넘김) — **RL 노드는 안 띄움** | `cmd_vel_rl` |
| `rl-only.launch.py` | PC | rl_corrector · eval_recorder — **로봇 노드는 안 띄움** | — |

### tb-only — RL 없음 (보정 없이 그대로, PC 불필요)

```mermaid
flowchart LR
  T["teleop / Nav2"] -->|"cmd_vel"| D["opencr_driver"] -->|"C,l,r 20Hz"| O["OpenCR"]
```

### tb-for-rl (Pi) + rl-only (PC) — RL 보정

```mermaid
flowchart LR
  subgraph Pi
    T["teleop / Nav2"]
    D["opencr_driver"]
    O["OpenCR"]
  end
  subgraph PC
    C["rl_corrector<br/>cmd_vel → cmd_vel_rl"]
    E["eval_recorder<br/>CSV"]
  end
  T -->|"cmd_vel"| C
  T -->|"cmd_vel"| E
  C -->|"cmd_vel_rl"| D
  C -->|"cmd_vel_rl"| E
  D -->|"C,l,r"| O
  D -.->|"odom"| E
```

- PC가 꺼지거나 WiFi가 끊기면 `cmd_vel_rl`이 끊김 → 드라이버가 0.5초 뒤 0 → 펌웨어도 500 ms watchdog로 정지 (안전 쪽).
- `nav2_rl_controller`(Nav2 내부 `cmd_vel_nav` 단계 보정)는 패키지만 있고 아직 launch에 연결 안 함.

## 3. TF 트리 (REP-105)

```mermaid
flowchart TD
  MAP["map"] -->|"slam_toolbox 또는 amcl"| ODOM["odom"]
  ODOM -->|"opencr_driver (mock_base)"| FP["base_footprint (바닥)"]
  FP -->|"URDF 고정 +0.010"| BL["base_link (바퀴축)"]
  BL -->|"URDF 고정"| SCAN["base_scan (LDS-03)"]
  BL -->|"URDF · /joint_states"| WL["wheel_left_link"]
  BL -->|"URDF · /joint_states"| WR["wheel_right_link"]
  BL -->|"URDF 고정"| CA["caster_back_left/right_link"]
```

| 프레임 | 누가 냄 | 기준으로 쓰는 곳 |
|---|---|---|
| `map → odom` | slam_toolbox(맵핑) · amcl(지도 주행) | Nav2 global costmap |
| `odom → base_footprint` | opencr_driver (PC 시험: mock_base) | SLAM `base_frame` · amcl `base_frame_id` |
| `base_footprint → base_link → …` | robot_state_publisher (`urdf/robot.urdf.xacro`) | Nav2 costmap · footprint 다각형 · 몸통 필터 |

## 4. 토픽

| 토픽 | 타입 | 내는 노드 | 받는 노드 |
|---|---|---|---|
| `/cmd_vel` | Twist | teleop · Nav2(`enable_stamped_cmd_vel: false`) · tool drive | opencr_driver(tb-only) · rl_corrector |
| `/cmd_vel_rl` | Twist | rl_corrector (PC) | opencr_driver(tb-for-rl) · eval_recorder |
| `/odom` | Odometry | opencr_driver | slam_toolbox · Nav2 · eval_recorder · tool |
| `/joint_states` | JointState | opencr_driver (effort = 전류 A) | robot_state_publisher |
| `/diagnostics` | DiagnosticArray | opencr_driver (전류·온도·모터 에러) | tool · rqt |
| `/opencr/reboot` | Empty | 사람 · tool | opencr_driver → `R` |
| `/scan_raw` | LaserScan | coin_d4_driver | scan_filter |
| `/scan` | LaserScan | scan_filter | slam_toolbox · amcl · Nav2 costmap |
| `/map` | OccupancyGrid | slam_toolbox · map_server | Nav2 · RViz · tool map save |
| `/initialpose` · `/goal_pose` | Pose… | RViz · tool | amcl · Nav2 |

## 5. 장치 · 포트

| 장치 | USB | udev 이름 | 규칙 |
|---|---|---|---|
| OpenCR | `0483:5740` STM Virtual COM | `/dev/opencr` | `documents/raspberry/udev/99-opencr.rules` |
| LDS-03 (USB2LDS) | `10c4:ea60` CP2102 | `/dev/tb3_lidar` | `documents/raspberry/udev/99-tb3-lidar.rules` |

설치·설명: [documents/raspberry/udev-setup.md](../documents/raspberry/udev-setup.md).

## 6. 패키지

| 패키지 | 내용 |
|---|---|
| `tb3_bringup` | `opencr_driver` · `mock_base` · launch 5개 · `config/robot.yaml` · `config/lidar.yaml` · URDF · 바퀴 메시 |
| `rl_correction` | `corrector` — `cmd_vel_in` → (ONNX 정책) → clip → `cmd_vel_out` |
| `nav2_rl_controller` | `controller_corrector` — Nav2 컨트롤러 출력 보정 (미연결) |
| `rl_eval` | `eval_recorder` — 보정 전/후 cmd·odom CSV · `benchmark_onnx.py` |
| `coin_d4_driver` (외부) | LDS-03 드라이버 — `deps.repos`로 받음 (apt 없음) |
| apt | slam_toolbox · nav2_bringup · laser_filters · robot_state_publisher · xacro |

SLAM·Nav2는 `ros2_ws` launch가 아니라 `tool/run-cli`가 켜고 끕니다 — 베이스(tb-only)는 켜 둔 채
맵핑 ↔ 지도 주행을 바꿔 켜기 때문. 설정은 `tool/config/slam_toolbox.yaml` · `nav2_params.yaml`.
