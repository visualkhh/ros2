# Isaac Sim ↔ ROS 2 토픽 계약 (`/isaacros`)

USD(로봇 자산)에는 ROS 토픽이 없다. 토픽은 **bridge 설정**에 있고, 우리는 그 이름을 지금 고정한다.
나중에 Nav2·SLAM·`rl_eval` 노드를 sim에 직접 붙일 때 이 이름으로 바로 remap하면 된다.

- 정본 코드: [`tasks/common/ros_topics.py`](tasks/common/ros_topics.py)
- real 로봇(pak): `ROS_DOMAIN_ID=25`
- **Isaac sim: `ROS_DOMAIN_ID=26`** (같은 WiFi에서 real과 동시에 켜도 DDS가 안 섞임)
- 네임스페이스: **`/isaacros`**
- 시간: `/isaacros/clock` → 모든 노드 `use_sim_time:=true`

> NVIDIA **Isaac ROS**(isaac_ros_* 패키지)와는 다른 것. 이건 우리 프로젝트의 sim 네임스페이스임.

## 토픽

| 토픽 | 타입 | 방향 | 프레임 / 비고 |
|---|---|---|---|
| `/isaacros/clock` | `rosgraph_msgs/Clock` | pub | sim 시간 |
| `/isaacros/tf` | `tf2_msgs/TFMessage` | pub | `map→odom→base_footprint→base_link` |
| `/isaacros/tf_static` | `tf2_msgs/TFMessage` | pub | `base_link→imu_link`, `base_scan` |
| `/isaacros/joint_states` | `sensor_msgs/JointState` | pub | `wheel_left_joint`, `wheel_right_joint` |
| `/isaacros/odom` | `nav_msgs/Odometry` | pub | frame `odom` · child `base_footprint` |
| `/isaacros/imu` | `sensor_msgs/Imu` | pub | frame `imu_link` |
| `/isaacros/scan` | `sensor_msgs/LaserScan` | pub | frame `base_scan` · 360° |
| `/isaacros/cmd_vel` | `geometry_msgs/Twist` | **sub** | Nav2·텔레오퍼 명령 |
| `/isaacros/battery_state` | `sensor_msgs/BatteryState` | pub | 12 V (가짜여도 형식 맞춤) |
| `/isaacros/map` | `nav_msgs/OccupancyGrid` | pub | SLAM 단계에서 |
| `/isaacros/ground_truth/pose` | `geometry_msgs/PoseStamped` | pub | **평가 전용** — 정책 obs 금지 |
| `/isaacros/reset` | `std_srvs/Empty` | sub | 에피소드 리셋 (L4 이후) |

TF 이름은 real과 같게 쓴다 (`map`, `odom`, `base_footprint`, `base_link`, `imu_link`, `base_scan`).
프레임 prefix를 붙이지 않고 **domain으로 분리**하는 방식.

## 노드 붙이는 법

```bash
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=26                 # sim
export ROS_LOCALHOST_ONLY=0

ros2 topic list | grep isaacros
ros2 topic hz /isaacros/scan
```

**모든 노드에 `use_sim_time:=true`**:

```bash
ros2 run rl_eval eval_recorder --ros-args \
  -r __ns:=/isaacros \
  -r cmd_vel_raw:=cmd_vel \
  -p use_sim_time:=true \
  -p output:=/tmp/sim_eval.csv
```

`__ns:=/isaacros`만 주면 `odom`·`imu`·`cmd_vel`이 자동으로 `/isaacros/...`로 붙는다.
`eval_recorder`는 `cmd_vel_raw`도 구독하므로 sim에선 `cmd_vel`로 remap.

Nav2 예:

```bash
ros2 launch nav2_bringup navigation_launch.py \
  namespace:=isaacros use_sim_time:=true \
  params_file:=<sim용 params>
```

SLAM 예:

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -r __ns:=/isaacros -p use_sim_time:=true
```

## QoS

- `/clock`: reliable, depth 10
- `/scan`·`/imu`: real 드라이버와 맞춰 일단 **reliable 기본**. (sim에서 best-effort로 바꾸면 real과 비교할 때 차이 생김)
- `/tf`·`/tf_static`: 기본
- `/cmd_vel`: reliable

## 지금 상태 / 다음

- **아직 bridge 구현 전** (L1 단계). 이 문서는 계약만 고정한 것.
- L3(센서)에서:
  - Isaac Lab `ImuCfg` → `imu_link`
  - `RayCasterCfg`(360°, `base_scan`) → `/isaacros/scan`
  - OmniGraph ROS 2 bridge로 `/clock`·`/tf`·`/odom`·`/joint_states` 발행
- L4(Env)에서 `/isaacros/cmd_vel` 구독 + `/isaacros/reset` 추가.
- bridge는 `tasks/common/ros_topics.py`의 상수만 import해서 이름이 한 곳에서만 바뀌게 한다.

## real과 동시에 볼 때

- real: `ROS_DOMAIN_ID=25`, sim: `26` → DDS 그래프가 분리됨
- 한 RViz에서 둘을 같이 보려면 domain bridge가 필요 (지금은 각각 RViz를 띄우는 게 단순)
- **sim과 real을 같은 domain에 올리지 말 것**: `odom`·`map` TF가 충돌함
