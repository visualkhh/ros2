# rl_maze — 미로 안 웨이포인트를 가장 빨리 따라가기 (task 이름 RlMaze)

목적: nav2의 **controller(로컬 플래너) 자리**를 RL로. planner가 준 경로(웨이포인트)를 받아 정책이 `[v, w]`를 직접 내고,
벽에 닿지 않으면서 가장 빨리 · 짧게 끝 점까지 간다. (rl_correction_waypoint는 기본 제어기 + 작은 보정 — 성격이 다르다.)

## 구조

```
training/rl_maze/
├── __init__.py     gym.register("RlMaze")
├── env.py          RlMazeEnv(BaseEnv) — 라이다(격자 + 동적 장애물) · 웨이포인트 진행 · 보상 · 충돌 · 화면
├── env_cfg.py      RlMazeEnvCfg — 관측 52 / 행동 2 · 보상 가중치 · 라이다 · 동적 장애물 · 256대 한 미로
├── dynamic.py      DynamicObstacles — 걷는 사람 · 공 (전역 1세트, 라이다/충돌/시각 마커)
├── ground.py       GroundCfg · spawn_friction_patches — 끈적(노랑)/미끄럼(파랑) 마찰 패치 (전역 1세트)
├── command.py      RlMazeCommand — paths.npz에서 env마다 랜덤 경로
├── agent.py        RlMazeAgentCfg — PPO (128×128)
├── maze_grid.py    미로 USD → 점유 격자(2 cm) · 벽까지 거리 · A* 경로 → usda/maze/grid.npz · paths.npz
├── calibration/ · robot_cfg.py · sim_robot.py · real_robot.py · dummy_robot.py · markers.py · usda/robot.usda
│                   rl_correction_waypoint에서 의도적으로 복사 (같은 로봇 — 바뀌면 같이)
└── usda/maze/      maze.usda · geo.usd (Lv2_bee_Assignment lv2_module6) · grid.npz · paths.npz · grid.png(확인용)
```

## 한 스텝 (20 Hz)

| 항목 | 값 |
|---|---|
| 관측 (52) | 다음 웨이포인트 5개 (로봇 기준 x, y ÷ 2 m) · 라이다 36방향 (÷ 3.5 m, `r - inflation`) · v_enc · w_enc · w_gyro · 이전 행동 2 · **inflation ÷ 0.5** |
| 행동 (2) | `[v, w]` ∈ [-1, 1] → × (0.26 m/s, 1.82 rad/s) · 후진은 20%까지 |
| 보상 | +2.0/m 진행 · +2 웨이포인트 통과 · +100 도착 − 0.10/스텝 − 200 실제 충돌 − 인플레이션 안쪽 겹침(soft) − 떨림 − 후진 |
| 끝 | 마지막 웨이포인트 도착 · 벽/사람/공 충돌(로봇 중심이 표면에서 0.17 m 안) · 50 s |
| 인플레이션 | `inflation_range=0.20~0.24 m` — env마다 **에피소드 시작 때** 뽑아 고정. obs 52번째 + 스캔 `r - inflation`. 사용자 입력값(실기 ROS 파라미터)과 같은 단위 |
| 웨이포인트 | 0.15 m 안이면 다음 · 다음 점이 더 가까우면 건너뜀 (모서리 질러가기) |
| 경로 | 랜덤 출발 · 도착 → A* (벽에서 0.20 m 이상 + 통로 가운데로) → 0.30 m 간격 · 2000개 중 **경로 전체 벽 0.34 m · 출발점 0.34 m 이상**만 |
| 세상 | 미로 하나에 256대 (env끼리 충돌 없음) · 위치 관측 잡음 2 cm · 2° · 라이다 잡음 1 cm |
| 동적 | 사람 3명 (0.25~0.65 m/s, 가끔 1~3 s 정지, 간격 1 m 이상) · 공 2개 (0.4~1.2 m/s, 벽 반사) — 8 s마다 한 명이 로봇 앞 1.8~3.2 m에 갑자기 · **2 s 이상 막히면 먼 곳으로 텔레포트 (교착 방지)** |
| 마찰 | 미로 전체에 노랑(끈적)·파랑(미끄럼) 패치 10개 — 바퀴와 평균 결합, seed 고정 |

## 실행

```bash
cd isaac
$ISAACLAB_DIR/isaaclab.sh -p training/rl_maze/maze_grid.py        # 미로 USD가 바뀌었을 때만 (grid.npz · paths.npz 다시)
./run-training.sh rl_maze --max_iterations 1000
./run-training.sh rl_maze --num_envs 16 --viz env.debug_vis=true env.lidar_vis=true   # 라이다 점군까지
./run-world.sh rl_maze --command keyboard --viz                    # 미로에서 직접 운전 · L = 라이다 점군
./run-world.sh rl_maze --viz                                       # 간단한 웨이포인트 추종으로 배경 구경
./run-calibration.sh rl_maze --robot sim                            # 같은 로봇 calibration (복사본)
```

`--command keyboard`는 `RlMazeEnv` 자체에서 `[v, w]`를 직접 넣는다 (CalibrationEnv 평면이 아님). 키:

```text
W/A/S/D  주행 · Shift 느리게 · X 정지 · `[`/`]` 속도 -/+
R        새 에피소드 (랜덤 경로 · 동적 장애물 새 배치)
F        카메라를 로봇 쪽으로
L        라이다 점군 켜기/끄기 — 시각화 360방향 (정책 입력은 36)
         raw=청록 · inflation=마젠타 · 3.5 m 반사 없음=회색 · 마진 안(inflation=0)=생략, 작은 점
```

`debug_vis` 화면의 바퀴 위 두 줄:

```text
아래 줄 = 그 바퀴의 유효 동마찰 (파랑=미끄럼 · 회색=보통 · 빨강=끈적)
위   줄 = 그 바퀴의 속도 (초록=전진 · 마젠타=후진, 길이 ∝ |v|/max_lin)
```

마찰 패치 위에 바퀴가 올라가면 아래 줄 색/길이가 바뀝니다. `run-world.sh ... --command keyboard --viz`에서는
터미널에도 `wheel fric L/R` · `jv L/R`가 같이 출력됩니다.

동적 장애물/패치 끄기·조절 (Hydra):

```bash
./run-training.sh rl_maze --num_envs 16 --max_iterations 5 --viz \
  env.n_people=0 env.n_balls=0 env.ground.friction_patches=0
```

## LiDAR 인플레이션 (사용자 입력)

`inflation_range = (0.20, 0.24) m` — 학습 중 env마다 **에피소드 시작 때** 뽑아 그 에피소드 동안 고정.
실기에서는 사용자가 `nav2_params.yaml` / ROS 파라미터로 넣는 값과 같은 단위(m)다.

- 관측 52번째 = `inflation / 0.50`
- 라이다 관측 = `clamp(raw_distance - inflation, 0, lidar_max)`
  → 정책이 보는 “안전거리”가 사용자 값에 맞게 줄어든다.
- 보상은 **인플레이션 안쪽 겹침(soft)**: true clearance < `inflation`일 때만 감점.
  실제 충돌(`collide_r=0.17`)은 hard 종료. 경로는 `max + path_margin(0.10) = 0.34`라
  정상 경로에서는 인플레이션 점과 겹치지 않고, 좁은 구간을 지날 때만 잠깐 감점된다.
- 경로는 `max(inflation_range) + path_margin(0.10) m` 이상 clearance만 사용 (기본 0.34 m, 가상 간격 0.10 m)
  → 사용자가 큰 인플레이션을 넣어도 경로가 “막힌 것”으로 보이지 않게.

### 실기

```yaml
# tb3_bringup/config/nav2_params.yaml · controller_server.FollowPath
lidar_inflation: 0.22      # [m] 사용자 입력 (costmap과 단위는 m, 값은 별개 — 로컬 안전 여유)
inflation_scale: 0.50      # 정규화 상수 — 사용자 값 아님, 학습/추론 동일
```

`rl_nav_controller`는 `scan_rays()`에서 `r - lidar_inflation`을 하고, obs 마지막에
`lidar_inflation / inflation_scale`을 넣는다. 모델은 **52차원**이어야 한다.

```bash
# 런타임 변경 (다음 주기부터 적용)
ros2 param set /controller_server FollowPath.lidar_inflation 0.24
```

- costmap의 `inflation_radius: 0.26`과 **단위는 m로 같지만 값은 별개**다.
  costmap은 planner가 footprint를 피하는 값, LiDAR inflation은 정책이 로컬에서 보는 **추가 안전 여유**다.
  둘을 같은 0.26으로 두면 좁은 통로가 가상으로 막힐 수 있어서 현재는 **0.20~0.24(기본 0.22)**,
  경로 최소 0.34로 **가상 간격 0.10 m**를 확보한다.
- 단, LiDAR가 로봇 중심보다 앞에 달려 있으면 전방 거리는 offset만큼 짧게 측정된다
  (현재 학습도 로봇 중심 기준이라 방향별 오차는 남아 있음).
- 기존 51차원 모델은 `lidar_inflation: 0`으로 두면 그대로 동작한다
  (노드는 `model_inputs < 52`면 inflation 입력을 생략). 새 52차원 모델은 위 값을 사용한다.

## 실기기 (nav2 controller)

`ros2_ws/src/rl_nav_controller` — nav2 `FollowPath` 플러그인. 안에 fallback controller(기본 DWB)를 품고
`enable=false`(기본) = fallback · `true` = 이 정책. `nav2_params.yaml`의 `fallback_controller: dwb|mppi|rpp|rotation_shim`.
```bash
./run-training.sh rl_maze --play --checkpoint artifacts/rl_maze/training/<시각>/model_N.pt --num_envs 1
#   → exported/policy.onnx (+ .onnx.data) → nav2_params FollowPath.model_path
#   → lidar_inflation은 학습 범위(0.20~0.24) 안, inflation_scale은 학습 값(0.50)과 동일
ros2 service call /controller_server/FollowPath/toggle std_srvs/srv/SetBool "{data: true}"
```

## 아직 없는 것

- 바퀴 응답 랜덤(gain · 지연 · τ — rl_correction_waypoint의 모터 모델)은 아직 안 넣음 — BaseEnv 드라이버 식 그대로.
- 라이다는 로봇 중심 기준으로 학습 — 실기기 LDS는 앞쪽에 달려 있어 조금 어긋난다 (방향만 tf로 맞춤).
- 관측은 52차원 단일 프레임 — 동적 장애물의 속도는 직접 안 보인다. 필요하면 scan history(2~3프레임)를
  추가해야 하지만, 실기 `rl_nav_controller` 관측 계약도 같이 바꿔야 한다.
