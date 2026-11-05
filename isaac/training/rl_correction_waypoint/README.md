# rl_correction_waypoint — 랜덤 웨이포인트를 이탈 없이 따라가기 (task 이름 RlCorrectionWaypoint)

목적: 웨이포인트까지 가는 **동안에도** 경로(앞 점 → 다음 점 직선)를 벗어나지 않게. 기본 제어기가 웨이포인트로 몰고,
정책이 그 명령에 보정 `[Δv, Δw]`를 더해 바퀴 gain 차이 · 지연 · 미끄러짐으로 생기는 드리프트를 잡는다.

## 구조

```
training/rl_correction_waypoint/
├── __init__.py   gym.register("RlCorrectionWaypoint") — 공식 train.py · play.py가 이 이름으로 찾음
├── env_cfg.py    RlCorrectionWaypointEnvCfg — 랜덤 폭 (중심 = 로봇 config 실측 응답) · 기본 제어기 · 보상 가중치 · 지형
├── env.py        RlCorrectionWaypointEnv(BaseEnv) — 추정 자세 · 기본 제어기 · 모터 모델 · 관측 · 보상 · 도착/실패
├── command.py    RlCorrectionWaypointCommand — env마다 랜덤 웨이포인트 4개 (각 0.5~1.5 m, 방향 랜덤)
├── calibration/  학습 전 1단계 측정 — command.py CalibrationCommand(네모) · env.py · env_cfg.py ·
│                 report.py (분석: python3 calibration/report.py <sim 폴더> <real 폴더> → fit · 그림 · RESPONSE_* 값)  (./run-calibration.sh rl_correction_waypoint --robot sim|real)
├── report.py     실기기 A/B 보고서 (../run-real-ab.sh가 부름) — 경로 · 끝점 오차 · 직진 휨 · 보정량
├── robot_cfg.py · sim_robot.py · real_robot.py · dummy_robot.py · usda/robot.usda   로봇 (설정 · sim · 실기기 ROS · 가짜 · USD)
└── agent.py      RlCorrectionWaypointAgentCfg(BaseAgentCfg) — PPO 값 (500회 기본)
```

## 한 스텝 (20 Hz) — 로봇 corrector goal 모드와 같은 약속

```
추정 자세 (시작 자세 기준) = 거리는 엔코더 v · 방향은 자이로 w 적분
  ↓ 다음 웨이포인트
기본 제어기: v = min(max_lin, k_dist·거리) · w = clip(k_heading·방향오차)   (방향오차 > 0.5 rad면 제자리 회전 먼저)
  ↓ + 정책 [-1, 1] × action_scale (0.10 m/s · 0.60 rad/s) — 기본 제어기가 멈춤이면 0
  ↓ [명령 지연] → 바퀴 (드라이버 식) → 모터 (좌/우 gain · 데드밴드 · 1차 지연 τ) → 물리
추정 위치가 5 cm 안이면 다음 웨이포인트 · 4개 다 가면 끝
```

| 항목 | 값 |
|---|---|
| 관측 (10) | `[gx, gy, px, py, yaw, v_enc, w_enc, w_gyro, prev_a0, prev_a1]` (센서 지연 포함) |
| 행동 (2) | `[Δv, Δw]` 정규화 [-1, 1] |
| 보상 (진짜 위치로 채점) | − 2·경로 이탈 [m] − 0.5·방향 오차 + 2·진행 [m] + 도착 1 − 5·도착 순간 진짜 오차 [m] − 보정 크기 · 떨림 |
| 종료 | 4개 다 도착 · 경로에서 1 m 이상 이탈(실패 −5) · 40 s |
| 랜덤 (에피소드마다) | 바퀴별 τ · gain(실측 중심 0.9927/0.9938) · 데드밴드 · 명령/센서 지연 · 반지름/간격 기준 오차 · 자이로 영점 ±0.005 |
| 지형 · 마찰 | 평평 + 요철(턱 1~3 cm) 칸 · 언덕 없음 (`env.ground.uneven=false` = 평면만) · 마찰 패치 env마다 8개 (노랑 끈적 1.1 · 파랑 미끄럼 0.2, 출발점 0.3~3 m, `env.ground.friction_patches=0` = 끔) · env마다 바퀴 마찰 랜덤 |

**/odom pose를 안 쓰는 이유**: 드라이버의 /odom 위치·방향은 엔코더만 적분한 값이라 바퀴 gain 차이로 실제로 휘어도 "똑바로"로 나온다.
방향을 자이로로 봐야 휨이 보인다. 미끄러져서 생긴 거리 오차는 여전히 못 본다 (나중에 SLAM 위치를 쓰면 됨).

## 실행

```bash
cd isaac
./run-training.sh rl_correction_waypoint --max_iterations 500                                   # task 폴더 → gym RlCorrectionWaypoint (창 없음, 512대)
./run-training.sh rl_correction_waypoint --num_envs 16 --max_iterations 5 --viz kit env.debug_vis=true   # 창 (노란 점 = 웨이포인트 경로)
./run-training.sh rl_correction_waypoint --play --checkpoint artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --num_envs 16 --viz kit env.debug_vis=true
./run-training.sh rl_correction_waypoint --max_iterations 500 --label v1 --snap 60   # 60초마다 화면 → training/<시각>_v1/snap/ + latest.png
tensorboard --logdir artifacts/rl_correction_waypoint/training          # Metrics/cross_track (경로 이탈 평균) · Metrics/waypoints_done (평균 도착 수)
```

## calibration (학습 전에 먼저) — sim · real에 같은 네모를 넣고 차이를 `robot_cfg.py` `RESPONSE_*`에

```bash
cd isaac
./run-calibration.sh rl_correction_waypoint --robot sim                  # sim (창 없음 · 보려면 --viz)
./run-calibration.sh rl_correction_waypoint --robot real --check-only    # real: 안 움직이고 상태만 → "check-only — 준비됨"
./run-calibration.sh rl_correction_waypoint --robot real --label real_floor   # real 측정 (약 54 s)
#   → artifacts/rl_correction_waypoint/calibration/<시각>_<label>/raw.csv
python3 training/rl_correction_waypoint/calibration/report.py artifacts/rl_correction_waypoint/calibration/<sim 폴더> artifacts/rl_correction_waypoint/calibration/<real 폴더>
#   → fit.json · report.png (폴더마다) · compare.png (real 폴더) · robot_cfg.py에 붙일 RESPONSE_* 값
```

### real 준비 순서 (`거부: MANUAL 아님 (state=None)`이면 1~2가 안 된 것)

`state=None` = PC가 로봇의 `/state_control`을 못 받음 → Pi에서 베이스(tb-only: 드라이버 · cmd_manager · state_manager)가 안 떠 있거나
네트워크(DDS)가 안 닿음. 로봇이 꺼져 있으면 아무것도 안 된다.

1. **로봇 켜기**: Raspberry Pi 전원 + 부팅 기다리기 (1~2분). OpenCR(모터 보드)은 로봇을 가만히 둔 채 켠다 (IMU 영점).
   PC에서 닿는지: `ping pak.local` (안 되면 같은 Wi-Fi인지)
2. **베이스 켜기** — Pi에서 직접 또는 PC에서 (PC의 run-cli는 켜기/끄기를 SSH로 Pi에 넘긴다). 2 · 4의 run-cli는 어느 쪽이든 같다:
   ```bash
   # Pi에서:  ssh pak@pak.local  →  cd /home/pak/ros2/tool
   # PC에서:  cd ~/workspaces/visualkhh/source/ros2/tool
   ./run-cli base start          # 바퀴 · odom · 라이다 (tb-only)
   ./run-cli status              # 로봇 수신 · odom 확인
   ```
3. **PC에서 RL 노드를 바이패스로** (보정 없이 통과 — 실제로 정책을 쓸 때와 같은 경로 · 같은 PC 왕복 지연을 재려고):
   ```bash
   cd ~/workspaces/visualkhh/source/ros2/ros2_ws
   ./rl-only.sh enable:=false label:=calib   # 바이패스 필수 (기본은 정책 켬 — 보정이 섞인다) · 확인: ./policy.sh status
   ```
4. **제어 MANUAL · 경로 rl** (run-cli, Pi든 PC든):
   ```bash
   ./run-cli control manual
   ./run-cli route rl            # 명령 → Pi cmd_manager → PC rl_corrector(바이패스) → Pi → 모터. corrector가 없으면 거부
   ```
5. **check-only** → `준비됨`이 나오면 로봇 주변 1 m 비우고 측정. 사각형(한 변 0.5 m) → 180° 턴 → 역주행, 중간에 잡지 말 것.
6. 끝나면 베이스 · rl-only는 그대로 둬도 된다 (끄기: `./run-cli base stop` · rl-only는 Ctrl+C).

## 실기기

- corrector goal 모드(`ros2_ws/src/rl_correction`)가 같은 관측 · 기본 제어기 · 스케일을 쓴다. `goal_mode=absolute`(기본)로
  첫 웨이포인트 때 시작 자세를 잡고, `/rl_goal`(Point, 시작 자세 기준 x · y)을 하나씩 보낸다. `/rl_goal_stop`으로 끝.
- 웨이포인트를 차례로 보내는 쪽(도착하면 다음 점)은 아직 없다 — 다음 단계.
- 기본 제어기 파라미터(k_dist · k_heading · turn_thresh · goal_tol)와 action_scale은 `env_cfg.py`와 corrector가 같아야 한다.

## 학습 후 — 실기기에 정책 올리기 · A/B

```bash
# 1) 학습 → 재생으로 ONNX 내보내기 (PC isaac/)
./run-training.sh rl_correction_waypoint --max_iterations 500
./run-training.sh rl_correction_waypoint --play --checkpoint artifacts/rl_correction_waypoint/training/<시각>/model_499.pt --num_envs 1   # → <시각>/exported/policy.onnx (+ .onnx.data)
# 2) 로봇 쪽은 calibration과 같게: base start · route rl (MANUAL)
```

정책을 켜는 방법은 두 가지 (PC `ros2_ws/`):

| 방법 | 명령 | 언제 |
|---|---|---|
| rl-only를 정책으로 띄움 | `./rl-only.sh enable:=true label:=after model:=../isaac/artifacts/rl_correction_waypoint/training/<시각>/exported/policy.onnx` | 처음부터 B만 |
| 띄운 채 켬/끔 | `ros2 service call /rl_corrector/toggle std_srvs/srv/SetBool "{data: false}"` (바이패스) · `"{data: true}"` (ONNX) — = `./policy.sh off` · `on` · 모델 교체 `./policy.sh <정책.onnx>` | **한 번 실행 안에서 A/B** (같은 배터리 · 바닥) |

- 기본은 켬(ONNX 적용). toggle = corrector의 `~/toggle` 서비스(nav2 collision_monitor와 같은 `std_srvs/SetBool`) — 다시 띄울 필요 없음, 다음 명령부터 적용. 기록 CSV의 `rl_policy` 칸으로 A/B 구간이 나뉜다 → `./report.py`.
- `policy.onnx.data`(가중치)가 `policy.onnx` 옆에 같이 있어야 한다. 로드 결과는 rl-only 로그의 `ONNX loaded: … mode=goal`.
- 이 정책은 관측 10차원이라 corrector가 **goal 모드**로 뜬다 — 키보드 · 네모(`/cmd_manual_vel`) 명령을 보정하는 게 아니라,
  `/rl_goal`(Point, 시작 자세 기준 x · y [m])을 받아 기본 제어기 + 보정으로 그 점까지 간다. 끝: `/rl_goal_stop`.
  ```bash
  ros2 topic pub --once /rl_goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"   # 도착하면 다음 점을 보낸다
  ros2 topic pub --once /rl_goal_stop std_msgs/msg/Empty "{}"
  ```

**같은 입력으로 A/B (실기기, run-calibration)** — 입력 = calibration 네모, 차이 = corrector 보정 켬/끔
```bash
./run-calibration.sh rl_correction_waypoint --robot real --label A                       # A: 바이패스 (--onnx 없음)
./run-calibration.sh rl_correction_waypoint --robot real --onnx <policy.onnx> --label B  # B: 그 정책으로 보정
```
- 측정 직전에 RealRobot이 corrector를 설정한다 (`model_path` + `~/toggle`). 기록 칸 `rl_onnx` = 모델 파일 이름 · `bypass` · `direct`(corrector 없음).
- 필요: Pi `route rl` + PC `./rl-only.sh`. `--onnx` 없이 corrector도 없으면(route direct) 보정 없이 그대로 측정.
- **이 task 정책(웨이포인트, goal 모드)은 속도 입력(네모)을 보정하지 않는다** — 위 A/B는 track 모드 정책(관측 6)일 때 차이가 난다.

**goal 모드 A/B (이 task 정책)** — 같은 `/rl_goal`을 A(바이패스) · B(정책)로 두 번

```bash
# A — 모델은 goal 모드 판별용으로 반드시 넣는다 (enable=false = nominal controller, 보정 0)
cd ros2_ws
./rl-only.sh enable:=false label:=wp_A output:=artifacts/real/wp_A.csv model:=<policy.onnx>
# Pi: control manual · route rl
ros2 topic pub --once /rl_goal geometry_msgs/msg/Point "{x: 1.0, y: 0.0, z: 0.0}"   # 시작 자세 기준
# rl-only 로그에 "goal 도착" → 
ros2 topic pub --once /rl_goal_stop std_msgs/msg/Empty "{}"
./rl-only.sh stop

# B — 로봇을 A와 같은 자리·방향으로 돌려놓고
./rl-only.sh label:=wp_B output:=artifacts/real/wp_B.csv model:=<policy.onnx>
# 같은 goal → "goal 도착" → stop
```

```bash
cd isaac
python3 training/rl_correction_waypoint/report_goal.py \
  ../ros2_ws/artifacts/real/wp_A.csv ../ros2_ws/artifacts/real/wp_B.csv --goal 1.0,0.0
# → compare_goal.png + 경로/최대 이탈/끝점/정책 출력 표
```

- goal은 **첫 goal 때의 로봇 자세 기준**: `x` 앞, `y` 왼쪽 [m]. `y=0`이면 그 방향 직진이라 A/B 경로 오차가 의미 있음.
- `rl_goal_stop`은 `start_pose`를 리셋한다. A/B는 새 `rl-only` 프로세스로 하는 게 깔끔.
- 웨이포인트 자동 송신기는 아직 없다 — 점마다 손으로 `/rl_goal`을 보낸다.

**아직 없는 것**
- **웨이포인트 송신기**: 도착하면 다음 `/rl_goal`을 보내는 쪽. 지금은 손으로 한 점씩.
- **A = 보정 0인 기본 제어기**: goal 모드에서 toggle 끔(바이패스)은 nominal controller만 돈다 (보정 0 = 학습 env의 행동 0) — 같은 웨이포인트로 A/B 가능.
