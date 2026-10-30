# Isaac 개념 실습 (L1~L7)

Pi/실기기 없이 **Isaac 안에서만** 개념을 하나씩 확인한다. 각 레슨은 "명령 1개 → 눈으로 확인 → 콘솔 출력"으로 끝난다.

| # | 개념 | 눈으로 보는 것 | 상태 |
|---|---|---|---|
| L1 | USD Stage · Prim · Articulation · Joint | 로봇이 prim 트리이고, 바퀴 joint를 직접 돌리면 몸체가 움직임 | ✅ |
| L2 | Physics · dt · substep | steps/frame을 1/2/4/8로 바꿀 때 `sim time / wall time`, 로봇이 빨라 보이는 이유 | ✅ |
| L3 | Sensor (IMU · LiDAR) | 벽에 찍히는 라이다 hit 점, 자이로/가속도 값 | ✅ |
| L4 | Env API (obs/action/reward/reset) | 한 스텝마다 obs 6개·action 2개·reward 출력 | 예정 |
| L5 | Vectorization (clone) | 로봇 64대가 한 화면에서 동시에 움직임 | 예정 |
| L6 | Domain randomization | env마다 마찰·모터 gain이 달라 다른 궤적 | 예정 |
| L7 | Train → export → play | TensorBoard, checkpoint, ONNX | 예정 |

## L1 — 로봇 해부 ✅

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./steps/run.sh l01_robot_anatomy.py
```

- 시작하면 `/World/Robot` 아래 **prim 트리**(경로·타입·적용된 물리 API)와 joint/link 목록을 출력
- 키:
  - `1` prim 트리 다시 출력
  - `2` joint 위치·속도, 몸체 pose, sim 시간 출력
  - `3` 왼쪽 바퀴만 +4 rad/s (1초) → 몸체가 오른쪽으로 돎
  - `4` 오른쪽 바퀴만 +4 rad/s (1초) → 몸체가 왼쪽으로 돎
  - `5` 양쪽 +4 rad/s (1초) → 앞으로 감
  - `0` 정지
- 배우는 것:
  - **Stage** = USD 문서, **Prim** = 그 안의 노드 (`/World/Robot/base_link` 등)
  - **Articulation** = joint로 연결된 링크 묶음. Isaac Lab은 이걸 `Articulation` 객체로 다룸
  - **Joint velocity target** = 우리가 실제 드라이버에 보내는 바퀴 rad/s와 같은 개념
  - `sim.step()` = 물리 시간을 `dt`만큼 진행

## L2 — 물리 시간 (dt · substep) ✅

```bash
./steps/run.sh l02_physics_time.py
```

- 로봇은 항상 같은 명령 `0.12 m/s`. 바뀌는 건 **한 화면(frame)마다 물리를 몇 번 돌리느냐**.
- 키:
  - `1` 한 프레임에 1스텝 → 천천히 보임
  - `2` 기본
  - `4` 빠르게
  - `8` 아주 빠르게
  - `0` 정지
- 콘솔:
  ```
  [l2] steps/frame=2 · inst 0.45x · cum 0.46x · sim 1.38s / wall 3.07s · x=+0.15 m
  ```
  `inst` = 최근 1초의 순간 비율, `cum` = 시작부터 누적. **키를 바꾼 효과는 `inst`로 봐야 함** (cum은 이전 구간에 끌려감).
- 이 PC GUI 실측: `2 → 0.45x`, `4 → 0.9x`, `8 → 1.8x`. 렌더(RTX)가 한 프레임에 ~35 ms 걸려서, 2스텝(16.7 ms)만으론 벽시계를 못 따라감.
- 같은 스크립트를 `--viz none`(렌더 없음)으로 돌리면: `2 → 1.0x`, `8 → 3.8x`. **렌더가 병목**임을 바로 비교할 수 있음.
- 기본은 벽 없는 바닥(`--room 0`) — 벽에 막혀 x가 멈추지 않게.
- 배우는 것:
  - `dt = 1/120 s`는 **물리 한 스텝**의 시간. 화면 프레임과 별개.
  - `steps/frame`을 늘리면 같은 벽시계 동안 sim이 더 많이 흐름 → 로봇이 빨라 보임.
  - RL 학습은 **렌더를 끄고(headless)** 이 비율을 수십~수백 배로 올려서 돌림.
  - 실제 로봇과 비교할 땐 `sim time ≈ wall time`이 되게 맞춰야 함 (S2 주행 스크립트가 그렇게 함).


## L3 — 센서 (IMU · LiDAR) ✅

```bash
./steps/run.sh l03_sensors.py
```

- 로봇에 실제 위치로 센서를 붙임:
  - **IMU**: `.../Geometry/base_footprint/base_link/imu_link` (URDF fixed joint)
  - **LiDAR**: `.../Geometry/base_footprint/base_link` + offset `(0.060, 0, 0.012)` = `base_scan`
    · 360° · 1채널 · 최대 3.5 m
- 키:
  - `3/4/5/0` = L1과 같은 바퀴 구동
  - `2` = 8방향 거리 + IMU 상세
  - `1` = 센서 설정(경로·offset·ray 수)
- 화면: 벽·장애물에 **빨간 점** = 라이다 hit. 로봇을 돌리면 점은 벽에 그대로 있고 스캔 패턴만 회전.
- 콘솔 예:
  ```
  [l3] t=  0.0s  IMU w=(-0.00,-0.00,+0.00) true w_z=+0.00 rad/s
       acc=(-0.00,-0.00,+9.81) m/s²
       LiDAR front=1.98 left=1.98 back=1.98 right=1.98 m | min=1.01 mean=1.94
       8방향: 앞 1.98 · 좌앞 1.73 · 왼쪽 1.98 · 좌뒤 2.75 · 뒤 1.98 · 우뒤 1.11 · 오른쪽 1.98 · 우앞 1.20
  ```
- 배우는 것:
  - **IMU** = 각속도(자이로) + 가속도(중력 포함). 정지 시 `az ≈ 9.81`.
  - **RayCaster** = 광선 360개를 쏘아 벽까지 거리를 계산. `debug_vis=True`면 hit 점을 빨간 구로 그림.
  - 센서는 **prim 경로**에 붙는다. USD 트리에서 실제 경로를 확인해야 함:
    `/World/Robot` 아래 `Geometry/base_footprint/base_link` (우리가 처음에 `/World/Robot/base_link`로 잘못 붙여서 `sim.reset()`에서 죽었음).
- `--no-imu` / `--no-lidar`로 센서를 하나씩 끌 수 있음 (문제 분리용).
