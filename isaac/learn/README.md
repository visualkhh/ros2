# Isaac 개념 실습 (L1~L7)

Pi/실기기 없이 **Isaac 안에서만** 개념을 하나씩 확인한다. 각 레슨은 "명령 1개 → 눈으로 확인 → 콘솔 출력"으로 끝난다.

| # | 개념 | 눈으로 보는 것 | 상태 |
|---|---|---|---|
| L1 | USD Stage · Prim · Articulation · Joint | 로봇이 prim 트리이고, 바퀴 joint를 직접 돌리면 몸체가 움직임 | ✅ |
| L2 | Physics · dt · substep | steps/frame을 1/2/4/8로 바꿀 때 `sim time / wall time`, 로봇이 빨라 보이는 이유 | ✅ |
| L3 | Sensor (IMU · LiDAR) | 라이다 광선/점, 자이로 값 | 예정 |
| L4 | Env API (obs/action/reward/reset) | 한 스텝마다 obs 6개·action 2개·reward 출력 | 예정 |
| L5 | Vectorization (clone) | 로봇 64대가 한 화면에서 동시에 움직임 | 예정 |
| L6 | Domain randomization | env마다 마찰·모터 gain이 달라 다른 궤적 | 예정 |
| L7 | Train → export → play | TensorBoard, checkpoint, ONNX | 예정 |

## L1 — 로봇 해부 ✅

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./steps/run.sh ../learn/l01_robot_anatomy.py
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
./steps/run.sh ../learn/l02_physics_time.py
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
  [l2] steps/frame=2 · sim 1.38s / wall 3.07s (0.45x) · x=+0.15 m
  ```
  `sim time / wall time` = 시뮬이 벽시계보다 몇 배 빠른가.
- 이 PC 실측: `2 → 0.45x`, `4 → 0.9x`, `8 → 1.8x`. 렌더(RTX)가 한 프레임에 ~35 ms 걸려서, 2스텝(16.7 ms)만으론 벽시계를 못 따라감.
- 배우는 것:
  - `dt = 1/120 s`는 **물리 한 스텝**의 시간. 화면 프레임과 별개.
  - `steps/frame`을 늘리면 같은 벽시계 동안 sim이 더 많이 흐름 → 로봇이 빨라 보임.
  - RL 학습은 **렌더를 끄고(headless)** 이 비율을 수십~수백 배로 올려서 돌림.
  - 실제 로봇과 비교할 땐 `sim time ≈ wall time`이 되게 맞춰야 함 (S2 주행 스크립트가 그렇게 함).
