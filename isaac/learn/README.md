# Isaac 개념 실습 (L1~L7)

Pi/실기기 없이 **Isaac 안에서만** 개념을 하나씩 확인한다. 각 레슨은 "명령 1개 → 눈으로 확인 → 콘솔 출력"으로 끝난다.

| # | 개념 | 눈으로 보는 것 | 상태 |
|---|---|---|---|
| L1 | USD Stage · Prim · Articulation · Joint | 로봇이 prim 트리이고, 바퀴 joint를 직접 돌리면 몸체가 움직임 | ✅ |
| L2 | Physics · dt · decimation | 같은 명령을 60/120/240 Hz로 돌릴 때 응답 차이 | 예정 |
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

다음 레슨(L2)은 "같은 명령인데 sim dt를 바꾸면 왜 응답이 달라지나"를 숫자로 본다.
