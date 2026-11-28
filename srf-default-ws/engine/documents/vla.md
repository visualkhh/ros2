# VLA: waffle-color policy, 4 Hz action chunk

> VLA는 **학습된 action policy**다. LLM처럼 plan 텍스트를 만들지 않는다.
> `vla_node`(C++)가 4 Hz 주기/큐/안전을 소유하고, Python sidecar는 `model.forward`만 한다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `srf-default-ws/engine/nodes/vla/vla_node.cpp` | 관측 수집, `/plan` 호출, 4 Hz 큐, 안전, 상태 |
| `srf-default-ws/tools/vla_server.py` | waffle-color policy HTTP sidecar `/plan` |
| `srf-default-ws/tools/learning/` | `policy.py`, `color_imitation.py`, `actions.py` |
| `srf-default-ws/artifacts/models/vla/` | `waffle-hf-vla.pt`, `waffle-color-73.npz` |
| `srf-default-ws/engine/nodes/llm_input/llm_input.cpp` | `vla_pick` → `vla/command` |

## 2. 경로

```text
LLM intent=vla_pick target="red block"
  → vla/command (String)
    → vla_node worker
      observation:
        camera/color/image_raw  (최신 프레임, base64)
        joint_states            (j1..j4, gripper)
        odom                    (linear.x, angular.z)
        state = [v, w, j1, j2, j3, j4, gripper]   (7)
      → POST http://127.0.0.1:8765/plan
          {image_b64, encoding, width, height, instruction, state, context}
            → ColorPolicy.plan()
              → 92 x 7 actions (base v/w + 4 joint + gripper)
      → actions_flat 파싱
      → 4 Hz timer: 큐에서 하나씩 꺼내
        cmd_vel_nav        (v, w)
        arm_joint_command  (JointState, 5 joints)
      → vla/status (String JSON)
```

## 3. 주기와 큐 (중요)

- **주기는 `vla_node`의 timer(0.25 s = 4 Hz)** 가 지킨다.
- 모델 추론은 워커 스레드에서 1회 92스텝(23초) 청크를 만든다.
- `SRF_DEFAULT_WS_VLA_REPLAN_STEPS`(기본 8)마다 큐를 비우고 최신 카메라/관절/odom으로
  다시 계획한다. 0이면 한 청크를 끝까지 open-loop로 실행한다. 재계획 중에는
  베이스를 정지하고 팔은 마지막 목표를 유지한다.
- 추론이 느려도 제어 루프는 모델을 기다리지 않는다. 큐가 비면 마지막 목표 유지/정지.
- OpenCR/모터 서보 루프는 더 빠르고(100 Hz), 4 Hz 상위 목표를 보간/유지한다.
- SmolVLA(10 Hz, 50스텝)로 바꿔도 같은 구조: 청크 큐 + prefetch.

## 4. 안전

- 관절 리밋: j1 ±2.827, j2 [-1.791,1.571], j3 [-0.942,1.382],
  j4 [-1.791,2.042], gripper [-0.010,0.019].
- base: |v| ≤ 0.26, |w| ≤ 1.82.
- `controller_manager/switch_controller`로 `arm_command_controller` 활성화
  (best effort; strictness 0).
- `vla/stop` 서비스: 큐 비우기, base zero, MoveIt/nav goal 취소.
- stale observation/타임아웃은 error.

## 5. 모델

`context`에는 `robot_context` JSON(TF/URDF 크기/라이다/카메라/팔/바퀴/배터리)이
문자열로 포함된다. sidecar는 행동 전에 이 상태를 함께 고려해야 한다.

```text
waffle-color-73.npz  (color trajectory head)
  parent: waffle-hf-vla.pt
  backbone: SmolVLM2-500M-Video-Instruct (4cde9d2...)
  input: 시작 RGB + 영어 지시 ("Pick up the red/blue cube...") + state 7
  output: 92 x 7 (4 Hz)
```

- **red/blue 전용**이다. 노란 퍽은 `pick`(결정적)으로 간다.
- 학습은 Isaac Sim의 로봇 전방 D435 뷰다. SRF 시뮬 D435와 조명/기하가 달라
  plan이 부정확할 수 있다. 그대로 성공을 보장하지 않는다.
- `vla_server.py`는 `image_features`에서 red/blue를 찾고, 없으면 400 에러를 낸다.
- CPU/MPS에서 500M 추론은 실시간 10 Hz가 아니다. 청크 큐로 주기를 지킨다.

## 6. sidecar 실행

로봇(Pi)에는 torch/transformers가 없으므로 **PC에서 sidecar를 띄우고 로봇이
PC LAN IP로 접속**한다. Pi와 PC가 분리된 구성에서는 반드시 `--host 0.0.0.0`.

```bash
# PC 최초 1회 + VLM/VLA 동시 실행
srf-default-ws/tools/ai-sidecars.sh start

# 개별 실행
srf-default-ws/tools/vla-venv/bin/python srf-default-ws/tools/vla_server.py   --host 0.0.0.0 --threads 8
# [vla_server] ready http://0.0.0.0:8765/plan
```

로봇 쪽:

```bash
SRF_DEFAULT_WS_VLA_URL=http://10.2.150.129:8765/plan ./build/srf_default_ws_real
```

PC 방화벽(UFW)이 켜져 있으면 8765를 열어야 한다.

```bash
sudo ufw allow from 10.2.150.0/23 to any port 8765 proto tcp
```

Pi에는 torch/transformers가 없어 sidecar를 직접 돌리지 않는다.
결정적 `pick`/`place`는 VLA 없이 동작한다.

모델/캐시:

```text
srf-default-ws/artifacts/models/vla/waffle-hf-vla.pt
srf-default-ws/artifacts/models/vla/waffle-color-73.npz
srf-default-ws/artifacts/models/vla/hf-cache/   # SmolVLM2 weights (HF_HOME)
```

## 7. 요청/응답 예시

요청:

```json
{
  "image_b64": "...",
  "encoding": "rgb8",
  "width": 160,
  "height": 120,
  "instruction": "Pick up the red cube and place it on the green square.",
  "state": [0.0, 0.0, 0.02, -0.1, 0.2, 0.1, 0.008]
}
```

응답:

```json
{"ok":true,"instruction":"Pick up the red cube...","steps":92,
 "actions":[[...7...], ...],
 "actions_flat":[...644 numbers...]}
```

`vla_node`는 `actions_flat`을 7개씩 잘라 92개 `Action`으로 만든다.

## 8. 한계와 다음

- 공개 HF `anuaju/smolvla_openmanipulator`는 탁자 외부 카메라 데이터라
  우리 D435에 그대로는 안 맞는다. 필요하면 우리 sim D435 시연으로 파인튜닝한다.
- VLA의 성공 pick/place는 아직 실측 전이다. 현재 검증된 것은
  sidecar 로드/응답, `vla_node` 경로, 4 Hz 큐/리밋/상태다.
- 노란 퍽은 `pick`(MoveIt 결정적)을 쓴다. VLA는 red/blue 학습 정책용이다.

## 8. VLA와 MoveIt의 관계

현재 `vla_node`는 **MoveIt goal을 보내지 않는다.**

- VLA sidecar가 `92 x 7` action chunk(베이스 v/w + 4관절 + 그리퍼)를 반환
- `vla_node`가 4Hz timer로 `cmd_vel_nav`(Twist)와 `arm_joint_command`(JointState)를 직접 publish
- MoveIt goal(`move_action`)은 `pick_place`의 결정적 pick/place, MoveIt 패널,
  `arm_preset` 등에서 사용

즉 VLA는 "목표 pose를 MoveIt에 넘기는 상위 planner"가 아니라 학습된 저수준
action policy다. 목표 pose 기반으로 바꾸려면 action head가 관절이 아니라
TCP pose/waypoint를 출력하도록 새 모델을 학습하고, `vla_node`가 그 목표를
`move_action`으로 보내야 한다. 현재 모델 출력(관절)을 MoveIt goal로 변환하려면
별도 IK가 필요하고, 학습된 trajectory를 버리게 된다.

- 안전한 결정적 pick/place: `pick_place` → MoveIt
- 학습된 red/blue cube pick: `vla` → 관절 action chunk (MoveIt 미사용)
