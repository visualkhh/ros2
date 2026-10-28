# Isaac 처음부터 (개념 → 실행)

ROS는 아는데 Isaac이 처음인 사람을 위한 문서. 수식 없음.

## 1. Isaac Sim vs Isaac Lab (한 줄씩)

- **Isaac Sim**: 로봇용 가상 세계 + 물리엔진. "가짜 로봇과 가짜 방"을 띄우는 프로그램.
- **Isaac Lab**: 그 위에서 **강화학습**을 돌리는 도구 모음. "로봇 연습장 + 코치" 역할.

관계: Lab이 Sim을 켜고, 로봇 수백 대를 복제해서 동시에 연습시킵니다.

## 2. 우리 폴더 지도

```
isaac/
├── tasks/rl_correct/     1번 연습장 (빈 방, 직진·회전 트래킹 보정)
├── tasks/nav2_rl/        2번 연습장 (장애물 방, Nav2 명령 다듬기)
├── tasks/common/tb3_simple.usda   연습용 가짜 로봇 (바퀴 2개짜리 단순 모델)
├── urdf/beebot/          진짜 로봇 설계도 (학습용, 무거워서 1번·2번엔 안 씀)
├── usd/beebot/           진짜 로봇 3D (나중에 씀)
├── scripts/train.py      학습 시작 버튼
├── scripts/play.py       학습된 애 구경하기
├── scripts/export.py     학습 결과 ONNX로 뽑기
├── logs/<task>/<날짜>/   학습 기록 (체크포인트 .pt)
└── export/<task>/        로봇에 올릴 모델 (.onnx)
```

## 3. 강화학습 5단어

| 단어 | 뜻 | 우리 예시 |
|---|---|---|
| 환경(env) | 연습장 | 빈 방 / 장애물 방 |
| 관측(obs) | 로봇이 보는 것 | `[v_cmd, w_cmd, v_odom, w_odom]` 숫자 4개 |
| 행동(action) | 로봇이 내는 답 | `[Δv, Δw]` 보정량 2개 |
| 보상(reward) | 칭찬·꾸중 점수 | 명령대로 움직이면 + , 어긋나면 − |
| 에피소드 | 한 판 | 10초 주행 후 리셋 |

**PPO**는 점수 잘 받는 방향으로 정책을 조금씩 고치는 알고리즘이고, 우리는 그냥 씁니다 (만들 필요 없음).

## 4. 전체 흐름 (외울 것)

```
① Sim에 로봇 수백 대 띄우기 → ② 멋대로 움직이며 점수 따기 → ③ 잘하는 쪽으로 정책 갱신
→ ④ ONNX 파일로 뽑기 → ⑤ Pi에 복사 → ⑥ ROS 노드가 불러서 실주행에 더하기
```

## 5. 명령어 (복붙용)

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac

# 1번 학습 (빈 방) — 창 보면서: 뒤에 --visualizer kit
./train_rl_correct.sh 300 --visualizer kit

# 2번 학습 (장애물 방)
./train_nav2_rl.sh 300 --visualizer kit

# 구경 (학습된 모델로 움직이는 거 보기)
./play_rl_correct.sh export/rl_correct/rl_correct_v1.onnx 4

# 체크포인트 → ONNX 뽑기
./export_rl.sh rl_correct logs/rl_correct/<날짜_시간>/model_29.pt rl_correct_v2.onnx
```

## 6. 로봇(ROS) 연결

- 학습 결과물은 숫자 덩어리(`[1,4] → [1,2]`) ONNX 파일 하나입니다.
- Pi `~/rl_models/`에 복사 → ROS 노드(`corrector`)가 `model_path`로 읽음.
- ROS 쪽은 이미 준비됨: `ros2_ws/src/rl_correction/` (4-obs), `nav2_rl_controller/` (5-obs).
- Before/After 비교: `rl_eval/eval_recorder` → `/tmp/before.csv` vs `/tmp/after.csv`.

## 7. 자주 묻는 것

- **Q. 왜 가짜 로봇으로 연습하나요?** A. 진짜 로봇으로 수천 번 부딪히면 부서지니까. 시뮬에서 수백 대로 병렬 연습합니다.
- **Q. 1번과 2번 차이가 뭔가요?** A. 1번은 "명령대로 안 움직이는 것" 보정, 2번은 "장애물 보고 속도 다듬기"입니다.
- **Q. 창이 안 떠요.** A. `--visualizer kit` (또는 `newton_gl`)을 붙이세요. 안 붙이면 헤드리스(창 없음)로 돕니다.
- **Q. beebot 풀모델은 언제 쓰나요?** A. 팔 쓰는 3번 Task 만들 때. 1·2번은 바퀴만 써서 가벼운 모델로 돌립니다.
