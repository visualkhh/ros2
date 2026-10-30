# rl — RL task 모음

Isaac Lab에서 돌리는 RL task를 **폴더별**로 둔다. 공통 로봇 값·산출물은 `isaac/` 루트에 있다.

| 폴더 | 내용 | 상태 |
|---|---|---|
| `wheel_drift/` | **바퀴 드리프트 보정** (rl_correct v2) — obs 6, action 2 | 센서 싱크 후 학습 |

```
rl/
├── wheel_drift/          # 바퀴 드리프트 보정
│   ├── task/             # env (rl_correct v2)
│   ├── agents/           # PPO cfg
│   ├── train.py · play.py · export.py · smoke.py
│   ├── train.sh · play.sh · export.sh
│   └── README.md
└── README.md             # 이 파일
```

- 공통: `tasks/common/robot_spec.py` (로봇 값) · `tasks/common/onnx_export.py`
- 산출물: `logs/<task>/` · `export/<task>/` (git 제외)
- 새 task 추가: `rl/<name>/`에 같은 구조로 넣는다. 공통 로봇 값은 `tasks/common`을 import.

## 실행 (wheel_drift)

```bash
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./rl/wheel_drift/train.sh 300 [--visualizer kit]
./rl/wheel_drift/export.sh logs/rl_correct/<날짜>/model_299.pt rl_correct_v2.onnx
./rl/wheel_drift/play.sh export/rl_correct/rl_correct_v2.onnx 4
```
