# training — RL 학습

Isaac Lab에서 돌리는 RL 학습 task를 **폴더별**로 둔다. 여기는 **학습만** 한다.
공통 로봇 값·산출물은 `isaac/` 루트에 있다.

```text
calibration/ 보정 (sim plant를 real에 맞춤 → real fit.json)
  → training/rl_correction_wheel_drift/ 학습 (Isaac Lab, PPO, --fit)
  → ONNX
  → PC의 rl_correction 노드 (track 모드)
  → /cmd_vel_rl_out
  → Pi cmd_manager → 모터
```

- Pi는 **모델을 돌리지 않는다**. Pi는 base·cmd_manager·드라이버만.
- RL 추론은 **PC**에서 한다. 학습은 sim에서 하고, real에서는 ONNX만 실행한다.

## 구조

```text
training/
├── Train.py                    실행기 + task 공통 베이스 — python -m training <task> [옵션...]
│                               (task 찾기 · --watch · AppLauncher · 결과 폴더 · P 키 스크린샷)
├── __main__.py
├── onnx/onnx_export.py         actor → ONNX (train.py 끝에서 씀)
└── rl_correction_wheel_drift/  바퀴 드리프트 보정 (track) — 자세한 건 그 폴더 README
```

- task = `train.py`가 있는 폴더. `Train.main()`(베이스로 호출)은 그 폴더의 `train.main(argv)`로 넘기기만 한다.
- 새 task는 자기 폴더에 env·장면(`task/`)·URDF를 따로 둘 수 있다. 로봇은 env cfg에서 `robot_spec.robot_usd("training/<task>/urdf/robot.xacro", "<task>")`(메시는 `<xacro 폴더>/meshes`).
- 새 task는 `train.py`에서 `Train`을 상속해 `add_args` / `before_launch` / `run`만 구현하고
  `def main(argv=None): return MyTrain.main(argv)`를 둔다. 결과 폴더는 `self.run_dir`.
- 보정값(`--fit`) 해석은 task가 한다 (env cfg가 task마다 다르다).

## 실행

```bash
cd isaac   # Lab env (./isaaclab.sh -p ...)

# 목록
./isaaclab.sh -p -m training

# 학습 (기본: headless, 512 envs, 마찰 패치 켜짐 — 끄려면 --no-friction)
./isaaclab.sh -p -m training rl_correction_wheel_drift --iterations 300 \
  --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json

# 창 보면서 (512 envs는 무거움 → 16~32 권장)
./isaaclab.sh -p -m training rl_correction_wheel_drift --watch --envs 16 \
  --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<sim>/fit.json

tensorboard --logdir artifacts/training
```

- 결과: `artifacts/training/<시각>-rl_correct/` — checkpoint(`model_<iter>.pt`, 50 iter마다), `policy.onnx`, `eval.json`(sim 정책 끔/켬 비교),
  `--fit`/`--latency`면 `base-fit.json`(적용한 fit · 지연 · 랜덤 범위)
- `--fit real/fit.json,sim/fit.json` — real−sim 차이를 학습 env 랜덤 범위의 중심으로. sim fit은 raw(`--fit` 없이) 잰 것.
- `--watch`는 학습 장면(로봇들, 파랑/주황 패치)을 Isaac 창으로 보여주고,
  **TRAINING FIT** 창(읽기만)에 fit 값 · 적용된 랜덤 범위 · 마찰 설정을 띄운다.
  창에서 **P** 키 → `artifacts/training/<시각>-<exp-name>/screen_shot_<시각>.png` (뷰포트 캡처).
  headless일 때는 터미널 출력 + `base-fit.json`으로 남는다.
- `--snap SEC`: 창 없이도 SEC초마다 `snaps/<시각>.png` + `latest.png` (화면 없는 Kit visualizer + `--enable_cameras` 자동, ~20% 느려짐).
- `--slope DEG`: env마다 바닥판을 랜덤 방향 · 0~DEG° 랜덤으로 기울임 (패치·`--uneven` 턱도 같이). 기본 3, 0 = 평평.
- `--gyro-bias RAD_S`: 에피소드마다 자이로 영점 오차 ±RAD_S (기본 0.05, 0 = 끔).

### 화면 (`--watch` · `--snap`)

`--watch`(창)나 `--snap`(스냅샷)일 때만 env의 `debug_vis`를 켠다 — headless 학습엔 영향 없음.

| 표시 | 뜻 |
|---|---|
| 초록 막대 | 지금 명령 (v, w)대로라면 갈 방향 (길이 ∝ 속도) |
| 파랑 막대 | 실제 몸체 속도로 가는 방향 |
| 노란 점 | **목표 경로** — 랜덤 명령이 바뀐 순간의 자세에서 그 명령대로 유지 시간(0.5~2 s) 동안 갈 길 (env마다 12점) |

로봇이 노란 점에서 벗어나는 만큼이 드리프트다. 학습이 될수록 줄어야 한다. 정지 명령인 env는 경로를 안 그린다.
구현: `rl_correct_env.py`의 `_set_debug_vis_impl` / `_debug_vis_callback` (Isaac Lab `DirectRLEnv` 훅) +
`calibration/scene/CommandMarkers.py` (calibration sim과 공용).

### 직접 운전 (학습 env)

`../run-world.sh --fit $FIT --uneven --policy <결과>/policy.onnx` — `--drive`: 로봇 1대 · 키보드 명령 · C 정책 켬/끔. 자세히: [../README.md](../README.md) 3.4c.

### 학습 뒤 sim에서 play

```bash
./run-calibration.sh sensor-sync-sim --watch --fit <real>/fit.json,<sim>/fit.json --policy artifacts/training/<시각>-rl_correct/policy.onnx
```

같은 사각형을 corrector와 같은 규칙으로 달린다. A/B 비교·옵션: [../calibration/README.md](../calibration/README.md) "sim play".

## 보정값과 RL의 관계

- 바닥/바퀴/캐스터 **마찰**은 `FrictionParam` 기본값 (숨은 보정 파일은 안 읽는다). 기본은 env별 랜덤 + 패치.
- 바퀴 **τ·gain·지연·데드밴드**는 env가 에피소드마다 랜덤으로 뽑는다.
  `--fit`을 주면 그 범위의 **중심을 real fit 값**으로 옮긴다 (폭은 그대로).
- RL은 그 위의 **잔차**(좌우 비대칭, 슬립, 드리프트)를 학습한다.

RL이 보정 자체를 대체하지는 못한다. wheel radius/sep 스케일, 바닥 슬립, 캐스터 마찰은
odom만으로 관측이 안 되므로 sim plant를 먼저 맞춘다 (`calibration/`).

## PPO (`agents/rsl_rl_ppo_cfg.py`)

- 512 envs (기본), 20 Hz 정책 주기, `num_steps_per_env = 64`, `max_iterations = 300`
- actor/critic 64×64, lr `3e-4`, gamma `0.99`, lambda `0.95`, entropy `0.005`, clip `0.2`
- 학습 끝나면 actor를 ONNX로 자동 export (입력 `[1, 6]`, 출력 `[1, 2]` 물리 보정량 m/s, rad/s)

## real 배포 (PC / Pi) — 학습 후 참고

```text
Pi:  base + cmd_manager,  /cmd_vel_rl_out → 드라이버 → 바퀴
PC:  rl_correction/corrector,  /cmd_vel_rl_in + /odom + /imu → ONNX → /cmd_vel_rl_out
```

```bash
# Pi — RL 경로로
ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli base start && ./run-cli control manual && ./run-cli route rl'

# PC — corrector (ONNX 입력이 6차원이면 자동 track 모드)
cd ros2_ws && colcon build --packages-select rl_correction && source install/setup.bash
source /opt/ros/lyrical/setup.bash && export ROS_DOMAIN_ID=25 && source ../tool/peers.sh
ros2 launch tb3_bringup rl-only.launch.py enable:=true \
  model:=<isaac>/artifacts/training/<시각>-rl_correct/policy.onnx

# 끝나면 복귀
ssh pak@pak.local 'cd ~/ros2/tool && ./run-cli route direct'   # PC corrector: Ctrl+C
```

## 문제 생기면

| 증상 | 확인 |
|---|---|
| 학습이 느림 | `--envs` 줄이기, `--watch` 끄기 |
| 보상이 안 오름 | `Metrics/track_err`, action std |
| real에서 안 됨 | ONNX 입력 차원(6), `corrector_node` 모드, ONNX 경로 |
| sim이 real과 다름 | `calibration/`으로 floor 보정 먼저 |

## 평가 (track)

- `v_enc`, `w_enc` RMS · `w_gyro` RMS · 직진/회전 bias
