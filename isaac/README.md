# isaac — 바퀴 드리프트 보정 RL (sim 보정 → 학습 → 실기기)

같은 `(v, w)` 명령을 넣어도 실제 로봇은 모터 지연 · 좌우 바퀴 차이 · 데드밴드 · 슬립 때문에 다르게 움직인다.
이걸 PC의 정책이 `[Δv, Δw]`로 보정한다. **학습은 Isaac sim에서, 추론은 PC(ROS)에서** — Pi에는 모델이 없다.

목적·합격 기준: [../documents/isaac/task-spec.md](../documents/isaac/task-spec.md)
(같은 경로를 RL 끄고/켜고 달려 회전 추종 RMS 20%↓ · 직진 RMS 악화 없음).

```text
calibration/  real·sim 응답 측정 → fit.json (sim을 real에 맞춤)
  → training/   Isaac Lab PPO 학습 (--fit으로 real 값 중심 랜덤화) → policy.onnx
  → ros2_ws/rl-only.sh   PC corrector가 ONNX로 보정 + 기록(raw.csv)
  → ros2_ws/report.py    Before/After 그림
```

## 1. 처음이면 — 강화학습 5단어

| 단어 | 뜻 | 여기서 |
|---|---|---|
| 환경(env) | 연습장 | 빈 바닥(마찰 패치) + 모터가 늦고 좌우가 다르고 통신이 늦는 가짜 로봇 512대 |
| 관측(obs) | 정책이 보는 것 | `[명령 v, 명령 w, 엔코더 v, 엔코더 w, 자이로 w, 명령 바뀐 뒤 시간]` — **실기기에서도 보이는 것만** |
| 행동(action) | 정책의 답 | `[Δv, Δw]` 보정량 (최대 0.10 m/s, 0.60 rad/s) |
| 보상(reward) | 점수 | 실제 몸체 속도가 명령에 가까우면 +, 보정이 크거나 출렁이면 − |
| 에피소드 | 한 판 | 10초. 명령은 0.5~2초마다 바뀜 (텔레오퍼 키 · 부드러운 곡선 · 정지) |

Isaac Sim = 물리 시뮬레이터, Isaac Lab = 그 위의 강화학습 도구(로봇 수백 대 복제). PPO는 그냥 쓴다.
개념 실습(하드웨어 없이): [learn/README.md](learn/README.md) — `$ISAACLAB_DIR/isaaclab.sh -p learn/l01_robot_anatomy.py --viz kit`.

## 2. 구조

```text
isaac/
├── run-calibration.sh        보정 실행기 (sim=IsaacLab · real=ROS · dummy/plot=.venv)
├── run-training.sh           학습 실행기 (IsaacLab, 기본 headless)
├── calibration/              sim/real 응답 측정 · fit · 비교 리포트       → calibration/README.md
│   ├── Calibrate.py          기록 실행기 + --fit 해석 (fit.json 1개 = real 값, 2개 = real − sim 차이)
│   ├── sensor/               DefaultSensorSim · DefaultSensorReal · DummySensor (Isaac/ROS 없이 검증)
│   ├── excitation/           측정 경로: 사각형 → 180° 턴 → 역주행 (54 s)
│   ├── parameter/ scene/ world/   바퀴 지연 모델 · 장면 · 랜덤 월드(키보드 주행, C 보정값 창)
│   └── report/report.py      fit(gain/지연/τ/데드밴드) + report.png · sim/real 비교
├── training/                 학습                                           → training/README.md
│   ├── Train.py              실행기 + task 공통 베이스 (task 찾기 · --watch · 결과 폴더 · P 키 스크린샷)
│   ├── onnx/onnx_export.py   actor → ONNX
│   └── rl_correction_wheel_drift/   task: env · PPO 설정 · train.py · FitPanel(보정값 창)
├── urdf/                     학습용 URDF(ROS용과 같은 내용 — urdf/check_same.sh) · robot_spec.py(로봇 숫자 한 곳)
├── learn/                    Isaac 개념 실습 L1~L5
├── artifacts/                산출물 (git 제외) — calibration/ · training/ · real/
├── requirements.txt · setup_venv.sh   실기기·리포트용 .venv (sim/학습은 IsaacLab venv)
```

로봇 숫자(`urdf/robot_spec.py`): 바퀴 반지름 0.033 m · 간격 0.287 m · 최대 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s · 20 Hz.
USD는 실행할 때마다 URDF에서 자동 변환된다 (`robot_spec.robot_usd()`, `urdf/usd/` git 제외).
따로 변환만: `$ISAACLAB_DIR/isaaclab.sh -p urdf/convert_urdf.py`.

## 3. 절차

### 3.1 보정 — sim을 real에 맞춤

#### real 기준 먼저 재기

`calibrate`는 sim+real을 같이 돌린다. real만 따로 잴 때는:

```bash
# Pi (또는 PC에서 tool/run-cli로)
cd ~/ros2/tool
./run-cli base start
./run-cli control manual
./run-cli route direct
./run-cli status              # state=MANUAL · route=direct · estop=off

# PC
cd /home/pank/workspaces/visualkhh/source/ros2/isaac
./run-calibration.sh sensor-sync-real --check-only   # [02-real] ROS_DOMAIN_ID=25 · state=MANUAL
./run-calibration.sh sensor-sync-real --label real_floor
```

- 로봇 앞뒤좌우 1 m 비우고, 약 54초 동안 사각형 2바퀴. 중간에 잡거나 바닥을 밀지 말 것.
- 결과: `artifacts/calibration/<시각>-real_floor/{raw.csv,fit.json,report.png}` (`--label`이 폴더 이름; 없으면 `<시각>-02-real`)
- 이 단계에서는 **`rl-only.sh`를 띄우지 않는다.** calibration은 `route direct`라 PC corrector를 안 거치고,
  `sensor-sync-real`이 직접 명령을 내고 기록한다. `rl-only.sh`는 §3.3 RL Before/After용이다.
- `base start`와 `ros2_ws/base-only.sh`는 **둘 중 하나만**. 이미 떠 있으면 `status`만 확인.
- `--label`은 CSV label이자 기본 폴더 이름. 정확한 경로는 `--output`(real) / `--out`(sim)으로 지정.
- `--check-only`는 로봇을 움직이지 않는다.

```bash
./run-calibration.sh calibrate --side 0.5        # sim(raw) + real 같은 경로 기록 → 비교
#   → artifacts/calibration/<시각>-calibrate/{sim,real}/{raw.csv,fit.json,report.png} · compare.png
./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json,<sim>/fit.json   # 차이를 sim 입력에 넣어 다시 기록
#   → 그 폴더의 fit.json ≈ real/fit.json 이면 맞춰진 것 (넣은 값은 base-fit.json)
```

- sim은 **`--fit`으로 준 값만** 쓴다. 숨은 보정 파일은 읽지 않는다 (없으면 raw plant).
- 차이 계산(`real,sim`)의 sim 기준은 반드시 `--fit` 없이 잰 것.
- 차이: gain = real/sim 비율, 지연·τ = real − sim, 데드밴드 = real 문턱 ÷ sim gain (sim 문턱이 더 크면 0).
- Isaac/ROS 없이 흐름만: `./run-calibration.sh calibration/sensor/DummySensor.py --fit ...`.
- 실기기 준비는 위 **real 기준 먼저 재기**: Pi `base start` · `control manual` · `route direct`.
- damping/effort는 **고정 모델값**이다 (`calibration/parameter/ActuatorParam.py`). calibration 변수로 쓰지 않는다.
- 자세히: [calibration/README.md](calibration/README.md) · [calibration/EXPERIMENT.md](calibration/EXPERIMENT.md).

### 3.2 학습

```bash
./run-training.sh --iterations 300 \
  --fit artifacts/calibration/<시각>-calibrate/real/fit.json,artifacts/calibration/<시각>-calibrate/sim/fit.json
./run-training.sh --watch --envs 16 --fit ...     # 창 + TRAINING FIT 창(보정값·랜덤 범위·마찰) · P 키 = 스크린샷
#   → artifacts/training/<시각>-rl_correct/ (checkpoint · policy.onnx · eval.json · TensorBoard · base-fit.json · screen_shot_*.png)
# RL 경로 지연까지 반영: --latency ../ros2_ws/artifacts/real/before.csv  (3.3의 Before 기록)
tensorboard --logdir artifacts/training
```

- `--fit real,sim`: 차이(real − raw sim)를 바퀴 τ·gain·지연(·데드밴드) 랜덤 범위의 **중심**으로 (폭 유지).
  sim fit은 **`--fit` 없이(raw)** 잰 것을 쓴다. 1개만 주면 real 값 그대로.
- 마찰: 기본 env별 랜덤 + 파랑(미끄럼)/주황(끈적) 네모 패치 4개 (`--no-friction`으로 끔). `--uneven`이면 낮은 턱.
- `--latency before.csv`: rl-only Before 기록(경로 rl, enable:=false)에서 관측 나이·corrector 처리 시간을 뽑아
  명령 지연 범위에 RL 경로 왕복(≈ 2×관측 나이 + 처리)을 더하고, 관측 지연 범위의 중심으로 둔다. PC·Pi 시계가 안 맞으면 거부.
- 학습이 끝나면 **sim Before/After**를 자동으로 돈다: 같은 env·같은 seed로 정책 끔(보정 0) / 켬(policy.onnx와 같은 그래프),
  실제 몸체 속도 vs 명령 RMS → `eval.json` (onnxruntime이 있으면 ONNX 출력이 같은지도 확인).
- `--iterations` 1회 = envs×64스텝 수집 + PPO 업데이트. `--envs` 기본 512 (`--watch`는 16~32 권장).
- 자세히: [training/README.md](training/README.md) · [training/rl_correction_wheel_drift/README.md](training/rl_correction_wheel_drift/README.md).

### 3.3 실기기 Before / After (ros2_ws)

```bash
cp artifacts/training/<시각>-rl_correct/policy.onnx artifacts/real/policy.onnx
cd ../ros2_ws
./rl-only.sh label:=before output:=artifacts/real/before.csv      # PC — 보정 없이 통과
./route.sh rl                                                     # 바퀴 명령이 PC를 거치게 (corrector 없으면 거부)
#   ... 같은 경로 주행 ...  → Ctrl+C
./rl-only.sh enable:=true label:=after output:=artifacts/real/after.csv   # isaac/artifacts/real/policy.onnx 사용
#   ... 같은 경로 주행 ...  → Ctrl+C
./route.sh direct
./report.py artifacts/real/before.csv artifacts/real/after.csv   # → report.png
```

**한 번 실행에 A/B** (같은 배터리·같은 바닥 — 더 공정): `./rl-only.sh label:=ab` (바이패스로 시작) → `./route.sh rl`
→ 주행 → `./policy.sh ../isaac/artifacts/real/policy.onnx` → 같은 주행 → `./policy.sh off` … → Ctrl+C → `./report.py`.
`policy.sh`는 ROS 파라미터(`enable` · `model_path`)를 바꾸고, corrector는 `model_path`가 바뀌면 바로 다시 로드한다
(없는 경로면 바이패스). 기록의 `rl_policy` 칸으로 report가 구간을 나눠 한 보고서에 겹쳐 그린다.
같은 명령으로 비교하려면 주행을 `run-calibration.sh sensor-sync-real`(사각형, `/cmd_manual_vel`)로 한다.

- 기록(`raw.csv`, 입력 1개 = 1줄): 입력 · 보정 출력 · 정책 원출력 · 드라이버 명령 · odom · 엔코더 · 바퀴 속도/전류
  · IMU · 모터 온도 · 배터리 · 상태 · 메시지 시각. 기본 경로 `ros2_ws/artifacts/real/raw.csv`.
- 정책 입출력: ONNX `obs [1, 6]` → `actions [1, 2]` (물리 보정량 m/s, rad/s). 정지 명령엔 보정 0.
- RL 경로에서 PC 응답이 0.3초 끊기면 로봇 정지 (cmd_manager 안전).

## 4. 아직 안 맞는 것 (학습 전에 결정 필요)

1. **sim actuator는 고정** — damping/effort는 실측해도 sim 응답이 변하지 않아 calibration 변수에서 뺐다.
   남은 차이는 friction/contact 쪽에서 본다.
2. (해결) ~~학습 env 응답 측정 도구 없음~~ — 같은 모터라 raw sim fit이 학습 env 응답. `--fit real,sim` 사용.
3. (해결) ~~RL 경로 지연 미반영~~ — `--latency before.csv`. 왕복은 관측 나이로 추정해 센서 주기 절반만큼 크게 잡힌다.
4. **데드밴드** — 측정 경로(사각형)엔 저속 구간이 없어 fit의 0은 "못 잼". 학습은 기본 범위(0~0.3) 사용.
5. (해결) ~~sim 검증 단계 없음~~ — 학습 끝에 sim Before/After 자동 → `eval.json`.

## 5. sim ROS 브리지 (계획 · 미구현)

Nav2·SLAM·`eval_recorder`를 sim에 직접 붙일 때 쓸 이름만 정해 둔다. 코드에는 아직 없다.

- `ROS_DOMAIN_ID=26` (real은 25 — 같은 WiFi에서 섞이지 않게), 네임스페이스 `/isaacros`, 모든 노드 `use_sim_time:=true`
- 토픽: `/isaacros/{clock, tf, tf_static, odom, imu, joint_states, scan, battery_state, cmd_vel(sub)}`
  · 평가 전용 `/isaacros/ground_truth/pose` (정책 obs 금지). TF 이름은 real과 같게(`odom`, `base_footprint`, `base_link`, `imu_link`, `base_scan`).
- `eval_recorder`를 붙일 때 입력과 출력을 같은 토픽으로 remap하지 말 것 (보정량이 0으로 기록됨).

## 6. 환경

- 학습 PC: RTX 5060 Laptop 8GB · Isaac Sim 6.1 · Isaac Lab 6.1 (`ISAACLAB_DIR`, 기본 `/home/pank/pa-course/IsaacLab-6.1`)
- Lab 6.1 쿼터니언은 **(x, y, z, w)** — 손으로 풀지 말고 `root_*_b` · `quat_from_euler_xyz` 사용
- 실기기·리포트용 `.venv`: `./setup_venv.sh` (ROS lyrical 상속 + matplotlib/numpy)
