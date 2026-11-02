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

로봇 숫자(`urdf/robot_spec.py`): 바퀴 반지름 0.033 m · 간격 0.287 m · 최대 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s · 정책 20 Hz · 물리 480 Hz.
USD는 실행할 때마다 URDF에서 자동 변환된다 (`robot_spec.robot_usd()`, `urdf/usd/` git 제외).
따로 변환만: `$ISAACLAB_DIR/isaaclab.sh -p urdf/convert_urdf.py`.

## 3. 절차 (2026-11-02 실제로 돈 순서)

real 측정 → 같은 경로를 sim으로 측정 → 차이(`--fit real,sim`)로 학습 → 실기기에서 A/B.
경로는 `isaac/` 기준. 예시 폴더 이름은 그날 실제 결과.

### 3.0 준비 (처음 한 번)

```bash
cd isaac && ./setup_venv.sh                                               # real 측정·리포트용 .venv
cd ../ros2_ws && colcon build --packages-select rl_correction rl_eval tb3_bringup   # PC
```

PC·Pi 시계 동기(chrony)는 `--latency`(3.4)를 쓸 때만 필요하다. 2026-11-02 기준 Pi가 약 52 ms 빠름
(기록의 메시지 지연이 음수 · fit의 `age` ≈ −0.05로 보임).

### 3.1 real 측정 — 실기기 사각형

```bash
# 로봇: 베이스 · MANUAL · 직결 (Pi에서 또는 PC tool로)
./tool/run-cli base start  &&  ./tool/run-cli control manual  &&  ./tool/run-cli route direct
# PC
cd isaac
./run-calibration.sh sensor-sync-real --check-only     # 움직이지 않고 상태만 확인 (state=MANUAL)
./run-calibration.sh sensor-sync-real --label real_floor
#   → artifacts/calibration/<시각>-real_floor/{raw.csv, fit.json, report.png}   (--label 없으면 <시각>-02-real)
```

- 로봇 주변 1 m 비우기. 사각형(한 변 0.5 m) → 180° 턴 → 역주행, 약 54초. 중간에 잡지 말 것.
- 이 단계엔 `rl-only.sh`를 띄우지 않는다 (직결, `sensor-sync-real`이 직접 명령·기록).
- 볼 것(`fit.json`): gain 좌/우 · 지연 · τ. 2026-11-02 결과(`20261102_140931-02-real`): gain 0.994/0.995 · 지연 0.097 s · τ 0.045 s.
  τ는 20 Hz 기록의 한계값(이보다 빠른 건 구분 못 함) · 데드밴드 0은 "못 잼"(저속 구간 없음).

### 3.2 sim 측정 — 같은 명령, 보정 없이 (raw)

```bash
./run-calibration.sh sensor-sync-sim --viz none --label sim        # 창 보려면 --viz none 빼기
#   → artifacts/calibration/<시각>-sim/{raw.csv, fit.json, report.png}
./run-calibration.sh plot-compare artifacts/calibration/<시각>-sim/raw.csv artifacts/calibration/<real>/raw.csv
```

- 사각형 기본값(`--side 0.5 --speed 0.15 --turn 1.0`)이 real과 같아 **같은 명령**이 재생된다.
- `--fit` 없이 잰다 — 이게 3.4 차이 계산의 sim 기준. 정상이면 "이상적인 모터": gain ≈ 1 · 지연 0 · 사각형이 몇 cm 안에서 닫힘.
  (2026-11-02 `20261102_144928-sim_480`: gain 1.002/1.001 · 회전 자이로/명령 0.956, real 0.950)
- sim은 **`--fit`으로 준 값만** 쓴다. 숨은 보정 파일은 읽지 않는다.
- real·sim 한 번에: `./run-calibration.sh calibrate --side 0.5` → `<시각>-calibrate/{sim,real}/…` · `compare.png`.
- sim이 real과 크게 다르면 먼저 sim 물리를 의심한다 (2026-11-02에 고친 것 — 아래 4번).

### 3.3 (선택) 맞춰졌는지 확인

```bash
./run-calibration.sh sensor-sync-sim --viz none --fit artifacts/calibration/<real>/fit.json,artifacts/calibration/<시각>-sim/fit.json
#   → 새 폴더의 fit.json ≈ real의 fit.json 이면 OK (넣은 값은 base-fit.json)
```

`--fit` 형식: 경로 1개 = 그 값 그대로 · 경로 2개(쉼표, 띄어쓰기 없이, **real 먼저**) = real − sim 차이
(gain은 비율, 지연·τ는 차, 데드밴드 = real 문턱 ÷ sim gain).

### 3.4 학습

```bash
FIT=artifacts/calibration/<real>/fit.json,artifacts/calibration/<시각>-sim/fit.json
./run-training.sh --watch --envs 16 --iterations 5 --fit $FIT     # 짧게 확인: 창 + TRAINING FIT 창 · 창 클릭 후 P = 스크린샷
./run-training.sh --iterations 300 --fit $FIT                     # 본 학습 (창 없음, 512대, 약 30분)
./run-training.sh --iterations 300 --fit $FIT --uneven --snap 60  # + 낮은 턱 + 1분마다 화면 저장
tensorboard --logdir artifacts/training                           # 진행 그래프 (다른 터미널)
#   → artifacts/training/<시각>-rl_correct/ (policy.onnx · eval.json · base-fit.json · checkpoint · TensorBoard)
```

- **`--fit`은 꼭 준다.** 3.2의 sim은 지연 0 — 실기기 지연 0.1 s는 `--fit`이 학습 env 모터 모델(바퀴 τ·gain·지연 랜덤 범위)의
  **중심**으로 넣어야 들어간다 (폭은 유지). 궤적 그림이 비슷해 보여도 지연은 그림에 안 보인다.
- `--fit`의 sim은 학습과 **같은 물리 설정**(`robot_spec.PHYSICS_HZ`)으로 잰 raw여야 한다.
- 창 없이 돌면서 보고 싶으면 `--snap 60`: 60초마다 `<결과>/snaps/<시각>.png` + `<결과>/latest.png`(최신으로 덮어씀).
  `eog <결과>/latest.png`로 열어 두면 바뀔 때 다시 그린다. 렌더가 켜져 학습이 ~20% 느려진다 (32대 5.2→6.2 s/iter).
  P 키 스크린샷은 `--watch`(창)에서만.
- 바닥: env별 마찰 랜덤 + 파랑(미끄럼)/주황(끈적) 패치 4개 (`--no-friction`으로 끔) · `--uneven`이면 낮은 턱(1~3 cm)
  · 기울기: **기본 켜짐** — env마다 바닥판을 랜덤 방향·랜덤 각도 0~3°로 (오르막·내리막 = 바퀴 부하, 옆 기울기 = 흘러서 휨.
  실내 바닥 1~2°, 경사로 ~5°). 상한만 `--slope 5`로 바꾸고 `--slope 0`이면 평평. 패치·턱도 같은 판 위에 같이 기운다.
- 자이로 영점 오차: **기본 켜짐** — 에피소드마다 ±0.05 rad/s 고정값을 w_gyro에 더함 (정책이 엔코더와 비교해 영점을 무시하게).
  `--gyro-bias 0.1`로 폭 변경 · `0`이면 끔. 실기기 11-02의 −0.257은 고장 수준 — 학습으로 덮지 말고 OpenCR 재부팅(가만히)으로 고친다.
  slope·gyro_bias 값은 `base-fit.json` ranges에 남는다.
- `--latency ../ros2_ws/artifacts/real/before.csv`: RL 경로(PC 왕복) 지연까지 반영. rl-only Before 기록(경로 rl, enable:=false)에서
  관측 나이·corrector 처리 시간을 뽑는다. **PC·Pi 시계가 안 맞으면 거부** (3.0).
- 끝나면 sim Before/After를 자동으로 돈다(같은 env·seed, 정책 끔/켬) → `eval.json`:
  `improve_pct.rms_w`가 + 면 sim에서 회전 추종이 좋아짐 · `onnx_max_abs_diff` ≈ 0 이면 배포 파일 = 평가한 정책.
- `--iterations` 1회 = envs×64스텝 수집 + PPO 업데이트 (512대 ≈ 5.5 s). 자세히: [training/README.md](training/README.md).

### 3.5 실기기 A/B — 한 번 실행에 보정 끔/켬

```bash
cp artifacts/training/<시각>-rl_correct/policy.onnx artifacts/real/policy.onnx   # isaac/ 에서 — 본 학습(300회) 결과, eval.json 개선 + 확인 후
# 로봇: 베이스 · MANUAL (3.1과 같음) · 출발 위치·방향을 A와 B에서 같게
cd ../ros2_ws
./rl-only.sh label:=ab output:=artifacts/real/ab.csv          # 터미널 A — RL 노드 + 기록 (바이패스로 시작 = A)
# 터미널 B
./route.sh rl                                                  # 바퀴 명령이 PC를 거치게 (corrector 없으면 거부)
../isaac/run-calibration.sh sensor-sync-real --label A         # A: 같은 사각형, 보정 없음
./policy.sh ../isaac/artifacts/real/policy.onnx                # 정책 켜기 (멈춰 있을 때) → 터미널 A에 'ONNX loaded' 확인
../isaac/run-calibration.sh sensor-sync-real --label B         # B: 같은 사각형, 보정함
./route.sh direct
# 터미널 A: Ctrl+C
./report.py artifacts/real/ab.csv                              # → artifacts/real/report.png (A/B 구간별 지표)
```

- `rl-only.sh`는 **끝나지 않고 계속 돈다** — 아래 명령은 **다른 터미널(B)**에서. 한 터미널에 다 붙여 넣으면 첫 줄에서 멈춘 것처럼 보인다.
  터미널 하나로 하려면 백그라운드: `./rl-only.sh start label:=ab output:=…` → 주행 … → `./rl-only.sh stop`.
  ONNX를 바꿔 다시 켤 땐 `./rl-only.sh restart enable:=true output:=<새 이름>.csv` (인자 없이 restart면 지난 인자 그대로 — output 덮어씀).
  `status`(켜짐·인자·모델 로드) · `log`(로그 따라 보기).
- **`ONNX loaded`가 떠야** 정책이 들어간 것. `./policy.sh status`로 enable·경로 확인.
- 실수했던 것 (2026-11-02): ① 정책을 `ros2_ws/artifacts/real/`에 복사하고 `isaac/artifacts/real/`을 가리킴 → B도 보정 없이 달림(A/A).
  지금은 `policy.sh`가 없는 파일을 **거부**하고, `report.py`가 "켬인데 실제로는 바이패스" 구간을 경고한다.
  ② `--watch` 5회 짧은 시험 정책을 올림 → 거의 학습 안 됨(sim 개선 ±1%) → 실기기에서 오히려 나빠짐. 본 학습 결과를 쓴다.
- **A/B 전에 OpenCR를 가만히 둔 채 켠다** — 부팅 2초 동안 자이로 영점을 잡는다. 움직이면 정지 중에도 w_gyro ≠ 0
  (11-02 ab.csv: −0.257 rad/s) → 정책 입력이 오염돼 B가 엉뚱하게 돈다. report 표의 '자이로 정지 바이어스'가 노랗고 경고가 뜨면 재기록.
- 볼 것 (report 맨 위 표, B vs A가 초록이면 좋아짐): **궤적오차 · 끝점 오차 · 정상상태 RMS** B < A · B 보정량 ≠ 0 (정책이 일함).
  A와 B는 시작 자리·방향을 (0,0,0)·t=0으로 맞춰 겹쳐 그린다. '전체 RMS'는 계단 지연(≈0.1 s)이 대부분이라 참고만. 합격 기준은 task-spec.
- `policy.sh <onnx> | off | on | status`: 실행 중 정책 교체(바로 다시 로드). 없는 경로는 거부 · 바이패스는 `off`. 기록의 `rl_policy` 칸으로 report가 구간을 나눈다.
- 따로 기록해도 된다: `rl-only.sh label:=before output:=…/before.csv` → 주행 → Ctrl+C → `rl-only.sh enable:=true label:=after output:=…/after.csv`
  → 주행 → `report.py before.csv after.csv`. 한 번 실행(A/B)이 배터리·바닥이 같아 더 공정하다.
- 기록(입력 1개 = 1줄): 입력 · 보정 출력 · 정책 원출력 · 드라이버 명령 · odom · 엔코더 · 바퀴 속도/전류 · IMU · 모터 온도 · 배터리 · 상태 · 메시지 시각.
- 정책 입출력: ONNX `obs [1, 6]` → `actions [1, 2]` (물리 보정량). 정지 명령엔 보정 0.
- RL 경로에서 PC 응답이 0.3초 끊기면 로봇 정지 (cmd_manager 안전) — WiFi가 약하면 주행 중 멈출 수 있다.

## 4. sim 물리 — 고친 것 · 남은 것

1. **볼 캐스터 마찰** (2026-11-02 고침): calibration sim이 캐스터 마찰 0을 바닥과 `average`로 합쳐 0.4로 끌림 →
   바퀴가 명령의 87~95%만 냄(사각형 37 cm 어긋남). 학습 env처럼 `min` → gain ≈ 1.
2. **물리 480 Hz** (2026-11-02, `urdf/robot_spec.py` `PHYSICS_HZ`): 120 Hz는 제자리 회전 때 바퀴가 붙었다 미끄러져 떨림.
   정상 회전 구간 바퀴 속도 표준편차 120 Hz 0.62 · 240 Hz 0.15 · 360 Hz 0.11 · **480 Hz 0.027** · real 0.018 rad/s.
   solver 반복(16/4)·바퀴 마찰 0.6은 거의 효과 없음. calibration sim · 랜덤 월드 · 학습 env가 같은 값을 쓴다.
3. **damping/effort는 고정** (`calibration/parameter/ActuatorParam.py`, damping 50) — 바꿔도 sim 응답이 안 변해서 calibration 변수에서 뺐다.
4. **남은 차이**: 회전 때 몸체가 바퀴보다 덜 돎 — real 자이로/명령 0.950 vs 엔코더 0.995 (5%), sim 0.956 vs 1.017.
   학습 env의 바퀴 간격 오차 랜덤 범위(±8%)가 덮는다.
5. **데드밴드** — 측정 경로에 저속 구간이 없어 fit의 0은 "못 잼". 학습은 기본 범위(0~0.3) 사용.
6. 자세한 실험·판단: [calibration/README.md](calibration/README.md) · [calibration/EXPERIMENT.md](calibration/EXPERIMENT.md) · 개념: [../documents/isaac/concepts.md](../documents/isaac/concepts.md).

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
