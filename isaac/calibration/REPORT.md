# 휠 드리프트 보정 — 현재 상태 & 계획

작성 2026-11-01 · 대상 `training/rl_correction_wheel_drift` (track) · 환경은 **floor 하나**

---

## 0. 한 줄 요약

real 로봇이 명령(`v`, `w`)대로 움직이도록 **RL 보정 정책**을 학습시키기 전에,
**시뮬레이터가 실제 로봇의 버릇(오차)을 똑같이 내도록 맞추는 작업**을 정리한 문서다.

- 지금까지: 측정 → fit → 리포트 → sim 적용까지 **파이프라인 완성**, 더미로 검증
- 측정 프로파일에 **저속(데드밴드) · 단륜(좌/우) · 고부하(포화)** 진단 구간 추가
- 아직: **진짜 real 로봇 데이터 없음** (ROS2 PC에서 캡처 필요)

---

## 1. 왜 하나 (쉬운 설명)

- 로봇은 명령을 100% 그대로 못 따른다. 좌우 모터가 조금씩 다르고, 반응이 늦고, 작은 명령은 무시한다.
- 그래서 직진이 한쪽으로 휘고, 회전이 덜/많이 돈다 → **휠 드리프트**.
- RL은 **시뮬레이터(연습장)** 안에서 "이럴 땐 이렇게 보정해라"를 배운다.
- 연습장이 실전과 다르면 배운 게 실전에서 안 통한다.
  → **그래서 연습장(sim)을 실전(real)에 맞춘다.**

**방향: real → sim.**
real 로봇을 sim에 맞추는 게 아니다(그건 로봇을 개조하는 일).
단, real 쪽에서 바꿀 수 있는 건 "명령 보정(컨트롤러)"뿐이고, 그건 별개의 일이다.

---

## 2. 가장 중요한 개념

### (1) 명령 vs 관측
| 용어 | 뜻 |
|---|---|
| `v_cmd`, `w_cmd` (`lin_raw`, `ang_raw`) | 우리가 시키는 속도 |
| `v_enc`, `w_enc` | 엔코더(바퀴 회전)로 계산한 속도 |
| `w_gyro` | 자이로(IMU)로 측정한 진짜 몸체 회전 |

### (2) fit = 로봇의 "버릇"을 숫자로 적은 것
`fit.json`에는 이런 값이 들어간다.

| 키 | 뜻 | 예 |
|---|---|---|
| `wheel_gain_l` / `wheel_gain_r` | 왼/오른쪽 모터가 명령을 따라가는 비율 | 0.99 = 99%만 나옴 |
| `wheel_tau_s` | 반응이 따라잡는 데 걸리는 시간(1차 지연) | 0.045 s |
| `wheel_delay_steps` | 명령이 도착하기까지 걸리는 시간 | 2 step = 0.1 s |
| `wheel_deadband` | 이보다 작은 명령은 바퀴가 안 도는 구간 | 0.2 rad/s |
| `wheel_deadband_l/r` | 좌/우 각각의 데드밴드 | 0.225 / 0.175 |
| `wheel_max_obs` | 관측된 최대 바퀴 속도 | 8.08 rad/s (포화) |
| `saturated_samples` | 8 rad/s 포화가 걸린 샘플 수 | 1 |
| `power` | 고부하 구간의 gain (전력/전압 강하 진단) | `straight_l/r`, `turn_l/r` |

### (3) sim을 real에 맞추는 두 방법 — 결과는 같다
fit 값은 "로봇 파라미터의 절대값"이다. sim을 real에 맞추는 방법은 두 가지가 있고, **수학적으로 같다**.

- 방법 A (대체): `sim_new = real` ← 우리 코드
- 방법 B (delta 더하기): `sim_new = sim_current + (real − sim_current) = real`

예: sim gain 0.99, real gain 0.95 → `0.99 + (0.95 − 0.99) = 0.95` ✅

**틀리는 경우는 이 두 가지뿐이다.**
1. delta를 sim 값으로 그대로 넣기: `sim gain = −0.04` ❌ (delta는 "차이"지 값이 아님)
2. 이미 real로 맞춘 sim에 delta를 또 더하기: `0.95 + (−0.04) = 0.91` ❌ (이중 보정)

**주의**
- delta가 통하는 건 더하기가 되는 파라미터(`gain`, `tau`, `delay`)뿐.
- `deadband`, `포화(8 rad/s)`, `마찰`은 명령 크기에 따라 달라져서 "5%" 같은 단일 오차율로 표현이 안 된다. → 구간별 곡선을 맞춘다.
- "real 5%, sim 1%"는 **같은 조건(같은 속도·바퀴·측정)** 에서 나온 값이어야 뺄 수 있다.
- **명령 보정(피드포워드)은 부호가 반대**: real gain 0.95면 명령을 `1/0.95 ≈ +5.26%`로 키운다. (`+4%` 아님)

---

## 3. 그래프 숫자 읽는 법

| 지표 | 뜻 |
|---|---|
| `bias` | 한쪽으로 치우친 오차 (평균) |
| `rms` | 전체 흔들림 크기 (작을수록 좋음) |
| `p95` | 나쁜 순간 5%의 크기 |
| `t10` | 명령이 바뀐 뒤 10% 도달까지 걸린 시간 |
| `궤적오차` | 이상 궤적(명령 100%)과 실제 궤적의 평균 거리 [m] |

그래프 선 의미:

| 선 | 뜻 |
|---|---|
| 초록 점선 | 명령을 100% 따랐을 때의 **이상 궤적** |
| 파랑/주황 실선 | 실제로 간 **궤적(실측)** |
| 빨강 점선 | `fit` 파라미터로 **실제를 재현한 모델** (실측에 붙는 게 정답) |
| 마젠타 띠 | 그 `fit` gain을 **명령에 역보상**했을 때의 예상 궤적 (이상 쪽으로 이동) |

> 빨강은 "실제를 잘 흉내 내는가"(모델 검증), 마젠타는 "보정하면 좋아지는가"(보정 효과)다.
> 둘을 헷갈리면 안 된다.

---

## 4. 측정 프로파일 (excitation) — 무엇을 재나

현재 기본 프로파일은 **사각형 → 180° 턴 → 같은 사각형 역주행**만이다 (약 54 s).
진단 구간(저속·단륜·고부하)은 `DefaultCalibExcitation._diagnostics`에 남아 있지만
기본으로는 안 붙는다 — 필요하면 서브클래스의 `_extra`에서 붙인다. 아래 표의 *진단* 행이 그것이다.

| 구간 | 명령 | 재는 것 |
|---|---|---|
| 정지 | `v=w=0` | 영점/노이즈 |
| 사각형 전진 (4속도) | 양쪽 같은 방향, 0.15/0.10/0.20/0.12 m/s | gain L/R, tau, delay, 직진 추종 |
| 90°/180° 회전 | 양쪽 반대 방향 | 차동 회전 gain, 회전 추종 |
| 역주행 | 양쪽 같은 방향(후진) | 전/후진 비대칭 |
| *진단* 저속 계단 (0.10~0.40 rad/s) | 양쪽 | **deadband** (안 도는 최소 명령) |
| *진단* 단륜 좌/우 (3 rad/s) | 한쪽만 | **좌/우 모터를 따로** 분리 |
| *진단* 고부하 직진 (`v=0.26`) | 양쪽 최대 | 고속/전류에서 gain 변화 |
| *진단* 고부하 회전 (`v=0.26`, `w=1.82`) | 양쪽, 바퀴 8 rad/s 초과 | **포화(비례 축소)** 거동 |

핵심:
- **양쪽 동시는 이미 사각형에서 잰다** (직진·후진·회전).
- **단륜은 진단용**이다. 실제 주행/학습은 항상 양쪽이 함께 움직이므로, 단륜은 모터를 분리해 볼 때만 쓴다.
  바닥에서 단륜을 돌리면 로봇이 제자리 회전하고 반대 바퀴가 끌리므로, 값은 "모터 단품"으로만 해석한다.
- **전력/배터리**: 고부하 구간에서 gain이 떨어지는지 본다. 한 번의 주행 안에서보다,
  배터리 잔량을 바꿔 **여러 번 캡처**해서 `speed_fit`/`power`의 gain 변화를 비교하는 게 정확하다.
- 진단 구간이 없으면 **deadband는 측정되지 않는다** (사각형 바퀴 명령이 3 rad/s 이상).
  이때 fit.json의 `wheel_deadband: 0`은 "0"이 아니라 "못 잼"이다.
- 진단 구간을 붙이면 전체 ≈ **75 s** (54 s + 21 s). 고부하 회전·단륜 때문에 **주변 공간을 비워둘 것**.

---

## 5. 지금 파이프라인 (무엇이 어디에 쓰이나)

```
[real 캡처]  DefaultSensorReal  →  raw.csv (20열)
[fit]        report.fit_wheel   →  fit.json (gain L/R, delay, tau, deadband, power, sat)
[리포트]      report.write_report_png → report.png
[비교]        report.compare    →  compare_<tag>.png
[sim 적용]    SimParam.load(condition) → WheelLag (gain/지연/tau/deadband)
[학습]        rl_correct_env_cfg.py (gain/tau/deadband/delay randomization + friction)
[배포]        rl_correction/corrector (ONNX)
[평가]        eval_recorder + ros2_ws/report.py (Before/After)
```

최근에 추가/수정한 것:

- **좌/우 gain 따로 fit** (`wheel_gain_l`, `wheel_gain_r`) → `WheelLag`가 좌우 따로 적용
- **fit이 모든 구간 사용**: 전진/후진/회전/단륜/저속/고부하 (전에는 전진 구간만)
- **바퀴별 target으로 gain 계산**: `cmd_l = (v − w·sep/2)/r`, `cmd_r = (v + w·sep/2)/r`
- **8 rad/s 비례 축소**를 fit·리포트 모델에도 적용 (전에는 포화 미반영)
- **deadband fit**: 저속 계단에서 안 도는 최대 명령과 도는 최소 명령의 중간값
- **포화/전력 진단**: `wheel_max_obs`, `saturated_samples`, `power`
- **각속도 모델**: 바퀴 모델 → 차동기구학 `w=(wr−wl)·r/sep` (`fit_ang`)
- **바퀴 L/R 패널 분리**
- **궤적 4종**: 이상(초록) / 실측(파랑) / fit model(빨강) / 보정 후(마젠타)
- `DummySensor`: `gain_l/gain_r`, `slip`, `deadband`, `noise` 옵션 (검증용, 포화도 재현)

---

## 6. 더미 검증 결과 (real 아님!)

`DummySensor`로 파이프라인만 검증한 값이다. 실제 로봇 측정값이 아니다.
`dummy_real`에는 **gain 비대칭 0.99/1.01 + deadband 0.2 rad/s**를 넣었다.

| | gain L/R | delay | tau | deadband | 포화 샘플 | 궤적오차(실측) | 보정 후 |
|---|---|---|---|---|---|---|---|
| `dummy_sim` | 1.000/1.000 | 2 step | 0.0455 s | 0.00 | 1 | 0.016 m | 0.019 m |
| `dummy_real` | 0.990/1.010 | 2 step | 0.0455 s | 0.20 | 1 | 0.080 m | 0.021 m |

- `dummy_sim`은 gain이 1.0이라 보정할 게 없고, 남은 지연/tau만 반영됨
  (그래서 보정 후 수치가 실측보다 조금 커 보이는 게 정상)
- `dummy_real`은 gain 비대칭 + deadband를 넣었고, 보정 후 0.080 → 0.021 m로 줄어드는 것을 확인
- `power` 진단: straight L/R = 0.99/1.01, turn R = 1.01 (포화 구간 gain)
- 산출물: `artifacts/calibration/dummy_sim/`, `dummy_real/`, `compare_dummy.png`

---

## 7. 결정 사항 (2026-11-01)

| # | 결정 | 결과 |
|---|---|---|
| 1 | 환경은 **floor 하나만** | 조건별 다중 캘리브 없음. 반복 측정 분산으로만 랜덤화 폭 결정 |
| 2 | 지연 범위는 **문서 기준** | `rl_correct_env_cfg.py`의 `cmd_delay_steps`, `obs_delay_steps`를 `(0, 2)` → **`(0, 4)` (0~200 ms)** 로 수정 완료 |
| 5 | `ang_weight=0.5`와 합격 기준은 **정합** | 보상/평가 가중치 변경 없음 |

---

## 8. 실행 순서

### Phase 0 — real 캡처 (ROS2 PC)
```bash
cd ros2/isaac
source /opt/ros/lyrical/setup.bash
export ROS_DOMAIN_ID=25
source ../tool/peers.sh

# 로봇 안 움직이고 상태만 확인
.venv/bin/python -m calibration.sensor.DefaultSensorReal --check-only

# 실제 캡처 (MANUAL 상태 + Pi base/cmd_manager 필요, ~75 s, 주변 공간 확보)
.venv/bin/python -m calibration.sensor.DefaultSensorReal --label real_floor
```
- 결과: `artifacts/calibration/<시각>-02-real/raw.csv` 또는 지정 경로
- **2~3회 반복**해서 값의 흩어짐(분산)도 기록
- 배터리 잔량을 바꿔 한 번 더 찍으면 **전력/전압 강하** 영향을 볼 수 있다

### Phase 1 — fit / 리포트
캡처할 때 `fit.json` + `report.png`가 자동 생성된다.
CSV만 다시 그릴 때:
```bash
.venv/bin/python -m calibration.report.report <raw.csv>
```
fit 값 확인 포인트:
- `gain L/R`이 1에서 얼마나 벗어나는지
- `delay`, `tau`가 sim 기본값(0.05 s, 2 step)과 얼마나 다른지
- `wheel_deadband`가 0보다 큰지 (저속에서 안 도는지)
- `saturated_samples`, `wheel_max_obs`로 포화 거동 확인
- `power.straight_*`, `power.turn_*`로 고부하 gain 확인
- `bias_w`, `rms_w`, `t10_w` (회전 추종)

### Phase 2 — sim에 real fit 적용
```bash
# sim(raw)과 real의 fit.json 차이를 sim 입력 보정으로
./run-calibration.sh sensor-sync-sim --viz none --fit <real>/fit.json,<sim>/fit.json
```
- sim은 `--fit`으로 준 값만 쓴다 (숨은 `sim_calib_floor.json`은 안 읽는다). 넣은 값은 `base-fit.json`.
- `<sim>/fit.json`은 반드시 `--fit` 없이(raw) 잰 것.

### Phase 3 — sim 재검증
```bash
./run-calibration.sh sensor-sync-sim --viz none
```
- 새로 나온 sim CSV를 다시 fit
- **`fit_sim ≈ fit_real`** 이면 성공 (sim이 real을 닮음)
- 다르면 그 차이는 모델에 없는 성분(friction/contact) → 추가 fit 필요

### Phase 4 — 학습 / export
```bash
./run-training.sh --iterations 300 --fit artifacts/calibration/<real>/fit.json
# → artifacts/training/<시각>-rl_correct/policy.onnx (학습 끝에 자동 export)
```
- 모니터: `Metrics/track_err`, 보정량 크기, 출렁임(jerk)

### Phase 5 — real Before/After
```bash
# scripted drive 패턴을 before/after 2회 재생
# enable=false → before.csv, enable=true → after.csv
ros2_ws/report.py before.csv after.csv   # → report.png + 지표 표
```
합격 기준 (task-spec §4):
- `RMS(w_cmd − w_gyro)` **20% 이상 감소**
- `RMS(v_cmd − v_enc)` **악화 없음**
- 보정량 평균이 과도하지 않을 것

---

## 9. 주의 / 한계

1. **real 데이터가 아직 없다.** 지금 수치는 전부 `DummySensor` 검증용이다.
2. **translation slip은 관측 불가.** 정책 입력은 엔코더(`v_enc`)와 자이로(`w_gyro`)뿐이라,
   바퀴가 헛도는 직진 미끄러짐은 보정 범위 밖이다(문서와 일치). 회전 slip만 `w_gyro`로 보인다.
3. **fit은 wheel 파라미터**(gain L/R, tau, delay, deadband)를 잡는다. 마찰/contact은 별도 노브다.
4. **단일 floor라도** 반복 측정 분산이 있으므로, 학습 랜덤화 폭은 0으로 두지 말고 분산 기반으로 잡는다.
5. **delta는 더하기 파라미터만.** deadband/포화/마찰은 단일 %로 환산하지 말고 구간별로 맞춘다.
6. `run-calibration.sh` / `run-training.sh`는 `isaac/` 루트에 있다 (`python -m calibration|training` 래퍼).
7. 진단 구간 때문에 궤적 그래프의 x/y 범위가 넓어져 메인 사각형이 작게 보인다.
   회전/포화 거동은 v/w/바퀴 패널에서 보는 게 낫다.

---

## 10. 용어 사전

| 용어 | 쉬운 뜻 |
|---|---|
| sim-to-real / real-to-sim | sim을 real에 맞추는 것(우리 방향) |
| fit | 측정값에서 로봇 버릇(파라미터)을 뽑는 것 |
| delta | `real − sim` 차이. 더하기 파라미터에만 쓸 수 있음 |
| gain | 명령을 얼마나 충실히 따르는가 (1.0 = 100%) |
| tau (τ) | 반응이 늦게 따라오는 정도 |
| delay | 명령이 도착하기까지 걸리는 시간 |
| deadband | 아주 작은 명령을 무시하는 구간 |
| saturation | 최대 속도(8 rad/s)에 걸려 더 못 가는 것 |
| 단륜 (single-wheel) | 한쪽 바퀴만 돌리는 진단 명령 |
| power (고부하) | 양쪽을 최대로 돌려 전류/전압 강하를 보는 구간 |
| randomization | 학습할 때 파라미터를 일부러 조금씩 흔드는 것 (일반화) |
| Before/After | RL 끄고/켜고 같은 주행을 두 번 해서 비교 |

---

## 11. 관련 파일

| 역할 | 파일 |
|---|---|
| 측정 오케스트레이터 | `calibration/Calibrate.py` |
| 측정 프로파일 | `calibration/excitation/DefaultCalibExcitation.py` (`_diagnostics`) |
| fit / 비교 / 리포트 | `calibration/report/report.py` |
| 센서 (sim/real/더미) | `calibration/sensor/DefaultSensorSim.py`, `DefaultSensorReal.py`, `DummySensor.py` |
| sim 파라미터 로드 | `calibration/parameter/SimParam.py`, `WheelLag.py`, `WheelParam.py` |
| 학습 환경 설정 | `training/rl_correction_wheel_drift/task/rl_correct_env_cfg.py` |
| 학습 환경 로직 | `training/rl_correction_wheel_drift/task/rl_correct_env.py` |
| 배포 노드 | `ros2_ws/src/rl_correction/rl_correction/corrector_node.py` |
| Before/After 평가 | `ros2_ws/src/rl_eval/` |
| 목적/설계 원본 | `documents/isaac/task-spec.md` |
| 값이 들어가는 위치 | `calibration/PRE_RL.md` |
