# RL 보정 — 목적 · 설계 (v2, 2026-10-29 다시 잡음)

## 0. 목적 (documents·ros2_ws README "RL 보정 Before / After"에서)

> **같은 경로를 RL 끄고(Before) / 켜고(After) 달려서, 로봇이 명령대로 움직이는 정도가 좋아졌음을 수치로 보인다.**

- 정책은 Isaac Lab에서 학습 → ONNX → `rl_correction/corrector`가 cmd_manager의 **RL 경로**(`/cmd_vel_rl_in` → PC → `/cmd_vel_rl_out`)에서 보정.
- 비교는 `rl_eval/eval_recorder` CSV 두 개(before/after). RL은 실행 중 `./run-cli route rl|direct`로 켜고 끈다 (bringup 그대로).
- 범위: **이동 보정 1종(`rl_correction`)만.** `nav2_rl_controller`는 보류 (launch 미연결, 목적 문서에 없음 — 1종이 끝난 뒤 다시 판단).

## 1. v1이 목적을 못 이루는 이유 (리뷰 결과)

| v1 | 문제 |
|---|---|
| 시뮬 로봇 = `beebot.usda` (팔·그리퍼·수납함 달린 다른 로봇) | 질량·무게중심·관성이 우리 Waffle Pi(위판 제거·앞 라이다)와 다름 → 다른 로봇을 보정하는 법을 배움 |
| 관측 `v_odom·w_odom` = 시뮬 몸체 **진짜 속도** | 실제 `/odom`은 **바퀴 엔코더 × 기준 반지름**. 학습에서 넣은 오차(반지름 ±5%)는 실제 엔코더 odom에 **안 보임** → 실제에선 보정 신호가 없음 |
| 한계 0.22 / 2.84 (Burger) | 우리 0.26 m/s · 1.82 rad/s · 바퀴 8 rad/s(넘으면 두 바퀴 같은 비율 축소 — `opencr_driver.wheel_speeds`) |
| 30 Hz · 지연 1스텝(33 ms) | 실제 cmd_manager 20 Hz · RL이 PC에서 돌아 WiFi 왕복 50~200 ms |
| 명령 = 에피소드당 고정 (v0, w0) | 실제 오차가 가장 큰 건 **명령이 바뀌는 순간**(텔레오퍼 키·Nav2 MPPI 변화) |

## 2. 무엇을 보정하나 — 실제로 관측 가능한 것만

| 오차 원인 | 실제에서 보이나 | 보정 대상 |
|---|---|---|
| 모터 응답 지연 (명령 → 바퀴 속도) | ✅ 엔코더 바퀴 속도 (`S` 줄 l_vel·r_vel) | ✅ |
| 좌우 모터 비대칭 · 저속 데드밴드 | ✅ 엔코더 | ✅ |
| 바퀴 8 rad/s 포화 | ✅ (명령 대비 부족) | ✅ |
| 회전 미끄러짐 · 유효 바퀴 간격 오차 | ⚠ 엔코더로는 안 보임 → **OpenCR 내장 IMU 자이로**로 보임 | ✅ (자이로 추가 시) |
| 바퀴 반지름 오차(직진 거리) | ❌ 엔코더·자이로 모두 안 보임 (외부 기준 필요) | ✗ 범위 밖 |

→ **결정 필요 D1**: OpenCR IMU 자이로 z를 펌웨어 `S` 줄에 추가하고 `/imu`로 낼까? (권장 — 회전 보정이 Before/After에서 가장 눈에 띄는 개선)
  안 하면 관측은 엔코더만, 보정 대상은 모터 동특성(지연·비대칭·데드밴드·포화)으로 한정.

## 3. Task `rl_correct` v2

| 항목 | v2 |
|---|---|
| 로봇 | **우리 URDF** `ros2_ws/src/tb3_bringup/urdf/robot.urdf.xacro` → USD (`isaac/scripts/load_urdf.py`가 이미 변환함). 바퀴만 |
| 제어 주기 | **20 Hz** (cmd_manager와 같게). sim 120 Hz · decimation 6 |
| 지연 | 명령·관측 각각 0~4 스텝(0~200 ms) 에피소드마다 무작위 (WiFi 왕복) |
| 모터 모델 | 바퀴별 1차 지연(τ 0.05~0.25 s) · 좌우 이득 비대칭 ±10% · 데드밴드 0~0.3 rad/s · 바퀴 8 rad/s 포화(두 바퀴 같은 비율, 드라이버와 동일) |
| 기구 오차 | 유효 바퀴 간격 ±8% · 회전 미끄러짐(자이로가 있을 때만 의미) |
| 명령 | 에피소드(10 s) 안에서 **계단식으로 바뀜**: 텔레오퍼 패턴(w/x/a/d·사선 R=0.5 m, 설정 속도 0.1~0.26) + 부드러운 곡선(MPPI 흉내). 0.5~2 s마다 전환 |
| obs (N=6) | `[v_cmd, w_cmd, v_enc, w_enc, w_gyro, Δt_since_change]` — 엔코더는 **기준 반지름·간격으로 계산**(실제 드라이버와 같은 식), 자이로는 진짜 각속도 + 잡음. D1을 안 하면 N=5 (`w_gyro` 없음) |
| action (2) | `[Δv, Δw]` → 노드가 더한 뒤 clip 0.26 / 1.82 |
| reward | `-(|v_cmd - v_true| + 0.5·|w_cmd - w_true|) - 0.01·|Δ|² - 0.05·|Δ_t - Δ_(t-1)|` (진짜 속도로 채점 — 학습 중엔 정답을 앎. 마지막 항 = 출렁임 방지) |
| 종료 | 10 s |

- 한계값·주기·지연 범위는 **한 파일**(`isaac/tasks/common/robot_spec.py`)에 두고 학습 환경·문서가 같이 씀. ROS 노드 기본값도 같은 값(0.26/1.82)으로.

## 4. 평가 (Before / After)

- **같은 입력**: 손 텔레오퍼 대신 **스크립트 패턴**(`run-cli drive` 단계 조합 — 직진 2 s · 제자리 회전 · 사선 · 정지 반복)을 두 번 재생.
- 지표 (eval_recorder CSV에서): `RMS(v_cmd − v_enc)` · `RMS(w_cmd − w_gyro)`(D1) · 계단 명령 후 90% 도달 시간 · 보정량 평균.
- 합격 기준(초안): 회전 RMS 20% 이상 감소, 직진 RMS 악화 없음.

## 5. ONNX 규칙

- 입력 `obs` `[1, N]` float32 → 출력 `actions` `[1, 2]`. 정규화 포함·결정적 출력 (v1 export 방식 유지). opset 18.
- 파일명 `rl_correct_v2.onnx` → 로봇/PC `~/rl_models/`. `benchmark_onnx.py` 평균 50 ms 이하.
- v0 · v1 ONNX는 beebot·Burger 한계로 학습 → **쓰지 않음**.

## 6. 할 일 (순서)

1. D1 결정 → (예) 펌웨어 `S` 줄에 gyro_z 추가 · 드라이버 `/imu` · 업로드
2. `robot_spec.py` 공통값 · 우리 URDF USD 생성
3. `rl_correct` v2 환경 (모터 모델·지연·명령 패턴·obs/reward) + 짧은 스모크 학습
4. ROS `corrector` obs N=6(5) · 한계 0.26/1.82 · eval_recorder에 enc/gyro 열 추가 · 스크립트 패턴
5. 학습 → export v2 → Before/After 실측 → 결과를 issue-tracking에
