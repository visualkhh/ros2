# ml/vla — 영상 + 지시 + 로봇 상태 → 로봇의 모든 움직임

> 상태: **시범 데이터 생성기 완료** (`srf_recycling_ws_vla_demos`). 학습은 VLM(v1) 학습이 끝난 뒤.

## 목표에서 정한 것

목표: 실기 로봇이 지시대로 물체에 **다가가 집어서, 맞는 바구니 앞에서 다가가 놓는다**. CPU만 쓴다.

- 팔 작업 범위가 좁다 (로봇 앞 약 15~30 cm). Nav2 는 근처(오차 5~10 cm)까지만 → 마지막 접근은 **영상을 보며** 바퀴로.
  그래서 VLA 가 바퀴와 팔을 **같이** 낸다.
- **출력 = 로봇이 움직일 수 있는 모든 것** (`OpenCrDevice` 명령 전부 중 움직임·신호):

| 출력 | 실기 명령 | 값 |
|---|---|---|
| 바퀴 | `write(Twist)` | 선속도 v (m/s), 회전 ω (rad/s) — 차동 구동이라 좌·우 바퀴와 1:1 |
| 팔 | `write_arm` | j1, j2, j3, j4 (rad, 절대 목표) |
| 그리퍼 | `write_arm` 5번째 | 열림 (m, 0.019 열림 ~ -0.005 닫힘) |
| 부저 | `buzzer` | 0/1 (시작 알림, 사람이 1.5 m 안으로 들어올 때) |

  토크 on/off·모터 재부팅은 움직임이 아니라 정비·복구라 VLA 가 아니라 안전 계층이 맡는다.
- 4 Hz 로 앞으로 N 스텝 묶음(청크)을 낸다. 한 스텝 = 0.25 s 동안의 목표 (팔은 컨트롤러가 그 사이를 보간).

## 입력 (기준: 다음 동작에 정보가 되나 · 실기에서도 같은 값을 얻나)

| 입력 | 어디서 |
|---|---|
| 지시 문장 | `pick up the can`, `put the can in the can bin` (사람 말 → `ml/llm` intent → 문장) |
| RGB | D435 color (424×240) |
| 깊이 | D435 depth (16UC1 mm) — 대상 주변 조각 + 대상 3D 위치로 가공해 쓴다 |
| 차체 속도 | odom twist (v, ω) (좌·우 바퀴 속도와 같은 정보라 하나만) |
| 팔 관절 위치·속도·전류 | Dynamixel (전류 = 쥐었나·걸렸나) |
| 그리퍼 위치·전류 | 〃 |
| 엔드이펙터 자세 | 관절 FK (base_link 기준 x, y, z, 하향각, yaw) |
| 앞쪽 장애물 거리 | 라이다 앞 ±20° 최소값 |
| 대상 3D 위치 | VLM bbox → 깊이 → TF → base_link (m) |
| 바구니 위치 | map_marker → TF → base_link (m) |

넣지 않는 것: map/odom 자세(팔·접근은 로봇 기준 상대 위치만 필요), 카메라→base TF(고정), 모터 온도·에러(안전 감시).

## 구조 (계획)

```
RGB + 지시 ─▶ [ml/vlm 의 SmolVLM2 — 얼려 둠: 선별장 물체를 이미 앎] ─▶ 장면 이해 벡터 ─┐
깊이 조각 ─▶ 작은 CNN ────────────────────────────────────────────────────────────────┤
상태 (odom·관절·전류·그리퍼·EE·라이다) + 대상·바구니 3D ──────────────────────────────┼─▶ 행동 헤드 ─▶ N × 8
                                                                                      ┘
```
- 큰 VLM 은 얼리고 작은 헤드만 학습 → CPU 로 학습, 추론은 헤드 수 ms (VLM 은 재계획 때만).
- 모델 밖 어댑터: 속도·관절 한계·작업 영역 검사, 범위 밖은 보내지 않고 기록, collision_monitor 는 그대로, 사람이 가까우면 바퀴 무시.

## 시범 데이터: `srf_recycling_ws_vla_demos` (`simulator/tools/vla_demos.cpp`)

시뮬레이터 "조교"가 진실값으로 닫힌 루프 제어를 하며 4 Hz 마다 관측·행동을 기록한다 (헤드리스, 약 4 s/회).
- 명령은 실기와 같은 `OpenCrDevice`(SimulatorOpenCr)로, 영상은 `SimulatorRealSense` 로 → 엔진이 받는 값과 같은 형식.
- 팔 충돌(자기 몸·카메라·마운트·바닥), 쥐기·놓기·자유낙하·바구니 판정은 시뮬레이터 물리 그대로.
- 회차: **pick** (매트 근처 아무 데서 → 다가감 → 위 → 내려감 → 닫음 → 들어 올림 → 들고 다니는 자세)
  → 성공하면 **place** (해당 바구니 앞 — Nav 이동은 순간이동으로 대신 → 다가감 → 바구니 위 → 엶 → 물러남).
- 무작위: 장면·대상 종류·물품 수·누움/섬·방해물·사람·지게차·시작 위치/방향·팔 시작 자세, 대상 위치 추정 오차(편향 + 잡음).
- DART: `--dart` 비율의 회차는 스텝의 30% 에서 실행 행동에 잡음 (라벨은 조교의 깨끗한 행동) → 틀어지면 되돌아오는 법.

```bash
./build.sh simulator && source paths.sh
$SRF_RECYCLING_WS_BUILD_DIR/srf_recycling_ws_vla_demos --out artifacts/datasets/vla --episodes 1000 --seed 1 --dart 0.5
```

출력:
```
DIR/episodes.jsonl                         회차: kind, instruction, category, variant, lying, success, reason, dart
DIR/episodes/000001_pick/rgb/000.jpg ...   관측 영상
                        depth/000.png      16비트 mm
                        steps.jsonl        스텝마다 state(입력) · action(정답) · truth(채점용) · phase
```
`steps.jsonl` 한 줄: `state` = odom·wheels·arm(position/velocity/current)·ee·scan_front_min·target_xyz(추정)·bin_xyz,
`action` = v·w·joints[4]·gripper·buzzer, `truth` = target_xyz(진실)·holding·blocked(충돌)·person_min·robot 자세.

현재 확인: 깨끗한 회차 16/16, 잡음 회차 24/24 성공 (pick·place 모두, 낙하 없음). 약 6 MB/회.

## 다음

1. VLM v1 이 끝나면 얼려서 특징 캐시 → 행동 헤드 학습 (CPU), 보고서는 ml/vlm 과 같은 metrics.csv/report.html.
2. 시뮬에서 정책을 닫힌 루프로 돌려 채점 (집기 성공률·놓기 성공률·충돌·낙하) — 조교 대신 정책을 넣는다.
3. 엔진 `vla_node` 를 이 입출력(8개 출력, 위 입력)으로 바꾸고 ONNX 로 내보낸다.
4. 실물 시연 소량(텔레옵)으로 미세조정·평가.
