# 지도 만들기 → 지도로 주행 → 특정 위치로 보내기

SLAM·Nav는 로봇(Pi)에서 돌고, 조작은 **토픽이 보이는 곳 어디서나** `./run-cli`로 합니다 (보통 화면이 있는 PC에서 RViz와 같이).
명령 전체는 [README.md](README.md), 노드 구성은 [../ros2_ws/ARCH.md](../ros2_ws/ARCH.md).

- 켜기/끄기·지도 저장은 로봇의 프로세스·파일이라, 로봇이 아닌 곳에서는 tool이 SSH로 로봇에 넘깁니다 (`[pak.local] …`). 지도는 항상 로봇 `~/ros2/ros2_ws/map/`.
- 텔레오퍼·목표·초기 위치·모드는 토픽이라 어디서나 바로.
- 아래 명령은 PC든 Pi든 `tools/` 폴더에서 그대로 실행.

## 두 가지 방법: 명령 한 줄 · 메뉴

같은 기능을 두 방식으로 씁니다. 아래 각 단계에 둘 다 적었습니다.

- **명령 한 줄**: `./run-cli slam start`처럼 바로 실행 (스크립트·복붙에 편함).
- **메뉴**: `./run-cli` (인자 없이) → 화면에서 `↑↓`로 고르고 `Enter`, 또는 번호 키. `Esc` 뒤로 · `m` 메인 · `q` 종료.
  위쪽 상태바(베이스·SLAM·Nav ●/○ · 조작·지도·위치 · E-STOP·충돌감시·연결 · odom·스캔 · 주행 RL/DWB · 모터·전원)가 2초마다 갱신돼서 보면서 하기 좋습니다.

```text
./run-cli                       메인 메뉴
├─ c) 제어 상태 바꾸기 (지금 …)   MANUAL · NAV_PLAN · SLAM_AUTO_MAPPING (RViz 2D Goal Pose·goal은 자동으로 NAV_PLAN)
├─ 1) 상태                      1 상태 출력 · 2 모터 재부팅 · 3 제어 상태 · 4 경로(직결/RL) · 5 비상정지
├─ 2) 베이스                    1 시작/정지 · 2 재시작 · l 로그
├─ 3) SLAM 맵핑                 1 시작/정지 · 2 재시작 · 3 지도 저장 · 4 live 지도 보기 · 5 지도 보며 수동 이동 · v RViz · t 텔레오퍼 · a 자동 맵핑 · l 로그
├─ 4) Nav (지도 주행)           1 시작(지도 선택)/정지 · 2 재시작 · 3 지도에서 목표·초기위치·취소 (한 화면) · c 충돌 감시 설정 · v RViz · l 로그
├─ 5) 수동 주행 / 텔레오퍼      1 텔레오퍼 · 2~5 전진·후진·좌·우 한 번 · 6~8 속도·시간 설정
├─ 6) 지도 파일                 s 지금 지도 저장 · v 지금 /map 보기 · g 목표·초기위치·취소 · 번호 = 저장 지도 보기
├─ 7) SLAM 중 Nav               1 시작/정지 · 2 재시작 · 4 목표 보내기 · 5 목표 취소 · v RViz · l 로그
├─ 8) 노드 목록
├─ t) 토픽 · bag                 s 선택 녹화 · d 진단 녹화 · u 부하 기록 · p 재생 · 숫자/Enter 토픽 선택
├─ 9) 전부 정지·재시작           = stop / restart-all
└─ 0) 비프음                    1 삑 · 2 높이·길이 지정 · 3 타이핑삑 (= beep · say)
```

`1 시작/정지`는 같은 자리에 지금 상태에 맞는 쪽만 보입니다 (꺼져 있으면 시작, 켜져 있으면 정지).
`v RViz로 보기`는 CLI를 켠 컴퓨터에 RViz 창을 따로 띄웁니다 (화면 없는 SSH 터미널이면 안내만, 이미 떠 있으면 그대로).

## 0. 로봇 켜고 bringup (매번 먼저)

로봇 전원을 켜면 Pi는 부팅만 하고 **ROS는 아무것도 안 떠 있습니다** (자동 시작 없음). 바퀴·odom·라이다를 내는
**베이스(bringup = `tb-only`)** 를 먼저 띄워야 SLAM·Nav·텔레오퍼가 동작합니다.

```bash
# ① 로봇 전원 ON (OpenCR 스위치 + 배터리) → Pi 부팅 1~2분 → 접속되는지
ping -c2 pak.local                       # 응답 없으면 아직 부팅 중 · WiFi 확인

# ② 코드 최신화 (PC에서 고친 게 있으면)
ssh pak 'cd ~/ros2 && git pull'          # ros2_ws를 바꿨으면: ~/ros2/setup/pi/build.sh

# ③ 베이스 bringup — 둘 중 하나
./run-cli base start                     # PC에서 (tool이 SSH로 Pi에 띄움)
ssh -t pak 'cd ~/ros2/tools && ./run-cli base start'   # 또는 Pi에 들어가서 직접

# ④ 확인
./run-cli status
#   ● 베이스 · 로봇 ● ok · 제어 MANUAL · 운행 IDLE · 경로 직결 · 비상정지 off · odom 20Hz · scan 10Hz 이면 OK
```

메뉴: `./run-cli` → `2) 베이스` → `1) 시작` → 위쪽 상태바가 `● 베이스` · `로봇 ● ok`가 되면 OK.

베이스가 띄우는 것 (`ros2 launch tb3_bringup tb-only.launch.py`): state_manager · cmd_manager · opencr_driver ·
robot_state_publisher · coin_d4_driver(라이다) · scan_filter · scan_resample — [ARCH.md](../ros2_ws/ARCH.md).
한 번 띄우면 끌 때까지(또는 Pi 재부팅) 계속 돕니다. 이미 떠 있으면 ③은 "이미 실행 중"으로 끝납니다.

| 증상 | 확인 |
|---|---|
| `로봇 ? 대기 중` · `odom ✗` | 베이스가 안 떴거나 죽음 → `./run-cli base log` · OpenCR 전원·USB (`/dev/opencr`) |
| `로봇 ✗ no S line` | OpenCR 펌웨어 응답 없음 → OpenCR 전원 스위치 · [OpenCR 문서](../documents/opencr/opencr-setup-ubuntu26.04.md) |
| `scan ✗` | 라이다 USB (`/dev/tb3_lidar`) · `./run-cli base log`에 `coin_d4_driver` |
| 상태바가 다 `✗` (ping은 됨) | WiFi DDS 탐색 → `run-cli`로 실행했는지 (피어 설정 자동) · [README 문제 확인](README.md#문제-확인) |
| `[pak.local] … SSH 실패` | `ssh pak@pak.local`이 비번 없이 되는지 |

## 1. 지도 만들기 (SLAM)

```bash
# ① SLAM 시작 (로봇에서 돔)
./run-cli slam start     # 상태바: ● SLAM 맵핑 · 운행 SLAM (지도 (SLAM 작성 중)) · map …m

# ② 터미널 1 — 지도 커지는 거 보기
./run-rviz slam

# ③ 터미널 2 — 천천히 돌아다니기
./run-cli teleop        # w 전진 · x 후진 · a/d 제자리 회전 · q/e 앞 사선 · z/c 뒤 사선 (설정 속도 유지, 반경 0.5 m 원호 · 0.20 m/s 넘으면 약 0.21에서 모양 유지) · s 정지 · -/= 속도 · b 삑 · Esc 종료
```

메뉴: `./run-cli` → `3) SLAM 맵핑` → `1) 시작` → `5) 지도 보면서 수동 이동` (WASD/방향키 · `s` 정지 · live 지도). RViz로 보려면 `v`.
텔레오퍼는 제어 상태가 MANUAL이 아니면 바꿀지 먼저 묻습니다.

요령:

- **천천히.** 0.10 m/s, 회전은 `a`/`d` 짧게 → `s`. 빨리 돌면 스캔 정합이 깨져 지도가 겹치거나 휩니다.
- 벽을 따라 한 바퀴 → **출발점으로 돌아오기**(루프가 닫히며 지도가 바로잡힘) → 방 가운데.
- 라이다가 1층(바닥에서 ~3 cm)이라 책상 상판 같은 높은 것은 다리만 보입니다.
- CLI 상단 `map 10.4x9.6m` · 상태 화면의 `탐색 N%`로 진행 확인.

### 1-1. 자동 맵핑 (스스로 탐색)

로봇이 지도의 **모르는 쪽 경계(frontier)** 로 Nav2 목표를 스스로 계속 보내며 지도를 넓힙니다.
탐색은 **explore_lite**(m-explore-ros2, BSD), 우리 제어 상태와의 연결은 `explore_bridge` — 둘 다 "SLAM 중 Nav"(`explore-nav.launch.py`)에 포함.

```bash
./run-cli automap start       # SLAM·SLAM 중 Nav가 꺼져 있으면 켜고 → 제어 상태 SLAM_AUTO_MAPPING
./run-rviz slam               # 보기 · 상태바 "탐색" 줄에 진행 (탐색 중 · 완료) · RViz에 /explore/frontiers 추가하면 후보 경계가 보임
./run-cli automap stop        # 도중에 멈추기 → MANUAL
./run-cli map save office     # 다 찾으면 스스로 MANUAL로 — 그다음 저장
```

메뉴: `./run-cli` → `3) SLAM 맵핑` → `a) 자동 맵핑 시작` (확인창) → `v) RViz로 보기` → 끝나면 `3) 지도 저장`.

- 시작하면 단계가 처리 중 창에 나옵니다: SLAM 켜기 → SLAM 중 Nav 켜기 → Nav2 활성화 대기 → **제자리 한 바퀴**(약 13초) → 탐색.
  한 바퀴는 멈춘 채로는 SLAM 지도에 스캔이 1장뿐이라 로봇 바로 옆만 빈 칸이 되고, explore_lite가 갈 곳을 다 포기해 바로 끝나기 때문.
- 주변을 비우세요. 라이다 뒤쪽 135°가 몸통에 가려 안 보이므로 **후진은 위험** — Nav2가 주로 앞으로 갑니다.
- 갈 수 없는 frontier(30초 동안 진척 없음)는 건너뜁니다. 시작 15초 안에 "완료"가 나오면(빈 지도) 무시하고 다시 탐색. 덜 찍힌 구석은 `t) 수동으로 이동`으로 마저.
- RViz에서 Nav2 Goal을 찍거나 텔레오퍼로 MANUAL로 바꾸면 자동 맵핑은 그 자리에서 멈춥니다.

## 2. 지도 저장

**SLAM을 끄기 전에** 저장 (끄면 만든 지도가 사라짐).

```bash
./run-cli map save office    # 이름 생략 = 날짜_시각
./run-cli map list           # 확인
./run-cli slam stop          # 저장 후 SLAM 끄기
```

- 저장 위치 (항상 로봇): `~/ros2/ros2_ws/map/office/map.yaml` + `map.pgm`
- PC에서 저장 지도를 열면 로봇의 `ros2_ws/map/<이름>`에서 두 파일을 `~/.cache/beebot/maps/<이름>`으로 내려받아 표시한다. **원본은 로봇의 `ros2_ws/map`**이고 `.cache`는 PC 화면용 임시 사본이다 (로봇에서 실행하면 캐시 없이 바로 읽는다).
- 메뉴: `./run-cli` → `3) SLAM 맵핑` → `3) 지도 저장` → 이름 입력 (빈칸 Enter = 날짜_시각) → 끝나면 같은 메뉴 `1) 정지`.
  목록·내용은 `6) 지도 파일` (`s`로 저장도 됨).
- `/map 수신 없음`으로 거부되면 SLAM이 지도를 안 내는 중 → `./run-cli slam log`, 라이다 `scan` 확인.

## 3. 저장한 지도로 주행 (Nav)

```bash
# ① Nav 시작 (SLAM·SLAM 중 Nav가 켜져 있으면 자동 정지 후 시작 — SLAM 지도는 저장 안 했으면 사라지니 먼저 저장)
./run-cli nav start office   # 이름 생략 = 가장 최근 지도 · 상태바: 운행 LOCALIZATION (지도 office)
./run-cli nav start office --model <policy.onnx>   # RL 정책까지 같이 로드 (PC 경로는 로봇 복사)

# ② 주행 화면
./run-rviz nav
```

메뉴: `./run-cli` → `4) Nav (지도 주행)` → `1) 시작 (지도 선택)` → 목록에서 지도 고르기 (최신 먼저) → `3) 지도에서 목표·초기위치·취소` (한 화면) → `i` 초기 위치 → 지도에서 `Enter`로 목표 → `g` 주행.

실행 중 지도를 바꾸려면:

```text
4) Nav (지도 주행) → 4) 지도 바꾸기 (다른 지도 선택 → Nav 재시작)
또는
6) 지도/위치 → n) Nav 지도 바꾸기 (다른 지도 선택 → Nav 재시작)
또는
./run-cli nav switch office
```

Nav를 멈추고 새 지도로 다시 시작하므로 **초기 위치(3)를 다시 넣어야 한다**.

지도 화면 키:

```text
Enter         웨이포인트 추가 (십자선)        g  주행 (1개=NavigateToPose, 여러 개=한 번에)
Backspace     마지막 웨이포인트 취소          d  전체 지우기
i             십자선을 초기 위치로            x  주행 취소 + 예상경로/경로·웨이포인트 지움
v / V         예상 경로 계산 / 지우기         p  Nav2 /plan·/local_plan 겹쳐 보기
c             costmap 겹침 (OFF → GLOBAL → LOCAL)
s             웨이포인트 안전 스냅 (벽/costmap 팽창 피해 가까운 빈 칸) — 기본 켜짐
m / o         map 원점 M / odom 원점 O 표시   [ ] { }  방향 15° / 1°
+ - f r       확대·축소 / 전체 보기 / 로봇 따라가기
```

SLAM 중 live 지도:

```text
3) SLAM 맵핑 → 4) 지도 보기          /map이 0.5초마다 갱신 — 지도가 자라는 모습을 본다 (이동 없음)
3) SLAM 맵핑 → 5) 지도 보면서 수동 이동  W/↑ 전진 · X/↓ 후진 · A/← 좌회전 · D/→ 우회전
                                      S/space 정지 · [ / ] 속도 · + / - 지도 확대·축소 · Esc 나가면 자동 정지
```

`5)`는 제어 상태가 MANUAL이 아니면 먼저 바꿀지 묻는다. 지도가 포커스를 잃으면(Tab/Esc) 즉시 정지하고, 화면을 나갈 때도 0 속도를 세 번 보낸다. 속도는 `[`/`]`, 지도 확대·축소는 `+`/`-`로 서로 겹치지 않는다. SLAM 중에는 AMCL이 없으므로 로봇 위치·방향은 TF(`map→base_footprint`)로 표시한다 — 지도 아래 `로봇 (x, y) θ` 줄과 빨간 `●`+화살표가 그것이다. 지도가 화면 밖으로 커지면 `f` 전체 보기, `r` 로봇 따라가기.

로봇은 빨간 `●`와 진행 방향 화살표, `M`은 map 원점, `O`은 odom 원점이다. `v`는 로봇을 움직이지 않고 planner에 경로만 물어본다. `c`는 코스트맵(전역/지역)을 겹쳐 본다. `s`가 켜져 있으면 벽이나 costmap 팽창/치명 칸을 찍어도 가장 가까운 안전한 빈 칸으로 옮겨서 웨이포인트를 놓는다. SLAM 중에는 `3) SLAM 맵핑 → 4) 지도 보기`로 지도가 그려지는 것을 보거나, `5) 지도 보면서 수동 이동`에서 WASD/방향키로 움직이며 live 지도를 볼 수 있다.

### RL ↔ fallback 토글 유지

`8) 노드 → controller_server → b) FollowPath.toggle`로 RL을 끄면 **런타임 값**이 `tools/.run/runtime_params.json`에 저장된다.

- Nav/`controller_server`를 **재시작하면 저장한 값(false = MPPI 바이패스)을 다시 적용**한다.
- 저장 파일이 없는 **첫 시작**은 `nav2_params.yaml`의 `FollowPath.enable` 기본값을 쓴다.
- fallback controller(MPPI/DWB)는 `nav2_params.yaml`에 저장되므로 재시작해도 유지된다.
- 저장값을 지워 기본값으로 돌리려면:

```bash
./run-cli fallback-enable status   # 저장된 값 확인 (없음 = YAML 기본값)
./run-cli fallback-enable off      # 다음 재시작에도 RL OFF 유지
./run-cli fallback-enable apply    # 지금 실행 중인 Nav에 즉시 적용
./run-cli fallback-enable clear    # 저장값 삭제 — 다음 시작은 YAML 기본값
```

메뉴에서는 `8) 노드 → controller_server → d) 저장된 FollowPath.enable 지우기`로 지울 수 있다.

### ③ 초기 위치 — **반드시 먼저**

Nav2는 로봇이 지도 어디에 있는지 알아야 켜집니다. 초기 위치를 주기 전에는 목표를 찍어도 **조용히 무시**됩니다.

- RViz: 위쪽 **2D Pose Estimate** → 지도에서 로봇 자리를 클릭하고 **앞쪽 방향으로 드래그**
- 또는 CLI (map 좌표, 각도는 °):

  ```bash
  ./run-cli initpose 0 0 0          # 지도 만들기 시작한 자리 = (0, 0, 0°)
  ```

  메뉴: `./run-cli` → `4) Nav` → `3) 지도에서 목표·초기위치·취소` → `i` (십자선 위치를 초기 위치로). 어디서 해도 됨 (`/initialpose` 토픽 → 로봇 amcl).

  출력이 `→ amcl 반영: x=… y=…` 이면 성공. RViz에서 초록 화살표 구름(파티클)이 로봇 주변에 모이고,
  빨간 스캔 점이 지도 벽과 겹치면 맞은 것. 어긋나면 다시 찍습니다.

초기 위치 후 Nav 주행부가 켜지기까지 로봇에서 수십 초 걸릴 수 있습니다 (`./run-cli nav log`에
`lifecycle_manager_navigation … Managed nodes are active`가 뜨면 준비 완료).

## 4. 특정 위치로 보내기

제어 상태: Nav2 명령은 `/state_control`가 **NAV_PLAN**(또는 SLAM_AUTO_MAPPING)일 때만 바퀴로 갑니다. RViz Nav2 Goal·`./run-cli goal`은 자동으로 NAV_PLAN으로 바꾸고,
텔레오퍼·`drive`는 MANUAL이 아니면 바꿀지 **물어봅니다** (수동 명령은 MANUAL에서만). 상단 상태바 `제어 …`로 확인.

### RViz에서 찍기

- 툴바 **2D Goal Pose** → 갈 곳 클릭 + 도착했을 때 바라볼 방향으로 드래그 (`/goal_pose` → 제어 상태가 자동으로 NAV_PLAN).
  ⚠ Nav2 패널의 다른 목표 버튼(GoalTool)은 `/goal_pose`를 안 거쳐 MANUAL 그대로 → 안 움직임. 그래서 nav.rviz엔 2D Goal Pose만 둠.
- 경로(초록 선)가 그려지고 출발. 다른 곳을 찍으면 목표가 바뀝니다.

### CLI로 좌표 보내기

```bash
./run-cli goal 1.0 0.5 90           # x=1.0 m, y=0.5 m, 도착 후 90°(왼쪽)를 보게 — 도착까지 기다림
./run-cli goal 1.0 0.5 90 --no-wait # 보내고 바로 끝냄
./run-cli goal --cancel             # 이 CLI가 보낸 목표 취소 (RViz 목표는 RViz에서 다시 찍기)
```

- 결과: `도착` / `취소됨` / `실패(ABORTED)` (막혀서 경로 없음 등).
- 메뉴: `./run-cli` → `4) Nav` → `3) 지도에서 목표·초기위치·취소` — 지도에서 `Enter`로 목표(여러 개 가능) → `g` 주행, `x` 취소, `i` 초기 위치.
- 좌표 찾기: RViz 아래쪽 상태줄에 마우스가 가리키는 map 좌표가 나옵니다. 또는 **Publish Point**로 찍으면 `/clicked_point`에 나옴.

좌표계 (`map`): 원점(0,0) = 지도 만들기를 **시작한 자리**, +x = 그때 로봇이 보던 앞쪽, +y = 왼쪽, 각도 0° = +x 방향, 반시계 +.

### 여러 곳 순서대로

```bash
for p in "1.0 0.0 0" "1.0 1.0 90" "0 0 180"; do ./run-cli goal $p || break; done   # 실패하면 멈춤
```

## 5. 지도 넓히며 주행 (SLAM 중 Nav)

지도를 아직 다 안 만든 상태에서 목표를 찍어 움직이며 지도를 넓힐 때. 초기 위치 불필요 (SLAM이 위치를 냄).

```bash
./run-cli slam start && ./run-cli explore start
./run-rviz slam                      # 2D Goal Pose로 찍기 (또는 ./run-cli goal x y yaw)
# 끝나면: map save → explore stop → slam stop
```

메뉴: `./run-cli` → `3) SLAM 맵핑` → `1) 시작` → `m` → `7) SLAM 중 Nav` → `1) 시작` → `4) 목표 보내기` (또는 RViz에서 찍기).
끝나면 `3) SLAM 맵핑` → `3) 지도 저장` → `7) SLAM 중 Nav` → `1) 정지` → `3) SLAM 맵핑` → `1) 정지`.

## 6. 끄기

```bash
./run-cli nav stop        # 또는 slam stop / explore stop
./run-cli stop            # 바퀴 정지 → 베이스까지 전부
```

메뉴: 각 메뉴(`2)`·`3)`·`4)`·`7)`)의 `1) 정지` · 전부는 메인 `9) 전부 정지·재시작` (확인창 — SLAM이 켜져 있으면 "저장 안 한 지도는 사라짐" 경고).
`./run-cli exit`·메뉴에서 `q`는 **CLI만** 끄고 로봇 ROS는 계속 돕니다.

## 7. 문제

| 증상 | 확인 |
|---|---|
| RViz에 아무것도 안 보임 (로봇·스캔·지도 다) | **베이스가 꺼져 있음** — 상태바 `○ 베이스` → `./run-cli base start` (SLAM·Nav는 베이스가 꺼져 있으면 시작이 거부됨) |
| RViz에 지도가 안 보임 | 상단 `map ✗`? → `./run-cli status`에서 SLAM/Nav 켜졌는지 · RViz는 `run-rviz`로 켰는지 (DDS 피어 설정이 들어감) |
| 목표를 찍어도 안 움직임 | 초기 위치를 줬는지 · Nav 준비 완료됐는지 (`nav log`) · 텔레오퍼가 같이 켜져 있으면 끄기 (명령이 섞임) |
| `실패(ABORTED)` | 목표가 벽 안/지도 밖 · 좁은 통로(로봇 폭 31 cm + 여유) · `nav log` |
| 베이스를 재시작한 뒤 위치가 튐 | 베이스(드라이버)를 다시 켜면 **odom이 0으로 초기화**됨 → Nav면 2D Pose Estimate로 다시, SLAM이면 저장 후 `slam restart` |
| 위치가 점점 틀어짐 | RViz 2D Pose Estimate로 다시 · 바퀴 미끄러짐 · 지도가 실제와 다름(다시 만들기) |
| 지도가 겹침·휨 | 맵핑 때 너무 빠름 → 다시 천천히, 출발점으로 돌아와 루프 닫기 |

## 파일

| 파일 | 내용 |
|---|---|
| `ros2_ws/src/tb3_bringup/config/cartographer_2d.lua` | SLAM(cartographer) 설정 (오도메트리 사용 · IMU 안 씀 · 사거리 0.12~8 m) |
| `ros2_ws/src/tb3_bringup/config/nav2_params.yaml` | Nav2 설정 (Waffle Pi footprint 다각형 · Twist cmd_vel · amcl `base_footprint`) |
| `rviz/slam.rviz` · `rviz/nav.rviz` | RViz 화면 |
| 로봇 `~/ros2/ros2_ws/map/<이름>/` | 저장한 지도 (`map.yaml` · `map.pgm`) |
| 로봇 `~/ros2/toolss/.run/<slam·nav·…>.log` | 실행 로그 (`./run-cli slam log`) |
