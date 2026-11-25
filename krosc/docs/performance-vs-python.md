# C++ `krosc` vs Python `kros` 성능 비교

측정일: **2026-11-24 (KST)**  
대상: TurtleBot3 실기 + Raspberry Pi + PC Viz

> **2026-11-25 갱신**
> C++ `krosc_ws`는 이제 nav2 풀스택(Navfn/Smac, RPP/DWB/MPPI, costmap, BT), ros2_control
> (`controller_manager`, JSB/JTC/diff_drive), MoveIt 플러그인까지 돌린다. 아래 Pi 수치는
> 2026-11-24의 **A* + pure pursuit** 시점이므로 현재 C++ 풀스택의 수치가 아니다.
> 현재 스택의 알고리즘 비용은 **10장의 Mac 마이크로벤치마크**를 참고하고, Pi는 8장 샘플러로
> 다시 측정해야 한다.

---

## 1. 요약

| 항목 | Python 풀스택 | C++ 풀스택 | 차이 |
|---|---:|---:|---:|
| Pi CPU (1코어 기준) | **33~36%** | **3.5~5.1%** | **약 6.5~10배 낮음** (1 client 주행 8~9배) |
| Pi CPU (4코어 전체 환산) | 8.3~9.0% | **0.9~1.3%** | 약 7~9배 낮음 |
| Pi load average | 1.2~1.5 | **0.4~0.9** | 약 2~3배 낮음 |
| Pi RSS | 로그 없음 | **17.9~23.7 MiB** | - |
| PC Viz CPU (1코어 기준) | - | **14~18%** | 24코어 전체의 약 0.6~0.75% |
| PC Viz RSS | - | **66.9 MiB** | - |
| PC Viz FPS | - | **30 fps** | QPainter CPU 렌더링 |

핵심:

- **Pi는 주행·SLAM·자율주행 중에도 1코어의 5% 안팎**만 사용한다.
- 4코어 전체로 보면 **약 1%** 수준이라, 센서·SLAM·네트워크가 CPU 병목이 아니다.
- Python은 GIL 때문에 코어 1개를 거의 혼자 쓰는 구조였고, 33~36%에서도 여유가 크지 않았다.
- 단일 클라이언트 주행 기준으로는 **약 8~9배**, 2 clients를 포함해도 **6.5배 이상** 낮다.
- **단, 이 Pi 수치는 C++이 아직 A* + pure pursuit 최소 nav이던 시점의 값**이다. 2026-11-25 이후
  C++도 nav2 풀스택 + ros2_control + MoveIt을 돌리므로, Pi 수치는 다시 측정해야 한다.
  언어/구현 차이는 10장의 동일 알고리즘 마이크로벤치마크로 분리했다.

---

## 2. 측정 환경

### Raspberry Pi

| 항목 | 값 |
|---|---|
| OS | Ubuntu 26.04.1 LTS |
| Kernel | Linux 7.0.0-1023-raspi |
| Arch | aarch64 |
| CPU | 4 cores |
| RAM | 3.7 GiB |
| 실행 바이너리 | `krosc_ws/build/krosc_real` |
| 측정 시 소스 | `f81231c8` (2026-11-24 11:05 빌드) |
| 지도 | `map_20261123_204217` |
| 모드 | localization |

### PC

| 항목 | 값 |
|---|---|
| OS | Linux x86_64 |
| Kernel | 7.0.0-43-generic |
| CPU | Intel Core Ultra 9 275HX, 24 logical cores |
| RAM | 30 GiB |
| Viz | Qt6 QPainter, 30 fps (33ms repaint) |
| 측정 시 소스 | `0cf70c8` |

> 측정 후 멀티 클라이언트 서버(`449770a`)를 추가하고 Pi에 배포했다. 아래 `2 clients` 항목은 그 이후 재측정한 값이다.

---

## 3. 측정 방법

### Pi CPU

Python 프로파일러와 같은 방식으로 `/proc`를 직접 샘플링했다.

```text
CPU% = (utime + stime 변화량) / CLK_TCK / 경과시간 * 100
```

- **1개 코어 = 100%** 기준
- 1초 간격, 프로세스 전체 + 스레드별
- RSS는 `/proc/<pid>/status`의 `VmRSS`
- load는 `/proc/loadavg`

### PC Viz CPU

```bash
pidstat -p <viz_pid> 1 <seconds>
```

- `%CPU`는 1개 코어 기준
- 24개 논리 코어 전체 환산은 `%CPU / 24`

### 주행 시나리오

| 시나리오 | 내용 |
|---|---|
| localization + teleop | 0.08 m/s, 1.2초 전진 → 0.2초 정지 → 1.2초 후진 → 0.2초 정지, 4사이클 |
| SLAM mapping + teleop | 같은 주행, 3사이클. mapping 모드에서 지도 생성 |
| 자율주행 | 현재 map pose 기준 **0.45 m 전진 goal → 0.45 m 후진 복귀 goal**, 둘 다 `reached` 확인 |
| multi-client | live Viz + 벤치 클라이언트가 **동시에** 접속한 상태에서 자율주행 |

### Python 기준값

같은 Pi에 남아 있던 Python 풀스택 프로파일 로그(2026-11-23, `/tmp/kos.log`)를 사용했다.

```text
[prof] load=1.2~1.5 cpu=33~36%
```

- Python 앱은 `kos_ws/main.py` 기준으로 **Smac, MPPI, local/global costmap, behavior server, bt_navigator, collision monitor**까지 포함한다.
- C++은 현재 **A* global planner + pure pursuit + final yaw**만 사용한다.
- 따라서 이 비교는 **"현재 실기 스택 전체의 CPU 차이"**이지, 동일 알고리즘을 언어만 바꾼 순수 언어 비교는 아니다.

---

## 4. Pi 측정 결과

### 4.1 모드별 CPU / 메모리

| 모드 | CPU 평균 | CPU 범위 | RSS | load | 샘플 |
|---|---:|---:|---:|---:|---:|
| localization idle | **3.50%** | 3~4% | 17.9 MiB | 0.66 | 8 |
| localization + teleop | **3.93%** | 3~5% | 17.9 MiB | 0.67 | 14 |
| SLAM mapping idle | **3.50%** | 2~5% | 17.9 MiB | 0.61 | 6 |
| SLAM mapping + teleop | **4.08%** | 3~5% | 20.1 MiB | 0.44 | 12 |
| 자율주행, 1 client (active) | **3.90%** | 3~5% | 20.1 MiB | 0.43 | 10 |
| 자율주행, 2 clients (live Viz + bench) | **5.08%** | 4~6% | 23.7 MiB | 0.88 | 12 |

- 1 client 자율주행의 `3.90%`는 goal을 따라 실제로 움직인 **첫 10초** 평균이다. 30초 전체로 잡으면 정지 구간이 포함되어 2.90%까지 내려간다.
- 4코어 전체 환산: **3.50~5.08% / 4 = 0.88~1.27%**
- 스레드는 7개이며, 어느 한 스레드도 2%를 넘지 않았다. (측정 시 스레드 이름은 전부 `krosc_real`로 나왔다.)

### 4.2 SLAM mapping

- 3사이클(8.6초) 주행 후 지도 **6007 cells**
- RSS: **17.9 → 20.1 MiB** (+2.2 MiB)
- localization 지도(11696 cells) 로드 후에도 RSS는 20.1 MiB
- multi-client 2명 접속 시 23.7 MiB (+3.6 MiB)

지도 격자와 클라이언트 큐를 포함해도 **24 MiB 미만**이다.

### 4.3 자율주행

| 조건 | 전진 goal | 후진 복귀 goal | 결과 |
|---|---:|---:|---|
| 1 client | 1.90초, plan 12 | 6.76초, plan 8 | 둘 다 `reached` |
| 2 clients | 3.21초, plan 18 | 7.82초, plan 16 | 둘 다 `reached` |

- 후진 복귀는 180° 회전이 포함되어 전진보다 오래 걸린다.
- 2 clients에서 시간이 더 걸린 것은 출발 자세/경로 길이 차이이며, CPU가 원인은 아니다.
- live Viz는 2 clients 테스트 동안 **재접속 0회**로 계속 살아 있었다.

### 4.4 Python과 비교

| 스택 | Pi CPU (1코어) | Pi 전체(4코어) | load |
|---|---:|---:|---:|
| Python 풀스택 (기존 로그) | 33~36% | 8.3~9.0% | 1.2~1.5 |
| C++ 풀스택 (주행) | 3.9~5.1% | 0.9~1.3% | 0.4~0.9 |

단일 클라이언트 주행 기준 **약 8~9배**, 2 clients 포함 최대 5.1%에서도 **6.5배 이상** 낮다.

Python 프로파일러 코드에는 이런 주석이 있다.

> GIL 때문에 이 앱은 코어 하나만 쓴다: 75%만 넘어도 다른 스레드(제어 20Hz)가 밀린다.

Python은 35%에서도 단일 코어를 공유하는 구조라 제어 주기 여유가 크지 않았고, C++은 5% 수준에서 7개 스레드가 나뉘어 돌아 **제어 주기 여유가 훨씬 크다**.

---

## 5. PC Viz 측정 결과

| 상태 | CPU 평균 (1코어) | RSS | FPS |
|---|---:|---:|---:|
| idle | 17.88% | 66.9 MiB | 30 |
| localization + teleop | 17.50% | 66.9 MiB | 30 |
| SLAM mapping + teleop | 17.40% | 66.9 MiB | 30 |
| 자율주행, 2 clients | **14.25%** | 66.9 MiB | 30 |

- 24개 논리 코어 전체 환산: **0.6~0.75%**
- 모드별 차이가 거의 없다. 즉 **센서/지도 데이터 처리보다 30fps QPainter 렌더링이 CPU를 지배**한다.
- Pi보다 PC Viz가 전체 시스템에서 더 큰 CPU 소비자다.
- 필요하면 repaint를 20fps로 낮추거나 OpenGL 렌더러로 바꿀 수 있다.

---

## 6. 해석

### 좋아진 것

1. **Pi CPU가 1% 수준**이라 SLAM/자율주행을 동시에 돌려도 4코어 중 3코어 이상이 남는다.
2. **GIL 병목 제거**로 제어 20Hz, SLAM, 네트워크가 서로 밀리지 않는다.
3. **메모리도 20 MiB 내외**라 Pi에서 다른 작업을 같이 돌릴 여유가 크다.
4. **멀티 클라이언트**가 가능해져 PC Viz와 벤치/디버그 도구를 동시에 붙일 수 있다.

### 주의할 점

- Python은 **nav2 전체**, C++은 **최소 nav**이다. C++에 MPPI/Smac/local costmap을 포팅하면 CPU는 지금보다 올라간다. 그래도 현재 여유(1코어의 95%)를 보면 충분히 수용 가능하다.
- Python RSS는 기존 로그에 없어서 비교하지 않았다.
- load average는 다른 프로세스/SSH의 영향을 받으므로 CPU%가 더 신뢰할 만하다.
- PC Viz CPU는 창 크기, 탭, 표시 항목, FPS 설정에 따라 달라진다.

---

## 7. 멀티 클라이언트 (측정 후 적용)

측정 중 Pi 서버가 **클라이언트 1개만 허용**하는 것을 발견했다. 새 클라이언트가 붙으면 기존 클라이언트를 끊어서, live Viz와 벤치 도구가 서로 재접속을 반복했다.

커밋 `449770a`에서 수정:

- 연결마다 **reader/writer 스레드 + bounded queue**
- 느린 클라이언트가 다른 클라이언트를 막지 않음
- 종료된 연결은 **reaper 스레드**가 join
- 래치 토픽은 최신 1개만 재전송 (`tf_static`만 전체 유지)
- 최대 **8 clients**, listen backlog 16

검증:

```text
2 clients 동시 접속                    OK
한 클라이언트 명령 → 다른 클라이언트 유지  OK
live Viz + 두 번째 Viz 스크린샷         OK
자율주행 중 live Viz 재접속             0회
```

---

## 8. 재현 방법

### Pi CPU 샘플러

```python
import os, sys, time

pid = int(sys.argv[1])
duration = float(sys.argv[2])
clock = os.sysconf("SC_CLK_TCK")

def ticks(path):
    fields = open(path).read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])

last_total = ticks(f"/proc/{pid}/stat")
last = time.monotonic()
t0 = last
while time.monotonic() - t0 < duration:
    time.sleep(1.0)
    now = time.monotonic()
    dt = max(now - last, 1e-6)
    last = now
    total = ticks(f"/proc/{pid}/stat")
    cpu = (total - last_total) / clock / dt * 100.0
    last_total = total
    rss = next(int(line.split()[1]) for line in open(f"/proc/{pid}/status")
               if line.startswith("VmRSS:"))
    print(f"cpu={cpu:.1f}% rss={rss/1024:.1f}MiB load={os.getloadavg()[0]:.2f}", flush=True)
```

### PC Viz

```bash
pidstat -p "$(pgrep -x krosc_viz_app)" 1 10
```

### 주행

- teleop: `krosc::net::Client`로 `cmd_vel`을 20Hz publish (0.08 m/s, 1.2초 전진/후진)
- 자율주행: `navigate_to_pose` 액션 goal을 map 좌표로 보내고 `goal_status`가 `reached`가 될 때까지 대기
- SLAM: `cartographer_node`의 `mode` 파라미터를 `mapping`으로 설정

---

## 9. 2026-11-25 Mac nav2 마이크로벤치마크 (C++ vs Python)

Pi 전체 스택 재측정 전에, **같은 알고리즘을 같은 맵/횟수로** 돌려 언어/구현 차이를 분리했다.
C++는 Release `-O2`, Python은 CPython이다. 맵은 200x200(0.05 m, 10 m x 10 m)이고 내부 벽/방 구조가 같다.

| 작업 | C++ `krosc` | Python `kros` | 배수 |
|---|---:|---:|---:|
| costmap inflate (50회) | **1.708 ms/op** | 163.164 ms/op | **95.5x** |
| Navfn (A*, 100회) | **19.710 ms/op** | 174.941 ms/op | **8.9x** |
| Theta* (50회) | **60.026 ms/op** | 3663.412 ms/op | **61.0x** |
| Smac2D (50회) | **19.640 ms/op** | 192.149 ms/op | **9.8x** |
| SmacHybrid (10회) | **29.040 ms/op** | 172.965 ms/op | **6.0x** |
| RPP (2000회) | **0.00085 ms/op** | 0.058 ms/op | **68x** |
| DWB (100회) | **0.136 ms/op** | 2.380 ms/op | **17.5x** |
| MPPI (100회) | **0.170 ms/op** | 3.726 ms/op | **21.9x** |

- 재현:
  ```bash
  # C++ (Release)
  cmake -S krosc -B krosc/build-bench -DCMAKE_BUILD_TYPE=Release \
        -DKROSC_BUILD_BENCH=ON -DKROSC_BUILD_TESTS=OFF \
        -DKROSC_BUILD_VIZ=OFF -DKROSC_BUILD_EXAMPLES=OFF
  cmake --build krosc/build-bench --target krosc_nav2_bench -j
  ./krosc/build-bench/engine/krosc_nav2_bench

  # Python (ros2/에서)
  PYTHONPATH=. python3 kros/benchmarks/nav2_bench.py
  ```
- 둘 다 **같은 맵·같은 start/goal·같은 횟수**다. C++ 플래너는 `std::map` 기반 open set,
  Python은 dict/heapq라 자료구조 차이도 포함된다.
- **costmap inflate**가 약 95배 차이 난다. local costmap 5Hz와 지도 로드에서 가장 체감이 큰 부분이다.
- **Theta***는 Python에서 3.66 s/op로 사실상 실시간 계획에 쓰기 어렵다. C++도 61 ms라
  1Hz 재계획에는 충분하지만, 200x200 맵에서는 Navfn/Smac2D가 더 낫다.
- **MPPI**는 C++ 0.18 ms/op, Python 3.7 ms/op다. 20Hz 제어 주기(50 ms) 기준으로 둘 다 여유가
  있지만, Pi에서는 C++ 쪽이 훨씬 안전하다.
- MoveIt/ros2_control은 Python `kros`에 동일 구현이 없어 이 표에 넣지 않았다. C++ 쪽은
  `krosc_ws/tests/test_moveit_stack.cpp`에서 실제 OpenMANIPULATOR URDF로 IK → 계획 → JTC 실행을
  확인했다.

---

## 10. 결론

현재 C++ 실기 스택은 **Pi CPU 1% 내외(4코어 환산), RSS 20~24 MiB**로 매우 가볍다.

Python 풀스택 대비 주행 중 CPU가 **단일 클라이언트 기준 약 8~9배 낮고**, 멀티 클라이언트 2명이 붙어도 **1코어의 5%** 수준이다. 지금 상태라면 SLAM, 자율주행, 원격 Viz, 추가 디버그 도구를 동시에 돌려도 Pi가 병목이 될 가능성은 낮다.

다만 위 Pi 수치는 C++이 A* + pure pursuit이던 시점의 값이다. 현재 C++은 nav2 풀스택 +
ros2_control + MoveIt을 돌리므로, **Pi에서 같은 주행 시나리오로 다시 측정**해야 한다. 그 전까지는
9장의 Mac 마이크로벤치마크가 알고리즘별 비용 비교의 기준이다.
