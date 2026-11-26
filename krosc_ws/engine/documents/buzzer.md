# Buzzer: OpenCR beep / melody

> 버저는 **소리만** 낸다. 제어와 분리되어 있고, LLM은 `buzzer` intent + `melody`만 고른다.
> 실제 재생은 `RobotStatus`의 20ms 시퀀서 → OpenCR `B,<Hz>,<ms>` → `tone()`이다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/robot_status.cpp` | `buzzer`, `buzzer/melody`, 멜로디 시퀀서 |
| `krosc_ws/engine/nodes/command_rules.cpp` | 20개 비프 문장 → melody |
| `krosc_ws/engine/hardwares/opencr/opencr_serial.cpp` | `B,<Hz>,<ms>` 전송 |
| `firmware/opencr/tb3_wheels/tb3_wheels.ino` | `tone(BDPIN_BUZZER, hz, ms)` |
| `krosc_ws/simulator/hardwares/opencr/simulator_buzzer.cpp` | Qt `QAudioSink`로 시뮬 소리 |
| `krosc_ws/viz-web/www/plugins/ai.js` | AI 패널 Mario/성공음 버튼 |

## 2. 서비스

```text
buzzer         : krosc_ws/srv/Buzzer {frequency, duration} → success
buzzer/melody  : String {mario_death|success|alarm|beep} → TriggerResponse
```

- `buzzer`는 단일 톤. `B,<Hz>,<ms>` (100~8000Hz, 10~2000ms clamp).
- `buzzer/melody`는 `RobotStatus`가 20ms 타이머로 음을 순서대로 보낸다.
- 단일 `buzzer`가 오면 진행 중 멜로디를 취소한다.

## 3. 멜로디

```cpp
if (name == "mario_death") {
  notes = {
    {1047, 80}, {1047, 80}, {1047, 80}, {0, 320},
    {988, 160}, {1397, 160}, {0, 160}, {1397, 160},
    {1397, 240}, {1319, 240}, {1175, 160}, {1047, 160},
    {0, 160}, {659, 160}, {0, 160}, {523, 160},
  };
} else if (name == "success") { {523,100},{659,100},{784,100},{1047,200} }
else if (name == "alarm")   { {880,180},{0,80} x3 }
else if (name == "beep")    { {2000,150} }
```

- `melody_index_`, `melody_next_`, 20ms 타이머.
- 각 음 후 `duration + 20ms` 대기. OpenCR `tone()`은 non-blocking.

## 4. 20개 자연어 명령

| # | 명령 | melody | 소리 |
|---:|---|---|---|
| 1 | 마리오 죽을 때 효과음 | `mario_death` | 마리오 죽음 16음 |
| 2 | 마리오 죽는 소리 | `mario_death` | 위와 같음 |
| 3 | 게임오버 소리 | `mario_death` | 위와 같음 |
| 4 | mario death | `mario_death` | 위와 같음 |
| 5 | 죽을 때 효과음 | `mario_death` | 위와 같음 |
| 6 | 성공음 | `success` | 도미솔도 상승 |
| 7 | 성공 소리 | `success` | 위와 같음 |
| 8 | 딩동댕 | `success` | 위와 같음 |
| 9 | 축하음 | `success` | 위와 같음 |
| 10 | success sound | `success` | 위와 같음 |
| 11 | 정답 소리 | `success` | 위와 같음 |
| 12 | 경고음 | `alarm` | 880Hz 3회 |
| 13 | 경고 알람 | `alarm` | 위와 같음 |
| 14 | 위험 알림 | `alarm` | 위와 같음 |
| 15 | alarm | `alarm` | 위와 같음 |
| 16 | 삐삐 | `alarm` | 위와 같음 |
| 17 | 삐 소리 | `beep` | 2000Hz 150ms |
| 18 | 비프음 | `beep` | 위와 같음 |
| 19 | beep | `beep` | 위와 같음 |
| 20 | 짧게 삐 | `beep` | 위와 같음 |

### 경계

- `축하해`는 **춤**(entertainment celebrate)이다. `축하음`만 버저 success.
- `삐삐`는 alarm, `삐 소리`는 beep.
- `마리오`/`게임오버`/`죽`+`소리`는 mario_death.

## 5. 경로

```text
"성공음"
  → command_rules: {"intent":"buzzer","melody":"success"}
  → llm_input::play_melody("success")
    → service buzzer/melody
      → RobotStatus::play_melody
        → opencr_.buzzer(523, 100) ...
          → 실제: serial "B,523,100" → tone()
          → 시뮬: SimulatorBuzzer::play()
```

## 6. 시뮬레이터 소리

`SimulatorOpenCr::buzzer()`가 `SimulatorBuzzer::play()`로 넘긴다.

- `QAudioSink` + `ToneStream(QIODevice)`; 44.1kHz mono Int16.
- 사각파(0.55 진폭)로 피에조에 가깝게. `sink->setVolume(1.0)`.
- 엔진 스레드 → `QMetaObject::invokeMethod(QCoreApplication::instance(), ...)`
  로 Qt 메인 스레드에서 재생.
- Qt Multimedia 없으면 컴파일 정의 `KROSC_WS_SIM_AUDIO` 없이 무음 스텁.
- macOS에서 `-framework AGL` 링크 오류가 나면 `WrapOpenGL`의 AGL 항목을 제거한다
  (`krosc_ws/CMakeLists.txt`).

## 7. 디버깅

| 증상 | 확인 |
|---|---|
| 소리가 안 남 | 시스템 볼륨, `sink->setVolume`, Qt Multimedia 링크 |
| 멜로디가 끊김 | `buzzer` 단일 톤이 멜로디를 취소했는지 |
| 음이 이상함 | OpenCR `B,Hz,ms` 범위 clamp, `tone()` non-blocking |
| 시뮬만 무음 | `KROSC_WS_SIM_AUDIO` 정의, 오디오 장치 |
| 명령이 버저로 안 감 | `command_rules` 경계 (`축하해` vs `축하음`) |

## 8. 한계

- 실제 OpenCR 부저는 단일 톤 `tone()`이다. 멜로디는 SBC가 음을 시간차로 보내는 방식.
- 시뮬레이터 소리는 Qt 오디오 장치/볼륨에 의존한다.
- 부저 서비스는 `robot_status` 내부 기능이지만 이름은 flat `buzzer`다
  (서비스 이름 규칙은 README 참고).
