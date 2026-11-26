# Buzzer: OpenCR beep / melody

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/robot_status.cpp` | `buzzer`, `buzzer/melody`, 멜로디 시퀀서 |
| `krosc_ws/engine/hardwares/opencr/opencr_serial.cpp` | `B,<Hz>,<ms>` 전송 |
| `firmware/opencr/tb3_wheels/tb3_wheels.ino` | `tone(BDPIN_BUZZER, hz, ms)` |
| `krosc_ws/simulator/hardwares/opencr/simulator_buzzer.cpp` | Qt `QAudioSink`로 시뮬 소리 |
| `krosc_ws/viz-web/www/plugins/ai.js` | AI 패널 Mario/성공음 버튼 |

## 2. 서비스

```text
buzzer         : krosc_ws/srv/Buzzer {frequency, duration} → success
buzzer/melody  : String {mario_death|success|alarm|beep} → TriggerResponse
```

- `buzzer`는 단일 톤. `B,<Hz>,<ms>` (100~8000 Hz, 10~2000 ms clamp).
- `buzzer/melody`는 `RobotStatus`가 20 ms 타이머로 음을 순서대로 `opencr_.buzzer`에 보낸다.
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

- `melody_index_`, `melody_next_`, 20 ms 타이머.
- 각 음 후 `duration + 20 ms` 대기. OpenCR `tone()`은 non-blocking.

## 4. 시뮬레이터 소리

`SimulatorOpenCr::buzzer()`가 `SimulatorBuzzer::play()`로 넘긴다.

- `QAudioSink` + `ToneStream(QIODevice)`; 44.1 kHz mono Int16.
- 사각파(0.55 진폭)로 피에조에 가깝게. `sink->setVolume(1.0)`.
- 엔진 스레드 → `QMetaObject::invokeMethod(QCoreApplication::instance(), ...)`
  로 Qt 메인 스레드에서 재생.
- Qt Multimedia 없으면 컴파일 정의 `KROSC_WS_SIM_AUDIO` 없이 무음 스텁.
- macOS에서 `-framework AGL` 링크 오류가 나면 `WrapOpenGL`의 AGL 항목을 제거한다
  (`krosc_ws/CMakeLists.txt`).

## 5. 예시

```text
"마리오 죽을 때 효과음 내줘" → intent=buzzer, melody=mario_death
"성공음 들려줘"              → success
"삐 소리 내줘"               → beep
```

`llm_input`의 buzzer 분기:

```cpp
if (intent == "buzzer") {
  const std::string name = lower(melody.empty() ? "beep" : melody);
  play_melody(name, error);  // buzzer/melody 서비스
  publish_status(id, "done", ",\"tool\":\"buzzer\",\"melody\":...");
}
```

## 6. 한계

- 실제 OpenCR 부저는 단일 톤 `tone()`이다. 멜로디는 SBC가 음을 시간차로 보내는 방식.
- 시뮬레이터 소리는 Qt 오디오 장치/볼륨에 의존한다. 소리가 작으면 시스템 볼륨과
  `sink->setVolume`을 확인한다.
- 부저 서비스는 `robot_status` 내부 기능이지만 이름은 flat `buzzer`다
  (서비스 이름 규칙은 README 참고).
