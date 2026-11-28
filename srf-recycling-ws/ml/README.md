# ml — 학습 기본 구조

> 상태: `vlm/`은 데이터 생성 → SmolVLM2 LoRA 학습 → 평가 → ONNX 까지 코드가 있다. `vla/`·`llm/`은 아직 설계만.
> 전제: **GPU(CUDA) 없음, CPU만.** 모든 선택은 "CPU에서 학습 가능 + CPU에서 실시간 추론 가능"을 먼저 따진다.

| 폴더 | 맡는 일 | 미션에서 |
|---|---|---|
| [`llm/`](llm/README.md) | 말로 한 지시 → intent·slot (무엇을 할지) | 작업 지시 해석, 상태 질문, 셀 요청(공급·비우기) |
| [`vlm/`](vlm/README.md) | 영상 + 지시 → 대상 물품 선택 또는 **보류** | 물품 선택(정상·모호·부재), 결과 확인 보조 |
| [`vla/`](vla/README.md) | 영상 + 지시 + 로봇 상태 → 팔 행동(접근·집기·놓기) | VLA 담당 동작 |

## 한 회차의 흐름

```
지시(말) ──▶ llm: intent {sort.category: can}
               │
영상 ──────▶ vlm: 대상 선택 {item, bbox} 또는 보류(모호·부재) ──▶ 보류면 여기서 멈춘다
               │
깊이·TF ───▶ 3D 위치 (카메라 → base_link, mm) ──▶ Nav 이동
               │
영상·상태 ─▶ vla: 행동 청크 ──▶ 어댑터 검사(좌표계·단위·범위, 범위 밖 거부) ──▶ 팔
               │
새 영상 ───▶ vlm/기하: 바구니 안인가 → 성공·실패·미확인
```

## 위치: 왜 워크스페이스 최상위 `ml/`인가

- ML은 시뮬레이터 전용이 아니다: 실물 사진 수집·실물 평가·로봇/PC 추론까지 다룬다. 시뮬레이터는 **데이터 출처 하나**.
- 시뮬레이터는 공개된 입구로만 쓴다 — `simulator/tools/`의 스냅샷·데이터셋 도구(렌더러·월드 내부가 필요한 것은 거기), `srf bag` 기록, 토픽(`camera/*`, `recycling/truth`, `map_markers`). `ml/`이 시뮬레이터 코드를 직접 링크하지 않는다.
- 추론 노드(`engine/nodes/vlm`, `vla`, `llm_input`)는 엔진에 있다. `ml/`은 모델을 **만드는** 쪽 (데이터 → 학습 → 평가 → `artifacts/models/`로 내보내기).

```
srf-recycling-ws/
  simulator/tools/   데이터 공장 (snapshot, dataset 생성기) ← ml/* 가 결과 파일을 읽는다
  ml/{vlm,vla,llm}/  데이터 정리·학습·평가·내보내기 (python, CPU)
  engine/nodes/      추론 노드 (C++), artifacts/models/의 모델을 읽는다
```

## 공통 원칙

- **CPU 친화**: 작은 모델(수백만~수억 파라미터), 양자화(int8/Q4), ONNX/GGUF로 내보내 CPU 런타임에서 돌린다. 엔진은 이미 **onnxruntime(CPU)** 를 쓴다 (`engine/ml/onnx_policy.cpp`, RL 주행) → 같은 런타임을 우선 쓴다.
- **시뮬레이터가 데이터 공장**: `srf_recycling_ws_snapshot`(헤드리스 영상·깊이·진실 JSON), `recycling/truth`(정답), 시드 재현, 장면 `normal/multi/ambiguous/absent`, 비대상·사람·지게차로 방해 요소. 진실값으로 라벨을 자동으로 만든다.
- **sim2real**: 시뮬 영상만으로는 실물에서 틀린다. 도메인 무작위화(조명·텍스처·카메라 잡음·배치) + **실물 사진 소량**을 섞고, 평가는 반드시 실물 데이터로 따로 한다. (발제: 시뮬 성공은 실물 성공에 합산하지 않는다)
- **기록·재현**: 모든 추론은 회차 ID와 함께 입력(영상·지시)·모델 버전·원출력·변환 결과·채택/거부를 남긴다 (`srf bag`, 발제 ⑪). 저장된 원출력으로 판정을 다시 계산할 수 있어야 한다.
- **안전은 모델 밖에서**: 정지·관절 한계·속도·작업 영역·충돌 검사는 엔진 노드(`vla_node` 등)가 소유한다. 모델 출력은 언제나 "후보"다.

## 예정 폴더 구조 (각 폴더 공통)

```
ml/<이름>/
  README.md        # 이 문서: 방식·데이터·학습·평가
  configs/         # 학습·평가 설정 (yaml)
  scripts/         # 데이터 생성 → 학습 → 평가 → 내보내기 (python, CPU torch)
  serve/           # 추론 sidecar 또는 C++ onnxruntime 연결
  data/            # 생성 데이터 (git 제외, 큰 파일은 artifacts/로)
```

Python 환경은 CPU 전용 PyTorch(`pip install torch --index-url https://download.pytorch.org/whl/cpu`) 하나로 맞춘다. 모델 파일은 `artifacts/models/<이름>/` (기존 규약).

## 엔진에 이미 있는 연결 지점 (srf-default-ws에서 복사)

| 노드 | 인터페이스 | 지금 하는 일 |
|---|---|---|
| `engine/nodes/llm_input` | service `llm/command`, topic `llm/status` | Ollama `/api/chat`로 intent 추출, 규칙(`command_rules`) 우선 |
| `engine/nodes/vlm` | service `vlm/query` | 최신 카메라 프레임을 SmolVLM2 sidecar(HTTP)에 질문 |
| `engine/nodes/vla` | service `vla/command`, `vla/stop`, topic `vla/status` | VLA sidecar(HTTP)를 4 Hz로 호출해 청크 실행, 주기·안전·컨트롤러 전환 소유 |

ml/ 작업은 이 노드들의 **모델 쪽**을 채운다. 노드 인터페이스는 가능한 한 유지한다.
