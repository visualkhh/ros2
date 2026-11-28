# ml/llm — 지시 해석 전략 (말 → intent · slot)

> 상태: **설계**. 코드 없음. 제공자(provider)는 아직 고르지 않았다 — 아래 기준으로 측정해서 정한다.

## LLM이 하는 일과 하지 않는 일

| 한다 | 하지 않는다 |
|---|---|
| 한국어 지시를 **intent + slot** JSON으로 | 관절·속도 같은 **모션 명령을 직접 만들기** |
| 여러 단계 지시를 작업 순서로 ("캔 다 넣고 바구니 비워줘") | 정지 명령 처리 (규칙이 즉시, LLM 거치지 않음) |
| 상태 질문에 답하기 (셀 상태·진행 상황 요약) | 영상 판단 (→ `vlm/`) |
| 모르면 되묻기 (보류) | 허용 목록에 없는 도구 호출 |

## 처리 순서 (기존 `engine/nodes/llm_input` 구조를 따른다)

```
입력 문장
  │
  ├─ 1) 안전 규칙 (결정적, 즉시): "멈춰/정지/스톱" → control.stop  ← LLM 안 거침
  ├─ 2) 명령 규칙 (결정적, command_rules): 자주 쓰는 문장 → intent    (빠르고 재현됨)
  └─ 3) LLM (규칙에 없을 때만): 시스템 프롬프트 + 소수 예시 + JSON 스키마 → intent
         └─ 스키마 검증 실패 → 1번 재시도 → 그래도 실패면 clarify (되묻기)
```

## intent 목록 (이 미션)

| intent | slot | 예 | 연결 |
|---|---|---|---|
| `sort.all` | — | "전부 분리수거해", "매트 정리해" | 작업 루프 시작 |
| `sort.category` | `category`: can·pet·paper | "캔만 넣어", "페트병 치워" | VLM 대상 필터 |
| `sort.item` | `description` (색·상표·위치) | "빨간 콜라캔 넣어", "왼쪽 병" | VLM 2단(속성) |
| `query.status` | `topic`: progress·bins·count | "지금 뭐 해?", "캔 몇 개 했어?" | `recycling/truth`(시뮬)·작업 로그 요약 |
| `query.scene` | — | "매트에 뭐 있어?" | `vlm/query` |
| `cell.supply` | — | "물건 더 가져다줘" | `recycling/request {"action":"supply"}` (실물: 작업자 호출 알림) |
| `cell.empty` | `bin` | "캔 바구니 비워줘" | `recycling/request {"action":"empty","bin":"can"}` |
| `control.stop` / `pause` / `resume` / `home` | — | "멈춰", "잠깐", "계속해", "원위치" | 규칙 처리 (LLM 경유 금지) |
| `nav.goto` | `place`: work_area·bins·home | "작업대로 가" | Nav 목표 |
| `clarify` | `question` | (모호·모름) | 사용자에게 되묻고 **아무 동작도 안 함** |

## 응답 형식 (스키마로 강제)

```json
{"intent": "sort.category",
 "slots": {"category": "can"},
 "confidence": 0.86,
 "needs_confirmation": false,
 "reply": "캔만 분리할게요."}
```

- `confidence < 임계값` 또는 slot 누락 → `clarify`
- 되돌리기 어려운 요청(예: "전부 비워")은 `needs_confirmation: true` → 확인 후 실행
- 온도 0, JSON 모드(지원하면 tool/function calling), 한국어 시스템 프롬프트 + intent별 예시 2~3개

## 맥락으로 넣는 것

- 로봇 상태 요약(현재 작업, 바구니 채움, 최근 실패) — 짧게, 숫자 위주
- 직전 대화 몇 턴 ("그것도 넣어" 같은 지시어 해석)
- **넣지 않는 것**: 원시 영상, 비밀값, 개인 정보

## 제공자 후보 (아직 결정 안 함)

| 후보 | 장점 | 걱정 | 확인할 것 |
|---|---|---|---|
| **Ollama (로컬, 이미 설치)** | 무료, 오프라인, 데이터가 밖으로 안 나감, 기존 `llm_client`가 이미 붙어 있음 | CPU에서 느림 (수 B 모델 Q4 기준 응답 수 초), 작은 모델의 한국어·JSON 정확도 | 모델별(예: 3B·7B Q4) 한국어 intent 정확도·지연 |
| **OpenCode 키 (유료)** | 큰 모델 품질, 빠른 응답 | 비용, 네트워크 의존, 지시 내용이 외부로 감 | 제공 API 형식(OpenAI 호환 여부), 요금, 지연 |
| **muspark 1.3 constructor** (언급한 후보) | 싸고 좋다는 평가 | 이 문서 작성 시점에 확인한 정보 없음 | 제공처·API 형식·가격·한국어·JSON/tool 지원을 직접 확인 |

**정하는 방법**: 같은 평가 세트(한국어 지시 100~200문장, 기대 intent·slot 포함)를 제공자마다 돌려 **intent 정확도, slot 정확도, 지연(p50/p95), 회당 비용, JSON 실패율**을 표로 비교한다. 개발 중에는 규칙 + Ollama, 평가 결과로 기본 제공자를 고른다.

**구조**: 제공자는 설정 하나로 바꾼다 — `chat(messages, schema) → JSON` 인터페이스 하나에 Ollama / OpenAI 호환 / 기타 구현. 키는 환경변수로만 (코드·로그·bag에 남기지 않는다 — 발제 보안 안내).

## 기록 (발제 ⑪)

회차 ID와 함께: 입력 문장, 규칙/LLM 중 어디서 처리됐는지, 제공자·모델·버전, 원출력(JSON), 검증 결과, 실행한 도구 호출, 지연. 저장된 원출력만으로 같은 판정을 다시 계산할 수 있게.

## 예정 구조

```
ml/llm/
  README.md          # 이 문서
  prompts/           # 시스템 프롬프트, intent별 예시
  eval/              # 평가 세트 (문장 → 기대 intent·slot), 제공자 비교 결과
  schema/            # 응답 JSON 스키마
```

## 다음에 할 일

1. 평가 세트 초안 (intent마다 10~20문장, 오타·존댓말·반말 섞기)
2. `command_rules`에 이 미션 intent 추가 (규칙으로 처리되는 비율 올리기)
3. Ollama 모델 2~3개로 기준선 → 유료 후보 비교
