# VLM: SmolVLM2 image Q&A

> VLM은 **질의응답/설명**용이다. 제어 루프에 넣지 않는다.
> LLM(Ollama)이 intent를 뽑고, `vlm_query`일 때만 `vlm/query`로 간다.

## 1. 소스

| 파일 | 역할 |
|---|---|
| `krosc_ws/engine/nodes/vlm/vlm_node.cpp` | 최신 D435 프레임 보관, `vlm/query` 서비스 |
| `krosc_ws/tools/vlm_server.py` | SmolVLM2-500M HTTP sidecar `/query` |
| `krosc_ws/engine/nodes/llm_input/llm_input.cpp` | `vlm_query` → `vlm/query` |
| `krosc_ws/viz-web/www/plugins/ai.js` | AI 패널 결과 표시 |

## 2. 경로

```text
"뭐 보여?" → LLM intent=vlm_query
  → llm_input: call_optional("vlm/query", {question})
    → vlm_node: 최신 camera/color/image_raw (rgb8 또는 jpeg)
      → POST http://127.0.0.1:8766/query
        {image_b64, encoding, width, height, question, context}
          → vlm_server: AutoProcessor + AutoModelForImageTextToText
            → answer
  → llm/status {answer}
```

- `vlm_node`는 카메라 콜백에서 최신 `msgs::Image`만 들고 있다가 요청 시 1회 전송한다.
- 서비스 호출은 SmolVLM2 추론(1~3초) 동안 블록한다. `llm_input` 워커에서 호출하므로
  LLM/다른 서비스 큐가 막히지는 않는다.
- `llm/status`의 `answer`로 AI 패널에 표시된다.

## 3. 모델

`context`에는 `robot_context` JSON(TF/URDF 크기/라이다/카메라/팔/바퀴/배터리)이
문자열로 포함된다. 질문에 답할 때 로봇의 현재 위치/자세/가시 범위를 함께 고려한다.

```text
HuggingFaceTB/SmolVLM2-500M-Video-Instruct
revision 4cde9d208e19519c36917aa170fffc7e381aa0ad
```

- `vlm_server.py`가 `AutoProcessor` + `AutoModelForImageTextToText`를 로드.
- `do_image_splitting=False`로 단일 프레임 추론.
- device: Mac MPS, 없으면 CPU. `--device cpu`로 강제 가능.
- 모델 캐시: `krosc_ws/artifacts/models/vlm/hf-cache` (`HF_HOME`).

## 4. sidecar 실행

로봇(Pi)에는 torch/transformers를 설치하지 않는다. **PC에서 sidecar를 띄우고
로봇이 PC LAN IP로 접속**한다. Pi와 PC가 분리된 구성에서는 반드시 `--host 0.0.0.0`.

```bash
# PC 최초 1회 (venv + 의존성 + 모델 캐시 symlink)
krosc_ws/tools/ai-sidecars.sh start

# 개별 실행
krosc_ws/tools/vla-venv/bin/python krosc_ws/tools/vlm_server.py   --host 0.0.0.0 --device cpu --threads 8
# ready http://0.0.0.0:8766/query
```

로봇 쪽:

```bash
KROSC_VLM_URL=http://10.2.150.129:8766/query ./build/krosc_real
```

PC 방화벽(UFW)이 켜져 있으면 8766을 열어야 한다.

```bash
sudo ufw allow from 10.2.150.0/23 to any port 8766 proto tcp
```

필수 패키지: `torch`, `torchvision`, `transformers==5.10.4`, `num2words`, `pillow`.
`torchvision`이 없으면 SmolVLM image processor import가 실패하고
`Unrecognized image processor`가 난다. `num2words`도 필요하다.

### 연결 실패 체크

| 증상 | 원인 |
|---|---|
| `연결 실패: 127.0.0.1:8766` | 로봇이 자기 localhost를 봄 → `KROSC_VLM_URL`에 PC IP 지정 |
| 응답 없이 60초 timeout | PC UFW에서 8766 차단 또는 sidecar가 `127.0.0.1`에만 바인딩 |
| `Unrecognized image processor` | `torchvision` 누락 |
| 모델 다운로드 반복 | `artifacts/models/vlm/hf-cache`가 비어 있음 → `ai-sidecars.sh`가 `~/.cache/huggingface`를 symlink |

## 5. 예시

```bash
# health
curl -s http://127.0.0.1:8766/health
# {"ok": true, "model": "HuggingFaceTB/SmolVLM2-500M-Video-Instruct"}

# 웹 AI 패널
"뭐 보여?" → vlm_query
"앞에 뭐가 있어?" → vlm_query
"빨간 퍽 보여?" → vlm_query
```

## 6. 한계

- 500M 모델이라 세밀한 물체 위치/거리 추정은 약하다. 거리는 depth/라이다로 따로 잰다.
- 학습/Isaac과 krosc 시뮬의 조명/렌더가 달라 설명이 틀릴 수 있다.
- VLM 답변을 제어에 직접 쓰지 않는다. 제어는 LLM 라우터 + 결정적 도구/VLA.
