# ml/vlm — 물품 선택 (영상 + 지시 → 대상 또는 보류)

SmolVLM2(기본 256M)를 **시뮬레이터가 자동으로 만든 라벨 데이터**로 LoRA 미세조정하고 ONNX로 내보낸다. CPU만 쓴다.

```
srf_recycling_ws_dataset ──▶ images/*.png + labels.jsonl   (헤드리스 렌더, 물체 ID 버퍼로 정답 자동)
          vqa.py         ──▶ 질문·답 jsonl                 (그 이미지에 실제로 보이는 것으로 정답)
          train.py       ──▶ RUN/adapter, RUN/merged       (LoRA, 답 토큰에만 손실)
          evaluate.py    ──▶ 유형별 최초 판단 정답률 · bbox IoU · 회차 기록
          export_onnx.py ──▶ vision_encoder / embed_tokens / decoder_model_merged .onnx
```

## VLM이 배우는 것

입력: RGB 이미지 + 질문(지시). 출력: 글자(JSON). bbox 도 글자로 낸다 — 이미지 크기에 대한 0~1000 정수 `[x0, y0, x1, y1]`.

| 과제 | 질문 예 | 답 |
|---|---|---|
| select (선택·보류) | `Instruction: sort the can. Choose one object to grab, or hold. Reply in JSON.` | `{"decision":"select","reason":"ok","count":2,"target":{"category":"can","label":"cola","bbox":[..]}}` · `{"decision":"hold","reason":"ambiguous","bbox":[..]}` · `{"decision":"hold","reason":"absent"}` |
| detect (검출) | `List every object you can see on the work mat in JSON.` | `[{"category":"pet","label":"small_tea","bbox":[..]}, ...]` (왼쪽부터) |
| where (지도 마커) | `Where is the can bin? Reply in JSON.` | `{"name":"bin_can","visible":true,"bbox":[..]}` |

정답 규칙 (`vqa.py`, 결정적 코드 → 재현·검증 쉬움):
- **보이는 것**만 센다: ID 버퍼 픽셀 ≥ `--min-pixels`, 가려지지 않은 비율 ≥ 0.3. 장면 설정이 아니라 그 이미지 기준이라 가려짐·화면 밖이 반영된다.
- 대상 종류가 보이면 `select` — 여럿이면 미션 규칙 "접근·집기 가능한 것부터": 가깝고 · 덜 가려지고 · 화면에 다 들어온 것.
- 대상은 없고 분류 불확실 물품(`unknown`: 뚜껑 컵, 테트라팩)이 보이면 `hold/ambiguous`, 둘 다 없으면 `hold/absent`.
- 지시 문장은 영어 템플릿이다 (작은 모델은 한국어가 약하다). 사람의 한국어 지시는 `ml/llm`이 intent로 바꾸고, 그 intent를 이 문장으로 만든다.
- 지도 마커(`map_marker`의 바구니·매트)를 카메라로 투영해 "어디 있나" 정답을 만든다.

## 실행 (CPU)

```bash
cd srf-recycling-ws
uv venv ml/vlm/.venv --python 3.12 && uv pip install --python ml/vlm/.venv/bin/python -r ml/vlm/requirements.txt
./build.sh simulator && source paths.sh
D=artifacts/datasets   # git 에 안 올린다

# 1) 데이터: 학습·검증은 시드를 다르게
$SRF_RECYCLING_WS_BUILD_DIR/srf_recycling_ws_dataset --out $D/train --count 800 --seed 11
$SRF_RECYCLING_WS_BUILD_DIR/srf_recycling_ws_dataset --out $D/val   --count 100 --seed 99
ml/vlm/.venv/bin/python ml/vlm/vqa.py $D/train --out $D/train.jsonl --seed 1
ml/vlm/.venv/bin/python ml/vlm/vqa.py $D/val   --out $D/val.jsonl   --seed 2

# 2) 학습 → 3) 평가 → 4) ONNX (+ torch 와 같은 답인지 확인) → 5) ONNX 로 다시 평가
R=artifacts/models/vlm/runs/v1
ml/vlm/.venv/bin/python ml/vlm/train.py --data $D/train.jsonl --out $R --epochs 2
ml/vlm/.venv/bin/python ml/vlm/evaluate.py --data $D/val.jsonl --model $R/merged --records $R/eval.jsonl
ml/vlm/.venv/bin/python ml/vlm/export_onnx.py --model $R/merged --out artifacts/models/vlm/onnx/v1 --check $D/val/images/000001.png
ml/vlm/.venv/bin/python ml/vlm/evaluate.py --data $D/val.jsonl --onnx artifacts/models/vlm/onnx/v1
```

- 모델 캐시는 `artifacts/models/vlm/hf-cache` (`HF_HOME`, `smol.py`가 정한다). 처음 한 번만 받는다.
- 버전은 `srf-default-ws/tools/vla-venv`·`vlm_server.py`와 같은 torch 2.15 / transformers 5.10.4.

### 데이터 생성기 `srf_recycling_ws_dataset` (`simulator/tools/dataset.cpp`)

항상 헤드리스. 장면마다 무작위: 장면 유형(정상 35 · 다수 20 · 모호 30 · 부재 15 %), 지시 대상, 물품 수·자세, 방해물, 사람·지게차(움직인 뒤), 로봇 위치(매트를 봄 70 · 바구니 앞 20 · 아무 데 10 %), 팔 자세, 조명, 잡음.
`labels.jsonl` 한 줄 = 이미지 하나: 카메라 내부·외부 파라미터, 로봇·팔, 장면, 물체마다 종류·변형·누움·월드/카메라 좌표·거리·`bbox`(보이는 부분)·`bbox_full`(가려지지 않았을 때)·`visible_ratio`·`truncated`, 지도 마커·구역의 투영 상자.
옵션: `--out --count --seed --size W,H --min-pixels --depth --markers PATH`. 약 0.16 s/장.

## ONNX

공식 `HuggingFaceTB/SmolVLM2-*/onnx`와 **같은 파일 3개·같은 입출력 이름**으로 내보낸다 → 공식 ONNX를 읽는 런타임(onnxruntime, transformers.js …)에 그대로 넣을 수 있다.

| 파일 | 입력 → 출력 |
|---|---|
| `vision_encoder.onnx` | `pixel_values[b,n,3,512,512]`, `pixel_attention_mask[b,n,512,512]`(bool) → `image_features[N,64,576]` |
| `embed_tokens.onnx` | `input_ids[b,s]` → `inputs_embeds[b,s,576]` |
| `decoder_model_merged.onnx` | `inputs_embeds`, `attention_mask`, `position_ids`, `past_key_values.{0..29}.key/value` → `logits`, `present.{0..29}.key/value` |

생성 루프(`export_onnx.OnnxVlm`, C++로 옮길 때도 같은 순서): 프로세서로 이미지·프롬프트 → `vision_encoder` → `embed_tokens` → `<image>` 토큰 자리를 `image_features`로 바꿈 → `decoder`를 KV 캐시로 한 토큰씩(greedy) → `<end_of_utterance>`에서 멈춤.
`--check`는 같은 이미지·질문에서 torch 와 ONNX 답이 **글자까지 같은지** 본다 (원본 256M 으로 100 토큰 가까운 설명이 같음을 확인, CPU 약 2 s).

## 결과

(학습이 끝나면 채운다)

## 다음

- 엔진 연결: `engine/nodes/vlm` 은 지금 HTTP sidecar(`/query`)에 붙는다 → 이 ONNX 를 쓰는 sidecar 또는 C++ onnxruntime 직접.
- 실물 사진 평가 세트 (학습에 쓰지 않는다), 시뮬·실물 따로 집계.
- 데이터 늘리기·int8 양자화·500M 비교.
