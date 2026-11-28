"""SmolVLM2 공통: 모델·프로세서 불러오기, 프롬프트, 답 생성 (train / evaluate / export_onnx 가 같이 쓴다)."""
import os
from pathlib import Path

WS = Path(__file__).resolve().parents[2]  # srf-recycling-ws
os.environ.setdefault("HF_HOME", str(WS / "artifacts/models/vlm/hf-cache"))

import torch  # noqa: E402
from PIL import Image  # noqa: E402
from transformers import AutoModelForImageTextToText, AutoProcessor  # noqa: E402

BASE_MODEL = "HuggingFaceTB/SmolVLM2-256M-Video-Instruct"


def load(model=BASE_MODEL, device="cpu"):
    processor = AutoProcessor.from_pretrained(model)
    # vlm_server.py 와 같다: 512 타일 하나 (이미지 토큰 64개). 쪼개기는 CPU 시간만 n배로 늘린다.
    processor.image_processor.do_image_splitting = False
    net = AutoModelForImageTextToText.from_pretrained(model, dtype=torch.float32).to(device)
    return processor, net


def conversation(question, answer=None):
    turns = [{"role": "user", "content": [{"type": "image"}, {"type": "text", "text": question}]}]
    if answer is not None:
        turns.append({"role": "assistant", "content": [{"type": "text", "text": answer}]})
    return turns


def encode(processor, image, question, answer=None):
    """answer 가 있으면 학습용: labels 는 답 토큰만 (질문·이미지 토큰은 -100)."""
    prompt = processor.apply_chat_template(conversation(question), add_generation_prompt=True, tokenize=False)
    if answer is None:
        return processor(text=prompt, images=[image], return_tensors="pt")
    full = processor.apply_chat_template(conversation(question, answer), tokenize=False)
    inputs = processor(text=full, images=[image], return_tensors="pt")
    prompt_len = processor(text=prompt, images=[image], return_tensors="pt")["input_ids"].shape[1]
    labels = inputs["input_ids"].clone()
    labels[:, :prompt_len] = -100
    inputs["labels"] = labels
    return inputs


def open_image(path):
    return Image.open(path).convert("RGB")


@torch.inference_mode()
def generate(processor, net, image, question, max_new_tokens=128):
    inputs = encode(processor, image, question).to(net.device)
    out = net.generate(**inputs, max_new_tokens=max_new_tokens, do_sample=False)
    return processor.batch_decode(out[:, inputs["input_ids"].shape[1]:], skip_special_tokens=True)[0].strip()
