#!/usr/bin/env python3
"""미세조정한 SmolVLM2 → ONNX 3개 (공식 HuggingFaceTB/SmolVLM2-*/onnx 와 같은 파일·입출력 이름).

  ml/vlm/.venv/bin/python ml/vlm/export_onnx.py --model RUN/merged --out artifacts/models/vlm/onnx/v1

  vision_encoder.onnx        pixel_values[b,n,3,512,512], pixel_attention_mask[b,n,512,512] → image_features[N,64,576]
  embed_tokens.onnx          input_ids[b,s] → inputs_embeds[b,s,576]
  decoder_model_merged.onnx  inputs_embeds, attention_mask, position_ids, past_key_values.{i}.key/value
                             → logits, present.{i}.key/value
  + 프로세서(토크나이저·전처리 설정)를 같은 폴더에. 생성 루프는 OnnxVlm (C++ 쪽도 같은 순서로 하면 된다):
  이미지→vision_encoder, 프롬프트→embed_tokens, <image> 자리를 image_features로 바꿈, decoder 를 KV 캐시로 반복.

내보낸 뒤 같은 이미지·질문으로 torch 와 ONNX 답을 비교한다 (--check IMAGE).
"""
import argparse
from pathlib import Path

import numpy as np
import onnxruntime as ort
import smol
import torch
from transformers import AutoConfig, AutoProcessor, DynamicCache


class VisionEncoder(torch.nn.Module):
    def __init__(self, net):
        super().__init__()
        self.model = net.model

    def forward(self, pixel_values, pixel_attention_mask):
        # get_image_features 와 같은 계산에서 "빈(0) 패딩 이미지 빼기"만 뺐다 (추적 불가 연산, 실제 이미지만 넣는다)
        b, n = pixel_values.shape[:2]
        pixel_values = pixel_values.reshape(b * n, *pixel_values.shape[2:])
        mask = pixel_attention_mask.reshape(b * n, *pixel_attention_mask.shape[2:])
        patch = self.model.config.vision_config.patch_size
        h, w = mask.shape[1:]  # unfold 대신 reshape (같은 값, ONNX 로 내보내진다)
        patch_mask = mask.reshape(-1, h // patch, patch, w // patch, patch).sum(dim=(2, 4)) > 0
        hidden = self.model.vision_model(pixel_values=pixel_values, patch_attention_mask=patch_mask).last_hidden_state
        return self.model.connector(hidden)


class EmbedTokens(torch.nn.Module):
    def __init__(self, net):
        super().__init__()
        self.embed = net.model.text_model.embed_tokens

    def forward(self, input_ids):
        return self.embed(input_ids)


class Decoder(torch.nn.Module):
    def __init__(self, net):
        super().__init__()
        self.text = net.model.text_model
        self.lm_head = net.lm_head
        self.layers = net.config.text_config.num_hidden_layers

    def forward(self, inputs_embeds, attention_mask, position_ids, *past):
        cache = DynamicCache(config=self.text.config)
        for i in range(self.layers):
            cache.update(past[2 * i], past[2 * i + 1], i)
        out = self.text(inputs_embeds=inputs_embeds, attention_mask=attention_mask, position_ids=position_ids,
                        past_key_values=cache, use_cache=True)
        present = []
        for layer in out.past_key_values.layers:
            present += [layer.keys, layer.values]
        return (self.lm_head(out.last_hidden_state), *present)


def kv_shape(config):
    t = config.text_config
    return t.num_key_value_heads, t.hidden_size // t.num_attention_heads


def export(model_dir, out):
    out.mkdir(parents=True, exist_ok=True)
    processor, net = smol.load(str(model_dir))
    net.config.text_config._attn_implementation = "eager"  # sdpa 는 추적이 불안정
    net.eval()
    image = smol.Image.new("RGB", (640, 480), (128, 128, 128))
    inputs = smol.encode(processor, image, "What is this?")
    with torch.no_grad():
        torch.onnx.export(VisionEncoder(net), (inputs["pixel_values"], inputs["pixel_attention_mask"].bool()),  # 공식처럼 bool
                          out / "vision_encoder.onnx", input_names=["pixel_values", "pixel_attention_mask"],
                          output_names=["image_features"], opset_version=18, dynamo=True, external_data=False,
                          dynamic_axes={"pixel_values": {0: "batch_size", 1: "num_images"},
                                        "pixel_attention_mask": {0: "batch_size", 1: "num_images"},
                                        "image_features": {0: "num_crops"}})  # 비전은 레거시 추적이 타입을 틀린다
        ids = inputs["input_ids"][:, :8]
        torch.onnx.export(EmbedTokens(net), (ids,), out / "embed_tokens.onnx", input_names=["input_ids"],
                          output_names=["inputs_embeds"], opset_version=17, dynamo=False,
                          dynamic_axes={"input_ids": {0: "batch_size", 1: "sequence_length"},
                                        "inputs_embeds": {0: "batch_size", 1: "sequence_length"}})
        decoder = Decoder(net)
        heads, dim = kv_shape(net.config)
        past_len, seq = 5, 3  # 추적용 아무 크기 (실행 때는 past 0 도 된다)
        past = [torch.zeros(1, heads, past_len, dim) for _ in range(2 * decoder.layers)]
        embeds = torch.zeros(1, seq, net.config.text_config.hidden_size)
        mask = torch.ones(1, past_len + seq, dtype=torch.long)
        pos = torch.arange(past_len, past_len + seq).unsqueeze(0)
        past_names = [f"past_key_values.{i}.{kv}" for i in range(decoder.layers) for kv in ("key", "value")]
        present_names = [f"present.{i}.{kv}" for i in range(decoder.layers) for kv in ("key", "value")]
        axes = {"inputs_embeds": {0: "batch_size", 1: "sequence_length"},
                "attention_mask": {0: "batch_size", 1: "total_sequence_length"},
                "position_ids": {0: "batch_size", 1: "sequence_length"},
                "logits": {0: "batch_size", 1: "sequence_length"}}
        axes.update({n: {0: "batch_size", 2: "past_sequence_length"} for n in past_names})
        axes.update({n: {0: "batch_size", 2: "total_sequence_length"} for n in present_names})
        torch.onnx.export(decoder, (embeds, mask, pos, *past), out / "decoder_model_merged.onnx",
                          input_names=["inputs_embeds", "attention_mask", "position_ids", *past_names],
                          output_names=["logits", *present_names], opset_version=17, dynamo=False,
                          dynamic_axes=axes)
    processor.save_pretrained(out)
    net.config.save_pretrained(out)
    print(f"[export] → {out}")


class OnnxVlm:
    """ONNX 3개로 답을 만든다 (greedy). evaluate.py --onnx 도 이걸 쓴다."""

    def __init__(self, onnx_dir, threads=0):
        onnx_dir = Path(onnx_dir)
        options = ort.SessionOptions()
        if threads > 0:
            options.intra_op_num_threads = threads
        session = lambda name: ort.InferenceSession(str(onnx_dir / name), options, providers=["CPUExecutionProvider"])  # noqa: E731
        self.vision = session("vision_encoder.onnx")
        self.embed = session("embed_tokens.onnx")
        self.decoder = session("decoder_model_merged.onnx")
        self.processor = AutoProcessor.from_pretrained(onnx_dir)
        self.processor.image_processor.do_image_splitting = False
        config = AutoConfig.from_pretrained(onnx_dir)
        self.image_token_id = config.image_token_id
        self.layers = config.text_config.num_hidden_layers
        self.heads, self.dim = kv_shape(config)
        self.eos = {self.processor.tokenizer.convert_tokens_to_ids("<end_of_utterance>"),
                    self.processor.tokenizer.eos_token_id}

    def generate(self, image, question, max_new_tokens=128):
        inputs = smol.encode(self.processor, image, question)
        ids = inputs["input_ids"].numpy()
        features = self.vision.run(None, {"pixel_values": inputs["pixel_values"].numpy(),
                                          "pixel_attention_mask": inputs["pixel_attention_mask"].numpy().astype(bool)})[0]
        embeds = self.embed.run(None, {"input_ids": ids})[0]
        embeds[ids == self.image_token_id] = features.reshape(-1, features.shape[-1])
        past = {f"past_key_values.{i}.{kv}": np.zeros((1, self.heads, 0, self.dim), np.float32)
                for i in range(self.layers) for kv in ("key", "value")}
        total = ids.shape[1]
        position = np.arange(total, dtype=np.int64)[None]
        tokens = []
        for _ in range(max_new_tokens):
            outputs = self.decoder.run(None, {"inputs_embeds": embeds, "attention_mask": np.ones((1, total), np.int64),
                                              "position_ids": position, **past})
            token = int(outputs[0][0, -1].argmax())
            if token in self.eos:
                break
            tokens.append(token)
            past = {name: outputs[1 + k] for k, name in enumerate(past)}
            embeds = self.embed.run(None, {"input_ids": np.array([[token]], np.int64)})[0]
            position = np.array([[total]], np.int64)
            total += 1
        return self.processor.tokenizer.decode(tokens, skip_special_tokens=True).strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True, help="train.py 의 RUN/merged (또는 HF 모델 이름)")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--check", help="이 이미지로 torch·ONNX 답 비교")
    parser.add_argument("--question", default="List every object you can see on the work mat in JSON.")
    args = parser.parse_args()
    export(args.model, args.out)
    if args.check:
        image = smol.open_image(args.check)
        processor, net = smol.load(args.model)
        expected = smol.generate(processor, net, image, args.question)
        got = OnnxVlm(args.out).generate(image, args.question)
        print(f"[check] torch: {expected}\n[check] onnx : {got}\n[check] {'같음' if expected == got else '다름'}")
        if expected != got:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
