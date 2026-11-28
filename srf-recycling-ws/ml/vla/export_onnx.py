#!/usr/bin/env python3
"""VLA 행동 헤드 → ONNX (물리 단위 출력). VLM 부분은 ml/vlm/export_onnx.py 의 ONNX 를 쓴다.

  ml/vlm/.venv/bin/python ml/vla/export_onnx.py --head RUN/head.pt --out artifacts/models/vla/onnx/v1

  vla_head.onnx: vlm[b,576], state[b,29], depth[b,768] → actions[b,8,8]
    actions[..., :7] = v(m/s), w(rad/s), j1-4(rad), 그리퍼(m), actions[..., 7] = 부저 확률(0~1)
  vla_head.json: 상태·행동 순서, 상태 정규화(state 는 STATE_SCALE 로 나눈 값을 넣는다), 청크 길이
"""
import argparse
import json
from pathlib import Path

import data
import numpy as np
import onnxruntime as ort
import torch
import train


class Physical(torch.nn.Module):
    def __init__(self, head):
        super().__init__()
        self.head = head
        self.register_buffer("scale", torch.from_numpy(data.ACTION_SCALE))
        self.register_buffer("center", torch.from_numpy(data.ACTION_CENTER))

    def forward(self, vlm, state, depth):
        raw = self.head(vlm, state, depth)
        physical = raw * self.scale + self.center
        return torch.cat([physical[..., :7], torch.sigmoid(raw[..., 7:])], dim=-1)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--head", required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    saved = torch.load(args.head, map_location="cpu", weights_only=False)
    head = train.Head(vlm_dim=saved["vlm_dim"], depth_dim=saved["depth_dim"])
    head.load_state_dict(saved["state_dict"])
    model = Physical(head).eval()
    args.out.mkdir(parents=True, exist_ok=True)
    inputs = (torch.zeros(1, saved["vlm_dim"]), torch.zeros(1, len(data.STATE_NAMES)), torch.zeros(1, saved["depth_dim"]))
    path = args.out / "vla_head.onnx"
    torch.onnx.export(model, inputs, path, input_names=["vlm", "state", "depth"], output_names=["actions"],
                      opset_version=17, dynamo=False,
                      dynamic_axes={"vlm": {0: "b"}, "state": {0: "b"}, "depth": {0: "b"}, "actions": {0: "b"}})
    (args.out / "vla_head.json").write_text(json.dumps({
        "state_names": data.STATE_NAMES, "state_scale": data.STATE_SCALE.tolist(),
        "action_names": data.ACTION_NAMES, "chunk": data.CHUNK, "period_s": 0.25}, ensure_ascii=False, indent=2))
    # torch 와 같은지
    x = [torch.randn_like(t) for t in inputs]
    with torch.no_grad():
        expected = model(*x).numpy()
    got = ort.InferenceSession(str(path), providers=["CPUExecutionProvider"]).run(
        None, {"vlm": x[0].numpy(), "state": x[1].numpy(), "depth": x[2].numpy()})[0]
    print(f"[export] → {path}  최대 차이 {np.abs(expected - got).max():.2e}")


if __name__ == "__main__":
    main()
