#!/usr/bin/env python3
"""얼린 VLM 으로 스텝마다 "장면 이해 벡터"를 미리 계산해 둔다 (학습은 작은 헤드만 → CPU 로 빠르다).

  ml/vlm/.venv/bin/python ml/vla/features.py DEMOS --vlm artifacts/models/vlm/runs/v1/merged

회차 폴더마다 features.npz: vlm[N,576] (RGB + 지시 → 마지막 토큰 hidden), depth[N,768] (32×24 깊이 격자).
VLM 은 ml/vlm 에서 선별장 물체로 미세조정한 것 (물체를 이미 안다). 이미 계산한 회차는 건너뛴다.
"""
import argparse
import time

import data
import numpy as np
import smol
import torch


@torch.inference_mode()
def vlm_feature(processor, net, image, instruction):
    inputs = smol.encode(processor, image, instruction)
    out = net.model(**inputs, use_cache=False)
    return out.last_hidden_state[0, -1].float().numpy()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demos")
    parser.add_argument("--vlm", default=smol.BASE_MODEL)
    parser.add_argument("--limit", type=int, default=0, help="회차 N 개만")
    args = parser.parse_args()
    processor, net = smol.load(args.vlm)
    net.eval()
    eps = data.episodes(args.demos)
    if args.limit:
        eps = eps[: args.limit]
    started, done = time.time(), 0
    for n, (d, meta, steps) in enumerate(eps, 1):
        out = d / "features.npz"
        if out.exists():
            continue
        vlm = np.stack([vlm_feature(processor, net, smol.open_image(d / s["rgb"]), meta["instruction"]) for s in steps])
        depth = np.stack([data.depth_grid(d / s["depth"]) for s in steps]).astype(np.float32)
        np.savez_compressed(out, vlm=vlm.astype(np.float32), depth=depth)
        done += len(steps)
        print(f"[features] {n}/{len(eps)} {meta['episode']} ({(time.time() - started) / done:.2f}s/스텝)", flush=True)


if __name__ == "__main__":
    main()
