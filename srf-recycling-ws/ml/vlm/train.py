#!/usr/bin/env python3
"""SmolVLM2 LoRA 미세조정 (CPU). vqa.py 가 만든 jsonl 로 학습한다.

  ml/vlm/.venv/bin/python ml/vlm/train.py --data vqa.jsonl --out artifacts/models/vlm/runs/v1

- 비전 인코더·커넥터는 고정, 언어 모델 attention(q/k/v/o)에만 LoRA. 답 토큰에만 손실.
- 에폭마다 <out>/adapter 저장. 끝나면 <out>/merged (LoRA 합친 전체 모델 + 프로세서) → evaluate / export_onnx 입력.
"""
import argparse
import json
import random
import time
from pathlib import Path

import smol
import torch
from peft import LoraConfig, get_peft_model


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--model", default=smol.BASE_MODEL)
    parser.add_argument("--epochs", type=int, default=2)
    parser.add_argument("--lr", type=float, default=2e-4)
    parser.add_argument("--accum", type=int, default=8, help="기울기 누적 (배치 1 × accum)")
    parser.add_argument("--rank", type=int, default=16)
    parser.add_argument("--limit", type=int, default=0, help="앞에서 N개만 (빠른 확인용)")
    parser.add_argument("--threads", type=int, default=0)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()

    if args.threads > 0:
        torch.set_num_threads(args.threads)
    torch.manual_seed(args.seed)
    rows = [json.loads(line) for line in args.data.open()]
    if args.limit:
        rows = rows[: args.limit]
    processor, net = smol.load(args.model, args.device)
    net = get_peft_model(net, LoraConfig(
        r=args.rank, lora_alpha=args.rank * 2, lora_dropout=0.05,
        target_modules=r".*text_model.*\.(q_proj|k_proj|v_proj|o_proj)"))
    net.print_trainable_parameters()
    optimizer = torch.optim.AdamW([p for p in net.parameters() if p.requires_grad], lr=args.lr)
    total = args.epochs * len(rows) // args.accum
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lambda s: max(0.0, 1.0 - s / max(1, total)))
    args.out.mkdir(parents=True, exist_ok=True)

    rng = random.Random(args.seed)
    step = 0
    for epoch in range(args.epochs):
        net.train()
        rng.shuffle(rows)
        running, started = 0.0, time.time()
        for i, row in enumerate(rows, 1):
            inputs = smol.encode(processor, smol.open_image(row["image"]), row["question"], row["answer"]).to(args.device)
            loss = net(**inputs).loss / args.accum
            loss.backward()
            running += loss.item() * args.accum
            if i % args.accum == 0 or i == len(rows):
                torch.nn.utils.clip_grad_norm_(net.parameters(), 1.0)
                optimizer.step()
                scheduler.step()
                optimizer.zero_grad()
                step += 1
                if step % 10 == 0 or i == len(rows):
                    rate = (time.time() - started) / i
                    print(f"[train] epoch {epoch + 1}/{args.epochs} {i}/{len(rows)} loss {running / i:.4f} "
                          f"({rate:.2f}s/샘플)", flush=True)
        net.save_pretrained(args.out / "adapter")
    merged = net.merge_and_unload()
    merged.save_pretrained(args.out / "merged")
    processor.save_pretrained(args.out / "merged")
    print(f"[train] 끝 → {args.out / 'merged'}")


if __name__ == "__main__":
    main()
