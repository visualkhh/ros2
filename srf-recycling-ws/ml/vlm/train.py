#!/usr/bin/env python3
"""SmolVLM2 LoRA 미세조정 (CPU). vqa.py 가 만든 jsonl 로 학습한다.

  ml/vlm/.venv/bin/python ml/vlm/train.py --data vqa.jsonl --out artifacts/models/vlm/runs/v1

- 비전 인코더·커넥터는 고정, 언어 모델 attention(q/k/v/o)에만 LoRA. 답 토큰에만 손실.
- 에폭마다 <out>/adapter 저장. 끝나면 <out>/merged (LoRA 합친 전체 모델 + 프로세서) → evaluate / export_onnx 입력.
- 보고서 (YOLO 처럼): <out>/metrics.csv 에 쌓고 <out>/report.html 을 계속 다시 쓴다 (브라우저로 열어 두면 된다).
  --val 을 주면 에폭마다 검증 손실 + 판단(select/hold) 정답률도 적는다.
"""
import argparse
import json
import random
import time
from pathlib import Path

import evaluate
import report
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
    parser.add_argument("--val", type=Path, help="검증 jsonl (vqa.py). 에폭마다 손실·판단 정답률")
    parser.add_argument("--val-limit", type=int, default=64, help="검증 손실에 쓸 문항 수")
    parser.add_argument("--val-generate", type=int, default=24, help="답을 생성해 판단을 채점할 select 문항 수")
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
    (args.out / "metrics.csv").unlink(missing_ok=True)
    (args.out / "config.json").write_text(json.dumps(
        {k: str(v) for k, v in vars(args).items()} | {"train_rows": len(rows)}, ensure_ascii=False, indent=2))
    val_rows = [json.loads(line) for line in args.val.open()] if args.val else []

    def validate():
        net.eval()
        with torch.no_grad():
            losses = [net(**smol.encode(processor, smol.open_image(r["image"]), r["question"], r["answer"])
                          .to(args.device)).loss.item() for r in val_rows[: args.val_limit]]
        chosen = [r for r in val_rows if r["task"] == "select"][: args.val_generate]
        verdicts = [evaluate.judge(r, evaluate.parse(smol.generate(processor, net, smol.open_image(r["image"]),
                                                                   r["question"]))) for r in chosen]
        net.train()
        return (sum(losses) / max(1, len(losses)), sum(bool(v.get("decision")) for v in verdicts) / max(1, len(verdicts)),
                sum(v["parsed"] for v in verdicts) / max(1, len(verdicts)))

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
                    left = (args.epochs - epoch - 1) * len(rows) + len(rows) - i
                    print(f"[train] epoch {epoch + 1}/{args.epochs} {i}/{len(rows)} loss {running / i:.4f} "
                          f"({rate:.2f}s/샘플)", flush=True)
                    report.append(args.out, {"time": round(time.time()), "epoch": epoch + 1, "epochs": args.epochs,
                                             "sample": i, "samples": len(rows), "loss": round(running / i, 5),
                                             "lr": scheduler.get_last_lr()[0], "sec_per_sample": round(rate, 3),
                                             "eta_min": round(left * rate / 60)})
                    report.write_report(args.out)
        net.save_pretrained(args.out / "adapter")
        if val_rows:
            val_loss, decision, parsed = validate()
            print(f"[train] epoch {epoch + 1} 검증 손실 {val_loss:.4f} 판단 {decision:.1%} JSON {parsed:.1%}", flush=True)
            report.append(args.out, {"time": round(time.time()), "epoch": epoch + 1, "epochs": args.epochs,
                                     "val_loss": round(val_loss, 5), "val_decision": round(decision, 4),
                                     "val_json": round(parsed, 4)})
            report.write_report(args.out)
    merged = net.merge_and_unload()
    merged.save_pretrained(args.out / "merged")
    processor.save_pretrained(args.out / "merged")
    print(f"[train] 끝 → {args.out / 'merged'}")


if __name__ == "__main__":
    main()
