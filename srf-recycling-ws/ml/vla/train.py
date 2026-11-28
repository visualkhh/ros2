#!/usr/bin/env python3
"""VLA 행동 헤드 학습 (CPU). features.py 로 미리 계산한 VLM·깊이 특징 + 로봇 상태 → 앞으로 2초 행동 청크.

  ml/vlm/.venv/bin/python ml/vla/train.py DEMOS --out artifacts/models/vla/runs/v1

- 출력: 스텝마다 8개 (v, w, j1-4, 그리퍼, 부저) × CHUNK. 연속값은 smooth L1, 부저는 BCE.
- 회차 단위로 검증 10% 를 뗀다 (같은 회차의 비슷한 프레임이 양쪽에 섞이지 않게).
- 보고서: <out>/metrics.csv, report.html (ml/vlm 과 같은 형식), 가장 좋은 검증 손실의 head.pt.
"""
import argparse
import json
import time
import zlib
from pathlib import Path

import data
import numpy as np
import report
import torch
from torch import nn

N_ACT = len(data.ACTION_NAMES)


class Head(nn.Module):
    # ponytail: 깊이는 32×24 격자를 MLP 로. 대상 주변 정밀도가 모자라면 작은 CNN·대상 중심 크롭으로.
    def __init__(self, vlm_dim=576, state_dim=len(data.STATE_NAMES), depth_dim=768, hidden=384):
        super().__init__()
        self.depth = nn.Sequential(nn.Linear(depth_dim, 128), nn.SiLU())
        width = vlm_dim + state_dim + 128
        self.net = nn.Sequential(nn.LayerNorm(width), nn.Linear(width, hidden), nn.SiLU(), nn.Dropout(0.1),
                                 nn.Linear(hidden, hidden), nn.SiLU(), nn.Linear(hidden, data.CHUNK * N_ACT))

    def forward(self, vlm, state, depth):
        x = torch.cat([vlm, state, self.depth(depth)], dim=-1)
        return self.net(x).reshape(-1, data.CHUNK, N_ACT)  # 정규화된 값 (부저는 logit)


def load_arrays(demos):
    rows = {"vlm": [], "state": [], "depth": [], "target": [], "mask": [], "episode": []}
    for d, meta, steps in data.episodes(demos):
        f = d / "features.npz"
        if not f.exists():
            continue
        z = np.load(f)
        target, mask = data.chunks(steps)
        rows["vlm"].append(z["vlm"])
        rows["depth"].append(z["depth"])
        rows["state"].append(np.stack([data.state_vector(s["state"]) for s in steps]))
        rows["target"].append(target)
        rows["mask"].append(mask)
        rows["episode"] += [meta["episode"]] * len(steps)
    if not rows["vlm"]:
        raise SystemExit("features.npz 가 없다 — features.py 를 먼저 돌린다")
    arrays = {k: torch.from_numpy(np.concatenate(v)) for k, v in rows.items() if k != "episode"}
    return arrays, rows["episode"]


def loss_fn(pred, target, mask):
    cont = nn.functional.smooth_l1_loss(pred[..., :7], target[..., :7], reduction="none").mean(-1)
    buzz = nn.functional.binary_cross_entropy_with_logits(pred[..., 7], target[..., 7], reduction="none")
    return ((cont + 0.2 * buzz) * mask).sum() / mask.sum().clamp(min=1)


def split(episodes, val_ratio):
    """회차 이름 해시로 결정적 분할."""
    val = np.array([(zlib.crc32(e.encode()) % 1000) < val_ratio * 1000 for e in episodes])
    return torch.from_numpy(~val), torch.from_numpy(val)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demos")
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--batch", type=int, default=256)
    parser.add_argument("--lr", type=float, default=1e-3)
    parser.add_argument("--val-ratio", type=float, default=0.1)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    torch.manual_seed(args.seed)
    arrays, episodes = load_arrays(args.demos)
    train_mask, val_mask = split(episodes, args.val_ratio)
    if not val_mask.any():
        val_mask = train_mask.clone()  # 회차가 아주 적을 때 (확인용)
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "metrics.csv").unlink(missing_ok=True)
    meta = {k: str(v) for k, v in vars(args).items()} | {
        "steps": int(len(episodes)), "train_steps": int(train_mask.sum()), "val_steps": int(val_mask.sum()),
        "state_names": data.STATE_NAMES, "action_names": data.ACTION_NAMES, "chunk": data.CHUNK}
    (args.out / "config.json").write_text(json.dumps(meta, ensure_ascii=False, indent=2))

    head = Head(vlm_dim=arrays["vlm"].shape[1], depth_dim=arrays["depth"].shape[1])
    optimizer = torch.optim.AdamW(head.parameters(), lr=args.lr, weight_decay=1e-4)
    scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, args.epochs)
    tr = {k: v[train_mask] for k, v in arrays.items()}
    va = {k: v[val_mask] for k, v in arrays.items()}
    best, started = float("inf"), time.time()
    for epoch in range(1, args.epochs + 1):
        head.train()
        order = torch.randperm(len(tr["vlm"]))
        total = 0.0
        for i in range(0, len(order), args.batch):
            idx = order[i: i + args.batch]
            loss = loss_fn(head(tr["vlm"][idx], tr["state"][idx], tr["depth"][idx]), tr["target"][idx], tr["mask"][idx])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
            total += loss.item() * len(idx)
        scheduler.step()
        head.eval()
        with torch.no_grad():
            pred = head(va["vlm"], va["state"], va["depth"])
            val_loss = loss_fn(pred, va["target"], va["mask"]).item()
            # 그리퍼 열림/닫힘이 맞는 비율 (첫 스텝)
            grip_ok = ((pred[:, 0, 6] > 0) == (va["target"][:, 0, 6] > 0)).float().mean().item()
        rate = (time.time() - started) / epoch
        print(f"[vla] epoch {epoch}/{args.epochs} loss {total / len(order):.4f} val {val_loss:.4f} 그리퍼 {grip_ok:.1%}",
              flush=True)
        report.append(args.out, {"time": round(time.time()), "epoch": epoch, "epochs": args.epochs, "sample": 1,
                                 "samples": 1, "loss": round(total / len(order), 5), "lr": scheduler.get_last_lr()[0],
                                 "sec_per_sample": round(rate, 3), "eta_min": round((args.epochs - epoch) * rate / 60, 1)})
        report.append(args.out, {"time": round(time.time()), "epoch": epoch, "epochs": args.epochs,
                                 "val_loss": round(val_loss, 5), "val_decision": round(grip_ok, 4)})
        report.write_report(args.out, title=f"VLA 학습 · {args.out.name}",
                            labels={"decision": "검증 그리퍼 열림/닫힘 정답률", "speed": "속도 (초/에폭)"})
        if val_loss < best:
            best = val_loss
            torch.save({"state_dict": head.state_dict(), "vlm_dim": arrays["vlm"].shape[1],
                        "depth_dim": arrays["depth"].shape[1], "config": meta}, args.out / "head.pt")
    print(f"[vla] 끝 → {args.out / 'head.pt'} (최저 검증 손실 {best:.4f})")


if __name__ == "__main__":
    main()
