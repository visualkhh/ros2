#!/usr/bin/env python3
"""VLA 헤드 오프라인 평가: 검증 회차에서 조교 행동과 얼마나 같은가 (물리 단위).

  ml/vlm/.venv/bin/python ml/vla/evaluate.py DEMOS --head artifacts/models/vla/runs/v1/head.pt

- 연속값: 첫 스텝 평균 절대 오차 (v m/s, w rad/s, 관절 rad, 그리퍼 mm)
- 그리퍼 열림/닫힘, 부저 켬/끔 일치율, 단계별(접근·집기 …) 관절 오차
닫힌 루프 채점(정책이 직접 몰아서 집기·놓기 성공률)은 시뮬레이터에 정책을 넣어서 한다 (다음 단계).
"""
import argparse
import collections

import data
import numpy as np
import torch
import train


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("demos")
    parser.add_argument("--head", required=True)
    parser.add_argument("--all", action="store_true", help="검증 회차만이 아니라 전부")
    args = parser.parse_args()
    saved = torch.load(args.head, map_location="cpu", weights_only=False)
    head = train.Head(vlm_dim=saved["vlm_dim"], depth_dim=saved["depth_dim"])
    head.load_state_dict(saved["state_dict"])
    head.eval()
    val_ratio = float(saved["config"].get("val_ratio", 0.1))
    errors, grip, buzz, by_phase = [], [], [], collections.defaultdict(list)
    for d, meta, steps in data.episodes(args.demos):
        f = d / "features.npz"
        if not f.exists():
            continue
        if not args.all and not train.split([meta["episode"]], val_ratio)[1][0]:
            continue
        z = np.load(f)
        state = torch.from_numpy(np.stack([data.state_vector(s["state"]) for s in steps]))
        with torch.no_grad():
            pred = head(torch.from_numpy(z["vlm"]), state, torch.from_numpy(z["depth"]))[:, 0].numpy()
        for s, p in zip(steps, pred):
            truth = data.physical_action(data.action_vector(s["action"]))
            got = data.physical_action(p)
            err = np.abs(got[:7] - truth[:7])
            errors.append(err)
            by_phase[s["phase"]].append(err[2:6].mean())
            grip.append((p[6] > 0) == (data.action_vector(s["action"])[6] > 0))
            buzz.append((p[7] > 0) == (truth[7] > 0.5))
    if not errors:
        raise SystemExit("평가할 회차가 없다 (features.npz?)")
    e = np.mean(errors, axis=0)
    print(f"스텝 {len(errors)}")
    print(f"v {e[0]:.4f} m/s · w {e[1]:.4f} rad/s · 관절 {e[2:6].mean():.4f} rad (j1-4: "
          + ", ".join(f"{x:.3f}" for x in e[2:6]) + f") · 그리퍼 {e[6] * 1000:.2f} mm")
    print(f"그리퍼 열림/닫힘 일치 {np.mean(grip):.1%} · 부저 일치 {np.mean(buzz):.1%}")
    for phase, values in sorted(by_phase.items()):
        print(f"  {phase:<9} 관절 오차 {np.mean(values):.4f} rad ({len(values)} 스텝)")


if __name__ == "__main__":
    main()
