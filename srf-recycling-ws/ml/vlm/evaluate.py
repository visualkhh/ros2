#!/usr/bin/env python3
"""VLM 평가 (발제 지표: 유형별 최초 판단 정답률) + 위치(bbox) 정확도. torch 또는 ONNX 로 돌린다.

  ml/vlm/.venv/bin/python ml/vlm/evaluate.py --data val.jsonl --model RUN/merged --records eval.jsonl
  ml/vlm/.venv/bin/python ml/vlm/evaluate.py --data val.jsonl --onnx artifacts/models/vlm/onnx/v1

- select: 정상(select)·모호(hold/ambiguous)·부재(hold/absent) 판단이 맞은 회차 / 그 유형 회차.
  select 가 맞으면 종류 일치·bbox IoU≥0.5 도 센다.
- detect: 종류 같고 IoU≥0.5 인 짝으로 정밀도·재현율.  where: 보임/안 보임 + IoU≥0.5.
- --records: 회차마다 이미지·질문·정답·원출력·판정 (발제의 기록 요구).
"""
import argparse
import json
import time
from pathlib import Path

import smol


def iou(a, b):
    ix = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def box_ok(pred, truth):
    return isinstance(pred, list) and len(pred) == 4 and iou(pred, truth) >= 0.5


def judge(row, pred):
    """→ dict(판정 항목: bool 또는 숫자). pred 는 파싱한 JSON (실패면 None)."""
    truth = json.loads(row["answer"])
    if pred is None:
        return {"parsed": False}
    task = row["task"]
    if task == "select":
        if not isinstance(pred, dict):
            return {"parsed": True, "decision": False}
        decided = (pred.get("decision"), pred.get("reason")) == (truth["decision"], truth["reason"])
        result = {"parsed": True, "decision": decided}
        if decided and truth["decision"] == "select":
            target = pred.get("target") or {}
            result["category"] = target.get("category") == truth["target"]["category"]
            result["box"] = box_ok(target.get("bbox"), truth["target"]["bbox"])
        return result
    if task == "detect":
        pred = [p for p in pred if isinstance(p, dict)] if isinstance(pred, list) else []
        used, hit = set(), 0
        for t in truth:
            for k, p in enumerate(pred):
                if k not in used and p.get("category") == t["category"] and box_ok(p.get("bbox"), t["bbox"]):
                    used.add(k)
                    hit += 1
                    break
        return {"parsed": True, "hit": hit, "pred": len(pred), "truth": len(truth)}
    if task == "where":
        if not isinstance(pred, dict):
            return {"parsed": True, "visible": False}
        result = {"parsed": True, "visible": pred.get("visible") == truth["visible"]}
        if truth["visible"] and result["visible"]:
            result["box"] = box_ok(pred.get("bbox"), truth["bbox"])
        return result
    raise ValueError(task)


def parse(text):
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        return None


def rate(n, d):
    return f"{n}/{d} = {n / d:.1%}" if d else "-"


def summarize(results):
    lines, summary = [], {}
    parsed = sum(j["parsed"] for _, j in results)
    lines.append(f"JSON 파싱       {rate(parsed, len(results))}")
    for expect in ("select", "ambiguous", "absent"):
        group = [j for r, j in results if r["task"] == "select" and r["expect"] == expect]
        ok = sum(bool(j.get("decision")) for j in group)
        summary[f"decision_{expect}"] = ok / len(group) if group else None
        lines.append(f"판단 {expect:<10} {rate(ok, len(group))}")
    chosen = [j for r, j in results if r["task"] == "select" and "category" in j]
    lines.append(f"  선택 종류 일치 {rate(sum(j['category'] for j in chosen), len(chosen))}")
    lines.append(f"  선택 IoU≥0.5  {rate(sum(j['box'] for j in chosen), len(chosen))}")
    det = [j for r, j in results if r["task"] == "detect" and j["parsed"]]
    hit, npred, ntruth = (sum(j.get(k, 0) for j in det) for k in ("hit", "pred", "truth"))
    summary["detect_precision"] = hit / npred if npred else None
    summary["detect_recall"] = hit / ntruth if ntruth else None
    lines.append(f"검출 정밀도     {rate(hit, npred)}")
    lines.append(f"검출 재현율     {rate(hit, ntruth)}")
    where = [j for r, j in results if r["task"] == "where"]
    lines.append(f"위치 보임 판단  {rate(sum(bool(j.get('visible')) for j in where), len(where))}")
    boxed = [j for j in where if "box" in j]
    lines.append(f"  위치 IoU≥0.5  {rate(sum(j['box'] for j in boxed), len(boxed))}")
    return "\n".join(lines), summary


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--model", help="torch 모델 (RUN/merged 또는 HF 이름)")
    parser.add_argument("--onnx", help="export_onnx.py 결과 폴더")
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--records", type=Path)
    args = parser.parse_args()
    if bool(args.model) == bool(args.onnx):
        parser.error("--model 또는 --onnx 하나")
    if args.onnx:
        from export_onnx import OnnxVlm
        vlm = OnnxVlm(args.onnx)
        answer = vlm.generate
    else:
        processor, net = smol.load(args.model)
        answer = lambda image, question: smol.generate(processor, net, image, question)  # noqa: E731
    rows = [json.loads(line) for line in args.data.open()]
    if args.limit:
        rows = rows[: args.limit]
    results, started = [], time.time()
    records = args.records.open("w") if args.records else None
    for i, row in enumerate(rows, 1):
        output = answer(smol.open_image(row["image"]), row["question"])
        verdict = judge(row, parse(output))
        results.append((row, verdict))
        if records:
            records.write(json.dumps({"index": i, "image": row["image"], "task": row["task"], "expect": row["expect"],
                                      "question": row["question"], "truth": row["answer"], "output": output,
                                      "verdict": verdict}, ensure_ascii=False) + "\n")
        if i % 20 == 0:
            print(f"[eval] {i}/{len(rows)} ({(time.time() - started) / i:.2f}s/회)", flush=True)
    text, summary = summarize(results)
    print(text)
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
