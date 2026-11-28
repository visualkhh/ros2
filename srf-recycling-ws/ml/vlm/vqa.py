#!/usr/bin/env python3
"""srf_recycling_ws_dataset 의 labels.jsonl → VLM 질문·답(VQA) jsonl.

  ml/vlm/.venv/bin/python ml/vlm/vqa.py DATASET_DIR [DATASET_DIR ...] --out vqa.jsonl

한 줄: {"image": 절대경로, "task": select|detect|where, "question", "answer"(JSON 문자열), "expect"}
- 정답은 장면 설정이 아니라 **그 이미지에 실제로 보이는 것**으로 정한다 (가려짐·화면 밖 반영).
- bbox 는 이미지 크기에 대한 0~1000 정수 [x0, y0, x1, y1] (해상도와 무관하게 같은 토큰).
"""
import argparse
import json
import random
from pathlib import Path

RECYCLABLE = ("can", "pet", "paper")
WORDS = {
    "can": ["can", "aluminum can", "drink can"],
    "pet": ["PET bottle", "plastic bottle"],
    "paper": ["paper", "paper box", "cardboard"],
}
SELECT_TEMPLATES = ["put the {w} in its bin", "sort the {w}", "pick up a {w}", "grab the {w}"]
ALL_TEMPLATES = ["sort all recyclables", "clean up the work mat", "put everything in the right bins"]
BIN_WORDS = {"bin_can": "can bin", "bin_pet": "PET bin", "bin_paper": "paper bin", "work_mat": "work mat"}
MIN_RATIO = 0.3  # 이보다 많이 가려진 물체는 "보인다"고 치지 않는다


def norm_box(box, width, height):
    x0, y0, x1, y1 = box
    return [round(x0 * 1000 / width), round(y0 * 1000 / height), round(x1 * 1000 / width), round(y1 * 1000 / height)]


def visible_objects(label):
    return [o for o in label["objects"] if o["visible"] and o["bbox"] and o["visible_ratio"] >= MIN_RATIO]


def access_cost(o):
    # 미션 규칙 "접근·집기 가능한 것부터": 가깝고, 덜 가려지고, 화면에 다 들어온 것
    return o["distance"] + 0.5 * (1.0 - o["visible_ratio"]) + (0.3 if o["truncated"] else 0.0)


def short(o):
    return o["variant"].split("/", 1)[-1]


def select_answer(label, target):
    """target: can|pet|paper|None(전부). → (answer dict, expect)"""
    w, h = label["width"], label["height"]
    seen = visible_objects(label)
    wanted = [o for o in seen if (o["category"] == target if target else o["category"] in RECYCLABLE)]
    if wanted:
        best = min(wanted, key=access_cost)
        return ({"decision": "select", "reason": "ok", "count": len(wanted),
                 "target": {"category": best["category"], "label": short(best),
                            "bbox": norm_box(best["bbox"], w, h)}}, "select")
    unknown = [o for o in seen if o["category"] == "unknown"]
    if unknown:
        u = min(unknown, key=access_cost)
        return {"decision": "hold", "reason": "ambiguous", "bbox": norm_box(u["bbox"], w, h)}, "ambiguous"
    return {"decision": "hold", "reason": "absent"}, "absent"


def select_question(target, rng):
    instruction = rng.choice(SELECT_TEMPLATES).format(w=rng.choice(WORDS[target])) if target else rng.choice(ALL_TEMPLATES)
    return f"Instruction: {instruction}. Choose one object to grab, or hold. Reply in JSON."


def samples(label, image, rng):
    w, h = label["width"], label["height"]
    # 1) 선택/보류: 장면이 의도한 대상 하나 + 무작위 대상 하나
    scene_target = label["scene"]["target"]
    second = rng.choice((None,) + RECYCLABLE)
    seen = {o["category"] for o in visible_objects(label)}
    missing = [c for c in RECYCLABLE if c not in seen]
    if "unknown" in seen and missing and rng.random() < 0.7:
        second = rng.choice(missing)  # 모호(hold/ambiguous) 사례가 드물어서 일부러 늘린다
    for target in (scene_target, second):
        answer, expect = select_answer(label, target)
        yield {"image": image, "task": "select", "question": select_question(target, rng),
               "answer": json.dumps(answer, separators=(",", ":")), "expect": expect,
               "scene": label["scene"]["type"], "target": target}
    # 2) 검출: 보이는 물체 전부 (왼쪽부터)
    objects = sorted(visible_objects(label), key=lambda o: o["bbox"][0])
    detect = [{"category": o["category"], "label": short(o), "bbox": norm_box(o["bbox"], w, h)} for o in objects]
    yield {"image": image, "task": "detect", "question": "List every object you can see on the work mat in JSON.",
           "answer": json.dumps(detect, separators=(",", ":")), "expect": "detect", "scene": label["scene"]["type"]}
    # 3) 지도 마커 위치: 바구니·매트 하나
    markers = [m for m in label["markers"] if m["name"] in BIN_WORDS]
    if markers:
        m = rng.choice(markers)
        shown = m["in_view"] and m["bbox"] and m["visible_ratio"] >= MIN_RATIO
        answer = {"name": m["name"], "visible": bool(shown)}
        if shown:
            answer["bbox"] = norm_box(m["bbox"], w, h)
        yield {"image": image, "task": "where", "question": f"Where is the {BIN_WORDS[m['name']]}? Reply in JSON.",
               "answer": json.dumps(answer, separators=(",", ":")), "expect": "where", "scene": label["scene"]["type"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("datasets", nargs="+", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    rng = random.Random(args.seed)
    counts = {}
    with args.out.open("w") as out:
        for root in args.datasets:
            for line in (root / "labels.jsonl").open():
                label = json.loads(line)
                image = str((root / label["image"]).resolve())
                for s in samples(label, image, rng):
                    counts[s["expect"]] = counts.get(s["expect"], 0) + 1
                    out.write(json.dumps(s, ensure_ascii=False) + "\n")
    print(f"[vqa] {sum(counts.values())}개 → {args.out}  {counts}")


if __name__ == "__main__":
    main()
