"""vqa.py 정답 규칙 확인:  ml/vlm/.venv/bin/python ml/vlm/test_vqa.py"""
from vqa import select_answer


def obj(category, distance, ratio=1.0, bbox=(100, 100, 200, 200), visible=True, truncated=False):
    return {"category": category, "variant": f"{category}/x", "distance": distance, "visible_ratio": ratio,
            "bbox": list(bbox), "visible": visible, "truncated": truncated}


def label(*objects):
    return {"width": 1000, "height": 1000, "objects": list(objects)}


# 가까운 대상을 고른다, 후보 수를 센다
answer, expect = select_answer(label(obj("can", 1.2, bbox=(0, 0, 10, 10)), obj("can", 0.6), obj("pet", 0.3)), "can")
assert expect == "select" and answer["count"] == 2 and answer["target"]["bbox"] == [100, 100, 200, 200]
# 많이 가려진(0.3 미만) 대상은 안 보이는 것 → 불명확 물품이 있으면 모호
assert select_answer(label(obj("can", 0.5, ratio=0.1), obj("unknown", 0.8)), "can")[1] == "ambiguous"
# 대상도 불명확 물품도 없으면 부재, 대상 None 이면 재활용품 아무거나
assert select_answer(label(obj("pet", 0.5), obj("other", 0.4)), "paper")[1] == "absent"
assert select_answer(label(obj("other", 0.4), obj("paper", 0.9)), None)[0]["target"]["category"] == "paper"
print("test_vqa ok")
