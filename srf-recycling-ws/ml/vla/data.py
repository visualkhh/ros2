"""VLA 시범 데이터 (srf_recycling_ws_vla_demos 출력) 읽기: 상태·행동 벡터, 정규화, 행동 청크.

입력 상태 29개 (순서 고정, 엔진 vla_node 도 같은 순서로 만든다):
  odom v, w | 팔 위치 j1-4·그리퍼 | 팔 속도 ×5 | 팔 전류 ×5 | EE x,y,z,하향각,yaw | 앞 장애물 거리 | 대상 xyz | 바구니 xyz
출력 행동 8개: v, w, j1, j2, j3, j4, 그리퍼, 부저(0/1)
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1] / "vlm"))  # smol.py, report.py 를 같이 쓴다 (뒤에 붙여야 이 폴더 train.py 가 먼저)

STATE_NAMES = (["v", "w"] + [f"q{i}" for i in range(5)] + [f"dq{i}" for i in range(5)] + [f"cur{i}" for i in range(5)]
               + ["ee_x", "ee_y", "ee_z", "ee_pitch", "ee_yaw", "front"] + ["tx", "ty", "tz", "bx", "by", "bz"])
ACTION_NAMES = ["v", "w", "j1", "j2", "j3", "j4", "gripper", "buzzer"]
# 고정 정규화 (데이터 통계가 아니라 물리 범위 → 실기·다른 데이터셋에서도 같은 값)
STATE_SCALE = np.array([0.1, 0.5] + [3.0, 2.0, 1.5, 2.0, 0.02] + [1.0] * 4 + [0.03] + [1.0] * 5
                       + [0.4, 0.4, 0.4, 1.6, 3.0, 3.5] + [1.0] * 6, dtype=np.float32)
ACTION_CENTER = np.array([0, 0, 0, 0, 0, 0, 0.007, 0], dtype=np.float32)
ACTION_SCALE = np.array([0.1, 0.5, 3.0, 2.0, 1.5, 2.0, 0.012, 1.0], dtype=np.float32)
CHUNK = 8  # 4 Hz × 8 = 앞으로 2초


def state_vector(state):
    arm = state["arm"]
    v = ([state["odom"]["v"], state["odom"]["w"]] + arm["position"] + arm["velocity"] + arm["current"] + state["ee"]
         + [min(state["scan_front_min"] or 3.5, 3.5)] + state["target_xyz"] + state["bin_xyz"])
    return np.asarray(v, dtype=np.float32) / STATE_SCALE


def action_vector(action):
    v = [action["v"], action["w"]] + action["joints"] + [action["gripper"], action["buzzer"]]
    return (np.asarray(v, dtype=np.float32) - ACTION_CENTER) / ACTION_SCALE


def physical_action(normalized):
    return np.asarray(normalized) * ACTION_SCALE + ACTION_CENTER


def episodes(root, success_only=True):
    """→ [(episode_dir, meta, steps)]"""
    root = Path(root)
    out = []
    for line in (root / "episodes.jsonl").open():
        meta = json.loads(line)
        if success_only and not meta["success"]:
            continue
        d = root / "episodes" / meta["episode"]
        steps = [json.loads(s) for s in (d / "steps.jsonl").open()]
        out.append((d, meta, steps))
    return out


def chunks(steps):
    """스텝마다 앞으로 CHUNK 개 행동 (끝을 넘으면 마지막 행동 반복, mask=0)."""
    actions = np.stack([action_vector(s["action"]) for s in steps])
    n = len(steps)
    target = np.zeros((n, CHUNK, len(ACTION_NAMES)), np.float32)
    mask = np.zeros((n, CHUNK), np.float32)
    for i in range(n):
        for k in range(CHUNK):
            j = min(i + k, n - 1)
            target[i, k] = actions[j]
            mask[i, k] = 1.0 if i + k < n else 0.0
    return target, mask


def depth_grid(path, size=(32, 24)):
    """16비트 mm 깊이 → 가까운 값 기준으로 줄인 격자 (m, 0~3). 없는 값(0)은 3 m."""
    from PIL import Image
    depth = np.asarray(Image.open(path), dtype=np.float32) / 1000.0
    depth[depth <= 0] = 3.0
    h, w = depth.shape
    gw, gh = size
    depth = depth[: h - h % gh, : w - w % gw].reshape(gh, h // gh, gw, w // gw).min(axis=(1, 3))
    return np.clip(depth, 0.0, 3.0).reshape(-1) / 3.0


if __name__ == "__main__":  # 확인: python data.py DIR
    eps = episodes(sys.argv[1])
    d, meta, steps = eps[0]
    s = state_vector(steps[0]["state"])
    t, m = chunks(steps)
    assert s.shape == (len(STATE_NAMES),) and t.shape[1:] == (CHUNK, len(ACTION_NAMES))
    back = physical_action(action_vector(steps[3]["action"]))
    assert abs(back[2] - steps[3]["action"]["joints"][0]) < 1e-5
    print(f"회차 {len(eps)}, 첫 회차 스텝 {len(steps)}, 상태 {s.shape}, 청크 {t.shape}, 깊이 {depth_grid(d / steps[0]['depth']).shape}")
