"""beebot CAD → 우리 로봇 몸체 메시 (tb3_bringup/meshes/body.stl, wheel_*.stl).

우리 로봇 = beebot에서 매니퓰레이터 · 맨 위 판(3층, z≈0.087)과 그 위 전부(바구니·팔 받침) · 뒤 pole(+카메라·브래킷) 제거.
지지봉(standoff, z 0.044~0.091)과 2층 보드(OpenCR·Pi)는 남김. 라이다는 URDF가 따로 붙임 (1층 앞).

  ./download_meshes.sh   # 먼저 (beebot 메시)
  /home/pank/pa-course/IsaacLab-6.1/.venv/bin/python make_tb3_body.py   # trimesh 필요 (Isaac Lab venv에 있음)

base_link.stl은 조각(판 타일·지지봉·나사 …) 10k개짜리 한 덩어리 → 연결된 조각으로 나눠 규칙으로 고른다.
"""
import os
import shutil

import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "../../../ros2_ws/src/tb3_bringup/meshes")

PLATE3_BOTTOM = 0.080   # [m] 3층 판 아래면 0.082 — 바닥이 이보다 위인 조각 = 3층 판과 그 위 (지지봉 바닥은 0.044)
POLE_HEIGHT = 0.20      # 이보다 높이 솟은 조각 = pole 기둥 (1층부터 섬)
PLATE_REAR = -0.199     # 판 뒤끝 -0.197 — 이보다 뒤로 나간 조각 = pole 브래킷
MIN_SIZE = 0.006        # 6 mm 미만 조각(나사·너트) 버림 — 보이지 않고 파일만 큼


def keep(piece) -> bool:
    lo, hi = piece.bounds * 0.001  # 메시는 mm
    return (lo[2] <= PLATE3_BOTTOM and hi[2] <= POLE_HEIGHT and lo[0] >= PLATE_REAR
            and max(hi - lo) >= MIN_SIZE)


def main():
    src = trimesh.load(os.path.join(HERE, "meshes/base_link.stl"))
    pieces = src.split(only_watertight=False)
    kept = [p for p in pieces if keep(p)]
    body = trimesh.util.concatenate(kept)
    lo, hi = body.bounds * 0.001
    assert hi[2] < 0.095 and lo[0] > -0.20, f"pole/top plate left? bounds {lo} {hi}"
    os.makedirs(OUT, exist_ok=True)
    body.export(os.path.join(OUT, "body.stl"))
    body.export(os.path.join(HERE, "meshes/body.stl"))  # beebot.urdf(이 폴더)도 같은 몸통
    for side in ("left", "right"):
        shutil.copy(os.path.join(HERE, f"meshes/wheel_{side}_link.stl"), os.path.join(OUT, f"wheel_{side}.stl"))
    print(f"body.stl: {len(kept)}/{len(pieces)} pieces, {len(body.faces)} faces, "
          f"x {lo[0]:.3f}..{hi[0]:.3f} y {lo[1]:.3f}..{hi[1]:.3f} z {lo[2]:.3f}..{hi[2]:.3f} m -> {os.path.normpath(OUT)}")


if __name__ == "__main__":
    main()
