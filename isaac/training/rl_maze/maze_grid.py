"""미로 점유 격자 · 최단경로 — 라이다(광선)와 랜덤 웨이포인트가 같은 격자를 쓴다. numpy + 표준 라이브러리 (격자 만들기만 pxr).

  격자 만들기 (미로 USD가 바뀌면 한 번, Isaac venv — pxr 필요):
    $ISAACLAB_DIR/isaaclab.sh -p training/rl_maze/maze_grid.py       → usda/maze/grid.npz · paths.npz (+ grid.png 확인용)
  학습 env는 grid.npz(라이다 · 충돌) · paths.npz(랜덤 경로 N_PATHS개)만 읽는다 — 경로 하나에 A* ~0.1 s라 미리 뽑아 둔다.

격자 = 미로 충돌체(벽 메시 · 기둥 원기둥)를 높이 SLICE_Z(라이다 높이)에서 자른 단면, RES 간격. 좌표 = 미로 prim 기준(env에서 미로를 원점에 둔다).
경로 = 벽에서 ROBOT_R 이상 떨어진 칸만 + 벽에 가까울수록 비싸게 (통로 가운데로) A* → WAYPOINT_STEP 간격 점.
"""
from __future__ import annotations

import heapq
import math
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAZE_USD = os.path.join(HERE, "usda", "maze", "maze.usda")
GRID_NPZ = os.path.join(HERE, "usda", "maze", "grid.npz")
PATHS_NPZ = os.path.join(HERE, "usda", "maze", "paths.npz")   # 랜덤 경로 묶음 (env가 에피소드마다 하나씩 뽑음)
N_PATHS = 2000
MAX_WP = 40          # 경로당 웨이포인트 최대 (첫 점 = 출발 포함 MAX_WP + 1)
RES = 0.02           # [m] 격자 한 칸
SLICE_Z = 0.15       # [m] 단면 높이 (라이다 LDS-03 높이 근처)
ROBOT_R = 0.20       # [m] 로봇 반경 (Waffle 0.281 × 0.306 m 외접원 근처) — 이보다 벽에 가까운 칸은 경로 금지
CLEAR_W = 4.0        # 벽에 가까울수록 추가 비용 가중치 (통로 가운데로)
WAYPOINT_STEP = 0.30 # [m] 웨이포인트 간격


# ------------------------------------------------------------------ 격자 만들기 (pxr)
def build(usd_path: str = MAZE_USD, res: float = RES, z: float = SLICE_Z):
    """미로 USD → (occ bool [ny, nx], origin (x0, y0)). occ[iy, ix] = 벽."""
    from pxr import Usd, UsdGeom

    st = Usd.Stage.Open(usd_path)
    xf = UsdGeom.XformCache(Usd.TimeCode.Default())
    segs, circles = [], []
    for p in st.Traverse():
        if "/collision" not in str(p.GetPath()):
            continue
        m = xf.GetLocalToWorldTransform(p)
        if p.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(p)
            pts = [m.Transform(v) for v in mesh.GetPointsAttr().Get()]
            counts, idx, k = mesh.GetFaceVertexCountsAttr().Get(), mesh.GetFaceVertexIndicesAttr().Get(), 0
            for c in counts:
                poly = [pts[i] for i in idx[k:k + c]]
                k += c
                for j in range(1, c - 1):   # 삼각형 부채꼴로 나눠 z 평면과의 교선
                    tri = (poly[0], poly[j], poly[j + 1])
                    cut = []
                    for a, b in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
                        if (a[2] - z) * (b[2] - z) < 0:
                            u = (z - a[2]) / (b[2] - a[2])
                            cut.append((a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])))
                    if len(cut) == 2:
                        segs.append(cut)
        elif p.IsA(UsdGeom.Cylinder):
            cyl = UsdGeom.Cylinder(p)
            r0 = cyl.GetRadiusAttr().Get()
            c0 = m.Transform((0.0, 0.0, 0.0))
            sx = m.TransformDir((1.0, 0.0, 0.0)).GetLength()
            circles.append((c0[0], c0[1], r0 * sx))
    xs = [x for s in segs for x, _ in s] + [c[0] for c in circles]
    ys = [y for s in segs for _, y in s] + [c[1] for c in circles]
    x0, y0 = min(xs) - 0.1, min(ys) - 0.1
    nx, ny = int(math.ceil((max(xs) + 0.1 - x0) / res)), int(math.ceil((max(ys) + 0.1 - y0) / res))
    occ = np.zeros((ny, nx), dtype=bool)
    for (ax, ay), (bx, by) in segs:   # 선분을 반 칸 간격으로 찍기
        n = max(2, int(math.hypot(bx - ax, by - ay) / (res * 0.5)) + 1)
        t = np.linspace(0.0, 1.0, n)
        ix = ((ax + (bx - ax) * t - x0) / res).astype(int)
        iy = ((ay + (by - ay) * t - y0) / res).astype(int)
        occ[iy.clip(0, ny - 1), ix.clip(0, nx - 1)] = True
    yy, xx = np.mgrid[0:ny, 0:nx]
    for cx, cy, r in circles:
        occ |= (x0 + (xx + 0.5) * res - cx) ** 2 + (y0 + (yy + 0.5) * res - cy) ** 2 <= r * r
    return occ, (x0, y0), len(segs), len(circles)


# ------------------------------------------------------------------ 격자 쓰기 (numpy)
def load(path: str = GRID_NPZ) -> dict:
    d = np.load(path)
    return {"occ": d["occ"], "origin": tuple(d["origin"]), "res": float(d["res"]), "dist": d["dist"]}


def wall_distance(occ: np.ndarray, res: float) -> np.ndarray:
    """칸마다 가장 가까운 벽까지 거리 [m] (8방향 brushfire)."""
    ny, nx = occ.shape
    dist = np.full(occ.shape, np.inf)
    q = []
    for iy, ix in zip(*np.nonzero(occ)):
        dist[iy, ix] = 0.0
        q.append((0.0, iy, ix))
    heapq.heapify(q)
    steps = [(dy, dx, math.hypot(dy, dx) * res) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
    while q:
        d, iy, ix = heapq.heappop(q)
        if d > dist[iy, ix]:
            continue
        for dy, dx, c in steps:
            jy, jx = iy + dy, ix + dx
            if 0 <= jy < ny and 0 <= jx < nx and d + c < dist[jy, jx]:
                dist[jy, jx] = d + c
                heapq.heappush(q, (d + c, jy, jx))
    return dist


def inside(occ: np.ndarray) -> np.ndarray:
    """미로 바깥 테두리 안쪽 칸 (격자 가장자리에서 벽을 안 넘고 닿는 칸 = 바깥)."""
    ny, nx = occ.shape
    out = np.zeros(occ.shape, dtype=bool)
    stack = [(iy, ix) for iy in (0, ny - 1) for ix in range(nx)] + [(iy, ix) for ix in (0, nx - 1) for iy in range(ny)]
    while stack:
        iy, ix = stack.pop()
        if 0 <= iy < ny and 0 <= ix < nx and not out[iy, ix] and not occ[iy, ix]:
            out[iy, ix] = True
            stack += [(iy + 1, ix), (iy - 1, ix), (iy, ix + 1), (iy, ix - 1)]
    return ~out & ~occ


def astar(free: np.ndarray, dist: np.ndarray, start, goal):
    """free 칸 위 8방향 A* (비용 = 거리 × (1 + CLEAR_W × 벽 근접)). start · goal = (iy, ix). 없으면 None."""
    ny, nx = free.shape
    near = np.clip(1.0 - (dist - ROBOT_R) / ROBOT_R, 0.0, 1.0)   # 벽 쪽 ROBOT_R 안에서 0 → 1
    steps = [(dy, dx, math.hypot(dy, dx)) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
    g = {start: 0.0}
    came = {}
    h = lambda a: math.hypot(a[0] - goal[0], a[1] - goal[1])  # noqa: E731
    q = [(h(start), start)]
    while q:
        _, cur = heapq.heappop(q)
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        for dy, dx, c in steps:
            nb = (cur[0] + dy, cur[1] + dx)
            if not (0 <= nb[0] < ny and 0 <= nb[1] < nx) or not free[nb]:
                continue
            ng = g[cur] + c * (1.0 + CLEAR_W * near[nb])
            if ng < g.get(nb, math.inf):
                g[nb], came[nb] = ng, cur
                heapq.heappush(q, (ng + h(nb), nb))
    return None


def sample_paths(grid: dict, n: int, seed: int = 0, min_len: float = 1.5, max_waypoints: int = MAX_WP) -> list[np.ndarray]:
    """랜덤 출발 · 도착(벽에서 ROBOT_R 이상) → A* → WAYPOINT_STEP 간격 웨이포인트 [(x, y), …] (첫 점 = 출발). n개."""
    occ, res, (x0, y0), dist = grid["occ"], grid["res"], grid["origin"], grid["dist"]
    free = (dist >= ROBOT_R) & inside(occ)
    cells = np.argwhere(free)
    rng = np.random.default_rng(seed)
    out = []
    while len(out) < n:
        a, b = (tuple(cells[i]) for i in rng.integers(0, len(cells), 2))
        if math.hypot(a[0] - b[0], a[1] - b[1]) * res < min_len:
            continue
        path = astar(free, dist, a, b)
        if path is None:
            continue
        xy = np.array([(x0 + (ix + 0.5) * res, y0 + (iy + 0.5) * res) for iy, ix in path])
        seg = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        if seg[-1] < min_len:
            continue
        s = np.r_[np.arange(0.0, seg[-1], WAYPOINT_STEP), seg[-1]][:max_waypoints + 1]
        out.append(np.stack([np.interp(s, seg, xy[:, 0]), np.interp(s, seg, xy[:, 1])], axis=-1))
    return out


if __name__ == "__main__":   # 격자 만들기 + 확인 그림
    occ, origin, n_seg, n_cyl = build()
    dist = wall_distance(occ, RES)
    np.savez_compressed(GRID_NPZ, occ=occ, origin=np.array(origin), res=RES, dist=dist)
    free = (dist >= ROBOT_R) & inside(occ)
    print(f"[grid] {GRID_NPZ} · {occ.shape[1]}×{occ.shape[0]} 칸 ({RES} m) · 원점 {origin} · 선분 {n_seg} · 기둥 {n_cyl} · "
          f"주행 가능 {free.mean():.1%}")
    paths = sample_paths(load(), N_PATHS)
    assert all(2 <= len(p) <= MAX_WP + 1 for p in paths)
    pts = np.zeros((N_PATHS, MAX_WP + 1, 2), dtype=np.float32)
    lens = np.array([len(p) for p in paths], dtype=np.int64)
    for i, p in enumerate(paths):
        pts[i, :len(p)], pts[i, len(p):] = p, p[-1]   # 남는 자리는 마지막 점
    np.savez_compressed(PATHS_NPZ, pts=pts, lens=lens)
    print(f"[grid] {PATHS_NPZ} · 경로 {N_PATHS}개 · 웨이포인트 {lens.min()}~{lens.max()}개 (평균 {lens.mean():.1f})")
    paths = paths[:5]
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        x0, y0 = origin
        ext = (x0, x0 + occ.shape[1] * RES, y0, y0 + occ.shape[0] * RES)
        plt.figure(figsize=(6, 8))
        plt.imshow(np.where(occ, 0, np.where(free, 2, 1)), origin="lower", extent=ext, cmap="gray")
        for p in paths:
            plt.plot(p[:, 0], p[:, 1], ".-", ms=4)
        plt.gca().set_aspect("equal")
        plt.title("black = wall · gray = within robot radius · white = drivable · lines = 5 random paths")
        png = GRID_NPZ.replace(".npz", ".png")
        plt.savefig(png, dpi=120)
        print(f"[grid] {png}")
    except Exception as exc:
        print(f"[grid] 그림 스킵 ({exc})")
