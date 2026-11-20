"""충돌/레이 계산용 도형. 순수 계산 (kros와 무관)."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Circle:
    radius: float


@dataclass(frozen=True)
class Box:
    """월드 축에 평행한 상자 (pose.theta는 무시). 반폭/반높이."""

    half_x: float
    half_y: float


Shape = Circle | Box


def ray_hit(shape: Shape, cx: float, cy: float, x: float, y: float, angle: float) -> float:
    """중심 (cx, cy)의 도형에 (x, y)에서 angle 방향 선이 처음 닿는 거리. 안 닿거나 출발점이 도형 안이면 inf."""
    dx, dy = math.cos(angle), math.sin(angle)
    if isinstance(shape, Circle):
        fx, fy = cx - x, cy - y
        along = fx * dx + fy * dy
        off2 = fx * fx + fy * fy - along * along
        if fx * fx + fy * fy <= shape.radius**2 or off2 > shape.radius**2 or along < 0:
            return math.inf
        return along - math.sqrt(shape.radius**2 - off2)
    t0, t1 = 0.0, math.inf  # slab 방법: 선이 x 구간과 y 구간에 동시에 들어가 있는 t 범위
    for o, d, lo, hi in ((x, dx, cx - shape.half_x, cx + shape.half_x), (y, dy, cy - shape.half_y, cy + shape.half_y)):
        if abs(d) < 1e-12:
            if not lo <= o <= hi:
                return math.inf  # 평행이고 구간 밖
            continue
        ta, tb = (lo - o) / d, (hi - o) / d
        t0, t1 = max(t0, min(ta, tb)), min(t1, max(ta, tb))
    if t0 > t1 or (t0 == 0.0 and cx - shape.half_x < x < cx + shape.half_x and cy - shape.half_y < y < cy + shape.half_y):
        return math.inf  # 못 맞췄거나 출발점이 상자 안
    return t0


def circle_vs(shape: Shape, cx: float, cy: float, x: float, y: float, radius: float) -> tuple[float, float, float] | None:
    """중심 (x, y) 반지름 radius 원이 (cx, cy)의 도형을 파고들었는가. 파고들었으면 (도형에서 원 쪽 단위 법선 nx, ny, 깊이)."""
    if isinstance(shape, Circle):
        dx, dy = x - cx, y - cy
        d = math.hypot(dx, dy)
        depth = radius + shape.radius - d
        if depth <= 0:
            return None
        return (dx / d, dy / d, depth) if d > 1e-9 else (1.0, 0.0, depth)  # 정확히 같은 자리면 임의 방향
    px, py = min(max(x, cx - shape.half_x), cx + shape.half_x), min(max(y, cy - shape.half_y), cy + shape.half_y)  # 상자에서 가장 가까운 점
    dx, dy = x - px, y - py
    d = math.hypot(dx, dy)
    if d > 1e-9:
        return (dx / d, dy / d, radius - d) if d < radius else None
    sides = ((x - (cx - shape.half_x), -1.0, 0.0), ((cx + shape.half_x) - x, 1.0, 0.0), (y - (cy - shape.half_y), 0.0, -1.0), ((cy + shape.half_y) - y, 0.0, 1.0))
    dist, nx, ny = min(sides)  # 중심이 상자 안: 가장 가까운 면 쪽으로 빼낸다
    return nx, ny, dist + radius


def ray_distances_batch(xp, x: float, y: float, angles, boxes, circles):  # type: ignore[no-untyped-def]
    """한 출발점 (x, y)에서 여러 방향(angles: 배열)을 한꺼번에 쏜다. numpy와 같은 API의 배열 모듈 xp(numpy 또는 cupy)로 계산한다.

    boxes: (M, 4) 배열 [cx, cy, half_x, half_y], circles: (K, 3) 배열 [cx, cy, radius]. 방향별 가장 가까운 거리 배열(없으면 inf).
    ray_hit과 같은 규칙: 출발점이 도형 안이면 그 도형은 무시한다.
    """
    dx, dy = xp.cos(angles)[:, None], xp.sin(angles)[:, None]  # (N, 1)
    best = xp.full(angles.shape[0], xp.inf)
    inf = xp.inf
    if boxes.shape[0]:
        cx, cy, hx, hy = (boxes[:, i][None, :] for i in range(4))  # (1, M)
        t0, t1 = xp.zeros((angles.shape[0], boxes.shape[0])), xp.full((angles.shape[0], boxes.shape[0]), inf)
        for o, d, lo, hi in ((x, dx, cx - hx, cx + hx), (y, dy, cy - hy, cy + hy)):
            parallel = xp.abs(d) < 1e-12
            safe = xp.where(parallel, 1.0, d)
            ta, tb = (lo - o) / safe, (hi - o) / safe
            inside = (lo <= o) & (o <= hi)
            tmin = xp.where(parallel, xp.where(inside, -inf, inf), xp.minimum(ta, tb))  # 평행하고 구간 밖이면 못 맞춘다
            tmax = xp.where(parallel, xp.where(inside, inf, -inf), xp.maximum(ta, tb))
            t0, t1 = xp.maximum(t0, tmin), xp.minimum(t1, tmax)
        origin_inside = ((cx - hx < x) & (x < cx + hx) & (cy - hy < y) & (y < cy + hy))  # (1, M)
        hit = (t0 <= t1) & ~origin_inside
        best = xp.minimum(best, xp.where(hit, t0, inf).min(axis=1))
    if circles.shape[0]:
        fx, fy, r = circles[:, 0][None, :] - x, circles[:, 1][None, :] - y, circles[:, 2][None, :]
        along = fx * dx + fy * dy
        off2 = fx * fx + fy * fy - along * along
        hit = ((fx * fx + fy * fy) > r * r) & (off2 <= r * r) & (along >= 0)
        dist = along - xp.sqrt(xp.maximum(r * r - off2, 0.0))
        best = xp.minimum(best, xp.where(hit, dist, inf).min(axis=1))
    return best
