"""cartographer의 ProbabilityGrid 흉내: 칸마다 "벽일 확률"을 log-odds로 쌓는 2D 격자 (칸은 필요할 때만 생긴다)."""
import math

from kros.msgs.geometry_msgs import Point, Pose, Quaternion
from kros.msgs.nav_msgs import OCCUPIED, UNKNOWN, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header

HIT, MISS = 0.85, -0.4  # 레이가 끝난 칸(벽)과 지나간 칸(빈 곳)에 더하는 log-odds (cartographer의 hit/miss 확률 0.55/0.49보다 빨리 확신한다)
LIMIT_LOW, LIMIT_HIGH = -2.0, 3.5  # 너무 확신해서 다시는 못 바뀌지 않게


def _probability(log_odds: float) -> float:
    return 1.0 - 1.0 / (1.0 + math.exp(log_odds))


class ProbabilityGrid:
    """칸 (i, j)는 x가 [i*res, (i+1)*res), y가 [j*res, (j+1)*res)인 사각형 (map 좌표계). 모르는 칸은 없다(확률 0.5)."""

    def __init__(self, resolution: float) -> None:
        self.resolution = resolution
        self.log_odds: dict[tuple[int, int], float] = {}
        self._p: dict[tuple[int, int], float] = {}  # 확률 캐시 (스캔 매칭이 자주 읽는다)

    def cell(self, x: float, y: float) -> tuple[int, int]:
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    def probability(self, x: float, y: float) -> float:
        """(x, y)에서 벽일 확률을 둘레 네 칸 중심에서 이중선형 보간한다 (스캔 매칭 점수가 칸 경계에서 계단처럼 튀지 않게). 모르는 칸은 0.5."""
        u, v = x / self.resolution - 0.5, y / self.resolution - 0.5
        i, j = math.floor(u), math.floor(v)
        fu, fv = u - i, v - j
        p = self._p
        return (
            p.get((i, j), 0.5) * (1 - fu) * (1 - fv) + p.get((i + 1, j), 0.5) * fu * (1 - fv)
            + p.get((i, j + 1), 0.5) * (1 - fu) * fv + p.get((i + 1, j + 1), 0.5) * fu * fv
        )

    def _add(self, cell: tuple[int, int], amount: float) -> None:
        value = min(LIMIT_HIGH, max(LIMIT_LOW, self.log_odds.get(cell, 0.0) + amount))
        self.log_odds[cell] = value
        self._p[cell] = _probability(value)

    def insert(self, sensor: tuple[float, float], hits: list[tuple[float, float]]) -> None:
        """센서 위치에서 각 끝점까지 레이를 그어, 지나간 칸은 빈 곳 쪽으로, 끝점 칸은 벽 쪽으로 (map 좌표, m)."""
        start = self.cell(*sensor)
        ends = {self.cell(*hit) for hit in hits}
        misses: set[tuple[int, int]] = set()
        for end in ends:
            misses.update(_line(start, end))
        for cell in misses - ends:  # 한 스캔에서 칸마다 한 번만 (같은 칸을 여러 레이가 지나도)
            self._add(cell, MISS)
        for cell in ends:
            self._add(cell, HIT)

    def to_message(self, header: Header) -> OccupancyGrid:
        """닿은 칸을 모두 담는 가장 작은 사각형으로 OccupancyGrid를 만든다 (확률 0~100, 닿지 않은 칸 -1)."""
        if not self.log_odds:
            return OccupancyGrid(header, MapMetaData(header.stamp, self.resolution, 0, 0))
        xs, ys = [c[0] for c in self.log_odds], [c[1] for c in self.log_odds]
        x0, y0, width, height = min(xs), min(ys), max(xs) - min(xs) + 1, max(ys) - min(ys) + 1
        p = self._p
        data = tuple(UNKNOWN if (c := (x0 + i, y0 + j)) not in p else round(p[c] * 100) for j in range(height) for i in range(width))
        origin = Pose(Point(x0 * self.resolution, y0 * self.resolution, 0.0), Quaternion())
        return OccupancyGrid(header, MapMetaData(header.stamp, self.resolution, width, height, origin), data)

    @classmethod
    def from_message(cls, grid: OccupancyGrid) -> "ProbabilityGrid":
        """저장한 지도(map_server)로 만든다: 칸 중심을 이 격자의 칸에 놓는다. shortcut: 지도 origin의 회전은 무시한다 (map_server 지도는 보통 0)."""
        info = grid.info
        made = cls(info.resolution)
        o = info.origin.position
        for j in range(info.height):
            for i in range(info.width):
                value = grid.data[j * info.width + i]
                if value == UNKNOWN:
                    continue
                cell = made.cell(o.x + (i + 0.5) * info.resolution, o.y + (j + 0.5) * info.resolution)
                p = min(0.99, max(0.01, value / OCCUPIED))  # map_server 지도는 0/100, SLAM이 낸 지도는 그 사이 확률
                made._add(cell, math.log(p / (1 - p)))
        return made


def _line(start: tuple[int, int], end: tuple[int, int]) -> list[tuple[int, int]]:
    """브레젠험 직선: start부터 end 앞까지의 칸."""
    (x, y), (x1, y1) = start, end
    dx, dy = abs(x1 - x), -abs(y1 - y)
    sx, sy = (1 if x < x1 else -1), (1 if y < y1 else -1)
    error = dx + dy
    cells = []
    while (x, y) != (x1, y1):
        cells.append((x, y))
        e2 = 2 * error
        if e2 >= dy:
            error += dy
            x += sx
        if e2 <= dx:
            error += dx
            y += sy
    return cells
