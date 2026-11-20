"""nav2_costmap_2d의 Costmap2D 흉내: 칸마다 0~255 비용. 경로 계획/제어가 "여기를 지나가도 되나, 얼마나 싫은가"를 묻는 격자.

비용 (nav2와 같다): FREE_SPACE 0, 1~252 벽에 가까울수록 비쌈(inflation), INSCRIBED_INFLATED_OBSTACLE 253 (로봇 중심이 여기 오면 몸이 벽에 닿는다),
LETHAL_OBSTACLE 254 (벽), NO_INFORMATION 255 (모름).
"""
import math
from dataclasses import dataclass, field

from kros.msgs.geometry_msgs import Point, Pose, Quaternion
from kros.msgs.nav_msgs import UNKNOWN, MapMetaData, OccupancyGrid
from kros.msgs.std_msgs import Header

FREE_SPACE, INSCRIBED_INFLATED_OBSTACLE, LETHAL_OBSTACLE, NO_INFORMATION = 0, 253, 254, 255
LETHAL_THRESHOLD = 65  # 지도 칸이 이 이상(점유 확률 %)이면 벽 (map_server의 occupied_thresh 0.65와 같다)


def translate(cost: int) -> int:
    """비용 -> OccupancyGrid 값 (nav2의 cost_translation_table): 0 -> 0, 1~252 -> 1~98, 253 -> 99, 254 -> 100, 255 -> -1."""
    if cost == NO_INFORMATION:
        return UNKNOWN
    if cost >= INSCRIBED_INFLATED_OBSTACLE:
        return cost - 154
    return 0 if cost == 0 else 1 + (97 * (cost - 1)) // 251


@dataclass
class Costmap:
    """origin은 칸 (0, 0)의 모서리 (global_frame 좌표). 칸 (i, j)는 x 방향 i번째, y 방향 j번째."""

    resolution: float
    width: int
    height: int
    origin_x: float = 0.0
    origin_y: float = 0.0
    costs: bytearray = field(default_factory=bytearray)

    def __post_init__(self) -> None:
        if not self.costs:
            self.costs = bytearray(self.width * self.height)

    def world_to_map(self, x: float, y: float) -> tuple[int, int] | None:
        i, j = math.floor((x - self.origin_x) / self.resolution), math.floor((y - self.origin_y) / self.resolution)
        return (i, j) if 0 <= i < self.width and 0 <= j < self.height else None

    def map_to_world(self, i: int, j: int) -> tuple[float, float]:
        """칸 중심."""
        return self.origin_x + (i + 0.5) * self.resolution, self.origin_y + (j + 0.5) * self.resolution

    def cost(self, i: int, j: int) -> int:
        return self.costs[j * self.width + i]

    def cost_at(self, x: float, y: float) -> int:
        """world 점의 비용. 지도 밖은 모름."""
        cell = self.world_to_map(x, y)
        return NO_INFORMATION if cell is None else self.cost(*cell)

    @classmethod
    def from_occupancy_grid(cls, grid: OccupancyGrid, track_unknown_space: bool = True) -> "Costmap":
        """static_layer: 지도의 벽은 LETHAL, 빈 곳 FREE, 모름은 track_unknown_space면 NO_INFORMATION(아니면 FREE).
        shortcut: 지도 origin의 회전은 무시한다 (map_server/SLAM 지도는 0)."""
        info = grid.info
        unknown = NO_INFORMATION if track_unknown_space else FREE_SPACE
        costs = bytearray(unknown if v == UNKNOWN else LETHAL_OBSTACLE if v >= LETHAL_THRESHOLD else FREE_SPACE for v in grid.data)
        return cls(info.resolution, info.width, info.height, info.origin.position.x, info.origin.position.y, costs)

    def inflate(self, inscribed_radius: float, inflation_radius: float, cost_scaling_factor: float) -> None:
        """inflation_layer: 벽 둘레를 거리에 따라 비싸게 칠한다.
        거리 d <= inscribed_radius면 INSCRIBED, 그 밖 inflation_radius까지는 252 * exp(-cost_scaling_factor * (d - inscribed_radius))."""
        res, w, h, costs = self.resolution, self.width, self.height, self.costs
        reach = math.ceil(inflation_radius / res)
        kernel = []  # (di, dj, 비용): 벽 칸에서 이만큼 떨어진 칸에 줄 비용 (가까운 칸에 큰 값)
        for di in range(-reach, reach + 1):
            for dj in range(-reach, reach + 1):
                d = math.hypot(di, dj) * res
                if 0 < d <= inflation_radius:
                    value = INSCRIBED_INFLATED_OBSTACLE if d <= inscribed_radius else int(252 * math.exp(-cost_scaling_factor * (d - inscribed_radius)))
                    if value > 0:
                        kernel.append((di, dj, value))
        lethal = [(i, j) for j in range(h) for i in range(w) if costs[j * w + i] == LETHAL_OBSTACLE]
        lethal_set = set(lethal)
        edges = [(i, j) for i, j in lethal if any((i + a, j + b) not in lethal_set for a, b in ((1, 0), (-1, 0), (0, 1), (0, -1)))]  # 벽 안쪽 칸은 바깥에 영향이 없다
        for i, j in edges:
            for di, dj, value in kernel:
                x, y = i + di, j + dj
                if 0 <= x < w and 0 <= y < h:
                    k = y * w + x
                    old = costs[k]
                    if old == NO_INFORMATION:
                        if value >= INSCRIBED_INFLATED_OBSTACLE:  # nav2 기본(inflate_unknown false): 모르는 칸은 몸이 닿을 만큼 가까울 때만 덮는다
                            costs[k] = value
                    elif old < value:  # 여러 벽에서 온 값 중 가장 큰 것 (LETHAL 254는 어떤 value보다 커서 그대로)
                        costs[k] = value

    def to_message(self, header: Header) -> OccupancyGrid:
        origin = Pose(Point(self.origin_x, self.origin_y, 0.0), Quaternion())
        return OccupancyGrid(header, MapMetaData(header.stamp, self.resolution, self.width, self.height, origin), tuple(translate(c) for c in self.costs))
