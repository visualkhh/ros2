import math
import random

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.compute import Compute, compute
from kros.worlds.objects import Puck, PuckShape
from kros.worlds.world import EARTH_GRAVITY, World

from . import track
from .track import Track

PUCK_COLORS: tuple[tuple[float, float, float], ...] = (
    (0.90, 0.20, 0.20), (0.95, 0.55, 0.10), (0.95, 0.85, 0.15), (0.30, 0.75, 0.30), (0.15, 0.70, 0.75),
    (0.25, 0.45, 0.90), (0.60, 0.35, 0.85), (0.90, 0.40, 0.70), (0.95, 0.95, 0.95), (0.55, 0.35, 0.20),
)


class MazeWorld(World):
    """isaac rl_maze 트랙: 벽 전체(Track 객체 하나). 시작할 때 퍽 puck_count개를 여러 색/모양으로 미로에 흩뿌려 놓는다."""

    name = "maze"
    start_pose = Pose2D(*track.START)

    def __init__(self, puck_count: int = 10, seed: int | None = None, compute_backend: Compute = compute, gravity: float = EARTH_GRAVITY, ground_friction: float = 0.3) -> None:
        super().__init__(compute_backend, gravity, ground_friction)  # 바닥 0.3: isaac 트랙 바닥 재질(PLA) 정도
        self.add(Track(), Pose2D(0.0, 0.0, 0.0))  # 트랙은 벽 전체를 가진 WorldObject 하나
        rng = random.Random(seed)  # seed를 주면 같은 배치
        shapes = list(PuckShape)
        for i in range(puck_count):
            puck = Puck(shapes[i % len(shapes)], PUCK_COLORS[i % len(PUCK_COLORS)])
            self.add(puck, self._free_spot(rng, puck.radius))

    def _free_spot(self, rng: random.Random, radius: float) -> Pose2D:
        """벽에서 10cm 이상, 시작 자리에서 0.5m 이상, 다른 퍽에서 0.3m 이상 떨어진 빈 곳을 뽑는다."""
        half_w, half_h = track.SIZE[0] / 2, track.SIZE[1] / 2
        for _ in range(1000):
            x, y = track.CENTER[0] + rng.uniform(-half_w, half_w), track.CENTER[1] + rng.uniform(-half_h, half_h)
            if self.push_out(x, y, radius + 0.10) != (x, y):
                continue
            if math.hypot(x - self.start_pose.x, y - self.start_pose.y) < 0.5:
                continue
            if any(math.hypot(x - o.pose.x, y - o.pose.y) < 0.3 for o in self.objects if isinstance(o, Puck)):
                continue
            return Pose2D(x, y, rng.uniform(-math.pi, math.pi))
        raise RuntimeError("퍽을 놓을 빈 곳을 찾지 못했다")
