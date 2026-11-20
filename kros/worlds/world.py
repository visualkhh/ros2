import math
from abc import ABC
from collections.abc import Sequence

from kros.msgs.geometry_msgs import Pose2D
from kros.worlds.clock import Clock
from kros.worlds.collision import Box, Circle, Compound, circle_vs, ray_distances_batch, ray_hit
from kros.worlds.compute import Compute, compute
from kros.worlds.world_object import WorldObject

EARTH_GRAVITY = 9.81  # m/s^2
MAX_MOVE_STEP = 0.02  # m, 벽 중심선을 건너뛰지 않도록 한 번에 이 이하로만 옮긴다


class World(ABC):
    """WorldObject들(벽, 로봇, 퍽 ...)이 놓이는 공간. 시계를 쥐고 있고, 객체들의 충돌과 레이를 풀어 준다.

    객체가 무엇인지(벽, 로봇, 센서, 명령)는 모른다. kros와 무관하다.
    """

    name: str
    start_pose: Pose2D  # 객체를 놓기 좋은 자리

    def __init__(self, compute_backend: Compute = compute, gravity: float = EARTH_GRAVITY, ground_friction: float = 0.3) -> None:
        self.compute = compute_backend  # GPU -> CPU -> 순수 파이썬 순으로 내려가는 계산 백엔드
        self.gravity = gravity  # m/s^2, 월드의 중력가속도 (달이면 1.62)
        self.ground_friction = ground_friction  # 바닥 재질의 마찰 계수 (얼음이면 0.05)
        self.time = 0.0  # 월드 시각(초)
        self.objects: list[WorldObject] = []
        self._statics: list[WorldObject] = []  # push_out이 매번 훑는 벽들 (add에서 모아 둔다)

    def add(self, obj: WorldObject, pose: Pose2D | None = None) -> None:
        """객체를 월드에 놓는다 (pose를 안 주면 start_pose)."""
        obj.pose = pose or self.start_pose
        self.objects.append(obj)
        if obj.static and obj.collider is not None:
            self._statics.append(obj)

    def weight(self, obj: WorldObject) -> float:
        """이 월드에서 객체의 무게(N) = 질량 x 중력가속도. 같은 질량도 월드가 다르면 무게가 다르다."""
        return obj.mass * self.gravity

    def friction_with_ground(self, obj: WorldObject) -> float:
        """객체와 바닥 사이의 마찰 계수: 두 재질의 계수를 기하평균으로 합친다 (물리 엔진들이 쓰는 방식)."""
        return math.sqrt(obj.friction * self.ground_friction)

    def step(self, dt: float) -> None:
        """시계를 dt초 진행시키고, 모든 객체에 시계 신호를 준다."""
        self.time += dt
        clock = Clock(self.time, dt)
        for obj in self.objects:
            obj.tick(clock, self)
        self._slide(dt)
        self._resolve_collisions(dt)

    def ray_distance(self, x: float, y: float, angle: float, height: float = 0.0) -> float:
        """(x, y)에서 angle 방향으로, 바닥에서 height(m) 높이로 수평하게 쏜 선이 처음 닿는 객체까지 거리 (m). 없으면 inf.

        height보다 낮은 객체는 선이 위로 지나가므로 닿지 않는다. 출발점이 도형 안이면 그 객체는 무시한다(센서가 자기 몸체에 가리지 않게).
        """
        best = math.inf
        for obj in self.objects:
            shape = obj.collider
            if shape is not None and obj.height > height:
                best = min(best, ray_hit(shape, obj.pose.x, obj.pose.y, x, y, angle))
        return best

    def ray_distances(self, x: float, y: float, angles: Sequence[float], height: float = 0.0) -> list[float]:
        """같은 출발점에서 여러 방향을 한꺼번에 묻는다 (라이다 한 바퀴). 작업량이 크면 GPU(cupy), 보통은 CPU(numpy),
        둘 다 없으면 순수 파이썬으로 계산한다. 어느 쪽이든 결과는 같다."""
        boxes: list[tuple[float, float, float, float]] = []
        circles: list[tuple[float, float, float]] = []
        for obj in self.objects:
            shape = obj.collider
            if shape is None or obj.height <= height:
                continue
            if isinstance(shape, Compound):
                boxes.extend((obj.pose.x + ox, obj.pose.y + oy, box.half_x, box.half_y) for ox, oy, box in shape.parts)
            elif isinstance(shape, Box):
                boxes.append((obj.pose.x, obj.pose.y, shape.half_x, shape.half_y))
            elif isinstance(shape, Circle):
                circles.append((obj.pose.x, obj.pose.y, shape.radius))
        xp = self.compute.pick(len(angles) * (len(boxes) + len(circles)))
        if xp is not None:
            try:
                result = ray_distances_batch(xp, x, y, xp.asarray(angles, dtype=float), xp.asarray(boxes, dtype=float).reshape(-1, 4), xp.asarray(circles, dtype=float).reshape(-1, 3))
                return self.compute.to_list(result)
            except Exception as error:
                if xp is not self.compute.cupy:
                    raise
                self.compute.disable_gpu(error)  # GPU가 안 되면 CPU로 다시
                return self.ray_distances(x, y, angles, height)
        return [self.ray_distance(x, y, a, height) for a in angles]  # numpy도 없을 때: 순수 파이썬

    def push_out(self, x: float, y: float, radius: float) -> tuple[float, float]:
        """(x, y)에 반지름 radius 원을 놓았을 때 static 객체(벽)를 파고들었으면 밀어낸 위치를 돌려준다."""
        for _ in range(4):  # 모서리에서 두 벽에 동시에 눌릴 때를 위해 반복
            for obj in self._statics:
                shape = obj.collider
                if shape is not None:
                    hit = circle_vs(shape, obj.pose.x, obj.pose.y, x, y, radius)
                    if hit is not None:
                        x, y = x + hit[0] * hit[2], y + hit[1] * hit[2]
        return x, y

    def _move(self, obj: WorldObject, mx: float, my: float) -> tuple[float, float]:
        """obj를 (mx, my)만큼 옮기되 벽을 파고들면 밀려난다. 벽 중심선을 건너뛰지 않게 잘게 쪼갠다. 실제로 움직인 거리를 돌려준다."""
        x0, y0 = obj.pose.x, obj.pose.y
        x, y = x0, y0
        n = max(1, math.ceil(math.hypot(mx, my) / MAX_MOVE_STEP))
        for _ in range(n):
            x, y = self.push_out(x + mx / n, y + my / n, obj.radius)
        obj.pose = Pose2D(x, y, obj.pose.theta)
        return x - x0, y - y0

    def _slide(self, dt: float) -> None:
        """밀려서 속도가 생긴 객체는 미끄러지고, 바닥 마찰(합친 마찰 계수 x 월드 중력)로 감속해 멈춘다. 벽에 막힌 만큼 속도도 줄어든다."""
        for obj in self.objects:
            vx, vy = obj.velocity
            if not obj.movable or (vx == 0.0 and vy == 0.0) or dt <= 0:
                continue
            dx, dy = self._move(obj, vx * dt, vy * dt)
            speed = math.hypot(dx, dy) / dt  # 벽에 막혀 실제로 움직인 만큼만 속도로 남는다
            if speed == 0.0:
                obj.velocity = (0.0, 0.0)
                continue
            slowed = max(0.0, speed - self.friction_with_ground(obj) * self.gravity * dt) / speed
            obj.velocity = (dx / dt * slowed, dy / dt * slowed)

    def _resolve_collisions(self, dt: float) -> None:
        """겹친 객체들을 밀어낸다. movable이 아닌 객체(로봇)는 안 밀리고, movable끼리는 질량이 작은 쪽이 더 밀린다.
        밀린 객체는 실제로 움직인 만큼의 속도를 얻는다. 밀리는 쪽이 벽에 막혀 못 물러나면 안 밀리는 쪽(로봇)이 대신 물러난다.
        static 객체(벽)는 push_out이 맡는다."""
        dynamic = [o for o in self.objects if not o.static and o.radius > 0]
        pushed = {id(o): [0.0, 0.0] for o in dynamic}  # 이번 step에 객체끼리 부딪혀 실제로 밀린 거리
        for _ in range(4):  # 한 번에 안 풀리는 겹침(여럿이 눌릴 때)을 위해 반복
            for i, a in enumerate(dynamic):
                for b in dynamic[i + 1 :]:
                    if not (a.movable or b.movable):
                        continue
                    dx, dy = b.pose.x - a.pose.x, b.pose.y - a.pose.y
                    d = math.hypot(dx, dy)
                    gap = a.radius + b.radius
                    if d >= gap:
                        continue
                    nx, ny = (dx / d, dy / d) if d > 1e-9 else (1.0, 0.0)  # a에서 b 방향 (정확히 같은 자리면 임의 방향)
                    overlap = gap - d
                    to_a = 0.0 if not a.movable else 1.0 if not b.movable else b.mass / (a.mass + b.mass)  # a가 물러나는 비율
                    for obj, share, sign in ((a, to_a, -1), (b, 1 - to_a, 1)):
                        if share > 0:
                            mx, my = self._move(obj, sign * nx * overlap * share, sign * ny * overlap * share)
                            pushed[id(obj)][0] += mx
                            pushed[id(obj)][1] += my
                    d = math.hypot(b.pose.x - a.pose.x, b.pose.y - a.pose.y)
                    if d < gap and not (a.movable and b.movable):  # 벽에 막혀 덜 물러났다: 안 밀리는 쪽이 남은 만큼 물러난다
                        rest = gap - d
                        yielder, sign = (a, -1) if not a.movable else (b, 1)
                        yielder.pose = Pose2D(yielder.pose.x + sign * nx * rest, yielder.pose.y + sign * ny * rest, yielder.pose.theta)
            for obj in dynamic:
                x, y = self.push_out(obj.pose.x, obj.pose.y, obj.radius)
                obj.pose = Pose2D(x, y, obj.pose.theta)
        if dt > 0:
            for obj in dynamic:
                if obj.movable:
                    mx, my = pushed[id(obj)]
                    obj.velocity = (obj.velocity[0] + mx / dt, obj.velocity[1] + my / dt)
