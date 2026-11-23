"""Viz의 내비게이션 그림: 격자(지도/costmap)를 바닥 텍스처로, 경로(plan, local_plan)와 지나온 궤적(trail)을 선으로.
viz.py가 매 프레임 update()를 부른다. 그리는 것은 모두 토픽(또는 TF)에서 온 것이다."""
import math
from collections.abc import Callable
from typing import TYPE_CHECKING, Any


def _yaw(q: Any) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

from panda3d.core import CardMaker, LineSegs, LQuaternionf, NodePath, SamplerState, Texture

from kros.msgs.geometry_msgs import TransformStamped
from kros.msgs.nav_msgs import UNKNOWN, OccupancyGrid, Path

if TYPE_CHECKING:
    from .viz import Viz

Color = tuple[float, float, float, float]
# 지도 칸 색 (rviz map 색과 비슷하게): 모름은 어두운 청회색, 점유 확률 0(빈 곳)~100(벽)은 밝은 회색~검정
MAP_COLORS = {UNKNOWN: bytes((60, 66, 78, 255)), **{v: bytes((round(225 - 2.15 * v),) * 3 + (255,)) for v in range(101)}}


def _costmap_color(value: int) -> bytes:
    """rviz costmap 색: 빈 곳/모름은 투명, 비용 1~98은 파랑 -> 빨강 (반투명), 99(몸이 닿음)는 하늘색, 100(벽)은 자주색."""
    if value <= 0:
        return bytes((0, 0, 0, 0))
    if value >= 100:
        return bytes((200, 0, 200, 230))
    if value == 99:
        return bytes((0, 220, 255, 200))
    t = value / 98
    return bytes((round(255 * t), 0, round(255 * (1 - t)), round(30 + 80 * t)))  # 옅게: 아래 지도가 보이게


COSTMAP_COLORS = {v: _costmap_color(v) for v in range(-1, 101)}
GRIDS = {  # 토픽 -> (색표, 높이: 겹칠 때 위에 오는 순서)
    "map": (MAP_COLORS, -0.002),
    "global_costmap/costmap": (COSTMAP_COLORS, 0.002),
    "local_costmap/costmap": (COSTMAP_COLORS, 0.004),
}
PATHS = {  # 토픽 -> (색, 굵기, 웨이포인트 점을 찍을 간격 (0이면 안 찍음))
    "plan": ((0.2, 1.0, 0.3, 1.0), 4.0, 6),
    "local_plan": ((1.0, 0.85, 0.1, 1.0), 6.0, 0),
}
TRAIL_COLOR: Color = (0.75, 0.45, 1.0, 1.0)  # 지나온 궤적 (보라)
TRAIL_STEP, TRAIL_LIMIT = 0.02, 5000  # 2cm 넘게 움직였을 때만 점을 더하고, 이만큼까지만 남긴다


def grid_card(grid: OccupancyGrid, colors: dict[int, bytes], z: float) -> NodePath | None:
    """격자를 칸마다 색을 칠한 텍스처 한 장으로 (grid의 frame 좌표). 크기가 틀리면 None."""
    info = grid.info
    if not (info.width and info.height and len(grid.data) == info.width * info.height):
        return None
    texture = Texture("grid")
    texture.setup2dTexture(info.width, info.height, Texture.T_unsigned_byte, Texture.F_rgba)
    texture.setRamImageAs(b"".join(colors.get(v, colors[UNKNOWN]) for v in grid.data), "RGBA")  # 텍스처 첫 줄이 아래(y가 작은 쪽): 지도의 행 순서와 같다
    texture.setMagfilter(SamplerState.FT_nearest)  # 칸이 번지지 않게
    texture.setMinfilter(SamplerState.FT_nearest)
    card = CardMaker("grid-card")
    card.setFrame(0, info.width * info.resolution, 0, info.height * info.resolution)
    o, q = info.origin.position, info.origin.orientation
    origin = NodePath("grid-origin")
    origin.setPos(o.x, o.y, o.z + z)
    origin.setQuat(LQuaternionf(q.w, q.x, q.y, q.z))
    plane = origin.attachNewNode(card.generate())
    plane.setP(-90)  # CardMaker는 xz 평면에 만든다: 바닥(xy)으로 눕힌다
    plane.setTexture(texture)
    plane.setLightOff()
    plane.setTwoSided(True)
    plane.setTransparency(True)
    plane.setDepthWrite(False)  # 반투명 costmap이 서로/지도를 가리지 않게
    return origin


def _line(points: list[tuple[float, ...]], color: Color, thickness: float, z: float, dots_every: int = 0) -> NodePath:
    segs = LineSegs()
    segs.setColor(*color)
    segs.setThickness(thickness)
    if points:
        segs.moveTo(points[0][0], points[0][1], z)
        for p in points[1:]:
            segs.drawTo(p[0], p[1], z)
    node = NodePath(segs.create())
    if dots_every:  # 웨이포인트: 경로 점 몇 개마다 작은 화살표 (로봇이 그 점에서 바라볼 방향)
        dots = LineSegs()
        dots.setColor(1, 1, 1, 1)
        dots.setThickness(3)
        saved_bounds = node.node().get_bounds()  # 화살표가 경로의 경계상자를 넓히지 않게 (viz 테스트의 bounds 검사)
        for p in points[::dots_every] + [points[-1]]:
            x, y = p[0], p[1]
            yaw = p[2] if len(p) > 2 else None
            if yaw is None:
                half = 0.06
                dots.moveTo(x - half, y, z)
                dots.drawTo(x + half, y, z)
                dots.moveTo(x, y - half, z)
                dots.drawTo(x, y + half, z)
            else:
                ex = x + 0.12 * math.cos(yaw)
                ey = y + 0.12 * math.sin(yaw)
                dots.moveTo(x, y, z)
                dots.drawTo(ex, ey, z)
                dots.moveTo(ex, ey, z)
                dots.drawTo(ex - 0.04 * math.cos(yaw - 0.5), ey - 0.04 * math.sin(yaw - 0.5), z)
                dots.moveTo(ex, ey, z)
                dots.drawTo(ex - 0.04 * math.cos(yaw + 0.5), ey - 0.04 * math.sin(yaw + 0.5), z)
        node.attachNewNode(dots.create())
        node.node().set_bounds(saved_bounds)
        node.node().set_final(True)
    node.setLightOff()
    node.setBin("fixed", 0)
    node.setDepthTest(False)  # 지도/모델에 가려지지 않게
    return node


class NavDisplay:
    def __init__(self, viz: "Viz") -> None:
        self.viz = viz
        self.grids: dict[str, NodePath] = {}  # 토픽 -> 그 격자를 TF로 놓는 노드
        self._grid_drawn: dict[str, object] = {}
        self.paths: dict[str, NodePath] = {}
        self._path_drawn: dict[str, object] = {}
        for topic in GRIDS:
            self.grids[topic] = viz.scene.attachNewNode(topic)
            self.grids[topic].hide()
        for topic in PATHS:
            self.paths[topic] = viz.scene.attachNewNode(topic)
            self.paths[topic].hide()
        self.trail: list[tuple[float, float]] = []  # fixed frame에서 지나온 자리
        self._trail_frame = ""
        self._trail_np: NodePath | None = None

    def update(self, lookup: Callable[[str], TransformStamped | None]) -> None:
        viz, node = self.viz, self.viz.node
        for topic, (colors, z) in GRIDS.items():
            grid = node.latest(topic)
            root = self.grids[topic]
            if isinstance(grid, OccupancyGrid) and grid is not self._grid_drawn.get(topic):
                self._grid_drawn[topic] = grid
                root.getChildren().detach()
                card = grid_card(grid, colors, z)
                if card is not None:
                    card.reparentTo(root)
            self._place(root, grid.header.frame_id if isinstance(grid, OccupancyGrid) and viz.enabled[topic] else None, lookup)
        for topic, (color, thickness, dots) in PATHS.items():
            path = node.latest(topic)
            root = self.paths[topic]
            if isinstance(path, Path) and path is not self._path_drawn.get(topic):
                self._path_drawn[topic] = path
                root.getChildren().detach()
                z = 0.05 if topic == "local_plan" else 0.03  # local_plan이 plan 위에
                poses: list[tuple[float, ...]] = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
                if topic == "plan":
                    poses = [(p.pose.position.x, p.pose.position.y, _yaw(p.pose.orientation)) for p in path.poses]
                _line(poses, color, thickness, z, dots).reparentTo(root)
            self._place(root, path.header.frame_id if isinstance(path, Path) and path.poses and viz.enabled[topic] else None, lookup)
        self._update_trail(lookup)

    @staticmethod
    def _place(root: NodePath, frame: str | None, lookup: Callable[[str], TransformStamped | None]) -> None:
        stamped = lookup(frame) if frame else None
        if stamped is None:
            root.hide()
            return
        t, q = stamped.transform.translation, stamped.transform.rotation
        root.setPos(t.x, t.y, t.z)
        root.setQuat(LQuaternionf(q.w, q.x, q.y, q.z))
        root.show()

    def _update_trail(self, lookup: Callable[[str], TransformStamped | None]) -> None:
        """로봇(base_footprint)이 fixed frame에서 지나온 자리. fixed frame을 바꾸면 처음부터 다시 쌓는다."""
        viz = self.viz
        if viz.fixed_frame != self._trail_frame:
            self.trail, self._trail_frame = [], viz.fixed_frame
        stamped = lookup("base_footprint")
        if stamped is not None:
            t = stamped.transform.translation
            if not self.trail or math.dist(self.trail[-1], (t.x, t.y)) >= TRAIL_STEP:
                self.trail.append((t.x, t.y))
                del self.trail[:-TRAIL_LIMIT]
                if self._trail_np is not None:
                    self._trail_np.removeNode()
                self._trail_np = _line(list(self.trail), TRAIL_COLOR, 2.0, 0.02)
                self._trail_np.reparentTo(viz.scene)
        if self._trail_np is not None:
            self._trail_np.show() if viz.enabled["trail"] else self._trail_np.hide()

    def clear_trail(self) -> None:
        self.trail = []
        if self._trail_np is not None:
            self._trail_np.removeNode()
            self._trail_np = None


__all__ = ["GRIDS", "MAP_COLORS", "NavDisplay", "PATHS", "grid_card"]
