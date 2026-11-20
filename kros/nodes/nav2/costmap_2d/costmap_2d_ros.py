import math
import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.geometry_msgs import Quaternion
from kros.msgs.nav_msgs import OccupancyGrid
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import Buffer, TransformException, TransformListener
from kros.topic import QOS_LATCHED

from .costmap import FREE_SPACE, LETHAL_OBSTACLE, Costmap

LAYERS = ("static_layer", "obstacle_layer", "inflation_layer")
Cell = tuple[int, int]


def _yaw(q: Quaternion) -> float:
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Costmap2DROS(Node):
    """nav2_costmap_2d::Costmap2DROS 흉내: 레이어를 겹쳐 만든 costmap을 들고 <노드 이름>/costmap(OccupancyGrid, latch)으로도 낸다.
    nav2처럼 planner_server가 "global_costmap"을, controller_server가 "local_costmap"을 자기 안에 만든다.

    plugins (nav2와 같은 레이어 이름, 이 순서로 겹친다):
      static_layer    map_topic의 지도 (벽/빈 곳/모름)
      obstacle_layer  observation_sources의 라이다: 끝점(obstacle_max_range 안)을 벽으로 칠하고, 레이가 지나간 칸(raytrace_max_range 안)은 비운다.
                      칠한 장애물은 global_frame에 기억해 둔다 (창이 움직여도 남고, 다시 비춰서 비면 지워진다). 지도에 없는 장애물을 피하게 해 준다.
      inflation_layer 벽 둘레를 robot_radius/inflation_radius/cost_scaling_factor로 비싸게
    rolling_window: 참이면 지도 없이 로봇을 가운데 둔 width x height(m) 창이 따라다닌다 (local costmap). 거짓이면 지도 크기 그대로 (global costmap).
    update_frequency마다 다시 만든다 (static만이면 지도가 바뀔 때만). 반지름/inflation 값을 바꾸면 바로 다시 만든다.
    그 밖 파라미터 (nav2와 같은 이름): global_frame, robot_base_frame, track_unknown_space, resolution(rolling window 칸), "<소스>.topic".
    shortcut: rolling window에 static_layer를 겹치는 것은 지도와 global_frame이 같을 때만 (local은 보통 obstacle만 쓴다).
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "global_costmap", parameters)
        positive = lambda value: value > 0  # noqa: E731
        rebuild = lambda _value: self._rebuild()  # noqa: E731
        self.global_frame = self.declare_parameter("global_frame", "map")
        self.robot_base_frame = self.declare_parameter("robot_base_frame", "base_footprint")
        self.plugins = self.declare_parameter("plugins", ("static_layer", "inflation_layer"), validator=lambda names: all(n in LAYERS for n in names))
        self.robot_radius = self.declare_parameter("robot_radius", 0.1, validator=positive, on_change=rebuild)
        self.inflation_radius = self.declare_parameter("inflation_radius", 0.55, validator=positive, on_change=rebuild)
        self.cost_scaling_factor = self.declare_parameter("cost_scaling_factor", 3.0, validator=positive, on_change=rebuild)
        self.track_unknown_space = self.declare_parameter("track_unknown_space", True, on_change=rebuild)
        self.map_topic = self.declare_parameter("map_topic", "map")
        self.rolling_window = self.declare_parameter("rolling_window", False)
        self.width = self.declare_parameter("width", 3.0, validator=positive)
        self.height = self.declare_parameter("height", 3.0, validator=positive)
        self.resolution = self.declare_parameter("resolution", 0.05, validator=positive)
        self.update_frequency = self.declare_parameter("update_frequency", 5.0, validator=positive)
        self.observation_sources = self.declare_parameter("observation_sources", ("scan",))
        self.obstacle_max_range = self.declare_parameter("obstacle_max_range", 2.5, validator=positive)
        self.raytrace_max_range = self.declare_parameter("raytrace_max_range", 3.0, validator=positive)
        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._costmap: Costmap | None = None
        self._obstacles: set[Cell] = set()  # obstacle_layer가 칠한 칸 (global_frame, resolution 칸)
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        layers = self.plugins.value
        if "static_layer" in layers:
            self.subscribe(self.map_topic.value, OccupancyGrid, self._on_map, QOS_LATCHED)
        if "obstacle_layer" in layers:
            for source in self.observation_sources.value:
                topic = self.declare_parameter(f"{source}.topic", str(source))
                self.subscribe(topic.value, LaserScan, self._on_scan)
        if self.rolling_window.value or "obstacle_layer" in layers:  # 로봇이 움직이거나 장애물이 바뀐다: 주기마다 다시 만든다
            self.create_timer(1.0 / self.update_frequency.value, self._rebuild)

    @property
    def costmap(self) -> Costmap | None:
        """지금 costmap (다시 만들면 통째로 바뀐다: 받은 것은 고치지 말고 읽기만 한다). 아직 없으면 None."""
        with self._lock:
            return self._costmap

    def _on_map(self, grid: OccupancyGrid) -> None:
        empty = grid.info.width == 0 or grid.info.height == 0  # map_server가 지도를 내렸다
        with self._lock:
            self._map = None if empty else grid
            if empty and not self.rolling_window.value:
                self._costmap = None  # 지도 크기의 costmap은 지도가 없으면 없다
        if empty and not self.rolling_window.value:
            self.publish(f"{self.name}/costmap", OccupancyGrid(Header(self.now(), self.global_frame.value)), QOS_LATCHED)
            return
        self._rebuild()

    # ---- obstacle_layer --------------------------------------------------------------------------------

    def _on_scan(self, scan: LaserScan) -> None:
        try:  # 스캔을 잰 순간의 센서 자리 (global_frame)
            t = self.buffer.lookup_transform(self.global_frame.value, scan.header.frame_id, scan.header.stamp).transform
        except TransformException:
            try:  # 그 시각의 TF가 아직 없다 (스캔이 TF보다 먼저 왔다): 최신 TF로. shortcut: nav2는 TF가 올 때까지 기다린다(message filter)
                t = self.buffer.lookup_transform(self.global_frame.value, scan.header.frame_id).transform
            except TransformException:
                return
        ox, oy, yaw = t.translation.x, t.translation.y, _yaw(t.rotation)
        res = self.resolution.value if self.rolling_window.value else (self._map.info.resolution if self._map else self.resolution.value)
        mark_range, clear_range = self.obstacle_max_range.value, self.raytrace_max_range.value
        origin = (math.floor(ox / res), math.floor(oy / res))
        marks: set[Cell] = set()
        clears: set[Cell] = set()
        for i, r in enumerate(scan.ranges):
            a = yaw + scan.angle_min + i * scan.angle_increment
            hit = math.isfinite(r) and scan.range_min <= r <= scan.range_max
            reach = min(r if hit else scan.range_max, clear_range)  # 측정이 없으면(inf) 최대 거리까지는 비어 있다
            end = (math.floor((ox + reach * math.cos(a)) / res), math.floor((oy + reach * math.sin(a)) / res))
            clears.update(_line(origin, end))
            if hit and r <= mark_range:
                marks.add((math.floor((ox + r * math.cos(a)) / res), math.floor((oy + r * math.sin(a)) / res)))
        with self._lock:
            self._obstacles -= clears - marks
            self._obstacles |= marks

    # ---- 만들기 -----------------------------------------------------------------------------------------

    def _rebuild(self) -> None:
        layers = self.plugins.value
        with self._lock:
            grid, obstacles = self._map, set(self._obstacles)
        if self.rolling_window.value:
            pose = self.get_robot_pose()
            if pose is None:
                return
            res = self.resolution.value
            w, h = round(self.width.value / res), round(self.height.value / res)
            ox, oy = math.floor((pose[0] - self.width.value / 2) / res) * res, math.floor((pose[1] - self.height.value / 2) / res) * res  # 칸에 맞춘 창 (칸이 미끄러지지 않게)
            costmap = Costmap(res, w, h, ox, oy)  # 모르는 곳도 빈 곳으로 (nav2 local costmap처럼)
            if "static_layer" in layers and grid is not None and grid.header.frame_id == self.global_frame.value:
                static = Costmap.from_occupancy_grid(grid, self.track_unknown_space.value)
                for j in range(h):
                    for i in range(w):
                        cell = static.world_to_map(*costmap.map_to_world(i, j))
                        costmap.costs[j * w + i] = static.cost(*cell) if cell else FREE_SPACE  # 지도 밖은 빈 곳
        elif "static_layer" in layers and grid is not None:
            costmap = Costmap.from_occupancy_grid(grid, self.track_unknown_space.value)
        else:
            return  # 지도도 창도 없으면 크기를 모른다
        if "obstacle_layer" in layers:
            base_i, base_j = round(costmap.origin_x / costmap.resolution), round(costmap.origin_y / costmap.resolution)
            for ci, cj in obstacles:
                i, j = ci - base_i, cj - base_j
                if 0 <= i < costmap.width and 0 <= j < costmap.height:
                    costmap.costs[j * costmap.width + i] = LETHAL_OBSTACLE
        if "inflation_layer" in layers:
            costmap.inflate(self.robot_radius.value, self.inflation_radius.value, self.cost_scaling_factor.value)
        with self._lock:
            if not self.rolling_window.value and grid is not self._map:
                return  # 만드는 동안 더 새 지도가 왔다: 그쪽이 다시 만든다
            self._costmap = costmap
        self.publish(f"{self.name}/costmap", costmap.to_message(Header(self.now(), self.global_frame.value)), QOS_LATCHED)

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        """global_frame에서 본 로봇 (x, y, yaw). TF가 아직 없으면 None."""
        try:
            t = self.buffer.lookup_transform(self.global_frame.value, self.robot_base_frame.value).transform
        except TransformException:
            return None
        return t.translation.x, t.translation.y, _yaw(t.rotation)


def _line(start: Cell, end: Cell) -> list[Cell]:
    """브레젠험 직선: start부터 end 앞까지의 칸 (레이가 지나간 칸)."""
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


