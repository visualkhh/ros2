import math
import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.nav_msgs import OccupancyGrid
from kros.msgs.std_msgs import Header
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import Buffer, TransformException, TransformListener
from kros.topic import QOS_LATCHED

from .costmap import Costmap


class Costmap2DROS(Node):
    """nav2_costmap_2d::Costmap2DROS 흉내: 지도(static layer)에 벽 둘레 비용(inflation layer)을 입힌 costmap을 들고 있다.
    nav2처럼 planner_server가 "global_costmap"을 자기 안에 만들어 쓴다. <노드 이름>/costmap 토픽(OccupancyGrid, latch)으로도 낸다.

    파라미터 (nav2와 같은 이름): global_frame ("map"), robot_base_frame ("base_footprint"), robot_radius, inflation_radius,
    cost_scaling_factor, track_unknown_space, map_topic. 반지름/inflation 값을 바꾸면 바로 다시 만든다.
    shortcut: obstacle layer(라이다로 지도에 없는 장애물 칠하기)는 없다, 움직이는 장애물을 피해야 할 때 추가.
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "global_costmap", parameters)
        positive = lambda value: value > 0  # noqa: E731
        rebuild = lambda _value: self._rebuild()  # noqa: E731
        self.global_frame = self.declare_parameter("global_frame", "map")
        self.robot_base_frame = self.declare_parameter("robot_base_frame", "base_footprint")
        self.robot_radius = self.declare_parameter("robot_radius", 0.1, validator=positive, on_change=rebuild)
        self.inflation_radius = self.declare_parameter("inflation_radius", 0.55, validator=positive, on_change=rebuild)
        self.cost_scaling_factor = self.declare_parameter("cost_scaling_factor", 3.0, validator=positive, on_change=rebuild)
        self.track_unknown_space = self.declare_parameter("track_unknown_space", True, on_change=rebuild)
        self.map_topic = self.declare_parameter("map_topic", "map")
        self._lock = threading.Lock()
        self._map: OccupancyGrid | None = None
        self._costmap: Costmap | None = None
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self.subscribe(self.map_topic.value, OccupancyGrid, self._on_map, QOS_LATCHED)

    @property
    def costmap(self) -> Costmap | None:
        """지금 costmap (새 지도가 오면 통째로 바뀐다: 받은 것은 고치지 말고 읽기만 한다). 지도가 아직 없으면 None."""
        with self._lock:
            return self._costmap

    def _on_map(self, grid: OccupancyGrid) -> None:
        with self._lock:
            self._map = grid
        self._rebuild()

    def _rebuild(self) -> None:
        with self._lock:
            grid = self._map
        if grid is None:
            return
        costmap = Costmap.from_occupancy_grid(grid, self.track_unknown_space.value)
        costmap.inflate(self.robot_radius.value, self.inflation_radius.value, self.cost_scaling_factor.value)
        with self._lock:
            if grid is not self._map:
                return  # 만드는 동안 더 새 지도가 왔다: 그쪽이 다시 만든다
            self._costmap = costmap
        self.publish(f"{self.name}/costmap", costmap.to_message(Header(self.now(), self.global_frame.value)), QOS_LATCHED)

    def get_robot_pose(self) -> tuple[float, float, float] | None:
        """global_frame에서 본 로봇 (x, y, yaw). TF가 아직 없으면 None."""
        try:
            t = self.buffer.lookup_transform(self.global_frame.value, self.robot_base_frame.value).transform
        except TransformException:
            return None
        q = t.rotation
        return t.translation.x, t.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
