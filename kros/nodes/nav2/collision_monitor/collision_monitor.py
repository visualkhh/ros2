import math
import threading
from collections.abc import Mapping

from kros.context import Context
from kros.msgs.geometry_msgs import Twist, Vector3
from kros.msgs.sensor_msgs import LaserScan
from kros.msgs.std_srvs import SetBool, SetBoolRequest, SetBoolResponse
from kros.nodes.node import Node
from kros.parameter import ParameterValue
from kros.tf import Buffer, TransformException, TransformListener
from kros.topic import QOS_COMMAND

Point = tuple[float, float]
ACTIONS = ("stop", "slowdown", "limit", "approach")
# TurtleBot3(반지름 0.2m)에 맞춘 기본 영역: 앞 0.3m 상자 안이면 정지, 0.5m 상자 안이면 감속 (base_footprint 좌표, x 앞, y 왼쪽)
DEFAULT_POLYGONS: dict[str, dict[str, ParameterValue]] = {
    "PolygonStop": {"type": "polygon", "points": (0.3, 0.15, 0.3, -0.15, 0.0, -0.15, 0.0, 0.15), "action_type": "stop"},
    "PolygonSlow": {"type": "polygon", "points": (0.5, 0.2, 0.5, -0.2, 0.0, -0.2, 0.0, 0.2), "action_type": "slowdown"},
}


def inside(polygon: list[Point], x: float, y: float) -> bool:
    """점이 다각형 안인가 (반직선 교차 수)."""
    hit = False
    for (x1, y1), (x2, y2) in zip(polygon, polygon[1:] + polygon[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


class Polygon:
    """collision_monitor의 영역 하나 (polygon 또는 circle)와 그 동작."""

    def __init__(self, node: Node, name: str) -> None:
        default = DEFAULT_POLYGONS.get(name, {})
        positive = lambda value: value > 0  # noqa: E731
        self.name = name
        self.type = node.declare_parameter(f"{name}.type", str(default.get("type", "circle")), validator=lambda value: value in ("polygon", "circle"))
        default_points = default.get("points")
        points = node.declare_parameter(
            f"{name}.points", default_points if isinstance(default_points, tuple) else (0.0,) * 6, validator=lambda value: len(value) >= 6 and len(value) % 2 == 0
        )  # x1, y1, x2, y2, ... (base 좌표, 꼭짓점 3개 이상)
        flat = [float(v) for v in points.value]
        self.points = list(zip(flat[0::2], flat[1::2]))
        self.radius = node.declare_parameter(f"{name}.radius", 0.3, validator=positive)
        self.action_type = node.declare_parameter(f"{name}.action_type", str(default.get("action_type", "stop")), validator=lambda value: value in ACTIONS)
        self.min_points = node.declare_parameter(f"{name}.min_points", 4, validator=positive)
        self.slowdown_ratio = node.declare_parameter(f"{name}.slowdown_ratio", 0.3, validator=lambda value: 0 <= value <= 1)
        self.linear_limit = node.declare_parameter(f"{name}.linear_limit", 0.1, validator=lambda value: value >= 0)
        self.angular_limit = node.declare_parameter(f"{name}.angular_limit", 0.5, validator=lambda value: value >= 0)
        self.time_before_collision = node.declare_parameter(f"{name}.time_before_collision", 2.0, validator=positive)
        self.simulation_time_step = node.declare_parameter(f"{name}.simulation_time_step", 0.1, validator=positive)
        self.enabled = node.declare_parameter(f"{name}.enabled", True)

    def contains(self, x: float, y: float) -> bool:
        if self.type.value == "circle":
            return math.hypot(x, y) <= self.radius.value
        return inside(self.points, x, y)

    def hits(self, points: list[Point]) -> bool:
        return sum(1 for x, y in points if self.contains(x, y)) >= self.min_points.value

    def apply(self, v: float, w: float, points: list[Point]) -> tuple[float, float]:
        """이 영역의 동작을 적용한 (v, w)."""
        action = self.action_type.value
        if action == "approach":  # 이대로 굴러가면 언제 영역에 점이 들어오나: time_before_collision보다 빠르면 그 비율만큼 늦춘다
            step, horizon = self.simulation_time_step.value, self.time_before_collision.value
            x = y = yaw = 0.0
            t = 0.0
            while t < horizon:
                if t > 0:
                    c, s = math.cos(-yaw), math.sin(-yaw)
                    moved = [(c * (px - x) - s * (py - y), s * (px - x) + c * (py - y)) for px, py in points]  # 굴러간 로봇에서 본 점들
                else:
                    moved = points
                if self.hits(moved):
                    ratio = t / horizon
                    return v * ratio, w * ratio
                t += step
                yaw += w * step
                x, y = x + v * math.cos(yaw) * step, y + v * math.sin(yaw) * step
            return v, w
        if not self.hits(points):
            return v, w
        if action == "stop":
            return 0.0, 0.0
        if action == "slowdown":
            return v * self.slowdown_ratio.value, w * self.slowdown_ratio.value
        lv, la = self.linear_limit.value, self.angular_limit.value  # limit
        return max(-lv, min(lv, v)), max(-la, min(la, w))


class CollisionMonitor(Node):
    """nav2 collision_monitor 흉내: cmd_vel_in_topic의 속도 명령을 라이다 점(observation_sources)과 로봇 둘레 영역(polygons)으로 검사해 cmd_vel_out_topic으로 낸다.
    costmap/경로 계획과 따로 도는 마지막 안전장치라 지도에 없는 장애물에도 반응한다.

    파라미터 (nav2와 같은 이름): base_frame_id, cmd_vel_in_topic, cmd_vel_out_topic, source_timeout (라이다가 이보다 오래 안 오면 멈춘다),
    polygons (영역 이름들), observation_sources (기본 ("scan",)), "<영역>.type/points/radius/action_type/min_points/slowdown_ratio/linear_limit/
    angular_limit/time_before_collision/simulation_time_step/enabled", "<소스>.topic".
    동작: stop(정지), slowdown(slowdown_ratio 배), limit(linear_limit/angular_limit로 자름), approach(이대로 가면 time_before_collision 안에 부딪히면 그만큼 늦춤).
    여러 영역이 걸리면 가장 느린 결과를 쓴다. shortcut: 소스는 LaserScan만, 영역은 base 좌표에 고정 (nav2의 동적 footprint 영역 없음).
    """

    def __init__(self, context: Context, name: str | None = None, parameters: Mapping[str, ParameterValue] | None = None) -> None:
        super().__init__(context, name or "collision_monitor", parameters)
        self.base_frame_id = self.declare_parameter("base_frame_id", "base_footprint")
        self.cmd_vel_in_topic = self.declare_parameter("cmd_vel_in_topic", "cmd_vel_smoothed")
        self.cmd_vel_out_topic = self.declare_parameter("cmd_vel_out_topic", "cmd_vel")
        self.source_timeout = self.declare_parameter("source_timeout", 2.0, validator=lambda value: value > 0)
        self.polygon_names = self.declare_parameter("polygons", tuple(DEFAULT_POLYGONS))
        self.observation_sources = self.declare_parameter("observation_sources", ("scan",))
        # 수동 텔레옵 중에는 충돌 검사를 끈다 (사람이 직접 조종; 자율주행 때만 감시).
        self.enabled = self.declare_parameter("enabled", True)  # Viz 버튼/서비스로 전체 충돌 감시 on/off
        self.ignore_collision_during_teleop = self.declare_parameter("ignore_collision_during_teleop", False)
        self.teleop_topic = self.declare_parameter("teleop_topic", "cmd_vel_teleop")
        self.teleop_timeout = self.declare_parameter("teleop_timeout", 0.5, validator=lambda value: value > 0)
        # 앞 장애물에 막혔을 때 후진으로 빠져나오는 것은 허용한다 (0이면 막는다). 후진은 비율만큼만.
        self.reverse_escape_ratio = self.declare_parameter("reverse_escape_ratio", 0.3, validator=lambda value: 0 <= value <= 1)
        # 제자리 회전: 가장 가까운 점이 로봇 반지름(0.22m) 밖이면 돌 수 있다 (벽 앞에서도 방향을 돌릴 수 있게).
        self.rotation_clearance = self.declare_parameter("rotation_clearance", 0.22, validator=lambda value: value >= 0)
        self.rotation_escape_ratio = self.declare_parameter("rotation_escape_ratio", 0.5, validator=lambda value: 0 <= value <= 1)
        self.polygons = [Polygon(self, str(name)) for name in self.polygon_names.value]
        self.buffer = Buffer()
        self._listener = TransformListener(self.buffer, self)
        self._lock = threading.Lock()
        self._points: dict[str, tuple[float, list[Point]]] = {}  # 소스 -> (측정 시각, base 좌표의 점들)
        self.state = ""  # 지난 명령에 걸린 영역과 동작 (시험/화면용)
        self._last_teleop = float("-inf")  # 아직 텔레옵 없음 (시각 0인 시뮬에서도 바로 켜지지 않게)
        for source in self.observation_sources.value:
            topic = self.declare_parameter(f"{source}.topic", str(source))
            self.subscribe(topic.value, LaserScan, lambda scan, key=str(source): self._on_scan(key, scan))  # type: ignore[misc]
        self.subscribe(self.cmd_vel_in_topic.value, Twist, self._on_cmd_vel, QOS_COMMAND)
        self.subscribe(self.teleop_topic.value, Twist, self._on_teleop, QOS_COMMAND)
        self.create_service(f"{self.name}/set_enabled", SetBool, self._on_set_enabled)

    def _on_scan(self, source: str, scan: LaserScan) -> None:
        try:  # 센서 -> base (라이다는 로봇에 고정이라 최신 TF면 된다)
            t = self.buffer.lookup_transform(self.base_frame_id.value, scan.header.frame_id).transform
        except TransformException:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        points = []
        for i, r in enumerate(scan.ranges):
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                a = scan.angle_min + i * scan.angle_increment
                px, py = r * math.cos(a), r * math.sin(a)
                points.append((t.translation.x + c * px - s * py, t.translation.y + s * px + c * py))
        with self._lock:
            self._points[source] = (scan.header.stamp, points)

    def check(self, v: float, w: float) -> tuple[float, float]:
        """명령 (v, w)를 검사한 결과. 소스가 하나라도 없거나 오래됐으면 멈춘다."""
        now = self.now()
        with self._lock:
            sources = dict(self._points)
        if len(sources) < len(self.observation_sources.value) or any(now - stamp > self.source_timeout.value for stamp, _ in sources.values()):
            self.state = "no fresh observation: stop"
            return 0.0, 0.0
        points = [p for _, ps in sources.values() for p in ps]
        best, self.state = (v, w), ""
        for polygon in self.polygons:
            if not polygon.enabled.value:
                continue
            result = polygon.apply(v, w, points)
            front = bool(polygon.points) and sum(px for px, _ in polygon.points) > 0.0
            if result == (0.0, 0.0) and polygon.action_type.value == "stop":
                if v < 0.0 and front:
                    # 앞이 막혀도 뒤로는 빠져나올 수 있게 (벽에 갇히지 않게). 아주 느리게.
                    result = (v * self.reverse_escape_ratio.value, w * self.reverse_escape_ratio.value)
                elif abs(v) < 1e-6 and w != 0.0:
                    nearest = min((math.hypot(px, py) for px, py in points), default=float("inf"))
                    if nearest >= self.rotation_clearance.value:
                        # 로봇 몸(반지름) 밖에만 있으면 제자리 회전은 안전하다
                        result = (0.0, w * self.rotation_escape_ratio.value)
            if (abs(result[0]), abs(result[1])) < (abs(best[0]), abs(best[1])):
                best, self.state = result, f"{polygon.name}: {polygon.action_type.value}"
        return best

    def _on_set_enabled(self, request: SetBoolRequest) -> SetBoolResponse:
        self.enabled.set(request.data)
        self.get_logger().info("collision monitor: %s", "enabled" if request.data else "disabled")
        return SetBoolResponse(True, "collision monitor enabled" if request.data else "collision monitor disabled")

    def _on_teleop(self, twist: Twist) -> None:
        self._last_teleop = self.now()

    def _on_cmd_vel(self, twist: Twist) -> None:
        if not self.enabled.value:
            self.state = "collision monitor disabled"
            self.publish(self.cmd_vel_out_topic.value, twist)
            return
        if self.ignore_collision_during_teleop.value and self.now() - self._last_teleop <= self.teleop_timeout.value:
            self.state = "teleop: collision check off"  # 수동 조종 중: 그대로 보낸다
            self.publish(self.cmd_vel_out_topic.value, twist)
            return
        v, w = self.check(twist.linear.x, twist.angular.z)
        self.publish(self.cmd_vel_out_topic.value, Twist(Vector3(x=v), Vector3(z=w)))
