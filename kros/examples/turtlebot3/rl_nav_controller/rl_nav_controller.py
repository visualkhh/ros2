"""TurtleBot3 예제의 controller 플러그인 (ros2_ws/src/rl_nav_controller의 kros판): isaac/training/rl_maze에서 학습한 정책으로 경로를 따라간다.
관측 · 행동은 실기 C++ 플러그인(rl_nav_controller.cpp)과 같은 약속 = 학습 env (바꾸면 같이):
  관측 52 = [다음 웨이포인트 lookahead(5)개 로봇 기준 (x, y) / wp_scale, 라이다 lidar_rays(36)방향 (r - lidar_inflation) / lidar_max,
             v / max_lin, w / max_ang, 자이로 w / max_ang, 이전 행동 2, lidar_inflation / inflation_scale (52차원 정책만)]
  행동 [-1, 1]^2 -> v = a0 * max_lin (후진은 v_min_ratio까지), w = a1 * max_ang
안에 fallback controller(기본 MPPI)를 품고 있어서 enable이 거짓이거나 모델/스캔이 없으면 그것으로 돈다.
"""
import math
from pathlib import Path

from kros.diagnostics import DiagnosticStatusWrapper
from kros.msgs.diagnostic_msgs import DiagnosticLevel
from kros.msgs.sensor_msgs import Imu, LaserScan
from kros.nodes.nav2.controller.controller import Controller, Pose2, Velocity, closest_index, simulate
from kros.nodes.nav2.controller.dwb import DWBLocalPlanner
from kros.nodes.nav2.controller.mppi import MPPIController
from kros.nodes.nav2.controller.regulated_pure_pursuit import RegulatedPurePursuitController
from kros.nodes.nav2.costmap_2d import Costmap
from kros.nodes.node import Node
from kros.tf import Buffer, TransformException, TransformListener

from .policy import OnnxPolicy

FALLBACKS: dict[str, type[Controller]] = {"mppi": MPPIController, "dwb": DWBLocalPlanner, "rpp": RegulatedPurePursuitController}


class RlNavController(Controller):
    """파라미터 ("<이름>." 아래, 실기와 같은 이름): enable (정책을 쓸지, 실행 중에 바꿀 수 있다), model_path (policy.onnx), fallback_controller (mppi/dwb/rpp,
    설정은 "<이름>.inner_<fallback>." 아래), max_lin, max_ang, v_min_ratio, lookahead, wp_step, wp_scale, lidar_rays, lidar_max,
    lidar_inflation (라이다 안전거리 m: 학습 범위에 맞춘다), inflation_scale, scan_topic, imu_topic, base_frame.
    shortcut: 실기의 toggle 서비스/status 토픽은 없다 (enable 파라미터를 바꾼다)."""

    def configure(self, node: Node, name: str) -> None:
        super().configure(node, name)
        positive = lambda value: value > 0  # noqa: E731
        self.enable = node.declare_parameter(f"{name}.enable", False)
        self.model_path = node.declare_parameter(f"{name}.model_path", "")
        self.fallback_controller = node.declare_parameter(f"{name}.fallback_controller", "mppi", validator=lambda value: value in FALLBACKS)
        self.max_lin = node.declare_parameter(f"{name}.max_lin", 0.26, validator=positive)
        self.max_ang = node.declare_parameter(f"{name}.max_ang", 1.82, validator=positive)
        self.v_min_ratio = node.declare_parameter(f"{name}.v_min_ratio", -0.2, validator=lambda value: value <= 0)
        self.lookahead = node.declare_parameter(f"{name}.lookahead", 5, validator=positive)
        self.wp_step = node.declare_parameter(f"{name}.wp_step", 0.30, validator=positive)
        self.wp_scale = node.declare_parameter(f"{name}.wp_scale", 2.0, validator=positive)
        self.lidar_rays = node.declare_parameter(f"{name}.lidar_rays", 36, validator=positive)
        self.lidar_max = node.declare_parameter(f"{name}.lidar_max", 3.5, validator=positive)
        self.lidar_inflation = node.declare_parameter(f"{name}.lidar_inflation", 0.22, validator=lambda value: value >= 0)
        self.inflation_scale = node.declare_parameter(f"{name}.inflation_scale", 0.5, validator=positive)
        self.base_frame = node.declare_parameter(f"{name}.base_frame", "base_footprint")
        scan_topic = node.declare_parameter(f"{name}.scan_topic", "scan")
        imu_topic = node.declare_parameter(f"{name}.imu_topic", "imu")
        kind = self.fallback_controller.value
        self.inner = FALLBACKS[kind]()
        self.inner.configure(node, f"{name}.inner_{kind}")
        self.policy: OnnxPolicy | None = None
        if self.model_path.value:
            try:
                self.policy = OnnxPolicy(Path(self.model_path.value).expanduser())
                node.get_logger().info("정책을 읽었다: %s (관측 %d)", self.model_path.value, self.policy.inputs)
            except Exception as error:  # noqa: BLE001  onnxruntime 없음/파일 없음/깨진 파일: 정책 없이 fallback으로 돈다 (실기와 같다)
                node.get_logger().error("정책을 못 읽었다 (%s): %s — fallback(%s)으로 돈다", self.model_path.value, error, kind)
        self.scan: LaserScan | None = None
        self.gyro: float | None = None
        self.prev_action = (0.0, 0.0)
        self.active = False  # 지난 주기에 정책이 냈나 (아니면 fallback)
        self.policy_steps = 0  # 정책이 명령을 낸 횟수
        self._laser_yaw: float | None = None
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, node)
        node.subscribe(scan_topic.value, LaserScan, self._on_scan)
        node.subscribe(imu_topic.value, Imu, self._on_imu)

    def _on_scan(self, scan: LaserScan) -> None:
        self.scan = scan

    def _on_imu(self, imu: Imu) -> None:
        self.gyro = imu.angular_velocity.z

    def diagnostic_status(self, status: DiagnosticStatusWrapper) -> None:
        """diagnostics: 어느 플러그인이고, 정책이 읽혔는지, 지난 주기에 정책이 냈는지 (viz 왼쪽 위 한 줄)."""
        status.add("plugin", "rl_nav_controller::RlNavController")
        status.add("enable", self.enable.value)
        status.add("policy", "loaded" if self.policy is not None else "missing")
        status.add("active", "policy" if self.active else "fallback")
        status.add("policy_steps", self.policy_steps)
        status.add("fallback", self.fallback_controller.value)
        status.add("model", self.model_path.value or "-")
        if not self.enable.value:
            status.summary(DiagnosticLevel.OK, f"disabled (fallback {self.fallback_controller.value})")
        elif self.policy is None:
            status.summary(DiagnosticLevel.ERROR, f"policy not loaded (fallback {self.fallback_controller.value})")
        elif self.active:
            status.summary(DiagnosticLevel.OK, "RL policy driving")
        elif self.policy_steps == 0:
            status.summary(DiagnosticLevel.OK, "ready (no command yet)")
        else:
            status.summary(DiagnosticLevel.WARN, f"last command from fallback {self.fallback_controller.value}")

    def compute_velocity_commands(self, pose: Pose2, velocity: Velocity, path: list[Pose2], costmap: Costmap, dt: float) -> Velocity:
        scan = self.scan
        if not (self.enable.value and self.policy is not None and scan is not None):
            self.active = False
            command = self.inner.compute_velocity_commands(pose, velocity, path, costmap, dt)
            self.local_plan = self.inner.local_plan
            return command
        if self._laser_yaw is None:  # 라이다가 로봇 기준으로 얼마나 돌아 있는지 (한 번)
            try:
                q = self._buffer.lookup_transform(self.base_frame.value, scan.header.frame_id).transform.rotation
                self._laser_yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            except TransformException:
                pass  # 아직 모르면 0도로 (실기와 같다)
        inflation = self.lidar_inflation.value
        max_lin, max_ang = self.max_lin.value, self.max_ang.value
        obs = self.waypoints(pose, path) + self.scan_rays(scan, self._laser_yaw or 0.0, inflation)
        obs += [velocity[0] / max_lin, velocity[1] / max_ang, (velocity[1] if self.gyro is None else self.gyro) / max_ang, *self.prev_action]
        if self.policy.inputs >= 52:
            obs.append(inflation / self.inflation_scale.value)  # inflation을 입력으로 받는 정책
        action = self.policy.run(obs)
        a0, a1 = max(-1.0, min(1.0, action[0])), max(-1.0, min(1.0, action[1]))
        self.prev_action, self.active = (a0, a1), True
        self.policy_steps += 1
        v, w = max(a0 * max_lin, self.v_min_ratio.value * max_lin), a1 * max_ang
        self.local_plan = simulate(pose, v, w, 1.0, 0.1)  # 화면용: 이 명령으로 1초
        return v, w

    def waypoints(self, pose: Pose2, path: list[Pose2]) -> list[float]:
        """경로에서 로봇에 가장 가까운 점부터 wp_step 간격으로 앞의 lookahead개 -> 로봇 기준 (x 앞, y 왼쪽) / wp_scale. 모자라면 마지막 점."""
        k_max, step, scale = self.lookahead.value, self.wp_step.value, self.wp_scale.value
        px, py, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        out: list[float] = []

        def emit(x: float, y: float) -> None:
            dx, dy = x - px, y - py
            out.extend(((c * dx + s * dy) / scale, (-s * dx + c * dy) / scale))

        acc, want = 0.0, step
        i = closest_index(pose, path)
        while len(out) < 2 * k_max and i + 1 < len(path):
            (ax, ay, _), (bx, by, _) = path[i], path[i + 1]
            seg = math.hypot(bx - ax, by - ay)
            while len(out) < 2 * k_max and acc + seg >= want:
                u = (want - acc) / seg if seg > 1e-9 else 0.0
                emit(ax + u * (bx - ax), ay + u * (by - ay))
                want += step
            acc += seg
            i += 1
        while len(out) < 2 * k_max:
            emit(path[-1][0], path[-1][1])
        return out

    def scan_rays(self, scan: LaserScan, laser_yaw: float, inflation: float) -> list[float]:
        """로봇 기준 0도부터 반시계로 lidar_rays방향: 각 방향의 가장 가까운 빔(+-1) 거리 - inflation, 없으면 lidar_max -> / lidar_max."""
        rays, lidar_max = self.lidar_rays.value, self.lidar_max.value
        n = len(scan.ranges)
        if n == 0 or scan.angle_increment == 0.0:
            return [1.0] * rays
        out = []
        for r in range(rays):
            a = (2.0 * math.pi * r / rays - laser_yaw - scan.angle_min) % (2.0 * math.pi)
            idx = round(a / scan.angle_increment)
            best = lidar_max
            for j in (idx - 1, idx, idx + 1):
                v = scan.ranges[j % n]
                if math.isfinite(v) and scan.range_min <= v <= scan.range_max:
                    best = min(best, v)
            out.append(min(max(best - max(0.0, inflation), 0.0), lidar_max) / lidar_max)
        return out
