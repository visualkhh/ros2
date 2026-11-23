"""실기 TurtleBot3 실행 (kros, no simulator/no viz). 라즈베리파이에서 돌린다.

- 화면(Panda3D)도 월드도 없다. 실시간 루프가 OpenCR/LDS-03에 Clock을 내려준다.
- kros 노드는 시뮬레이터 예제와 같다: RobotBridge, TF, EKF, Cartographer(SLAM/위치추정), map_server/saver/catalog,
  Smac planner + MPPI controller, behavior_server, bt_navigator, twist_mux, collision_monitor.
- 네트워크: 0.0.0.0:9000을 연다. PC의 `kros/examples/turtlebot3/viz_remote.py <pi-ip> 9000`이 붙어서 화면을 본다.
  Viz의 W/A/S/D는 cmd_vel_teleop으로 들어와 twist_mux를 거쳐 바퀴로 간다 (SLAM할 때 수동 조종).

실행 (라즈베리파이):
  cd ~/ros2
  ./kos_ws/run.sh                    # 포트 9000
  KROS_SERVER_PORT=0 ./kos_ws/run.sh # 네트워크 없이
"""
import os
import signal
import socket
import sys
import threading
import time
from pathlib import Path

if not __package__:  # 파일로 직접 실행 (python kos_ws/main.py)
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from kros import Kros
from kros.context import Context
from kros.log import enable_console
from kros.msgs.geometry_msgs import Twist
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, MapCatalog, MapSaver, MapServer, Node, RobotStatePublisher
from kros.nodes.nav2.behavior_server import BehaviorServer
from kros.nodes.nav2.bt_navigator import BtNavigator
from kros.nodes.nav2.collision_monitor import CollisionMonitor
from kros.nodes.nav2.controller import ControllerServer
from kros.nodes.nav2.planner import PlannerServer
from kros.nodes.twist_mux import TwistMux
from kros.parameter import ParameterValue
from kros.worlds.clock import Clock

from kos_ws.robot import OPENCR_PORT, WHEEL_RADIUS, WHEEL_SEPARATION, RealTurtleBot3
from kos_ws.robot_bridge import RobotBridge

MAPS_DIRECTORY = Path(__file__).resolve().parent / "artifacts" / "maps"
LOOP_PERIOD = 0.02  # 50Hz: 하드웨어 tick (OpenCR 명령은 10Hz, 라이다는 10Hz, IMU는 20Hz)

# robot_localization의 *_config 15칸 (x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az) 중 켤 칸
ODOM_CONFIG = tuple(i in (6, 11) for i in range(15))  # 바퀴: vx, vyaw
IMU_CONFIG = tuple(i == 11 for i in range(15))  # 자이로: vyaw (방향은 편향이 쌓여 쓰지 않는다)


def create_nodes(ctx: Context, robot: RealTurtleBot3) -> list[Node]:
    """시뮬레이터 예제와 같은 구성. RL 정책은 쓰지 않는다 (FollowPath = MPPI)."""
    description = RobotStatePublisher(ctx, parameters={"robot_description": robot.urdf_path.read_text()})
    odometry = DiffDriveOdometry(
        ctx, parameters={"wheel_radius": WHEEL_RADIUS, "wheel_separation": WHEEL_SEPARATION, "publish_tf": False}
    )
    ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG})
    # Pi 4는 pure Python 스캔매칭이 무거워서 두 번째 스캔마다(5Hz)만 맞춘다 (TF는 계속 20Hz)
    slam = Cartographer(ctx, parameters={"mode": "idle", "match_every": 2, "num_matching_points": 40, "map_publish_period_sec": 2.0})
    maps = {"maps_directory": str(MAPS_DIRECTORY)}
    footprint = {"robot_radius": robot.radius, "inflation_radius": 0.3}
    planner = PlannerServer(ctx, parameters={"GridBased.plugin": "nav2_smac_planner::SmacPlannerHybrid"}, costmap_parameters={**footprint, "update_frequency": 1.0})
    controller_parameters: dict[str, ParameterValue] = {
        "odom_topic": "odometry/filtered",
        "cmd_vel_topic": "cmd_vel_nav",
        "FollowPath.plugin": "nav2_mppi_controller::MPPIController",
        "general_goal_checker.xy_goal_tolerance": 0.4,
        "general_goal_checker.yaw_goal_tolerance": 0.4,
        "progress_checker.movement_time_allowance": 2.0,
        "progress_checker.required_movement_radius": 0.2,
    }
    controller = ControllerServer(ctx, parameters=controller_parameters, costmap_parameters={**footprint, "update_frequency": 3.0})
    behavior = BehaviorServer(ctx)
    mux = TwistMux(ctx, parameters={"cmd_vel_out": "cmd_vel_mux"})
    # 0.5m 안이면 감속: 0.3은 너무 느려서 0.6 (정지 상자 0.3m는 그대로)
    monitor = CollisionMonitor(ctx, parameters={"cmd_vel_in_topic": "cmd_vel_mux", "cmd_vel_out_topic": "cmd_vel", "PolygonSlow.slowdown_ratio": 0.6})
    navigator = BtNavigator(ctx, parameters={"replanning_period": 3.0})
    return [
        RobotBridge(ctx, robot),
        description,
        odometry,
        ekf,
        slam,
        MapServer(ctx, parameters=maps),
        MapSaver(ctx, parameters=maps),
        MapCatalog(ctx, parameters=maps),
        planner,
        controller,
        behavior,
        navigator,
        mux,
        monitor,
    ]


def run_hardware(robot: RealTurtleBot3, stop: threading.Event) -> None:
    """실시간 루프: OpenCR/LDS-03에 Clock을 내려준다. OpenCR 명령(10Hz), 라이다(10Hz), IMU(20Hz)는 부품이 주기를 지킨다.

    시각은 kros의 SystemTimeSource.now()와 같은 time.time()을 쓴다 (센서 stamp와 진단의 data_age가 맞도록).
    """
    last = time.time()
    while not stop.is_set():
        now = time.time()
        dt = now - last
        if dt < LOOP_PERIOD:
            time.sleep(min(LOOP_PERIOD - dt, 0.005))
            continue
        last = now
        robot.tick(Clock(now, dt))


def _local_addresses() -> list[str]:
    addresses: set[str] = set()
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            addresses.add(str(info[4][0]))
    except OSError:
        pass
    try:  # 기본 경로로 나가는 주소 (mDNS로 이름이 안 풀릴 때)
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))
        addresses.add(str(probe.getsockname()[0]))
        probe.close()
    except OSError:
        pass
    return sorted(address for address in addresses if not address.startswith("127."))


def main() -> None:
    enable_console()
    robot = RealTurtleBot3(
        opencr_port=os.environ.get("OPENCR_PORT", OPENCR_PORT),
        lidar_port=os.environ.get("LIDAR_PORT", ""),
    )
    print(f"[kos] {robot}", flush=True)
    print("[kos] OpenCR IMU는 켜고 2초 동안 가만히 둬야 자이로 편향을 잡습니다", flush=True)

    stop = threading.Event()

    def request_stop(signum: int, frame: object) -> None:
        stop.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    port = int(os.environ.get("KROS_SERVER_PORT", "9000"))
    host = {"ip": "0.0.0.0", "port": port} if port > 0 else None
    kros = Kros(lambda ctx: create_nodes(ctx, robot), host=host)
    if host is not None:
        addresses = ", ".join(f"{address}:{port}" for address in _local_addresses()) or f"<ip>:{port}"
        print(f"[kos] host mode: port {port} (PC에서: viz_remote.py <pi-ip> {port})", flush=True)
        print(f"[kos] 연결 주소: {addresses}", flush=True)

    hardware = threading.Thread(target=run_hardware, args=(robot, stop), name="hardware", daemon=True)
    hardware.start()
    try:
        while not stop.wait(0.5):
            pass
    finally:
        stop.set()
        hardware.join(timeout=2.0)
        robot.stop()  # 마지막 C,0,0 + 시리얼 닫기
        kros.shutdown()
        print("[kos] 종료", flush=True)


if __name__ == "__main__":
    main()
