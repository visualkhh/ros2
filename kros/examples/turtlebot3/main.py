"""TurtleBot3 3D 시뮬레이터. 실행: (ros2/ 에서) python3 -m kros.examples  (또는 run.sh)

- 현실 쪽(kros와 무관한 순수 시뮬레이션): MazeWorld 안에 TurtleBot3를 놓고, SimulationRunner가 화면과 따로 월드를 고정 간격으로 돌리고,
  SnapshotRecorder가 스텝마다 스냅샷을 만들어 Viewer(3D 창)가 그린다. 사람의 입력은 Keyboard(Interaction)가 받아 로봇이 receive()한다 (창의 키 -> Keyboard -> 로봇이 해석).
- Viz(kros): rviz 같은 두 번째 창. kros 인스턴스 하나만 받아 tf/robot_description/scan/imu/odom/odometry/filtered 토픽을 구독해 그린다 (Viewer는 시뮬레이션의 진실, Viz는 로봇이 내보낸 값).
  바퀴 odom(분홍 화살표)과 EKF 결과(하늘색 화살표, 모델 위치)를 겹쳐 본다. 왼쪽 패널 SLAM 칸에서 Cartographer를 켜고(mapping),
  지도를 저장/불러오고(localization), 2D Pose Estimate로 위치를 찍는다. 지도를 볼 때는 Fixed Frame을 map으로.
- kros 쪽(Node): RobotBridge, cmd_vel -> robot.command (다른 프로그램이 로봇을 움직일 때), 라이다 측정 -> scan.
  kros의 시간(타이머, node.now())은 시뮬레이션 시간이다: 러너가 매 스텝 SimulatedTimeSource를 밀어 준다
"""
from pathlib import Path

# 파일을 직접 실행할 때(python main.py / IDE Run 버튼) 저장소 루트(ros2/)를 경로에 넣는다.
# 모듈로 임포트할 때(__package__가 있음)는 건드리지 않는다.
import os
import sys
if __package__ is None:
    sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
if Path("/proc/driver/nvidia/version").exists():
    os.environ.setdefault("__NV_PRIME_RENDER_OFFLOAD", "1")
    os.environ.setdefault("__GLX_VENDOR_LIBRARY_NAME", "nvidia")

from kros import Kros
from kros.context import Context
from kros.interactions import Keyboard
from kros.log import enable_console
from kros.nodes import Node
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, MapCatalog, MapSaver, MapServer, RobotStatePublisher
from kros.nodes.nav2.behavior_server import BehaviorServer
from kros.nodes.nav2.bt_navigator import BtNavigator
from kros.nodes.nav2.collision_monitor import CollisionMonitor
from kros.nodes.nav2.controller import ControllerServer
from kros.nodes.nav2.planner import PlannerServer
from kros.nodes.twist_mux import TwistMux
from kros.parameter import ParameterValue
from kros.timer import SimulatedTimeSource
from kros.viewer.panda_viewer import Viewer
from kros.viewer.snapshot_recorder import SnapshotRecorder
from kros.viz import Viz
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import WHEEL_RADIUS, WHEEL_SEPARATION
from kros.worlds.simulation_runner import SimulationRunner

from kros.examples.turtlebot3 import rl_nav_controller  # noqa: F401  # 이 예제의 controller: controller_server 플러그인 표에 rl_nav_controller::RlNavController를 넣는다
from kros.examples.turtlebot3.robot_bridge import RobotBridge

MAPS_DIRECTORY = Path(__file__).resolve().parent / "artifacts" / "maps"
# isaac/training/rl_maze에서 학습해 export한 정책 (policy.onnx + policy.onnx.data). isaac/artifacts는 git에 없다: 없으면 MPPI로 돈다
RL_POLICY = Path(__file__).resolve().parents[3] / "isaac/artifacts/rl_maze/training/2026-11-19_11-38-08_nodyn1k/exported/policy.onnx"  # 저장한 지도 (git에는 안 올린다: kros/.gitignore)

# robot_localization의 *_config 15칸 (x y z roll pitch yaw vx vy vz vroll vpitch vyaw ax ay az) 중 켤 칸
ODOM_CONFIG = tuple(i in (6, 11) for i in range(15))  # 바퀴: vx, vyaw (위치는 바퀴가 적분한 것이라 다시 넣지 않는다)
IMU_CONFIG = tuple(i == 11 for i in range(15))  # 자이로: vyaw (방향은 편향이 쌓여 쓰지 않는다)


def main() -> None:
    enable_console()  # 노드 로그를 콘솔에 보인다 (수준은 노드의 log_level 파라미터)
    world, robot, keyboard = MazeWorld(), TurtleBot3(noisy=True), Keyboard()  # noisy: 바퀴 슬립/엔코더 해상도, IMU 편향, 라이다 잡음 (진실은 Viewer, 측정값은 Viz에서 비교)
    print(f"compute: {world.compute.name}")  # gpu(cupy) / cpu(numpy) / python
    world.add(robot)
    robot.receive(keyboard)  # 로봇이 키보드 입력을 받는다
    runner = SimulationRunner(world)  # 월드는 화면과 따로 고정 간격(60Hz)으로 돈다
    recorder = SnapshotRecorder(world, robot)  # 스텝마다 화면에 줄 스냅샷을 만든다
    runner.add_listener(recorder.record)
    time_source = SimulatedTimeSource()  # kros의 시간은 시뮬레이션 시간 (시뮬레이션이 매 스텝 밀어 준다)
    runner.add_listener(lambda w: time_source.set(w.time))

    def create_nodes(ctx: Context) -> list[Node]:
        # kros가 제공하는 노드를 쓸지는 여기서(launch 파일처럼) 정한다: URDF로 TF를 만드는 RobotStatePublisher
        description = RobotStatePublisher(ctx, parameters={"robot_description": robot.urdf_path.read_text()})
        # 바퀴 -> odom 토픽. TF(odom -> base_footprint)는 EKF가 내므로 끈다
        odometry = DiffDriveOdometry(ctx, parameters={"wheel_radius": WHEEL_RADIUS, "wheel_separation": WHEEL_SEPARATION, "publish_tf": False})
        # robot_localization ekf_node처럼: 바퀴의 앞 속도/회전 속도 + 자이로 회전 속도 -> odometry/filtered, odom -> base_footprint TF
        ekf = Ekf(ctx, parameters={"odom0": "odom", "odom0_config": ODOM_CONFIG, "imu0": "imu", "imu0_config": IMU_CONFIG})
        # SLAM/위치 추정 (map -> odom)과 지도 파일 (이 예제 폴더의 artifacts/maps). viz 패널에서 모드를 바꾸고, 지도를 저장/불러오고, 2D Pose Estimate를 찍는다
        slam = Cartographer(ctx, parameters={"mode": "idle"})
        maps = {"maps_directory": str(MAPS_DIRECTORY)}
        # 내비게이션 (nav2처럼): 경로 계획은 Smac(Hybrid-A*), 따라가기는 MPPI. 둘 다 안에 costmap을 만든다 (로봇 몸 반지름으로 inflation)
        footprint = {"robot_radius": robot.radius, "inflation_radius": 0.3}
        planner = PlannerServer(ctx, parameters={"GridBased.plugin": "nav2_smac_planner::SmacPlannerHybrid"}, costmap_parameters=footprint)
        # controller_parameters: dict[str, ParameterValue] = {"odom_topic": "odometry/filtered", "FollowPath.plugin": "nav2_mppi_controller::MPPIController", "cmd_vel_topic": "cmd_vel_nav"}
        # 따라가기는 isaac/training/rl_maze에서 학습한 정책 (rl_nav_controller: 실기와 같은 관측/행동). 모델이 없거나 못 읽으면 안에 품은 MPPI로 돈다
        controller_parameters: dict[str, ParameterValue] = {
            "odom_topic": "odometry/filtered",
            "cmd_vel_topic": "cmd_vel_nav",
            "FollowPath.plugin": "rl_nav_controller::RlNavController",
            "FollowPath.enable": True,
            "FollowPath.model_path": str(RL_POLICY),
            "FollowPath.lidar_inflation": 0.03,  # 이 정책은 inflation 0.01~0.05m로 학습했다 (params/env.yaml의 inflation_range)
            "FollowPath.fallback_controller": "mppi",
            "general_goal_checker.xy_goal_tolerance": 0.4,
            "general_goal_checker.yaw_goal_tolerance": 0.4,
            "progress_checker.movement_time_allowance": 2.0,
            "progress_checker.required_movement_radius": 0.2,
        }
        controller = ControllerServer(ctx, parameters=controller_parameters, costmap_parameters=footprint)
        behavior = BehaviorServer(ctx)  # 회복 동작 (spin · backup · wait) — bt_navigator가 실패 시 부른다
        # 속도 명령의 길: controller -> cmd_vel_nav -+-> twist_mux -> cmd_vel_mux -> collision_monitor -> cmd_vel -> RobotBridge
        #                 (다른 프로그램의) teleop -> cmd_vel_teleop -+   (사람이 이긴다)       (라이다로 앞이 막히면 멈추거나 늦춘다)
        mux = TwistMux(ctx, parameters={"cmd_vel_out": "cmd_vel_mux"})
        monitor = CollisionMonitor(ctx, parameters={"cmd_vel_in_topic": "cmd_vel_mux", "cmd_vel_out_topic": "cmd_vel"})
        navigator = BtNavigator(ctx, parameters={"replanning_period": 3.0})  # goal_pose(viz의 2D Goal Pose) -> 계획 -> 따라가기 -> 3초마다 재계획
        return [RobotBridge(ctx, robot), description, odometry, ekf, slam, MapServer(ctx, parameters=maps), MapSaver(ctx, parameters=maps), MapCatalog(ctx, parameters=maps), planner, controller, behavior, navigator, mux, monitor]

    import os
    server_port = int(os.environ.get("KROS_SERVER_PORT", "9000"))
    host = {"ip": "0.0.0.0", "port": server_port} if server_port > 0 else None
    kros = Kros(create_nodes, time_source=time_source, host=host)
    if host:
        print(f"[kros] host mode: port {server_port} (remote Viz can connect)", flush=True)
    runner.start()
    try:
        viewer = Viewer(recorder, lambda keys: keyboard.set_keys(keys))  # 창이 받은 키를 키보드 입력으로
        if os.environ.get("KROS_VIZ", "0") != "0":
            Viz(package_root=robot.description_dir, base=viewer, context=kros.context)  # rviz 같은 두 번째 창: 토픽만 구독해서 그린다. KROS_VIZ=1이면 띄움
        viewer.run()
    finally:
        runner.stop()
        kros.shutdown()


if __name__ == "__main__":
    main()
