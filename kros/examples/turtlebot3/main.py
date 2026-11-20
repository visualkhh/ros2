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

from kros import Kros
from kros.context import Context
from kros.interactions import Keyboard
from kros.log import enable_console
from kros.nodes import Node
from kros.nodes import Cartographer, DiffDriveOdometry, Ekf, MapSaver, MapServer, RobotStatePublisher
from kros.nodes.nav2.controller import ControllerServer
from kros.nodes.nav2.planner import PlannerServer
from kros.timer import SimulatedTimeSource
from kros.viewer.panda_viewer import Viewer
from kros.viewer.snapshot_recorder import SnapshotRecorder
from kros.viz import Viz
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import WHEEL_RADIUS, WHEEL_SEPARATION
from kros.worlds.simulation_runner import SimulationRunner

from .robot_bridge import RobotBridge

MAPS_DIRECTORY = Path(__file__).resolve().parent / "artifacts" / "maps"  # 저장한 지도 (git에는 안 올린다: kros/.gitignore)

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
        footprint = {"robot_radius": robot.radius}
        planner = PlannerServer(ctx, parameters={"GridBased.plugin": "nav2_smac_planner::SmacPlannerHybrid"}, costmap_parameters=footprint)
        controller = ControllerServer(ctx, parameters={"odom_topic": "odometry/filtered", "FollowPath.plugin": "nav2_mppi_controller::MPPIController"}, costmap_parameters=footprint)
        return [RobotBridge(ctx, robot), description, odometry, ekf, slam, MapServer(ctx, parameters=maps), MapSaver(ctx, parameters=maps), planner, controller]

    kros = Kros(create_nodes, time_source=time_source)
    runner.start()
    try:
        viewer = Viewer(recorder, lambda keys: keyboard.set_keys(keys))  # 창이 받은 키를 키보드 입력으로
        Viz(kros, robot.description_dir, base=viewer)  # rviz 같은 두 번째 창: 토픽만 구독해서 그린다
        viewer.run()
    finally:
        runner.stop()
        kros.shutdown()


if __name__ == "__main__":
    main()
