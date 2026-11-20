"""TurtleBot3 3D 시뮬레이터. 실행: (ros2/ 에서) python3 -m kros.examples  (또는 run.sh)

- 현실 쪽(kros와 무관한 순수 시뮬레이션): MazeWorld 안에 TurtleBot3를 놓고, SimulationRunner가 화면과 따로 월드를 고정 간격으로 돌리고,
  SnapshotRecorder가 스텝마다 스냅샷을 만들어 Viewer(3D 창)가 그린다. 사람의 입력은 Keyboard(Interaction)가 받아 로봇이 receive()한다 (창의 키 -> Keyboard -> 로봇이 해석).
- kros 쪽(Node): RobotBridge만. cmd_vel -> robot.command (다른 프로그램이 로봇을 움직일 때), 라이다 측정 -> scan.
  kros의 시간(타이머, node.now())은 시뮬레이션 시간이다: 러너가 매 스텝 SimulatedTimeSource를 밀어 준다
"""
from kros import Kros
from kros.context import Context
from kros.interactions import Keyboard
from kros.log import enable_console
from kros.nodes import Node
from kros.nodes import DiffDriveOdometry, RobotStatePublisher
from kros.timer import SimulatedTimeSource
from kros.viewer.panda_viewer import Viewer
from kros.viewer.snapshot_recorder import SnapshotRecorder
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.worlds.robots.turtlebot3.turtlebot3 import WHEEL_RADIUS, WHEEL_SEPARATION
from kros.worlds.simulation_runner import SimulationRunner

from .robot_bridge import RobotBridge


def main() -> None:
    enable_console()  # 노드 로그를 콘솔에 보인다 (수준은 노드의 log_level 파라미터)
    world, robot, keyboard = MazeWorld(), TurtleBot3(), Keyboard()
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
        odometry = DiffDriveOdometry(ctx, parameters={"wheel_radius": WHEEL_RADIUS, "wheel_separation": WHEEL_SEPARATION})  # 바퀴 -> odom -> base_footprint TF
        return [RobotBridge(ctx, robot), description, odometry]

    kros = Kros(create_nodes, time_source=time_source)
    runner.start()
    try:
        Viewer(recorder, lambda keys: keyboard.set_keys(keys)).run()  # 창이 받은 키를 키보드 입력으로
    finally:
        runner.stop()
        kros.shutdown()


if __name__ == "__main__":
    main()
