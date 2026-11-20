"""TurtleBot3 3D 시뮬레이터. 실행: (ros2/ 에서) python3 -m kros.examples  (또는 run.sh)

- 현실 쪽(kros와 무관한 순수 시뮬레이션): MazeWorld 안에 TurtleBot3를 놓고, SimulationRunner가 화면과 따로 월드를 고정 간격으로 돌리고,
  Viewer(3D 창)가 스냅샷을 그린다. 사람의 입력은 Keyboard(Interaction)가 받아 로봇이 receive()한다 (창의 키 -> Keyboard -> 로봇이 해석).
- kros 쪽(Node): RobotBridge만. cmd_vel -> robot.command (다른 프로그램이 로봇을 움직일 때), 라이다 측정 -> scan
"""
from kros import Kros
from kros.context import Context
from kros.node import Node
from kros.simulation.runner import SimulationRunner
from kros.simulation.viewer import Viewer
from kros.interactions import Keyboard
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3

from .robot_bridge import RobotBridge


def main() -> None:
    world, robot, keyboard = MazeWorld(), TurtleBot3(), Keyboard()
    print(f"compute: {world.compute.name}")  # gpu(cupy) / cpu(numpy) / python
    world.add(robot)
    robot.receive(keyboard)  # 로봇이 키보드 입력을 받는다
    runner = SimulationRunner(world, robot)  # 월드는 화면과 따로 고정 간격(60Hz)으로 돈다

    def create_nodes(ctx: Context) -> list[Node]:
        return [RobotBridge(ctx, robot)]

    kros = Kros(create_nodes)
    runner.start()
    try:
        Viewer(runner, lambda keys: keyboard.set_keys(keys)).run()  # 창이 받은 키를 키보드 입력으로
    finally:
        runner.stop()
        kros.shutdown()


if __name__ == "__main__":
    main()
