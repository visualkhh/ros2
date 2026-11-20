"""TF 사용법 예제: 노드에서 TF를 받아 쓰고, 내보내고, kros가 제공하는 노드를 띄운다. 실행: (ros2/ 에서) python3 -m kros.examples.turtlebot3.tf_usage

세 가지를 보여 준다.
1. 받기: 노드가 TransformListener를 만들고 버퍼로 조회한다 (라이다 점을 로봇 좌표로 옮기기).
2. 내기/바꾸기: 노드가 TransformBroadcaster로 변환을 내보낸다. 같은 자식 좌표계의 새 변환을 내면 그게 최신 값이다 (ROS처럼 "수정"은 새로 내는 것).
3. 제공 노드: RobotStatePublisher를 node_create에서 만들어 돌려주면 URDF로 TF가 생긴다 (안 만들면 아무 일도 안 한다).
"""
import math

from kros import Kros
from kros.context import Context
from kros.msgs.geometry_msgs import Quaternion, Transform, TransformStamped, Twist, Vector3
from kros.msgs.std_msgs import Header
from kros.node import Node
from kros.nodes import RobotStatePublisher
from kros.tf import Buffer, LookupException, TransformBroadcaster, TransformListener
from kros.timer import SimulatedTimeSource
from kros.worlds.maze.maze_world import MazeWorld
from kros.worlds.robots.turtlebot3 import TurtleBot3

from .robot_bridge import RobotBridge


class ScanToBase(Node):
    """1. 받기: 라이다가 본 점(base_scan 좌표)을 로봇 몸체 좌표(base_footprint)로 바꿔 보는 노드."""

    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)  # tf, tf_static을 구독해 버퍼를 채운다 (tf_static은 늦게 떠도 받는다)

    def lidar_point_in_base(self, distance_ahead: float) -> Vector3 | None:
        try:
            # (목표 좌표계, 원래 좌표계, 점). 시각을 안 주면 가장 최근 값. 아직 TF가 없으면 LookupException이다
            return self.tf_buffer.transform_point("base_footprint", "base_scan", Vector3(distance_ahead, 0.0, 0.0))
        except LookupException:
            return None  # RobotStatePublisher가 아직 안 떴다


class OdomPublisher(Node):
    """2. 내기/바꾸기: 로봇이 odom 좌표계에서 (x, 0) 위치에 있다고 매번 새로 알리는 노드 (header.frame_id가 부모, child_frame_id가 자식)."""

    def __init__(self, context: Context) -> None:
        super().__init__(context)
        self._broadcaster = TransformBroadcaster(self)

    def report(self, x: float, yaw: float) -> None:
        transform = Transform(Vector3(x, 0.0, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        self._broadcaster.send_transform(TransformStamped(Header(self.now(), "odom"), "base_footprint", transform))


def run(seconds: float = 2.0) -> dict[str, object]:
    """예제를 시뮬레이션 시간으로 돌려 결과를 돌려준다 (테스트가 이 함수를 실행해서 예제가 낡지 않는지 본다)."""
    source = SimulatedTimeSource()
    world, robot = MazeWorld(puck_count=0), TurtleBot3()
    world.add(robot)
    made: dict[str, Node] = {}

    def create_nodes(ctx: Context) -> list[Node]:  # ROS의 launch 파일처럼, 어떤 노드를 띄울지 여기서 정한다
        made["scan"], made["odom"] = ScanToBase(ctx), OdomPublisher(ctx)
        return [
            made["scan"],
            made["odom"],
            RobotBridge(ctx, robot),  # 로봇 -> joint_states, scan, 진단
            RobotStatePublisher(ctx, parameters={"robot_description": robot.urdf_path.read_text()}),  # 3. 제공 노드: URDF -> TF
        ]

    kros = Kros(create_nodes, time_source=source)
    scan, odom = made["scan"], made["odom"]
    assert isinstance(scan, ScanToBase) and isinstance(odom, OdomPublisher)
    robot.command = Twist(Vector3(0.1, 0.0, 0.0), Vector3())
    try:
        for _ in range(round(seconds / 0.05)):
            world.step(0.05)
            source.set(world.time)
            kros.context.timer_manager.join()  # 시뮬레이션 시간을 sleep 없이 밀 때는 타이머가 따라오게 기다린다
            odom.report(robot.pose.x, robot.pose.theta)
        kros.context.topic_manager.join()
        base = scan.lidar_point_in_base(1.0)
        buffer = scan.tf_buffer
        return {
            "frames": buffer.all_frames(),
            "lidar_point_1m_in_base": base,
            "robot_in_odom": buffer.lookup_transform("odom", "base_footprint").transform.translation,
            "lidar_in_odom": buffer.transform_point("odom", "base_scan", Vector3()),  # 라이다 위치가 odom에서는 어디인가 (체인: odom -> base_footprint -> base_link -> base_scan)
            "robot_x": robot.pose.x,
        }
    finally:
        kros.shutdown()


if __name__ == "__main__":
    result = run()
    for key, value in result.items():
        print(f"{key}: {value}")
