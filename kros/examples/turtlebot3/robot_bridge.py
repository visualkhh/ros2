from kros.context import Context
from kros.node import Node
from kros.worlds.robots.turtlebot3 import TurtleBot3
from kros.msgs.geometry_msgs import Twist
from kros.msgs.sensor_msgs import LaserScan

from .topics import CMD_VEL_TOPIC_NAME, SCAN_TOPIC_NAME


class RobotBridge(Node):
    """kros와 로봇 하드웨어를 잇는 접착제(실제 로봇의 하드웨어 드라이버 자리): cmd_vel -> robot.command, 라이다 측정 -> scan 토픽."""

    def __init__(self, context: Context, robot: TurtleBot3) -> None:
        super().__init__(context)  # 이름은 클래스 이름(RobotBridge)
        self._robot = robot
        self.subscribe(CMD_VEL_TOPIC_NAME, Twist, self._on_cmd_vel)
        robot.lidar.subscribe(self.publish_scan)  # 라이다가 새로 측정할 때마다 scan 토픽으로

    def _on_cmd_vel(self, twist: Twist) -> None:
        self._robot.command = twist  # 객체째 교체라 구독 스레드에서 넣어도 안전

    def publish_scan(self, scan: LaserScan) -> None:
        self.publish(SCAN_TOPIC_NAME, scan)
