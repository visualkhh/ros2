from kros.context import Context
from kros.diagnostics import DiagnosticUpdater
from kros.msgs.geometry_msgs import Twist
from kros.msgs.sensor_msgs import LaserScan
from kros.node import Node
from kros.worlds.robots.turtlebot3 import TurtleBot3

from .hardware_status import hardware_path, leaves, make_task
from .topics import CMD_VEL_TOPIC_NAME, SCAN_TOPIC_NAME


class RobotBridge(Node):
    """kros와 로봇 하드웨어를 잇는 접착제(실제 로봇의 하드웨어 드라이버 자리): cmd_vel -> robot.command, 라이다 측정 -> scan 토픽,
    하드웨어가 남긴 사실 -> diagnostics 토픽 (진단 수준 판단은 여기서 한다).

    파라미터: diagnostics_period(진단 발행 주기 초, 실행 중에도 바꿀 수 있다), data_timeout(센서가 이 시간 넘게 값이 없으면 ERROR).
    """

    def __init__(self, context: Context, robot: TurtleBot3) -> None:
        super().__init__(context)  # 이름은 클래스 이름(RobotBridge)
        self._robot = robot
        self.data_timeout = self.declare_parameter("data_timeout", 2.0, validator=lambda seconds: seconds > 0)
        self.diagnostics_period = self.declare_parameter(
            "diagnostics_period", 1.0, validator=lambda seconds: seconds > 0, on_change=lambda seconds: self._diagnostics.set_period(seconds)
        )
        self._diagnostics = DiagnosticUpdater(self, self.diagnostics_period.value, hardware_id=robot.name)
        for hardware in leaves(robot):  # 센서/구동기마다 진단 작업 하나
            self._diagnostics.add(hardware_path(hardware), make_task(hardware, self.now, lambda: self.data_timeout.value))
        self.get_logger().info("로봇 %s 연결: 진단 %d개, 주기 %.1f초", robot.name, len(leaves(robot)), self.diagnostics_period.value)
        self.subscribe(CMD_VEL_TOPIC_NAME, Twist, self._on_cmd_vel)
        robot.lidar.subscribe(self.publish_scan)  # 라이다가 새로 측정할 때마다 scan 토픽으로

    def _on_cmd_vel(self, twist: Twist) -> None:
        self.get_logger().debug("cmd_vel 받음: v=%.2f w=%.2f", twist.linear.x, twist.angular.z)
        self._robot.command = twist  # 객체째 교체라 구독 스레드에서 넣어도 안전

    def publish_scan(self, scan: LaserScan) -> None:
        self.publish(SCAN_TOPIC_NAME, scan)
