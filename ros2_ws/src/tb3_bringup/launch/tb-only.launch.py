"""tb-only — 로봇(Pi)만으로 주행: OpenCR 드라이버 + robot_state_publisher (+ 라이다는 추후).

  ros2 launch tb3_bringup tb-only.launch.py          # cmd_vel -> 드라이버 (보정 없이 그대로, PC 불필요)
  ros2 launch tb3_bringup tb-only.launch.py port:=/dev/ttyACM0

RL 보정과 같이 쓰려면 tb-rl.launch.py (Pi) + rl-only.launch.py (PC).
rl:=true는 tb-rl이 쓰는 내부 인자 — 드라이버가 cmd_vel 대신 cmd_vel_rl을 받는다.
SLAM·Nav는 tool/run-cli가 따로 띄운다 (TOOL_BASE_CMD로 이 launch를 지정).
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb3_bringup')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'robot.urdf.xacro')).toxml()
    rl = LaunchConfiguration('rl')
    port = LaunchConfiguration('port')
    params = os.path.join(share, 'config', 'robot.yaml')

    def driver(cmd_topic, condition):
        return Node(
            package='tb3_bringup', executable='opencr_driver', name='opencr_driver',
            parameters=[params, {'port': port, 'cmd_topic': cmd_topic}],
            output='screen', condition=condition)

    return LaunchDescription([
        DeclareLaunchArgument('rl', default_value='false', description='true면 cmd_vel_rl(PC RL 보정)을 받음'),
        DeclareLaunchArgument('port', default_value='/dev/opencr'),
        driver('cmd_vel', UnlessCondition(rl)),
        driver('cmd_vel_rl', IfCondition(rl)),
        LogInfo(msg='tb-rl: PC에서 ros2 launch tb3_bringup rl-only.launch.py 를 띄워야 바퀴가 움직입니다',
                condition=IfCondition(rl)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        # TODO 라이다 드라이버 (모델 정해지면): frame_id=base_scan, topic=/scan
    ])
