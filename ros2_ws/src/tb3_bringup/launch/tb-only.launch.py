"""tb-only — 로봇(Pi)만으로 주행: OpenCR 드라이버 + robot_state_publisher + LDS-03 라이다(+몸통 필터).

  ros2 launch tb3_bringup tb-only.launch.py          # cmd_vel -> 드라이버 (보정 없이 그대로, PC 불필요)
  ros2 launch tb3_bringup tb-only.launch.py port:=/dev/ttyACM0
  ros2 launch tb3_bringup tb-only.launch.py lidar:=false      # 라이다 없이 (coin_d4_driver 미설치여도 됨)

라이다: coin_d4_driver(ros2_ws/deps.repos) → /scan_raw → laser_filters 몸통 상자 제거 → /scan

RL 보정과 같이 쓰려면 tb-for-rl.launch.py (Pi) + rl-only.launch.py (PC).
rl:=true는 tb-for-rl이 쓰는 내부 인자 — 드라이버가 cmd_vel 대신 cmd_vel_rl을 받는다.
SLAM·Nav는 tool/run-cli가 따로 띄운다 (TOOL_BASE_CMD로 이 launch를 지정).
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import PackageNotFoundError
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
        DeclareLaunchArgument('lidar', default_value='true', description='LDS-03 + 몸통 필터'),
        driver('cmd_vel', UnlessCondition(rl)),
        driver('cmd_vel_rl', IfCondition(rl)),
        LogInfo(msg='tb-for-rl: PC에서 ros2 launch tb3_bringup rl-only.launch.py 를 띄워야 바퀴가 움직입니다',
                condition=IfCondition(rl)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        *lidar_nodes(share, IfCondition(LaunchConfiguration('lidar'))),
    ])


def lidar_nodes(share, condition):
    lidar_params = os.path.join(share, 'config', 'lidar.yaml')
    try:
        get_package_share_directory('coin_d4_driver')
    except PackageNotFoundError:
        return [LogInfo(msg='WARN: coin_d4_driver 없음 → 라이다 안 띄움 (cd ros2_ws && vcs import src < deps.repos && colcon build)',
                        condition=condition)]
    return [
        Node(package='coin_d4_driver', executable='single_coin_d4_node', name='coin_d4_driver',
             parameters=[lidar_params], output='screen', condition=condition,
             sigterm_timeout='15', sigkill_timeout='15'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', name='scan_filter',
             parameters=[lidar_params], remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
             condition=condition),
    ]
