"""tb-only — 로봇(Pi) bringup: state_manager + cmd_manager(mux) + OpenCR 드라이버 + robot_state_publisher + LDS-03(+필터).

  ros2 launch tb3_bringup tb-only.launch.py          # RL 끈 상태로 시작 (보정 없이 그대로, PC 불필요)
  ros2 launch tb3_bringup tb-only.launch.py port:=/dev/ttyACM0
  ros2 launch tb3_bringup tb-only.launch.py lidar:=false      # 라이다 없이 (coin_d4_driver 미설치여도 됨)

라이다: coin_d4_driver(ros2_ws/deps.repos) → /scan_raw → laser_filters 몸통 상자 제거 → /scan_filtered
       → scan_resample(고정 400칸: LDS-03 점 개수가 매번 달라 SLAM이 버림) → /scan

명령 경로 (ARCH.md): /cmd_manual_vel(텔레오퍼)·/cmd_vel(Nav2) → cmd_manager → /cmd_vel_base → 드라이버.
RL은 실행 중 /cmd_manual_rl(Bool)로 켜고 끔 — 켜면 /cmd_vel_rl_in → [PC rl-only] → /cmd_vel_rl_out을 거침.
rl:=true = RL 켠 상태로 시작 (PC rl-only가 있어야 움직임). 이 launch에 RL 노드는 없음.
SLAM·Nav는 tool/run-cli가 따로 띄운다 (TOOL_BASE_CMD로 이 launch를 지정).
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from ament_index_python.packages import PackageNotFoundError
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb3_bringup')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'robot.urdf.xacro')).toxml()
    rl = LaunchConfiguration('rl')   # 'true'/'false' → 파라미터는 yaml로 bool 변환됨
    port = LaunchConfiguration('port')
    params = os.path.join(share, 'config', 'robot.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('rl', default_value='false', description='true면 RL 켠 상태로 시작 (/state_rl)'),
        DeclareLaunchArgument('port', default_value='/dev/opencr'),
        DeclareLaunchArgument('lidar', default_value='true', description='LDS-03 + 몸통 필터'),
        Node(package='tb3_bringup', executable='state_manager', name='state_manager',
             parameters=[{'rl': rl}]),
        Node(package='tb3_bringup', executable='cmd_manager', name='cmd_manager'),
        Node(package='tb3_bringup', executable='opencr_driver', name='opencr_driver',
             parameters=[params, {'port': port}], output='screen'),
        LogInfo(msg='RL 켠 상태로 시작: PC에서 ros2 launch tb3_bringup rl-only.launch.py 를 띄워야 바퀴가 움직입니다',
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
        return [LogInfo(msg='WARN: coin_d4_driver 없음 → 라이다 안 띄움 (~/ros2/setup/pi/build.sh 로 외부 소스 받기)',
                        condition=condition)]
    return [
        Node(package='coin_d4_driver', executable='single_coin_d4_node', name='coin_d4_driver',
             parameters=[lidar_params], output='screen', condition=condition,
             sigterm_timeout='15', sigkill_timeout='15'),
        Node(package='laser_filters', executable='scan_to_scan_filter_chain', name='scan_filter',
             parameters=[lidar_params], remappings=[('scan', 'scan_raw')],
             condition=condition),
        Node(package='tb3_bringup', executable='scan_resample', name='scan_resample',
             parameters=[lidar_params], remappings=[('scan_in', 'scan_filtered'), ('scan_out', 'scan')],
             condition=condition),
    ]
