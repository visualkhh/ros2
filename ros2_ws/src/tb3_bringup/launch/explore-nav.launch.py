"""explore-nav — SLAM 중 Nav: SLAM이 /map·map→odom을 내는 동안 Nav2 주행부만 (지도·amcl 없이) + 자동 맵핑.

  Nav2 navigation_launch.py + lifecycle_manager + explore_lite(frontier 탐색) + explore_bridge(제어 상태 연결).
  explore_lite는 제어 상태가 SLAM_AUTO_MAPPING일 때만 탐색한다 (그 밖엔 explore_bridge가 바로 멈춤).

  ros2 launch tb3_bringup explore-nav.launch.py                 # 기본: share/tb3_bringup/config/nav2_params.yaml
  ros2 launch tb3_bringup explore-nav.launch.py params_file:=/path/nav2_params.yaml   # 덮어쓰기

Lyrical의 nav2_bringup navigation_launch.py가 lifecycle_manager까지 같이 띄운다
→ 여기서 manager를 또 붙이지 않는다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb3_bringup')
    nav = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'nav2_params.yaml'),
            description='Nav2 파라미터 (기본: tb3_bringup/config/nav2_params.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(nav, launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'}.items()),
        Node(package='explore_lite', executable='explore', name='explore_node', output='screen',
             parameters=[os.path.join(get_package_share_directory('tb3_bringup'), 'config', 'explore.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        Node(package='tb3_bringup', executable='explore_bridge', name='explore_bridge'),
    ])
