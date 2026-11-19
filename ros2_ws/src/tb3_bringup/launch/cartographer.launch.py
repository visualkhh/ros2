"""cartographer 2D mapping (LDS-03 /scan + wheel /odom, no IMU).

  ros2 launch tb3_bringup cartographer.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.03'),   # 맵 해상도 [m] — cartographer_2d.lua의 grid_options_2d.resolution과 같게
        DeclareLaunchArgument('publish_period_sec', default_value='0.3'),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            arguments=[
                '-configuration_directory',
                os.path.join(get_package_share_directory('tb3_bringup'), 'config'),
                '-configuration_basename', 'cartographer_2d.lua',
            ],
            remappings=[('scan', '/scan'), ('odom', '/odom')],
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            # resolution · publish_period_sec는 ROS 파라미터가 아니라 gflags(명령줄 플래그)다 — parameters로 주면 무시돼
            #   /map이 늘 0.05 m · 1.0 s로 나왔다 (2026-11-19 pak: launch 기본 0.03인데 저장 지도가 0.050)
            arguments=['-resolution', LaunchConfiguration('resolution'),
                       '-publish_period_sec', LaunchConfiguration('publish_period_sec')],
        ),
    ])
