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
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period_sec', default_value='1.0'),
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
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'resolution': LaunchConfiguration('resolution')},
                {'publish_period_sec': LaunchConfiguration('publish_period_sec')},
            ],
        ),
    ])
