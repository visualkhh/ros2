"""explore-nav — SLAM 중 Nav: SLAM이 /map·map→odom을 내는 동안 Nav2 주행부만 (지도·amcl 없이).

  ros2 launch tb3_bringup explore-nav.launch.py params_file:=tool/config/nav2_params.yaml

Lyrical의 nav2_bringup navigation_launch.py는 노드만 띄우고 lifecycle_manager는 bringup_launch.py에만 있다
→ 그대로 쓰면 노드가 영원히 inactive (navigate_to_pose 없음). 여기서 같은 노드 목록으로 manager를 붙인다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_bringup.navigation_launch import get_lifecycle_nodes


def lifecycle_manager(context):
    return [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 name='lifecycle_manager_navigation', output='screen',
                 parameters=[{'autostart': True,
                              'node_names': list(get_lifecycle_nodes(context)),
                              'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true'}])]


def generate_launch_description():
    nav = os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='Nav2 파라미터 (tool/config/nav2_params.yaml)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        IncludeLaunchDescription(nav, launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true'}.items()),
        OpaqueFunction(function=lifecycle_manager),
    ])
