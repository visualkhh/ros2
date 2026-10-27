"""view-model — PC에서 로봇 모델(URDF) 확인: robot_state_publisher + joint_state_publisher + RViz.

  ros2 launch tb3_bringup view-model.launch.py
  ros2 launch tb3_bringup view-model.launch.py gui:=true     # 바퀴 관절 슬라이더 (joint_state_publisher_gui 필요)

URDF 고친 뒤: colcon build --packages-select tb3_bringup → 다시 실행.
로봇이 떠 있는 상태에서는 쓰지 말 것 (robot_state_publisher·joint_states가 둘이 됨) — 도메인을 바꾸거나 로봇을 끄고.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('tb3_bringup')
    urdf = xacro.process_file(os.path.join(share, 'urdf', 'robot.urdf.xacro')).toxml()
    gui = LaunchConfiguration('gui')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher', condition=UnlessCondition(gui)),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui', condition=IfCondition(gui)),
        Node(package='rviz2', executable='rviz2', arguments=['-d', os.path.join(share, 'rviz', 'model.rviz')]),
    ])
