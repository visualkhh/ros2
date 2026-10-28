"""Basic bringup (PC test, no OpenCR): state_manager + cmd_manager + mock base.

Same command path as the robot (tb-only) with mock_base in place of opencr_driver:
  /cmd_manual_vel (teleop) · /cmd_vel (Nav2) -> cmd_manager -> /cmd_vel_base -> mock_base -> odom
RL: ros2 launch tb3_bringup rl-only.launch.py + /cmd_manual_rl true (tool: ./run-cli rl on).
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='tb3_bringup', executable='state_manager', name='state_manager'),
        Node(package='tb3_bringup', executable='cmd_manager', name='cmd_manager'),
        Node(package='tb3_bringup', executable='mock_base', name='mock_base',
             remappings=[('cmd_vel', 'cmd_vel_base')]),
    ])
