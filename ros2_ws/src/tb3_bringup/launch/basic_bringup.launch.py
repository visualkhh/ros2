"""Basic bringup: corrector(passthrough) + mock base + eval recorder.

Chain: teleop(cmd_vel_raw) -> corrector -> cmd_vel -> mock_base -> odom
Run teleop separately (needs keyboard stdin):
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    corrector = Node(
        package='rl_correction',
        executable='corrector',
        name='rl_corrector',
        remappings=[('cmd_vel_in', 'cmd_vel_raw'), ('cmd_vel_out', 'cmd_vel')],
        parameters=[{'enable': False, 'model_path': ''}],
    )
    mock_base = Node(
        package='tb3_bringup',
        executable='mock_base',
        name='mock_base',
    )
    eval_recorder = Node(
        package='rl_eval',
        executable='eval_recorder',
        name='eval_recorder',
        parameters=[{'output': '/tmp/basic.csv', 'label': 'basic'}],
    )
    return LaunchDescription([corrector, mock_base, eval_recorder])
