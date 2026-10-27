"""rl-only — PC용: RL 노드(corrector + eval)만 띄운다 (로봇 노드 없음).

  ros2 launch tb3_bringup rl-only.launch.py                                   # passthrough (enable:=false)
  ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/path/policy.onnx
  ros2 launch tb3_bringup rl-only.launch.py label:=after output:=/tmp/after.csv

흐름: teleop/Nav2(Pi) -> cmd_vel -> [PC] corrector -> cmd_vel_rl -> (Pi) opencr_driver
로봇은 tb-for-rl.launch.py 로 떠 있어야 한다. 같은 ROS_DOMAIN_ID(25)·같은 WiFi.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    arg = LaunchConfiguration
    return LaunchDescription([
        DeclareLaunchArgument('enable', default_value='false', description='false면 그대로 통과'),
        DeclareLaunchArgument('model', default_value='', description='ONNX 정책 경로'),
        DeclareLaunchArgument('label', default_value='rl'),
        DeclareLaunchArgument('output', default_value='/tmp/rl_eval.csv'),
        Node(package='rl_correction', executable='corrector', name='rl_corrector',
             remappings=[('cmd_vel_in', 'cmd_vel'), ('cmd_vel_out', 'cmd_vel_rl')],
             parameters=[{'enable': arg('enable'), 'model_path': arg('model')}],
             output='screen'),
        Node(package='rl_eval', executable='eval_recorder', name='eval_recorder',
             remappings=[('cmd_vel_raw', 'cmd_vel'), ('cmd_vel', 'cmd_vel_rl')],
             parameters=[{'output': arg('output'), 'label': arg('label')}]),
    ])
