"""rl-only — PC용: RL 노드(corrector + eval)만 띄운다 (로봇 노드 없음).

  ros2 launch tb3_bringup rl-only.launch.py                                   # passthrough (enable:=false)
  ros2 launch tb3_bringup rl-only.launch.py enable:=true model:=/path/policy.onnx
  ros2 launch tb3_bringup rl-only.launch.py label:=after output:=/tmp/after.csv

이동 보정 흐름: (Pi) cmd_manager -> /cmd_vel_rl_in -> [PC] corrector -> /cmd_vel_rl_out -> (Pi) cmd_manager -> 드라이버
로봇은 RL 경로여야 한다 (./run-cli route rl 또는 tb-only.launch.py rl:=true). 같은 ROS_DOMAIN_ID(25)·같은 WiFi.
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
        DeclareLaunchArgument('mode', default_value='auto', description='auto | track | goal'),
        DeclareLaunchArgument('goal_mode', default_value='relative', description='relative | absolute'),
        DeclareLaunchArgument('label', default_value='rl'),
        DeclareLaunchArgument('output', default_value='/tmp/rl_eval.csv'),
        Node(package='rl_correction', executable='corrector', name='rl_corrector',
             remappings=[('cmd_vel_in', 'cmd_vel_rl_in'), ('cmd_vel_out', 'cmd_vel_rl_out')],
             parameters=[{'enable': arg('enable'), 'model_path': arg('model'),
                          'mode': arg('mode'), 'goal_mode': arg('goal_mode')}],
             output='screen'),
        Node(package='rl_eval', executable='eval_recorder', name='eval_recorder',
             remappings=[('cmd_vel_raw', 'cmd_vel_rl_in'), ('cmd_vel', 'cmd_vel_rl_out')],
             parameters=[{'output': arg('output'), 'label': arg('label')}]),
    ])
