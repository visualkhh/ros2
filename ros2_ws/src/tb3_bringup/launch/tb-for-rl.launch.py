"""tb-for-rl — 로봇(Pi) 쪽만, PC RL 보정을 받아 움직임 (RL 노드는 안 띄움): tb-only와 같지만 드라이버가 cmd_vel_rl(PC 보정 결과)을 받는다.

  ros2 launch tb3_bringup tb-for-rl.launch.py            # Pi
  ros2 launch tb3_bringup rl-only.launch.py          # PC (같은 ROS_DOMAIN_ID=25, 같은 WiFi)

흐름: teleop/Nav2(Pi) -> cmd_vel -> [PC] corrector -> cmd_vel_rl -> (Pi) opencr_driver
PC가 없거나 WiFi가 끊기면 cmd_vel_rl이 안 와서 바퀴가 선다 (안전 쪽).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    tb_only = os.path.join(get_package_share_directory('tb3_bringup'), 'launch', 'tb-only.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/opencr'),
        DeclareLaunchArgument('lidar', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tb_only),
                                 launch_arguments={'rl': 'true', 'port': LaunchConfiguration('port'),
                                                   'lidar': LaunchConfiguration('lidar')}.items()),
    ])
