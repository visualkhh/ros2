"""KOS robot launch — tb3_bringup의 explore-nav을 kos_robot의 MPPI config로 실행."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bringup_share = get_package_share_directory('tb3_bringup')
    kos_share = get_package_share_directory('kos_robot')
    params_file = os.path.join(kos_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'explore-nav.launch.py')
            ),
            launch_arguments={'params_file': params_file}.items(),
        ),
    ])
