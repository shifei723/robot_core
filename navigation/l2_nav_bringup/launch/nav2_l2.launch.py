import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    bringup = get_package_share_directory('l2_nav_bringup')
    params = os.path.join(bringup, 'params', 'nav2_l2.yaml')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items())

    return LaunchDescription([navigation])
