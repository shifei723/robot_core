"""Reproducible low-lintel wheel-height experiment (simulation only)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('scan_planner')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('beam_clearance', default_value='0.62'),
        DeclareLaunchArgument('leg_height_rate', default_value='0.08'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'run.launch.py')),
            launch_arguments={'robot_model': 'slash', 'is_real_world': 'false',
                'controller_mode': 'closed_loop', 'sensor_type': 'lidar', 'use_gpu': 'false',
                'dynamic_obstacles': 'false', 'init_x': '0.0', 'init_y': '-2.0', 'init_yaw': '1.5707963267948966',
                'beam_clearance': LaunchConfiguration('beam_clearance'),
                'leg_height_rate': LaunchConfiguration('leg_height_rate')}.items()),
        Node(package='scan_planner', executable='height_experiment_goal.py', output='screen'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'rviz.launch.py')),
            launch_arguments={'robot_model': 'slash', 'use_sim_time': 'false'}.items(),
            condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
