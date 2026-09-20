"""Start RViz2 with the SCAN-Planner display configuration."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


RViz_CFGS = {
    "go2": "default.rviz",
    "slash": "slash.rviz",
}


def _setup(context):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    cfg = RViz_CFGS.get(robot_model, "default.rviz")
    config = os.path.join(get_package_share_directory("scan_planner"), "rviz", cfg)
    return [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", config],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value="go2"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
