from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fast_lio_share = FindPackageShare('fast_lio')
    open3d_loc_share = FindPackageShare('open3d_loc')

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fast_lio_share, 'launch', 'mapping.launch.py'])]),
        launch_arguments={'rviz': 'false'}.items())

    open3d_loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                open3d_loc_share, 'launch', 'open3d_loc_g1.launch.py'])]))

    slash_share = get_package_share_directory('wheel_robot_withlidar')
    urdf_path = os.path.join(slash_share, 'urdf', 'wheel_robot_withlidar.urdf')
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'publish_frequency': 30.0}])

    joint_state_publisher = Node(
        package='joint_state_publisher', executable='joint_state_publisher')

    # Connect FAST-LIO body(IMU) frame to the slash robot root (base_footprint).
    # Default identity; set to your IMU->base_footprint mount offset if needed.
    static_tf_body2base = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='body2base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'body', 'base_footprint'])

    rviz_config_path = PathJoinSubstitution([
        open3d_loc_share, 'rviz_cfg', 'fastlio.rviz'])
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz_map_cur',
        arguments=['-d', rviz_config_path], output='screen', prefix='nice')

    return LaunchDescription([
        fast_lio_launch, open3d_loc_launch,
        robot_state_publisher, joint_state_publisher,
        static_tf_body2base, rviz_node])
