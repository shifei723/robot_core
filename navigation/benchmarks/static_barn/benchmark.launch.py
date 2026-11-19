"""Isolated static BARN comparison; original repository remains unchanged."""
import importlib.util
import os
import sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory,get_package_prefix
import yaml
sys.path.insert(0,str(Path(get_package_prefix("dynabarn_ros2"))/"lib/dynabarn_ros2"))
from localization_profiles import prepare_setup

def setup(context):
    method = LaunchConfiguration('method').perform(context)
    idx = int(LaunchConfiguration('world_idx').perform(context))
    if method not in ('mppi', 'slash') or not 0 <= idx < 300:
        raise ValueError('method must be mppi/slash and world_idx must be 0..299')
    pkg = Path(get_package_share_directory('jackal_helper'))
    # Isolate generated configuration; use the same encoder/gyro fusion as DynaBARN.
    shared_setup=prepare_setup(Path(LaunchConfiguration('setup_path').perform(context)),'velocity_imu')
    nav=yaml.safe_load((shared_setup/'nav2.yaml').read_text())
    for node in ['controller_server','bt_navigator','velocity_smoother']:
        nav[node]['ros__parameters']['odom_topic']='/platform/odom/filtered'
    nav['velocity_smoother']['ros__parameters'].update(max_velocity=[.5,0.,1.],min_velocity=[-.35,0.,-1.])
    (shared_setup/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
    context.launch_configurations['setup_path']=str(shared_setup)
    spec = importlib.util.spec_from_file_location('upstream', pkg/'launch/BARN_runner.launch.py')
    upstream = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(upstream)
    actions = upstream.launch_ros_gazebo(context) + upstream.spawn_jackal(context)
    if method == 'mppi':
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/nav2_bringup.launch.py')),
            launch_arguments={'use_sim_time':'true','setup_path':LaunchConfiguration('setup_path'),
                              'scan_topic':'/front/scan','nav2_params_file':'nav2.yaml','log_level':'WARN'}.items()))
    else:
        src = Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/config'
        p = yaml.safe_load((src/'planner.yaml').read_text())['scan_planner_node']['ros__parameters']
        p.update({'use_sim_time':True, 'fsm.static_corridor_reuse':True, 'grid_map.frame_id':'odom', 'grid_map.world_bound_enable':False,
                  'grid_map.height_min':0.249, 'grid_map.height_max':0.251,
                  'grid_map.double_cylinder_radius':0.459, 'grid_map.double_cylinder_offset':0.0,
                  'optimization.height_min':0.25, 'optimization.height_max':0.25,
                  'optimization.height_nominal':0.25, 'manager.wheel_grid_trajectory':True,
                  'manager.wheel_search_timeout':.1, 'manager.wheel_search_heuristic':1.8,
                  'manager.static_tracking_margin':.125, 'manager.shortcut_wheel_path':True, 'manager.local_wheel_timing':True, 'manager.execution_radius':0.334, 'manager.execution_offset':0.0, 'manager.max_vel':0.5, 'optimization.max_vel':0.5})
        actions += [Node(package='scan_planner', executable='scan_planner_node', parameters=[p],
                         remappings=[('body_pose','/benchmark/body_pose'),('sensor_pose','/benchmark/sensor_pose'),('cloud','/benchmark/cloud')]),
                    Node(package='scan_planner', executable='closed_loop_controller', parameters=[{
                        'use_sim_time':True,'static_body_radius':.334,'static_body_offset':0.,'static_spatial_tracking':True,'static_acceleration_limit':.5,'static_lateral_feedback':True,'max_vx':0.5,'max_vy':0.0,'max_vyaw':1.0,'finish_dist':0.1,'dynamic_prediction':'off',
                        'leg_height_min':0.2178,'leg_height_max':0.2178,'leg_height_default':0.2178}],
                         remappings=[('body_pose','/benchmark/body_pose'),('cmd_vel','/benchmark/cmd_vel')])]
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('method',default_value='mppi'),DeclareLaunchArgument('world_idx',default_value='0'),
        DeclareLaunchArgument('gui',default_value='false'),DeclareLaunchArgument('rviz',default_value='false'),
        DeclareLaunchArgument('setup_path',default_value=str(Path(get_package_share_directory('jackal_helper'))/'config')),
        OpaqueFunction(function=setup)])
