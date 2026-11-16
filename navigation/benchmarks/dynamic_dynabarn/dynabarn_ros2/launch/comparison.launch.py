"""MPPI / FRS soft / FRS hard with common native lidar and static background."""
from pathlib import Path
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def setup(c):
 pkg=Path(get_package_share_directory('dynabarn_ros2'));method=LaunchConfiguration('method').perform(c)
 world=pkg/'worlds/official_world_2.world'
 actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/dynabarn.launch.py')),
  launch_arguments={'navigation':'mppi' if method=='mppi' else 'none','evaluate':'false','gui':'false',
   'motion_phase':LaunchConfiguration('motion_phase'),'setup_path':str(pkg/'comparison_config')}.items()),
  Node(package='dynabarn_ros2',executable='laser_perception.py',parameters=[{'use_sim_time':True,'world_file':str(world),'lost_timeout':ParameterValue(LaunchConfiguration('lost_timeout'),value_type=float)}])]
 if method!='mppi':
  planner=Path(get_package_share_directory('scan_planner'))
  p=yaml.safe_load((planner/'config/planner.yaml').read_text())['scan_planner_node']['ros__parameters']
  p.update({'use_sim_time':True,'grid_map.frame_id':'world','grid_map.world_bound_enable':False,
   'grid_map.static_prior_enabled':True,'grid_map.height_min':.249,'grid_map.height_max':.251,
   'grid_map.sliding_map_size_x':12.,'grid_map.sliding_map_size_y':12.,'grid_map.sliding_map_size_z':1.5,
   'grid_map.double_cylinder_radius':.459,'grid_map.double_cylinder_offset':0.,'grid_map.max_ray_length':5.,
   'optimization.height_min':.25,'optimization.height_max':.25,'optimization.height_nominal':.25,
   'manager.wheel_grid_trajectory':True,'manager.execution_radius':.334,'manager.execution_offset':0.,
   'manager.static_tracking_margin':.125,'manager.shortcut_wheel_path':True,'manager.local_wheel_timing':True,
   'manager.dynamic_frs_weight':.5,'manager.max_vel':.5,'optimization.max_vel':.5,
   'fsm.static_corridor_reuse':True,'fsm.planning_horizon':4.5,'manager.planning_horizon':4.5,
   'fsm.replan_interval':.4,'manager.wheel_search_timeout':.1,'manager.wheel_search_heuristic':1.8})
  actions.extend([Node(package='scan_planner',executable='scan_planner_node',parameters=[p],
    remappings=[('body_pose','/benchmark/body_pose'),('sensor_pose','/benchmark/sensor_pose'),('cloud','/benchmark/static_cloud')]),
   Node(package='scan_planner',executable='closed_loop_controller',parameters=[{
    'use_sim_time':True,'static_spatial_tracking':True,'frs_spatial_tracking':True,'static_body_radius':.334,
    'static_body_offset':0.,'static_acceleration_limit':.5,'static_prior_required':True,
    'static_cruise_speed':.5,'max_vx':.5,'max_vy':0.,'max_vyaw':1.,'finish_dist':.10,
    'leg_height_min':.2178,'leg_height_max':.2178,'leg_height_default':.2178,
    'dynamic_prediction':'frs','dynamic_input':'array','dynamic_soft_constraint':method=='frs_soft',
    'dynamic_frs_weight':2.,'dynamic_horizon':3.,'dynamic_safety_margin':.20,
    'dynamic_braking_deceleration':.5}],
    remappings=[('body_pose','/benchmark/body_pose'),('cmd_vel','/benchmark/cmd_vel')])])
 return actions

def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('lost_timeout',default_value='1.0'),DeclareLaunchArgument('method',default_value='mppi',choices=['mppi','frs_soft','frs_hard']),
  DeclareLaunchArgument('motion_phase',default_value='0'),OpaqueFunction(function=setup)])
