"""MPPI / FRS soft / FRS hard with common native lidar and static background."""
from pathlib import Path
import yaml
import sys
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory,get_package_prefix
sys.path.insert(0,str(Path(get_package_prefix('dynabarn_ros2'))/'lib/dynabarn_ros2'))
from benchmark_protocol import PROFILES
from localization_profiles import prepare_setup

def setup(c):
 pkg=Path(get_package_share_directory('dynabarn_ros2'));method=LaunchConfiguration('method').perform(c)
 alias_joint=method=='frs_joint'
 if alias_joint:method='frs_soft'
 protocol=PROFILES[LaunchConfiguration('protocol').perform(c)]
 localization=LaunchConfiguration('localization_profile').perform(c)
 setup=prepare_setup(pkg/'comparison_config',localization)
 joint=alias_joint or LaunchConfiguration('joint_rollout').perform(c)=='true'
 if joint and (LaunchConfiguration('active_escape').perform(c)=='true' or LaunchConfiguration('local_dynamic_planner').perform(c)=='true' or method!='frs_soft'):
  raise ValueError('联合滚动规划只与FRS软风险基础版本配对，不叠加旧避险模块')
 if LaunchConfiguration('local_dynamic_planner').perform(c)=='true' and LaunchConfiguration('active_escape').perform(c)=='true':
  raise ValueError('局部规划选择和控制器避险不能同时启用')
 world=pkg/'worlds/official_world_2.world'
 actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(str(pkg/'launch/dynabarn.launch.py')),
  launch_arguments={'navigation':'mppi' if method=='mppi' else 'none','evaluate':'false','gui':'false',
   'motion_phase':LaunchConfiguration('motion_phase'),'spawn_x':str(protocol.start_x),'spawn_yaw':str(protocol.start_yaw),'setup_path':str(setup)}.items()),
  Node(package='dynabarn_ros2',executable='laser_perception.py',parameters=[{'use_sim_time':True,'world_file':str(world),'require_imu':localization=='velocity_imu','start_world_x':protocol.start_x,'start_world_yaw':protocol.start_yaw,'lost_timeout':ParameterValue(LaunchConfiguration('lost_timeout'),value_type=float)}])]
 if method in ('frs_soft','frs_hard'):
  planner=Path(get_package_share_directory('scan_planner'))
  p=yaml.safe_load((planner/'config/planner.yaml').read_text())['scan_planner_node']['ros__parameters']
  p.update({'use_sim_time':True,'grid_map.frame_id':'world','grid_map.world_bound_enable':False,
   'grid_map.static_prior_enabled':True,'grid_map.height_min':.249,'grid_map.height_max':.251,
   'grid_map.sliding_map_size_x':12.,'grid_map.sliding_map_size_y':12.,'grid_map.sliding_map_size_z':1.5,
   'grid_map.double_cylinder_radius':.459,'grid_map.double_cylinder_offset':0.,'grid_map.max_ray_length':5.,
   'optimization.height_min':.25,'optimization.height_max':.25,'optimization.height_nominal':.25,
   'manager.wheel_grid_trajectory':True,'manager.execution_radius':.334,'manager.execution_offset':0.,
   'manager.static_tracking_margin':.125,'manager.shortcut_wheel_path':True,'manager.local_wheel_timing':True,
   'manager.dynamic_local_planning':ParameterValue(LaunchConfiguration('local_dynamic_planner'),value_type=bool),
   'manager.joint_rollout_planning':joint,
   'manager.dynamic_safety_margin':.20,'manager.dynamic_frs_weight':.5,'manager.max_vel':.5,'optimization.max_vel':.5,
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
    'dynamic_braking_deceleration':.5,
    'planned_local_speed_required':ParameterValue(LaunchConfiguration('local_dynamic_planner'),value_type=bool),
    'joint_rollout_required':joint,
    'active_escape_enabled':ParameterValue(LaunchConfiguration('active_escape'),value_type=bool)}],
    remappings=[('body_pose','/benchmark/body_pose'),('cmd_vel','/benchmark/cmd_vel')])])
 if method=='mpc_d_cbf':
  if LaunchConfiguration('local_dynamic_planner').perform(c)=='true' or LaunchConfiguration('active_escape').perform(c)=='true':
   raise ValueError('MPC-D-CBF测试不得同时启用其他规划/控制器')
  actions.append(Node(package='dynabarn_ros2',executable='mpc_d_cbf_node.py',parameters=[{'use_sim_time':True}]))
 return actions

def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('localization_profile',default_value='velocity_imu',choices=['velocity_imu','legacy']),DeclareLaunchArgument('protocol',default_value='challenge',choices=['challenge','legacy']),DeclareLaunchArgument('joint_rollout',default_value='false'),DeclareLaunchArgument('local_dynamic_planner',default_value='false'),DeclareLaunchArgument('active_escape',default_value='false'),DeclareLaunchArgument('lost_timeout',default_value='1.0'),DeclareLaunchArgument('method',default_value='mppi',choices=['mppi','frs_soft','frs_joint','frs_hard','mpc_d_cbf']),
  DeclareLaunchArgument('motion_phase',default_value='0'),OpaqueFunction(function=setup)])
