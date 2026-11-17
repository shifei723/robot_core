"""Native Jazzy/Harmonic DynaBARN. Ground-truth topics are evaluation-only."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription,SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetRemap
from ament_index_python.packages import get_package_share_directory,get_package_prefix

def setup(c):
 pkg=Path(get_package_share_directory('dynabarn_ros2'))
 world=LaunchConfiguration('world_file').perform(c)
 world=Path(world) if world else pkg/'worlds/official_world_2.world'
 if not world.is_file():raise FileNotFoundError(world)
 gui=LaunchConfiguration('gui').perform(c)=='true'
 actions=[SetEnvironmentVariable('DYNABARN_PHASE',LaunchConfiguration('motion_phase').perform(c)),SetEnvironmentVariable('GZ_SIM_SYSTEM_PLUGIN_PATH',str(Path(get_package_prefix('dynabarn_ros2'))/'lib')+os.pathsep+os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')),
  IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ros_gz_sim'))/'launch/gz_sim.launch.py')),
   launch_arguments={'gz_args':str(world)+' -r -v 2'+('' if gui else ' -s --headless-rendering')}.items()),
  Node(package='ros_gz_bridge',executable='parameter_bridge',name='dynabarn_bridge',arguments=[
   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/robot/touched@std_msgs/msg/Bool[gz.msgs.Boolean',
   '/world/default/control@ros_gz_interfaces/srv/ControlWorld','/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose',
   '/model/robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
   '/dynabarn/ground_truth@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],parameters=[{'use_sim_time':True}],
   ros_arguments=['--log-level','warn']),
  SetRemap(src='/sensors/lidar2d_0/scan',dst='/front/scan'),
  IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('clearpath_gz'))/'launch/robot_spawn.launch.py')),
   launch_arguments={'use_sim_time':'true','setup_path':LaunchConfiguration('setup_path'),'world':'default','rviz':'false','generate':'true','x':LaunchConfiguration('spawn_x'),'y':'0.0','z':'0.1','yaw':LaunchConfiguration('spawn_yaw')}.items())]
 if LaunchConfiguration('navigation').perform(c)=='mppi':
  actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(Path(get_package_share_directory('jackal_helper'))/'launch/nav2_bringup.launch.py')),
   launch_arguments={'use_sim_time':'true','setup_path':LaunchConfiguration('setup_path'),'scan_topic':'/front/scan','nav2_params_file':'nav2.yaml','log_level':'WARN'}.items()))
 if LaunchConfiguration('evaluate').perform(c)=='true':
  actions.append(Node(package='dynabarn_ros2',executable='evaluate.py',parameters=[{'use_sim_time':True}],arguments=['--output',LaunchConfiguration('output'),'--mode',LaunchConfiguration('navigation')]))
 return actions

def generate_launch_description():
 return LaunchDescription([DeclareLaunchArgument('spawn_x',default_value='12.0'),DeclareLaunchArgument('spawn_yaw',default_value='3.141592653589793'),DeclareLaunchArgument('motion_phase',default_value='0'),DeclareLaunchArgument('world_file',default_value=''),DeclareLaunchArgument('gui',default_value='false'),
  DeclareLaunchArgument('navigation',default_value='mppi',choices=['mppi','none']),DeclareLaunchArgument('evaluate',default_value='true'),
  DeclareLaunchArgument('setup_path',default_value=str(Path(get_package_share_directory('dynabarn_ros2'))/'config')),
  DeclareLaunchArgument('output',default_value='/tmp/dynabarn_ros2_result'),OpaqueFunction(function=setup)])
