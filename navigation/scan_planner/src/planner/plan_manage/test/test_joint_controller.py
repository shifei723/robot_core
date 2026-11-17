"""Exercise planner-owned controls on the actual controller, with no actuators."""
import os,subprocess,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header,Float64MultiArray,String
from scan_planner_msgs.msg import Bspline,DynamicObstacleArray,DynamicObstacle

def test_joint_sequence_requires_matching_fresh_safe_inputs():
 rclpy.init();n=Node('joint_sequence_test');commands=[];phases=[];states=[]
 n.create_subscription(Float64MultiArray,'/planning/execution_phase',lambda m:phases.append(list(m.data)),10)
 n.create_subscription(String,'/height_control/state',lambda m:states.append(m.data),10)
 exe=os.path.join(get_package_prefix('scan_planner'),'lib','scan_planner','closed_loop_controller')
 proc=subprocess.Popen([exe,'--ros-args','-p','joint_rollout_required:=true','-p','frs_spatial_tracking:=true',
  '-p','height_safety_enabled:=false','-p','dynamic_input:=array','-p','dynamic_prediction:=frs','-p','max_vx:=0.5',
  '-r','cmd_vel:=/joint_test/cmd'])
 n.create_subscription(Twist,'/joint_test/cmd',lambda m:commands.append((m.linear.x,m.angular.z)),10)
 op=n.create_publisher(Odometry,'/body_pose',10);sp=n.create_publisher(Bspline,'/planning/bspline',10)
 mp=n.create_publisher(PointCloud2,'/grid_map/occupancy',10);tp=n.create_publisher(DynamicObstacleArray,'/dynamic_obstacles/tracks',10)
 jp=n.create_publisher(Float64MultiArray,'/planning/joint_rollout',QoSProfile(depth=8,durability=DurabilityPolicy.TRANSIENT_LOCAL))
 od=Odometry();od.header.frame_id='world';od.pose.pose.orientation.w=1.;od.pose.pose.position.z=.25
 spline=Bspline();spline.order=3;spline.traj_id=1;spline.pos_pts=[Point(x=i*.2,y=0.,z=.25) for i in range(15)];spline.knots=[float(i)*.3 for i in range(-3,16)]
 fixed=None
 def pump(seconds,ident=None,healthy=True,blocked=False,fresh_plan=True):
  nonlocal fixed
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   stamp=n.get_clock().now().to_msg();now=n.get_clock().now().nanoseconds*1e-9;od.header.stamp=stamp;op.publish(od)
   header=Header();header.frame_id='world';header.stamp=stamp;mp.publish(point_cloud2.create_cloud_xyz32(header,[]))
   a=DynamicObstacleArray();a.header=header;a.healthy=healthy;a.epoch=1
   if blocked:
    z=DynamicObstacle();z.id='unknown';z.state=z.UNKNOWN;z.position.x=.1;z.radius=.5;z.last_observed=stamp;a.obstacles=[z]
   tp.publish(a)
   if ident is not None:
    if fixed is None or (fresh_plan and now-fixed>.35):fixed=now
    msg=Float64MultiArray();msg.data=[1.,float(ident),fixed,1.,1.,.4,8.]+[.3,.5]*8;jp.publish(msg)
   rclpy.spin_once(n,timeout_sec=.015)
 try:
  pump(.5);sp.publish(spline);pump(.25);assert commands and commands[-1]==(0.,0.)
  pump(.2,99);assert commands[-1]==(0.,0.),'Unrelated trajectory cannot unlock motion'
  commands.clear();pump(.7,1);assert any(v>.02 and w>.02 for v,w in commands),'Joint translation and turn must execute from zero'
  pump(.25,1,healthy=False);assert commands[-1]==(0.,0.),'Unhealthy perception must stop'
  pump(.25,1,blocked=True);assert commands[-1]==(0.,0.),'UNKNOWN cannot be crossed'
  fixed=None;pump(.2,1);pump(1.,1,fresh_plan=False);assert commands[-1]==(0.,0.),'A stale sequence cannot continue moving'
  od.pose.pose.position.x=2.55;phases.clear();states.clear();pump(.3,1,fresh_plan=False)
  assert 'GOAL_REACHED' in states and commands[-1]==(0.,0.),'Measured endpoint capture must finish even when the previous sequence has expired'
  assert any(p[0]==1 and p[1]>=3.59 and p[2]==0. for p in phases),'Completion must publish the end phase to the FSM'
 finally:
  proc.terminate()
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  n.destroy_node();rclpy.shutdown()
