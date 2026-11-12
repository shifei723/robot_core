#!/usr/bin/env python3
from pathlib import Path
import time,subprocess,os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point,Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray,Header
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from scan_planner_msgs.msg import Bspline
rclpy.init();n=Node('spatial_controller_integration_test')
commands=[];phases=[]
n.create_subscription(Twist,'cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)),1)
n.create_subscription(Float64MultiArray,'planning/execution_phase',lambda m:phases.append((time.monotonic(),list(m.data))),1)
map_pub=n.create_publisher(PointCloud2,'grid_map/occupancy',1)
odom=n.create_publisher(Odometry,'body_pose',10);path=n.create_publisher(Bspline,'planning/bspline',10)
p=subprocess.Popen([os.environ.get('SPATIAL_CONTROLLER_BIN',str(Path(os.environ['BENCH_WS'])/'install/scan_planner/lib/scan_planner/closed_loop_controller')),'--ros-args','-p','static_spatial_tracking:=true','-p','max_vx:=0.5','-p','max_vy:=0.0','-p','data_timeout:=0.2'],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
position=0.
def run(seconds,feed=True,move=False,invalid=False):
 global position
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  if feed:
   if move:position+=.001
   m=Odometry();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='world';m.pose.pose.position.x=float('nan') if invalid else position;m.pose.pose.position.z=.25;m.pose.pose.orientation.w=1.;odom.publish(m)

  map_pub.publish(create_cloud_xyz32(Header(stamp=n.get_clock().now().to_msg(),frame_id='world'),[]))
  for _ in range(4):rclpy.spin_once(n,timeout_sec=0.)
  time.sleep(.005)
def publish(ident,height=False):
 m=Bspline();m.order=3;m.traj_id=ident;m.start_time=n.get_clock().now().to_msg()
 xs=[position]*3+[position+i*.05 for i in range(1,101)]+[position+5]*2
 m.pos_pts=[Point(x=x,y=0.,z=.25+(i*.001 if height else 0.)) for i,x in enumerate(xs)]
 m.knots=[(i-3)*.1 for i in range(len(xs)+4)];path.publish(m)
try:
 run(1.0);assert path.get_subscription_count()>0
 publish(1);run(1.5,move=True)
 assert commands and max(x[1] for x in commands[-30:])>.05,'controller failed to start'
 before=commands[-1][1];publish(2);run(.15,move=True)
 assert commands[-1][1]>=before-.1,'new trajectory reset velocity'
 run(.4,move=True);assert phases[-1][1][1]>0,'execution phase did not advance'
 phase=phases[-1][1][1];run(.8,feed=False)
 assert abs(commands[-1][1])<1e-8,'stale odometry did not stop'
 assert abs(phases[-1][1][1]-phase)<.2,'stale odometry advanced execution clock'
 assert all(len(x[1])==4 for x in phases),'feedback missing speed/acceleration'
 run(.3);run(.1,invalid=True);assert abs(commands[-1][1])<1e-8,'invalid odometry did not stop'
 run(.04);assert commands[-1][1]<.08,'resume after invalid odometry jumped in speed'
 publish(3,height=True);start=time.monotonic();run(.4)
 assert not any(v[0]>start+.1 and v[1][0]==3 for v in phases),'height-changing path used planar tracker'
 print('PASS: live ROS controller continuity, stale-odom stop/phase hold, height-mode fallback')
finally:
 p.terminate()
 try:p.wait(timeout=3)
 except subprocess.TimeoutExpired:p.kill();p.wait()
 n.destroy_node();rclpy.shutdown()
