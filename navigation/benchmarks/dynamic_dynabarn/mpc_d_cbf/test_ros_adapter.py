import os,subprocess,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry,OccupancyGrid
from scan_planner_msgs.msg import DynamicObstacleArray,DynamicObstacle

def test_real_mpc_node_empty_scene_progress_and_stale_infeasible_stop():
 rclpy.init();n=Node('mpc_adapter_gate_test');commands=[]
 n.create_subscription(Twist,'/benchmark/cmd_vel',lambda m:commands.append((m.linear.x,m.angular.z)),10)
 op=n.create_publisher(Odometry,'/benchmark/body_pose',10);tp=n.create_publisher(DynamicObstacleArray,'/dynamic_obstacles/tracks',10)
 mp=n.create_publisher(OccupancyGrid,'/benchmark/static_map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL));gp=n.create_publisher(PoseStamped,'/move_base_simple/goal',10)
 path=Path(os.environ['BENCH_WS'])/'install/dynabarn_ros2/lib/dynabarn_ros2/mpc_d_cbf_node.py'
 proc=subprocess.Popen(['python3',str(path)])
 def pump(seconds,tracks=True,healthy=True,blocked=False):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   stamp=n.get_clock().now().to_msg();o=Odometry();o.header.frame_id='world';o.header.stamp=stamp;o.pose.pose.orientation.w=1.;op.publish(o)
   grid=OccupancyGrid();grid.header.frame_id='odom';grid.info.resolution=.1;grid.info.width=grid.info.height=10;grid.data=[0]*100;mp.publish(grid)
   if tracks:
    a=DynamicObstacleArray();a.header.frame_id='world';a.header.stamp=stamp;a.healthy=healthy;a.epoch=1
    if blocked:
     z=DynamicObstacle();z.id='blocker';z.state=z.UNKNOWN;z.position.x=.5;z.radius=.5;z.last_observed=stamp;a.obstacles=[z]
    tp.publish(a)
   rclpy.spin_once(n,timeout_sec=.02);time.sleep(.005)
 try:
  pump(1.);assert commands and commands[-1]==(0.,0.)
  goal=PoseStamped();goal.header.frame_id='world';goal.pose.position.x=3.;goal.pose.orientation.w=1.;gp.publish(goal);commands.clear();pump(1.2)
  assert any(v>.1 for v,w in commands),'Actual optimiser should authorise progress with fresh empty input'
  pump(.6,tracks=False);commands.clear();pump(.2,tracks=False);assert commands and all(abs(v)+abs(w)<1e-8 for v,w in commands)
  pump(.4,healthy=False);assert commands[-1]==(0.,0.)
  pump(1.,blocked=True);commands.clear();pump(.3,blocked=True);assert commands and all(abs(v)+abs(w)<1e-8 for v,w in commands),'Infeasible CBF must not replay the previous successful command'
 finally:
  proc.terminate()
  try:proc.wait(timeout=5)
  except subprocess.TimeoutExpired:proc.kill();proc.wait()
  n.destroy_node();rclpy.shutdown()
