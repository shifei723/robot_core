"""Real ROS2 regression: a single scan arriving BEFORE TF must be recovered."""
import sys,time,math
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from scan_planner_msgs.msg import DynamicObstacleArray
from tf2_ros import StaticTransformBroadcaster
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT/'scripts'))
from laser_perception import Perception

rclpy.init(args=['--ros-args','-p',f'world_file:={ROOT}/worlds/official_world_2.world'])
perception=Perception();driver=Node('delayed_laser_driver');ex=SingleThreadedExecutor();ex.add_node(perception);ex.add_node(driver)
pub=driver.create_publisher(LaserScan,'/front/scan',10);odom=driver.create_publisher(Odometry,'/platform/odom/filtered',10);tf=StaticTransformBroadcaster(driver);arrays=[]
driver.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',arrays.append,10)
def pump(duration):
 end=time.monotonic()+duration
 while time.monotonic()<end:
  m=Odometry();m.header.frame_id='odom';m.header.stamp=driver.get_clock().now().to_msg();m.pose.pose.orientation.w=1.;odom.publish(m)
  ex.spin_once(timeout_sec=.002)
try:
 pump(.7)
 scan=LaserScan();scan.header.stamp=driver.get_clock().now().to_msg();scan.header.frame_id='delayed_test_laser'
 scan.angle_min=-math.pi*3/4;scan.angle_max=math.pi*3/4;scan.angle_increment=(scan.angle_max-scan.angle_min)/719;scan.range_min=.02;scan.range_max=20.
 angle=scan.angle_min+np.arange(720)*scan.angle_increment;b=1.5*np.cos(angle);disc=b*b-1.5**2+.5**2
 scan.ranges=np.where((disc>=0)&(b>0),b-np.sqrt(np.maximum(disc,0)),np.inf).astype(np.float32).tolist()
 pub.publish(scan);pump(.08)
 assert perception.pending is not None,'Untransformable scan must remain pending'
 assert not any(a.healthy for a in arrays),'Missing TF must not produce healthy-empty'
 tr=TransformStamped();tr.header.frame_id='odom';tr.child_frame_id='delayed_test_laser';tr.transform.rotation.w=1.;tf.sendTransform(tr)
 pump(.18)
 healthy=[a for a in arrays if a.healthy]
 assert healthy and healthy[-1].obstacles,'The original scan must be processed after delayed TF'
 obj=healthy[-1].obstacles[0];assert abs(obj.position.x-1.5)<.05 and abs(obj.position.y)<.05
 pump(.5)
 assert not arrays[-1].healthy,'A truly stale sensor must still revoke health'
 print('PASS: delayed exact-stamp TF recovered, no healthy-empty fabrication, center fit, and sensor dropout stop preserved.')
finally:
 ex.shutdown();perception.destroy_node();driver.destroy_node();rclpy.shutdown()
