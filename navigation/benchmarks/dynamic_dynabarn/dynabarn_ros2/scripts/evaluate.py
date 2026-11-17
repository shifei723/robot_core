#!/usr/bin/env python3
"""Native ROS 2 DynaBARN integration/evaluation. Truth never goes to Nav2."""
import argparse,csv,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from ros_gz_interfaces.srv import ControlWorld,SetEntityPose
from nav2_msgs.action import NavigateToPose

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Eval(Node):
 def __init__(self,a):
  super().__init__('dynabarn_evaluation',automatically_declare_parameters_from_overrides=True)
  self.a=a;self.dest=Path(a.output);self.dest.mkdir(parents=True,exist_ok=False)
  self.truth=None;self.odom=None;self.scan=None;self.objects={};self.poses=[];self.rows=[];self.scan_checks=[]
  self.touched=False;self.start=None;self.done=False;self.goal_future=None;self.result_future=None;self.phase='waiting';self.phase_wall=time.monotonic();self.wall=time.monotonic();self.last_progress=-99.;self.samples={};self.checks={}
  self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  self.create_subscription(TFMessage,'/model/robot/pose',self.on_truth,10)
  self.create_subscription(TFMessage,'/dynabarn/ground_truth',self.on_objects,100)
  self.create_subscription(Odometry,'/platform/odom/filtered',lambda m:setattr(self,'odom',m),qos_profile_sensor_data)
  self.create_subscription(LaserScan,'/front/scan',self.on_scan,qos_profile_sensor_data)
  self.create_subscription(Bool,'/robot/touched',self.on_touch,10)
  self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
  self.control=self.create_client(ControlWorld,'/world/default/control');self.set_pose=self.create_client(SetEntityPose,'/world/default/set_pose')
  self.timer=self.create_timer(.1,self.tick)
 def on_truth(self,m):
  for t in m.transforms:
   if t.child_frame_id=='robot':self.truth=t.transform
 def on_touch(self,m):
  if self.start is not None and m.data:self.touched=True
 def on_objects(self,m):
  for t in m.transforms:
   name=t.child_frame_id.split('/')[-1]
   self.objects[name]=[t.transform.translation.x,t.transform.translation.y,t.transform.translation.z]
   stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9
   self.poses.append([stamp,name,*self.objects[name]])
   self.samples.setdefault(name,[]).append(self.objects[name][:2])
 def on_scan(self,m):
  self.scan=m
  if self.truth is None or self.odom is None or len(self.objects)<8:return
  try:trans=self.tf.lookup_transform('odom',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
  except Exception:return
  # Analytic foreground-ray validation of moving collision/visual geometry.
  w=self.truth;o=self.odom.pose.pose;angle=yaw(w.rotation)-yaw(o.orientation)
  R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
  origin=np.array([w.translation.x,w.translation.y])+R@(np.array([trans.translation.x,trans.translation.y])-[o.position.x,o.position.y])
  theta=m.angle_min+np.arange(len(m.ranges))*m.angle_increment+angle+yaw(trans.rotation)
  directions=np.c_[np.cos(theta),np.sin(theta)];expected=np.full(len(theta),np.inf)
  for p in self.objects.values():
   delta=np.array(p[:2])-origin;b=directions@delta;disc=b*b-delta@delta+.5**2
   hit=b-np.sqrt(np.maximum(0,disc));expected=np.minimum(expected,np.where((disc>0)&(hit>m.range_min),hit,np.inf))
  ranges=np.asarray(m.ranges)
  valid=np.isfinite(expected)&np.isfinite(ranges)&(expected<min(18.,m.range_max))&(ranges>.6)
  # Closer hits can be walls; compare rays whose observed endpoint is near the
  # dynamic foreground, and report the inlier fraction as well as the error.
  error=np.abs(ranges[valid]-expected[valid]);near=error<.12
  self.scan_checks.append([float(near.sum()),float(len(error)),float(error[near].sum())])
 def on_touch_summary(self):return {'contact_seen':self.touched}
 def now_s(self):return self.get_clock().now().nanoseconds/1e9
 def save(self,status):
  if self.done:return
  self.done=True
  for name,header,rows in [('robot',['sim_elapsed_s','world_x','world_y','yaw','odom_vx','odom_wz'],self.rows),('objects',['sim_s','name','world_x','world_y','world_z'],self.poses)]:
   with (self.dest/(name+'.csv')).open('w') as f:w=csv.writer(f);w.writerow(header);w.writerows(rows)
  ranges={k:float(np.linalg.norm(np.ptp(v,axis=0))) for k,v in self.samples.items()}
  check=np.sum(self.scan_checks,axis=0) if self.scan_checks else np.zeros(3)
  profile=getattr(self,'protocol',None)
  result={'status':status,'mode':self.a.mode,'sim_elapsed_s':self.now_s()-self.start if self.start is not None else None,
   'start_sim_s':self.start,'finish_sim_s':self.now_s(),'collision':self.touched,'models_seen':len(self.objects),'motion_ranges_m':ranges,
   'foreground_lidar':{'inlier_rays':int(check[0]),'tested_rays':int(check[1]),'mean_inlier_error_m':float(check[2]/check[0]) if check[0] else None},
   'checks':self.checks,'goal_world':[profile.goal_x if profile else -9.,0.],'start_world':[profile.start_x if profile else 12.,0.,profile.start_yaw if profile else math.pi],'goal_tolerance_m':profile.radius if profile else .3,
   'plant':'ROS2 Jazzy / Gazebo Harmonic / Jackal; native lidar, odometry, moving collisions',
   'scope':'Port functional verification; not official leaderboard reproduction; ground truth evaluation only'}
  (self.dest/'result.json').write_text(json.dumps(result,indent=2)+'\n');self.get_logger().info(json.dumps(result))
 def tick(self):
  if self.done:return
  now=self.now_s()
  if self.truth is None or self.odom is None or self.scan is None or len(self.objects)<8:
   if time.monotonic()-self.phase_wall>15:
    self.get_logger().info(f'Ready? robot={self.truth is not None}, odom={self.odom is not None}, scan={self.scan is not None}, moving_models={len(self.objects)}, sim={now:.1f}')
    self.phase_wall=time.monotonic()
   return
  if self.start is None:
   if now<5 or (self.a.mode=='mppi' and not self.action.server_is_ready()):return
   self.start=now;self.phase='moving';self.phase_wall=time.monotonic()
   if self.a.mode=='mppi':
    w=self.truth;o=self.odom.pose.pose;angle=yaw(o.orientation)-yaw(w.rotation);dx=-9.-w.translation.x;dy=-w.translation.y
    p=PoseStamped();p.header.frame_id='odom';p.header.stamp=self.get_clock().now().to_msg()
    p.pose.position.x=o.position.x+math.cos(angle)*dx-math.sin(angle)*dy;p.pose.position.y=o.position.y+math.sin(angle)*dx+math.cos(angle)*dy
    p.pose.orientation.z=math.sin((math.pi+angle)/2);p.pose.orientation.w=math.cos((math.pi+angle)/2)
    req=NavigateToPose.Goal();req.pose=p;self.goal_future=self.action.send_goal_async(req)
   self.get_logger().info('Native DynaBARN evaluation started')
  elapsed=now-self.start;w=self.truth
  self.rows.append([elapsed,w.translation.x,w.translation.y,yaw(w.rotation),self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z])
  if elapsed-self.last_progress>10:
   self.last_progress=elapsed;self.get_logger().info(f'Progress {elapsed:.1f}s, position ({w.translation.x:.2f},{w.translation.y:.2f}), models={len(self.objects)}')
  if self.a.mode=='smoke':
   if elapsed>8 and self.phase=='moving':
    self.checks['eight_models_move']=all(np.linalg.norm(np.ptp(v,axis=0))>.10 for v in self.samples.values())
    self.save('smoke_motion_complete')
  elif self.a.mode=='mppi':
   if self.touched:self.save('collision')
   elif math.hypot(w.translation.x+9,w.translation.y)<.3:self.save('success')
   elif elapsed>self.a.timeout:self.save('timeout')
   elif self.goal_future and self.goal_future.done():
    handle=self.goal_future.result()
    if not handle.accepted:self.save('goal_rejected')
    elif self.result_future is None:self.result_future=handle.get_result_async()
    elif self.result_future.done() and self.result_future.result().status in [5,6]:self.save('navigation_aborted')

def main():
 p=argparse.ArgumentParser();p.add_argument('--mode',choices=['mppi','none','smoke'],default='mppi');p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=150)
 a,ros=p.parse_known_args();rclpy.init(args=ros);n=Eval(a)
 try:
  while rclpy.ok() and not n.done:
   rclpy.spin_once(n,timeout_sec=.1)
   if time.monotonic()-n.wall>420:n.save('infrastructure_timeout')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
