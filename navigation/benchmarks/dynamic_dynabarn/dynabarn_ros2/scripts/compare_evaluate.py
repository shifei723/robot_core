#!/usr/bin/env python3
"""Matched dynamic comparison. No obstacle truth is published to any planner."""
import argparse,csv,json,math,time,hashlib
from pathlib import Path
import os
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from geometry_msgs.msg import PoseStamped,Twist,TwistStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String,Float64MultiArray
from scan_planner_msgs.msg import DynamicObstacleArray
from nav2_msgs.action import NavigateToPose
from evaluate import Eval,yaw

class Comparison(Eval):
 def __init__(self,a):
  super().__init__(a);self.anchor=None;self.tracks=None;self.prior_time=-1.;self.states=[];self.decisions=[];self.estimates=[];self.perception=[];self.state='';self.odom_rows=[];self.planner_state='';self.wait_target_since=None
  self.goal_pub=self.create_publisher(PoseStamped,'/move_base_simple/goal',10)
  self.vel_pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
  latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.create_subscription(Float64MultiArray,'/benchmark/anchor',lambda m:setattr(self,'anchor',m.data),latch)
  self.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',self.on_tracks,10)
  self.create_subscription(PointCloud2,'/grid_map/static_prior_applied',lambda m:setattr(self,'prior_time',m.header.stamp.sec+m.header.stamp.nanosec/1e9),10)
  self.create_subscription(Twist,'/benchmark/cmd_vel',self.on_cmd,10)
  self.create_subscription(String,'/height_control/state',lambda m:setattr(self,'state',m.data),10)
  self.create_subscription(String,'/planning/status',lambda m:setattr(self,'planner_state',m.data),10)
  self.create_subscription(String,'/dynamic_navigation/decision',lambda m:self.decisions.append(json.loads(m.data)),10)
  self.create_subscription(String,'/dynamic_obstacles/perception_stats',lambda m:self.perception.append(json.loads(m.data)),10)
 def on_cmd(self,m):
  cmd=TwistStamped();cmd.header.stamp=self.get_clock().now().to_msg();cmd.header.frame_id='base_link';cmd.twist=m
  cmd.twist.linear.x=float(np.clip(cmd.twist.linear.x,-.35,.5));cmd.twist.linear.y=0.;cmd.twist.angular.z=float(np.clip(cmd.twist.angular.z,-1.,1.));self.vel_pub.publish(cmd)
 def on_tracks(self,m):
  self.tracks=m
  self.estimates.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'healthy':m.healthy,'status':m.status,
    'objects':[{'id':o.id,'p':[o.position.x,o.position.y],'v':[o.velocity.x,o.velocity.y],'r':o.radius,'observed':o.observed,'state':o.state} for o in m.obstacles]})
 def tick(self):
  if self.done:return
  now=self.now_s()
  ready=self.truth is not None and self.odom is not None and self.scan is not None and len(self.objects)==8 and self.anchor is not None and self.tracks is not None
  if self.start is None:
   ready=ready and self.tracks.healthy and now-(self.tracks.header.stamp.sec+self.tracks.header.stamp.nanosec/1e9)<.35
   ready=ready and (self.action.server_is_ready() if self.a.method=='mppi' else self.goal_pub.get_subscription_count()>0 and now-self.prior_time<.8)
   if now<8:return
   if not ready:
    if now>8.3:self.save('infrastructure_late_ready')
    return
   if math.hypot(self.truth.translation.x-12,self.truth.translation.y)>.10:self.save('prestart_displacement');return
   self.start=now;self.phase='moving';ax,ay,heading=self.anchor
   goal=PoseStamped();goal.header.stamp=self.get_clock().now().to_msg();goal.header.frame_id='odom' if self.a.method=='mppi' else 'world'
   goal.pose.position.x=ax+21*math.cos(heading);goal.pose.position.y=ay+21*math.sin(heading);goal.pose.position.z=0. if self.a.method=='mppi' else .25
   goal.pose.orientation.z=math.sin(heading/2);goal.pose.orientation.w=math.cos(heading/2)
   if self.a.method=='mppi':req=NavigateToPose.Goal();req.pose=goal;self.goal_future=self.action.send_goal_async(req)
   else:self.goal_pub.publish(goal)
   self.get_logger().info(f'Fixed start: {self.a.method}, sim {now:.3f}, phase {self.a.motion_phase}')
  elapsed=now-self.start;p=self.truth.translation
  self.rows.append([elapsed,p.x,p.y,yaw(self.truth.rotation),self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z])
  self.odom_rows.append([elapsed,self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,yaw(self.odom.pose.pose.orientation)])
  self.states.append([elapsed,self.state,self.planner_state])
  if elapsed-self.last_progress>=10:
   self.last_progress=elapsed;self.get_logger().info(f'{self.a.method}: {elapsed:.1f}s, ({p.x:.2f},{p.y:.2f}), state={self.state}, planner={self.planner_state}, tracks={len(self.tracks.obstacles)}')
  if self.touched:self.save('collision')
  elif math.hypot(p.x+9,p.y)<.3:self.save('success')
  elif elapsed>=self.a.timeout:self.save('timeout')
  elif self.a.method=='mppi' and self.goal_future and self.goal_future.done():
   handle=self.goal_future.result()
   if not handle.accepted:self.save('goal_rejected')
   elif self.result_future is None:self.result_future=handle.get_result_async()
   elif self.result_future.done():
    status=self.result_future.result().status
    if status in (5,6):self.save('navigation_aborted')
    elif status==4:self.save('estimated_goal_only')
 def save(self,status):
  super().save(status)
  result=json.loads((self.dest/'result.json').read_text())
  result.update(method=self.a.method,motion_phase_s=self.a.motion_phase,protocol='dynabarn_ros2_shared_lidar_static_map_v1',
    navigation_anchor=list(self.anchor) if self.anchor is not None else None,timeout_sim_s=self.a.timeout,scheduled_start_sim_s=8.,sensor_range_m=5.,static_background='static wall boxes only; shared with MPPI',truth_input_to_perception=False)
  pkg=Path(os.environ['BENCH_WS'])/'src/dynabarn_ros2';core=Path(os.environ['BENCH_WS'])/'src/slash_planner'
  files=[pkg/'src/motion.cpp',pkg/'launch/comparison.launch.py',pkg/'launch/dynabarn.launch.py',pkg/'scripts/laser_perception.py',pkg/'scripts/map_geometry.py',pkg/'scripts/compare_evaluate.py',pkg/'comparison_config/nav2.yaml',pkg/'worlds/official_world_2.world',core/'geometric_obstacles/geometric_obstacles/core.py',core/'plan_manage/src/closed_loop_controller.cpp',core/'plan_manage/src/planner_manager.cpp',core/'plan_manage/include/plan_manage/intent_frs.hpp',core/'plan_manage/include/plan_manage/spatial_tracker.hpp']
  protocol_hash=hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()
  result['protocol_sha256']=protocol_hash
  result['configuration_sha256']=hashlib.sha256((protocol_hash+json.dumps([self.a.method,self.a.motion_phase,self.a.timeout])).encode()).hexdigest()
  (self.dest/'result.json').write_text(json.dumps(result,indent=2)+'\n')
  (self.dest/'decisions.json').write_text(json.dumps(self.decisions))
  (self.dest/'tracks.json').write_text(json.dumps(self.estimates))
  (self.dest/'perception.json').write_text(json.dumps(self.perception))
  with (self.dest/'odometry.csv').open('w') as f:w=csv.writer(f);w.writerow(['elapsed_s','x','y','yaw']);w.writerows(self.odom_rows)
  with (self.dest/'states.csv').open('w') as f:w=csv.writer(f);w.writerow(['elapsed_s','state','planner']);w.writerows(self.states)

def main():
 p=argparse.ArgumentParser();p.add_argument('--method',choices=['mppi','frs_soft','frs_hard'],required=True);p.add_argument('--motion-phase',type=float,default=0.);p.add_argument('--output',required=True);p.add_argument('--timeout',type=float,default=120.)
 a,ros=p.parse_known_args();a.mode=a.method;rclpy.init(args=ros);n=Comparison(a)
 try:
  while rclpy.ok() and not n.done:
   rclpy.spin_once(n,timeout_sec=.05)
   if time.monotonic()-n.wall>360:n.save('infrastructure_timeout')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
