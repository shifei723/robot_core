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
from benchmark_protocol import PROFILES,REFERENCE_REVISION,outcome,motion_started

class Comparison(Eval):
 def __init__(self,a):
  self.method_label='frs_joint' if a.method=='frs_joint' or (a.method=='frs_soft' and a.joint_rollout=='true') else a.method
  if a.method=='frs_joint':a.method='frs_soft';a.joint_rollout='true'
  self.protocol=PROFILES[a.protocol];self.goal_sent=None;self.startup_rows=[];self.action_events=[];self.action_recorded=False;self.controller_complete=None;self.mppi_velocity_topics=[]
  super().__init__(a);self.anchor=None;self.tracks=None;self.prior_time=-1.;self.states=[];self.decisions=[];self.estimates=[];self.perception=[];self.state='';self.odom_rows=[];self.planner_state='';self.wait_target_since=None;self.goal_odom=None;self.local_plans=[];self.commands=[];self.mpc_decisions=[];self.common_prior_time=-1.
  if a.protocol=='challenge':self.motion_timer=self.create_timer(.01,self.motion_watch)
  self.goal_pub=self.create_publisher(PoseStamped,'/move_base_simple/goal',10)
  self.vel_pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
  latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.create_subscription(Float64MultiArray,'/benchmark/anchor',lambda m:setattr(self,'anchor',m.data),latch)
  self.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',self.on_tracks,10)
  self.create_subscription(PointCloud2,'/grid_map/static_prior_applied',lambda m:setattr(self,'prior_time',m.header.stamp.sec+m.header.stamp.nanosec/1e9),10)
  self.create_subscription(Twist,'/benchmark/cmd_vel',self.on_cmd,10)
  if a.method=='mppi':self.create_subscription(TwistStamped,'/cmd_vel',lambda m:self.commands.append([self.now_s(),m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),10)
  self.create_subscription(String,'/height_control/state',lambda m:setattr(self,'state',m.data),10)
  self.create_subscription(String,'/planning/status',lambda m:setattr(self,'planner_state',m.data),10)
  self.create_subscription(String,'/dynamic_navigation/decision',lambda m:self.decisions.append(json.loads(m.data)),10)
  self.create_subscription(String,'/planning/dynamic_local_decision',lambda m:self.local_plans.append(json.loads(m.data)),10)
  self.create_subscription(String,'/mpc_d_cbf/decision',lambda m:self.mpc_decisions.append(json.loads(m.data)),10)
  self.create_subscription(PointCloud2,'/dynamic_obstacles/static_prior',lambda m:setattr(self,'common_prior_time',m.header.stamp.sec+m.header.stamp.nanosec*1e-9),latch)
  self.create_subscription(String,'/dynamic_obstacles/perception_stats',lambda m:self.perception.append(json.loads(m.data)),10)
 def on_cmd(self,m):
  self.commands.append([self.now_s(),m.linear.x,m.linear.y,m.angular.z])
  cmd=TwistStamped();cmd.header.stamp=self.get_clock().now().to_msg();cmd.header.frame_id='base_link';cmd.twist=m
  cmd.twist.linear.x=float(np.clip(cmd.twist.linear.x,-.35,.5));cmd.twist.linear.y=0.;cmd.twist.angular.z=float(np.clip(cmd.twist.angular.z,-1.,1.));self.vel_pub.publish(cmd)
 def on_tracks(self,m):
  self.tracks=m
  self.estimates.append({'stamp':m.header.stamp.sec+m.header.stamp.nanosec/1e9,'healthy':m.healthy,'status':m.status,
    'objects':[{'id':o.id,'p':[o.position.x,o.position.y],'v':[o.velocity.x,o.velocity.y],'r':o.radius,'observed':o.observed,'state':o.state,'last_observed':o.last_observed.sec+o.last_observed.nanosec*1e-9,'position_sigma':[math.sqrt(max(0.,o.covariance[i])) for i in [0,5]]} for o in m.obstacles]})
 def on_touch(self,m):
  if (self.start is not None or self.goal_sent is not None) and m.data:self.touched=True
 def motion_watch(self):
  if self.done or self.goal_sent is None or self.start is not None or self.truth is None:return
  p=self.truth.translation
  if motion_started(self.protocol,p.x,p.y):
   self.start=self.now_s();self.phase='moving';self.get_logger().info(f'Movement timing start: sim {self.start:.3f}, goal sent {self.goal_sent:.3f}')
 def action_diagnostics(self,now):
  if self.a.method!='mppi' or not self.goal_future or not self.goal_future.done() or self.action_recorded:return
  handle=self.goal_future.result()
  if not handle.accepted:self.action_events.append(dict(stamp=now,event='goal_rejected'));self.action_recorded=True
  elif self.result_future is None:self.result_future=handle.get_result_async()
  elif self.result_future.done():
   self.action_events.append(dict(stamp=now,event='action_result',status=self.result_future.result().status));self.action_recorded=True
 def mppi_velocity_ready(self):
  try:
   self.mppi_velocity_topics=[name for name,types in self.get_subscriber_names_and_types_by_node('controller_server','/') if 'nav_msgs/msg/Odometry' in types]
  except Exception:self.mppi_velocity_topics=[]
  return '/platform/odom/filtered' in self.mppi_velocity_topics and '/platform/odom' not in self.mppi_velocity_topics
 def tick(self):
  if self.a.protocol=='legacy':return self.tick_legacy()
  if self.done:return
  now=self.now_s();p=self.truth.translation if self.truth is not None else None
  if self.goal_sent is None:
   ready=p is not None and self.odom is not None and self.scan is not None and len(self.objects)==8 and self.anchor is not None and self.tracks is not None
   ready=ready and self.tracks.healthy and now-(self.tracks.header.stamp.sec+self.tracks.header.stamp.nanosec/1e9)<.35
   ready=ready and (self.action.server_is_ready() and self.mppi_velocity_ready() if self.a.method=='mppi' else self.goal_pub.get_subscription_count()>0 and now-(self.common_prior_time if self.a.method=='mpc_d_cbf' else self.prior_time)<.8)
   if now<8:return
   if not ready:
    if now>8.3:self.save('infrastructure_late_ready')
    return
   if math.hypot(p.x-self.protocol.start_x,p.y)>.10:self.save('prestart_displacement');return
   self.goal_sent=now
   goal=PoseStamped();goal.header.stamp=self.get_clock().now().to_msg();goal.header.frame_id='odom' if self.a.method=='mppi' else 'world'
   gx,gy=self.protocol.navigation_goal(self.anchor);goal.pose.position.x=gx;goal.pose.position.y=gy;goal.pose.position.z=0. if self.a.method=='mppi' else .25
   # Reference goal orientation is world yaw zero, expressed in the odom alias.
   angle=self.anchor[2]-self.protocol.start_yaw;goal.pose.orientation.z=math.sin(angle/2);goal.pose.orientation.w=math.cos(angle/2)
   self.goal_odom=(gx,gy)
   if self.a.method=='mppi':req=NavigateToPose.Goal();req.pose=goal;self.goal_future=self.action.send_goal_async(req)
   else:self.goal_pub.publish(goal)
   self.get_logger().info(f'Challenge goal issued at sim {now:.3f}; waiting for truth displacement >=0.1m')
  self.action_diagnostics(now)
  if self.controller_complete is None and self.state=='GOAL_REACHED' and self.planner_state=='WAIT_TARGET':
   self.controller_complete=dict(stamp=now,truth_goal_distance_m=math.hypot(p.x-self.protocol.goal_x,p.y))
  if self.start is None:
   self.startup_rows.append([now,p.x,p.y,math.hypot(p.x-self.protocol.start_x,p.y)])
   if self.touched:self.save('collision')
   elif now-self.goal_sent>=30.:self.save('infrastructure_startup_no_motion')
   return
  elapsed=now-self.start
  self.rows.append([elapsed,p.x,p.y,yaw(self.truth.rotation),self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z])
  self.odom_rows.append([elapsed,self.odom.pose.pose.position.x,self.odom.pose.pose.position.y,yaw(self.odom.pose.pose.orientation)])
  self.states.append([elapsed,self.state,self.planner_state])
  if elapsed-self.last_progress>=10:
   self.last_progress=elapsed;self.get_logger().info(f'{self.method_label}: timed {elapsed:.1f}s, ({p.x:.2f},{p.y:.2f}), state={self.state}')
  result=outcome(self.protocol,elapsed,math.hypot(p.x-self.protocol.goal_x,p.y),self.touched)
  if result:self.save(result)
 def tick_legacy(self):
  if self.done:return
  now=self.now_s()
  ready=self.truth is not None and self.odom is not None and self.scan is not None and len(self.objects)==8 and self.anchor is not None and self.tracks is not None
  if self.start is None:
   ready=ready and self.tracks.healthy and now-(self.tracks.header.stamp.sec+self.tracks.header.stamp.nanosec/1e9)<.35
   ready=ready and (self.action.server_is_ready() if self.a.method=='mppi' else self.goal_pub.get_subscription_count()>0 and now-(self.common_prior_time if self.a.method=='mpc_d_cbf' else self.prior_time)<.8)
   if now<8:return
   if not ready:
    if now>8.3:self.save('infrastructure_late_ready')
    return
   if math.hypot(self.truth.translation.x-12,self.truth.translation.y)>.10:self.save('prestart_displacement');return
   self.start=now;self.phase='moving';ax,ay,heading=self.anchor
   goal=PoseStamped();goal.header.stamp=self.get_clock().now().to_msg();goal.header.frame_id='odom' if self.a.method=='mppi' else 'world'
   goal.pose.position.x=ax+21*math.cos(heading);goal.pose.position.y=ay+21*math.sin(heading);goal.pose.position.z=0. if self.a.method=='mppi' else .25
   goal.pose.orientation.z=math.sin(heading/2);goal.pose.orientation.w=math.cos(heading/2)
   self.goal_odom=(goal.pose.position.x,goal.pose.position.y)
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
  elif (self.a.method!='mppi' and self.goal_odom is not None and
        self.state=='GOAL_REACHED' and self.planner_state=='WAIT_TARGET' and
        math.hypot(self.odom.pose.pose.position.x-self.goal_odom[0],
                   self.odom.pose.pose.position.y-self.goal_odom[1])<.10):
   # Same completion semantics as Nav2 action success outside the truth gate:
   # preserve a localization/accuracy failure, do not wait parked for a later contact.
   self.save('estimated_goal_only')
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
  result.update(method=self.a.method,motion_phase_s=self.a.motion_phase,protocol='dynabarn_challenge_rules_ros2_v1' if self.a.protocol=='challenge' else 'dynabarn_ros2_unknown_region_lifecycle_v2',
    navigation_anchor=list(self.anchor) if self.anchor is not None else None,timeout_sim_s=self.a.timeout,scheduled_start_sim_s=8.,sensor_range_m=5.,static_background='static wall boxes only; shared with MPPI',truth_input_to_perception=False)
  pkg=Path(os.environ['BENCH_WS'])/'src/dynabarn_ros2';core=Path(os.environ['BENCH_WS'])/'src/slash_planner'
  files=[pkg/'scripts/localization_profiles.py',pkg/'scripts/benchmark_protocol.py',pkg/'scripts/evaluate.py',pkg/'comparison_config/robot.yaml',core/'plan_manage/src/scan_replan_fsm.cpp',pkg/'src/motion.cpp',pkg/'launch/comparison.launch.py',pkg/'launch/dynabarn.launch.py',pkg/'scripts/laser_perception.py',pkg/'scripts/map_geometry.py',pkg/'scripts/compare_evaluate.py',pkg/'comparison_config/nav2.yaml',pkg/'worlds/official_world_2.world',core/'geometric_obstacles/geometric_obstacles/core.py',core/'plan_manage/src/closed_loop_controller.cpp',core/'plan_manage/src/planner_manager.cpp',core/'plan_manage/include/plan_manage/intent_frs.hpp',core/'plan_manage/include/plan_manage/joint_rollout.hpp',core/'plan_manage/include/plan_manage/active_escape.hpp',core/'plan_manage/include/plan_manage/dynamic_local_path.hpp',core/'plan_manage/include/plan_manage/spatial_tracker.hpp',core/'scan_planner_msgs/msg/DynamicObstacle.msg',core/'path_searching/src/dyn_a_star.cpp']
  if self.a.method=='mpc_d_cbf':
   files.extend([pkg/'../mpc_d_cbf/core.py',pkg/'../mpc_d_cbf/node.py',pkg/'../mpc_d_cbf/upstream/local_planner.py'])
  protocol_hash=hashlib.sha256(b''.join(p.read_bytes() for p in files)).hexdigest()
  result['method_label']=self.method_label
  result['velocity_feedback_source']='/platform/odom/filtered'
  result['mppi_observed_odometry_topics']=self.mppi_velocity_topics
  result['optimizer']='MPC-D-CBF ROS2 common-input adapter' if self.a.method=='mpc_d_cbf' else self.a.method
  result['local_dynamic_planner_enabled']=self.a.local_dynamic_planner=='true'
  result['joint_rollout_enabled']=self.a.joint_rollout=='true'
  result['active_escape_enabled']=self.a.active_escape=='true'
  result['navigation_goal_odom']=list(self.goal_odom) if self.goal_odom is not None else None
  result['completion_policy']='truth distance <0.3m succeeds; estimated controller/action completion outside truth gate is estimated_goal_only, never success'
  if self.a.protocol=='challenge':
   result.update(evaluation_profile=self.protocol.metadata(),reference_revision=REFERENCE_REVISION,
    reference_url=f'https://github.com/Daffan/the-barn-challenge/blob/{REFERENCE_REVISION}/run.py',
    goal_to_finish_s=self.now_s()-self.goal_sent if self.goal_sent is not None else None,scheduled_goal_sim_s=8.,scheduled_start_sim_s=None,goal_sent_sim_s=self.goal_sent,motion_start_sim_s=self.start,startup_delay_s=self.start-self.goal_sent if self.start is not None else None,
    controller_completion_diagnostic=self.controller_complete,action_diagnostics=self.action_events,
    completion_policy='truth distance <=1m succeeds; collision or 100s moving-clock timeout fail; controller/action completion is diagnostic only',
    scope='BARN Challenge dynamic task and termination rules, ROS2 matched-condition adaptation; not an official full-suite/leaderboard reproduction',
    deviations=['Jazzy/Harmonic and modern Jackal plant','matched 0.5m/s and 1rad/s caps','native wheel/IMU EKF and 5m lidar perception','fixed goal publication sim8s and selected motion phases','30s no-motion infrastructure watchdog; reference startup wait is unbounded','contacts after goal publication retained including pre-motion'],
    goal_world=[self.protocol.goal_x,0.],start_world=[self.protocol.start_x,0.,self.protocol.start_yaw],goal_tolerance_m=self.protocol.radius)
  else:result['evaluation_profile']=self.protocol.metadata()
  result['protocol_sha256']=protocol_hash
  result['localization_profile']=getattr(self.a,'localization_profile','velocity_imu')
  if result['localization_profile']=='velocity_imu':
   import yaml
   configs=list(Path('/tmp').glob('dynabarn-velocity-imu-*/platform/config/localization.yaml'))
   if len(configs)==1:
    text=configs[0].read_text();(self.dest/'runtime_localization.yaml').write_text(text)
    result['runtime_localization']=yaml.safe_load(text)['/']['ekf_node']['ros__parameters']
   else:result['runtime_localization']=None
  result['configuration_sha256']=hashlib.sha256((protocol_hash+json.dumps([self.a.method,self.a.motion_phase,self.a.timeout,self.a.active_escape,self.a.local_dynamic_planner,self.a.joint_rollout,self.a.protocol,result['localization_profile']])).encode()).hexdigest()
  (self.dest/'result.json').write_text(json.dumps(result,indent=2)+'\n')
  (self.dest/'local_plans.json').write_text(json.dumps(self.local_plans))
  (self.dest/'mpc_decisions.json').write_text(json.dumps(self.mpc_decisions))
  with (self.dest/'commands.csv').open('w') as f:w=csv.writer(f);w.writerow(['sim_s','vx','vy','wz']);w.writerows(self.commands)
  (self.dest/'decisions.json').write_text(json.dumps(self.decisions))
  (self.dest/'tracks.json').write_text(json.dumps(self.estimates))
  (self.dest/'perception.json').write_text(json.dumps(self.perception))
  with (self.dest/'odometry.csv').open('w') as f:w=csv.writer(f);w.writerow(['elapsed_s','x','y','yaw']);w.writerows(self.odom_rows)
  with (self.dest/'states.csv').open('w') as f:w=csv.writer(f);w.writerow(['elapsed_s','state','planner']);w.writerows(self.states)
  with (self.dest/'startup.csv').open('w') as f:w=csv.writer(f);w.writerow(['sim_s','truth_x','truth_y','displacement_m']);w.writerows(self.startup_rows)

def main():
 p=argparse.ArgumentParser();p.add_argument('--localization-profile',choices=['velocity_imu','legacy'],default='velocity_imu');p.add_argument('--protocol',choices=list(PROFILES),default='challenge');p.add_argument('--joint-rollout',choices=['true','false'],default='false');p.add_argument('--local-dynamic-planner',choices=['true','false'],default='false');p.add_argument('--active-escape',choices=['true','false'],default='false');p.add_argument('--method',choices=['mppi','frs_soft','frs_joint','frs_hard','mpc_d_cbf'],required=True);p.add_argument('--motion-phase',type=float,default=0.);p.add_argument('--output',required=True);p.add_argument('--timeout',type=float)
 a,ros=p.parse_known_args()
 if a.protocol=='challenge' and a.timeout not in (None,100.):p.error('challenge protocol requires timeout=100')
 if a.timeout is None:a.timeout=PROFILES[a.protocol].timeout
 a.mode=a.method;rclpy.init(args=ros);n=Comparison(a)
 try:
  while rclpy.ok() and not n.done:
   rclpy.spin_once(n,timeout_sec=.05)
   if time.monotonic()-n.wall>360:n.save('infrastructure_timeout')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
