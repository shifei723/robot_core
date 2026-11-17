#!/usr/bin/env python3
"""ROS2 common-sensor interface for the upstream MPC-D-CBF optimiser."""
import json,math,time
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,OccupancyGrid
from geometry_msgs.msg import PoseStamped,Twist
from std_msgs.msg import String
from scan_planner_msgs.msg import DynamicObstacleArray
from mpc_d_cbf_core import UpstreamMPC,predict_ellipses,static_map_ellipses

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
class MPCNode(Node):
 def __init__(self):
  super().__init__('mpc_d_cbf_controller');self.core=UpstreamMPC();self.pool=ThreadPoolExecutor(max_workers=1)
  self.body=None;self.body_time=-1.;self.samples=[];self.track_time=-1.;self.healthy=False;self.static=None;self.goal=None;self.path=None
  self.future=None;self.solve_started=-1.;self.last_good=-1.;self.good_input=-1.;self.next_solve=0.;self.command=np.zeros(2);self.completed=False
  self.pub=self.create_publisher(Twist,'/benchmark/cmd_vel',10);self.state=self.create_publisher(String,'/height_control/state',10)
  self.status=self.create_publisher(String,'/planning/status',10);self.log=self.create_publisher(String,'/mpc_d_cbf/decision',10)
  self.create_subscription(Odometry,'/benchmark/body_pose',self.on_body,10)
  self.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',self.on_tracks,10)
  self.create_subscription(OccupancyGrid,'/benchmark/static_map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
  self.create_subscription(PoseStamped,'/move_base_simple/goal',self.on_goal,10)
  self.create_timer(.02,self.tick)
 def now_s(self):return self.get_clock().now().nanoseconds*1e-9
 def on_body(self,m):
  if m.header.frame_id!='world':return
  self.body=np.array([m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)]);self.body_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 def on_tracks(self,m):
  self.healthy=bool(m.healthy and m.header.frame_id=='world');self.track_time=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
  self.samples=[dict(id=o.id,p=[o.position.x,o.position.y],v=[o.velocity.x,o.velocity.y],covariance=list(o.covariance),radius=o.radius,unknown=o.state==o.UNKNOWN) for o in m.obstacles]
 def on_map(self,m):
  if m.header.frame_id!='odom':return
  self.static=static_map_ellipses(np.asarray(m.data).reshape(m.info.height,m.info.width),m.info.resolution,np.array([m.info.origin.position.x,m.info.origin.position.y]))
 def on_goal(self,m):
  if self.body is None or m.header.frame_id not in ('world','odom'):return
  self.goal=np.array([m.pose.position.x,m.pose.position.y]);delta=self.goal-self.body[:2];length=np.linalg.norm(delta)
  if length<1e-8:return
  arc=np.r_[np.arange(0.,length,.1),length];xy=self.body[:2]+arc[:,None]*delta/length
  self.path=np.c_[xy,np.zeros(len(xy))];self.completed=False;self.last_good=-1.;self.command[:]=0
 def publish(self,v,w,state):
  cmd=Twist();cmd.linear.x=float(v);cmd.angular.z=float(w);self.pub.publish(cmd)
  s=String();s.data=state;self.state.publish(s);s=String();s.data='WAIT_TARGET' if self.completed or self.goal is None else 'EXEC_TRAJ';self.status.publish(s)
 def tick(self):
  now=self.now_s();fresh=self.body is not None and 0<=now-self.body_time<.35 and self.healthy and 0<=now-self.track_time<.35 and self.static is not None
  if self.future is not None and self.future.done():
   elapsed=(time.perf_counter()-self.solve_wall)*1000.;self.future_result_error=None
   try:result=self.future.result()
   except Exception as e:result=dict(success=False,errors=[repr(e)])
   accepted=bool(result['success'] and now-self.solve_started<=.4 and fresh and not self.completed)
   if accepted:self.command=np.clip(result['controls'][0],[-.1,-1.],[.5,1.]);self.last_good=now;self.good_input=self.solve_started
   else:self.command[:]=0.;self.last_good=-1.
   info=dict(stamp=now,input_stamp=self.solve_started,success=bool(result['success']),solver_status=result.get('solver_status','exception'),accepted=accepted,solve_ms=elapsed,errors=result.get('errors',[]),track_count=self.solve_tracks,static_count=self.solve_static,source_sha256=self.core.source_sha256,command=self.command.tolist(),active_mask=result.get('active_mask',[]),solver_constraints=result.get('solver_constraints'),input_pose=self.solve_pose,obstacle_predictions=self.solve_preview)
   if result['success']:info['path']=result['states'][:,:2].tolist()
   msg=String();msg.data=json.dumps(info);self.log.publish(msg);self.future=None
  if not fresh:self.command[:]=0.;self.last_good=-1.;self.publish(0.,0.,'MPC_WAIT_INPUT');return
  if self.goal is None:self.publish(0.,0.,'IDLE');return
  if np.linalg.norm(self.body[:2]-self.goal)<.1:self.completed=True
  if self.completed:self.command[:]=0.;self.publish(0.,0.,'GOAL_REACHED');return
  if self.future is None and now>=self.next_solve:
   self.next_solve=now+.05
   try:
    moving=predict_ellipses(self.samples,now-self.track_time)
    # Static connected-component enclosures come from the same shared map, not truth obstacles.
    static=np.repeat(self.static[:,None,:],25,axis=1);ellipses=np.concatenate([moving,static])
    self.solve_tracks=len(moving);self.solve_static=len(static);self.solve_started=now;self.solve_wall=time.perf_counter()
    self.solve_pose=self.body.tolist();self.solve_preview=ellipses[:,[0,9,24],:].tolist()
    self.future=self.pool.submit(self.core.solve,self.body.copy(),self.path.copy(),ellipses)
   except Exception as e:
    self.get_logger().error(str(e));self.command[:]=0.;self.last_good=-1.
  if self.last_good<0 or now-self.last_good>.25 or now-self.good_input>.4:self.publish(0.,0.,'MPC_HOLD_NO_PLAN')
  else:self.publish(*self.command,'TRACKING')
def main():
 rclpy.init();n=MPCNode()
 try:rclpy.spin(n)
 finally:
  n.publish(0.,0.,'STOPPED');n.pool.shutdown(wait=True);n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
