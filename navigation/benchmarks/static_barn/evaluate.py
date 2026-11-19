#!/usr/bin/env python3
"""Common simulated-time evaluator and sensor adapter. No truth map reaches a planner."""
import argparse, csv, json, math, time, os, hashlib
from pathlib import Path
import numpy as np
from protocol import goal_in_odom, score, STATIC_START, STATIC_GOAL, motion_started, outcome
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, PointCloud2, Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Evaluation(Node):
    def __init__(self, args):
        super().__init__('barn_evaluation', automatically_declare_parameters_from_overrides=True)
        self.tracking_rows=[];self.goal_sent=None;self.imu=None;self.action_events=[];self.action_recorded=False;self.anchor=None;self.goal_odom=None;self.commands=[];self.odom_rows=[];self.startup_rows=[];self.mppi_topics=[];self.saved=False
        self.args=args; self.truth=None; self.odom=None; self.scan=None; self.collided=False
        self.last_progress=-10.; self.start=None; self.rows=[]; self.status=None; self.last_row=-1.; self.ready_wall=time.monotonic()
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.create_subscription(TFMessage,'/model/robot/pose',self.truth_cb,10)
        self.create_subscription(Odometry,'/platform/odom/filtered',self.odom_cb,qos_profile_sensor_data)
        self.create_subscription(Imu,'/sensors/imu_0/data',lambda m:setattr(self,'imu',m),qos_profile_sensor_data)
        self.create_subscription(TwistStamped,'/cmd_vel',lambda m:self.commands.append([self.get_clock().now().nanoseconds/1e9,m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]),10)
        self.create_subscription(LaserScan,'/front/scan',self.scan_cb,qos_profile_sensor_data)
        self.create_subscription(Bool,'/robot/touched',self.touch_cb,10)
        self.create_subscription(String,'/planning/tracking_metrics',lambda m:self.tracking_rows.append([self.get_clock().now().nanoseconds/1e9,m.data]),10)
        self.body_pub=self.create_publisher(Odometry,'/benchmark/body_pose',10)
        self.sensor_pub=self.create_publisher(Odometry,'/benchmark/sensor_pose',10)
        self.cloud_pub=self.create_publisher(PointCloud2,'/benchmark/cloud',10)
        self.goal_pub=self.create_publisher(PoseStamped,'/move_base_simple/goal',10)
        self.vel_pub=self.create_publisher(TwistStamped,'/cmd_vel',10)
        if args.method=='slash':
            self.create_subscription(Twist,'/benchmark/cmd_vel',self.cmd_cb,10)
        self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.timer=self.create_timer(.1,self.tick)
        self.motion_timer=self.create_timer(.01,self.motion_watch)
        self.goal_future=None; self.result_future=None
    def truth_cb(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='robot': self.truth=t.transform
    def touch_cb(self,msg):
        if self.goal_sent is not None and msg.data: self.collided=True
    def odom_cb(self,msg):
        self.odom=msg
        if self.args.method=='slash':
            import copy
            body=copy.deepcopy(msg); body.pose.pose.position.z=.25
            self.body_pub.publish(body)
    def cmd_cb(self,msg):
        cmd=TwistStamped(); cmd.header.stamp=self.get_clock().now().to_msg();cmd.header.frame_id='base_link'
        cmd.twist=msg
        # Shared command bounds, no lateral motion on differential-drive Jackal.
        cmd.twist.linear.x=float(np.clip(cmd.twist.linear.x,-.35,.5));cmd.twist.linear.y=0.
        cmd.twist.angular.z=float(np.clip(cmd.twist.angular.z,-1.,1.))
        self.vel_pub.publish(cmd)
    def scan_cb(self,msg):
        self.scan=msg
        if self.args.method!='slash':return
        try:t=self.tf.lookup_transform('odom',msg.header.frame_id,Time.from_msg(msg.header.stamp)).transform
        except Exception:return
        a=msg.angle_min+np.arange(len(msg.ranges))*msg.angle_increment
        r=np.asarray(msg.ranges);valid=np.isfinite(r)&(r>=msg.range_min)&(r<=min(msg.range_max,5.))
        a=a[valid]+yaw(t.rotation);r=r[valid]
        points=np.column_stack((t.translation.x+r*np.cos(a),t.translation.y+r*np.sin(a),np.full(len(r),.25)))
        # Clear robot self returns, matching Nav2 costmap footprint clearing.
        if self.odom is None:return
        base=self.odom.pose.pose.position
        points=points[np.hypot(points[:,0]-base.x,points[:,1]-base.y)>.334]
        h=Header(stamp=msg.header.stamp,frame_id='odom')
        self.cloud_pub.publish(create_cloud_xyz32(h,points))
        sensor=Odometry();sensor.header=h;sensor.pose.pose.position.x=t.translation.x
        sensor.pose.pose.position.y=t.translation.y;sensor.pose.pose.position.z=.25
        sensor.pose.pose.orientation=t.rotation;self.sensor_pub.publish(sensor)
    def motion_watch(self):
        if self.status or self.goal_sent is None or self.start is not None or self.truth is None:return
        p=self.truth.translation
        if motion_started(p.x,p.y):
            self.start=self.get_clock().now().nanoseconds/1e9;self.start_wall=time.monotonic()
    def mppi_ready(self):
        try:self.mppi_topics=[name for name,types in self.get_subscriber_names_and_types_by_node('controller_server','/') if 'nav_msgs/msg/Odometry' in types]
        except Exception:self.mppi_topics=[]
        return '/platform/odom/filtered' in self.mppi_topics and '/platform/odom' not in self.mppi_topics
    def tick(self):
        if self.status:return
        now=self.get_clock().now().nanoseconds/1e9
        if self.goal_sent is None:
            ready=self.truth is not None and self.odom is not None and self.scan is not None and self.imu is not None
            if ready:
                stamps=[m.header.stamp.sec+m.header.stamp.nanosec*1e-9 for m in [self.odom,self.scan,self.imu]]
                ready=all(0<=now-t<.35 for t in stamps)
            if now<5:return
            if not ready:
                if now>30:self.status='infrastructure_inputs_not_ready';self.save()
                return
            if self.args.method=='mppi' and not (self.action.server_is_ready() and self.mppi_ready()):return
            if self.args.method=='slash' and self.goal_pub.get_subscription_count()==0:return
            o=self.odom.pose.pose
            self.anchor=[o.position.x,o.position.y,yaw(o.orientation)]
            # Known initial map pose, not ground-truth feedback to navigation.
            gx,gy,gyaw=goal_in_odom(STATIC_START,self.anchor)
            self.goal_odom=[gx,gy]
            goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=self.get_clock().now().to_msg()
            goal.pose.position.x=gx;goal.pose.position.y=gy;goal.pose.position.z=.25 if self.args.method=='slash' else 0.
            goal.pose.orientation.z=math.sin(gyaw/2);goal.pose.orientation.w=math.cos(gyaw/2)
            if self.args.method=='mppi':
                request=NavigateToPose.Goal();request.pose=goal;self.goal_future=self.action.send_goal_async(request)
            else:self.goal_pub.publish(goal)
            self.goal_sent=now
            self.get_logger().info(f'Goal dispatched: {self.args.method}, world {self.args.world}, sim {now:.2f}')
        if self.goal_future and self.goal_future.done() and not self.action_recorded:
            try:
                handle=self.goal_future.result()
                if not handle.accepted:self.action_events.append(dict(stamp=now,event='rejected'));self.action_recorded=True
                elif self.result_future is None:self.result_future=handle.get_result_async()
                elif self.result_future.done():self.action_events.append(dict(stamp=now,event='completed',status=self.result_future.result().status));self.action_recorded=True
            except Exception as e:self.action_events.append(dict(stamp=now,event='error',error=repr(e)));self.action_recorded=True
        x=self.truth.translation.x;y=self.truth.translation.y
        if self.start is None:
            self.startup_rows.append([now,x,y])
            if self.collided:self.status='collision';self.save()
            elif now-self.goal_sent>=30:self.status='startup_no_motion';self.save()
            return
        elapsed=now-self.start
        if elapsed-self.last_progress>=10.:
            self.get_logger().info(f'Progress: {elapsed:.1f} s, world=({x:.2f},{y:.2f}), goal distance={math.hypot(x+2.25,y-13):.2f} m')
            self.last_progress=elapsed
        if now-self.last_row>=.09:
            self.rows.append([elapsed,x,y,yaw(self.truth.rotation),self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z]);self.last_row=now
            o=self.odom.pose.pose;self.odom_rows.append([elapsed,o.position.x,o.position.y,yaw(o.orientation)])
        self.status=outcome(elapsed,math.hypot(x-STATIC_GOAL[0],y-STATIC_GOAL[1]),self.collided,self.args.goal_tolerance,self.args.timeout)
        if self.status:self.save()
    def save(self):
        if self.saved:return
        self.saved=True
        dest=Path(self.args.output);dest.mkdir(parents=True,exist_ok=True)
        name=f'{self.args.method}_world_{self.args.world}_trial_{self.args.trial}'
        with (dest/(name+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['sim_elapsed_s','world_x','world_y','yaw','odom_vx','odom_wz']);writer.writerows(self.rows)
        with (dest/(name+'_tracking.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(['sim_time','metrics']);w.writerows(self.tracking_rows)
        elapsed=self.get_clock().now().nanoseconds/1e9-self.start if self.start is not None else None
        for suffix,header,rows in [('commands',['sim_s','vx','vy','wz'],self.commands),('odometry',['elapsed_s','x','y','yaw'],self.odom_rows),('startup',['sim_s','world_x','world_y'],self.startup_rows)]:
            with (dest/(name+'_'+suffix+'.csv')).open('w') as f:
                writer=csv.writer(f);writer.writerow(header);writer.writerows(rows)
        path=np.load(str(Path(os.environ['BENCH_WS'])/f'src/The-Barn-Challenge-Ros2/jackal_helper/worlds/BARN/path_files/path_{self.args.world}.npy'))
        ref=np.vstack(([-2.25,3.],np.asarray(path)*.15+[-4.575,5.075],[-2.25,13.]))
        length=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum());optimal=length/2.
        metric=score(self.status=='success',elapsed,length)
        driven=float(np.linalg.norm(np.diff(np.asarray(self.rows)[:,1:3],axis=0),axis=1).sum()) if len(self.rows)>1 else 0.
        result={'method':self.args.method,'world':self.args.world,'trial':self.args.trial,'status':self.status,
                'elapsed_sim_s':elapsed,'path_length_m':driven,'reference_path_length_m':length,
                'upstream_runner_score':metric,'speed_cap_mps':.5,'yaw_rate_cap_radps':1.,
                'collision_signal':'Gazebo TouchPlugin','goal_tolerance_m':self.args.goal_tolerance,'timeout_sim_s':self.args.timeout,
                'configuration_sha256':hashlib.sha256(
                    Path(__file__).resolve().with_name('benchmark.launch.py').read_bytes()+Path(__file__).resolve().with_name('protocol.py').read_bytes()+
                    Path(__file__).resolve().read_bytes()+str([self.args.goal_tolerance,self.args.timeout]).encode()+
                    (Path(os.environ['BENCH_WS'])/'src/The-Barn-Challenge-Ros2/jackal_helper/config/nav2.yaml').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/planner_manager.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/wheel_timing.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/closed_loop_controller.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/scan_replan_fsm.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/spatial_tracker.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/wheel_shortcut.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/static_motion_safety.hpp').read_bytes()).hexdigest(),
                'inputs_seen':{'truth':self.truth is not None,'odometry':self.odom is not None,'scan':self.scan is not None},
                'protocol':'static_challenge_task_rules_velocity_imu_v2; not official leaderboard reproduction',
                'start_world':list(STATIC_START),'goal_world':list(STATIC_GOAL),'goal_sent_sim_s':self.goal_sent,'motion_start_sim_s':self.start,
                'startup_delay_s':self.start-self.goal_sent if self.start is not None else None,
                'goal_to_finish_s':self.get_clock().now().nanoseconds/1e9-self.goal_sent if self.goal_sent is not None else None,
                'localization_profile':'velocity_imu','truth_navigation_input':False,'navigation_anchor':self.anchor,'navigation_goal_odom':self.goal_odom,
                'mppi_odometry_topics':self.mppi_topics,'action_diagnostics':self.action_events,'completion_policy':'truth only; controller/action completion diagnostic',
                'reference_revision':'bf5a226f6088ec96bf0d2dbee3253a8ea6119b83'}
        import yaml
        configs=list(Path('/tmp').glob('dynabarn-velocity-imu-*/platform/config/localization.yaml'))
        if len(configs)==1:
            text=configs[0].read_text();(dest/(name+'_localization.yaml')).write_text(text);result['runtime_localization']=yaml.safe_load(text)['/']['ekf_node']['ros__parameters']
        (dest/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        self.get_logger().info(json.dumps(result))
        cmd=TwistStamped();cmd.header.stamp=self.get_clock().now().to_msg();self.vel_pub.publish(cmd)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--method',choices=['mppi','slash'],required=True)
    parser.add_argument('--world',type=int,default=0);parser.add_argument('--trial',type=int,default=0)
    parser.add_argument('--goal-tolerance',type=float,default=1.);parser.add_argument('--timeout',type=float,default=100);parser.add_argument('--wall-timeout',type=float,default=600)
    parser.add_argument('--output',default=str(Path(__file__).resolve().parent/'results'));parser.add_argument('--launch-pid',type=int)
    args,ros=parser.parse_known_args();
    if not math.isfinite(args.goal_tolerance) or args.goal_tolerance<=0:parser.error('goal tolerance must be finite and positive')
    rclpy.init(args=ros);node=Evaluation(args);last_health=0.
    try:
        while rclpy.ok() and not node.status:
            rclpy.spin_once(node,timeout_sec=.2)
            if node.start is None and time.monotonic()-last_health>15:
                node.get_logger().info(f'Readiness: truth={node.truth is not None}, odom={node.odom is not None}, scan={node.scan is not None}, sim={node.get_clock().now().nanoseconds/1e9:.1f}')
                last_health=time.monotonic()
            if args.launch_pid:
                try:os.kill(args.launch_pid,0)
                except ProcessLookupError:
                    node.status='infrastructure_launch_exit';node.save();break
            if time.monotonic()-node.ready_wall>args.wall_timeout:
                node.status='infrastructure_timeout';node.save()
    finally:node.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
