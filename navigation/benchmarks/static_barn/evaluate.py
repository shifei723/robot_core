#!/usr/bin/env python3
"""Common simulated-time evaluator and sensor adapter. No truth map reaches a planner."""
import argparse, csv, json, math, time, os, hashlib
from pathlib import Path
import numpy as np
from protocol import goal_in_odom, score
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Evaluation(Node):
    def __init__(self, args):
        super().__init__('barn_evaluation', automatically_declare_parameters_from_overrides=True)
        self.tracking_rows=[]
        self.args=args; self.truth=None; self.odom=None; self.scan=None; self.collided=False
        self.last_progress=-10.; self.start=None; self.rows=[]; self.status=None; self.last_row=-1.; self.ready_wall=time.monotonic()
        self.tf=Buffer(); self.listener=TransformListener(self.tf,self)
        self.create_subscription(TFMessage,'/model/robot/pose',self.truth_cb,10)
        self.create_subscription(Odometry,'/platform/odom/filtered',self.odom_cb,qos_profile_sensor_data)
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
        self.goal_future=None; self.result_future=None
    def truth_cb(self,msg):
        for t in msg.transforms:
            if t.child_frame_id=='robot': self.truth=t.transform
    def touch_cb(self,msg):
        if self.start is not None and msg.data: self.collided=True
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
    def tick(self):
        if self.status:return
        if self.truth is None or self.odom is None or self.scan is None:return
        now=self.get_clock().now().nanoseconds/1e9
        if self.start is None:
            if now<5:return
            if self.args.method=='mppi' and not self.action.server_is_ready():return
            if self.args.method=='slash' and self.goal_pub.get_subscription_count()==0:return
            # Transform fixed world goal into odometry frame using synchronized start poses.
            w=self.truth; o=self.odom.pose.pose
            gx,gy,gyaw=goal_in_odom((w.translation.x,w.translation.y,yaw(w.rotation)),
                                   (o.position.x,o.position.y,yaw(o.orientation)))
            goal=PoseStamped();goal.header.frame_id='odom';goal.header.stamp=self.get_clock().now().to_msg()
            goal.pose.position.x=gx
            goal.pose.position.y=gy
            goal.pose.position.z=.25 if self.args.method=='slash' else 0.
            goal.pose.orientation.z=math.sin(gyaw/2);goal.pose.orientation.w=math.cos(gyaw/2)
            if self.args.method=='mppi':
                request=NavigateToPose.Goal();request.pose=goal
                self.goal_future=self.action.send_goal_async(request)
            else:self.goal_pub.publish(goal)
            self.start=now; self.start_wall=time.monotonic()
            self.get_logger().info(f'Goal dispatched: {self.args.method}, world {self.args.world}, sim {now:.2f}')
        elapsed=now-self.start
        x=self.truth.translation.x;y=self.truth.translation.y
        if elapsed-self.last_progress>=10.:
            self.get_logger().info(f'Progress: {elapsed:.1f} s, world=({x:.2f},{y:.2f}), goal distance={math.hypot(x+2.25,y-13):.2f} m')
            self.last_progress=elapsed
        if now-self.last_row>=.09:
            self.rows.append([elapsed,x,y,yaw(self.truth.rotation),self.odom.twist.twist.linear.x,self.odom.twist.twist.angular.z]);self.last_row=now
        if self.collided:self.status='collision'
        elif math.hypot(x+2.25,y-13)<1.:self.status='success'
        elif elapsed>=self.args.timeout:self.status='timeout'
        elif self.goal_future and self.goal_future.done():
            handle=self.goal_future.result()
            if not handle.accepted:self.status='goal_rejected'
            elif self.result_future is None:self.result_future=handle.get_result_async()
            elif self.result_future.done() and self.result_future.result().status in (5,6):self.status='navigation_aborted'
        if self.status:self.save()
    def save(self):
        dest=Path(self.args.output);dest.mkdir(parents=True,exist_ok=True)
        name=f'{self.args.method}_world_{self.args.world}_trial_{self.args.trial}'
        with (dest/(name+'.csv')).open('w') as f:
            writer=csv.writer(f);writer.writerow(['sim_elapsed_s','world_x','world_y','yaw','odom_vx','odom_wz']);writer.writerows(self.rows)
        with (dest/(name+'_tracking.csv')).open('w') as f:
            w=csv.writer(f);w.writerow(['sim_time','metrics']);w.writerows(self.tracking_rows)
        elapsed=self.rows[-1][0] if self.rows else None
        path=np.load(str(Path(os.environ['BENCH_WS'])/f'src/The-Barn-Challenge-Ros2/jackal_helper/worlds/BARN/path_files/path_{self.args.world}.npy'))
        ref=np.vstack(([-2.25,3.],np.asarray(path)*.15+[-4.575,5.075],[-2.25,13.]))
        length=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum());optimal=length/2.
        metric=score(self.status=='success',elapsed,length)
        driven=float(np.linalg.norm(np.diff(np.asarray(self.rows)[:,1:3],axis=0),axis=1).sum()) if len(self.rows)>1 else 0.
        result={'method':self.args.method,'world':self.args.world,'trial':self.args.trial,'status':self.status,
                'elapsed_sim_s':elapsed,'path_length_m':driven,'reference_path_length_m':length,
                'upstream_runner_score':metric,'speed_cap_mps':.5,'yaw_rate_cap_radps':1.,
                'collision_signal':'Gazebo TouchPlugin','goal_tolerance_m':1.,'timeout_sim_s':self.args.timeout,
                'configuration_sha256':hashlib.sha256(
                    Path(__file__).resolve().with_name('benchmark.launch.py').read_bytes()+
                    Path(__file__).resolve().read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/The-Barn-Challenge-Ros2/jackal_helper/config/nav2.yaml').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/planner_manager.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/wheel_timing.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/closed_loop_controller.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/src/scan_replan_fsm.cpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/spatial_tracker.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/wheel_shortcut.hpp').read_bytes()+
                    (Path(os.environ['BENCH_WS'])/'src/slash_planner/plan_manage/include/plan_manage/static_motion_safety.hpp').read_bytes()).hexdigest(),
                'inputs_seen':{'truth':self.truth is not None,'odometry':self.odom is not None,'scan':self.scan is not None},
                'protocol':'static_jackal_matched_caps_v1; not historical official leaderboard reproduction'}
        (dest/(name+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        self.get_logger().info(json.dumps(result))
        cmd=TwistStamped();cmd.header.stamp=self.get_clock().now().to_msg();self.vel_pub.publish(cmd)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--method',choices=['mppi','slash'],required=True)
    parser.add_argument('--world',type=int,default=0);parser.add_argument('--trial',type=int,default=0)
    parser.add_argument('--timeout',type=float,default=120);parser.add_argument('--wall-timeout',type=float,default=600)
    parser.add_argument('--output',default=str(Path(__file__).resolve().parent/'results'));parser.add_argument('--launch-pid',type=int)
    args,ros=parser.parse_known_args();rclpy.init(args=ros);node=Evaluation(args);last_health=0.
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
