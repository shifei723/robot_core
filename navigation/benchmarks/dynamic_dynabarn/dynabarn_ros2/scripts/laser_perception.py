#!/usr/bin/env python3
"""Common native-lidar input and static background; no obstacle truth subscription.
Uses robot_core geometric clustering/KF, with a measured-circle observation for
this cylindrical benchmark. World is the continuous odometry frame (identity TF).
"""
import copy,json,math,time
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan,PointCloud2,Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header,String,Float64MultiArray
from scan_planner_msgs.msg import DynamicObstacleArray,DynamicObstacle
from tf2_ros import Buffer,TransformListener,StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from geometric_obstacles.core import laser_region_free, Config,Tracker
from map_geometry import static_wall_points,fit_circle

def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

class Perception(Node):
 def __init__(self):
  super().__init__('dynabarn_laser_perception')
  world=self.declare_parameter('world_file','').value
  self.require_imu=bool(self.declare_parameter('require_imu',False).value)
  self.imu_stamp=-1.
  self.start_x=float(self.declare_parameter('start_world_x',12.).value)
  self.start_yaw=float(self.declare_parameter('start_world_yaw',math.pi).value)
  self.walls=static_wall_points(world);self.tree=None;self.anchor=None;self.odom=None;self.last_sensor=-1.;self.epoch=time.time_ns()
  self.cfg=Config(lost_timeout=self.declare_parameter("lost_timeout",1.0).value,stride=1,voxel=.06,min_points=5,max_depth=5.,measurement_sigma=.08,accel_sigma=.4,max_cluster_extent=2.5)
  self.tracker=Tracker(self.cfg);self.pending=None
  self.tf=Buffer();self.listener=TransformListener(self.tf,self)
  self.static_tf=StaticTransformBroadcaster(self);t=TransformStamped();t.header.frame_id='world';t.child_frame_id='odom';t.transform.rotation.w=1.;self.static_tf.sendTransform(t)
  latch=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
  self.anchor_pub=self.create_publisher(Float64MultiArray,'/benchmark/anchor',latch)
  self.prior=self.create_publisher(PointCloud2,'/dynamic_obstacles/static_prior',latch)
  self.map=self.create_publisher(OccupancyGrid,'/benchmark/static_map',latch)
  self.body=self.create_publisher(Odometry,'/benchmark/body_pose',10)
  self.sensor=self.create_publisher(Odometry,'/benchmark/sensor_pose',10)
  self.cloud=self.create_publisher(PointCloud2,'/benchmark/static_cloud',10)
  self.array=self.create_publisher(DynamicObstacleArray,'/dynamic_obstacles/tracks',10)
  self.stats=self.create_publisher(String,'/dynamic_obstacles/perception_stats',10)
  self.create_subscription(Imu,'/sensors/imu_0/data',self.on_imu,qos_profile_sensor_data)
  self.create_subscription(Odometry,'/platform/odom/filtered',self.on_odom,qos_profile_sensor_data)
  self.create_subscription(LaserScan,'/front/scan',self.on_scan,qos_profile_sensor_data)
  self.create_timer(.2,self.publish_prior);self.create_timer(.1,self.watchdog);self.create_timer(.02,self.process_scan)
 def on_imu(self,m):
  self.imu_stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
 def now_s(self):return self.get_clock().now().nanoseconds/1e9
 def on_odom(self,m):
  self.odom=m
  if self.anchor is None and self.now_s()>2.5:
   self.anchor=(m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation))
   angle=self.anchor[2]-self.start_yaw;R=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
   self.nav_walls=(self.walls-[self.start_x,0.])@R.T+self.anchor[:2];self.tree=cKDTree(self.nav_walls)
   self.publish_grid();a=Float64MultiArray();a.data=list(self.anchor);self.anchor_pub.publish(a)
  b=copy.deepcopy(m);b.header.frame_id='world';b.pose.pose.position.z=.25;self.body.publish(b)
 def publish_grid(self):
  m=OccupancyGrid();m.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='odom')
  m.info.resolution=.1;m.info.width=m.info.height=600;m.info.origin.position.x=m.info.origin.position.y=-30.;m.info.origin.orientation.w=1.
  cells=np.floor((self.nav_walls+30)/.1).astype(int);g=np.zeros((600,600),np.int8)
  ok=np.all((cells>=0)&(cells<600),axis=1);g[cells[ok,1],cells[ok,0]]=100;m.data=g.ravel().tolist();self.map.publish(m)
 def publish_prior(self):
  if self.tree is None:return
  xy=np.repeat(self.nav_walls,9,axis=0);z=np.tile(np.arange(.15,1.,.1),len(self.nav_walls))
  h=Header(stamp=self.get_clock().now().to_msg(),frame_id='world');self.prior.publish(create_cloud_xyz32(h,np.c_[xy,z]))
 def invalid(self,reason):
  m=DynamicObstacleArray();m.header=Header(stamp=self.get_clock().now().to_msg(),frame_id='world');m.epoch=self.epoch;m.healthy=False;m.status=reason;self.array.publish(m)
 def watchdog(self):
  if self.require_imu and not 0<=self.now_s()-self.imu_stamp<.35:self.invalid('no fresh native IMU')
  elif self.now_s()-self.last_sensor>.35:self.invalid('no fresh native laser/TF')
 def on_scan(self,m):
  self.pending=m
 def process_scan(self):
  m=self.pending
  if m is None:return
  if self.tree is None or self.odom is None:return
  if self.require_imu and not 0<=self.now_s()-self.imu_stamp<.35:
   self.invalid('no fresh native IMU');return
  begin=time.perf_counter();stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
  if self.now_s()-stamp>.35:self.pending=None;return
  try:tr=self.tf.lookup_transform('odom',m.header.frame_id,Time.from_msg(m.header.stamp)).transform
  except Exception:return
  self.pending=None
  position=np.array([tr.translation.x,tr.translation.y]);heading=yaw(tr.rotation)
  a=m.angle_min+np.arange(len(m.ranges))*m.angle_increment;raw=np.asarray(m.ranges)
  valid=np.isfinite(raw)&(raw>=max(.35,m.range_min))&(raw<=min(5.,m.range_max));ids=np.flatnonzero(valid)
  xy=position+np.c_[np.cos(a[valid]+heading),np.sin(a[valid]+heading)]*raw[valid,None]
  keep=np.linalg.norm(xy-[self.odom.pose.pose.position.x,self.odom.pose.pose.position.y],axis=1)>.40
  xy=xy[keep];ids=ids[keep];points=np.c_[xy,np.full(len(xy),.25)]
  known=self.tree.query(xy)[0]<.16 if len(xy) else np.zeros(0,bool)
  residual=points[~known];pixels=np.c_[np.zeros((~known).sum(),int),ids[~known]]
  try:
   clusters=self.tracker.clusters(residual,pixels)
   fit_count=0;reliable=[];dynamic_pixels=[]
   for c in clusters:
    p=residual[np.isin(pixels[:,1],c.pixels[:,1])];f=fit_circle(p)
    if f is not None:
     center,radius,error=f;c.center[:2]=center;c.radius=radius+max(.04,3*error);c.bounds=None;fit_count+=1;reliable.append(c);dynamic_pixels.extend(c.pixels[:,1].tolist())
   self.tracker.step(stamp,reliable)
   self.tracker.previous=residual.copy();self.tracker.reference_time=stamp
  except (ValueError,np.linalg.LinAlgError) as e:self.invalid(str(e));return
  for identity,t in list(self.tracker.tracks.items()):
   if t.observed:continue
   r=t.radius+3*np.sqrt(max(t.P[0,0],t.P[1,1]));d=t.x[:2]-position
   if np.linalg.norm(d)>self.cfg.retire_distance:del self.tracker.tracks[identity];continue
   if laser_region_free(t.x[:2],r,position,a+heading,raw,min(5.,m.range_max)):
    del self.tracker.tracks[identity]
  out=DynamicObstacleArray();out.header=Header(stamp=m.header.stamp,frame_id='world');out.epoch=self.epoch;out.healthy=True;out.status='native laser, static-map residuals'
  for t in self.tracker.tracks.values():
   o=DynamicObstacle();o.id=f'obstacle_{t.id}';o.observed=t.observed;o.last_observed=Time(seconds=t.last_seen).to_msg();o.state=o.UNKNOWN if t.unknown else o.COASTING if not t.observed else (o.DYNAMIC if t.state=='dynamic' else o.TENTATIVE)
   o.position.x=float(t.x[0]);o.position.y=float(t.x[1]);o.position.z=.25;o.velocity.x=float(t.x[2]);o.velocity.y=float(t.x[3]);o.radius=float(t.radius);o.covariance=t.P.ravel().tolist();out.obstacles.append(o)
  self.array.publish(out);self.last_sensor=stamp
  sensor=Odometry();sensor.header=out.header;sensor.pose.pose.position.x=float(position[0]);sensor.pose.pose.position.y=float(position[1]);sensor.pose.pose.position.z=.25;sensor.pose.pose.orientation=tr.rotation;self.sensor.publish(sensor)
  # Keep unfitted measurements as occupied, and add real no-hit rays to clear
  # their old cells. Omit reliably fitted moving rays entirely, never clear
  # free-space behind a tracked object. GridMap treats >5m as clearing-only.
  ray_ok=((np.isfinite(raw)&(raw>=max(.35,m.range_min)))|np.isposinf(raw))
  ray_ok &= ~np.isin(np.arange(len(raw)),dynamic_pixels)
  ranges=np.where(np.isposinf(raw)|(raw>5.),5.1,raw)[ray_ok]
  ray_xy=position+np.c_[np.cos(a[ray_ok]+heading),np.sin(a[ray_ok]+heading)]*ranges[:,None]
  clear=np.linalg.norm(ray_xy-[self.odom.pose.pose.position.x,self.odom.pose.pose.position.y],axis=1)>.40
  self.cloud.publish(create_cloud_xyz32(out.header,np.c_[ray_xy[clear],np.full(clear.sum(),.25)]))
  stats=String();stats.data=json.dumps({'stamp':stamp,'input':len(points),'residual':len(residual),'clusters':len(clusters),'circle_fits':fit_count,'tracks':len(out.obstacles),'unknown_regions':sum(t.unknown for t in self.tracker.tracks.values()),'coast_timeout_s':self.cfg.lost_timeout,'filter_and_tracking_ms':(time.perf_counter()-begin)*1000,'require_imu':self.require_imu,'imu_age_s':self.now_s()-self.imu_stamp,'source':'native LaserScan + odometry TF, no obstacle truth'})
  self.stats.publish(stats)

def main():
 rclpy.init();n=Perception()
 try:rclpy.spin(n)
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
