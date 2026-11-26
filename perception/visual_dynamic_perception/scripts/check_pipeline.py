#!/usr/bin/python3
"""Sample live VINS inputs/outputs, drift and timestamp availability (no ground truth)."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,ReliabilityPolicy
from sensor_msgs.msg import Imu,Image,PointCloud2
from nav_msgs.msg import Odometry
from tf2_ros import Buffer,TransformListener

def main():
 p=argparse.ArgumentParser();p.add_argument('--backend',choices=('vins','openvins'),default='vins');p.add_argument('--seconds',type=float,default=30);p.add_argument('--output',default='runtime/336l/vins_live_check.json');a=p.parse_args()
 rclpy.init();n=rclpy.create_node('vins_live_validation');b=Buffer();l=TransformListener(b,n);samples={};subs=[];q=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE)
 topics={'/camera/gyro_accel/sample':Imu,'/camera/left_ir/image_raw':Image,'/camera/right_ir/image_raw':Image,'/vins/odometry':Odometry,'/vins/imu_propagate':Odometry,'/detection/odometry':Odometry,'/onboard_detector/filtered_depth_cloud':PointCloud2}
 if a.backend=='openvins':
  topics.pop('/vins/odometry');topics.pop('/vins/imu_propagate');topics['/openvins/odomimu']=Odometry
 def cb(m,t):
  st=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;row={'stamp':st,'age':n.get_clock().now().nanoseconds*1e-9-st}
  if isinstance(m,Odometry):row.update(position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],quaternion=[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w],frame=m.child_frame_id)
  if isinstance(m,PointCloud2):row['points']=m.width*m.height
  samples.setdefault(t,[]).append(row)
 for t,c in topics.items():subs.append(n.create_subscription(c,t,lambda m,t=t:cb(m,t),q))
 start=time.monotonic()
 while time.monotonic()-start<a.seconds:rclpy.spin_once(n,timeout_sec=.02)
 report={'duration_s':a.seconds,'topics':{}}
 for t,rows in samples.items():
  stamps=np.array([x['stamp'] for x in rows]);d={'count':len(rows),'hz':len(rows)/a.seconds,'max_stamp_gap_s':float(np.diff(stamps).max()) if len(stamps)>1 else None,'median_age_s':float(np.median([x['age'] for x in rows]))}
  if 'position' in rows[0]:
   xyz=np.array([x['position'] for x in rows]);d.update(position_span_m=np.ptp(xyz,axis=0).tolist(),net_displacement_m=float(np.linalg.norm(xyz[-1]-xyz[0])),finite=bool(np.isfinite(xyz).all()),last_position_m=xyz[-1].tolist(),child_frame=rows[-1]['frame'])
  if 'points' in rows[0]:d['nonempty']=sum(x['points']>0 for x in rows)
  report['topics'][t]=d
 try:
  tf=b.lookup_transform('map','camera_link',rclpy.time.Time());report['tf_latest_age_s']=n.get_clock().now().nanoseconds*1e-9-(tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9)
 except Exception as e:report['tf_error']=str(e)
 Path(a.output).write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
