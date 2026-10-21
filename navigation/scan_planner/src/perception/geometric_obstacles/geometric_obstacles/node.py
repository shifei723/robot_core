#!/usr/bin/env python3
"""Depth-only obstacle tracking. All estimates are expressed in continuous world TF."""
import time
import copy
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String, Header
import json
from geometric_obstacles.static_prior import StaticPrior
from geometric_obstacles.online_background import BackgroundConfig, OnlineBackground, protected_points
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scan_planner_msgs.msg import DynamicObstacle, DynamicObstacleArray
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from geometric_obstacles.core import Config, Tracker, project, rotation


class GeometricObstacles(Node):
    def __init__(self):
        super().__init__('geometric_obstacles')
        def param(name, value): return self.declare_parameter(name, value).value
        self.world = param('world_frame', 'world')
        self.cfg = Config(**{k: param(k, v) for k, v in vars(Config()).items()})
        self.max_age = param('max_frame_age', .4)
        self.minimum_valid = param('minimum_valid_fraction', .05)
        self.max_translation = param('pose_jump_distance', .3)
        self.max_rotation = param('pose_jump_angle', .35)
        self.prior_enabled = param('static_prior_enabled',False)
        self.prior_topic = param('static_prior_topic','/cloud_obstacles')
        self.prior_voxel = param('static_prior_voxel',.05)
        self.prior_distance = param('static_prior_match_distance',.10)
        self.prior_radius = param('static_prior_local_radius',8.)
        self.online_enabled = param('online_background_enabled',True)
        self.online = OnlineBackground(BackgroundConfig(**{k:param('online_'+k,v) for k,v in vars(BackgroundConfig()).items()}))
        self.prior = None; self.prior_frame = ''; self.prior_transform = None
        self.last_prior_publish = -1.; self.prior_stats = {}
        self.prior_pub = self.create_publisher(PointCloud2,'/dynamic_obstacles/static_prior',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.stats_pub = self.create_publisher(String,'/dynamic_obstacles/perception_stats',10)
        if self.prior_enabled:
            # Support both latched prebuilt maps and ordinary periodic ROS clouds.
            self.create_subscription(PointCloud2,self.prior_topic,self.on_prior,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.create_subscription(PointCloud2,self.prior_topic,self.on_prior,1)
        self.tracker = Tracker(self.cfg)
        self.bridge = CvBridge(); self.info = None; self.pending = None
        self.epoch = time.time_ns(); self.fault = ''; self.last_pose = None
        self.last_success = None
        self.rectification_key = None; self.rectification_maps = None
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.array_pub = self.create_publisher(DynamicObstacleArray, '/dynamic_obstacles/tracks', 10)
        self.depth_pub = self.create_publisher(Image, '/dynamic_obstacles/depth_static', 10)
        self.pose_pub = self.create_publisher(Odometry, '/dynamic_obstacles/camera_pose', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/dynamic_obstacles/camera_info', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/dynamic_obstacles/markers', 10)
        self.cloud_pubs = {s: self.create_publisher(PointCloud2, '/dynamic_obstacles/cloud_'+s, 2)
                           for s in ('dynamic', 'static', 'unknown')}
        self.create_subscription(CameraInfo, param('camera_info_topic', '/camera/camera/color/camera_info'), self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, param('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'), self.on_depth, qos_profile_sensor_data)
        self.create_timer(.02, self.process)
        self.create_timer(.1, self.watchdog)

    def on_prior(self,msg):
        if self.prior is not None: return # Freeze the navigation background; restart to replace it.
        try:
            if not msg.header.frame_id:raise ValueError('static map has no frame_id')
            points=point_cloud2.read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)
            self.prior=StaticPrior(points,self.prior_voxel,self.prior_distance)
            self.prior_frame=msg.header.frame_id
            self.get_logger().info(f'Frozen static prior: {len(self.prior.points)} voxels in {self.prior_frame}')
        except (ValueError,RuntimeError,AssertionError) as exc:
            self.get_logger().error(f'Invalid static prior: {exc}')
            self.unhealthy('invalid static prior')

    def prior_filter(self,points,pixels,T,stamp,depth=None,K=None):
        seconds=stamp.nanoseconds*1e-9
        known=np.zeros(len(points),bool)
        M=np.eye(4)
        if self.prior_enabled:
            if self.prior_frame!=self.world:
                tr=self.tf.lookup_transform(self.prior_frame,self.world,stamp).transform
                q=tr.rotation;v=tr.translation
                M[:3,:3]=rotation([q.x,q.y,q.z,q.w]);M[:3,3]=[v.x,v.y,v.z]
            if self.prior_transform is not None:
                D=M@np.linalg.inv(self.prior_transform)
                angle=np.arccos(np.clip((np.trace(D[:3,:3])-1)/2,-1,1))
                if np.linalg.norm(D[:3,3])>self.max_translation or angle>self.max_rotation:
                    raise ValueError('map/world discontinuity: restart perception and replan after relocalization')
            self.prior_transform=M
            known=self.prior.known(points@M[:3,:3].T+M[:3,3])
        online_known=np.zeros(len(points),bool)
        removed=0
        if self.online_enabled:
            if depth is not None:
                removed=self.online.observe_free(depth,K,T,seconds,self.cfg.min_depth,self.cfg.max_depth)
            # Fixed-map points do not trigger reinspection of nearby objects.
            foreground=(~known)&(points[:,2]>self.cfg.ground_z+self.cfg.ground_band)&(points[:,2]<self.cfg.ground_z+self.cfg.max_height)
            for track in self.online.wake_for_changes(points[foreground],seconds,self.tracker.tracks):
                self.tracker.tracks[track.id]=track
            online_known=self.online.known(points,seconds)
        protected=protected_points(points,self.tracker.tracks.values(),seconds)
        fixed_background=self.prior.known(points@M[:3,:3].T+M[:3,3],protected) if self.prior_enabled else known
        background=fixed_background|(online_known&~protected)
        self.prior_stats={'input_points':len(points),'known_static_points':int(known.sum()),
                          'online_matched_points':int(online_known.sum()),
                          'protected_background_points':int(((known|online_known)&~background).sum()),
                          'residual_points':int((~background).sum()),
                          'prior_voxels':len(self.prior.points) if self.prior else 0,
                          'online_cleared_this_frame':removed}
        return points[~background],pixels[~background]

    def publish_background(self,T,stamp):
        # GridMap replaces this snapshot atomically. The fixed map is never
        # mutated; only the separately confirmed online layer can be cleared.
        if not self.prior_enabled:return
        seconds=stamp.nanoseconds*1e-9
        if seconds-self.last_prior_publish<.2:return
        M=self.prior_transform
        center=M[:3,:3]@T[:3,3]+M[:3,3]
        crop=self.prior.local(center,self.prior_radius)
        world=(crop-M[:3,3])@M[:3,:3]
        if self.online_enabled and len(self.online.points):
            local=self.online.points[np.linalg.norm(self.online.points-T[:3,3],axis=1)<=self.prior_radius]
            world=np.vstack((world,local))
        world=world[world[:,2]>self.cfg.ground_z+.05]
        header=Header();header.frame_id=self.world;header.stamp=stamp.to_msg()
        self.prior_pub.publish(point_cloud2.create_cloud_xyz32(header,world.astype(np.float32)))
        self.last_prior_publish=seconds

    def on_info(self, msg): self.info = msg
    def on_depth(self, msg): self.pending = msg  # Latest only; never build a latency queue.

    def unhealthy(self, reason):
        msg = DynamicObstacleArray()
        msg.header.frame_id = self.world; msg.header.stamp = self.get_clock().now().to_msg()
        msg.epoch = self.epoch; msg.healthy = False; msg.status = reason
        self.array_pub.publish(msg)

    def watchdog(self):
        if self.fault: self.unhealthy(self.fault)
        elif self.last_success is None or time.monotonic()-self.last_success > self.max_age:
            self.unhealthy('no fresh depth/pose')

    def process(self):
        if self.fault or self.pending is None: return
        if self.prior_enabled and self.prior is None:
            self.unhealthy('waiting for prebuilt static map');return
        msg = self.pending
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds*1e-9
        if age < -.05 or age > self.max_age:
            self.pending = None; self.unhealthy('depth timestamp stale/future'); return
        if self.info is None:
            self.unhealthy('waiting for CameraInfo'); return
        info = self.info
        if (info.width, info.height, info.header.frame_id) != (msg.width, msg.height, msg.header.frame_id):
            self.pending = None; self.unhealthy('CameraInfo dimensions/frame mismatch'); return
        if info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset:
            self.pending = None; self.unhealthy('unsupported CameraInfo ROI/binning'); return
        K = np.array(info.k).reshape(3,3)
        if not np.isfinite(K).all() or K[0,0] <= 0 or K[1,1] <= 0:
            self.pending = None; self.unhealthy('invalid camera intrinsics'); return
        # Wait briefly for TF at exposure time, without blocking the executor.
        if not self.tf.can_transform(self.world, msg.header.frame_id, stamp): return
        if self.prior_enabled and self.prior_frame!=self.world and not self.tf.can_transform(self.prior_frame,self.world,stamp):
            self.unhealthy('waiting for map/world TF at depth timestamp');return
        self.pending = None
        try:
            transform = self.tf.lookup_transform(self.world, msg.header.frame_id, stamp).transform
            q = transform.rotation; p = transform.translation
            T = np.eye(4); T[:3,:3] = rotation([q.x,q.y,q.z,q.w]); T[:3,3] = [p.x,p.y,p.z]
            if not np.isfinite(T).all(): raise ValueError('nonfinite camera pose')
            if self.last_pose is not None:
                angle = np.arccos(np.clip((np.trace(T[:3,:3] @ self.last_pose[:3,:3].T)-1)/2,-1,1))
                if np.linalg.norm(T[:3,3]-self.last_pose[:3,3]) > self.max_translation or angle > self.max_rotation:
                    raise ValueError('pose discontinuity: restart perception AND replan after localization recovery')
            raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if msg.encoding == '16UC1': depth = raw.astype(float)*.001
            elif msg.encoding == '32FC1': depth = raw.astype(float)
            else: raise ValueError('unsupported depth encoding')
            # Rectify the depth pixel grid using nearest sampling; never interpolate
            # foreground/background depths across an occlusion boundary.
            distortion = np.asarray(info.d, float)
            if not np.isfinite(distortion).all(): raise ValueError('invalid distortion coefficients')
            if np.any(np.abs(distortion)>1e-10):
                if info.distortion_model not in ('plumb_bob','rational_polynomial'):
                    raise ValueError('unsupported distortion model; supply rectified depth and CameraInfo')
                key = (msg.width,msg.height,tuple(info.k),tuple(info.d))
                if key != self.rectification_key:
                    self.rectification_maps = cv2.initUndistortRectifyMap(K,distortion,np.eye(3),K,
                        (msg.width,msg.height),cv2.CV_32FC1)
                    self.rectification_key = key
                depth = cv2.remap(depth,*self.rectification_maps,cv2.INTER_NEAREST,
                                  borderMode=cv2.BORDER_CONSTANT,borderValue=0)
            info = copy.deepcopy(info); info.d = [0.]*5; info.distortion_model = 'plumb_bob'
            points, pixels, valid = project(depth,K,T,self.cfg)
            if valid < self.minimum_valid:
                self.unhealthy('insufficient valid depth'); return
            started=time.perf_counter()
            residual,uv=self.prior_filter(points,pixels,T,stamp,depth,K)
            self.tracker.process(residual,uv,stamp.nanoseconds*1e-9)
            if self.online_enabled:
                handed_off=self.online.update(self.tracker.tracks.values(),points,pixels,stamp.nanoseconds*1e-9)
                for identity in handed_off:self.tracker.tracks.pop(identity,None)
            self.publish_background(T,stamp)
            if self.prior_enabled or self.online_enabled:
                stats=String();stats.data=json.dumps({**self.prior_stats,'online_background_points':len(self.online.points),
                    'online_promotions':self.online.promoted_total,'online_reactivations':self.online.reactivated_total,'online_cleared_total':self.online.cleared_total,
                    'filter_and_tracking_ms':(time.perf_counter()-started)*1000,
                    'visibility_corrected_tracks':sum(t.observed and t.visibility_corrected for t in self.tracker.tracks.values())})
                self.stats_pub.publish(stats)
            healthy = self.tracker.retire(stamp.nanoseconds*1e-9,depth,K,T)
            self.last_pose = T
            # Account for computation latency; an old result must never unlock the controller.
            if (self.get_clock().now()-stamp).nanoseconds*1e-9 > self.max_age:
                self.unhealthy('processing deadline exceeded'); return
            self.publish(msg,info,depth,T,transform,points,pixels,healthy)
            self.last_success = time.monotonic()
        except (ValueError, RuntimeError, np.linalg.LinAlgError, cv2.error) as exc:
            self.fault = str(exc); self.get_logger().error(self.fault); self.unhealthy(self.fault)

    def publish(self, source, info, depth, T, transform, points, pixels, healthy):
        out = DynamicObstacleArray(); out.header.stamp = source.header.stamp
        out.header.frame_id = self.world; out.epoch = self.epoch
        out.healthy = healthy; out.status = 'ok' if healthy else 'unresolved occlusion'
        mask = np.zeros(depth.shape,np.uint8); unknown = np.zeros_like(mask)
        markers = MarkerArray(); clear = Marker(); clear.action = Marker.DELETEALL; markers.markers.append(clear)
        for t in self.tracker.tracks.values():
            if t.state == 'dynamic' and t.observed: mask[t.pixels[:,0],t.pixels[:,1]] = 1
            elif t.state == 'tentative' and t.observed: unknown[t.pixels[:,0],t.pixels[:,1]] = 1
            if t.observed and t.state != 'static' and len(t.pixels):
                # Clustering excludes the floor band. Mask the object's contact
                # band too, or its moving feet/bottom leave a permanent low wall.
                # This only omits rays: it never declares hidden ground free.
                ground = np.array([[t.x[0]+dx,t.x[1]+dy,self.cfg.ground_z]
                                   for dx in [-t.radius,t.radius] for dy in [-t.radius,t.radius]])
                optical = (ground-T[:3,3]) @ T[:3,:3]
                visible = optical[:,2] > self.cfg.min_depth
                if np.any(visible):
                    rows = np.asarray(info.k).reshape(3,3)[1,1]*optical[visible,1]/optical[visible,2]+info.k[5]
                    bottom = min(depth.shape[0],int(np.ceil(rows.max()))+self.cfg.stride+1)
                    top = int(t.pixels[:,0].max())
                    left,right = int(t.pixels[:,1].min()),int(t.pixels[:,1].max())+1
                    target_mask = mask if t.state == 'dynamic' else unknown
                    target_mask[top:bottom,left:right] = 1
            # Confident stationary clusters remain solely in the static map.
            if t.state == 'static' and t.observed: continue
            item = DynamicObstacle(); item.id = 'obstacle_'+str(t.id)
            item.state = item.COASTING if not t.observed else (item.DYNAMIC if t.state == 'dynamic' else item.TENTATIVE)
            item.observed = t.observed
            item.last_observed = Time(seconds=t.last_seen).to_msg()
            item.position.x,item.position.y,item.position.z = float(t.x[0]),float(t.x[1]),float(t.z)
            item.velocity.x,item.velocity.y = float(t.x[2]),float(t.x[3])
            item.radius = float(t.radius); item.covariance = t.P.ravel().tolist()
            out.obstacles.append(item)
            m = Marker(); m.header = out.header; m.ns = 'tracked_obstacles'; m.id = t.id
            m.type = Marker.CYLINDER; m.action = Marker.ADD; m.pose.position = item.position; m.pose.orientation.w = 1.
            m.scale.x = m.scale.y = 2*t.radius; m.scale.z = .5
            m.color.r = 1.; m.color.g = .5 if t.state == 'tentative' else 0.; m.color.a = .6
            markers.markers.append(m)
        self.array_pub.publish(out); self.marker_pub.publish(markers)
        # 65535 means OMIT THIS RAY, never free space. Dilation covers stride gaps.
        mask = cv2.dilate(mask,np.ones((self.cfg.stride*2+1,)*2,np.uint8)).astype(bool)
        filtered = np.clip(np.nan_to_num(depth,nan=0,posinf=0,neginf=0)*1000,0,65534).astype(np.uint16)
        # Tentative objects already live in the conservative track bank. Do not
        # commit their first frames to the persistent static occupancy map.
        pending_mask = cv2.dilate(unknown,np.ones((self.cfg.stride*2+1,)*2,np.uint8)).astype(bool)
        filtered[mask | pending_mask] = 65535
        if healthy:
            # Runtime intrinsics precede depth/pose, but mapper also checks matching timestamp.
            info.header.stamp = source.header.stamp; self.info_pub.publish(info)
            img = self.bridge.cv2_to_imgmsg(filtered,'16UC1'); img.header = source.header
            pose = Odometry(); pose.header = out.header; pose.child_frame_id = source.header.frame_id
            pose.pose.pose.position.x = transform.translation.x
            pose.pose.pose.position.y = transform.translation.y
            pose.pose.pose.position.z = transform.translation.z
            pose.pose.pose.orientation = transform.rotation
            self.depth_pub.publish(img); self.pose_pub.publish(pose)
        dyn = mask[pixels[:,0],pixels[:,1]]; unk = unknown[pixels[:,0],pixels[:,1]].astype(bool)&~dyn
        for name, keep in [('dynamic',dyn),('unknown',unk),('static',~dyn&~unk)]:
            xyz = np.asarray(points[keep],dtype=np.float32)
            cloud = PointCloud2(); cloud.header = out.header; cloud.height = 1; cloud.width = len(xyz)
            cloud.fields = [PointField(name=n,offset=i*4,datatype=PointField.FLOAT32,count=1) for i,n in enumerate(('x','y','z'))]
            cloud.point_step = 12; cloud.row_step = 12*len(xyz); cloud.is_dense = True; cloud.data = xyz.tobytes()
            self.cloud_pubs[name].publish(cloud)


def main():
    rclpy.init(); node = GeometricObstacles()
    try: rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__': main()
