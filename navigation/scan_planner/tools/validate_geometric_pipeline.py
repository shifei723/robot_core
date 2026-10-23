#!/usr/bin/env python3
"""Headless depth-only closed-loop integration experiment.
Ground truth is used ONLY by the raycaster, kinematic plant and evaluator.
Actual geometric_obstacles, grid_map, planner and controller binaries consume ROS sensors.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--case',choices=['static','crossing','continuous_crossing','turnaround','stop_restart','encounter','multi_encounter','indoor_turn_height','dropout'],default='crossing')
    parser.add_argument('--static-prior',action='store_true')
    parser.add_argument('--shifted-map-frame',action='store_true')
    parser.add_argument('--duration',type=float,default=40)
    parser.add_argument('--domain',type=int,default=191)
    parser.add_argument('--timing-policy',choices=['soft','hard'],default='soft')
    parser.add_argument('--prediction',choices=['cv','frs'],default='frs')
    parser.add_argument('--camera-focal',type=float,default=220.)
    parser.add_argument('--encounter-end-y',type=float,default=None)
    parser.add_argument('--measurement-sigma',type=float,default=.06)
    parser.add_argument('--accel-sigma',type=float,default=.8)
    parser.add_argument('--frs-weight',type=float,default=2.0)
    parser.add_argument('--spatial-weight',type=float,default=1.5)
    parser.add_argument('--encounter-trigger-x',type=float,default=.6)
    parser.add_argument('--encounter-x',type=float,default=1.8)
    parser.add_argument('--encounter-speed',type=float,default=.22)
    parser.add_argument('--encounter-side',type=float,default=1.1)
    args=parser.parse_args()
    if min(args.frs_weight,args.spatial_weight)<0 or min(args.encounter_speed,args.encounter_side)<=0:
        parser.error('weights must be nonnegative and encounter scales positive')
    if args.encounter_end_y is None:args.encounter_end_y=args.encounter_side
    numeric=[args.camera_focal,args.measurement_sigma,args.accel_sigma,args.frs_weight,args.spatial_weight,args.encounter_speed,args.encounter_side,args.encounter_end_y,args.encounter_x,args.encounter_trigger_x]
    if not all(math.isfinite(x) for x in numeric) or min(args.camera_focal,args.measurement_sigma,args.accel_sigma)<=0 or args.encounter_end_y<=-args.encounter_side:
        parser.error('invalid camera, uncertainty, or encounter parameters')
    encounter_started=None
    multi_specs=[{'id':'truth_0','x':1.8,'start_y':-1.1,'end_y':.8,'speed':.22,'trigger_x':.6},
                 {'id':'truth_1','x':2.5,'start_y':1.1,'end_y':-.8,'speed':.22,'delay_after_first':6.},
                 {'id':'truth_2','x':3.7,'start_y':-1.1,'end_y':.8,'speed':.22,'trigger_x':2.5}]
    multi_onsets={}
    goal_xy=np.array([4.9 if args.case=='multi_encounter' else 3.,0.])
    import yaml
    reference_path=Path(__file__).resolve().parents[1]/'src/planner/plan_manage/config/simulator.yaml'
    reference=yaml.safe_load(reference_path.read_text())['mockamap_node']['ros__parameters']
    indoor=args.case=='indoor_turn_height'
    if indoor:
        goal_xy=np.array([4.,4.7])
        multi_specs=[{'id':'truth_0','start':[1.8,-1.1],'end':[1.8,.8],'speed':args.encounter_speed,'trigger_axis':0,'trigger_value':.5},
                     {'id':'truth_1','start':[2.9,1.5],'end':[4.85,1.5],'speed':args.encounter_speed,'trigger_axis':1,'trigger_value':.2},
                     {'id':'truth_2','start':[2.9,3.6],'end':[4.85,3.6],'speed':args.encounter_speed,'trigger_axis':1,'trigger_value':2.25}]
    static=[([-2.,-1.65,0.],[8.,-1.55,2.]),([-2.,1.55,0.],[8.,1.65,2.]),([7.9,-1.65,0.],[8.,1.65,2.])]
    if indoor:
        static=[([-2.,-1.65,0.],[5.65,-1.55,2.]),
                ([-2.,1.55,0.],[2.45,1.65,2.]),
                ([2.35,1.55,0.],[2.45,6.,2.]),
                ([5.55,-1.65,0.],[5.65,6.,2.]),
                ([2.35,5.9,0.],[5.65,6.,2.]),
                ([2.45,2.5-reference['indoor.beam_depth']/2,reference['indoor.beam_clearance']],[5.55,2.5+reference['indoor.beam_depth']/2,.9])]
    if args.case in ('static','dropout'):static.append(([1.3,.65,0.],[1.7,1.05,.8]))
    args.output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from ament_index_python.packages import get_package_prefix,get_package_share_directory
    from geometry_msgs.msg import PoseStamped,Twist,TransformStamped
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,CameraInfo,PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import String,Float32
    from scan_planner_msgs.msg import DynamicObstacleArray,Bspline
    from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
    from cv_bridge import CvBridge
    from scipy.spatial.transform import Rotation
    import yaml
    rclpy.init();node=rclpy.create_node('geometric_pipeline_simulator');bridge=CvBridge();tf=TransformBroadcaster(node)
    width,height=320,180;fx=fy=args.camera_focal;cx=width/2;cy=height/2
    v,u=np.mgrid[:height,:width]
    optical_rays=np.stack(((u-cx)/fx,(v-cy)/fy,np.ones_like(u)),axis=-1).reshape(-1,3)
    optical_to_body=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    state=np.array([0.,0.,0.]);base_z=.2822
    command=np.zeros(2);last_command=0.;start=None
    data={'tracks':None,'decision':None,'map':np.empty((0,3)),'plans':0,'filtered':None,'status':'','height_target':.2822}
    histories={'poses':[],'tracks':[],'decisions':[],'maps':[],'plans':[],'sensors':[],'perception_stats':[]}
    snapshots=[];processes=[];logs=[]
    pubs={
      'depth':node.create_publisher(Image,'/sim/depth',qos_profile_sensor_data),
      'info':node.create_publisher(CameraInfo,'/sim/camera_info',qos_profile_sensor_data),
      'odom':node.create_publisher(Odometry,'/sim/odom',qos_profile_sensor_data),
      'goal':node.create_publisher(PoseStamped,'/move_base_simple/goal',10)}
    def elapsed():return 0. if start is None else time.monotonic()-start
    def on_cmd(m):
        nonlocal last_command
        command[:]=m.linear.x,m.angular.z;last_command=time.monotonic()
    def on_tracks(m):
        row={'t':elapsed(),'healthy':m.healthy,'status':m.status,'age':(node.get_clock().now().nanoseconds-(m.header.stamp.sec*10**9+m.header.stamp.nanosec))*1e-9,
             'objects':[{'id':o.id,'state':o.state,'observed':o.observed,'p':[o.position.x,o.position.y],
                         'v':[o.velocity.x,o.velocity.y],'radius':o.radius,'covariance':list(o.covariance)} for o in m.obstacles]}
        data['tracks']=row;histories['tracks'].append(row)
    def on_decision(m):
        row=json.loads(m.data);row['t']=elapsed();data['decision']=row;histories['decisions'].append(row)
    def on_map(m):
        pts=np.asarray(point_cloud2.read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)).reshape(-1,3)
        data['map']=pts;histories['maps'].append({'t':elapsed(),'points':len(pts)})
    def on_plan(m):
        data['plans']+=1;histories['plans'].append({'t':elapsed(),'points':[[p.x,p.y,p.z] for p in m.pos_pts]})
    def on_filtered(m):data['filtered']=bridge.imgmsg_to_cv2(m).copy()
    subs=[node.create_subscription(String,'/dynamic_obstacles/perception_stats',lambda m:histories['perception_stats'].append({'t':elapsed(),**json.loads(m.data)}),10),
          node.create_subscription(Float32,'/sim/leg_height_cmd',lambda m:data.update(height_target=m.data+.0322),10),
          node.create_subscription(Twist,'/sim/cmd_vel',on_cmd,10),
          node.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',on_tracks,10),
          node.create_subscription(String,'/dynamic_navigation/decision',on_decision,10),
          node.create_subscription(String,'/height_control/state',lambda m:data.update(status=m.data),10),
          node.create_subscription(PointCloud2,'/grid_map/occupancy',on_map,qos_profile_sensor_data),
          node.create_subscription(Bspline,'/planning/bspline',on_plan,10),
          node.create_subscription(Image,'/dynamic_obstacles/depth_static',on_filtered,10)]
    params={'grid_map.static_prior_enabled':args.static_prior,'grid_map.sensor_type':'depth','grid_map.need_extrinsic':False,
      'grid_map.require_camera_info':True,'grid_map.dynamic_mask_sentinel':65535,
      'grid_map.skip_invalid_depth':True,'grid_map.k_depth_scaling_factor':1000.,
      'grid_map.allow_unverified_ceiling_clearance':not args.static_prior,'grid_map.world_bound_enable':False,
      'grid_map.double_cylinder_radius':.43,'grid_map.double_cylinder_offset':0.,
      'grid_map.sliding_map_size_x':10.,'grid_map.sliding_map_size_y':6.,'grid_map.sliding_map_size_z':3.,
      'grid_map.skip_pixel':2,'grid_map.local_update_range_y':3.,'grid_map.local_update_range_z':1.5,
      'manager.wheel_grid_trajectory':True,'manager.wheel_search_timeout':.25,'manager.dynamic_frs_weight':args.spatial_weight,
      'fsm.collision_horizon':3.,'fsm.collision_step':.05,'fsm.replan_interval':.4,
      'manager.max_vel':.3,'manager.max_acc':.3,'optimization.max_vel':.3,'optimization.max_acc':.3,
      'fsm.max_replan_fail_count':1000}
    if indoor:
        params.update({'manager.wheel_search_timeout':1.0,'grid_map.sliding_map_size_y':12.,'grid_map.sliding_map_size_x':12.,
                       'grid_map.local_update_range_y':5.,'grid_map.body_height':.328,
                       'grid_map.obstacles_inflation_z_down':.328,'grid_map.double_cylinder_radius':.50,
                       'grid_map.ceiling_margin':.05,'grid_map.height_min':.1722})
    paramfile=args.output/'planner_override.yaml';paramfile.write_text(yaml.safe_dump({'scan_planner_node':{'ros__parameters':params}}))
    def launch(package,exe,arguments):
        path=Path(get_package_prefix(package))/'lib'/package/exe
        f=(args.output/(exe+'.log')).open('w');logs.append(f)
        proc=subprocess.Popen([str(path),'--ros-args']+arguments,stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(proc)
    if args.static_prior:
        # Prebuilt STATIC surfaces only; dynamic object positions are never published here.
        xx,zz=np.meshgrid(np.arange(-2,8.01,.04),np.arange(.08,2.01,.04))
        prior=np.concatenate([np.column_stack((xx.ravel(),np.full(xx.size,y),zz.ravel())) for y in [-1.55,1.55]])
        if indoor:
            surfaces=[]
            for lo,hi in static:
                for axis in range(3):
                    others=[j for j in range(3) if j!=axis]
                    a,b=np.meshgrid(*[np.arange(lo[j],hi[j]+.02,.04) for j in others])
                    for edge in (lo[axis],hi[axis]):
                        pts=np.zeros((a.size,3));pts[:,axis]=edge
                        pts[:,others[0]]=a.ravel();pts[:,others[1]]=b.ravel();surfaces.append(pts)
            prior=np.concatenate(surfaces)
        prior_frame='world'
        if args.shifted_map_frame:
            prior_frame='prebuilt_map';angle=.25;Rmap=Rotation.from_euler('z',angle).as_matrix();offset=np.array([1.2,-.4,0.])
            prior=prior@Rmap.T+offset
            static_tf=StaticTransformBroadcaster(node)
            tr=TransformStamped();tr.header.stamp=node.get_clock().now().to_msg();tr.header.frame_id='world';tr.child_frame_id=prior_frame
            shift=-Rmap.T@offset;tr.transform.translation.x,tr.transform.translation.y,tr.transform.translation.z=map(float,shift)
            q=Rotation.from_matrix(Rmap.T).as_quat()
            tr.transform.rotation.x,tr.transform.rotation.y,tr.transform.rotation.z,tr.transform.rotation.w=map(float,q)
            static_tf.sendTransform(tr)
        prior_file=args.output/'prebuilt_static.pcd'
        with prior_file.open('w') as f:
            f.write(f'VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(prior)}\nHEIGHT 1\nPOINTS {len(prior)}\nDATA ascii\n')
            np.savetxt(f,prior,fmt='%.5f')
        launch('map_generator','map_pub',['-p',f'file_name:={prior_file}','-p',f'frame_id:={prior_frame}',
               '-p','downsample_res:=0.04','-r','global_cloud:=/sim/static_prior'])
    launch('geometric_obstacles','geometric_obstacles_node',['-p',f'static_prior_enabled:={str(args.static_prior).lower()}',
        '-p','static_prior_topic:=/sim/static_prior','-p',f'measurement_sigma:={args.measurement_sigma}','-p',f'accel_sigma:={args.accel_sigma}','-p','depth_topic:=/sim/depth','-p','camera_info_topic:=/sim/camera_info'])
    launch('scan_planner','scan_planner_node',['--params-file',str(Path(get_package_share_directory('scan_planner'))/'config/planner.yaml'),
       '--params-file',str(paramfile),'-r','body_pose:=/sim/odom','-r','sensor_pose:=/dynamic_obstacles/camera_pose',
       '-r','depth:=/dynamic_obstacles/depth_static'])
    launch('scan_planner','closed_loop_controller',['-p',f'static_prior_required:={str(args.static_prior).lower()}','-p',f'dynamic_soft_constraint:={str(args.timing_policy=="soft").lower()}','-p',f'dynamic_frs_weight:={args.frs_weight}','-p','dynamic_horizon:=3.0','-p','dynamic_input:=array','-p',f'dynamic_prediction:={args.prediction}',
       '-p','height_safety_enabled:=true','-r','cloud:=/dynamic_obstacles/cloud_static','-p','max_vx:=0.3','-p','max_vyaw:=0.5',
       '-r','body_pose:=/sim/odom','-r','cmd_vel:=/sim/cmd_vel',
       '-r','leg_height_cmd:=/sim/leg_height_cmd','-r','planning/go2_execution_frozen:=/planning/go2_execution_frozen'])
    def moving(t):
        if args.case=='multi_encounter' or indoor:
            objects=[]
            for spec in multi_specs:
                start_t=multi_onsets.get(spec['id'],{}).get('t')
                age=0. if start_t is None else max(0.,t-start_t)
                if indoor:
                    a=np.array(spec['start']);b=np.array(spec['end']);length=np.linalg.norm(b-a)
                    point=a+(b-a)*min(1.,spec['speed']*age/length)
                    objects.append((np.r_[point,0.],np.array([.22,.22,.9])))
                    continue
                direction=np.sign(spec['end_y']-spec['start_y'])
                y=spec['start_y']+direction*min(abs(spec['end_y']-spec['start_y']),spec['speed']*age)
                objects.append((np.array([spec['x'],y,0.]),np.array([.22,.22,.9])))
            return objects
        if args.case=='encounter':
            age=0. if encounter_started is None else max(0.,t-encounter_started)
            y=float(np.clip(-args.encounter_side+args.encounter_speed*age,-args.encounter_side,args.encounter_end_y))
            return [(np.array([args.encounter_x,y,0.]),np.array([.22,.22,.9]))]
        if args.case not in ('crossing','continuous_crossing','turnaround','stop_restart'):return []
        # Cross then stop at the side, remaining visible long enough to confirm stationary.
        motion_start=0. if args.case=='continuous_crossing' else 3.
        y=float(np.clip(-.95+.22*max(t-motion_start,0),-.95,.95))
        if args.case=='stop_restart':y=float(np.clip(-.75+.22*max(t-6.,0),-.75,.95))
        if args.case=='turnaround' and t>8.:y=float(np.clip(.15-.22*(t-8.),-.95,.95))
        return [(np.array([1.8,y,0.]),np.array([.22,.22,.9]))]
    def raycast(t):
        yaw=state[2];R=Rotation.from_euler('z',yaw).as_matrix()@optical_to_body
        origin=np.array([state[0],state[1],base_z+.1678])
        rays=optical_rays@R.T
        distance=np.full(len(rays),8.)
        floor=np.divide(-origin[2],rays[:,2],out=np.full(len(rays),np.inf),where=rays[:,2]<-1e-7)
        distance=np.minimum(distance,floor)
        boxes=[(np.array(a),np.array(b)) for a,b in static]
        for center,size in moving(t):boxes.append((center-np.r_[size[:2],0.],center+size))
        for lo,hi in boxes:
            with np.errstate(divide='ignore',invalid='ignore'):
                a=(lo-origin)/rays;b=(hi-origin)/rays
            near=np.max(np.minimum(a,b),axis=1);far=np.min(np.maximum(a,b),axis=1)
            hit=(far>=np.maximum(near,0))&(near>0)
            distance=np.minimum(distance,np.where(hit,near,np.inf))
        depth=distance.reshape(height,width)
        # Beyond range is represented by measured far depth, not injected free-space cloud.
        return np.clip(depth*1000,0,65534).astype(np.uint16),R,origin
    summary={'case':args.case,'reached':False,'collision':False,'error':None,'ground_truth_input_to_tracker':False,'prebuilt_static_map':args.static_prior,'shifted_map_frame':args.shifted_map_frame,
      'plant':'planar kinematics; ideal odometry, pinhole raycast depth; no contact physics or VINS estimator',
      'arrival_criterion':'distance < 0.16 m AND speed < 0.01 m/s AND controller GOAL_REACHED',
      'goal':goal_xy.tolist(),'camera_intrinsics':[fx,fy,cx,cy],'sensor_size':[width,height],'static_boxes':static}
    if indoor:summary['scene_reference']={'path':str(reference_path),'sha256':hashlib.sha256(reference_path.read_bytes()).hexdigest(),'beam_clearance':reference['indoor.beam_clearance'],'beam_depth':reference['indoor.beam_depth'],'motion':'Ground-plane linear segments, alternating crossings as in combined experiment; deterministic progress triggers; geometry adapted to L corridor.'}
    summary['experiment_parameters']={'timing_policy':args.timing_policy,'prediction':args.prediction,'frs_weight':args.frs_weight,
      'spatial_weight':args.spatial_weight,'camera_focal':args.camera_focal,'encounter_end_y':args.encounter_end_y,'measurement_sigma':args.measurement_sigma,'accel_sigma':args.accel_sigma,'encounter_trigger_x':args.encounter_trigger_x,
      'encounter_x':args.encounter_x,'encounter_speed':args.encounter_speed,'encounter_side':args.encounter_side}
    if args.case=='multi_encounter' or indoor:summary['multi_specs']=multi_specs;summary['multi_onsets']=multi_onsets
    import geometric_obstacles.core as tracking_core
    summary['tracker_sha256']=hashlib.sha256(Path(tracking_core.__file__).read_bytes()).hexdigest()
    import geometric_obstacles.node as perception_node
    import geometric_obstacles.online_background as online_background
    import geometric_obstacles.static_prior as static_prior
    summary['online_background_sha256']=hashlib.sha256(Path(online_background.__file__).read_bytes()).hexdigest()
    summary['static_prior_sha256']=hashlib.sha256(Path(static_prior.__file__).read_bytes()).hexdigest()
    summary['perception_node_sha256']=hashlib.sha256(Path(perception_node.__file__).read_bytes()).hexdigest()
    summary['controller_sha256']=hashlib.sha256((Path(get_package_prefix('scan_planner'))/'lib/scan_planner/closed_loop_controller').read_bytes()).hexdigest()
    summary['planner_sha256']=hashlib.sha256((Path(get_package_prefix('scan_planner'))/'lib/scan_planner/scan_planner_node').read_bytes()).hexdigest()
    summary['simulator_sha256']=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    route=[np.array([4.,0.]),goal_xy] if indoor else [goal_xy]
    route_index=0
    summary['route_waypoints']=[p.tolist() for p in route]
    summary['waypoint_events']=[]
    goal_sent=False;next_frame=0.;next_snapshot=0.;last=time.monotonic();minimum=float('inf')
    try:
        start=time.monotonic()
        while elapsed()<args.duration:
            now=time.monotonic();dt=min(now-last,.05);last=now;t=elapsed()
            if any(p.poll() is not None for p in processes):raise RuntimeError('pipeline process exited; inspect logs')
            speed,yawrate=command if now-last_command<.3 else (0.,0.)
            state[0]+=speed*dt*math.cos(state[2]+yawrate*dt/2)
            state[1]+=speed*dt*math.sin(state[2]+yawrate*dt/2);state[2]+=yawrate*dt
            if args.case=='encounter' and encounter_started is None and state[0]>=args.encounter_trigger_x:
                encounter_started=t
                summary['encounter_onset']={'t':t,'pose':state.tolist(),'v':float(speed),'w':float(yawrate)}
            if args.case=='multi_encounter' or indoor:
                for spec in multi_specs:
                    ready=False if indoor else ((state[0]>=spec['trigger_x']) if 'trigger_x' in spec else ('truth_0' in multi_onsets and t>=multi_onsets['truth_0']['t']+spec['delay_after_first']))
                    if indoor:ready=state[spec['trigger_axis']]>=spec['trigger_value']
                    if spec['id'] not in multi_onsets and ready:
                        multi_onsets[spec['id']]={'t':t,'pose':state.tolist(),'v':float(speed),'w':float(yawrate)}
                        if encounter_started is None:
                            encounter_started=t;summary['encounter_onset']=multi_onsets[spec['id']]
            base_z+=float(np.clip(data["height_target"]-base_z,-.08*dt,.08*dt))
            stamp=node.get_clock().now().to_msg()
            odom=Odometry();odom.header.stamp=stamp;odom.header.frame_id='world';odom.child_frame_id='base_link'
            odom.pose.pose.position.x=float(state[0]);odom.pose.pose.position.y=float(state[1]);odom.pose.pose.position.z=base_z
            odom.pose.pose.orientation.z=math.sin(state[2]/2);odom.pose.pose.orientation.w=math.cos(state[2]/2)
            # Match the existing Slash simulator / planner contract: world-frame linear velocity.
            odom.twist.twist.linear.x=float(speed*math.cos(state[2]))
            odom.twist.twist.linear.y=float(speed*math.sin(state[2]))
            odom.twist.twist.angular.z=float(yawrate)
            pubs['odom'].publish(odom)
            if t>=next_frame:
                next_frame=t+.075
                depth,R,origin=raycast(t)
                tfmsg=TransformStamped();tfmsg.header.stamp=stamp;tfmsg.header.frame_id='world';tfmsg.child_frame_id='sim_optical'
                tfmsg.transform.translation.x=float(origin[0]);tfmsg.transform.translation.y=float(origin[1]);tfmsg.transform.translation.z=float(origin[2])
                quat=Rotation.from_matrix(R).as_quat()
                tfmsg.transform.rotation.x,tfmsg.transform.rotation.y,tfmsg.transform.rotation.z,tfmsg.transform.rotation.w=map(float,quat)
                tf.sendTransform(tfmsg)
                dropped=args.case=='dropout' and 7.<t<8.5
                if not dropped:
                    info=CameraInfo();info.header.frame_id='sim_optical';info.header.stamp=stamp;info.width=width;info.height=height
                    info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];pubs['info'].publish(info)
                    img=bridge.cv2_to_imgmsg(depth,'16UC1');img.header=info.header;pubs['depth'].publish(img)
                histories['sensors'].append({'t':t,'published':not dropped})
                if t>=next_snapshot:
                    next_snapshot=t+.4
                    snapshots.append({'t':t,'pose':state.copy(),'base_z':base_z,'depth':depth,'map':data['map'][::3].copy(),
                                      'filtered':data['filtered'],'decision':data['decision'],'tracks':data['tracks']})
            if t>3 and not goal_sent and data['tracks'] is not None and data['tracks']['healthy'] and len(data['map'])>20:
                goal=PoseStamped();goal.header.frame_id='world';goal.header.stamp=stamp;goal.pose.position.x=float(route[route_index][0]);goal.pose.position.y=float(route[route_index][1]);goal.pose.position.z=.2822;goal.pose.orientation.w=1.
                pubs['goal'].publish(goal);goal_sent=True;summary.setdefault('goal_sent_at',t)
            clearance=[];beam_margin=None
            for center,size in moving(t):clearance.append(float(np.linalg.norm(np.maximum(np.abs(state[:2]-center[:2])-size[:2],0))-.43))
            for lo,hi in static:
                center=(np.array(lo[:2])+hi[:2])/2;half=(np.array(hi[:2])-lo[:2])/2
                horizontal=float(np.linalg.norm(np.maximum(np.abs(state[:2]-center)-half,0))-.43)
                if lo[2]>0:
                    if horizontal<=0:beam_margin=lo[2]-base_z-.328
                    if base_z+.328>lo[2]:clearance.append(horizontal)
                else:clearance.append(horizontal)
            gap=min(clearance);minimum=min(minimum,gap)
            histories['poses'].append({'t':t,'pose':state.tolist(),'base_z':base_z,'v':float(speed),'w':float(yawrate),'clearance':gap,'status':data['status'],'height_target':data['height_target'],'beam_margin':beam_margin,
                'truth':[{'id':multi_specs[i]['id'] if args.case=='multi_encounter' or indoor else 'truth_0','p':c[:2].tolist(),'half':s[:2].tolist()} for i,(c,s) in enumerate(moving(t))]})
            if gap<0:summary['collision']=True
            if route_index<len(route)-1 and goal_sent and np.linalg.norm(state[:2]-route[route_index])<.16 and abs(speed)<.01 and data['status']=='GOAL_REACHED':
                summary['waypoint_events'].append({'t':t,'index':route_index,'pose':state.tolist()})
                route_index+=1;goal_sent=False
            if route_index==len(route)-1 and np.linalg.norm(state[:2]-goal_xy)<.16:
                summary.setdefault('goal_region_time',t)
                if abs(speed)<.01 and data['status']=='GOAL_REACHED':
                    summary['reached']=True;summary['arrival_time']=t;break
            rclpy.spin_once(node,timeout_sec=.005)
        summary.update(duration=elapsed(),final_pose=state.tolist(),min_clearance=minimum,plans=data['plans'],
            map_messages=len(histories['maps']),track_messages=len(histories['tracks']),decision_messages=len(histories['decisions']),
            dynamic_ids=sorted({o['id'] for r in histories['tracks'] for o in r['objects'] if o['state']==1}),
            unhealthy_frames=sum(not r['healthy'] for r in histories['tracks']),
            frs_waits=sum(r['scale']==0 and not r['stale'] for r in histories['decisions']),
            frs_slow=sum(0<r['scale']<1 for r in histories['decisions']))
        if indoor:
            rows=histories['poses'];margins=[r['beam_margin'] for r in rows if r['beam_margin'] is not None]
            summary['height_validation']={'min_leg_height':min(r['base_z']-.0322 for r in rows),
                'final_leg_height':base_z-.0322,'min_beam_margin':min(margins) if margins else None,
                'beam_crossed':bool(margins and state[1]>3.23),
                'yaw_range':max(r['pose'][2] for r in rows)-min(r['pose'][2] for r in rows)}
            summary['combined_pass']=bool(summary['reached'] and not summary['collision'] and margins and min(margins)>=.03-1e-5
                and summary['height_validation']['min_leg_height']<.215 and abs(base_z-.2822)<.025
                and summary['height_validation']['yaw_range']>1.0 and len(multi_onsets)==3)
    except Exception as exc:summary['error']=str(exc)
    finally:
        for proc in processes:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in processes:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
        for f in logs:f.close()
        (args.output/'trace.json').write_text(json.dumps(histories,ensure_ascii=False))
        np.savez_compressed(args.output/'snapshots.npz',frames=np.array(snapshots,dtype=object))
        (args.output/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False))
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(summary,indent=2,ensure_ascii=False))
    return 1 if summary['error'] else 0

if __name__=='__main__':raise SystemExit(main())
