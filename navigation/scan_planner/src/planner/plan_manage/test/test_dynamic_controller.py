"""Exercise the real controller binary, remapping actuator outputs to test topics."""
import pytest
import copy
import json
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import os
import subprocess
import time
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Point
from scan_planner_msgs.msg import Bspline, DynamicObstacleArray, DynamicObstacle


@pytest.mark.parametrize("prior_required",[False,True])
def test_array_input_gates_controller_without_height_safety(prior_required):
    rclpy.init(); node=Node('dynamic_controller_test'); velocities=[];decisions=[]
    node.create_subscription(String,'/dynamic_navigation/decision',lambda m:decisions.append(json.loads(m.data)),10)
    exe=os.path.join(get_package_prefix('scan_planner'),'lib','scan_planner','closed_loop_controller')
    process=subprocess.Popen([exe,'--ros-args','-p','dynamic_input:=array','-p','dynamic_prediction:=frs',
        '-p',f'static_prior_required:={str(prior_required).lower()}','-p','height_safety_enabled:=false','-r','cmd_vel:=/test_frs/cmd_vel',
        '-r','leg_height_cmd:=/test_frs/leg_height','-r','planning/go2_execution_frozen:=/test_frs/frozen'])
    node.create_subscription(Twist,'/test_frs/cmd_vel',lambda m:velocities.append(m.linear.x),10)
    odom_pub=node.create_publisher(Odometry,'/body_pose',10)
    spline_pub=node.create_publisher(Bspline,'/planning/bspline',10)
    prior_pub=node.create_publisher(PointCloud2,'/grid_map/static_prior_applied',1)
    map_pub=node.create_publisher(PointCloud2,'/grid_map/occupancy',10)
    tracks_pub=node.create_publisher(DynamicObstacleArray,'/dynamic_obstacles/tracks',10)
    odom=Odometry(); odom.header.frame_id='world';odom.pose.pose.orientation.w=1.
    odom.pose.pose.position.x=.1;odom.pose.pose.position.z=.2822
    spline=Bspline();spline.order=3;spline.traj_id=1
    spline.pos_pts=[Point(x=i*.1,y=0.,z=.2822) for i in range(8)]
    spline.knots=[float(i) for i in range(-3,9)]
    def pump(seconds,health=None,obstacle=False,prior=True,coasting=False,far=False,unknown=False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            now=node.get_clock().now().to_msg();odom.header.stamp=now;odom_pub.publish(odom)
            occupancy=PointCloud2();occupancy.header.frame_id='world';occupancy.header.stamp=now
            map_pub.publish(occupancy)
            if prior_required and prior:
                ack=PointCloud2();ack.header.frame_id='world';ack.header.stamp=now;prior_pub.publish(ack)
            if health is not None:
                a=DynamicObstacleArray();a.header.stamp=now;a.header.frame_id='world';a.epoch=1;a.healthy=health
                if obstacle:
                    o=DynamicObstacle();o.id='obstacle_far' if far else 'obstacle_1';o.observed=not(coasting or unknown);o.state=o.UNKNOWN if unknown else (o.COASTING if coasting else o.DYNAMIC);o.last_observed=copy.deepcopy(now)
                    if coasting or unknown:o.last_observed.sec-=2
                    o.position.x=3. if far else .3;o.radius=.2;a.obstacles=[o]
                tracks_pub.publish(a)
            rclpy.spin_once(node,timeout_sec=.02)
    try:
        pump(.5);spline_pub.publish(spline);pump(.25)
        assert velocities and velocities[-1]==0., 'Missing perception must hold'
        if prior_required:
            velocities.clear();pump(.9,True,prior=False)
            assert velocities[-1]==0., 'Healthy perception cannot bypass missing/stale planner prior acknowledgement'
        velocities.clear();pump(.3,True)
        assert any(v>.01 for v in velocities), 'Healthy empty must permit progress'
        retained=velocities[-1];velocities.clear();spline.traj_id=2;spline_pub.publish(spline);pump(.08,True)
        assert velocities and min(velocities)>=retained-.05, 'FRS reference swap must retain speed'
        pump(.2,False);assert velocities[-1]==0., 'Unhealthy empty must hold'
        pump(.2,True,True);assert velocities[-1]==0., 'Nearby obstacle must hold'
        velocities.clear();pump(.25,True)
        assert any(v>.01 for v in velocities), 'Retired target must not block forever'
        pump(.7);assert velocities[-1]==0., 'Stale perception must hold'
        pump(.2,True,True,far=True);velocities.clear();pump(.3,True,True,coasting=True,far=True)
        assert any(v>.01 for v in velocities), 'A remote known coasting target must not globally freeze motion'
        pump(.2,True,True);pump(.2,True,True,coasting=True)
        assert velocities[-1]==0., 'A nearby coasting target remains a hard threat'
        pump(.1,True);velocities.clear();pump(.25,True,True,coasting=True,far=True)
        assert any(v>.01 for v in velocities), 'A newly adopted remote predicted ID must not freeze the frame'
        pump(.1,True);pump(.2,True,True,coasting=True)
        assert velocities[-1]==0., 'A newly adopted nearby predicted ID remains a hard threat'
        pump(.3,True,True,unknown=True)
        assert velocities[-1]==0., 'Unknown region must retain local collision protection'
        assert decisions[-1]['unknown_region_count']==1 and not decisions[-1]['tracks'], 'Unknown area must not publish motion FRS'
        velocities.clear();pump(.3,True,True,unknown=True,far=True)
        assert any(v>.01 for v in velocities), 'A remote unknown region must not freeze all motion'
        pump(.1,True);velocities.clear();pump(.3,True)
        assert any(v>.01 for v in velocities), 'A retired unknown region must release the controller'
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()


@pytest.mark.parametrize("nonplanar",[False,True])
def test_height_unknown_reobservation_requires_fresh_inputs_and_safe_rotation(nonplanar):
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header, Float64MultiArray
    rclpy.init();node=Node('unknown_reobservation_test');commands=[];states=[];phases=[]
    exe=os.path.join(get_package_prefix('scan_planner'),'lib','scan_planner','closed_loop_controller')
    proc=subprocess.Popen([exe,'--ros-args','-p','dynamic_input:=array','-p','dynamic_prediction:=frs',
        '-p','height_safety_enabled:=true','-p','height_spatial_tracking:=true','-p','static_prior_required:=false',
        '-r','cmd_vel:=/test_unknown/cmd','-r','leg_height_cmd:=/test_unknown/height'])
    node.create_subscription(Twist,'/test_unknown/cmd',lambda m:commands.append((m.linear.x,m.angular.z)),10)
    node.create_subscription(String,'/height_control/state',lambda m:states.append(m.data),10)
    node.create_subscription(Float64MultiArray,'/planning/execution_phase',lambda m:phases.append(m.data),10)
    pubs={k:node.create_publisher(t,topic,10) for k,t,topic in [
        ('odom',Odometry,'/body_pose'),('path',Bspline,'/planning/bspline'),
        ('map',PointCloud2,'/grid_map/occupancy'),('cloud',PointCloud2,'/cloud'),
        ('tracks',DynamicObstacleArray,'/dynamic_obstacles/tracks')]}
    odom=Odometry();odom.header.frame_id='world';odom.pose.pose.orientation.w=1.
    odom.pose.pose.position.x=.1;odom.pose.pose.position.z=.2822
    spline=Bspline();spline.order=3;spline.traj_id=1
    spline.pos_pts=[Point(x=i*.2,y=0.,z=.1822 if nonplanar and i>=5 else .2822) for i in range(12)]
    spline.knots=[(i-3)*.4 for i in range(16)]
    def pump(seconds,blocked=False,fresh=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            stamp=node.get_clock().now().to_msg();odom.header.stamp=stamp;pubs['odom'].publish(odom)
            header=Header();header.frame_id='world';header.stamp=stamp
            cloud=point_cloud2.create_cloud_xyz32(header,[[.1,0.,.2]] if blocked else [])
            pubs['map'].publish(cloud);pubs['cloud'].publish(cloud)
            if fresh:
                a=DynamicObstacleArray();a.header=header;a.epoch=1;a.healthy=True
                o=DynamicObstacle();o.id='obstacle_0';o.state=o.UNKNOWN;o.observed=False
                o.last_observed=copy.deepcopy(stamp);o.last_observed.sec-=2
                o.position.x=.9;o.position.y=.5;o.radius=.1
                o.covariance=[.001 if i in (0,5) else 0. for i in range(16)]
                a.obstacles=[o];pubs['tracks'].publish(a)
            rclpy.spin_once(node,timeout_sec=.015)
    try:
        pump(1.);spline.start_time=node.get_clock().now().to_msg();pubs['path'].publish(spline)
        commands.clear();pump(.8)
        assert phases, 'Geometric tracking must remain active on height-changing paths'
        assert 'OBSERVING_UNKNOWN' in states
        assert commands and all(abs(v)<1e-6 for v,w in commands)
        assert any(w>.05 for v,w in commands)
        pump(.9,fresh=False);commands.clear();pump(.2,fresh=False)
        assert commands and all(abs(v)+abs(w)<1e-6 for v,w in commands)
        commands.clear();pump(.4,blocked=True)
        assert commands and all(abs(v)+abs(w)<1e-6 for v,w in commands)
    finally:
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()
