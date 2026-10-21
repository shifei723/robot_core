"""Exercise the real controller binary, remapping actuator outputs to test topics."""
import pytest
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
    rclpy.init(); node=Node('dynamic_controller_test'); velocities=[]
    exe=os.path.join(get_package_prefix('scan_planner'),'lib','scan_planner','closed_loop_controller')
    process=subprocess.Popen([exe,'--ros-args','-p','dynamic_input:=array','-p','dynamic_prediction:=frs',
        '-p',f'static_prior_required:={str(prior_required).lower()}','-p','height_safety_enabled:=false','-r','cmd_vel:=/test_frs/cmd_vel',
        '-r','leg_height_cmd:=/test_frs/leg_height','-r','planning/go2_execution_frozen:=/test_frs/frozen'])
    node.create_subscription(Twist,'/test_frs/cmd_vel',lambda m:velocities.append(m.linear.x),10)
    odom_pub=node.create_publisher(Odometry,'/body_pose',10)
    spline_pub=node.create_publisher(Bspline,'/planning/bspline',10)
    prior_pub=node.create_publisher(PointCloud2,'/grid_map/static_prior_applied',1)
    tracks_pub=node.create_publisher(DynamicObstacleArray,'/dynamic_obstacles/tracks',10)
    odom=Odometry(); odom.header.frame_id='world';odom.pose.pose.orientation.w=1.
    odom.pose.pose.position.x=.1;odom.pose.pose.position.z=.2822
    spline=Bspline();spline.order=3;spline.traj_id=1
    spline.pos_pts=[Point(x=i*.1,y=0.,z=.2822) for i in range(8)]
    spline.knots=[float(i) for i in range(-3,9)]
    def pump(seconds,health=None,obstacle=False,prior=True):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            now=node.get_clock().now().to_msg();odom.header.stamp=now;odom_pub.publish(odom)
            if prior_required and prior:
                ack=PointCloud2();ack.header.frame_id='world';ack.header.stamp=now;prior_pub.publish(ack)
            if health is not None:
                a=DynamicObstacleArray();a.header.stamp=now;a.header.frame_id='world';a.epoch=1;a.healthy=health
                if obstacle:
                    o=DynamicObstacle();o.id='obstacle_1';o.observed=True;o.last_observed=now
                    o.position.x=.3;o.radius=.2;a.obstacles=[o]
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
        pump(.2,False);assert velocities[-1]==0., 'Unhealthy empty must hold'
        pump(.2,True,True);assert velocities[-1]==0., 'Nearby obstacle must hold'
        velocities.clear();pump(.25,True)
        assert any(v>.01 for v in velocities), 'Retired target must not block forever'
        pump(.7);assert velocities[-1]==0., 'Stale perception must hold'
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node();rclpy.shutdown()
