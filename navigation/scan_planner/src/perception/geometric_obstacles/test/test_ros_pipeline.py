"""Synthetic ROS transport test; does not launch a controller or publish motor commands.
Run with an isolated ROS_DOMAIN_ID after sourcing the built workspace.
"""
import time
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
from sensor_msgs.msg import Image, CameraInfo
from scan_planner_msgs.msg import DynamicObstacleArray
from geometric_obstacles.node import GeometricObstacles


def test_depth_tf_tracking_mask_and_dropout():
    rclpy.init()
    perception = GeometricObstacles(); driver = Node('synthetic_depth_source')
    executor = SingleThreadedExecutor(); executor.add_node(perception); executor.add_node(driver)
    arrays=[]; filtered=[]
    driver.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',arrays.append,10)
    driver.create_subscription(Image,'/dynamic_obstacles/depth_static',filtered.append,10)
    pub=driver.create_publisher(Image,'/camera/camera/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
    camera=driver.create_publisher(CameraInfo,'/camera/camera/color/camera_info',qos_profile_sensor_data)
    tf=StaticTransformBroadcaster(driver)
    transform=TransformStamped(); transform.header.frame_id='world'; transform.child_frame_id='test_optical'
    # optical x right/y down/z forward -> world x forward/y left/z up
    transform.transform.rotation.x=-.5; transform.transform.rotation.y=.5
    transform.transform.rotation.z=-.5; transform.transform.rotation.w=.5
    transform.transform.translation.z=.6; tf.sendTransform(transform)
    info=CameraInfo();info.header.frame_id='test_optical';info.width=160;info.height=120
    info.k=[100.,0.,80.,0.,100.,60.,0.,0.,1.]
    info.d=[.02,-.01,0.,0.,0.];info.distortion_model='plumb_bob'
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end: executor.spin_once(timeout_sec=.005)
    def send(offset):
        depth=np.full((120,160),3000,np.uint16)
        if offset is not None: depth[40:98,offset:offset+24]=1600
        msg=perception.bridge.cv2_to_imgmsg(depth,'16UC1')
        msg.header.frame_id='test_optical';msg.header.stamp=driver.get_clock().now().to_msg()
        info.header.stamp=msg.header.stamp;camera.publish(info);pub.publish(msg)
        spin(.065)
    try:
        spin(.25)
        info.width=159;send(20)
        assert not arrays[-1].healthy and any('mismatch' in a.status for a in arrays)
        info.width=160
        send(20)
        assert any(o.state==o.TENTATIVE for o in arrays[-1].obstacles)
        assert np.any(perception.bridge.imgmsg_to_cv2(filtered[-1])==65535), "tentative surface must not enter static occupancy"
        assert perception.bridge.imgmsg_to_cv2(filtered[-1])[94,30]==65535, "object contact band must not leave a low static wall"
        for i in range(40): send(20+i*2)
        healthy=[a for a in arrays if a.healthy]
        assert healthy, [a.status for a in arrays[-5:]]
        dynamic=[o for a in healthy for o in a.obstacles if o.state==o.DYNAMIC]
        assert dynamic, [(o.id,o.state,o.velocity.y) for a in healthy[-3:] for o in a.obstacles]
        assert len(set(o.id for o in dynamic))==1
        assert any(np.any(perception.bridge.imgmsg_to_cv2(f)==65535) for f in filtered)
        # Actual visible background lets the missing moving object retire.
        for _ in range(8): send(None)
        assert arrays[-1].healthy and not arrays[-1].obstacles, arrays[-1]
        spin(.65)
        assert not arrays[-1].healthy and 'fresh' in arrays[-1].status
        send(20)
        assert perception.fault and not arrays[-1].healthy
        print('ROS pipeline verified: stable moving ID, sentinel mask, healthy empty, dropout unhealthy')
    finally:
        executor.shutdown(); perception.destroy_node(); driver.destroy_node(); rclpy.shutdown()


def test_prebuilt_background_and_new_obstacle_transport():
    from rclpy.qos import QoSProfile,DurabilityPolicy
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs_py import point_cloud2
    from std_msgs.msg import Header
    rclpy.init(args=['--ros-args','-p','static_prior_enabled:=true','-p','static_prior_topic:=/test/prebuilt'])
    perception=GeometricObstacles();driver=Node('prior_test_source')
    executor=SingleThreadedExecutor();executor.add_node(perception);executor.add_node(driver)
    arrays=[];priors=[]
    driver.create_subscription(DynamicObstacleArray,'/dynamic_obstacles/tracks',arrays.append,10)
    driver.create_subscription(PointCloud2,'/dynamic_obstacles/static_prior',priors.append,10)
    pub=driver.create_publisher(Image,'/camera/camera/aligned_depth_to_color/image_raw',qos_profile_sensor_data)
    camera=driver.create_publisher(CameraInfo,'/camera/camera/color/camera_info',qos_profile_sensor_data)
    maps=driver.create_publisher(PointCloud2,'/test/prebuilt',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    tf=StaticTransformBroadcaster(driver)
    tr=TransformStamped();tr.header.frame_id='world';tr.child_frame_id='prior_optical'
    tr.transform.rotation.x=-.5;tr.transform.rotation.y=.5;tr.transform.rotation.z=-.5;tr.transform.rotation.w=.5
    tr.transform.translation.z=.6;tf.sendTransform(tr)
    info=CameraInfo();info.header.frame_id='prior_optical';info.width=160;info.height=120
    info.k=[100.,0.,80.,0.,100.,60.,0.,0.,1.]
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    def send(new=False):
        depth=np.full((120,160),1600,np.uint16)
        if new:depth[40:80,40:64]=1200
        msg=perception.bridge.cv2_to_imgmsg(depth,'16UC1');msg.header.frame_id='prior_optical'
        msg.header.stamp=driver.get_clock().now().to_msg();info.header.stamp=msg.header.stamp
        camera.publish(info);pub.publish(msg);spin(.07)
    try:
        spin(.2);send()
        assert any(not a.healthy and 'static map' in a.status for a in arrays)
        y,z=np.meshgrid(np.arange(-1.5,1.51,.04),np.arange(-.5,1.71,.04))
        cloud=np.column_stack((np.full(y.size,1.6),y.ravel(),z.ravel())).astype(np.float32)
        header=Header();header.frame_id='world';header.stamp=driver.get_clock().now().to_msg()
        maps.publish(point_cloud2.create_cloud_xyz32(header,cloud));spin(.08)
        for _ in range(6):send()
        assert arrays[-1].healthy and not arrays[-1].obstacles
        assert priors and priors[-1].width>0
        assert perception.prior_stats['known_static_points']/perception.prior_stats['input_points']>.95
        send(True)
        assert arrays[-1].healthy and arrays[-1].obstacles
        assert any(abs(o.position.x-1.2)<.1 for o in arrays[-1].obstacles)
        print('Prebuilt prior verified: missing-map hold, known wall excluded, new obstacle retained')
    finally:
        executor.shutdown();perception.destroy_node();driver.destroy_node();rclpy.shutdown()
