#!/usr/bin/python3
"""Isolated ROS-domain stereo+IMU integration check; not a hardware accuracy benchmark."""
import os
os.environ['ROS_DOMAIN_ID'] = '37'
import json
import subprocess
import time
from pathlib import Path
import cv2
import numpy as np
import rclpy
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from openvins_config import read_yaml


def main():
    root = Path(__file__).resolve().parents[1]
    output = root/'runtime/openvins_synthetic'
    output.mkdir(parents=True, exist_ok=True)
    config = root/'config/336l_openvins/estimator_config.yaml'
    cameras = read_yaml(config.parent/'kalibr_imucam_chain.yaml')
    fx = cameras['cam0']['intrinsics'][0]
    left_imu = np.array(cameras['cam0']['T_cam_imu'])
    right_imu = np.array(cameras['cam1']['T_cam_imu'])
    baseline = float(np.linalg.norm((left_imu@np.linalg.inv(right_imu))[:3,3]))
    rclpy.init()
    node = rclpy.create_node('openvins_synthetic_inputs')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    pubs = [node.create_publisher(Image, f'/camera/{s}/image_raw', qos) for s in ('left_ir','right_ir')]
    imu_pub = node.create_publisher(Imu, '/camera/gyro_accel/sample', 400)
    poses = []
    body_poses = []
    sub = node.create_subscription(Odometry, '/openvins/odomimu', lambda m: poses.append((
        m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
        [m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z])), 100)
    body_sub = node.create_subscription(Odometry, '/openvins/body_odometry', lambda m: body_poses.append(m), 100)
    log = open(output/'node.log','w')
    process = subprocess.Popen([str(root.parents[1]/'install/ws_perception/ov_msckf/lib/ov_msckf/run_subscribe_msckf'),str(config),
        '--ros-args','-r','__ns:=/openvins','-p','publish_global_to_imu_tf:=false',
        '-p','publish_calibration_tf:=false'],stdout=log,stderr=subprocess.STDOUT)
    adapter_log = open(output/'adapter.log','w')
    adapter = subprocess.Popen(['/usr/bin/python3',str(root/'scripts/body_odometry.py'),
        '--calibration',str(root/'tests/fixtures/gemini336l/calibration.json'),'--input-topic','/openvins/odomimu',
        '--input-body-frame','imu','--output-topic','/openvins/body_odometry'],stdout=adapter_log,stderr=subprocess.STDOUT)
    rng = np.random.default_rng(12)
    texture = cv2.GaussianBlur(rng.integers(0,256,(400,640),dtype=np.uint8),(3,3),0)
    for _ in range(250):
        x,y=rng.integers([10,10],[630,390]);cv2.circle(texture,(int(x),int(y)),3,int(rng.integers(30,230)),-1)
    try:
        time.sleep(1.)
        start = time.monotonic(); epoch = node.get_clock().now().nanoseconds*1e-9
        next_image = 0.; next_imu = 0.
        while time.monotonic()-start < 12.:
            t = time.monotonic()-start
            if process.poll() is not None:raise RuntimeError('OpenVINS exited; inspect node.log')
            moving = 4. < t < 8.
            displacement = 0. if t <= 4. else (.2*(1-np.cos(np.pi*(t-4)/4)) if moving else .4)
            acceleration = .2*(np.pi/4)**2*np.cos(np.pi*(t-4)/4) if moving else 0.
            stamp = rclpy.time.Time(seconds=epoch+t).to_msg()
            if t >= next_imu:
                msg=Imu();msg.header.stamp=stamp;msg.header.frame_id='camera_accel_gyro_optical_frame'
                msg.linear_acceleration.x=float(acceleration);msg.linear_acceleration.y=-9.81
                imu_pub.publish(msg);next_imu=t+.005
            if t >= next_image:
                for index,pub in enumerate(pubs):
                    image=cv2.warpAffine(texture,np.float32([[1,0,-fx*(displacement+index*baseline)/2.],[0,1,0]]),(640,400))
                    msg=Image();msg.header.stamp=stamp;msg.header.frame_id=f'camera_{("left","right")[index]}_ir_optical_frame'
                    msg.height=400;msg.width=640;msg.encoding='mono8';msg.step=640;msg.data=image.tobytes();pub.publish(msg)
                next_image=t+1/30.
            rclpy.spin_once(node,timeout_sec=.001)
        times=np.array([p[0]-epoch for p in poses]);xyz=np.array([p[1] for p in poses])
        initial=xyz[(times>2.5)&(times<4.)];final=xyz[times>10.]
        result={'source':'synthetic stationary -> 0.4m lateral translation -> stationary, domain37',
            'body_odometry_count':len(body_poses),'odometry_count':len(poses),'initial_stationary_count':len(initial),'final_stationary_count':len(final)}
        if len(initial) and len(final):
            distance=float(np.linalg.norm(np.median(final,axis=0)-np.median(initial,axis=0)))
            result.update(estimated_translation_m=distance,truth_translation_m=.4,final_span_m=np.ptp(final,axis=0).tolist(),finite=bool(np.isfinite(xyz).all()))
        else:distance=0.
        result['passed']=bool(len(body_poses)>100 and len(poses)>100 and len(initial)>5 and len(final)>5 and np.isfinite(xyz).all() and .15<distance<.7)
        (output/'result.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
        if not result['passed']:raise RuntimeError('Stereo/IMU integration failed')
    finally:
        process.terminate()
        try:process.wait(timeout=5)
        except subprocess.TimeoutExpired:process.kill();process.wait()
        adapter.terminate()
        try:adapter.wait(timeout=5)
        except subprocess.TimeoutExpired:adapter.kill();adapter.wait()
        adapter_log.close()
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
