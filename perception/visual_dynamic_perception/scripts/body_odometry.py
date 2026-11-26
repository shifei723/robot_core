#!/usr/bin/python3
"""Convert optimized VINS IMU pose into the camera_link pose, at the same timestamp."""
import argparse
import json
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation


def camera_pose(world_imu, body_imu):
    return world_imu @ np.linalg.inv(body_imu)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--input-topic', default='/vins/odometry')
    parser.add_argument('--output-topic', default='/detection/odometry')
    parser.add_argument('--input-body-frame', default=None)
    parser.add_argument('--world-frame', default='map')
    args = parser.parse_args()
    calibration = json.load(open(args.calibration))
    body_imu = np.asarray(calibration['body_T_imu'], dtype=float).reshape(4, 4)
    rclpy.init()
    node = rclpy.create_node('vins_camera_body_odometry')
    publisher = node.create_publisher(Odometry, args.output_topic, 10)
    expected_frame = args.input_body_frame or calibration['imu_frame']

    def callback(msg):
        if msg.child_frame_id != expected_frame:
            node.get_logger().error('Unexpected VINS body frame; refusing incorrect camera pose')
            return
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        quaternion = np.array([q.x, q.y, q.z, q.w])
        if not np.isfinite(quaternion).all() or np.linalg.norm(quaternion) < .5:
            return
        world_imu = np.eye(4)
        world_imu[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
        world_imu[:3, 3] = [p.x, p.y, p.z]
        if not np.isfinite(world_imu).all() or np.linalg.norm(world_imu[:3, 3]) > 1000:
            return
        pose = camera_pose(world_imu, body_imu)
        output = Odometry()
        output.header = msg.header
        output.header.frame_id = args.world_frame
        output.child_frame_id = 'camera_link'
        output.pose.pose.position.x, output.pose.pose.position.y, output.pose.pose.position.z = pose[:3, 3].tolist()
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        output.pose.pose.orientation.x, output.pose.pose.orientation.y, output.pose.pose.orientation.z, output.pose.pose.orientation.w = q.tolist()
        # No fabricated covariance or untransformed IMU-frame twist.
        publisher.publish(output)

    subscription = node.create_subscription(Odometry, args.input_topic, callback,
        QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
    node.get_logger().info('Converting VINS IMU pose to camera_link using factory extrinsics')
    rclpy.spin(node)


if __name__ == '__main__':
    main()
