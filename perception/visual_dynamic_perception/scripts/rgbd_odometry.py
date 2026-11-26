#!/usr/bin/python3
"""Measured camera motion for the 336L detector; no stationary fallback."""
import argparse
import json
import time
from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from rgbd_motion import RGBDMotion


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--calibration', type=Path, required=True)
    args = parser.parse_args()
    cfg = yaml.safe_load((args.calibration/'detector.yaml').read_text())
    intrinsics = cfg['/dynamic_detector']['ros__parameters']['onboard_detector']['depth_intrinsics']
    factory = json.loads((args.calibration/'calibration.json').read_text())
    color_k = cfg['/dynamic_detector']['ros__parameters']['onboard_detector']['color_intrinsics']
    if factory['color_size'] != factory['depth_size'] or not np.allclose(color_k, intrinsics, atol=.5, rtol=0):
        raise RuntimeError('Enable native depth_registration to align depth with color before RGB-D odometry')
    cv2.setNumThreads(2)
    estimator = RGBDMotion(intrinsics, factory['transforms']['depth'])
    width, height = factory['depth_size']
    color_maps = cv2.initUndistortRectifyMap(estimator.k, np.array(factory['color_distortion']),
        None, estimator.k, (width, height), cv2.CV_32FC1)
    depth_maps = cv2.initUndistortRectifyMap(estimator.k, np.array(factory['depth_distortion']),
        None, estimator.k, (width, height), cv2.CV_32FC1)
    rclpy.init()
    node = rclpy.create_node('gemini336l_rgbd_egomotion')
    bridge = CvBridge()
    publisher = node.create_publisher(Odometry, '/detection/odometry', 10)
    rectified_depth = node.create_publisher(Image, '/detection/depth_rect', 10)
    diagnostics = node.create_publisher(DiagnosticArray, '/detection/odometry_status', 10)
    state = {'last_stamp': -1., 'valid': None}

    def reset_origin(request, response):
        estimator.reset_local_origin()
        state.update(last_stamp=-1., valid=None)
        node.get_logger().warn('Explicit local origin reset: previous map coordinates are no longer continuous')
        response.success = True
        response.message = 'New local odometry session requested; awaiting a valid camera frame'
        return response

    reset_service = node.create_service(Trigger, '/detection/reset_odometry', reset_origin)

    def callback(color, depth):
        started = time.monotonic()
        stamp = depth.header.stamp.sec+depth.header.stamp.nanosec*1e-9
        # Prefer the newest pair over processing a backlog while the camera
        # moves. Delayed poses must not be applied to newer obstacle clouds.
        input_age = node.get_clock().now().nanoseconds*1e-9-stamp
        if input_age > .1:
            return
        if stamp-state['last_stamp'] < .065:
            return
        state['last_stamp'] = stamp
        gray = bridge.imgmsg_to_cv2(color, 'mono8')
        metric_depth = bridge.imgmsg_to_cv2(depth, 'passthrough')
        if depth.encoding != '16UC1' or gray.shape != metric_depth.shape:
            node.get_logger().error('RGB-D odometry needs co-registered color / 16UC1 depth')
            return
        gray = cv2.remap(gray, *color_maps, cv2.INTER_LINEAR)
        metric_depth = cv2.remap(metric_depth, *depth_maps, cv2.INTER_NEAREST)
        rect = bridge.cv2_to_imgmsg(metric_depth, '16UC1')
        rect.header = depth.header
        try:
            pose, quality = estimator.process(gray, metric_depth, stamp)
        except (cv2.error, ValueError, np.linalg.LinAlgError) as error:
            pose, quality = None, {'valid': False, 'reason': str(error)}
        quality['processing_ms'] = round((time.monotonic()-started)*1000, 2)
        quality['input_age_ms'] = round(input_age*1000, 2)
        valid = pose is not None
        status = DiagnosticStatus()
        status.name = '336L RGB-D camera motion'
        status.hardware_id = factory.get('camera_serial', '')
        status.level = DiagnosticStatus.OK if valid else DiagnosticStatus.ERROR
        status.message = 'Measured motion compensation' if valid else 'Motion unknown: dynamic classification suspended'
        status.values = [KeyValue(key=str(k), value=str(v)) for k, v in quality.items() if k != 'indices']
        diag = DiagnosticArray()
        diag.header.stamp = depth.header.stamp
        diag.status = [status]
        diagnostics.publish(diag)
        if valid != state['valid']:
            if valid:
                node.get_logger().info(status.message)
            else:
                node.get_logger().warn(status.message)
            state['valid'] = valid
        if not valid:
            return
        msg = Odometry()
        msg.header.stamp = depth.header.stamp
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'camera_link'
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = map(float, pose[:3, 3])
        q = Rotation.from_matrix(pose[:3, :3]).as_quat()
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = map(float, q)
        # Conservative engineering covariance, not a calibrated uncertainty model.
        for i in (0, 7, 14, 21, 28, 35):
            msg.pose.covariance[i] = .001
        publisher.publish(msg)
        rectified_depth.publish(rect)

    # Camera publishers offer RELIABLE. Best-effort readers were losing most
    # fragmented color/depth images and creating ~1 s gaps in otherwise valid VO.
    latest_only = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
    color_sub = message_filters.Subscriber(node, Image, '/camera/color/image_raw', qos_profile=latest_only)
    depth_sub = message_filters.Subscriber(node, Image, '/camera/depth/image_raw', qos_profile=latest_only)
    sync = message_filters.ApproximateTimeSynchronizer([color_sub, depth_sub], 3, .015)
    sync.registerCallback(callback)
    node.get_logger().info('RGB-D camera-motion compensation enabled; degraded estimates are not published')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
