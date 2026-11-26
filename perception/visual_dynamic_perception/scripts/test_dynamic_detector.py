#!/usr/bin/python3
"""Isolated ROS-domain regression: stationary depth scene, then an approaching sphere.

This is synthetic data, not a claim about a real person/camera detection trial.
"""
import json
import copy
import argparse
import os
import signal
import subprocess
import time
from pathlib import Path

import numpy as np
import yaml

os.environ['ROS_DOMAIN_ID'] = '37'
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--motion', choices=('approach', 'lateral'), default='approach')
    parser.add_argument('--pose-loss', action='store_true')
    parser.add_argument('--semantic',choices=('off','matched','unmatched','stale','unknown-strong'),default='off')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = root/('runtime/synthetic_test' if args.motion=='approach' else 'runtime/synthetic_lateral_test')
    if args.pose_loss:
        output = root/'runtime/synthetic_pose_loss_test'
    if args.semantic!='off':output=root/f'runtime/synthetic_semantic_{args.semantic}'
    output.mkdir(parents=True, exist_ok=True)
    cfg = yaml.safe_load((root/'tests/fixtures/gemini336l/detector.yaml').read_text())
    params = cfg['/dynamic_detector']['ros__parameters']['onboard_detector']
    params['semantic_assist']=args.semantic!='off'
    params.update(depth_image_topic='/test/depth', aligned_depth_image_topic='/test/depth', odom_topic='/test/odom')
    # Test geometry is independent of whether the live camera uses 400-row IR
    # depth or 480-row registered color depth.
    params.update(image_cols=640, image_rows=400,
                  depth_intrinsics=[311., 311., 320., 200.],
                  body_to_camera=np.array([[0.,0.,1.,0.],[-1.,0.,0.,0.],
                                           [0.,-1.,0.,0.],[0.,0.,0.,1.]]).ravel().tolist())
    params['color_intrinsics']=list(params['depth_intrinsics'])
    params['body_to_camera_color']=list(params['body_to_camera'])
    (output/'params.yaml').write_text(yaml.safe_dump(cfg))
    process = subprocess.Popen([str(root.parents[1]/'install/ws_perception/onboard_detector/lib/onboard_detector/detector_node'),
        '--ros-args', '--params-file', str(output/'params.yaml')], stdout=(output/'detector.log').open('w'), stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('synthetic_dodt_test')
    from vision_msgs.msg import Detection2DArray,Detection2D,ObjectHypothesisWithPose
    semantics=node.create_publisher(Detection2DArray,'/detection/semantic_boxes',10)
    images = node.create_publisher(Image, '/test/depth', 10)
    odometry = node.create_publisher(Odometry, '/test/odom', 10)
    counts = {'static_frames_with_dynamic_boxes': 0, 'moving_frames_with_dynamic_boxes': 0,
              'moving_dynamic_point_messages': 0, 'filtered_cloud_messages': 0,
              'cleared_frames_with_dynamic_boxes': 0}
    counts['pose_loss_frames_with_dynamic_boxes'] = 0
    phase = ['warmup']
    counts['tracked_peak_boxes'] = 0
    def boxes(msg):
        if msg.markers and phase[0] in ('static', 'moving'):
            counts[phase[0]+'_frames_with_dynamic_boxes'] += 1
        if msg.markers and phase[0] == 'cleared':
            counts['cleared_frames_with_dynamic_boxes'] += 1
        if msg.markers and phase[0] == 'pose_loss':
            counts['pose_loss_frames_with_dynamic_boxes'] += 1
    def points(msg):
        if msg.width and phase[0] == 'moving': counts['moving_dynamic_point_messages'] += 1
    subscriptions = [node.create_subscription(MarkerArray, '/onboard_detector/dynamic_bboxes', boxes, 10),
        node.create_subscription(MarkerArray, '/onboard_detector/tracked_bboxes',
            lambda msg: counts.__setitem__('tracked_peak_boxes', max(counts['tracked_peak_boxes'], len(msg.markers))), 10),
        node.create_subscription(PointCloud2, '/onboard_detector/dynamic_point_cloud', points, 10),
        node.create_subscription(PointCloud2, '/onboard_detector/filtered_depth_cloud',
            lambda msg: counts.__setitem__('filtered_cloud_messages', counts['filtered_cloud_messages']+1), 10)]
    try:
        ready=time.monotonic()+3.
        while (images.get_subscription_count()==0 or odometry.get_subscription_count()==0) and time.monotonic()<ready:
            rclpy.spin_once(node,timeout_sec=.05)
        assert images.get_subscription_count()>0 and odometry.get_subscription_count()>0, 'detector inputs not discovered'
        for frame in range(175):
            if process.poll() is not None: raise RuntimeError('detector exited: inspect synthetic_test/detector.log')
            phase[0] = 'static' if frame < 45 else ('moving' if frame < 150 else ('settling' if frame < 160 else 'cleared'))
            if args.pose_loss and frame >= 160:
                phase[0] = 'pose_loss'
            depth = np.full((400, 640), 4500, dtype=np.uint16)
            yy, xx = np.mgrid[:400, :640]
            fx, fy, cx, cy = params['depth_intrinsics']
            rx = (xx-cx)/fx
            ry = (yy-cy)/fy
            step=.025 if args.semantic=='unknown-strong' else (.02 if args.semantic=='off' else .015)
            center = 3.0 if frame < 45 else 3.0-(min(frame,149)-45)*step
            center_x = 0.
            if args.motion == 'lateral':
                center=2.
                center_x=-.6 if frame<45 else -.6+(frame-45)*.02
            a = rx*rx+ry*ry+1
            ray_dot_center=rx*center_x+center
            discriminant = 4*ray_dot_center*ray_dot_center-4*a*(center*center+center_x*center_x-.25*.25)
            hit = (discriminant > 0) & (frame < 150 or args.pose_loss)
            near = (2*ray_dot_center-np.sqrt(np.maximum(discriminant, 0)))/(2*a)
            depth[hit] = (near[hit]*1000).astype(np.uint16)
            stamp = node.get_clock().now().to_msg()
            image = Image()
            image.header.stamp, image.header.frame_id = stamp, 'camera_depth_optical_frame'
            image.width, image.height, image.encoding, image.step = 640, 400, '16UC1', 1280
            image.data = depth.tobytes()
            odom = Odometry()
            odom.header.stamp, odom.header.frame_id = stamp, 'map'
            odom.pose.pose.orientation.w = 1.
            if frame < 150 or not args.pose_loss:
                odometry.publish(odom)
            if args.semantic!='off':
                semantic=Detection2DArray();semantic.header=copy.deepcopy(image.header)
                if args.semantic=='stale':semantic.header.stamp.sec-=1
                if args.semantic in ('matched','stale'):
                    box=Detection2D();box.bbox.center.position.x=320.;box.bbox.center.position.y=200.;box.bbox.size_x=640.;box.bbox.size_y=400.
                    result=ObjectHypothesisWithPose();result.hypothesis.class_id='person';result.hypothesis.score=.9
                    box.results=[result];semantic.detections=[box]
                semantics.publish(semantic)
            images.publish(image)
            end = time.monotonic()+1/15
            while time.monotonic() < end: rclpy.spin_once(node, timeout_sec=.005)
        counts['node_alive'] = process.poll() is None
        (output/'result.json').write_text(json.dumps(counts, indent=2))
        print(json.dumps(counts, indent=2))
        assert counts['filtered_cloud_messages'] > 20
        assert counts['static_frames_with_dynamic_boxes'] == 0, 'stationary synthetic scene classified as moving'
        if args.semantic=='unmatched':
            assert counts['moving_frames_with_dynamic_boxes']==0, 'weak unsupported motion passed semantic evidence gate'
        else:
            assert counts['moving_frames_with_dynamic_boxes'] > 0, 'moving synthetic obstacle not classified'
        assert counts['cleared_frames_with_dynamic_boxes'] == 0, 'removed obstacle still marked dynamic'
        assert counts['pose_loss_frames_with_dynamic_boxes'] == 0, 'motion classified without fresh camera pose'
    finally:
        process.send_signal(signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.terminate()
            try:process.wait(timeout=3)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
