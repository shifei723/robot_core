#!/usr/bin/python3
"""Read factory CameraInfo/TF, generate stereo VINS and depth-detector parameters."""
import argparse
import json
import os
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformListener
from openvins_config import generate_openvins


def matrix(transform):
    q = transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    result = np.eye(4)
    result[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                      [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                      [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    result[:3, 3] = [transform.translation.x, transform.translation.y, transform.translation.z]
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--timeout', type=float, default=40.)
    parser.add_argument('--localization-mode', choices=('rgbd', 'vins', 'openvins', 'stationary'), default='vins')
    parser.add_argument('--recover-camera', action='store_true',
                        help='Soft reboot this ROS-domain camera once if no calibration arrives after 10 s')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    rclpy.init()
    node = rclpy.create_node('configure_gemini336l')
    infos = {}
    imu_frames = []
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    subscriptions = [node.create_subscription(CameraInfo, f'/camera/{name}/camera_info',
        lambda msg, name=name: infos.__setitem__(name, msg), qos_profile_sensor_data)
        for name in ('left_ir', 'right_ir', 'depth', 'color')]
    # The driver publishes some calibration alongside subscribed image streams.
    subscriptions += [node.create_subscription(Image, f'/camera/{name}/image_raw',
        lambda msg: None, qos_profile_sensor_data)
        for name in ('left_ir', 'right_ir', 'depth', 'color')]
    subscriptions.append(node.create_subscription(Imu, '/camera/gyro_accel/sample',
        lambda msg: imu_frames.append(msg.header.frame_id) if not imu_frames else None, qos_profile_sensor_data))
    imu_body = None
    end = time.monotonic()+args.timeout
    recovery_time = time.monotonic()+10.
    recovery_client = node.create_client(Empty, '/camera/reboot_device') if args.recover_camera else None
    recovery_requested = False
    transforms = {}
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        if (recovery_client is not None and not recovery_requested and len(infos) != 4 and
                time.monotonic() >= recovery_time and recovery_client.service_is_ready()):
            recovery_client.call_async(Empty.Request())
            recovery_requested = True
            print('Incomplete camera streams after 10 s; requesting one soft reboot in the detection ROS domain', flush=True)
        if len(infos) != 4:
            continue
        try:
            transforms = {name: matrix(buffer.lookup_transform('camera_link', info.header.frame_id,
                rclpy.time.Time()).transform) for name, info in infos.items()}
            if args.localization_mode in ('vins', 'openvins'):
                if not imu_frames:
                    continue
                imu_body = matrix(buffer.lookup_transform('camera_link', imu_frames[0],
                    rclpy.time.Time()).transform)
            break
        except Exception:
            continue
    if len(transforms) != 4 or (args.localization_mode in ('vins', 'openvins') and imu_body is None):
        raise RuntimeError('No complete 336L CameraInfo/TF calibration; check camera driver topics')
    args.output.mkdir(parents=True, exist_ok=True)
    for name, short in (('left_ir', 'left'), ('right_ir', 'right')):
        info = infos[name]
        d = list(info.d)+[0.]*4
        calib = {'model_type': 'PINHOLE', 'camera_name': short, 'image_width': info.width,
            'image_height': info.height,
            'distortion_parameters': dict(zip(('k1', 'k2', 'p1', 'p2'), map(float, d[:4]))),
            'projection_parameters': {'fx': float(info.k[0]), 'fy': float(info.k[4]),
                                      'cx': float(info.k[2]), 'cy': float(info.k[5])}}
        (args.output/f'{short}.yaml').write_text('%YAML:1.0\n'+yaml.safe_dump(calib, sort_keys=False))
    output_path = args.output/'vins_output'
    output_path.mkdir(exist_ok=True)
    fs = cv2.FileStorage(str(args.output/'vins.yaml'), cv2.FILE_STORAGE_WRITE)
    settings = {'imu': int(args.localization_mode in ('vins', 'openvins')), 'num_of_cam': 2, 'imu_topic': '/camera/gyro_accel/sample',
        'image0_topic': '/camera/left_ir/image_raw', 'image1_topic': '/camera/right_ir/image_raw',
        'output_path': str(output_path)+'/', 'cam0_calib': 'left.yaml', 'cam1_calib': 'right.yaml',
        'image_width': infos['left_ir'].width, 'image_height': infos['left_ir'].height,
        'use_gpu': 0, 'use_gpu_acc_flow': 0, 'use_gpu_ceres': 0, 'estimate_extrinsic': 0,
        'body_T_cam0': np.linalg.inv(imu_body)@transforms['left_ir'] if imu_body is not None else transforms['left_ir'],
        'body_T_cam1': np.linalg.inv(imu_body)@transforms['right_ir'] if imu_body is not None else transforms['right_ir'],
        'multiple_thread': 1, 'max_cnt': 150, 'min_dist': 20, 'freq': 30, 'F_threshold': 1.,
        'show_track': 1, 'flow_back': 1, 'max_solver_time': .03, 'max_num_iterations': 8,
        'keyframe_parallax': 5., 'acc_n': .1, 'gyr_n': .01, 'acc_w': .001, 'gyr_w': .0001,
        'g_norm': 9.81, 'estimate_td': 0, 'td': 0., 'world_frame_id': 'map',
        'body_frame_id': imu_frames[0] if imu_body is not None else 'camera_link', 'camera_frame_id': 'vins_camera_optical'}
    for key, value in settings.items():
        fs.write(key, value)
    fs.release()
    if imu_body is not None:
        generate_openvins(args.output/'openvins', root.parents[1]/'localization/visual_slam/open_vins/config/rs_d455', infos, transforms, imu_body)
    cfg = yaml.safe_load((root.parents[1]/'perception/onboard_detector/cfg/detector_param.yaml').read_text())
    params = cfg['/dynamic_detector']['ros__parameters']['onboard_detector']
    depth = infos['depth']
    depth_topic = '/detection/depth_rect' if args.localization_mode=='rgbd' else '/camera/depth/image_raw'
    params.update(localization_mode=1, depth_image_topic=depth_topic,
        aligned_depth_image_topic=depth_topic,
        odom_topic='/detection/odometry',
        depth_intrinsics=[float(depth.k[i]) for i in (0, 4, 2, 5)],
        color_intrinsics=[float(infos['color'].k[i]) for i in (0, 4, 2, 5)],
        depth_scale_factor=1000., image_cols=depth.width, image_rows=depth.height,
        body_to_camera=transforms['depth'].ravel().tolist(),
        body_to_camera_color=transforms['color'].ravel().tolist(),
        depth_skip_pixel=2, depth_min_value=.3, depth_max_value=5., raycast_max_length=5.,
        time_difference=1/15., ground_height=-5., dbscan_min_points_cluster=8,
        dbscan_search_range_epsilon=.2, frame_skip=3, history_size=60,
        dynamic_voting_threshold=.7, dynamic_consistency_threshold=2, constrain_size=False,
        require_uv_confirmation=False, ground_filter_enabled=False,
        geometry_motion_classifier=True, semantic_assist=os.environ.get('YOLO_ASSIST', '1')=='1')
    params['exact_depth_odom_sync'] = args.localization_mode=='rgbd'
    (args.output/'detector.yaml').write_text(yaml.safe_dump(cfg, sort_keys=False))
    report = {'camera_serial': os.environ.get('CAMERA_SERIAL', ''),
        'mode': {'stationary': 'fixed-camera mounting constraint (camera must remain stationary)',
                 'rgbd': 'measured registered color/depth RGB-D camera motion; invalid motion suspends detection',
                 'vins': 'stereo visual-inertial odometry; factory IMU-camera extrinsics from driver TF',
                 'openvins': 'OpenVINS stereo + IMU; factory extrinsics, conditional zero-velocity updates'}[args.localization_mode],
        'depth_size': [depth.width, depth.height], 'depth_intrinsics': params['depth_intrinsics'],
        'color_size': [infos['color'].width, infos['color'].height],
        'color_intrinsics': params['color_intrinsics'],
        'depth_distortion': list(depth.d), 'color_distortion': list(infos['color'].d),
        'body_frame': 'camera_link', 'stereo_baseline_m': float(np.linalg.norm(
            transforms['left_ir'][:3, 3]-transforms['right_ir'][:3, 3])),
        'transforms': {k: v.tolist() for k, v in transforms.items()},
        'imu_frame': imu_frames[0] if imu_body is not None else None,
        'body_T_imu': imu_body.tolist() if imu_body is not None else None}
    (args.output/'calibration.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
