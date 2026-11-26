"""Generate a Gemini336L OpenVINS stereo/IMU configuration from measured driver calibration."""
from pathlib import Path
import numpy as np
import yaml


class OpenCVDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def read_yaml(path):
    return yaml.safe_load('\n'.join(line for line in Path(path).read_text().splitlines() if not line.startswith('%YAML:')))


def write_yaml(path, data):
    Path(path).write_text('%YAML:1.0\n---\n' + yaml.dump(data, Dumper=OpenCVDumper, sort_keys=False, default_flow_style=None))


def generate_openvins(output, template, infos, transforms, body_imu):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    settings = read_yaml(Path(template)/'estimator_config.yaml')
    settings.update(max_cameras=2, use_stereo=True, calib_cam_extrinsics=False,
        calib_cam_intrinsics=False, calib_cam_timeoffset=False, calib_imu_intrinsics=False,
        calib_imu_g_sensitivity=False, try_zupt=True, zupt_chi2_multipler=0.,
        zupt_max_velocity=.08, zupt_noise_multiplier=10., zupt_max_disparity=.2,
        zupt_only_at_beginning=False, init_window_time=2., init_imu_thresh=1.,
        init_max_disparity=1., init_dyn_use=False, track_frequency=30.,
        num_opencv_threads=2, num_pts=180, max_slam=30,
        relative_config_imu='kalibr_imu_chain.yaml', relative_config_imucam='kalibr_imucam_chain.yaml')
    cameras = {}
    for index, name in enumerate(('left_ir', 'right_ir')):
        info = infos[name]
        # Kalibr/OpenVINS needs camera_T_imu; VINS-Fusion needs the inverse.
        camera_imu = np.linalg.inv(transforms[name]) @ body_imu
        cameras[f'cam{index}'] = dict(T_cam_imu=camera_imu.tolist(), cam_overlaps=[1-index],
            camera_model='pinhole', distortion_model='radtan',
            distortion_coeffs=[float(x) for x in (list(info.d)+[0.]*4)[:4]],
            intrinsics=[float(info.k[i]) for i in (0, 4, 2, 5)],
            resolution=[int(info.width), int(info.height)], rostopic=f'/camera/{name}/image_raw',
            timeshift_cam_imu=0.)
    imu = read_yaml(Path(template)/'kalibr_imu_chain.yaml')
    # Engineering initial noise values shared with VINS, not an Allan calibration.
    imu['imu0'].update(rostopic='/camera/gyro_accel/sample', update_rate=200., time_offset=0.,
        accelerometer_noise_density=.1, gyroscope_noise_density=.01,
        accelerometer_random_walk=.001, gyroscope_random_walk=.0001)
    write_yaml(output/'estimator_config.yaml', settings)
    write_yaml(output/'kalibr_imucam_chain.yaml', cameras)
    write_yaml(output/'kalibr_imu_chain.yaml', imu)


def main():
    import argparse
    import json
    from types import SimpleNamespace
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--saved-calibration', type=Path, required=True,
                        help='Previously captured 336L calibration directory (calibration.json, left/right.yaml)')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    report = json.loads((args.saved_calibration/'calibration.json').read_text())
    if not report.get('body_T_imu'):
        raise ValueError('Saved calibration has no measured body_T_imu; do not invent IMU extrinsics')
    infos = {}
    for name, short in [('left_ir', 'left'), ('right_ir', 'right')]:
        data = read_yaml(args.saved_calibration/f'{short}.yaml')
        k = data['projection_parameters']
        d = data['distortion_parameters']
        infos[name] = SimpleNamespace(width=data['image_width'], height=data['image_height'],
            k=[k['fx'], 0., k['cx'], 0., k['fy'], k['cy'], 0., 0., 1.],
            d=[d[v] for v in ('k1', 'k2', 'p1', 'p2')])
    generate_openvins(args.output, root.parents[1]/'localization/visual_slam/open_vins/config/rs_d455', infos,
        {k: np.array(v) for k, v in report['transforms'].items()}, np.array(report['body_T_imu']))
    print(f"Generated OpenVINS stereo+IMU configuration for {report['camera_serial']} in {args.output}")


if __name__ == '__main__':
    main()
