"""Check camera/IMU transform conventions and generated stereo parameters."""
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import cv2
from scipy.spatial.transform import Rotation
from openvins_config import generate_openvins, read_yaml


class OpenVINSConfigTest(unittest.TestCase):
    def test_factory_frame_conversion_and_stereo(self):
        root = Path(__file__).resolve().parents[1]
        body_imu = np.eye(4)
        body_imu[:3, :3] = Rotation.from_euler('xyz', [8, -12, 33], degrees=True).as_matrix()
        body_imu[:3, 3] = [.01, -.02, .03]
        transforms = {}
        for name, x in [('left_ir', 0.), ('right_ir', .0946)]:
            t = np.eye(4)
            t[:3, :3] = Rotation.from_euler('xyz', [-90, 0, -90], degrees=True).as_matrix()
            t[:3, 3] = [0., -x, 0.]
            transforms[name] = t
        info = SimpleNamespace(width=640, height=400, k=[311.,0.,320.,0.,311.,198.,0.,0.,1.],d=[0.]*4)
        with tempfile.TemporaryDirectory() as tmp:
            generate_openvins(tmp, root.parents[1]/'localization/visual_slam/open_vins/config/rs_d455', dict(left_ir=info,right_ir=info), transforms, body_imu)
            for filename in ('estimator_config.yaml','kalibr_imucam_chain.yaml','kalibr_imu_chain.yaml'):
                fs=cv2.FileStorage(str(Path(tmp)/filename),cv2.FILE_STORAGE_READ)
                self.assertTrue(fs.isOpened())
                fs.release()
            cameras = read_yaml(Path(tmp)/'kalibr_imucam_chain.yaml')
            for index, name in enumerate(('left_ir', 'right_ir')):
                np.testing.assert_allclose(transforms[name]@np.array(cameras[f'cam{index}']['T_cam_imu']),body_imu,atol=1e-12)
            settings = read_yaml(Path(tmp)/'estimator_config.yaml')
            self.assertEqual(settings['max_cameras'],2)
            self.assertTrue(settings['use_stereo'])
            self.assertTrue(settings['try_zupt'])
            self.assertEqual(settings['zupt_chi2_multipler'],0.)
            self.assertFalse(settings['calib_cam_extrinsics'])
            imu = read_yaml(Path(tmp)/'kalibr_imu_chain.yaml')['imu0']
            self.assertEqual(imu['rostopic'],'/camera/gyro_accel/sample')
            self.assertEqual(imu['update_rate'],200.)


if __name__ == '__main__': unittest.main()
