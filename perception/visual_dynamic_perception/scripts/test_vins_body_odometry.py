"""Frame-composition checks for the VINS IMU -> camera body adapter."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from body_odometry import camera_pose


class FrameConversion(unittest.TestCase):
    def test_nonidentity_extrinsic_and_rotating_body(self):
        body_imu = np.eye(4)
        body_imu[:3, :3] = Rotation.from_euler('xyz', [-90, 0, -90], degrees=True).as_matrix()
        body_imu[:3, 3] = [-.014248, -.007886, -.001086]
        world_body = np.eye(4)
        world_body[:3, :3] = Rotation.from_euler('xyz', [10, 25, 40], degrees=True).as_matrix()
        world_body[:3, 3] = [1.2, -.4, .7]
        # Physically co-mounted IMU and body must give the same body pose.
        world_imu = world_body @ body_imu
        np.testing.assert_allclose(camera_pose(world_imu, body_imu), world_body, atol=1e-12)
        self.assertGreater(np.linalg.norm(world_imu-world_body), .1)


if __name__ == '__main__':
    unittest.main()
