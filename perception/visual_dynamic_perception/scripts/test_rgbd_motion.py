#!/usr/bin/python3
"""Known-geometry tests: moving camera, dynamic outliers, lost tracking."""
import unittest
import cv2
import numpy as np
from rgbd_motion import RGBDMotion, estimate_motion


class MotionTests(unittest.TestCase):
    def setUp(self):
        cv2.setRNGSeed(7)
        self.rng = np.random.default_rng(7)
        self.k = np.array([[311., 0, 320.], [0, 311., 200.], [0, 0, 1.]])

    def test_motion_with_independently_moving_foreground(self):
        points = self.rng.uniform([-2., -1.3, 2.5], [2., 1.3, 5.], (300, 3))
        rotation = np.array([.015, .06, -.01])
        translation = np.array([.06, -.015, .025])
        pixels = cv2.projectPoints(points, rotation, translation, self.k, np.zeros(4))[0].reshape(-1, 2)
        pixels += self.rng.normal(0, .15, pixels.shape)
        # A moving object supplies 25% of all correspondences.
        pixels[:75] += self.rng.uniform(15, 60, (75, 2))
        transform, quality = estimate_motion(points, pixels, self.k, (640, 400), .1)
        self.assertIsNotNone(transform, quality)
        self.assertLess(np.linalg.norm(transform[:3, 3]-translation), .005)
        self.assertLess(np.linalg.norm(transform[:3, :3]-cv2.Rodrigues(rotation)[0]), .005)
        self.assertGreater(quality['inlier_ratio'], .65)

    def test_moving_camera_over_static_textured_plane(self):
        gray = self.rng.integers(15, 240, (400, 640), dtype=np.uint8)
        gray = cv2.GaussianBlur(gray, (5, 5), .8)
        rotation = cv2.Rodrigues(np.array([.005, .025, -.008]))[0]
        translation = np.array([.04, -.01, .015])
        homography = self.k@(rotation+np.outer(translation, [0, 0, 1])/3.)@np.linalg.inv(self.k)
        moved = cv2.warpPerspective(gray, homography, (640, 400))
        yy, xx = np.mgrid[:400, :640]
        rays = np.stack(((xx-320)/311., (yy-200)/311., np.ones_like(xx)), axis=-1)
        normal = rotation@np.array([0., 0., 1.])
        depth = ((3.+normal@translation)/(rays@normal)*1000).astype(np.uint16)
        estimator = RGBDMotion([311, 311, 320, 200], np.eye(4))
        origin, _ = estimator.process(gray, np.full((400, 640), 3000, np.uint16), 0.)
        np.testing.assert_allclose(origin, np.eye(4))
        pose, quality = estimator.process(moved, depth, .1)
        self.assertIsNotNone(pose, quality)
        expected = np.eye(4)
        expected[:3, :3], expected[:3, 3] = rotation, translation
        self.assertLess(np.linalg.norm(pose[:3, 3]-np.linalg.inv(expected)[:3, 3]), .01)
        # Static scene points must remain fixed after estimated camera compensation.
        original = np.array([.4, .2, 3., 1.])
        self.assertLess(np.linalg.norm((pose@expected@original-original)[:3]), .02)
        trusted = estimator.world_camera.copy()
        invalid, _ = estimator.process(np.zeros_like(gray), np.zeros_like(depth), .2)
        self.assertIsNone(invalid)
        np.testing.assert_allclose(estimator.world_camera, trusted)
        estimator.reset_local_origin()
        self.assertIsNone(estimator.reference)
        restarted, _ = estimator.process(moved, depth, .3)
        np.testing.assert_allclose(restarted, np.eye(4))

    def test_small_foreground_patch_is_not_camera_motion(self):
        pixels = self.rng.uniform([270, 170], [350, 220], (100, 2))
        z = self.rng.uniform(1, 2, 100)
        points = np.column_stack(((pixels[:, 0]-320)*z/311, (pixels[:, 1]-200)*z/311, z))
        transform, quality = estimate_motion(points, pixels+np.array([15., 0.]), self.k, (640, 400), .1)
        self.assertIsNone(transform, quality)


if __name__ == '__main__':
    unittest.main(verbosity=2)
