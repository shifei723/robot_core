"""Metric RGB-D camera motion from robust 3D-to-2D correspondences."""
import cv2
import numpy as np


def estimate_motion(points, pixels, camera_matrix, image_size, dt):
    """Return current_camera_T_previous_camera, or None on weak geometry.

    Dynamic-object correspondences are RANSAC outliers. Spatial coverage and
    inlier checks prevent accepting a small moving foreground patch as ego pose.
    """
    points = np.asarray(points, np.float64).reshape(-1, 3)
    pixels = np.asarray(pixels, np.float64).reshape(-1, 2)
    report = {'matches': len(points), 'valid': False}
    if len(points) < 40 or dt <= 0:
        return None, report
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        points, pixels, camera_matrix, np.zeros(4), iterationsCount=160,
        reprojectionError=2., confidence=.999, flags=cv2.SOLVEPNP_EPNP)
    if not ok or inliers is None:
        return None, report
    indices = inliers.ravel()
    ratio = len(indices)/len(points)
    width, height = image_size
    tiles = np.clip((pixels[indices]/[width, height]*[6, 4]).astype(int), [0, 0], [5, 3])
    coverage = len(np.unique(tiles, axis=0))
    report.update(inliers=len(indices), inlier_ratio=ratio, tiles=coverage)
    if len(indices) < 35 or ratio < .6 or coverage < 6:
        return None, report
    ok, rvec, tvec = cv2.solvePnP(points[indices], pixels[indices], camera_matrix,
        np.zeros(4), rvec, tvec, True, flags=cv2.SOLVEPNP_ITERATIVE)
    projection, _ = cv2.projectPoints(points[indices], rvec, tvec, camera_matrix, np.zeros(4))
    error = float(np.median(np.linalg.norm(projection.reshape(-1, 2)-pixels[indices], axis=1)))
    angle = float(np.linalg.norm(rvec))
    translation = float(np.linalg.norm(tvec))
    report.update(reprojection_px=error, translation_m=translation, rotation_rad=angle)
    if not ok or error > 1.3 or translation > .025+3.*dt or angle > .025+3.*dt:
        return None, report
    transform = np.eye(4)
    transform[:3, :3] = cv2.Rodrigues(rvec)[0]
    transform[:3, 3] = tvec.ravel()
    if not np.isfinite(transform).all():
        return None, report
    report.update(valid=True, indices=indices)
    return transform, report


class RGBDMotion:
    def __init__(self, intrinsics, body_to_camera):
        fx, fy, cx, cy = intrinsics
        self.k = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)
        self.body_to_camera = np.asarray(body_to_camera, np.float64).reshape(4, 4)
        self.world_camera = self.body_to_camera.copy()
        self.reference = None
        self.orb = cv2.ORB_create(nfeatures=1100)

    def unproject(self, pixels, depth):
        xy = np.rint(pixels).astype(int)
        valid = ((xy[:, 0] >= 2) & (xy[:, 0] < depth.shape[1]-2) &
                 (xy[:, 1] >= 2) & (xy[:, 1] < depth.shape[0]-2))
        z = np.zeros(len(xy))
        z[valid] = depth[xy[valid, 1], xy[valid, 0]]*.001
        valid &= (z > .35) & (z < 5.)
        points = np.column_stack(((pixels[:, 0]-self.k[0, 2])*z/self.k[0, 0],
                                  (pixels[:, 1]-self.k[1, 2])*z/self.k[1, 1], z))
        return points, valid

    def set_reference(self, gray, depth, stamp):
        self.reference = (gray.copy(), depth.copy(), stamp)

    def reset_local_origin(self):
        """Explicit new local session; not relocalization into the old map."""
        self.reference = None
        self.world_camera = self.body_to_camera.copy()

    def process(self, gray, depth, stamp):
        if self.reference is None:
            corners = cv2.goodFeaturesToTrack(gray, 650, .015, 10)
            if corners is None:
                return None, {'valid': False, 'reason': 'no initial texture'}
            _, valid = self.unproject(corners.reshape(-1, 2), depth)
            if valid.sum() < 50:
                return None, {'valid': False, 'reason': 'insufficient initial metric features'}
            self.set_reference(gray, depth, stamp)
            return np.eye(4), {'valid': True, 'reason': 'metric origin initialized'}
        previous, previous_depth, previous_stamp = self.reference
        dt = stamp-previous_stamp
        if dt <= 0:
            return None, {'valid': False, 'reason': 'non-increasing acquisition time'}
        corners = cv2.goodFeaturesToTrack(previous, 650, .015, 10)
        if corners is None:
            return None, {'valid': False, 'reason': 'no reference features'}
        current, tracked, _ = cv2.calcOpticalFlowPyrLK(previous, gray, corners, None,
            winSize=(25, 25), maxLevel=4)
        back, backward, _ = cv2.calcOpticalFlowPyrLK(gray, previous, current, None,
            winSize=(25, 25), maxLevel=4)
        keep = tracked.ravel().astype(bool) & backward.ravel().astype(bool)
        keep &= np.linalg.norm(back-corners, axis=2).ravel() < .8
        old_pixels = corners.reshape(-1, 2)[keep]
        new_pixels = current.reshape(-1, 2)[keep]
        points, depth_valid = self.unproject(old_pixels, previous_depth)
        transform, report = estimate_motion(points[depth_valid], new_pixels[depth_valid],
            self.k, (gray.shape[1], gray.shape[0]), dt)
        if transform is None:
            # Re-match to the last trusted image, never silently reset to a fixed
            # pose after tracking loss. This also permits recovery after a pause.
            old_keys, old_desc = self.orb.detectAndCompute(previous, None)
            new_keys, new_desc = self.orb.detectAndCompute(gray, None)
            if old_desc is not None and new_desc is not None:
                pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(old_desc, new_desc, k=2)
                matches = [a for pair in pairs if len(pair)==2 for a, b in [pair] if a.distance < .7*b.distance]
                if len(matches) >= 40:
                    old_pixels = np.array([old_keys[m.queryIdx].pt for m in matches])
                    new_pixels = np.array([new_keys[m.trainIdx].pt for m in matches])
                    points, depth_valid = self.unproject(old_pixels, previous_depth)
                    transform, report = estimate_motion(points[depth_valid], new_pixels[depth_valid],
                        self.k, (gray.shape[1], gray.shape[0]), dt)
        if transform is None:
            return None, report
        # Verify the motion against independently acquired current depth too.
        projected = new_pixels[depth_valid][report['indices']]
        current_points, available = self.unproject(projected, depth)
        predicted = (transform[:3, :3]@points[depth_valid][report['indices']].T).T+transform[:3, 3]
        consistent = np.abs(predicted[:, 2]-current_points[:, 2]) < .06+.02*predicted[:, 2]
        support = float(np.mean(consistent[available])) if available.sum() >= 25 else 0.
        report.pop('indices', None)
        report['depth_support'] = support
        if support < .7:
            report.update(valid=False, reason='current depth contradicts visual motion')
            return None, report
        self.world_camera = self.world_camera@np.linalg.inv(transform)
        self.set_reference(gray, depth, stamp)
        return self.world_camera@np.linalg.inv(self.body_to_camera), report
