"""Shared Quest body-frame extraction (Unity tracking positions are in metres)."""
import numpy as np


def body_features(poses, gravity_aligned=False):
    if poses.shape != (70, 7) or not np.isfinite(poses).all():
        raise ValueError('Expected 70 finite joint poses')
    p = poses[:, :3]
    up = p[5]-p[1]
    if np.linalg.norm(up) < .05:
        raise ValueError('Degenerate torso tracking')
    torso_up = up/np.linalg.norm(up)
    up = np.array([0., 1., 0.]) if gravity_aligned else torso_up
    left = p[10]-p[15]
    left -= up*np.dot(left, up)
    if np.linalg.norm(left) < .05:
        raise ValueError('Degenerate shoulder tracking')
    left /= np.linalg.norm(left)
    basis = np.array([np.cross(up, left), left, up])
    segments = []
    for shoulder, elbow, wrist in ((10, 11, 19), (15, 16, 45)):
        arm = np.array([p[elbow]-p[shoulder], p[wrist]-p[elbow]])
        if np.min(np.linalg.norm(arm, axis=1)) < .05:
            raise ValueError('Degenerate arm tracking')
        segments.append(arm @ basis.T)
    return basis, np.asarray(segments), p[[10, 15], 1].copy(), torso_up
