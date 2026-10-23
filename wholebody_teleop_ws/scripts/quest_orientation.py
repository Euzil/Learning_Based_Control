"""Orientation helpers for Unity quaternions and original PM01 end-joint tracking."""
import math
import mujoco
import numpy as np


DEFAULT_WRIST_OFFSETS_DEG = (90., -90.)


def rotation_xyzw(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .1:
        return None
    q = q/np.linalg.norm(q)
    matrix = np.empty(9)
    mujoco.mju_quat2Mat(matrix, q[[3, 0, 1, 2]])
    return matrix.reshape(3, 3)


def rotation_error(target, current):
    q = np.empty(4)
    mujoco.mju_mat2Quat(q, np.ascontiguousarray(target @ current.T).ravel())
    if q[0] < 0:
        q = -q
    length = np.linalg.norm(q[1:])
    return q[1:]*(2*math.atan2(length, q[0])/length) if length > 1e-10 else 2*q[1:]


def swing_rotation(source, target):
    """Shortest rotation of the forearm shaft, without imposing an axial twist."""
    dot = float(np.clip(np.dot(source, target), -1., 1.))
    if dot < -.9999:
        return None  # A 180-degree shaft reversal has no unique swing plane.
    cross = np.cross(source, target)
    x, y, z = cross
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3)+skew+(skew @ skew)/(1.+dot)


class WristOrientation:
    """Map original forearm twist and optional simulated palm flexion independently."""
    def __init__(self, model, names, ids, dadr, offsets_deg=DEFAULT_WRIST_OFFSETS_DEG):
        offsets = np.asarray(offsets_deg, dtype=float)
        if offsets.shape != (2,) or not np.isfinite(offsets).all() or np.any(abs(offsets) > 150.):
            raise ValueError("wrist offsets must be two finite angles within +/-150 degrees")
        self.model, self.ids = model, ids
        self.arms = []
        for arm_index, (side, human) in enumerate((('l', (11, 19)), ('r', (16, 45)))):
            name = f'elbow_yaw_{side}_joint'
            if name not in names:
                continue
            self.arms.append(dict(human=human, indices=[names.index(name)],
                                  body=model.body(f'elbow_yaw_{side}_link').id,
                                  elbow=model.body(f'elbow_pitch_{side}_link').id,
                                  neutral=None, direction=None, filtered=None,
                                  previous_angle=0., reference=0., sign=1.,
                                  offset=float(np.deg2rad(offsets[arm_index])),
                                  flex_index=(names.index(f'sim_wrist_flex_{side}_joint')
                                              if f'sim_wrist_flex_{side}_joint' in names else None),
                                  knuckles=(25, 30, 40) if side == 'l' else (51, 56, 66),
                                  # Both simulated hinges use local +X; the measured
                                  # knuckle ordering already carries the right-hand mirror.
                                  flex_sign=1.,
                                  flex_neutral=None, flex_filtered=None))

    def reset(self):
        for arm in self.arms:
            arm.update(neutral=None, direction=None, filtered=None, previous_angle=0.,
                       flex_neutral=None, flex_filtered=None)

    def solve_flexion(self, arm, poses, basis, direction, q, alpha):
        index = arm['flex_index']
        if index is None:
            return
        first, middle, little = arm['knuckles']
        # Proximal knuckles define the palm, so finger curling does not bend
        # the wrist. Signed angle is invariant to whole-arm motion and twist.
        across = basis @ (poses[first, :3]-poses[little, :3])
        hand = basis @ (poses[middle, :3]-poses[arm['human'][1], :3])
        if not np.isfinite(np.r_[across, hand]).all() or np.linalg.norm(across) < .015:
            return
        across /= np.linalg.norm(across)
        hand -= across*np.dot(hand, across)
        forearm = direction-across*np.dot(direction, across)
        if np.linalg.norm(hand) < .02 or np.linalg.norm(forearm) < .2:
            return
        hand /= np.linalg.norm(hand)
        forearm /= np.linalg.norm(forearm)
        angle = arm['flex_sign']*math.atan2(np.dot(np.cross(forearm, hand), across),
                                           np.dot(forearm, hand))
        if arm['flex_neutral'] is None:
            arm['flex_neutral'] = angle
            arm['flex_filtered'] = float(q[index])
        delta = (angle-arm['flex_neutral']+math.pi) % (2*math.pi)-math.pi
        target = np.clip(delta, *self.model.jnt_range[self.ids[index]])
        arm['flex_filtered'] += alpha*(target-arm['flex_filtered'])
        q[index] = arm['flex_filtered']

    def solve(self, poses, basis, q, qadr, data, alpha):
        reflection = np.diag([1., 1., -1.])
        for arm in self.arms:
            elbow_id, wrist_id = arm['human']
            wrist = rotation_xyzw(poses[wrist_id, 3:7])
            if wrist is None:
                continue
            human_wrist = basis @ wrist @ reflection
            direction = basis @ (poses[wrist_id, :3]-poses[elbow_id, :3])
            length = np.linalg.norm(direction)
            if length < .05:
                continue
            direction /= length
            self.solve_flexion(arm, poses, basis, direction, q, alpha)
            index = arm['indices'][0]
            if arm['neutral'] is None:
                mujoco.mj_forward(self.model, data)
                arm['neutral'] = human_wrist.copy()
                arm['direction'] = direction.copy()
                # Absolute joint zero plus correction, never the previous command:
                # resetting after calibration must not accumulate the offset.
                arm['reference'] = arm['offset']
                arm['filtered'] = float(q[index])
                shaft = data.xpos[arm['body']]-data.xpos[arm['elbow']]
                axis = data.xaxis[self.ids[index]]
                arm['sign'] = 1. if np.dot(axis, shaft) >= 0. else -1.
            swing = swing_rotation(arm['direction'], direction)
            if swing is None:
                continue
            change = swing.T @ human_wrist @ arm['neutral'].T
            quaternion = np.empty(4)
            mujoco.mju_mat2Quat(quaternion, np.ascontiguousarray(change).ravel())
            # Keep palm flexion separate from the original axial twist command.
            axial = float(np.dot(quaternion[1:], arm['direction']))
            if math.hypot(quaternion[0], axial) < 1e-8:
                continue
            raw = 2*math.atan2(axial, quaternion[0])
            angle = arm['previous_angle'] + (raw-arm['previous_angle']+math.pi) % (2*math.pi)-math.pi
            arm['previous_angle'] = angle
            low, high = self.model.jnt_range[self.ids[index]]
            target = np.clip(arm['reference']+arm['sign']*angle, low, high)
            arm['filtered'] += alpha*(target-arm['filtered'])
            q[index] = arm['filtered']
            data.qpos[qadr] = q
