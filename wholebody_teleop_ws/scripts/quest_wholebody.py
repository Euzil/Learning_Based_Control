"""Controller hand positions -> supported WBC whole-body posture references.

All MuJoCo writes are to a separate kinematic MjData, never the simulated robot.
Targets use forward/left/up axes, yaw-aligned with the robot, and ground-relative Z.
"""
import math
import numpy as np
import mujoco
from quest_tracking import body_features
from quest_locomotion import remove_locomotion
from quest_orientation import rotation_error


def parse_controller(line):
    label, sep, payload = line.partition(':')
    name = label.split('|', 1)[0].strip().lower()
    if not sep or name not in ('left controller_wrist', 'right controller_wrist'):
        return None
    # Old controller packets include unlabelled locomotion offsets: do not guess.
    if not any(p.strip().startswith('locomotion =') for p in label.split('|')[1:]):
        return None
    try:
        values = np.array([float(v) for v in payload.strip().lstrip(',').split(',')])
    except ValueError:
        return None
    if values.shape != (7,) or not np.isfinite(values).all():
        return None
    values = remove_locomotion(label, values)
    if values is None:
        return None
    return (0 if name.startswith('left') else 1), values[:3]


def euler_quaternion(roll, pitch):
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    return np.array([cr*cp, sr*cp, cr*sp, -sr*sp])  # wxyz, yaw=0


def roll_pitch(quaternion_xyzw):
    q = np.asarray(quaternion_xyzw, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < .1:
        raise ValueError('Invalid feedback quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
                     math.asin(np.clip(2*(w*y-z*x), -1., 1.))])


class HandTargetMapper:
    """Fixed operator shoulder anchors preserve reach and torso displacement.

    This is position scaling, not normalized arm directions. The axes are captured
    once per streaming session so bending does not rotate the goal away.
    """
    def __init__(self, retargeter):
        self.solver = retargeter
        self.robot_shoulders = np.array([retargeter.data.xpos[a[2][0]].copy()
                                        for a in retargeter.arms])
        self.robot_shoulders[:, 2] += retargeter.default_body_height-retargeter.model.qpos0[2]
        self.reset()

    def reset(self):
        self.basis = None
        self.filtered = [None, None]

    def initialize(self, poses):
        if self.basis is not None:
            return
        self.basis, segments, _, _ = body_features(poses, gravity_aligned=True)
        self.human_shoulders = poses[[10, 15], :3].copy()
        lengths = np.linalg.norm(segments, axis=2).sum(axis=1)
        self.scales = np.array([sum(a[3:5]) for a in self.solver.arms])/lengths
        if self.solver.profile:
            self.scales = np.array([
                (a[3]*profile.upper_gain+a[4]*profile.forearm_gain) /
                (profile.upper_arm_m+profile.forearm_m)
                for a, profile in zip(self.solver.arms, self.solver.profile.arms)])
        if not np.isfinite(self.scales).all() or np.any(self.scales <= 0):
            self.reset()
            raise ValueError('Invalid hand target scale')

    def map(self, side, position, dt):
        if self.basis is None:
            return None
        position = np.asarray(position, dtype=float)
        if position.shape != (3,) or not np.isfinite(position).all():
            raise ValueError('Hand position must contain three finite values')
        target = self.robot_shoulders[side] + self.scales[side]*(self.basis @
                    (position-self.human_shoulders[side]))
        alpha = 1.-math.exp(-np.clip(dt, 0., .1)/.08)
        if self.filtered[side] is None:
            self.filtered[side] = target.copy()
        else:
            self.filtered[side] += alpha*(target-self.filtered[side])
        return self.filtered[side].copy()


class WholeBodyPlanner:
    """Damped whole-body IK with fixed horizontal base/yaw and anchored feet.

    Solve base height/roll/pitch, both legs, and arm positioning joints together.
    Wrist/head references stay supplied by tracking. Waist yaw stays measured:
    the loaded WBC exposes no independent waist-reference channel.
    """
    def __init__(self, model, policy, hand_local=None):
        self.model, self.policy = model, policy
        self.data = mujoco.MjData(model)
        self.names = policy.names
        self.ids = np.array([model.joint(n).id for n in self.names])
        self.qa, self.va = model.jnt_qposadr[self.ids], model.jnt_dofadr[self.ids]
        if model.jnt_type[0] != mujoco.mjtJoint.mjJNT_FREE:
            raise ValueError('Whole-body planning requires a floating base')
        self.hand_local = np.array(hand_local if hand_local is not None else [[.03,0,-.14]]*2)
        self.hands = [model.body(f'elbow_yaw_{s}_link').id for s in ('l','r')]
        self.feet = [model.body(f'ankle_roll_{s}_link').id for s in ('l','r')]
        self.indices = np.array([i for i,n in enumerate(self.names) if n.startswith(
            ('hip_', 'knee_', 'ankle_', 'shoulder_')) or n.startswith('elbow_pitch_')])
        self.dofs = self.va[self.indices]
        lo, hi = model.jnt_range[self.ids[self.indices]].T.copy()
        for n, bounds in zip(policy.arm_names, policy.arm_ranges):
            if n in [self.names[i] for i in self.indices]:
                j = list(self.indices).index(self.names.index(n))
                lo[j], hi[j] = max(lo[j],bounds[0]), min(hi[j],bounds[1])
        # Preserve the existing .9 rad tilt guard with margin for execution error.
        self.lower = np.r_[policy.meta['range_height'][0],
                           max(policy.meta['range_roll'][0],-.3),
                           max(policy.meta['range_pitch'][0],-.3), lo]
        self.upper = np.r_[policy.meta['range_height'][1],
                           min(policy.meta['range_roll'][1],.3),
                           min(policy.meta['range_pitch'][1],.75), hi]
        self.reset()

    def reset(self):
        self.previous = None
        self.last_result = None

    def height_command(self, planned_height, measured_height):
        # Correct the observed WBC height tracking bias; QuestPolicy applies its normal slew limit.
        return float(np.clip(planned_height+1.5*(planned_height-measured_height),
                             self.lower[0], self.upper[0]))

    def _set(self, x, fixed):
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[:3] = [0.,0.,x[0]]
        self.data.qpos[3:7] = euler_quaternion(x[1], x[2])
        self.data.qpos[self.qa] = fixed
        self.data.qpos[self.qa[self.indices]] = x[3:]
        mujoco.mj_forward(self.model, self.data)

    def _hand_points(self):
        return np.array([self.data.xpos[b]+self.data.xmat[b].reshape(3,3) @ loc
                         for b,loc in zip(self.hands,self.hand_local)])

    def _jac(self, point, body, roll):
        jp, jr = np.zeros((3,self.model.nv)), np.zeros((3,self.model.nv))
        mujoco.mj_jac(self.model,self.data,jp,jr,point,body)
        transform = np.zeros((self.model.nv, len(self.indices)+3))
        transform[2,0] = 1.
        transform[3,1] = 1.
        transform[4,2], transform[5,2] = math.cos(roll), -math.sin(roll)
        transform[self.dofs, np.arange(len(self.indices))+3] = 1.
        return jp @ transform, jr @ transform

    def solve(self, targets, position, quaternion_xyzw, base_height, upper_reference, preferred_height):
        targets = np.asarray(targets, dtype=float)
        if targets.shape != (2,3) or not np.isfinite(targets).all():
            raise ValueError('Expected two finite robot-heading hand targets')
        fixed = np.asarray(position, dtype=float).copy()
        if fixed.shape != (len(self.names),) or not np.isfinite(fixed).all() or not np.isfinite(base_height):
            raise ValueError('Invalid whole-body feedback')
        rp = roll_pitch(quaternion_xyzw)
        actual_x = np.r_[base_height,rp,fixed[self.indices]]
        self._set(actual_x,fixed)
        feet_pos = self.data.xpos[self.feet].copy()
        feet_rot = self.data.xmat[self.feet].reshape(2,3,3).copy()
        actual_hands = self._hand_points()
        fixed[self.policy.upper_ids] = upper_reference
        seed = fixed[self.indices].copy()
        preferred = np.r_[np.clip(preferred_height,self.lower[0],self.upper[0]),0.,0.,seed]
        x = np.clip(actual_x if self.previous is None else self.previous,self.lower,self.upper)
        # Foot anchors follow current feedback, not an old stance during commanded walking.
        for _ in range(14):
            self._set(x,fixed)
            errors, jacobians = [], []
            for target,point,body in zip(targets,self._hand_points(),self.hands):
                jp,_ = self._jac(point,body,x[1])
                errors.extend(8.*(target-point));jacobians.append(8.*jp)
            for pos,rot,body in zip(feet_pos,feet_rot,self.feet):
                jp,jr = self._jac(self.data.xpos[body],body,x[1])
                errors.extend(20.*(pos-self.data.xpos[body]));jacobians.append(20.*jp)
                errors.extend(3.*rotation_error(rot,self.data.xmat[body].reshape(3,3)));jacobians.append(3.*jr)
            # Weak upright/default-height preference resolves redundancy and recovers when reach permits.
            weight = np.r_[.65,.35,.35,np.full(len(self.indices),.035)]
            errors.extend(weight*(preferred-x));jacobians.append(np.diag(weight))
            jac = np.vstack(jacobians);err=np.array(errors)
            delta = np.linalg.solve(jac.T@jac+.025*np.eye(len(x)),jac.T@err)
            delta = np.clip(delta, -np.r_[.04,.08,.08,np.full(len(self.indices),.12)],
                                   np.r_[.04,.08,.08,np.full(len(self.indices),.12)])
            x = np.clip(x+delta,self.lower,self.upper)
            if np.linalg.norm(delta)<1e-5:
                break
        self._set(x,fixed)
        q = self.data.qpos[self.qa].copy()
        result = dict(upper=q[self.policy.upper_ids], posture=x[:3].copy(),
                      planned_joints=q, hand_error=np.linalg.norm(self._hand_points()-targets,axis=1),
                      actual_hand_error=np.linalg.norm(actual_hands-targets,axis=1),
                      foot_error=np.linalg.norm(self.data.xpos[self.feet]-feet_pos,axis=1))
        self.previous = x.copy();self.last_result = result
        return result
