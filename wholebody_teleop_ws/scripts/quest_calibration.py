"""Robot-led multi-pose calibration. No ROS imports: state machine is testable offline."""
import logging
import math
import os
from pathlib import Path
import tempfile

import mujoco
import numpy as np
import yaml

from quest_body_profile import BodyProfile
from quest_tracking import body_features

# pitch, outward roll magnitude, elbow pitch. All four poses are fitted.
POSES = (
    ('直立垂臂：保持站立，模仿机器人双臂自然下垂', 0., .08, -.10),
    ('双臂前举：模仿机器人的前向伸臂姿态', -1.35, .08, -.10),
    ('双臂侧举：模仿机器人向两侧伸臂的姿态', 0., 1.30, -.10),
    ('屈肘：上臂下垂，双肘弯曲，前臂向前', 0., .08, -1.45),
)


def robot_points(solver, q):
    data = mujoco.MjData(solver.model)
    data.qpos[:] = solver.data.qpos
    data.qpos[solver.qadr] = q
    mujoco.mj_forward(solver.model, data)
    arms = []
    for i, (_, _, bodies, _, _) in enumerate(solver.arms):
        points = data.xpos[bodies].copy()
        points[2] += data.xmat[bodies[2]].reshape(3, 3) @ solver.hand_local[i]
        arms.append(points)
    return np.asarray(arms)


def demo_pose(solver, index):
    _, pitch, roll, elbow = POSES[index]
    q = solver.rest.copy()
    for side, sign in (('l', 1), ('r', -1)):
        for joint, value in (('shoulder_pitch', pitch), ('shoulder_roll', sign*roll),
                             ('shoulder_yaw', 0.), ('elbow_pitch', elbow), ('elbow_yaw', 0.)):
            q[solver.joints['names'].index(f'{joint}_{side}_joint')] = value
    limits = solver.model.jnt_range[solver.ids]
    if np.any(q < limits[:, 0]) or np.any(q > limits[:, 1]):
        raise ValueError('Calibration demonstration exceeds robot joint limits')
    return q


def relative_targets(points, shoulder_z):
    targets = points[:, 1:]-points[:, :1]
    targets[:, :, 2] = points[:, 1:, 2]-np.asarray(shoulder_z)[:, None]
    return targets


def fit_scale_offset(x, y):
    """Fit the collected data without a motion-quality acceptance threshold.

    Identical input vectors cannot identify a scale: use zero slope and the
    mean output instead of inventing motion or requiring another collection.
    """
    xc, yc = x-x.mean(axis=0), y-y.mean(axis=0)
    denominator = np.sum(xc*xc)
    scale = float(np.sum(xc*yc)/denominator) if denominator > np.finfo(float).eps else 0.
    return scale, y.mean(axis=0)-scale*x.mean(axis=0)


def fit_profile(solver, records):
    if len(records) != len(POSES):
        raise ValueError('All four sampling poses are required')
    human = np.array([r['segments'] for r in records])
    heights = np.array([r['heights'] for r in records])
    robot = np.array([r['robot'] for r in records])
    if (human.shape != (len(POSES), 2, 2, 3) or heights.shape != (len(POSES), 2) or
            robot.shape != (len(POSES), 2, 3, 3) or not all(np.isfinite(x).all() for x in (human, heights, robot))):
        raise ValueError('Invalid calibration observations')
    human_ref, robot_ref = heights[0], robot[0, :, 0, 2]
    # Standing demonstrations do not identify a separate crouching gain.
    # Preserve vertical shoulder displacement at 1:1, explicitly saved for tuning.
    vertical_scale = 1.
    expected = np.array([relative_targets(p, robot_ref) for p in robot])
    fit_targets = expected.copy()
    fit_targets[:, :, :, 2] -= (heights-human_ref)[:, :, None]*vertical_scale
    cfg = {'elbow_weight': .5, 'hand_weight': 1., 'calibration': {
        'version': 1, 'fit_mode': 'sampled', 'human_shoulder_y_m': human_ref.tolist(),
        'robot_shoulder_z_m': robot_ref.tolist(), 'vertical_scale': vertical_scale}}
    predicted = np.zeros_like(expected)
    for i, side in enumerate(('left', 'right')):
        upper, lower = human[:, i, 0], human[:, i, 1]
        a, offset = fit_scale_offset(upper, fit_targets[:, i, 0])
        # Fit hand using fitted upper target, preserving the deployed formula.
        b, hand_offset = fit_scale_offset(lower, fit_targets[:, i, 1]-(a*upper+offset))
        lengths = np.median(np.linalg.norm(human[:, i], axis=2), axis=0)
        robot_lengths = solver.arms[i][3:5]
        cfg[side] = {
            'upper_arm_m': float(lengths[0]), 'forearm_m': float(lengths[1]),
            'upper_gain': float(a*lengths[0]/robot_lengths[0]),
            'forearm_gain': float(b*lengths[1]/robot_lengths[1]),
            'elbow_offset_m': offset.tolist(), 'hand_offset_m': hand_offset.tolist(),
            'robot_hand_local_m': solver.hand_local[i].tolist()}
        predicted[:, i, 0] = a*human[:, i, 0]+offset
        predicted[:, i, 1] = predicted[:, i, 0]+b*human[:, i, 1]+hand_offset
    predicted[:, :, :, 2] += (heights-human_ref)[:, :, None]*vertical_scale
    errors = np.linalg.norm(predicted-expected, axis=3)
    report = {'fit_rms_cm': (100*np.sqrt(np.mean(errors**2, axis=0))).tolist(),
              'max_error_cm': float(100*errors.max()),
              'per_pose_error_cm': (100*errors).tolist(),
              'error_order': 'rows: left/right; columns: elbow/hand',
              'note': 'Model endpoint errors from measured simulated joints; not ground-height calibration.'}
    # Check serialization/numerical validity only; residuals never reject a fit.
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory)/'profile.yaml'
        path.write_text(yaml.safe_dump(cfg))
        BodyProfile.load(path)
    return cfg, report


def atomic_yaml(path, value, overwrite=False):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    name = None
    try:
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as f:
            name = f.name
            yaml.safe_dump(value, f, allow_unicode=True, sort_keys=False)
            f.flush()
            os.fsync(f.fileno())
        if overwrite:
            os.replace(name, path)
        else:
            os.link(name, path)  # Atomic no-clobber, including a race with another session.
    finally:
        if name and os.path.exists(name):
            os.unlink(name)


class CalibrationSession:
    def __init__(self, solver, output, overwrite=False, prepare_seconds=5., sample_seconds=3.):
        self.solver, self.output, self.overwrite = solver, Path(output), overwrite
        if self.output.exists() and not overwrite:
            raise ValueError(f'{output} 已存在；使用新的文件名或 --overwrite-calibration')
        self.goals = [demo_pose(solver, i) for i in range(len(POSES))]
        self.prepare_seconds, self.sample_seconds = prepare_seconds, sample_seconds
        self.index, self.phase = 0, 'waiting'
        self.since, self.last_sample, self.last_log = None, None, -math.inf
        self.samples, self.records = [], []
        self.done = False
        self.report = None
        self.failure = None

    def tell(self, now, message, force=False):
        if force or now-self.last_log >= 5:
            logging.warning('标定 %d/%d：%s', self.index+1, len(POSES), message)
            self.last_log = now

    def step(self, now, dt, poses, feedback, feedback_time):
        if self.done or self.failure:
            return
        # Feedback and human frames must both be fresh. Main feeds only new body packets.
        if feedback is None or now-feedback_time > .2 or poses is None:
            if self.last_sample is not None and now-self.last_sample > .2:
                self.samples.clear()
                self.since = None
                self.phase = 'waiting'
            if self.last_sample is None or now-self.last_sample > .2:
                self.tell(now, '等待新鲜的身体追踪与仿真关节反馈；保持当前指令')
            return
        try:
            _, segments, heights, _ = body_features(poses, gravity_aligned=True)
        except ValueError:
            self.samples.clear()
            self.since = None
            self.phase = 'waiting'
            self.tell(now, '身体追踪无效，暂停采样')
            return
        if self.last_sample is not None and now-self.last_sample > .2:
            self.samples.clear()
            self.since = None
            self.phase = 'waiting'
        self.last_sample = now
        if self.phase == 'waiting':
            self.phase = 'moving'
            self.tell(now, POSES[self.index][0], force=True)
        solver, target = self.solver, self.goals[self.index]
        max_step = .6*min(max(dt, 0.), .05)
        solver.q += np.clip(target-solver.q, -max_step, max_step)
        solver.data.qpos[solver.qadr] = solver.q
        mujoco.mj_forward(solver.model, solver.data)
        if np.max(abs(target-solver.q)) > .005 or np.max(abs(feedback-target)) > .12:
            self.phase, self.since = 'moving', None
            self.samples.clear()
            self.tell(now, '机器人正在到位，请模仿其动作')
            return
        if self.phase == 'moving':
            self.phase, self.since = 'prepare', now
            self.tell(now, f'{POSES[self.index][0]}；准备 {self.prepare_seconds:g} 秒', force=True)
        if self.phase == 'prepare':
            if now-self.since < self.prepare_seconds:
                return
            self.phase, self.since = 'sampling', now
            self.tell(now, f'稳定保持约 {self.sample_seconds:g} 秒，正在采样', force=True)
        points = robot_points(solver, feedback)
        self.samples.append((now, segments.copy(), heights.copy(), points))
        if now-self.samples[0][0] < self.sample_seconds:
            return
        # Use every valid frame in the collection interval, without pose matching,
        # stability scoring or outlier rejection.
        centres = [np.mean([sample[k] for sample in self.samples], axis=0) for k in (1, 2, 3)]
        self.records.append(dict(zip(('segments', 'heights', 'robot'), centres)))
        self.samples.clear()
        if self.index+1 < len(POSES):
            self.index += 1
            self.phase, self.since = 'waiting', None
            return
        try:
            cfg, self.report = fit_profile(solver, self.records)
            atomic_yaml(self.output, cfg, self.overwrite)
            # The profile is already valid. A diagnostic write must not invalidate it.
            try:
                atomic_yaml(str(self.output)+'.report.yaml', self.report, overwrite=True)
            except OSError as exc:
                logging.warning('标定配置已保存，但报告写入失败：%s', exc)
            solver.profile = BodyProfile.load(self.output)
            solver.hand_height_bias = 0.
            solver.filtered_directions = solver.filtered_positions = solver.filtered_head = None
            self.done = True
            logging.warning('标定完成，已保存 %s；拟合误差（仅记录，不筛选）cm（左/右 肘/手）：%s。开始真人跟随。',
                            self.output, self.report['fit_rms_cm'])
        except (ValueError, OSError) as exc:
            self.failure = str(exc)
            logging.error('标定失败，保持最后姿态：%s', exc)
