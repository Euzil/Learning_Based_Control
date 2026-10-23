"""Original wrist twist and simulated palm/head mapping, model, and dynamics checks."""
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import mujoco
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter
from quest_orientation import rotation_error, rotation_xyzw
from quest_tracking import body_features
from test_quest_retarget import sample, yaw_quat


def axis_quat(axis, angle):
    axis = np.asarray(axis, dtype=float)
    return np.r_[axis/np.linalg.norm(axis)*np.sin(angle/2), np.cos(angle/2)]


def multiply(a, b):
    return np.r_[a[3]*b[:3]+b[3]*a[:3]+np.cross(a[:3], b[:3]),
                 a[3]*b[3]-np.dot(a[:3], b[:3])]


class OrientationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = tempfile.TemporaryDirectory()
        cls.scripts = Path(__file__).resolve().parents[1]
        subprocess.run([sys.executable, str(cls.scripts/'prepare_quest_teleop.py'), cls.runtime.name], check=True)
        cls.config = str(Path(cls.runtime.name)/'config.yaml')

    @classmethod
    def tearDownClass(cls):
        cls.runtime.cleanup()

    def test_runtime_adds_flexion_preserving_original_wrists(self):
        solver = Retargeter(self.config)
        physical = yaml.safe_load((self.scripts.parent/'src/robots/pm01_description/configs/pm01.yaml').read_text())
        names = physical['robots'][0]['joints']['names']
        self.assertEqual(solver.joints['names'][:len(names)], names)
        self.assertEqual(len(solver.q), len(names)+3)
        self.assertEqual(len(solver.wrists.arms), 2)
        self.assertEqual([a['indices'][0] for a in solver.wrists.arms], [17, 22])
        for side in ('l', 'r'):
            self.assertIn(f'sim_wrist_flex_{side}_joint', solver.joints['names'])
            self.assertEqual(solver.model.geom(f'sim_palm_{side}').contype, 0)
        self.assertIsNotNone(solver.head_pitch)
        self.assertNotIn('sim_head_pitch_joint', names)
        self.assertEqual(solver.model.camera('head_camera').bodyid, solver.model.body('head_yaw_link').id)

    def test_head_camera_looks_up_and_down_with_camera_and_bone_input(self):
        for bone in (False, True):
            for angle in (-.4, .4):
                with self.subTest(bone=bone, pitch=angle):
                    solver = Retargeter(self.config)
                    p = sample()
                    head = np.r_[np.zeros(3), axis_quat([1, 0, 0], angle)]
                    if bone:
                        p[7, 3:] = multiply(head[3:], p[7, 3:])
                    for _ in range(90):
                        before = solver.q.copy()
                        solver.solve(p, 1/60, None if bone else head)
                        self.assertLessEqual(max(abs(solver.q-before)), .025001)
                    self.assertAlmostEqual(solver.q[solver.head_pitch], angle, places=5)
                    camera = solver.model.camera('head_camera').id
                    gaze = -solver.data.cam_xmat[camera].reshape(3, 3)[:, 2]
                    self.assertAlmostEqual(gaze[2], -np.sin(angle), places=5)

    def test_twist_moves_original_mesh_and_bending_does_not_add_motion(self):
        for axis in ([1, 0, 0], [0, 1, 0], [0, 0, 1]):
            with self.subTest(axis=axis):
                solver = Retargeter(self.config, wrist_offsets_deg=(0., 0.))
                p = sample()
                for _ in range(80):
                    solver.solve(p, 1/60)
                original_q = solver.q.copy()
                wrist = solver.wrists.arms[0]
                initial = solver.data.xmat[wrist['body']].reshape(3, 3).copy()
                p[19, 3:] = axis_quat(axis, .3)
                p[11, 3:] = p[19, 3:]
                for _ in range(90):
                    before = solver.q.copy()
                    solver.solve(p, 1/60)
                    self.assertLessEqual(max(abs(solver.q-before)), .025001)
                if axis == [0, 1, 0]:  # Forearm is vertical: only Unity Y is twist.
                    self.assertAlmostEqual(solver.q[17], -.3, places=5)
                    current = solver.data.xmat[wrist['body']].reshape(3, 3)
                    self.assertGreater(np.linalg.norm(rotation_error(initial, current)), .2)
                else:
                    self.assertAlmostEqual(solver.q[17], 0., places=5)
                np.testing.assert_allclose(solver.q[18:23], original_q[18:23], atol=.001)

    def test_zero_offsets_twist_and_reset(self):
        solver = Retargeter(self.config)
        p = sample()
        for _ in range(180):
            before = solver.q.copy()
            solver.solve(p, 1/60)
            self.assertLessEqual(max(abs(solver.q-before)), .025001)
        np.testing.assert_allclose(solver.q[[17, 22]], [np.pi/2, -np.pi/2], atol=1e-5)
        p[19, 3:] = axis_quat([0, 1, 0], .3)
        for _ in range(100):
            solver.solve(p, 1/60)
        np.testing.assert_allclose(solver.q[[17, 22]], [np.pi/2-.3, -np.pi/2], atol=1e-5)
        for _ in range(2):
            solver.wrists.reset()
            for _ in range(100):
                solver.solve(p, 1/60)
            np.testing.assert_allclose(solver.q[[17, 22]], [np.pi/2, -np.pi/2], atol=1e-5)

    def test_palm_flexion_independent_of_arm_motion_and_tracking_loss(self):
        solver = Retargeter(self.config)
        basis = body_features(sample())[0]
        def pose(left, right):
            p = sample()
            for wrist, knuckles, sign, angle in ((19, (25, 30, 40), 1, left),
                                                (45, (51, 56, 66), -1, right)):
                hand = basis.T @ np.array([0., np.sin(angle), -np.cos(angle)])*.075
                across = basis.T @ np.array([sign*.03, 0., 0.])
                p[knuckles[1], :3] = p[wrist, :3]+hand
                p[knuckles[0], :3] = p[wrist, :3]+hand+across
                p[knuckles[2], :3] = p[wrist, :3]+hand-across
            return p
        for _ in range(80):
            solver.solve(pose(0., 0.), 1/60)
        indices = [a['flex_index'] for a in solver.wrists.arms]
        for _ in range(100):
            before = solver.q.copy()
            solver.solve(pose(.4, -.3), 1/60)
            self.assertLessEqual(max(abs(solver.q-before)), .025001)
        np.testing.assert_allclose(solver.q[indices], [.4, .3], atol=1e-5)
        p = pose(.4, -.3)
        rot = rotation_xyzw(axis_quat([0, 1, 0], .7))
        p[:, :3] = p[:, :3] @ rot.T + [1., 0., 2.]
        p[:, 3:] = yaw_quat(p[:, 3:], .7)
        for _ in range(80):
            solver.solve(p, 1/60)
        np.testing.assert_allclose(solver.q[indices], [.4, .3], atol=1e-5)
        p[[25, 30, 40, 51, 56, 66], :3] = 0.
        held = solver.q[indices].copy()
        for _ in range(10):
            solver.solve(p, 1/60)
        np.testing.assert_equal(solver.q[indices], held)
        for _ in range(100):
            solver.solve(pose(1.4, -1.4), 1/60)
        np.testing.assert_allclose(solver.q[indices], [1.0472, 1.0472], atol=1e-5)
        solver.wrists.reset()
        for _ in range(100):
            solver.solve(pose(.2, -.2), 1/60)
        np.testing.assert_allclose(solver.q[indices], [0., 0.], atol=1e-5)

    def test_invalid_zero_offsets(self):
        for offsets in ((float('nan'), 0.), (151., 0.), (0.,), (0., float('inf'))):
            with self.subTest(offsets=offsets), self.assertRaises(ValueError):
                Retargeter(self.config, wrist_offsets_deg=offsets)

    def test_wrist_global_yaw_invariance_and_quaternion_sign(self):
        a, b = Retargeter(self.config), Retargeter(self.config)
        c, t = np.cos(.8), np.sin(.8)
        rotation = np.array([[c, 0, t], [0, 1, 0], [-t, 0, c]])
        for frame in range(100):
            p = sample()
            if frame > 10:
                p[19, 3:] = axis_quat([0, 1, 0], .4)
            rotated = p.copy()
            rotated[:, :3] = p[:, :3] @ rotation.T + [2, 0, -1]
            rotated[:, 3:] = -yaw_quat(p[:, 3:], .8)
            a.solve(p, 1/60)
            b.solve(rotated, 1/60)
        np.testing.assert_allclose(a.q, b.q, atol=1e-7)

    def test_invalid_wrist_holds_and_extreme_inputs_stay_limited(self):
        solver = Retargeter(self.config)
        p = sample()
        solver.solve(p, 1/60)
        p[19, 3:] = axis_quat([0, 1, 0], 2.8)
        head = np.r_[np.zeros(3), axis_quat([1, 0, 0], 1.4)]
        for _ in range(100):
            before = solver.q.copy()
            solver.solve(p, 1/60, head)
            self.assertLessEqual(max(abs(solver.q-before)), .025001)
            limits = solver.model.jnt_range[solver.ids]
            self.assertTrue(np.all(solver.q >= limits[:, 0]))
            self.assertTrue(np.all(solver.q <= limits[:, 1]))
        wrist_ids = solver.wrists.arms[0]['indices']
        held = solver.q[wrist_ids].copy()
        p[19, 3:] = 0.
        for _ in range(5):
            solver.solve(p, 1/60, head)
        np.testing.assert_equal(solver.q[wrist_ids], held)

    def test_new_actuators_track_with_model_dynamics(self):
        solver = Retargeter(self.config)
        model, data = solver.model, solver.data
        target = solver.q.copy()
        for i, wrist in enumerate(solver.wrists.arms):
            target[wrist['indices']] = .4*(1 if i == 0 else -1)
        target[solver.head_pitch] = .35
        target[[25, 26]] = [.5, -.4]
        kp = np.array(solver.joints['default_stiffness'])
        kd = np.array(solver.joints['default_damping'])
        actuators = [int(np.flatnonzero(model.actuator_trnid[:, 0] == j)[0]) for j in solver.ids]
        command = solver.q.copy()
        for _ in range(int(3/model.opt.timestep)):
            step = 1.5*model.opt.timestep
            command += np.clip(target-command, -step, step)
            data.ctrl[actuators] = kp*(command-data.qpos[solver.qadr])-kd*data.qvel[solver.dadr]
            mujoco.mj_step(model, data)
        self.assertLess(np.max(abs(data.qpos[solver.qadr[[17, 22, 24, 25, 26]]]-target[[17, 22, 24, 25, 26]])), .04)
        self.assertLess(np.max(abs(data.qvel[solver.dadr[[17, 22, 24, 25, 26]]])), .01)


if __name__ == '__main__':
    unittest.main()
