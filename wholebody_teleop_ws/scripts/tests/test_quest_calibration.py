"""Offline calibration tests using the real PM01 model and synthetic matched users."""
from copy import deepcopy
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import mujoco
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter
from quest_body_profile import BodyProfile
from quest_calibration import (CalibrationSession, POSES, atomic_yaml, demo_pose,
                               fit_profile, relative_targets, robot_points)
from quest_tracking import body_features
from test_quest_retarget import sample


class CalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = tempfile.TemporaryDirectory()
        cls.scripts = Path(__file__).resolve().parents[1]
        subprocess.run([sys.executable, str(cls.scripts/'prepare_quest_teleop.py'), cls.runtime.name], check=True)
        cls.config = str(Path(cls.runtime.name)/'config.yaml')

    @classmethod
    def tearDownClass(cls):
        cls.runtime.cleanup()

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'person.yaml'
        self.solver = Retargeter(self.config)
        self.robot_ref = robot_points(self.solver, demo_pose(self.solver, 0))[:, 0, 2]

    def person(self, points, height=1.4):
        """Known inverse mapping: upper scale .6, lower 1.1, small fitted offsets."""
        target = relative_targets(points, self.robot_ref)
        target[:, :, 2] -= height-1.4
        upper = (target[:, 0]-[.005, 0, -.005])/.6
        lower = (target[:, 1]-target[:, 0]-[0, .003, .004])/1.1
        p = sample()
        p[:, 1] += height-1.4
        for i, (shoulder, elbow, hand) in enumerate(((10, 11, 19), (15, 16, 45))):
            def unity(v):
                return np.array([-v[1], v[2], v[0]])
            p[elbow, :3] = p[shoulder, :3]+unity(upper[i])
            p[hand, :3] = p[elbow, :3]+unity(lower[i])
        return p

    def records(self):
        records = []
        for i in range(len(POSES)):
            robot = robot_points(self.solver, demo_pose(self.solver, i))
            _, segments, heights, _ = body_features(self.person(robot), True)
            records.append(dict(segments=segments, heights=heights, robot=robot))
        return records

    def test_fitting_recovers_mapping(self):
        cfg, report = fit_profile(self.solver, self.records())
        atomic_yaml(self.path, cfg)
        profile = BodyProfile.load(self.path)
        self.assertLess(report['max_error_cm'], 1e-8)
        for i, arm in enumerate(profile.arms):
            upper, lower = self.solver.arms[i][3:5]
            self.assertAlmostEqual(upper/arm.upper_arm_m*arm.upper_gain, .6)
            self.assertAlmostEqual(lower/arm.forearm_m*arm.forearm_gain, 1.1)
            np.testing.assert_allclose(arm.elbow_offset_m, [.005, 0, -.005], atol=1e-10)
            np.testing.assert_allclose(arm.hand_offset_m, [0, .003, .004], atol=1e-10)

    def test_large_error_is_saved_and_loaded_without_acceptance_gate(self):
        records = self.records()
        records[2]['robot'][:, 2, 0] += 1.
        cfg, report = fit_profile(self.solver, records)
        self.assertGreater(report['max_error_cm'], 5.)
        self.assertNotIn('validation_error_cm', report)
        atomic_yaml(self.path, cfg)
        profile = BodyProfile.load(self.path)
        self.assertEqual(profile.calibration['fit_mode'], 'sampled')

    def test_repeated_pose_is_used_but_nonfinite_data_is_rejected(self):
        records = self.records()
        for record in records:
            record['segments'] = records[0]['segments'].copy()
        cfg, report = fit_profile(self.solver, records)
        atomic_yaml(self.path, cfg)
        profile = BodyProfile.load(self.path)
        self.assertEqual(profile.arms[0].upper_gain, 0.)
        self.assertGreater(report['max_error_cm'], 5.)
        records[1]['segments'][0, 0, 0] = float('nan')
        with self.assertRaises(ValueError):
            fit_profile(self.solver, records)

    def test_fixed_height_reference_preserves_crouch(self):
        cfg, _ = fit_profile(self.solver, self.records())
        atomic_yaml(self.path, cfg)
        a = Retargeter(self.config, body_profile=self.path)
        b = Retargeter(self.config, body_profile=self.path)
        p = self.person(robot_points(a, demo_pose(a, 3)))
        crouched = p.copy()
        crouched[:, 1] -= .10
        a.solve(p, 1/60)
        b.solve(crouched, 1/60)
        np.testing.assert_allclose(b.filtered_positions[:, :, :2], a.filtered_positions[:, :, :2])
        np.testing.assert_allclose(b.filtered_positions[:, :, 2]-a.filtered_positions[:, :, 2], -.10)
        # Height target affects the actual IK solution with the correct absolute-Z Jacobian.
        for _ in range(100):
            before = b.q.copy()
            a.solve(p, 1/60)
            b.solve(crouched, 1/60)
            self.assertLessEqual(max(abs(b.q-before)), .025001)
        self.assertGreater(np.linalg.norm(a.q-b.q), .1)
        self.assertTrue(np.isfinite(b.position_errors_m).all())

    def test_state_machine_collects_then_saves_and_activates(self):
        session = CalibrationSession(self.solver, self.path, prepare_seconds=.1, sample_seconds=1.)
        now = 0.
        with patch('quest_calibration.logging.warning'):
            for _ in range(5000):
                now += .02
                feedback = self.solver.q.copy()
                points = robot_points(self.solver, demo_pose(self.solver, session.index))
                pose = self.person(points)
                session.step(now, .02, pose, feedback, now)
                if session.done or session.failure:
                    break
        self.assertIsNone(session.failure)
        self.assertTrue(session.done)
        self.assertTrue(self.path.exists())
        self.assertTrue(Path(str(self.path)+'.report.yaml').exists())
        self.assertIsNotNone(self.solver.profile.calibration)
        self.assertEqual(self.solver.hand_height_bias, 0.)
        self.assertEqual(len(session.records), 4)

    def test_stale_data_pauses_but_mismatched_moving_pose_is_sampled(self):
        session = CalibrationSession(self.solver, self.path, prepare_seconds=0., sample_seconds=1.)
        q = self.solver.q.copy()
        with patch('quest_calibration.logging.warning'):
            session.step(1., .02, sample(), q, 0.)
            np.testing.assert_equal(q, self.solver.q)
            session.step(2., .02, None, q, 2.)
            np.testing.assert_equal(q, self.solver.q)
            self.solver.q = session.goals[1].copy()
            session.index = 1
            for n in range(53):
                pose = sample()  # Arms down while robot points forward; deliberately mismatched.
                pose[19, 2] += .10 if n % 2 else -.10
                session.step(3+n*.02, .02, pose, self.solver.q.copy(), 3+n*.02)
            self.assertEqual(len(session.records), 1)
            session.samples = [(0, None, None, None)]
            session.step(20., .02, None, q, 20.)
            self.assertEqual(session.samples, [])
        self.assertFalse(self.path.exists())

    def test_motion_is_limited_and_output_never_silently_overwritten(self):
        self.path.write_text('existing personal data')
        with self.assertRaises(ValueError):
            CalibrationSession(self.solver, self.path)
        with self.assertRaises(FileExistsError):
            atomic_yaml(self.path, {'replacement': True})
        self.assertEqual(self.path.read_text(), 'existing personal data')
        session = CalibrationSession(self.solver, self.path, overwrite=True)
        session.index = 1
        pose = self.person(robot_points(self.solver, session.goals[1]))
        with patch('quest_calibration.logging.warning'):
            for n in range(20):
                before = self.solver.q.copy()
                session.step(n*.02, .02, pose, before, n*.02)
                self.assertLessEqual(max(abs(self.solver.q-before)), .012001)
                limits = self.solver.model.jnt_range[self.solver.ids]
                self.assertTrue(np.all(self.solver.q >= limits[:, 0]))
                self.assertTrue(np.all(self.solver.q <= limits[:, 1]))

    def test_demonstrations_settle_with_actual_model_pd_dynamics(self):
        solver = self.solver
        model, data = solver.model, solver.data
        kp = np.array(solver.joints['default_stiffness'])
        kd = np.array(solver.joints['default_damping'])
        actuators = [int(np.flatnonzero(model.actuator_trnid[:, 0] == joint)[0])
                     for joint in solver.ids]
        command = solver.q.copy()
        for i in range(len(POSES)):
            target = demo_pose(solver, i)
            for _ in range(int(5/model.opt.timestep)):
                step = .6*model.opt.timestep
                command += np.clip(target-command, -step, step)
                data.ctrl[actuators] = (kp*(command-data.qpos[solver.qadr]) -
                                       kd*data.qvel[solver.dadr])
                mujoco.mj_step(model, data)
            with self.subTest(pose=i):
                self.assertLess(np.max(abs(target-data.qpos[solver.qadr])), .12)
                self.assertLess(np.max(abs(data.qvel[solver.dadr])), .01)

    def test_cli_preflight_validates_calibration_options(self):
        command = [sys.executable, str(self.scripts/'quest_retarget.py'), '--config', self.config,
                   '--calibrate', str(self.path), '--check-profile']
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(self.path.exists())
        self.path.write_text('existing')
        result = subprocess.run(command, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('已存在', result.stderr)
        result = subprocess.run(command+['--overwrite-calibration'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


if __name__ == '__main__':
    unittest.main()
