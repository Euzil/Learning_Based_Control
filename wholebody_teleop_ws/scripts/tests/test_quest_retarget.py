import sys
from pathlib import Path
import unittest
import tempfile
import subprocess
import numpy as np
import yaml
from copy import deepcopy
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter, parse_pose, parse_head
from quest_body_profile import BodyProfile




def yaw_quat(quats, angle):
    quats = np.asarray(quats)
    x,y,z,w = np.moveaxis(quats, -1, 0)
    c, t = np.cos(angle/2), np.sin(angle/2)
    return np.stack([c*x+t*z, c*y+t*w, c*z-t*x, c*w-t*y], axis=-1)


def sample():
    p = np.zeros((70, 7))
    p[:, 6] = 1.
    p[7, 3:] = [.5, -.5, -.5, .5]  # Meta head bone, not camera frame.
    p[1, :3] = [0, 1, 0]
    p[5, :3] = [0, 1.4, 0]
    p[10, :3] = [-.2, 1.4, 0]
    p[15, :3] = [.2, 1.4, 0]
    p[11, :3] = [-.2, 1.1, 0]
    p[16, :3] = [.2, 1.1, 0]
    p[19, :3] = [-.2, .85, 0]
    p[45, :3] = [.2, .85, 0]
    return p


class TrackingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = tempfile.TemporaryDirectory()
        subprocess.run([sys.executable, str(Path(__file__).resolve().parents[1] / "prepare_quest_teleop.py"), cls.runtime.name], check=True)
        cls.config = str(Path(cls.runtime.name) / "config.yaml")

    @classmethod
    def tearDownClass(cls):
        cls.runtime.cleanup()

    def test_parser_rejects_corruption(self):
        self.assertIsNone(parse_pose('Body joints_pose:, 1,2,nan'))
        self.assertIsNone(parse_pose('Body joints_pose:,' + ','.join(['nan']*490)))
        p = sample()
        np.testing.assert_equal(parse_pose('Body joints_pose:,' + ','.join(map(str,p.ravel()))),p)

    def test_arm_motion_limits_and_head_sign(self):
        s = Retargeter(self.config)
        p = sample()
        for _ in range(100):
            s.solve(p, 1/60)
        initial = s.q.copy()
        # Left arm straight forward; right arm stays down.
        p[11, :3] = [-.2, 1.4, .3]
        p[19, :3] = [-.2, 1.4, .55]
        # Look left (negative Unity yaw).
        head = np.array([0, 0, 0, 0, np.sin(-.2), 0, np.cos(-.2)])
        for _ in range(150):
            prev = s.q.copy()
            s.solve(p, 1/60, head_pose=head)
            self.assertLessEqual(np.max(np.abs(s.q-prev)), .025001)
        self.assertGreater(np.linalg.norm(s.q[13:17]-initial[13:17]), .5)
        np.testing.assert_allclose(s.q[18:23], initial[18:23], atol=.05)
        self.assertAlmostEqual(s.q[s.head], .4, places=5)
        np.testing.assert_equal(s.q[:13], initial[:13])
        self.assertTrue(np.all(s.q >= s.model.jnt_range[s.ids, 0]))
        self.assertTrue(np.all(s.q <= s.model.jnt_range[s.ids, 1]))
        _, _, bodies, _, _ = s.arms[0]
        delta = (s.data.xpos[bodies[2]] + s.data.xmat[bodies[2]].reshape(3,3) @ np.array([.03,0,-.14]) - s.data.xpos[bodies[0]])
        self.assertGreater(delta[0], .25)
        self.assertLess(abs(delta[2]), .1)


    def test_bone_head_neutral_turn_and_vertical_gaze(self):
        s = Retargeter(self.config)
        p = sample()
        for _ in range(30):
            s.solve(p, 1/60)
        self.assertAlmostEqual(s.q[s.head], 0., places=6)
        # Left yaw .4 radians applied to the neutral bone frame.
        p[7,3:] = yaw_quat(p[7,3:], -.4)
        for _ in range(100):
            s.solve(p, 1/60)
        self.assertAlmostEqual(s.q[s.head], .4, places=5)
        before = s.q[s.head]
        # Camera looking straight up must not inject an arbitrary yaw.
        head = np.array([0,0,0, -np.sqrt(.5),0,0,np.sqrt(.5)])
        for _ in range(20):
            s.solve(p, 1/60, head_pose=head)
        self.assertAlmostEqual(s.q[s.head], before, places=6)

    def test_scale_translation_and_global_yaw_invariance(self):
        a, b = Retargeter(self.config), Retargeter(self.config)
        p = sample()
        p[19,:3] = [-.2,1.1,.25]  # Bent left elbow.
        rotated = p.copy()
        c, t = np.cos(1.1), np.sin(1.1)
        rotation = np.array([[c,0,t],[0,1,0],[-t,0,c]])
        rotated[:,:3] = (p[:,:3] @ rotation.T)*1.7 + [2,3,-1]
        rotated[:,3:] = yaw_quat(p[:,3:], 1.1)
        for _ in range(120):
            a.solve(p, 1/60)
            b.solve(rotated, 1/60)
        np.testing.assert_allclose(a.q,b.q,atol=1e-6)
        self.assertLess(a.q[a.joints['names'].index('elbow_pitch_l_joint')], -.7)

    def test_debug_headers_and_head_parser(self):
        line = 'Body joints_pose | f = 5 | t = 123:,' + ','.join(map(str,sample().ravel()))
        np.testing.assert_equal(parse_pose(line), sample())
        np.testing.assert_equal(parse_head('Head pose | f = 6: ,0,0,0,0,0,0,1'), [0,0,0,0,0,0,1])
        self.assertIsNone(parse_head('Head pose:,0,0,0,0,0,0,0'))

    def test_height_bias_raises_both_robot_hands(self):
        baseline = Retargeter(self.config, hand_height_bias=0.)
        lifted = Retargeter(self.config, hand_height_bias=.15)
        p = sample()
        p[19,:3] = [-.2,1.1,.25]
        p[45,:3] = [.2,1.1,.25]
        for _ in range(120):
            baseline.solve(p, 1/60)
            lifted.solve(p, 1/60)
        for arm in range(2):
            def endpoint(solver):
                body = solver.arms[arm][2][2]
                return (solver.data.xpos[body] +
                        solver.data.xmat[body].reshape(3,3) @ np.array([.03,0,-.14]))
            rise = endpoint(lifted)[2]-endpoint(baseline)[2]
            self.assertGreater(rise, .015)
            self.assertLess(rise, .07)
        self.assertAlmostEqual(baseline.q[baseline.head], lifted.q[lifted.head])

    def test_invalid_tracking_does_not_modify_target(self):
        s = Retargeter(self.config)
        before = s.q.copy()
        with self.assertRaises(ValueError):
            s.solve(np.zeros((70, 7)), 1/60)
        np.testing.assert_equal(s.q, before)


class PersonalCalibrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runtime = tempfile.TemporaryDirectory()
        cls.scripts = Path(__file__).resolve().parents[1]
        subprocess.run([sys.executable, str(cls.scripts / 'prepare_quest_teleop.py'),
                        cls.runtime.name], check=True)
        cls.config = str(Path(cls.runtime.name) / 'config.yaml')
        cls.example = yaml.safe_load((cls.scripts.parent /
                                    'configs/quest_body_profile.example.yaml').read_text())

    @classmethod
    def tearDownClass(cls):
        cls.runtime.cleanup()

    def profile(self, cfg=None):
        path = Path(self.runtime.name) / 'person.yaml'
        path.write_text(yaml.safe_dump(self.example if cfg is None else cfg))
        return str(path)

    def test_segment_ratios_and_hand_offset(self):
        cfg = deepcopy(self.example)
        cfg['left'].update(upper_gain=1.1, forearm_gain=.9, hand_offset_m=[.01, 0, .02])
        profile = BodyProfile.load(self.profile(cfg))
        targets = profile.targets(0, np.array([.3, 0, 0]), np.array([0, 0, -.25]), (.2, .3))
        np.testing.assert_allclose(targets, [[.22, 0, 0], [.23, 0, -.25]], atol=1e-10)

    def test_reject_invalid_profiles(self):
        for field, value in [('upper_arm_m', 0), ('forearm_m', float('nan')),
                             ('upper_gain', 2), ('forearm_gain', True),
                             ('hand_offset_m', [0, 0]), ('robot_hand_local_m', [0, 0, float('inf')]),
                             ('foream_m', .25)]:
            cfg = deepcopy(self.example)
            cfg['left'][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                BodyProfile.load(self.profile(cfg))
        cfg = deepcopy(self.example)
        del cfg['right']
        with self.assertRaises(ValueError):
            BodyProfile.load(self.profile(cfg))

    def test_model_lengths_include_configured_endpoint(self):
        s = Retargeter(self.config, body_profile=self.profile())
        self.assertEqual(s.hand_height_bias, 0.)
        for i, (_, _, bodies, upper, forearm) in enumerate(s.arms):
            shoulder, elbow, hand = s.data.xpos[bodies].copy()
            hand += s.data.xmat[bodies[2]].reshape(3, 3) @ s.hand_local[i]
            self.assertAlmostEqual(upper, np.linalg.norm(elbow-shoulder))
            self.assertAlmostEqual(forearm, np.linalg.norm(hand-elbow))
            self.assertGreater(forearm, .25)

    def test_gains_change_motion_and_keep_other_arm_unchanged(self):
        a = Retargeter(self.config, body_profile=self.profile())
        cfg = deepcopy(self.example)
        cfg['left']['forearm_gain'] = .75
        b = Retargeter(self.config, body_profile=self.profile(cfg))
        p = sample()
        p[19, :3] = [-.2, 1.1, .25]
        for _ in range(120):
            for solver in (a, b):
                before = solver.q.copy()
                solver.solve(p, 1/60)
                self.assertLessEqual(np.max(abs(solver.q-before)), .025001)
                self.assertTrue(np.isfinite(solver.q).all())
                self.assertTrue(np.all(solver.q >= solver.model.jnt_range[solver.ids, 0]))
                self.assertTrue(np.all(solver.q <= solver.model.jnt_range[solver.ids, 1]))
        self.assertGreater(np.linalg.norm(a.q[a.arms[0][1]]-b.q[b.arms[0][1]]), .05)
        np.testing.assert_allclose(a.q[a.arms[1][1]], b.q[b.arms[1][1]], atol=1e-8)
        np.testing.assert_equal(a.q[:13], a.rest[:13])
        self.assertLess(a.position_errors_m[0, 1], .04)

    def test_yaw_translation_and_matching_human_scale(self):
        a = Retargeter(self.config, body_profile=self.profile())
        cfg = deepcopy(self.example)
        for side in ('left', 'right'):
            cfg[side]['upper_arm_m'] *= 1.2
            cfg[side]['forearm_m'] *= 1.2
        b = Retargeter(self.config, body_profile=self.profile(cfg))
        p = sample()
        p[19, :3] = [-.2, 1.1, .25]
        rotated = p.copy()
        c, t = np.cos(.7), np.sin(.7)
        rotation = np.array([[c, 0, t], [0, 1, 0], [-t, 0, c]])
        rotated[:, :3] = 1.2*(p[:, :3] @ rotation.T) + [2, 3, -1]
        rotated[:, 3:] = yaw_quat(p[:, 3:], .7)
        for _ in range(60):
            a.solve(p, 1/60)
            b.solve(rotated, 1/60)
        np.testing.assert_allclose(a.q, b.q, atol=1e-7)

    def test_invalid_frame_preserves_filters_and_command(self):
        s = Retargeter(self.config, body_profile=self.profile())
        s.solve(sample(), 1/60)
        q, targets = s.q.copy(), s.filtered_positions.copy()
        p = sample()
        p[45, :3] = p[16, :3]  # Second arm invalid after first arm was processed.
        with self.assertRaises(ValueError):
            s.solve(p, 1/60)
        np.testing.assert_equal(s.q, q)
        np.testing.assert_equal(s.filtered_positions, targets)
        for dt in (float('nan'), float('inf'), -1):
            with self.assertRaises(ValueError):
                s.solve(sample(), dt)
        for bias in (.6, 1., float('nan')):
            with self.assertRaises(ValueError):
                Retargeter(self.config, hand_height_bias=bias)

    def test_unreachable_target_is_bounded_and_reports_residual(self):
        cfg = deepcopy(self.example)
        for side in ('left', 'right'):
            cfg[side].update(upper_arm_m=.1, forearm_m=.1, upper_gain=1.5, forearm_gain=1.5)
        s = Retargeter(self.config, body_profile=self.profile(cfg))
        for _ in range(50):
            before = s.q.copy()
            s.solve(sample(), 1/60)
            self.assertLessEqual(np.max(abs(s.q-before)), .025001)
            self.assertTrue(np.isfinite(s.q).all())
            self.assertTrue(np.all(s.q >= s.model.jnt_range[s.ids, 0]))
            self.assertTrue(np.all(s.q <= s.model.jnt_range[s.ids, 1]))
        self.assertGreater(s.position_errors_m[0, 1], .5)

    def test_cli_profile_validation_without_ros(self):
        result = subprocess.run([sys.executable, str(self.scripts / 'quest_retarget.py'),
                                 '--config', self.config, '--body-profile', self.profile(),
                                 '--check-profile'], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn('effective scales', result.stderr)


if __name__ == '__main__':
    unittest.main()
