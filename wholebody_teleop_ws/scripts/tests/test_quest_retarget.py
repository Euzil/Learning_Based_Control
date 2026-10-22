import sys
from pathlib import Path
import unittest
import tempfile
import subprocess
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter, parse_pose, parse_head




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


if __name__ == '__main__':
    unittest.main()
