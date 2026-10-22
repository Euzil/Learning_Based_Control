import sys
from pathlib import Path
import unittest
import tempfile
import subprocess
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_retarget import Retargeter, parse_pose




def sample():
    p = np.zeros((70, 7))
    p[:, 6] = 1.
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
        p[7, 3:] = [0, np.sin(-.2), 0, np.cos(-.2)]
        for _ in range(150):
            prev = s.q.copy()
            s.solve(p, 1/60)
            self.assertLessEqual(np.max(np.abs(s.q-prev)), .025001)
        self.assertGreater(np.linalg.norm(s.q[13:17]-initial[13:17]), .5)
        np.testing.assert_allclose(s.q[18:23], initial[18:23], atol=.05)
        self.assertAlmostEqual(s.q[s.head], .4, places=5)
        np.testing.assert_equal(s.q[:13], initial[:13])
        self.assertTrue(np.all(s.q >= s.model.jnt_range[s.ids, 0]))
        self.assertTrue(np.all(s.q <= s.model.jnt_range[s.ids, 1]))
        _, _, bodies, _, _ = s.arms[0]
        delta = s.data.xpos[bodies[2]] - s.data.xpos[bodies[0]]
        self.assertGreater(delta[0], .25)
        self.assertLess(abs(delta[2]), .1)

    def test_invalid_tracking_does_not_modify_target(self):
        s = Retargeter(self.config)
        before = s.q.copy()
        with self.assertRaises(ValueError):
            s.solve(np.zeros((70, 7)), 1/60)
        np.testing.assert_equal(s.q, before)


if __name__ == '__main__':
    unittest.main()
