import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_locomotion import WorldReferenceController, parse_reference, remove_locomotion


class WorldControlTests(unittest.TestCase):
    def setUp(self):
        self.control = WorldReferenceController(.8, [.35, .8])
        self.position = np.array([3., 4., .8])

    def ref(self, seq, forward=0., height=0., move=1., turn=0., yaw=0., session='a'*32, heading=0.):
        return parse_reference(f'Locomotion reference:, {session}, {seq}, 0, {height}, {forward}, {yaw}, {heading}, {move}, {turn}')

    def test_release_discards_position_error_and_keeps_height(self):
        c = self.control
        c.accept(self.ref(1), self.position, 0., 0.)
        c.accept(self.ref(2, .1, -.2), self.position, 0., .02)
        cmd, h = c.command(self.position, 0., .02)
        self.assertGreater(cmd[0], 0)
        self.assertAlmostEqual(h, .6)
        c.accept(self.ref(3, .1, -.2, move=0), self.position, 0., .04)
        np.testing.assert_equal(c.command(self.position, 0., .04)[0], np.zeros(3))
        np.testing.assert_equal(c.goal[:2], self.position[:2])
        self.assertAlmostEqual(c.height, .6)
        c.accept(self.ref(4, 8., -.2), self.position, 0., .06)
        np.testing.assert_equal(c.command(self.position, 0., .06)[0], np.zeros(3))

    def test_stop_on_loss_and_no_catchup_after_reconnect(self):
        c = self.control
        c.accept(self.ref(1), self.position, 0, 0)
        c.accept(self.ref(2, .1, -.3), self.position, 0, .02)
        np.testing.assert_equal(c.command(self.position, 0, .4)[0], np.zeros(3))
        c.accept(self.ref(3, 10., -.3), self.position, 0, .5)
        np.testing.assert_equal(c.command(self.position, 0, .5)[0], np.zeros(3))
        self.assertAlmostEqual(c.height, .5)

    def test_frame_rotation_and_height_bounds(self):
        c = self.control
        c.accept(self.ref(1), self.position, math.pi/2, 0)
        c.accept(self.ref(2, .1, -2.), self.position, math.pi/2, .02)
        cmd, h = c.command(self.position, math.pi/2, .02)
        self.assertGreater(c.goal[1], self.position[1])
        self.assertAlmostEqual(cmd[1], 0.)
        self.assertGreater(cmd[0], 0.)
        self.assertEqual(h, .35)

    def test_right_turn_and_release(self):
        c = self.control
        c.accept(self.ref(1, move=0, turn=1), self.position, 0, 0)
        c.accept(self.ref(2, move=0, turn=1, yaw=.1), self.position, 0, .02)
        self.assertLess(c.command(self.position, 0, .02)[0][2], 0)
        c.accept(self.ref(3, move=0, yaw=.1), self.position, 0, .04)
        self.assertEqual(c.command(self.position, 0, .04)[0][2], 0)

    def test_duplicate_old_sessions_and_reanchor(self):
        c = self.control
        c.accept(self.ref(1), self.position, 0, 0)
        self.assertFalse(c.accept(self.ref(1, 10), self.position, 0, .02))
        self.assertTrue(c.accept(self.ref(1, 10, session='b'*32), self.position, 0, .03))
        self.assertFalse(c.accept(self.ref(2, 20), self.position, 0, .04))
        np.testing.assert_equal(c.goal[:2], self.position[:2])

    def test_goal_lead_is_bounded(self):
        c = self.control
        c.accept(self.ref(1), self.position, 0, 0)
        c.accept(self.ref(2, 100), self.position, 0, .02)
        cmd, _ = c.command(self.position, 0, .02)
        self.assertLessEqual(np.linalg.norm(c.goal[:2]-self.position[:2]), .250001)
        self.assertLessEqual(np.linalg.norm(cmd[:2]), .375001)

    def test_frame_metadata_undoes_offset_without_double_motion(self):
        angle = .7
        original = np.array([[.2, 1.3, .4, 0, 0, 0, 1.]])
        shifted = original.copy()
        shifted[:, :3] += [3, -.2, 4]
        shifted[:, 3:] = [0, math.sin(angle/2), 0, math.cos(angle/2)]
        result = remove_locomotion(f'Body joints_pose | f=1 | locomotion = 3 -.2 4 {angle}', shifted)
        np.testing.assert_allclose(result, original, atol=1e-12)
        self.assertIsNone(remove_locomotion('Head pose | locomotion = nan 0 0 0', shifted))

    def test_backward_and_reversal_clear_unfinished_forward_goal(self):
        c = self.control
        c.accept(self.ref(1), self.position, 0, 0)
        c.accept(self.ref(2, .2), self.position, 0, .02)
        self.assertGreater(c.command(self.position, 0, .02)[0][0], 0)
        c.accept(self.ref(3, .18), self.position, 0, .04)
        self.assertLess(c.command(self.position, 0, .04)[0][0], 0)
        c.accept(self.ref(4, .16), self.position, 0, .06)
        self.assertLess(c.command(self.position, 0, .06)[0][0], 0)

    def test_height_reversal_after_saturation_has_no_dead_travel(self):
        c = self.control
        c.accept(self.ref(1, height=0, move=0), self.position, 0, 0)
        c.accept(self.ref(2, height=.4, move=0), self.position, 0, .02)
        self.assertEqual(c.height, .8)
        c.accept(self.ref(3, height=.39, move=0), self.position, 0, .04)
        self.assertAlmostEqual(c.height, .79)
        c.accept(self.ref(4, height=-.6, move=0), self.position, 0, .06)
        self.assertEqual(c.height, .35)
        c.accept(self.ref(5, height=-.59, move=0), self.position, 0, .08)
        self.assertAlmostEqual(c.height, .36)

    def test_height_ignores_unreceived_motion_during_outage(self):
        c = self.control
        c.accept(self.ref(1, height=-.1, move=0), self.position, 0, 0)
        c.command(self.position, 0, .4)
        c.accept(self.ref(2, height=-.5, move=0), self.position, 0, .5)
        self.assertAlmostEqual(c.height, .7)

    def test_large_turn_can_reach_model_turn_rate_without_planar_motion(self):
        c = self.control
        c.accept(self.ref(1, move=0, turn=1), self.position, 0, 0)
        c.accept(self.ref(2, move=0, turn=1, yaw=2), self.position, 0, .02)
        cmd, _ = c.command(self.position, 0, .02)
        np.testing.assert_allclose(cmd, [0., 0., -1.2])
        c.accept(self.ref(3, move=0, turn=0, yaw=2), self.position, -.3, .04)
        np.testing.assert_array_equal(c.command(self.position, -.3, .04)[0], np.zeros(3))

    def test_malformed_packet(self):
        self.assertIsNone(parse_reference('Locomotion reference:, a, 1, 0'))
        self.assertIsNone(self.ref(1, forward=float('nan')))
        self.assertIsNone(self.ref(1, move=2))


if __name__ == '__main__':
    unittest.main()
