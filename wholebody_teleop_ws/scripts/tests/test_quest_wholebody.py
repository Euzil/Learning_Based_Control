import sys
from pathlib import Path
import tempfile
import subprocess
import unittest
import numpy as np
import mujoco
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from quest_wholebody import WholeBodyPlanner, HandTargetMapper, parse_controller
from quest_policy import QuestPolicy
from quest_retarget import Retargeter
from test_quest_retarget import sample


class WholeBodyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        root = Path(__file__).resolve().parents[2]
        subprocess.run([sys.executable,str(root/'scripts/prepare_quest_teleop.py'),cls.tmp.name,
                        '--wbc-model',str(root/'models/pm01_wbc/20260721_211647.onnx')],check=True)
        cls.config=Path(cls.tmp.name)/'config.yaml'
        cfg=yaml.safe_load(cls.config.read_text())
        cls.policy=QuestPolicy(cfg['robots'][0]['joints'])
        cls.model=mujoco.MjModel.from_xml_path(cfg['ik_scene'])

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def setUp(self):
        self.policy.reset()
        self.planner=WholeBodyPlanner(self.model,self.policy)
        self.q=self.policy.robot_default.copy()
        self.ref=self.q[self.policy.upper_ids]
        self.x=np.r_[.8,0,0,self.q[self.planner.indices]]
        self.planner._set(self.x,self.q)
        self.neutral=self.planner._hand_points().copy()

    def solve(self, target, times=4):
        for _ in range(times):
            result=self.planner.solve(target,self.q,[0,0,0,1],.8,self.ref,.8)
        return result

    def test_forward_low_side_and_high_use_same_solver(self):
        forward=self.solve(self.neutral+[.3,0,0])
        self.assertGreater(forward['posture'][2], .05)
        self.planner.reset()
        low=self.solve(self.neutral+[.2,0,-.3])
        self.assertLess(low['posture'][0], .65)
        self.assertGreater(low['posture'][2], .1)
        self.assertLess(max(low['hand_error']), .025)
        self.assertLess(max(low['foot_error']), .015)
        self.planner.reset()
        side=self.solve(self.neutral+[0,.25,0])
        self.assertGreater(abs(side['posture'][1]), .04)
        self.planner.reset()
        high=self.solve(self.neutral+[.1,0,.35])
        self.assertGreater(high['posture'][0], .75)
        self.assertLess(max(high['hand_error']), .04)

    def test_whole_body_reduces_error_beyond_fixed_body_reach(self):
        target=self.neutral+[.2,0,-.3]
        full=self.solve(target)
        self.planner.reset()
        self.planner.lower[:3]=[.8,0,0]
        self.planner.upper[:3]=[.8,0,0]
        arms=self.solve(target)
        self.assertLess(np.mean(full['hand_error']), .5*np.mean(arms['hand_error']))

    def test_returns_toward_upright_when_hands_return(self):
        self.solve(self.neutral+[.2,0,-.3])
        restored=self.solve(self.neutral,times=8)
        self.assertGreater(restored['posture'][0], .78)
        self.assertLess(np.linalg.norm(restored['posture'][1:]), .03)

    def test_analytic_hand_jacobian_matches_numerical_kinematics(self):
        p=self.planner;x=self.x.copy();x[:3]=[.7,.1,.2]
        p._set(x,self.q);point=p._hand_points()[0]
        analytic,_=p._jac(point,p.hands[0],x[1]);numeric=[]
        for j in range(len(x)):
            xx=x.copy();xx[j]+=1e-6;p._set(xx,self.q)
            numeric.append((p._hand_points()[0]-point)/1e-6)
        np.testing.assert_allclose(analytic,np.array(numeric).T,atol=1e-5)

    def test_unreachable_goals_remain_finite_and_bounded(self):
        out=self.solve(self.neutral+[2,0,-1])
        self.assertTrue(np.isfinite(out['upper']).all())
        self.assertGreater(np.mean(out['hand_error']), .1)
        self.assertTrue(np.all(self.planner.previous >= self.planner.lower))
        self.assertTrue(np.all(self.planner.previous <= self.planner.upper))

    def test_body_pose_enters_correct_wbc_observation_slots(self):
        p=self.policy
        p.step(self.q,np.zeros(len(self.q)),[0,0,0,1],[0,0,0],[0,0,0],self.ref,
               height=.6,body_tilt=[-.15,.25])
        self.assertEqual(p.loco_command[4],0.)
        self.assertGreater(p.loco_command[5],0.)
        self.assertLess(p.loco_command[6],0.)
        self.assertAlmostEqual(p.loco_command[3],.796)
        self.assertAlmostEqual(p.loco_command[5],.01)

    def test_height_feedback_corrects_bias_and_respects_model_bounds(self):
        self.assertLess(self.planner.height_command(.55,.65),.55)
        self.assertGreater(self.planner.height_command(.7,.65),.7)
        self.assertEqual(self.planner.height_command(.35,.8),.35)
        self.assertEqual(self.planner.height_command(.8,.35),.8)

    def test_controller_packet_removes_exact_offset_and_requires_metadata(self):
        line='Left controller_wrist | locomotion = 2 -.1 3 0:, 2.2, .9, 3.4, 0,0,0,1'
        side,point=parse_controller(line)
        self.assertEqual(side,0)
        np.testing.assert_allclose(point,[.2,1.,.4])
        self.assertIsNone(parse_controller('Left controller_wrist:, 1,2,3,0,0,0,1'))
        self.assertIsNone(parse_controller(line.replace('2.2','nan')))

    def test_fixed_mapper_keeps_hand_displacement_instead_of_normalizing(self):
        retargeter=Retargeter(self.config)
        mapper=HandTargetMapper(retargeter);poses=sample();mapper.initialize(poses)
        start=mapper.map(0,poses[19,:3],.02)
        end_position=poses[19,:3]+[0,-.2,.15]
        for _ in range(50):target=mapper.map(0,end_position,.05)
        np.testing.assert_allclose(target-start,
             mapper.scales[0]*(mapper.basis @ [0,-.2,.15]),atol=1e-8)
        old_basis=mapper.basis.copy();poses[5,2]+=.15;mapper.initialize(poses)
        np.testing.assert_equal(mapper.basis,old_basis)
        mapper.reset();self.assertIsNone(mapper.map(0,end_position,.02))


if __name__=='__main__':
    unittest.main()
