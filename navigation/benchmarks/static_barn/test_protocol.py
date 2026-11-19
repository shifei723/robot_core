import unittest,math
from protocol import goal_in_odom,score,motion_started,outcome
class ProtocolTests(unittest.TestCase):
    def test_spawn_rotated_odom(self):
        x,y,t=goal_in_odom((-2.25,3.,math.pi/2),(0.,0.,0.))
        self.assertAlmostEqual(x,10.);self.assertAlmostEqual(y,0.);self.assertAlmostEqual(t,0.)
    def test_world_aligned_odom(self):
        x,y,t=goal_in_odom((-2.25,3.,math.pi/2),(-2.25,3.,math.pi/2))
        self.assertAlmostEqual(x,-2.25);self.assertAlmostEqual(y,13.)
    def test_score_and_failure(self):
        self.assertEqual(score(False,None,10),0)
        self.assertEqual(score(True,0,10),.5)
        self.assertEqual(score(True,20,10),.25)
        self.assertEqual(score(True,100,10),.125)
    def test_invalid_time(self):
        with self.assertRaises(ValueError):score(True,-1,10)
    def test_moving_clock_and_reference_priority(self):
        self.assertFalse(motion_started(-2.25,3.09))
        self.assertTrue(motion_started(-2.25,3.11))
        self.assertEqual(outcome(99.9,1.,False),'success')
        self.assertIsNone(outcome(1.,1.00001,False))
        self.assertEqual(outcome(100.,.5,False),'timeout')
        self.assertEqual(outcome(100.,.5,True),'collision')

if __name__=='__main__':unittest.main()
