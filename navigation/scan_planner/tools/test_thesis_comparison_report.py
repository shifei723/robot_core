"""Small synthetic fixtures test metric definitions, never used as experiment data."""
import json,tempfile,unittest
from pathlib import Path
from report_thesis_indoor_comparison import evaluate
class ReportMetrics(unittest.TestCase):
 def fixture(self,root,statuses,times,reached=False):
  rows=[dict(t=t,status=s,v=0.,pose=[0.,0.,0.]) for t,s in zip(times,statuses)]
  summary=dict(experiment_parameters={},reached=reached,collision=False,error=None,duration=times[-1],goal_sent_at=0.,min_clearance=.1,multi_onsets={})
  if reached:summary['arrival_time']=times[-1]
  (root/'summary.json').write_text(json.dumps(summary));(root/'trace.json').write_text(json.dumps({'poses':rows,'decisions':[]}))
 def test_timeout_is_not_arrival_and_long_wait_is_integrated(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);self.fixture(p,['TRACKING','DYNAMIC_WAIT','WAIT_TRACK','GOAL_REACHED'],[0,5,17,20]);r=evaluate(p)
   self.assertIsNone(r['arrival_s']);self.assertIsNone(r['task_s'])
   self.assertEqual(r['dynamic_wait_s'],12);self.assertEqual(r['sensor_wait_s'],3)
   self.assertEqual(r['long_wait_episodes'],1);self.assertEqual(r['longest_dynamic_stop_s'],12)
 def test_separate_stops_are_not_a_freezing_episode(self):
  with tempfile.TemporaryDirectory() as d:
   p=Path(d);self.fixture(p,['DYNAMIC_WAIT','TRACKING','DYNAMIC_WAIT','GOAL_REACHED'],[0,6,7,13],True);r=evaluate(p)
   self.assertEqual(r['dynamic_wait_s'],12);self.assertEqual(r['longest_dynamic_stop_s'],6)
   self.assertEqual(r['long_wait_episodes'],0);self.assertEqual(r['arrival_s'],13)
if __name__=='__main__':unittest.main()
