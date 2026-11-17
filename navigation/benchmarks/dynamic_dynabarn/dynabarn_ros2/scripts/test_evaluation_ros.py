"""Exercise the real evaluator: planner completion must not override truth."""
import sys,time,json,argparse
from pathlib import Path
import pytest,rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,Transform
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from scan_planner_msgs.msg import DynamicObstacleArray
sys.path.insert(0,str(Path(__file__).resolve().parent))
from compare_evaluate import Comparison

@pytest.mark.parametrize('method',['frs_soft','frs_joint'])
@pytest.mark.parametrize('ending,expected',[('arrive','success'),('touch','collision'),('deadline','timeout')])
def test_real_evaluator_timing_and_terminal_rules(tmp_path,ending,expected,method):
 rclpy.init();fixture=Node('evaluation_fixture');goals=[]
 fixture.create_subscription(PoseStamped,'/move_base_simple/goal',lambda m:goals.append(m),10)
 a=argparse.Namespace(protocol='challenge',method=method,mode=method,output=str(tmp_path/'run'),timeout=100.,motion_phase=0.,active_escape='false',local_dynamic_planner='false',joint_rollout='false')
 n=Comparison(a);clock=[8.];n.now_s=lambda:clock[0]
 try:
  n.truth=Transform();n.truth.translation.x=11.;n.truth.rotation.w=1.;n.odom=Odometry();n.scan=LaserScan()
  n.anchor=[0.,0.,0.];n.objects={str(i):[0.,0.,0.] for i in range(8)};n.tracks=DynamicObstacleArray();n.tracks.healthy=True;n.tracks.header.stamp.sec=8;n.prior_time=8.
  deadline=time.monotonic()+2.
  while n.goal_pub.get_subscription_count()==0 and time.monotonic()<deadline:time.sleep(.01)
  assert n.goal_pub.get_subscription_count()>0
  n.tick();assert n.goal_sent==8. and n.start is None and not n.done
  n.state='GOAL_REACHED';n.planner_state='WAIT_TARGET';clock[0]=8.5;n.tick()
  assert not n.done,'Controller completion outside true goal must remain diagnostic'
  n.truth.translation.x=10.89;clock[0]=9.;n.motion_watch();n.tick();assert n.start==9.
  n.truth.translation.x=-7.99;clock[0]=108.9;n.tick();assert not n.done
  n.truth.translation.x=-8.;clock[0]=109. if ending=='deadline' else 10.
  if ending=='touch':n.touched=True
  n.tick();assert n.done
  d=json.loads((tmp_path/'run/result.json').read_text());assert d['status']==expected
  assert d['goal_tolerance_m']==1. and d['start_world'][:2]==[11.,0.]
  assert d['motion_start_sim_s']==9. and d['goal_sent_sim_s']==8.
  assert d['controller_completion_diagnostic'] is not None
  assert d['method_label']==method and d['joint_rollout_enabled']==(method=='frs_joint')
 finally:n.destroy_node();fixture.destroy_node();rclpy.shutdown()

def test_actual_graph_rejects_raw_wheel_feedback_and_accepts_fused(tmp_path):
 rclpy.init();controller=Node('controller_server')
 raw=controller.create_subscription(Odometry,'/platform/odom',lambda m:None,10)
 a=argparse.Namespace(protocol='challenge',method='mppi',mode='mppi',output=str(tmp_path/'run'),timeout=100.,motion_phase=0.,active_escape='false',local_dynamic_planner='false',joint_rollout='false')
 n=Comparison(a)
 try:
  deadline=time.monotonic()+2.
  while time.monotonic()<deadline:
   n.mppi_velocity_ready()
   if '/platform/odom' in n.mppi_velocity_topics:break
   time.sleep(.02)
  assert '/platform/odom' in n.mppi_velocity_topics and not n.mppi_velocity_ready()
  controller.destroy_subscription(raw)
  controller.create_subscription(Odometry,'/platform/odom/filtered',lambda m:None,10)
  deadline=time.monotonic()+2.
  while not n.mppi_velocity_ready() and time.monotonic()<deadline:time.sleep(.02)
  assert n.mppi_velocity_ready() and '/platform/odom' not in n.mppi_velocity_topics
 finally:n.destroy_node();controller.destroy_node();rclpy.shutdown()
