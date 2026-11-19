"""Exercise the real ROS2 static evaluator; action completion is not truth arrival."""
from pathlib import Path
from types import SimpleNamespace as NS
import sys,argparse,json
import pytest,rclpy
from rclpy.time import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu,LaserScan
from geometry_msgs.msg import Transform
sys.path.insert(0,str(Path(__file__).resolve().parent))
from evaluate import Evaluation

@pytest.mark.parametrize('ending,status',[('arrive','success'),('contact','collision'),('deadline','timeout')])
@pytest.mark.parametrize('method',['slash','mppi'])
def test_real_terminal_rules(tmp_path,ending,status,method):
 rclpy.init();a=argparse.Namespace(method=method,world=0,trial=0,timeout=100.,goal_tolerance=1.,output=str(tmp_path),launch_pid=None)
 n=Evaluation(a);clock=[5.]
 n.get_clock=lambda:NS(now=lambda:Time(seconds=clock[0]))
 try:
  n.truth=Transform();n.truth.translation.x=-2.25;n.truth.translation.y=3.;n.truth.rotation.w=1.
  n.odom=Odometry();n.odom.pose.pose.orientation.w=1.;n.imu=Imu();n.scan=LaserScan()
  for z in [n.odom,n.imu,n.scan]:z.header.stamp.sec=5
  goals=[];n.goal_pub=NS(get_subscription_count=lambda:1,publish=lambda z:goals.append(z))
  handle=NS(accepted=True,get_result_async=lambda:NS(done=lambda:True,result=lambda:NS(status=4)))
  n.action=NS(server_is_ready=lambda:True,send_goal_async=lambda request:NS(done=lambda:True,result=lambda:handle))
  n.mppi_ready=lambda:True
  n.tick();assert n.goal_sent==5. and n.start is None and n.status is None
  n.tick();assert n.status is None,'Action success outside truth gate must not terminate'
  n.truth.translation.y=3.11;clock[0]=6.;n.motion_watch();n.tick();assert n.start==6.
  n.truth.translation.y=12.;clock[0]=106. if ending=='deadline' else 7.
  n.collided=ending=='contact';n.tick();assert n.status==status
  d=json.loads((tmp_path/f'{method}_world_0_trial_0.json').read_text());assert d['status']==status
  assert d['startup_delay_s']==1. and d['goal_sent_sim_s']==5. and d['truth_navigation_input'] is False
  assert d['navigation_goal_odom']==pytest.approx([10.,0.])
 finally:n.destroy_node();rclpy.shutdown()
