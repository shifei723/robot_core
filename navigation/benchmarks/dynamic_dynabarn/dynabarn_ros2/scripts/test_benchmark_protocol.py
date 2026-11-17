import math,sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from benchmark_protocol import PROFILES,outcome,motion_started

@pytest.mark.parametrize('elapsed,distance,collision,expected',[
 (1.,1.,False,'success'),(1.,1.00001,False,None),(99.99,.5,False,'success'),
 (100.,.5,False,'timeout'),(100.1,2.,False,'timeout'),(1.,.5,True,'collision'),
 (100.,.5,True,'collision'),(10.,2.,False,None),
])
def test_reference_boundary_and_priority(elapsed,distance,collision,expected):
 assert outcome(PROFILES['challenge'],elapsed,distance,collision)==expected

def test_motion_clock_uses_truth_displacement():
 p=PROFILES['challenge'];assert not motion_started(p,11.,0.)
 assert not motion_started(p,10.91,0.)
 assert motion_started(p,10.89,0.)
 assert motion_started(p,11.,.1)

@pytest.mark.parametrize('anchor',[(0.,0.,0.),(2.,3.,.7),(-1.,4.,-2.)])
def test_goal_and_static_map_anchor_have_same_transform(anchor):
 p=PROFILES['challenge'];gx,gy=p.navigation_goal(anchor);angle=anchor[2]-p.start_yaw
 dx=gx-anchor[0];dy=gy-anchor[1]
 assert math.cos(angle)*dx+math.sin(angle)*dy+p.start_x==pytest.approx(-9.)
 assert -math.sin(angle)*dx+math.cos(angle)*dy==pytest.approx(0.,abs=1e-12)

def test_legacy_is_explicit_and_unchanged():
 p=PROFILES['legacy'];assert (p.start_x,p.radius,p.timeout,p.timing)==(12.,.3,120.,'goal_publication')
 assert p.navigation_goal((0.,0.,0.))==pytest.approx((21.,0.),abs=1e-12)

def test_mppi_velocity_feedback_matches_shared_fused_odometry():
 import yaml
 cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'comparison_config/nav2.yaml').read_text())
 for node in ['controller_server','bt_navigator','velocity_smoother']:
  assert '/'+cfg[node]['ros__parameters']['odom_topic'].lstrip('/')=='/platform/odom/filtered'


def test_mppi_smoother_clamps_both_signs_of_shared_actuator_limits():
 import yaml
 cfg=yaml.safe_load((Path(__file__).resolve().parents[1]/'comparison_config/nav2.yaml').read_text())['velocity_smoother']['ros__parameters']
 assert cfg['max_velocity']==[.5,0.,1.]
 assert cfg['min_velocity']==[-.35,0.,-1.]
