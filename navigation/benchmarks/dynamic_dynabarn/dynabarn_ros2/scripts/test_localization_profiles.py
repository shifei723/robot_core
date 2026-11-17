from pathlib import Path
import sys
import yaml
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from localization_profiles import prepare_setup,VELOCITY_IMU

def test_planar_fusion_has_independent_encoder_velocity_and_gyro():
 assert [i for i,v in enumerate(VELOCITY_IMU['odom0_config']) if v]==[6,7]
 assert [i for i,v in enumerate(VELOCITY_IMU['imu0_config']) if v]==[11]
 assert VELOCITY_IMU['imu0']=='sensors/imu_0/data'
 assert VELOCITY_IMU['odom0']=='platform/odom'
 assert len(VELOCITY_IMU['odom0_config'])==len(VELOCITY_IMU['imu0_config'])==15

def test_setup_is_isolated_and_preserves_nav2_and_sensors(tmp_path):
 base=tmp_path/'base';base.mkdir()
 source={'platform':{'attachments':[{'name':'front_fender'}]},'sensors':{'lidar2d':[{'model':'hokuyo_ust'}]}}
 (base/'robot.yaml').write_text(yaml.safe_dump(source));(base/'nav2.yaml').write_text('unchanged: true\n')
 prepared=prepare_setup(base,'velocity_imu')
 try:
  assert prepared!=base
  robot=yaml.safe_load((prepared/'robot.yaml').read_text())
  assert robot['platform']['extras']['ros_parameters']['ekf_node']==VELOCITY_IMU
  assert robot['sensors']==source['sensors']
  assert robot['platform']['attachments']==source['platform']['attachments']
  assert (prepared/'nav2.yaml').read_bytes()==(base/'nav2.yaml').read_bytes()
  assert yaml.safe_load((base/'robot.yaml').read_text())==source
  assert prepare_setup(base,'legacy')==base
  with pytest.raises(ValueError):prepare_setup(base,'ideal_truth')
 finally:
  import shutil
  shutil.rmtree(prepared)
