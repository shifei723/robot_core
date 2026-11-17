"""Reproducible native wheel/gyro fusion for the ROS2 benchmark plant.

Wheel-integrated pose shares its errors with wheel velocity. On skid steering,
feeding both pose/yaw and velocity as independent inputs makes turn slip persist
as a heading error. Fuse body planar velocity and native IMU yaw rate instead.
This is dead reckoning, not a truth pose or global localization source.
"""
from pathlib import Path
import shutil,tempfile
import yaml

VELOCITY_IMU={
 'odom0':'platform/odom','odom0_differential':False,'odom0_queue_size':10,
 'imu0':'sensors/imu_0/data','imu0_differential':False,'imu0_queue_size':10,
 'odom0_config':[False]*6+[True,True]+[False]*7,
 'imu0_config':[False]*11+[True,False,False,False],
}

def prepare_setup(base,profile):
 base=Path(base)
 if profile=='legacy':return base
 if profile!='velocity_imu':raise ValueError('Unknown localization profile: '+profile)
 setup=Path(tempfile.mkdtemp(prefix='dynabarn-velocity-imu-'))
 for filename in ('robot.yaml','nav2.yaml'):shutil.copy2(base/filename,setup/filename)
 robot=yaml.safe_load((setup/'robot.yaml').read_text())
 robot['platform'].setdefault('extras',{}).setdefault('ros_parameters',{})['ekf_node']=VELOCITY_IMU
 (setup/'robot.yaml').write_text(yaml.safe_dump(robot,sort_keys=False))
 return setup
