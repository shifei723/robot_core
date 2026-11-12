#!/usr/bin/env python3
"""Wall-clock integration checks, including a deliberately induced collision."""
import argparse,json,time
from pathlib import Path
import numpy as np
import rclpy
from evaluate import Eval
from ros_gz_interfaces.srv import ControlWorld,SetEntityPose

class Probe(Eval):
 def __init__(self,a):
  super().__init__(a);self.timer.cancel();self.object_stamps={};self.future=None
  from ament_index_python.packages import get_package_share_directory
  self.first={x['name']:np.array(x['keyframes'][0][1:]) for x in json.loads((Path(get_package_share_directory('dynabarn_ros2'))/'worlds/official_world_2.json').read_text())['objects']}
 def on_objects(self,m):
  super().on_objects(m)
  for t in m.transforms:self.object_stamps[t.child_frame_id.split('/')[-1]]=t.header.stamp.sec+t.header.stamp.nanosec/1e9
 def transition(self,state):self.phase=state;self.phase_wall=time.monotonic()
 def command(self,pause,reset=False):
  r=ControlWorld.Request();r.world_control.pause=pause;r.world_control.reset.time_only=reset
  self.future=self.control.call_async(r)
 def ack(self):
  if not self.future.done():return False
  if not self.future.result().success:raise RuntimeError('Gazebo service rejected request')
  return True
 def save(self,status):
  self.checks={k:(v.item() if isinstance(v,np.generic) else v) for k,v in self.checks.items()}
  super().save(status)
  path=self.dest/'result.json';r=json.loads(path.read_text())
  r['clock_delta_after_reset_s']=r.pop('sim_elapsed_s');r['sim_elapsed_s']=None
  r['purpose']='Pause/reset and deliberately induced moving-obstacle contact; collision=true is expected in this probe.'
  path.write_text(json.dumps(r,indent=2)+'\n')
 def tick(self):
  if self.done:return
  age=time.monotonic()-self.phase_wall
  if self.phase in ('waiting','moving'):
   if self.start is not None and self.now_s()-self.start>6:
    self.checks['eight_models_move']=len(self.samples)==8 and all(np.linalg.norm(np.ptp(v,axis=0))>.1 for v in self.samples.values())
    self.command(True);self.transition('pause_ack')
   else:super().tick()
  elif self.phase=='pause_ack' and self.ack():self.transition('drain')
  elif self.phase=='drain' and age>.4:
   self.frozen_clock=self.now_s();self.frozen={k:np.array(v) for k,v in self.objects.items()};self.transition('paused')
  elif self.phase=='paused' and age>.5:
   self.checks['paused_clock_frozen']=abs(self.now_s()-self.frozen_clock)<.005
   self.checks['paused_geometry_frozen']=max(np.linalg.norm(np.array(self.objects[k])-v) for k,v in self.frozen.items())<1e-8
   self.command(True);self.transition('contact_pause_ack')
  elif self.phase=='reset_ack' and self.ack():
   self.command(False);self.transition('unpause_ack')
  elif self.phase=='unpause_ack' and self.ack():self.transition('after_reset')
  elif self.phase=='after_reset' and self.now_s()<1.5 and len(self.object_stamps)==8 and all(t<1.5 for t in self.object_stamps.values()):
   error=max(np.linalg.norm(np.array(self.objects[k])-self.first[k]) for k in self.first)
   self.checks['reset_returns_to_start']=error<.5;self.checks['reset_error_m']=error
   a=np.sum(self.scan_checks,axis=0) if self.scan_checks else np.zeros(3)
   self.checks['lidar_tracks_geometry']=bool(a[0]>1000 and a[0]/max(1,a[1])>.9 and a[2]/max(1,a[0])<.05)
   keys=['eight_models_move','paused_clock_frozen','paused_geometry_frozen','reset_returns_to_start','moving_model_contact_detected','lidar_tracks_geometry']
   self.save('controls_pass' if all(self.checks.get(k,False) for k in keys) else 'controls_failed')
  elif self.phase=='contact_pause_ack' and self.ack():
   self.touched=False;self.transition('contact_drain')
  elif self.phase=='contact_drain' and age>.3:
   p=min(self.objects.values(),key=lambda p:abs(p[1]));r=SetEntityPose.Request();r.entity.name='robot';r.entity.type=r.entity.MODEL
   r.pose.position.x=p[0];r.pose.position.y=p[1];r.pose.position.z=.12;r.pose.orientation.w=1.
   self.future=self.set_pose.call_async(r);self.transition('teleport_ack')
  elif self.phase=='teleport_ack' and self.ack():self.command(False);self.transition('resume_contact_ack')
  elif self.phase=='resume_contact_ack' and self.ack():self.transition('contact')
  elif self.phase=='contact' and (self.touched or age>3):
   self.checks['moving_model_contact_detected']=self.touched
   self.command(True,True);self.transition('reset_ack')
  elif age>12 and self.phase not in ('waiting','moving'):self.save('control_service_timeout')

def main():
 p=argparse.ArgumentParser();p.add_argument('--output',required=True);p.add_argument('--mode',default='smoke');a,ros=p.parse_known_args();a.timeout=150
 rclpy.init(args=ros);n=Probe(a);last=0
 try:
  while rclpy.ok() and not n.done:
   rclpy.spin_once(n,timeout_sec=.01)
   if time.monotonic()-last>.02:n.tick();last=time.monotonic()
   if time.monotonic()-n.wall>100:n.save('infrastructure_timeout')
 finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
