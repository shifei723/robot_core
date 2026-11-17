"""Explicit DynaBARN evaluation profiles, independent of ROS and planners.

Challenge task/termination rules follow the pinned reference run.py. The ROS2
plant and matched speed/sensor settings remain an adaptation, not leaderboard
equivalence. Legacy preserves the previously published internal experiments.
"""
from dataclasses import dataclass,asdict
import math

REFERENCE_REVISION='bf5a226f6088ec96bf0d2dbee3253a8ea6119b83'

@dataclass(frozen=True)
class Protocol:
 name:str
 start_x:float
 start_yaw:float
 goal_x:float
 radius:float
 timeout:float
 motion_threshold:float
 timing:str
 def navigation_goal(self,anchor):
  x,y,heading=anchor;angle=heading-self.start_yaw;dx=self.goal_x-self.start_x
  return (x+dx*math.cos(angle),y+dx*math.sin(angle))
 def metadata(self):return asdict(self)

PROFILES={
 'challenge':Protocol('challenge',11.,3.14,-9.,1.,100.,.1,'first_truth_displacement'),
 'legacy':Protocol('legacy',12.,math.pi,-9.,.3,120.,0.,'goal_publication'),
}

def outcome(protocol,elapsed,distance,collision):
 """Reference ordering: collision wins; expiry at 100 s wins over arrival."""
 if collision:return 'collision'
 if elapsed>=protocol.timeout:return 'timeout'
 if distance<=protocol.radius:return 'success'
 return None

def motion_started(protocol,x,y):return math.hypot(x-protocol.start_x,y)>=protocol.motion_threshold
