import math
import numpy as np

def goal_in_odom(world_pose, odom_pose, goal=(-2.25,13.)):
    wx,wy,wyaw=world_pose;ox,oy,oyaw=odom_pose
    angle=oyaw-wyaw;dx=goal[0]-wx;dy=goal[1]-wy
    return (ox+math.cos(angle)*dx-math.sin(angle)*dy,
            oy+math.sin(angle)*dx+math.cos(angle)*dy,math.pi/2+angle)

def score(success,elapsed,reference_length):
    if not success:return 0.
    if not math.isfinite(elapsed) or elapsed<0 or reference_length<=0:
        raise ValueError('invalid elapsed time or reference length')
    optimal=reference_length/2.
    return float(optimal/np.clip(elapsed,2*optimal,8*optimal))

# Fixed spawn prior; truth is only used to start the evaluation clock and score.
STATIC_START=(-2.25,3.,math.pi/2)
STATIC_GOAL=(-2.25,13.)

def motion_started(x,y):
    return math.hypot(x-STATIC_START[0],y-STATIC_START[1])>=.1

def outcome(elapsed,distance,collision,tolerance=1.,timeout=100.):
    if collision:return 'collision'
    if elapsed>=timeout:return 'timeout'
    if distance<=tolerance:return 'success'
    return None
