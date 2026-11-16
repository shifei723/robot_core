"""World-frame actor paths for the indoor study; evaluation/sensor generation only."""
import numpy as np


def indoor_actors(speed, variant="follow_exit"):
    if variant=="dual_crossing":
        return [
            dict(id="truth_0",waypoints=[[1.8,-1.1],[1.8,.8]],speed=speed,trigger_axis=0,trigger_value=.5),
            dict(id="truth_1",waypoints=[[2.9,1.5],[4.85,1.5]],speed=speed,trigger_axis=1,trigger_value=.2),
            dict(id="truth_2",waypoints=[[4.85,3.1],[2.9,3.1]],speed=speed,trigger_axis=1,trigger_value=.4),
        ]
    if variant!="follow_exit":raise ValueError("unknown indoor actor variant")
    return [
        dict(id='truth_0',waypoints=[[1.8,-1.1],[1.8,.8]],speed=speed,trigger_axis=0,trigger_value=.5),
        dict(id='truth_1',waypoints=[[2.9,1.5],[4.2,1.5],[4.2,1.9],[5.2,1.9]],speed=speed,trigger_axis=1,trigger_value=.2),
        dict(id='truth_2',waypoints=[[2.9,3.6],[4.85,3.6]],speed=speed,trigger_axis=1,trigger_value=.9),
    ]


def actor_state(spec, age, acceleration=.4, pause=.25):
    """Stop-to-stop trapezoids, so direction changes occur at zero speed.
    Position and velocity are continuous; acceleration magnitude <= acceleration.
    """
    points=np.asarray(spec['waypoints'],float);age=max(0.,float(age));vmax=spec['speed']
    for i,(a,b) in enumerate(zip(points,points[1:])):
        length=float(np.linalg.norm(b-a))
        if length<=0:raise ValueError('actor path must have positive segment lengths')
        direction=(b-a)/length;ramp=min(vmax/acceleration,np.sqrt(length/acceleration))
        peak=acceleration*ramp;cruise=max(0.,(length-acceleration*ramp*ramp)/peak);duration=2*ramp+cruise
        if age<=duration:
            if age<ramp:distance=.5*acceleration*age*age;velocity=acceleration*age
            elif age<ramp+cruise:distance=.5*acceleration*ramp*ramp+peak*(age-ramp);velocity=peak
            else:remaining=duration-age;distance=length-.5*acceleration*remaining*remaining;velocity=acceleration*remaining
            return a+direction*distance,direction*velocity
        age-=duration
        if i<len(points)-2:
            if age<pause:return b.copy(),np.zeros(2)
            age-=pause
    return points[-1].copy(),np.zeros(2)
