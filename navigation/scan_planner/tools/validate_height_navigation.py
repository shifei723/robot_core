#!/usr/bin/env python3
"""End-to-end, headless wheel-height experiment; run after sourcing install/ws_nav.
The oracle uses scene geometry only for assertions, never as planner input.
Nonzero exit means a behavioral assertion failed, not just a node startup failure.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

CASES = ('normal', 'low', 'blocked', 'detour', 'slow_low', 'sensor_loss')


def box_overlap(x, y, yaw, box):
    x0, x1, y0, y1 = box
    for side in (-1, 1):
        cx = x + side * .18 * math.cos(yaw)
        cy = y + side * .18 * math.sin(yaw)
        dx, dy = max(x0-cx, 0., cx-x1), max(y0-cy, 0., cy-y1)
        if dx*dx + dy*dy <= .25**2:
            return True
    return False


def geometry(x, y, z, yaw, clearance):
    # Same documented scene dimensions, independent of the perceived voxel map.
    walls = [(-10,-9.8,-10,10),(9.8,10,-10,10),(-10,10,-10,-9.8),
             (-10,10,9.8,10),(-9.8,-1,-.1,.1),(1,9.8,-.1,.1)]
    tables = [(-6,-4.8,4,4.6),(3,4.2,5,5.6),(-2,-.8,-4,-3.4),(5,6.2,-6,-5.4),
              (-3,-2.4,9.4,9.8),(-7,-6.4,-.5,-.1),(9.4,9.8,-7,-6.4)]
    overlap = box_overlap(x,y,yaw,(-1,1,-.3,.3))
    return (any(box_overlap(x,y,yaw,b) for b in walls+tables), overlap,
            clearance-z-.328 if overlap else None)


def run_case(args):
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Twist
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import Float32, String
    from scan_planner_msgs.msg import Bspline

    args.output.mkdir(parents=True, exist_ok=True)
    case=args.case
    clearance={'normal':.8,'low':.62,'blocked':.48,'detour':.8,'slow_low':.62,'sensor_loss':.62}[case]
    if args.clearance is not None: clearance=args.clearance
    start=(5.6,-7.) if case=='detour' else (0.,-2.)
    goal=(5.6,-4.5) if case=='detour' else (0.,2.)
    rate=.02 if case=='slow_low' else .08
    env=os.environ.copy(); env['ROS_DOMAIN_ID']=str(args.domain)
    # ROS scripts must use the Python matching the installed ROS distribution.
    env['PATH']=f'/opt/ros/{env.get("ROS_DISTRO","jazzy")}/bin:/usr/bin:/bin:'+env.get('PATH','')
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    log=(args.output/f'{case}.log').open('w')
    proc=subprocess.Popen(['ros2','launch','scan_planner','run.launch.py',
        'robot_model:=slash','is_real_world:=false','controller_mode:=closed_loop','use_gpu:=false',
        'dynamic_obstacles:=false',f'init_x:={start[0]}',f'init_y:={start[1]}',
        f'beam_clearance:={clearance}',f'leg_height_rate:={rate}',
        f'init_yaw:={args.initial_yaw}',f'stable_heading:={str(not args.baseline).lower()}'],
        env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init(); node=rclpy.create_node('height_experiment_validator')
    data={'pose':None,'clouds':0,'maps':0,'plans':0,'height_cmd':.25,'vx':0.,'wz':0.,'state':'',
          'states':set(),'planner_states':set(),'t0':None}
    rows=[]; trajectories=[]; injected=None; stopped_pid=None

    def odom(msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        data['pose']=(p.x,p.y,p.z,yaw)
        if data['t0'] is not None:
            collision,under,margin=geometry(p.x,p.y,p.z,yaw,clearance)
            rows.append(dict(t=time.monotonic()-data['t0'],stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
                x=p.x,y=p.y,z=p.z,yaw=yaw,leg_height=p.z-.0322,height_cmd=data['height_cmd'],
                vx=data['vx'],wz=data['wz'],state=data['state'],solid_overlap=int(collision),
                beam_overlap=int(under),beam_margin=margin if margin is not None else float('nan')))
    def command(msg): data.update(vx=msg.linear.x,wz=msg.angular.z)
    def height(msg): data['height_cmd']=float(msg.data)
    def cloud(msg): data['clouds']+=1
    def occupancy(msg): data['maps']+=1
    def state(msg): data['state']=msg.data; data['states'].add(msg.data)
    def planner_state(msg): data['planner_states'].add(msg.data)
    def trajectory(msg):
        data['plans']+=1
        trajectories.append(dict(order=msg.order,knots=list(msg.knots),
                                 points=[[p.x,p.y,p.z] for p in msg.pos_pts]))
    subscriptions=[node.create_subscription(Odometry,'/quad_0/body_pose',odom,qos_profile_sensor_data),
        node.create_subscription(Twist,'/quad_0/cmd_vel',command,10),
        node.create_subscription(Float32,'/leg_height_cmd',height,10),
        node.create_subscription(PointCloud2,'/quad_0/cloud',cloud,qos_profile_sensor_data),
        node.create_subscription(PointCloud2,'/grid_map/occupancy',occupancy,qos_profile_sensor_data),
        node.create_subscription(String,'/height_control/state',state,10),
        node.create_subscription(String,'/planning/status',planner_state,QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Bspline,'/planning/bspline',trajectory,10)]
    publisher=node.create_publisher(PoseStamped,'/move_base_simple/goal',10)
    result={'case':case,'passed':False}
    try:
        begin=time.monotonic()
        while time.monotonic()-begin<20:
            rclpy.spin_once(node,timeout_sec=.02)
            if (time.monotonic()-begin>4 and data['clouds']>=10 and data['maps']>=10
                    and data['pose'] is not None and publisher.get_subscription_count()): break
        else: raise RuntimeError('Timed out waiting for odometry, lidar, map and goal subscriber')
        msg=PoseStamped(); msg.header.frame_id='world'; msg.header.stamp=node.get_clock().now().to_msg()
        msg.pose.position.x,msg.pose.position.y=goal; msg.pose.position.z=.28; msg.pose.orientation.w=1.
        data['t0']=time.monotonic(); publisher.publish(msg)
        print(f'{case}: start={start}, goal={goal}, clearance={clearance}, actuator={rate} m/s',flush=True)
        settled=None; reached=False; last_print=0.; budget=25 if case=='blocked' else args.timeout
        while time.monotonic()-data['t0']<budget:
            rclpy.spin_once(node,timeout_sec=.01)
            t=time.monotonic()-data['t0']; x,y,z,yaw=data['pose']
            error=math.hypot(x-goal[0],y-goal[1])
            if error<.15 and abs(data['vx'])<.03 and abs(data['wz'])<.05:
                settled=t if settled is None else settled
                if t-settled>2: reached=True;break
            else: settled=None
            if case=='sensor_loss' and injected is None and abs(y)<.10:
                # Only stop a sensor process owned by this launch's process group.
                processes=subprocess.check_output(['ps','-eo','pid=,pgid=,args='],text=True)
                for line in processes.splitlines():
                    fields=line.strip().split(None,2)
                    if len(fields)==3 and int(fields[1])==proc.pid and '/pcl_render_node ' in fields[2]:
                        stopped_pid=int(fields[0]);os.kill(stopped_pid,signal.SIGSTOP);injected=t;break
                if injected is None: raise RuntimeError('Could not locate owned sensor process for fault injection')
            if injected is not None and t-injected>4: break
            if case=='blocked' and 'NO_PATH' in data['planner_states'] and t>8: break
            if t-last_print>10:
                print(f'{case}: {t:.0f}s xy=({x:.2f},{y:.2f}) h={z-.0322:.3f} state={data["state"]}',flush=True);last_print=t
            if proc.poll() is not None: raise RuntimeError('Simulation launch exited unexpectedly')
        if not rows: raise RuntimeError('No odometry samples recorded after goal')
        # Sweep between consecutive samples too, including body rotation and raising.
        collision_count=0; margins=[]
        for a,b in zip(rows,rows[1:]):
            angle=math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw']))
            n=max(1,math.ceil(math.hypot(b['x']-a['x'],b['y']-a['y'])/.005),math.ceil(abs(angle)/.01),math.ceil(abs(b['z']-a['z'])/.005))
            for i in range(n+1):
                f=i/n; x=a['x']+(b['x']-a['x'])*f; y=a['y']+(b['y']-a['y'])*f; z=a['z']+(b['z']-a['z'])*f
                collision,under,margin=geometry(x,y,z,a['yaw']+angle*f,clearance)
                collision_count+=collision
                if under: margins.append(margin)
        errors=[]
        if collision_count: errors.append('solid_obstacle_overlap')
        if margins and min(margins)<.03-1e-5: errors.append('beam_clearance_below_3cm')
        if min(r['leg_height'] for r in rows)<.14-1e-5 or max(r['leg_height'] for r in rows)>.36+1e-5: errors.append('leg_limit')
        if case=='blocked':
            if 'NO_PATH' not in data['planner_states']: errors.append('no_explicit_no_path_status')
            if 'NO_PATH' not in data['states']: errors.append('controller_did_not_latch_no_path')
            if max(math.hypot(r['x']-start[0],r['y']-start[1]) for r in rows)>.03: errors.append('moved_towards_impassable_door')
        elif case=='sensor_loss':
            after=[r for r in rows if injected is not None and r['t']>injected+1.5]
            if len(after)<10 or not any(r['state']=='WAIT_MAP' for r in after): errors.append('missing_sensor_timeout_stop')
            elif (max(math.hypot(r['x']-after[0]['x'],r['y']-after[0]['y']) for r in after)>.02 or
                  max(abs(r['vx'])+abs(r['wz']) for r in after)>.01 or
                  max(r['leg_height'] for r in after)>after[0]['leg_height']+.01): errors.append('unsafe_sensor_timeout_behavior')
        else:
            if not reached: errors.append('goal_timeout')
            if case in ('low','slow_low'):
                if not margins or min(r['leg_height'] for r in rows)>.215: errors.append('no_lowered_door_crossing')
                if abs(rows[-1]['leg_height']-.2478)>.02: errors.append('height_not_restored')
                if 'LOWERING' not in data['states']: errors.append('height_gate_not_exercised')
        # Measure actual motion, not only command values. Exclude the initial
        # approach until 0.5 m progress for the post-approach turn metric.
        path_length=0.; turn=0.; stop_turn_time=0.; descent_turn=0.; lateral_sq=0.; lateral_time=0.
        for a,b in zip(rows,rows[1:]):
            dt=b['stamp']-a['stamp']
            if dt<=0: continue
            ds=math.hypot(b['x']-a['x'],b['y']-a['y'])
            da=abs(math.atan2(math.sin(b['yaw']-a['yaw']),math.cos(b['yaw']-a['yaw'])))
            path_length+=ds;turn+=da
            if math.hypot(a['x']-start[0],a['y']-start[1])>.5 and ds/dt<.015 and da/dt>.1:
                stop_turn_time+=dt
            if (b['leg_height']-a['leg_height'])/dt<-.0025: descent_turn+=da
            lateral_sq+=a['x']**2*dt;lateral_time+=dt
        if not args.baseline and case in ('low','slow_low') and stop_turn_time>0.5:
            errors.append('unnecessary_stationary_rotation')
        result=dict(case=case,passed=not errors,errors=errors,start=start,goal=goal,beam_clearance_m=clearance,
            actuator_rate_m_s=rate,initial_yaw_rad=args.initial_yaw,baseline=args.baseline,
            path_length_m=path_length,total_yaw_variation_rad=turn,stationary_rotation_s=stop_turn_time,
            descent_yaw_variation_rad=descent_turn,lateral_rmse_m=math.sqrt(lateral_sq/max(lateral_time,1e-9)),
            reached=reached,duration_s=rows[-1]['t'],final_error_m=math.hypot(rows[-1]['x']-goal[0],rows[-1]['y']-goal[1]),
            leg_min_m=min(r['leg_height'] for r in rows),leg_final_m=rows[-1]['leg_height'],
            min_beam_margin_m=min(margins) if margins else None,solid_collision_samples=collision_count,
            controller_states=sorted(data['states']),planner_states=sorted(data['planner_states']),
            trajectories=data['plans'],sensor_interruption_s=injected,odom_samples=len(rows))
        with (args.output/f'{case}.csv').open('w') as f:
            w=csv.DictWriter(f,fieldnames=rows[0].keys());w.writeheader();w.writerows(rows)
        (args.output/f'{case}_trajectories.json').write_text(json.dumps(trajectories))
    except Exception as e:
        result.update(errors=[str(e)])
    finally:
        (args.output/f'{case}.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
        node.destroy_node();rclpy.shutdown()
        if stopped_pid is not None:
            try:os.kill(stopped_pid,signal.SIGCONT)
            except ProcessLookupError:pass
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:pass
        try:os.killpg(proc.pid,signal.SIGKILL)
        except ProcessLookupError:pass
        proc.wait();log.close()
    return 0 if result['passed'] else 1


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=CASES)
    p.add_argument('--baseline',action='store_true',help='Pre-fix direction controller, for controlled comparison')
    p.add_argument('--initial-yaw',type=float,default=0.)
    p.add_argument('--clearance',type=float)
    p.add_argument('--output',type=Path,default=Path('outputs/height_experiment'))
    p.add_argument('--timeout',type=float,default=100.)
    p.add_argument('--domain',type=int,default=149)
    p.add_argument('--repeat',type=int,default=1)
    args=p.parse_args()
    if args.case:return run_case(args)
    args.output.mkdir(parents=True,exist_ok=True)
    codes=[];results=[]
    for repeat in range(args.repeat):
        folder=args.output/f'run_{repeat+1}'
        for case in CASES:
            codes.append(subprocess.call([sys.executable,__file__,'--case',case,'--output',str(folder),
                '--timeout',str(args.timeout),'--domain',str(args.domain),'--initial-yaw',str(args.initial_yaw)]+
                (['--baseline'] if args.baseline else [])+(['--clearance',str(args.clearance)] if args.clearance is not None else [])))
            results.append(json.loads((folder/f'{case}.json').read_text()))
    (args.output/'summary.json').write_text(json.dumps(results,indent=2))
    print(f'Passed {sum(r["passed"] for r in results)}/{len(results)} experiments',flush=True)
    return int(any(codes))

if __name__=='__main__':sys.exit(main())
