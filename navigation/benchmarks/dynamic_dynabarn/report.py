#!/usr/bin/env python3
"""Offline comparisons, keeping unsuccessful trials and discontinuous loop boundaries."""
import csv,json,math,re
from pathlib import Path
import numpy as np
from scipy.optimize import linear_sum_assignment
R=Path(__file__).resolve().parent
P=R/'dynabarn_ros2'
periods={o['name']:o['duration'] for o in json.loads((P/'worlds/official_world_2.json').read_text())['objects']}

def table(p):return list(csv.DictReader(p.open()))
def stats(x):
 a=np.asarray(x,float)
 return {'count':len(a),'mean':float(a.mean()),'p95':float(np.percentile(a,95)),'max':float(a.max())} if len(a) else None

def run(path):
 d=json.loads((path/'result.json').read_text());method=d.get('method',d['mode']);phase=d.get('motion_phase_s',0)
 rr=table(path/'robot.csv');a=np.array([[float(z[k]) for k in ['sim_elapsed_s','world_x','world_y','yaw','odom_vx','odom_wz']] for z in rr]).reshape(-1,6)
 objects={}
 for z in table(path/'objects.csv'):objects.setdefault(z['name'],[]).append([float(z[k]) for k in ['sim_s','world_x','world_y']])
 objects={k:np.array(v) for k,v in objects.items()}
 def at(v,t,name):
  i=np.clip(np.searchsorted(v[:,0],t)-1,0,len(v)-2);dt=v[i+1,0]-v[i,0]
  wrap=np.floor((v[i,0]+phase)/periods[name])!=np.floor((v[i+1,0]+phase)/periods[name])
  if wrap:
   boundary=(np.floor((v[i,0]+phase)/periods[name])+1)*periods[name]-phase
   return v[i if t<boundary else i+1,1:]
  u=np.clip((t-v[i,0])/max(dt,1e-9),0,1);return v[i,1:]+u*(v[i+1,1:]-v[i,1:])
 gaps=[]
 for row in a:
  gaps.append(min(np.linalg.norm(row[1:3]-at(v,row[0]+d['start_sim_s'],name))-.834 for name,v in objects.items()))
 derived=R/'derived';derived.mkdir(exist_ok=True)
 if len(a):np.savetxt(derived/(path.name+'_clearance.csv'),np.c_[a[:,0],gaps],delimiter=',',header='elapsed_s,clearance_m',comments='')
 dt=np.r_[np.diff(a[:,0]),0.] if len(a) else np.zeros(0)
 stopped=np.abs(a[:,4])<.03 if len(a) else np.zeros(0,bool)
 longest=span=0.
 for stop,duration in zip(stopped,dt):span=span+duration if stop else 0.;longest=max(longest,span)
 states=table(path/'states.csv');durations={}
 for z,n in zip(states,states[1:]):durations[z['state']]=durations.get(z['state'],0)+float(n['elapsed_s'])-float(z['elapsed_s'])
 decisions=json.loads((path/'decisions.json').read_text());perception=json.loads((path/'perception.json').read_text());tracks=json.loads((path/'tracks.json').read_text())
 anchor=d.get('navigation_anchor');errors=[];velocity_errors=[];unmatched=matched=discontinuities=0
 if anchor is not None:
  theta=math.pi-anchor[2];rotation=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
  names=list(objects)
  for row in tracks:
   if not row['healthy'] or d['start_sim_s'] is None or row['stamp']<d['start_sim_s']:continue
   observed=[o for o in row['objects'] if o['observed']]
   if not observed:continue
   estimated=np.array([o['p'] for o in observed]);estimated=(estimated-anchor[:2])@rotation.T+[12.,0.]
   actual=np.array([at(objects[name],row['stamp'],name) for name in names])
   cost=np.linalg.norm(estimated[:,None,:]-actual[None,:,:],axis=2);i,j=linear_sum_assignment(cost)
   accepted=0
   for ii,jj in zip(i,j):
    if cost[ii,jj]>.8:continue
    accepted+=1;matched+=1;errors.append(estimated[ii]-actual[jj]);v=objects[names[jj]]
    k=int(np.clip(np.searchsorted(v[:,0],row['stamp']),1,len(v)-2))
    if np.floor((v[k-1,0]+phase)/periods[names[jj]])!=np.floor((v[k+1,0]+phase)/periods[names[jj]]):discontinuities+=1;continue
    true_v=(v[k+1,1:]-v[k-1,1:])/(v[k+1,0]-v[k-1,0]);est_v=rotation@observed[ii]['v'];velocity_errors.append(est_v-true_v)
   unmatched+=len(observed)-accepted
 ego_error=None
 if anchor is not None and (path/'odometry.csv').exists() and len(a):
  od=table(path/'odometry.csv');ego=np.array([[float(z['x']),float(z['y'])] for z in od]);world_ego=(ego-anchor[:2])@rotation.T+[12.,0.]
  ego_error=float(np.sqrt(np.mean(np.sum((world_ego-a[:len(ego),1:3])**2,axis=1))))
 name=path.name;log=R/'logs/final'/f'{name}_launch.log';ms=[]
 if log.exists():ms=[float(x) for x in re.findall(r'WHEEL_PLAN_MS ([0-9.]+)',log.read_text())]
 return {'run':name,'method':method,'phase_s':phase,'status':d['status'],'success':d['status']=='success','collision':d['collision'],
  'arrival_s':d['sim_elapsed_s'] if d['status']=='success' else None,'termination_s':d['sim_elapsed_s'],
  'start_sim_s':d['start_sim_s'],'path_m':float(np.linalg.norm(np.diff(a[:,1:3],axis=0),axis=1).sum()) if len(a)>1 else 0.,
  'stopped_s':float(dt[stopped].sum()),'longest_stop_s':longest,'min_dynamic_circle_clearance_m':float(min(gaps)) if gaps else None,
  'terminal_odom_vx_mps':float(a[-1,4]) if len(a) else None,'terminal_state':states[-1]['state'] if states else None,'terminal_frs_decision':{k:v for k,v in decisions[-1].items() if k not in ['boxes','tracks','nominal_path','chosen_path']} if decisions else None,
  'final_goal_distance_m':float(np.linalg.norm(a[-1,1:3]-[-9,0])) if len(a) else None,
  'state_duration_s':durations,'frs_stale_decisions':sum(bool(z['stale']) for z in decisions),'frs_shield_decisions':sum(bool(z['shield']) for z in decisions),'frs_infeasible_decisions':sum(not bool(z['feasible']) for z in decisions),'active_frs_decision_ms':stats([z['decision_ms'] for z in decisions if z['track_count']>0 and not z['stale']]),
  'perception_callback_ms':stats([z['filter_and_tracking_ms'] for z in perception]),'planner_call_ms':stats(ms),
  'matched_observed_position_rmse_xy_m':np.sqrt(np.mean(np.square(errors),axis=0)).tolist() if errors else None,
  'matched_velocity_rmse_xy_mps':np.sqrt(np.mean(np.square(velocity_errors),axis=0)).tolist() if velocity_errors else None,
  'matched_observations':matched,'unmatched_observations':unmatched,'velocity_samples_excluded_at_loop_wrap':discontinuities,
  'ego_position_rmse_m':ego_error,'peak_frs_tracks':max((z['track_count'] for z in decisions),default=None),'protocol_sha256':d.get('protocol_sha256'),
  'configuration_sha256':d.get('configuration_sha256'),'source_result':str(path/'result.json')}

attempts=[run(p.parent) for p in sorted((R/'results/final').glob('*/result.json'))]
selected={}
for r in attempts:
 if r['start_sim_s'] is not None:selected.setdefault((r['method'],r['phase_s']),r)
rows=list(selected.values())
summary={}
for method in ['mppi','frs_soft','frs_hard']:
 a=[x for x in rows if x['method']==method];success=[x for x in a if x['success']]
 summary[method]={'runs':len(a),'successes':len(success),'collisions':sum(x['collision'] for x in a),'timeouts':sum(x['status']=='timeout' for x in a),
  'mean_success_arrival_s':float(np.mean([x['arrival_s'] for x in success])) if success else None,
  'mean_stopped_s_all_runs':float(np.mean([x['stopped_s'] for x in a])) if a else None,
  'min_dynamic_circle_clearance_m':min((x['min_dynamic_circle_clearance_m'] for x in a if x['min_dynamic_circle_clearance_m'] is not None),default=None)}
report={'scope':'One official DynaBARN world, three preset motion phase variants; native ROS2/Gazebo/Jackal, common 5m lidar and static background. Not three independent maps or official leaderboard.',
 'safety_metric':'Gazebo TouchPlugin contacts primary; radius .334+.5 circle clearance offline, interpolation never crosses original loop jumps.',
 'tracking_metric':'Observed estimates matched offline to truth with 0.8m gate. Includes ego-odometry errors; unmatched points separately counted. No truth used for inference.',
 'summary':summary,'runs':rows,'attempts':attempts,'startup_failures':[r for r in attempts if r['start_sim_s'] is None]}
(R/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
if rows:
 fields=['run','method','phase_s','status','arrival_s','termination_s','path_m','stopped_s','longest_stop_s','min_dynamic_circle_clearance_m','final_goal_distance_m']
 with (R/'comparison.csv').open('w') as f:w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
print(json.dumps(summary,indent=2))
