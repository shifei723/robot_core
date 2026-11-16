#!/usr/bin/env python3
"""Prospective trial analysis. No failed run is omitted or called an arrival."""
from pathlib import Path
import json,csv,re,math
import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial import cKDTree
R=Path(__file__).resolve().parent;CORE=R.parents[1]
PERIOD={x['name']:x['duration'] for x in json.loads((CORE/'navigation/benchmarks/dynamic_dynabarn/dynabarn_ros2/worlds/official_world_2.json').read_text())['objects']}

def series(p):return np.atleast_1d(np.genfromtxt(p,delimiter=',',names=True))
def duty(t,mask):return float(np.sum(np.r_[np.diff(t),0.]*mask)) if len(t) else 0.
def longest(t,mask):
 best=span=0.
 for h,d in zip(mask,np.r_[np.diff(t),0.]):span=span+d if h else 0.;best=max(best,span)
 return float(best)
def latency(values):
 a=np.array(values,float)
 return {'n':len(a),'mean_ms':float(a.mean()),'p95_ms':float(np.percentile(a,95))} if len(a) else None

def position_at(a,t,name,phase):
 i=int(np.clip(np.searchsorted(a[:,0],t)-1,0,len(a)-2));period=PERIOD[name]
 if np.floor((a[i,0]+phase)/period)!=np.floor((a[i+1,0]+phase)/period):
  edge=(np.floor((a[i,0]+phase)/period)+1)*period-phase
  return a[i if t<edge else i+1,1:]
 u=np.clip((t-a[i,0])/max(1e-9,a[i+1,0]-a[i,0]),0,1)
 return a[i,1:]+u*(a[i+1,1:]-a[i,1:])

def barn(row):
 p=Path(row['source_result']);d=json.loads(p.read_text());kind=row['kind'];static=kind=='static'
 a=series(p.with_suffix('.csv') if static else p.parent/'robot.csv');t=a['sim_elapsed_s'];v=a['odom_vx'];x=a['world_x'];y=a['world_y']
 gaps=[];states=[];decisions=[];unknown_samples=0;max_coast=0.;unknown_overlap=0
 if static:
  scene=ET.parse(CORE/f'navigation/benchmarks/static_barn/upstream/jackal_helper/worlds/BARN/world_{row["world"]}.world').find('world');centers=[];radii=[]
  for m in scene.findall('model'):
   c=m.find('.//collision/geometry/cylinder')
   if c is not None:centers.append(list(map(float,m.findtext('pose').split()))[:2]);radii.append(float(c.findtext('radius')))
  dist,ids=cKDTree(centers).query(np.c_[x,y]);gaps=dist-np.array(radii)[ids]-.334
  group='retest_20261116_current';log=CORE/f'navigation/benchmarks/static_barn/logs/{group}/{row["method"]}_{row["world"]}_{row["trial"]}_launch.log'
  arrival=d['elapsed_sim_s'];start=0;goal_distance=float(np.hypot(x[-1]+2.25,y[-1]-13)) if len(t) else None
 else:
  objects={}
  for z in csv.DictReader((p.parent/'objects.csv').open()):objects.setdefault(z['name'],[]).append([float(z[k]) for k in ['sim_s','world_x','world_y']])
  objects={k:np.array(v) for k,v in objects.items()};start=d['start_sim_s']
  if start is not None:
   for tt,xx,yy in zip(t,x,y):gaps.append(min(np.linalg.norm([xx,yy]-position_at(a,tt+start,k,row['phase']))-.834 for k,a in objects.items()))
  states=list(csv.DictReader((p.parent/'states.csv').open()));decisions=json.loads((p.parent/'decisions.json').read_text())
  if (p.parent/'tracks.json').exists():
   for f in json.loads((p.parent/'tracks.json').read_text()):
    for o in f['objects']:
     if o['state']==3:unknown_samples+=1
     elif o['state']==2 and 'last_observed' in o:max_coast=max(max_coast,f['stamp']-o['last_observed'])
  for z in decisions:unknown_overlap+=bool({u['id'] for u in z.get('unknown_regions',[])} & {tr['id'] for tr in z['tracks']})
  group='retest_20261116_current';log=CORE/f'navigation/benchmarks/dynamic_dynabarn/logs/{group}/{p.parent.name}_launch.log'
  arrival=d['sim_elapsed_s'];goal_distance=float(np.hypot(x[-1]+9,y[-1])) if len(t) else None
 status=d['status'];success=status=='success';stopped=np.abs(v)<.03
 goalxy=np.array([-2.25,13]) if static else np.array([-9,0]);distance=np.linalg.norm(np.c_[x,y]-goalxy,axis=1);near=t[distance<1.]
 first_near=float(near[0]) if len(near) else None
 risks=np.array([s['state'] in ('DYNAMIC_WAIT','DYNAMIC_SHIELD') for s in states],bool)
 tt=np.array([float(s['elapsed_s']) for s in states]);wait=duty(tt,risks) if len(tt) else None
 perception=np.array([s['state'] in ('WAIT_TRACK','WAIT_STATIC_MAP') for s in states],bool)
 ms=[float(z) for z in re.findall(r'WHEEL_PLAN_MS ([0-9.]+)',log.read_text())] if log.exists() else []
 return {**row,'condition':f'地图{row["world"]}' if static else f'初相位{row["phase"]}s','status':status,'success':success,'collision':status=='collision','arrival_s':arrival if success else None,'termination_s':arrival,'first_within_1m_s':first_near,'final_approach_s':float(arrival-first_near) if success and first_near is not None else None,'path_m':float(np.linalg.norm(np.diff(np.c_[x,y],axis=0),axis=1).sum()),'stopped_s':duty(t,stopped),'longest_stop_s':longest(t,stopped),'risk_wait_s':wait,'perception_wait_s':duty(tt,perception) if len(tt) else None,'longest_risk_wait_s':longest(tt,risks) if len(tt) else None,'min_clearance_m':float(min(gaps)) if len(gaps) else None,'final_goal_distance_m':goal_distance,'min_leg_height_m':None,'min_beam_margin_m':None,'planner_latency':latency(ms),'frs_decision_latency':latency([z['decision_ms'] for z in decisions if z.get('moving_track_count',z['track_count'])>0 and not z['stale']]),'unknown_snapshot_samples':unknown_samples,'max_coast_age_s':max_coast,'unknown_motion_overlap_packets':unknown_overlap,'protocol_sha256':d.get('protocol_sha256',d.get('configuration_sha256'))}

def indoor(row):
 p=Path(row['source_result']);s=json.loads(p.read_text());h=json.loads((p.parent/'trace.json').read_text());goal=s.get('goal_sent_at');a=[x for x in h['poses'] if goal is not None and x['t']>=goal];t=np.array([x['t']-goal for x in a]);poses=np.array([x['pose'][:2] for x in a]);v=np.array([x['v'] for x in a]);dec=h['decisions']
 status='infrastructure_error' if s['error'] else 'collision' if s['collision'] else 'success' if s.get('combined_pass') else 'incomplete_height' if s['reached'] else 'timeout' if goal is not None else 'infrastructure_no_goal'
 risk=np.array([x['status'] in ('DYNAMIC_WAIT','DYNAMIC_SHIELD') and abs(x['v'])<.01 for x in a]);perception=np.array([x['status'] in ('WAIT_TRACK','WAIT_STATIC_MAP') for x in a]);stopped=np.abs(v)<.03
 log=p.parent/'scan_planner_node.log';ms=[float(z) for z in re.findall(r'WHEEL_PLAN_MS ([0-9.]+)',log.read_text())];height=s.get('height_validation',{})
 return {**row,'condition':f'障碍速度{row["speed"]:.2f}m/s','status':status,'success':status=='success','collision':s['collision'],'arrival_s':s.get('arrival_time')-goal if status=='success' else None,'termination_s':s['duration']-goal if goal else s['duration'],'path_m':float(np.linalg.norm(np.diff(poses,axis=0),axis=1).sum()) if len(poses) else 0.,'stopped_s':duty(t,stopped),'longest_stop_s':longest(t,stopped),'risk_wait_s':duty(t,risk),'perception_wait_s':duty(t,perception),'longest_risk_wait_s':longest(t,risk),'min_clearance_m':min((x['clearance'] for x in a),default=None),'final_goal_distance_m':float(np.linalg.norm(poses[-1]-s['goal'])) if len(poses) else None,'min_leg_height_m':height.get('min_leg_height'),'min_beam_margin_m':height.get('min_beam_margin'),'goal_sent_at_s':goal,'combined_pass':s.get('combined_pass',False),'planner_latency':latency(ms),'frs_decision_latency':latency([z['decision_ms'] for z in dec if z.get('moving_track_count',z['track_count'])>0 and not z['stale']]),'unknown_snapshot_samples':sum(o['state']==3 for f in h['tracks'] for o in f['objects']),'max_coast_age_s':None,'unknown_motion_overlap_packets':sum(bool({u['id'] for u in d.get('unknown_regions',[])} & {tr['id'] for tr in d['tracks']}) for d in dec),'protocol_sha256':s['controller_sha256']+':'+s['simulator_sha256']}

def main():
 progress=json.loads((R/'progress.json').read_text());rows=[]
 for row in progress['completed']:
  if not Path(row['source_result']).exists():rows.append({**row,'success':False,'collision':False});continue
  rows.append(indoor(row) if row['kind']=='indoor' else barn(row))
 summary=[]
 for kind in ['static','dynamic','indoor']:
  for method in sorted({r['method'] for r in rows if r['kind']==kind}):
   part=[r for r in rows if r['kind']==kind and r['method']==method];ok=[r for r in part if r['success']]
   def mean(k,rs=part):return float(np.mean([r[k] for r in rs if r.get(k) is not None])) if any(r.get(k) is not None for r in rs) else None
   summary.append({'kind':kind,'method':method,'runs':len(part),'successes':len(ok),'collisions':sum(r['collision'] for r in part),'timeouts':sum(r['status']=='timeout' for r in part),'other_failures':sum(r['status'] not in ('success','collision','timeout') for r in part),'mean_success_arrival_s':mean('arrival_s',ok),'mean_risk_wait_s_all':mean('risk_wait_s'),'mean_stopped_s_all':mean('stopped_s'),'min_clearance_m':min((r['min_clearance_m'] for r in part if r.get('min_clearance_m') is not None),default=None),'mean_path_m_all':mean('path_m')})
 (R/'results.json').write_text(json.dumps({'runs':rows,'summary':summary,'planned':progress['planned'],'completed':len(rows)},indent=2,ensure_ascii=False)+'\n')
 fields=['kind','method','condition','trial','status','arrival_s','termination_s','first_within_1m_s','final_approach_s','path_m','stopped_s','longest_stop_s','risk_wait_s','perception_wait_s','longest_risk_wait_s','min_clearance_m','min_leg_height_m','min_beam_margin_m','final_goal_distance_m']
 with (R/'runs.csv').open('w') as f:w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
 if summary:
  with (R/'summary.csv').open('w') as f:w=csv.DictWriter(f,summary[0].keys());w.writeheader();w.writerows(summary)
 print(json.dumps(summary,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
