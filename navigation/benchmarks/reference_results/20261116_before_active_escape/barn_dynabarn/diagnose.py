#!/usr/bin/env python3
"""Offline contact/stop evidence; no truth feeds inference."""
import json,csv,math,runpy,re
from pathlib import Path
import numpy as np
R=Path(__file__).resolve().parent;CORE=R.parents[1]
a=runpy.run_path(str(R/'analyze.py'));interp=a['position_at'];out=[]
for row in json.loads((R/'results.json').read_text())['runs']:
 if row['kind']!='dynamic':continue
 p=Path(row['source_result']).parent;raw=json.loads((p/'result.json').read_text());rr=list(csv.DictReader((p/'robot.csv').open()))
 if not rr or raw['navigation_anchor'] is None:continue
 terminal=rr[-1];t=float(terminal['sim_elapsed_s']);xy=np.array([float(terminal['world_x']),float(terminal['world_y'])]);stamp=t+raw['start_sim_s'];objects={}
 for o in csv.DictReader((p/'objects.csv').open()):objects.setdefault(o['name'],[]).append([float(o[k]) for k in ['sim_s','world_x','world_y']])
 objects={k:np.array(v) for k,v in objects.items()};truth={k:interp(v,stamp,k,row['phase']) for k,v in objects.items()};name=min(truth,key=lambda k:np.linalg.norm(truth[k]-xy));q=truth[name];phase=row['phase'];period=a['PERIOD'][name]
 velocity=None
 lo=max(stamp-.1,float(objects[name][0,0]));hi=min(stamp+.1,float(objects[name][-1,0]))
 if hi>lo and math.floor((lo+phase)/period)==math.floor((hi+phase)/period):velocity=((interp(objects[name],hi,name,phase)-interp(objects[name],lo,name,phase))/(hi-lo)).tolist()
 stop=t
 for r in reversed(rr):
  if abs(float(r['odom_vx']))>=.03:break
  stop=float(r['sim_elapsed_s'])
 frames=json.loads((p/'tracks.json').read_text());frames=[f for f in frames if f['stamp']<=stamp];last=frames[-1] if frames else None
 anchor=np.array(raw['navigation_anchor']);theta=math.pi-anchor[2];rot=np.array([[math.cos(theta),-math.sin(theta)],[math.sin(theta),math.cos(theta)]])
 matches=[]
 if last:
  for tr in last['objects']:
   pworld=(np.array(tr['p'])-anchor[:2])@rot.T+[12.,0.];vworld=rot@np.array(tr['v'])
   matches.append({'id':tr['id'],'world_position':pworld.tolist(),'world_velocity':vworld.tolist(),'state':tr['state'],'observed':tr['observed'],'last_observed_age_s':last['stamp']-tr['last_observed'],'distance_to_contacting_obstacle_m':float(np.linalg.norm(pworld-q))})
 dec=json.loads((p/'decisions.json').read_text());d=dec[-1] if dec else {}
 log=CORE/f'navigation/benchmarks/dynamic_dynabarn/logs/retest_20261116_current/{p.name}_launch.log';lines=log.read_text().splitlines() if log.exists() else []
 relevant=[s for s in lines if any(word in s.lower() for word in ['fail','fallback','seed','unknown','invalid','unreachable'])][-20:]
 out.append({'method':row['method'],'phase_s':phase,'status':row['status'],'termination_s':t,'robot_world_xy':xy.tolist(),'terminal_odom_vx_mps':float(terminal['odom_vx']),'continuous_terminal_low_speed_s':t-stop,'nearest_obstacle':name,'obstacle_world_xy':q.tolist(),'obstacle_world_velocity_mps':velocity,'sampled_circle_clearance_m':float(np.linalg.norm(q-xy)-.834),'terminal_decision':{k:d.get(k) for k in ['stamp','feasible','shield','scale','v','before_v','moving_track_count','unknown_region_count','nominal_clearance']},'terminal_tracks':matches,'recent_planner_evidence':relevant})
(R/'failure_diagnostics.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
print(json.dumps([{k:r[k] for k in ['method','phase_s','status','continuous_terminal_low_speed_s','nearest_obstacle']} for r in out],indent=2))
