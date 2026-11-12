#!/usr/bin/env python3
import csv,json,re
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree
import xml.etree.ElementTree as ET
root=Path(__file__).parent;out=root/'results/spatial_comparison';out.mkdir(parents=True,exist_ok=True)
rows=[]
for group,method in [('corridor_reuse','slash'),('spatial','slash'),('spatial_safe','slash'),('spatial_final','slash'),('spatial_guard','slash'),('spatial_release','slash'),('spatial_mppi','mppi')]:
 for file in sorted((root/f'results/{group}').glob('*_trial_*.json')):
  d=json.loads(file.read_text());trace=file.with_suffix('.csv')
  a=np.atleast_1d(np.genfromtxt(trace,delimiter=',',names=True));
  if not a.size:continue
  dt=np.r_[np.diff(a['sim_elapsed_s']),0.]
  scene=ET.parse(root/f"upstream/jackal_helper/worlds/BARN/world_{d['world']}.world").find('world')
  centers=[list(map(float,m.findtext('pose').split()))[:2] for m in scene.findall('model') if m.find('.//cylinder') is not None]
  clearance=float((cKDTree(centers).query(np.c_[a['world_x'],a['world_y']])[0]-.409).min())
  log=(root/f"logs/{group}/{method}_{d['world']}_{d['trial']}_launch.log").read_text()
  ms=np.array([float(v) for v in re.findall(r'WHEEL_PLAN_MS ([\d.]+)',log)])
  error=[];control=[];command=[];rotating=0
  telemetry=file.with_name(file.stem+'_tracking.csv')
  if telemetry.exists():
   for r in csv.DictReader(telemetry.open()):
    metrics=dict(s.split('=',1) for s in r['metrics'].split(','))
    error.append(float(metrics['error']));control.append(float(metrics['control_ms']));command.append(float(metrics['v']))
  rows.append({'group':group,'world':d['world'],'trial':d['trial'],'status':d['status'],
   'time_s':d['elapsed_sim_s'],'path_m':d['path_length_m'],'sampled_circle_clearance_m':clearance,
   'mean_abs_odom_vx':float(np.mean(np.abs(a['odom_vx']))),
   'stopped_s':float(dt[np.abs(a['odom_vx'])<.03].sum()),
   'stopped_turning_s':float(dt[(np.abs(a['odom_vx'])<.03)&(np.abs(a['odom_wz'])>.1)].sum()),
   'trajectory_updates':log.count('Received trajectory') if method=='slash' else None,
   'planning_calls':len(ms),'planning_mean_ms':float(ms.mean()) if len(ms) else None,
   'planning_total_ms':float(ms.sum()) if len(ms) else None,
   'control_mean_ms':float(np.mean(control)) if control else None,
   'sampled_tracking_max_error_m':float(np.max(error)) if error else None,
   'max_command_v':float(np.max(command)) if command else None})
with (out/'runs.csv').open('w') as f:
 w=csv.DictWriter(f,rows[0].keys());w.writeheader();w.writerows(rows)
summary={}
for group in ['corridor_reuse','spatial','spatial_safe','spatial_final','spatial_guard','spatial_release','spatial_mppi']:
 part=[r for r in rows if r['group']==group];success=[r for r in part if r['status']=='success']
 summary[group]={'runs':len(part),'successes':len(success),'collisions':sum(r['status']=='collision' for r in part),
  'mean_success_time_s':float(np.mean([r['time_s'] for r in success])) if success else None,
  'min_sampled_circle_clearance_m':min((r['sampled_circle_clearance_m'] for r in part),default=None),
  'mean_stopped_s':float(np.mean([r['stopped_s'] for r in part])) if part else None}
(out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
