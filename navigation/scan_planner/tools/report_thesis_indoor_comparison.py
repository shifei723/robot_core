#!/usr/bin/env python3
"""Matched CV/FRS hard/soft indoor comparison. Includes every scheduled run.
Duration, stops, and collision measurements are evaluated from recorded samples.
Failed runs are right-censored, never assigned a successful arrival time.
"""
import argparse,json,hashlib
from pathlib import Path
import numpy as np

METHODS=['cv_hard','frs_hard','frs_soft']
SPEEDS=[.18,.22,.26]
def evaluate(run):
 s=json.loads((run/'summary.json').read_text());trace=json.loads((run/'trace.json').read_text());p=trace['poses']
 t=np.array([r['t'] for r in p]);dt=np.diff(t);goal=s.get('goal_sent_at',0.)
 wait=sensor=longest=span=0.;long_episodes=0;last_in=False
 for a,b,d in zip(p,p[1:],dt):
  dynamic=a['status'] in ('DYNAMIC_WAIT','DYNAMIC_SHIELD') and abs(a['v'])<.01
  if dynamic:wait+=d;span+=d;longest=max(longest,span)
  else:
   if span>=10:long_episodes+=1
   span=0
  if a['status'] in ('WAIT_TRACK','WAIT_MAP','WAIT_ODOM','WAIT_STATIC_PRIOR') and a['t']>=goal:sensor+=d
 if span>=10:long_episodes+=1
 path=sum(np.linalg.norm(np.array(b['pose'][:2])-a['pose'][:2]) for a,b in zip(p,p[1:]))
 timing=[r['latency_ms'] for r in trace['decisions'] if 'latency_ms' in r]
 return dict(run=run.name,parameters=s['experiment_parameters'],reached=s['reached'],collision=s['collision'],error=s['error'],
  arrival_s=s.get('arrival_time'),task_s=s['arrival_time']-goal if s['reached'] else None,
  observed_s=s['duration'],timeout_s=180.,dynamic_wait_s=wait,sensor_wait_s=sensor,longest_dynamic_stop_s=longest,
  long_wait_episodes=long_episodes,min_clearance_m=s['min_clearance'],path_length_m=float(path),height=s.get('height_validation'),
  combined_pass=s.get('combined_pass',False),onsets=s['multi_onsets'],
  source_hashes={k:v for k,v in s.items() if k.endswith('_sha256')},trace_sha256=hashlib.sha256((run/'trace.json').read_bytes()).hexdigest())
def main():
 parser=argparse.ArgumentParser();parser.add_argument('root',type=Path);parser.add_argument('--output',type=Path,required=True);a=parser.parse_args()
 rows=[]
 for speed in SPEEDS:
  for method in METHODS:
   path=a.root/f'{method}_{speed:.2f}'
   if not (path/'summary.json').exists():raise RuntimeError(f'Missing scheduled run: {path}')
   row=evaluate(path);row.update(method=method,speed=speed);rows.append(row)
 assert not any(r['error'] for r in rows),'Infrastructure failure: do not report as algorithm failure'
 for field in ['controller_sha256','planner_sha256','simulator_sha256','tracker_sha256']:
  assert len({r['source_hashes'][field] for r in rows})==1,field
 for r in rows:
  assert r['parameters']['spatial_weight']==0 and r['parameters']['frs_weight']==2
 groups={}
 for method in METHODS:
  sub=[r for r in rows if r['method']==method];successful=[r for r in sub if r['reached']]
  groups[method]=dict(n=len(sub),reached=sum(r['reached'] for r in sub),collision_runs=sum(r['collision'] for r in sub),
    completed_without_collision=sum(r['reached'] and not r['collision'] for r in sub),
    task_s_mean_successful=float(np.mean([r['task_s'] for r in successful])) if successful else None,
    arrival_s_mean_successful=float(np.mean([r['arrival_s'] for r in successful])) if successful else None,
    wait_s_mean=float(np.mean([r['dynamic_wait_s'] for r in sub])),
    longest_dynamic_stop_s_max=max(r['longest_dynamic_stop_s'] for r in sub),
    min_clearance_m=min(r['min_clearance_m'] for r in sub),long_wait_episodes=sum(r['long_wait_episodes'] for r in sub))
 report={'design':'3 methods x 3 speeds, one recorded run per condition. Same five speed candidates, perception, map, height controller and route. Spatial FRS cost disabled in all conditions. 180 s horizon. Sequential execution. No significance claim.',
 'methods':{'cv_hard':'CV with position/velocity uncertainty; 3 s hard constraint','frs_hard':'learned-acceleration FRS; 3 s hard constraint','frs_soft':'3 s FRS risk, weight 2; near-term CV hard horizon 0.5+abs(v)/0.3 s'},
 'long_wait_definition':'continuous DYNAMIC_WAIT/SHIELD with abs(v)<0.01 m/s for >=10 s',
 'timing_definition':'arrival_s since simulation start; task_s since first goal publication; incomplete runs censored at 180 s',
 'rows':rows,'groups':groups}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
 print(json.dumps(groups,ensure_ascii=False,indent=2))
if __name__=='__main__':main()
