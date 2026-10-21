#!/usr/bin/env python3
"""Compare timing policies without treating frame counts as stopped duration."""
import argparse,json
from pathlib import Path
import numpy as np


def evaluate(run):
    s=json.loads((run/'summary.json').read_text())
    tr=json.loads((run/'trace.json').read_text())
    rows=tr['poses'];onset=s.get('encounter_onset');p=s['experiment_parameters']
    start=onset['t'] if onset else float('inf')
    stopped=0.;longest=0.;span=0.;dynamic_wait=0.;slow=0.;total=0.;sensor_wait=0.
    for a,b in zip(rows,rows[1:]):
        dt=max(0.,b['t']-max(a['t'],start))
        if not dt or a['status']=='GOAL_REACHED':continue
        total+=dt
        if abs(a['v'])<.01:
            stopped+=dt;span+=dt;longest=max(longest,span)
        else:span=0.
        if a['status']=='DYNAMIC_WAIT':dynamic_wait+=dt
        if a['status']=='DYNAMIC_SLOW':slow+=dt
        if a['status'] in ('WAIT_TRACK','WAIT_MAP','WAIT_ODOM','WAIT_STATIC_PRIOR'):sensor_wait+=dt
    dynamic_gaps=[float(np.linalg.norm(np.maximum(np.abs(np.array(a['pose'][:2])-o['p'])-o['half'],0))-.43)
                  for a in rows if a['t']>=start for o in a['truth']]
    nominal_gap=None
    if onset:
        # Open-loop counterfactual only: keep measured onset forward speed and heading.
        # No perception truth is supplied to either tested controller.
        t=np.linspace(0,(p['encounter_side']+p.get('encounter_end_y',p['encounter_side']))/p['encounter_speed'],1001)
        xy=np.asarray(onset['pose'][:2])+t[:,None]*onset['v']*np.array([np.cos(onset['pose'][2]),np.sin(onset['pose'][2])])
        xy[:,0]=np.minimum(xy[:,0],3.)
        obstacle=np.column_stack((np.full_like(t,p['encounter_x']),np.clip(-p['encounter_side']+p['encounter_speed']*t,-p['encounter_side'],p.get('encounter_end_y',p['encounter_side']))))
        nominal_gap=float(np.min(np.linalg.norm(np.maximum(np.abs(xy-obstacle)-.22,0),axis=1)-.43))
    first_motion=min((a['t'] for a in tr['tracks'] if a['t']>=start and any(o['state']==1 for o in a['objects'])),default=None)
    result={'run':run.name,'parameters':p,'reached':s['reached'],'collision':s['collision'],'error':s['error'],
      'encounter_onset':onset,'arrival_s':s.get('arrival_time'),'interaction_to_arrival_s':None if not onset or not s.get('arrival_time') else s['arrival_time']-start,
      'first_dynamic_after_onset_s':None if first_motion is None else first_motion-start,
      'open_loop_min_gap_m':nominal_gap,'dynamic_min_gap_m':min(dynamic_gaps,default=None),'all_obstacles_min_gap_m':s['min_clearance'],
      'stopped_after_onset_s':stopped,'longest_stop_after_onset_s':longest,'dynamic_wait_after_onset_s':dynamic_wait,'dynamic_slow_after_onset_s':slow,
      'sensor_wait_after_onset_s':sensor_wait,
      'shield_decisions':sum(bool(d['shield']) for d in tr['decisions'] if d['t']>=start),
      'source_summary':str((run/'summary.json').resolve())}
    errors=[];covered=[]
    for sample in tr['tracks']:
        tt=sample['t']-sample.get('age',0.)
        if not onset or tt<start:continue
        age=tt-start;vy=p['encounter_speed'] if 0<age<(p['encounter_side']+p.get('encounter_end_y',p['encounter_side']))/p['encounter_speed'] else 0.
        truth=np.array([p['encounter_x'],np.clip(-p['encounter_side']+p['encounter_speed']*age,-p['encounter_side'],p.get('encounter_end_y',p['encounter_side'])),0.,vy])
        for obj in sample['objects']:
            if not obj['observed'] or 'covariance' not in obj:continue
            error=np.r_[obj['p'],obj['v']]-truth
            sigma=np.sqrt(np.maximum(np.diag(np.array(obj['covariance']).reshape(4,4)),0))
            errors.append(error);covered.append(np.abs(error)<=3*sigma)
    result['tracking_samples']=len(errors)
    result['tracking_rmse_xy_vxvy']=np.sqrt(np.mean(np.square(errors),axis=0)).tolist() if errors else None
    result['empirical_3sigma_axis_coverage']=np.mean(covered,axis=0).tolist() if covered else None
    result['interaction_valid']=bool(onset and onset['v']>.05 and nominal_gap is not None and nominal_gap<0 and first_motion is not None)
    result['safe_arrival']=bool(s['reached'] and not s['collision'] and s['error'] is None)
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rows=[evaluate(r) for r in a.runs]
    report={'scope':'Sensor-driven software closed loop, ideal pose and raycast depth, no real-hardware safety claim. Trigger at matched robot x; actual scheduling differs across runs.',
      'metrics':'Stopped time integrates |v|<0.01 m/s after encounter onset, before GOAL_REACHED. Open-loop conflict is a constant onset-speed counterfactual, not a separately simulated controller.',
      'all_pass':all(r['interaction_valid'] and r['safe_arrival'] for r in rows),'runs':rows}
    hard=[r for r in rows if r['parameters']['timing_policy']=='hard' and r['interaction_valid'] and r['safe_arrival']]
    soft=[r for r in rows if r['parameters']['timing_policy']=='soft' and r['interaction_valid'] and r['safe_arrival']]
    report['group_means']={}
    for name,group in [('hard',hard),('soft',soft)]:
        if group:report['group_means'][name]={'count':len(group),**{key:float(np.mean([r[key] for r in group])) for key in ['stopped_after_onset_s','longest_stop_after_onset_s','interaction_to_arrival_s','dynamic_wait_after_onset_s','sensor_wait_after_onset_s']}}
    if hard and soft:
        report['soft_reduction_fraction']={key:1-report['group_means']['soft'][key]/report['group_means']['hard'][key] for key in ['stopped_after_onset_s','longest_stop_after_onset_s','interaction_to_arrival_s'] if report['group_means']['hard'][key]>0}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
