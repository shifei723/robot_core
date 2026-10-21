#!/usr/bin/env python3
"""Evaluate multi-target perception and navigation; truth is evaluation-only."""
import argparse,json
from pathlib import Path
from collections import Counter,defaultdict
import numpy as np
from scipy.optimize import linear_sum_assignment


def evaluate(run):
    s=json.loads((run/'summary.json').read_text());r=json.loads((run/'trace.json').read_text())
    poses=r['poses'];start=s.get('encounter_onset',{}).get('t',float('inf'))
    times=np.array([a['t'] for a in poses]);track_times=np.array([a['t'] for a in r['tracks']])
    seen=defaultdict(set);dynamic=Counter();limiting=Counter();max_dynamic=0;max_frs=0;switches=Counter();last_id={};errors=[];false_dynamic=Counter();motion_limiting=Counter()
    def moving_truth(identity,t):
        onset=s['multi_onsets'].get(identity)
        spec=next(x for x in s['multi_specs'] if x['id']==identity)
        return onset is not None and onset['t']<=t<onset['t']+abs(spec['end_y']-spec['start_y'])/spec['speed']
    def truth(t):
        return poses[min(np.searchsorted(times,t),len(poses)-1)]['truth']
    def match(objs,t):
        targets=truth(t)
        if not objs:return {}
        cost=np.array([[np.linalg.norm(np.array(o['p'])-z['p']) for z in targets] for o in objs])
        i,j=linear_sum_assignment(cost)
        return {objs[a]['id']:targets[b]['id'] for a,b in zip(i,j) if cost[a,b]<.55}
    for sample in r['tracks']:
        if sample['t']<start:continue
        observed=[o for o in sample['objects'] if o['observed']]
        tt=sample['t']-sample['age'];matches=match(observed,tt)
        max_dynamic=max(max_dynamic,sum(o['state']==1 and o['id'] in matches and moving_truth(matches[o['id']],tt) for o in observed))
        truths={z['id']:z for z in truth(tt)}
        for o in observed:
            if o['id'] not in matches:continue
            identity=matches[o['id']];seen[identity].add(o['id'])
            if identity in last_id and last_id[identity]!=o['id']:switches[identity]+=1
            last_id[identity]=o['id']
            errors.append(np.array(o['p'])-truths[identity]['p'])
            if o['state']==1:
                if moving_truth(identity,tt):dynamic[identity]+=1
                else:false_dynamic[identity]+=1
    for d in r['decisions']:
        if d['t']<start:continue
        max_frs=max(max_frs,len(d['tracks']))
        if d['stale'] or d['scale']>=1:continue
        i=np.searchsorted(track_times,d['t'],side='right')-1
        if i<0:continue
        sample=r['tracks'][i];matches=match(sample['objects'],sample['t']-sample['age'])
        identity=matches.get(d['nominal_obstacle'])
        if identity:
            limiting[identity]+=1
            if moving_truth(identity,d['t']):motion_limiting[identity]+=1
    stop=wait=slow=sensor=longest=span=0.
    for a,b in zip(poses,poses[1:]):
        dt=max(0,b['t']-max(a['t'],start))
        if not dt or a['status']=='GOAL_REACHED':continue
        if abs(a['v'])<.01:stop+=dt;span+=dt;longest=max(longest,span)
        else:span=0.
        if a['status']=='DYNAMIC_WAIT':wait+=dt
        if a['status']=='DYNAMIC_SLOW':slow+=dt
        if a['status'] in ('WAIT_TRACK','WAIT_MAP','WAIT_ODOM','WAIT_STATIC_PRIOR'):sensor+=dt
    ids=[x['id'] for x in s['multi_specs']]
    gaps={identity:min(float(np.linalg.norm(np.maximum(np.abs(np.array(a['pose'][:2])-o['p'])-o['half'],0))-.43)
         for a in poses for o in a['truth'] if o['id']==identity) for identity in ids}
    result={'run':run.name,'policy':s['experiment_parameters']['timing_policy'],'parameters':s['experiment_parameters'],
      'reached':s['reached'],'collision':s['collision'],'error':s['error'],'onsets':s['multi_onsets'],'multi_specs':s['multi_specs'],
      'duration_s':s['duration'],'arrival_s':s.get('arrival_time'),'after_onset_s':s.get('arrival_time',s['duration'])-start if np.isfinite(start) else None,
      'stopped_s':stop,'longest_stop_s':longest,'dynamic_wait_s':wait,'dynamic_slow_s':slow,'sensor_wait_s':sensor,
      'min_clearance_m':s['min_clearance'],'target_clearances_m':gaps,
      'max_simultaneous_observed_dynamic':max_dynamic,'max_simultaneous_frs':max_frs,
      'false_dynamic_observations_while_stationary':{i:false_dynamic[i] for i in ids},
      'limiting_decisions_while_moving':{i:motion_limiting[i] for i in ids},
      'dynamic_observations_by_truth':{i:dynamic[i] for i in ids},'limiting_decisions_by_truth':{i:limiting[i] for i in ids},
      'associated_ids_by_truth':{i:sorted(seen[i]) for i in ids},'id_switches_by_truth':{i:switches[i] for i in ids},
      'matched_position_rmse_xy':np.sqrt(np.mean(np.square(errors),axis=0)).tolist() if errors else None,
      'shield_decisions':sum(bool(d['shield']) for d in r['decisions']),
      'source_hashes':{k:v for k,v in s.items() if k.endswith('_sha256')},
      'source_summary':str((run/'summary.json').resolve())}
    result['pass']=bool(s['reached'] and not s['collision'] and not s['error'] and max_dynamic>=2 and max_frs>=2 and all(dynamic[i]>0 and motion_limiting[i]>0 for i in ids))
    return result


def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    rows=[evaluate(x) for x in a.runs]
    report={'scope':'Real ROS perception, mapping, FRS and controller; raycast depth, ideal pose, planar kinematics. Three objects exist from startup; no truth input to perception.',
      'criterion':'Reached without collision, all three targets observed as dynamic and limiting a slowdown/wait decision, at least two simultaneous observed dynamic targets and FRS tracks.',
      'association_evaluation':'Hungarian distance association to truth, 0.55 m gate, evaluation only. ID switch counts include reacquisition; not a full MOT benchmark.',
      'all_pass':all(x['pass'] for x in rows),'runs':rows,'group_means':{}}
    for policy in ['soft','hard']:
        group=[x for x in rows if x['policy']==policy]
        if group:report['group_means'][policy]={'count':len(group),'passed':sum(x['pass'] for x in group),**{k:float(np.mean([x[k] for x in group if x[k] is not None])) if any(x[k] is not None for x in group) else None for k in ['after_onset_s','stopped_s','longest_stop_s','dynamic_wait_s','sensor_wait_s']}}
    if all(k in report['group_means'] for k in ['soft','hard']):
        report['soft_reduction_fraction']={key:1-report['group_means']['soft'][key]/report['group_means']['hard'][key]
          for key in ['after_onset_s','stopped_s','longest_stop_s']
          if report['group_means']['soft'][key] is not None and report['group_means']['hard'][key] is not None and report['group_means']['hard'][key]>0}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n');print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
