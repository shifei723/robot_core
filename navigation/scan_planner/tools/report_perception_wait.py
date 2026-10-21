#!/usr/bin/env python3
"""Separate sensor/track waiting from intentional FRS yielding in recorded runs."""
import argparse,json
from collections import Counter
from pathlib import Path
import numpy as np


def evaluate(run):
    s=json.loads((run/'summary.json').read_text());r=json.loads((run/'trace.json').read_text())
    start=s.get('encounter_onset',{}).get('t',0.)
    intervals=[]
    for a,b in zip(r['poses'],r['poses'][1:]):
        if not a['status'].startswith('WAIT_'):continue
        if intervals and intervals[-1]['state']==a['status'] and abs(intervals[-1]['end']-a['t'])<1e-6:
            intervals[-1]['end']=b['t']
        else:intervals.append({'start':a['t'],'end':b['t'],'state':a['status'],'pose':a['pose']})
    for interval in intervals:
        tracks=[t for t in r['tracks'] if interval['start']<=t['t']<=interval['end']]
        interval['duration']=interval['end']-interval['start']
        interval['perception_status_counts']=dict(Counter(t['status'] for t in tracks))
        interval['unobserved_track_ids']=sorted({o['id'] for t in tracks if not t['healthy'] for o in t['objects'] if not o['observed']})
    initial_static={}
    # Offline truth association only: evaluate spurious speed before each target starts.
    for spec in s.get('multi_specs',[]):
        onset=s.get('multi_onsets',{}).get(spec['id'],{}).get('t',float('inf'))
        position=np.array(spec['start']) if 'start' in spec else np.array([spec['x'],spec['start_y']])
        matches=[o for t in r['tracks'] if t['t']-t['age']<onset for o in t['objects'] if o['observed'] and np.linalg.norm(np.array(o['p'])-position)<.55]
        initial_static[spec['id']]={'observations':len(matches),'dynamic_observations':sum(o['state']==1 for o in matches),
          'peak_estimated_speed_m_s':max((float(np.linalg.norm(o['v'])) for o in matches),default=None)}
    return {'run':run.name,'reached':s['reached'],'collision':s['collision'],'arrival_s':s.get('arrival_time'),
      'height_validation':s.get('height_validation'),'min_clearance_m':s.get('min_clearance'),
      'node_sha256':s.get('perception_node_sha256'),'simulator_sha256':s.get('simulator_sha256'),
      'experiment_parameters':s.get('experiment_parameters'),'route_waypoints':s.get('route_waypoints'),
      'sensor_wait_after_first_onset_s':sum(max(0,i['end']-max(start,i['start'])) for i in intervals),
      'wait_intervals':intervals,'before_motion_estimation':initial_static,'tracker_sha256':s['tracker_sha256']}


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    report={'scope':'WAIT_* time from actual controller states, separate from DYNAMIC_WAIT; initial static truth association is evaluation-only with 0.55 m distance gate.', 'runs':[evaluate(x) for x in a.runs]}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(report,ensure_ascii=False,indent=2))

if __name__=='__main__':main()
