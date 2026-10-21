#!/usr/bin/env python3
"""Evaluate indoor turning/height/navigation logs, with truth used offline only."""
import argparse,json
from pathlib import Path
import numpy as np
from report_multi_encounter import evaluate

p=argparse.ArgumentParser(description=__doc__)
p.add_argument('runs',type=Path,nargs='+');p.add_argument('--output',type=Path,required=True)
a=p.parse_args();results=[]
for run in a.runs:
    summary=json.loads((run/'summary.json').read_text())
    trace=json.loads((run/'trace.json').read_text());rows=trace['poses']
    result=evaluate(run);result['strict_simultaneous_multi_interaction_pass']=result.pop('pass')
    result['scene_reference']=summary.get('scene_reference')
    result['goal']=summary['goal']
    result['path_length_m']=sum(np.linalg.norm(np.array(b['pose'][:2])-c['pose'][:2]) for c,b in zip(rows,rows[1:]))
    result['total_turn_deg']=float(sum(abs(np.arctan2(np.sin(b['pose'][2]-c['pose'][2]),np.cos(b['pose'][2]-c['pose'][2]))) for c,b in zip(rows,rows[1:]))*180/np.pi)
    result['controller_states']=sorted(set(r['status'] for r in rows))
    result['all_three_perceived']=all(v>0 for v in result['dynamic_observations_by_truth'].values())
    result['interaction_targets']=sum(v>0 for v in result['limiting_decisions_while_moving'].values())
    result['pass']=bool(summary.get('combined_pass') and result['all_three_perceived'] and result['interaction_targets']>=2)
    results.append(result)
report={'scope':'Software closed loop with ideal pose and raycast depth; real ROS perception/map/planner/controller. No real sensor or leg contact physics.',
        'criterion':'Reached, no sampled entity collision, beam passed with >=3 cm clearance, leg height lowered below .215 m and restored, yaw excursion >1 rad, all 3 targets observed while moving, at least 2 moving targets limit navigation decisions. Simultaneous detection is separately reported, not required for sequential corridor encounters.',
        'runs':results,'all_pass':all(r['pass'] for r in results)}
a.output.parent.mkdir(parents=True,exist_ok=True)
a.output.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False,indent=2))

raise SystemExit(0 if report['all_pass'] else 1)
