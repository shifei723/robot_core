#!/usr/bin/env python3
from pathlib import Path
import json,hashlib,runpy,collections,math
R=Path(__file__).resolve().parent;CORE=R.parents[1]
m=json.loads((R/'manifest.json').read_text());p=json.loads((R/'progress.json').read_text());d=json.loads((R/'results.json').read_text())
checks=[]
def check(name,value,detail=None):checks.append({'name':name,'pass':bool(value),'detail':detail})
key=lambda x:(x['kind'],x['method'],x.get('world',x.get('phase')),x['trial'])
check('all_planned_runs_completed',collections.Counter(map(key,m['tasks']))==collections.Counter(map(key,p['completed'])))
check('all_runs_analyzed',len(d['runs'])==len(m['tasks'])==21)
x=runpy.run_path(str(R/'run_experiments.py'));check('frozen_sources_unchanged',x['digests']()==m['source_sha256'])
for name,digest in m['binary_sha256'].items():check('binary_'+Path(name).name,hashlib.sha256((CORE/name).read_bytes()).hexdigest()==digest)
old=json.loads((CORE/'output/navigation_study_20261116_v3/manifest.json').read_text())['source_sha256']
common=[s for s in m['source_sha256'] if s.startswith('navigation/scan_planner/') and s in old]
changed=[s for s in common if old[s]!=m['source_sha256'][s]]
check('same_navigation_sources_as_indoor_v3',not changed,{'compared_files':len(common),'changed':changed})
for r in d['runs']:
 tag=str(key(r));path=Path(r['source_result']);check(tag+' raw_result_exists',path.is_file())
 if not path.is_file():continue
 raw=json.loads(path.read_text())
 check(tag+' goal_tolerance',raw['goal_tolerance_m']==.3)
 check(tag+' success_has_true_arrival',not r['success'] or (r['arrival_s'] is not None and r['final_goal_distance_m']<.3 and not r['collision']))
 check(tag+' failed_duration_not_arrival',r['success'] or r.get('arrival_s') is None)
 if r['kind']=='dynamic':
  check(tag+' no_truth_to_perception',raw['truth_input_to_perception'] is False)
  check(tag+' eight_moving_entities',raw['models_seen']==8)
  check(tag+' sensor_range',raw['sensor_range_m']==5.)
  check(tag+' scheduled_start',raw['start_sim_s'] is None or 8<=raw['start_sim_s']<=8.3)
  check(tag+' no_unknown_motion_overlap',r['unknown_motion_overlap_packets']==0)
  check(tag+' finite_coast_duration',r['max_coast_age_s']<=1.1,r['max_coast_age_s'])
 for k in ['path_m','min_clearance_m','final_goal_distance_m']:
  check(tag+' finite_'+k,r.get(k) is None or math.isfinite(r[k]))
check('no_source_replacements',len({key(r) for r in d['runs']})==len(d['runs']))
report={'purpose':'Data/protocol integrity only; passing does not mean every navigation trial succeeded.', 'passed':sum(c['pass'] for c in checks),'total':len(checks),'all_passed':all(c['pass'] for c in checks),'checks':checks}
(R/'verification.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(report['passed'],'/',report['total'],'integrity checks passed')
for c in checks:
 if not c['pass']:print('FAILED',c)
raise SystemExit(0 if report['all_passed'] else 1)
