#!/usr/bin/env python3
"""Frozen prospective experiment scheduler. Serial simulation, all outcomes retained."""
import datetime,hashlib,json,os,subprocess,sys,time
from pathlib import Path
R=Path(__file__).resolve().parent;CORE=R.parents[1];GROUP='retest_20261116_current'
TASKS=[]
for j,(world,phase,speed) in enumerate(zip([18,36,54],[0,7,14],[.18,.22,.26])):
 for trial in range(2):
  for method in (['mppi','slash'] if (j+trial)%2==0 else ['slash','mppi']):
   TASKS.append({'kind':'static','world':world,'method':method,'trial':trial})
 for method in [['mppi','frs_soft','frs_hard'],['frs_hard','mppi','frs_soft'],['frs_soft','frs_hard','mppi']][j]:
  TASKS.append({'kind':'dynamic','phase':phase,'method':method,'trial':0})

def source_files():
 roots=[CORE/'navigation/scan_planner',CORE/'navigation/benchmarks']
 for root in roots:
  for p,dirs,files in os.walk(root):
   dirs[:]=[d for d in dirs if d not in ['.workspaces','results','logs','assets','reference_results','__pycache__','.pytest_cache','plot_fonts']]
   for name in files:
    q=Path(p)/name
    if q.suffix not in ['.pyc','.png','.gif','.pdf'] and q.is_file():yield q

def digests():return {str(p.relative_to(CORE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source_files())}

def main():
 manifest=json.loads((R/'manifest.json').read_text());frozen=manifest['source_sha256']
 (R/'formal').mkdir(exist_ok=True);(R/'logs').mkdir(exist_ok=True)
 state={'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'completed':[],'planned':len(TASKS)}
 for i,t in enumerate(TASKS):
  if digests()!=frozen:raise RuntimeError('Frozen sources changed; refusing to mix versions.')
  tag=f'{i+1:02d}_{t["kind"]}_{t["method"]}_'+str(t.get('world',t.get('phase',t.get('speed'))))
  env=os.environ.copy();log=R/'logs'/f'{tag}.log'
  if t['kind']=='static':
   env.update(BARN_RUN_SET=GROUP,BARN_GOAL_TOLERANCE='.3')
   cmd=[str(CORE/'navigation/benchmarks/static_barn/run.sh'),t['method'],str(t['world']),str(t['trial'])]
   output=CORE/f'navigation/benchmarks/static_barn/results/{GROUP}/{t["method"]}_world_{t["world"]}_trial_{t["trial"]}.json'
  elif t['kind']=='dynamic':
   env['DYNA_GROUP']=GROUP
   cmd=[str(CORE/'navigation/benchmarks/dynamic_dynabarn/run.sh'),t['method'],str(t['phase']),str(t['trial'])]
   output=CORE/f'navigation/benchmarks/dynamic_dynabarn/results/{GROUP}/{t["method"]}_phase_{t["phase"]}_trial_{t["trial"]}/result.json'
  if output.exists():raise FileExistsError(output)
  state['active']={'index':i+1,**t,'started_wall':time.time()};(R/'progress.json').write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n')
  print(f'START {i+1}/{len(TASKS)} {t}',flush=True);begin=time.monotonic()
  with log.open('w') as f:done=subprocess.run(cmd,cwd=CORE,env=env,stdout=f,stderr=subprocess.STDOUT)
  if output.exists():
   d=json.loads(output.read_text());status=d.get('status',('infrastructure_error' if d.get('error') else 'collision' if d.get('collision') else 'success' if d.get('combined_pass') else 'timeout'))
  else:status='infrastructure_no_result'
  state['completed'].append({**t,'status':status,'exit_code':done.returncode,'wall_s':time.monotonic()-begin,'source_result':str(output),'log':str(log)})
  print(f'END {i+1}/{len(TASKS)} {status}',flush=True)
  (R/'progress.json').write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n')
  with (R/'analysis.log').open('w') as f:subprocess.run([sys.executable,str(R/'analyze.py')],cwd=CORE,stdout=f,stderr=subprocess.STDOUT,check=True)
 state.pop('active',None);state['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat()
 (R/'progress.json').write_text(json.dumps(state,indent=2,ensure_ascii=False)+'\n')
if __name__=='__main__':main()
