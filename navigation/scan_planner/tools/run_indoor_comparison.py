#!/usr/bin/env python3
"""Prospective paired indoor study. Serial runs; no outcome-dependent selection."""
from pathlib import Path
import argparse,datetime,hashlib,json,os,runpy,subprocess,tarfile,time
CORE=Path(__file__).resolve().parents[3];R=CORE/'output/navigation_study_20261116_v3'
def source_files():
 for root in [CORE/'navigation/scan_planner',CORE/'navigation/benchmarks']:
  for p,dirs,files in os.walk(root):
   dirs[:]=[d for d in dirs if d not in ['.workspaces','results','logs','assets','reference_results','__pycache__','.pytest_cache','plot_fonts']]
   for name in files:
    q=Path(p)/name
    if q.suffix not in ['.pyc','.png','.gif','.pdf'] and q.is_file():yield q

def digests():
 return {str(p.relative_to(CORE)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(source_files())}

METHODS={'cv_hard':('cv','hard',0.),'frs_hard':('frs','hard',0.),'soft_time':('frs','soft',0.),'full':('frs','soft',.5)}
TASKS=[]
for i,speed in enumerate([.18,.22,.26]):
 for repeat in range(2):
  order=list(METHODS);order=order[i:]+order[:i]
  if repeat:order.reverse()
  for method in order:TASKS.append(dict(kind='indoor',speed=speed,method=method,trial=repeat))
def main():
 global R
 parser=argparse.ArgumentParser();parser.add_argument('--output-root',type=Path,default=R);args=parser.parse_args();R=args.output_root.resolve();R.mkdir(parents=True,exist_ok=True)
 frozen=digests();assert not (R/'progress.json').exists()
 manifest={'version':'indoor_repair_v3','previous_results':str(CORE/'output/navigation_study_20261116_v2'),'trace_compression':'gzip-lossless','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_sha256':frozen,'tasks':TASKS,'methods':METHODS,'scenario':'dual_crossing','beam_clearance':.60,'goal_height_tolerance':.008,'camera_fx_fy':170.,'record_period':.05,'search_budget':.1,'minimum_leg_height':.13,'search_heuristic':1.8,'robot_limits':dict(speed=.5,yaw=1.,acceleration=.5,braking=.5),'git_head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=CORE,text=True).strip(),'run_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
 (R/'manifest.json').write_text(json.dumps(manifest,indent=2))
 with tarfile.open(R/'source_snapshot.tar.gz','w:gz') as tf:
  for name in frozen:tf.add(CORE/name,arcname=name,recursive=False)
 state={'planned':len(TASKS),'completed':[],'started_utc':manifest['created_utc']}
 for index,task in enumerate(TASKS):
  assert digests()==frozen,'Sources changed during study'
  prediction,timing,spatial=METHODS[task['method']];out=R/'indoor'/f"{task['method']}_v{task['speed']:.2f}_r{task['trial']}";assert not out.exists()
  cmd=['/usr/bin/python3',str(CORE/'navigation/scan_planner/tools/validate_geometric_pipeline.py'),'--case','indoor_turn_height','--indoor-scenario','dual_crossing','--beam-clearance','.60','--goal-height-tolerance','.008','--record-period','.05','--compress-trace','--search-budget','.1','--minimum-leg-height','.13','--search-heuristic','1.8','--camera-focal','170','--static-prior','--height-spatial-tracking','--robot-speed','.5','--robot-yaw-rate','1','--robot-acceleration','.5','--robot-braking','.5','--prediction',prediction,'--timing-policy',timing,'--spatial-weight',str(spatial),'--frs-weight','2','--encounter-speed',str(task['speed']),'--coast-timeout','1','--duration','180','--domain','193','--output',str(out)]
  log=R/f"run_{index+1:02d}.log";state['active']={**task,'index':index+1,'started_wall':time.time()};(R/'progress.json').write_text(json.dumps(state,indent=2));print('START',index+1,task,flush=True);begin=time.monotonic()
  with log.open('w') as f:done=subprocess.run(cmd,cwd=CORE,stdout=f,stderr=subprocess.STDOUT)
  p=out/'summary.json';s=json.load(open(p)) if p.exists() else {}
  status='infrastructure_error' if not s or s.get('error') else 'collision' if s['collision'] else 'success' if s.get('combined_pass') else 'incomplete_height' if s.get('reached') else 'timeout'
  state['completed'].append({**task,'status':status,'exit_code':done.returncode,'wall_s':time.monotonic()-begin,'source_result':str(p),'log':str(log),'command':cmd});print('END',index+1,status,flush=True);(R/'progress.json').write_text(json.dumps(state,indent=2))
  if (R/'STOP_AFTER_CURRENT').exists():state['stopped_reason']='Diagnostic stop; completed results retained';break
 state.pop('active',None);state['finished_utc']=datetime.datetime.now(datetime.timezone.utc).isoformat();(R/'progress.json').write_text(json.dumps(state,indent=2))
if __name__=='__main__':main()
