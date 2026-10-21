#!/usr/bin/env python3
"""Paired perception replay: identical raycast depth and recorded ego motion.
Truth only generates sensors and computes metrics; trackers receive world points.
Baseline comes from the requested git checkpoint, not a reimplemented algorithm.
"""
import argparse,importlib.util,json,subprocess,sys,tempfile
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'navigation/scan_planner/src/perception/geometric_obstacles'))
from geometric_obstacles import core
from geometric_obstacles.static_prior import StaticPrior

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);p.add_argument('--baseline',default='4f7ceb0');p.add_argument('--frs-evaluator',type=Path);a=p.parse_args()
 path='navigation/scan_planner/src/perception/geometric_obstacles/geometric_obstacles/core.py'
 src=subprocess.check_output(['git','show',f'{a.baseline}:{path}'],cwd=ROOT,text=True)
 with tempfile.TemporaryDirectory() as tmp:
  f=Path(tmp)/'baseline.py';f.write_text(src);spec=importlib.util.spec_from_file_location('baseline',f);old=importlib.util.module_from_spec(spec);sys.modules['baseline']=old;spec.loader.exec_module(old)
 modules={'baseline':old,'candidate':core};trackers={k:m.Tracker() for k,m in modules.items()}
 trace=json.loads((a.run/'trace.json').read_text());summary=json.loads((a.run/'summary.json').read_text())
 
 if summary['case'] not in ('crossing','continuous_crossing','turnaround'):
  raise ValueError('Replay supports only the single moving-box scenarios')
 times=np.array([r['t'] for r in trace['poses']]);ego=np.array([r['pose'] for r in trace['poses']]);zs=np.array([r['base_z'] for r in trace['poses']])
 v,u=np.mgrid[:180,:320];K=np.array([[220.,0,160],[0,220.,90],[0,0,1.]])
 rays0=np.stack(((u-160)/220,(v-90)/220,np.ones_like(u)),axis=-1).reshape(-1,3)
 body=np.array([[0.,0,1],[-1,0,0],[0,-1,0]])
 xx,zz=np.meshgrid(np.arange(-2,8.01,.04),np.arange(.08,2.01,.04))
 prior=StaticPrior(np.concatenate([np.column_stack((xx.ravel(),np.full(xx.size,y),zz.ravel())) for y in [-1.55,1.55]]),.05,.10)
 rows={k:[] for k in modules}
 for sensor in trace['sensors']:
  t=sensor['t']
  if t>15:break
  pose=np.array([np.interp(t,times,ego[:,i]) for i in range(3)]);origin=np.r_[pose[:2],np.interp(t,times,zs)+.1678]
  R=Rotation.from_euler('z',pose[2]).as_matrix()@body; rays=rays0@R.T
  depth=np.minimum(np.full(len(rays),8.),np.divide(-origin[2],rays[:,2],out=np.full(len(rays),np.inf),where=rays[:,2]<-1e-7))
  motion_start=0 if summary['case']=='continuous_crossing' else 3
  y=float(np.clip(-.95+.22*max(0,t-motion_start),-.95,.95));vy=.22 if motion_start<t<motion_start+1.9/.22 else 0.
  if summary['case']=='turnaround' and t>8:y=float(np.clip(.15-.22*(t-8),-.95,.95));vy=-.22 if t<13 else 0.
  center=np.array([1.8,y]);boxes=list(summary['static_boxes'])+[(np.r_[center-.22,0],np.r_[center+.22,.9])]
  for lo,hi in boxes:
   with np.errstate(divide='ignore',invalid='ignore'):aa=(lo-origin)/rays;bb=(hi-origin)/rays
   near=np.max(np.minimum(aa,bb),axis=1);far=np.min(np.maximum(aa,bb),axis=1)
   depth=np.minimum(depth,np.where((far>=np.maximum(near,0))&(near>0),near,np.inf))
  depth=np.clip(depth.reshape(180,320)*1000,0,65534).astype(np.uint16).astype(float)/1000
  T=np.eye(4);T[:3,:3]=R;T[:3,3]=origin
  points,pixels,_=core.project(depth,K,T,core.Config());known=prior.known(points);points,pixels=points[~known],pixels[~known]
  for key,tr in trackers.items():
   tracks=tr.process(points,pixels,t)
   tr.retire(t,depth,K,T)
   obs=[q for q in tracks if q.observed]
   if not obs:continue
   best=min(obs,key=lambda q:np.linalg.norm(q.x[:2]-center))
   rows[key].append({'t':t,'id':best.id,'p':best.x[:2].tolist(),'v':best.x[2:].tolist(),'truth':center.tolist(),'truth_v':[0,vy],'state':best.state,'radius':best.radius,'covariance_diagonal':np.diag(best.P).tolist(),'visibility_corrected':getattr(best,'visibility_corrected',False)})
 if a.frs_evaluator:
  for rr in rows.values():
   if len({r['id'] for r in rr}) != 1:
    raise ValueError('FRS paired evaluator requires one uninterrupted track identity')
   # Feed exact same production predictor for each estimator; no visual clipping.
   text=''.join(' '.join(map(str,[r['t'],*r['p'],*r['v'],*r['covariance_diagonal']]))+'\n' for r in rr)
   prediction=subprocess.check_output([str(a.frs_evaluator.resolve())],input=text,text=True)
   for r,line in zip(rr,prediction.splitlines()):
    values=list(map(float,line.split()));r['frs_1_5s_center']=values[1:3];r['frs_1_5s_size']=values[3:5]
 metrics={}
 for key,rr in rows.items():
  moving=[r for r in rr if 4<r['t']<7.9];errors=np.array([np.array(r['v'])-r['truth_v'] for r in moving])
  metrics[key]={'pre_turn_velocity_rmse':np.sqrt(np.mean(errors**2,axis=0)).tolist(),'pre_turn_max_abs_vx':max(abs(r['v'][0]) for r in moving),'at_6_5':min(rr,key=lambda r:abs(r['t']-6.5)),'dynamic_frames':sum(r['state']=='dynamic' for r in rr), 'mean_frs_1_5s_area':float(np.mean([np.prod(r['frs_1_5s_size']) for r in moving])) if a.frs_evaluator else None}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps({'baseline_commit':a.baseline,'run':str(a.run.resolve()),'metrics':metrics,'tracks':rows},indent=2));print(json.dumps(metrics,indent=2))
if __name__=='__main__':main()
