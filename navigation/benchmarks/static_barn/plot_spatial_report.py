#!/usr/bin/env python3
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import xml.etree.ElementTree as ET
root=Path(__file__).parent;out=root/'results/spatial_comparison'
rows=list(csv.DictReader((out/'runs.csv').open()))
methods=[('corridor_reuse','Before','#E69F00'),('spatial_release','Optimized','#0072B2'),('spatial_mppi','MPPI','#777777')]
fig,ax=plt.subplots(figsize=(7,3.8));x=np.arange(3);width=.24
world_rows=[]
for j,(group,name,color) in enumerate(methods):
 vals=[];errors=[]
 for world in [0,6,12]:
  subset=[r for r in rows if r['group']==group and int(r['world'])==world]
  times=[float(r['time_s']) for r in subset if r['status']=='success']
  vals.append(np.mean(times) if times else np.nan);errors.append(np.std(times,ddof=1) if len(times)>1 else 0.)
  if subset:
   world_rows.append({'method':name,'world':world,'runs':len(subset),'successes':len(times),
    'mean_success_time_s':vals[-1],'std_success_time_s':errors[-1],
    'min_sampled_circle_clearance_m':min(float(r['sampled_circle_clearance_m']) for r in subset),
    'mean_stopped_s':np.mean([float(r['stopped_s']) for r in subset]),
    'mean_path_m':np.mean([float(r['path_m']) for r in subset])})
 bars=ax.bar(x+(j-1)*width,vals,width,color=color,label=name,yerr=errors,capsize=3,error_kw={'linewidth':.8})
 for bar,val,err in zip(bars,vals,errors):
  if np.isfinite(val):ax.text(bar.get_x()+bar.get_width()/2,val+err+.6,f'{val:.1f}',ha='center',fontsize=9)
ax.set_xticks(x,['BARN 0','BARN 6','BARN 12']);ax.set_ylabel('Navigation time (s)');ax.set_ylim(0,60)
ax.legend(ncol=3,frameon=False,loc='upper center');ax.spines[['right','top']].set_visible(False)
fig.tight_layout();fig.savefig(out/'navigation_time.png',dpi=220);fig.savefig(out/'navigation_time.pdf');plt.close(fig)
with (out/'world_summary.csv').open('w') as f:
 w=csv.DictWriter(f,world_rows[0].keys());w.writeheader();w.writerows(world_rows)
fig,axs=plt.subplots(1,3,figsize=(12,7))
for ax,world in zip(axs,[0,6,12]):
 scene=ET.parse(root/f'upstream/jackal_helper/worlds/BARN/world_{world}.world').find('world')
 for m in scene.findall('model'):
  c=m.find('.//collision/geometry/cylinder')
  if c is not None:ax.add_patch(Circle(list(map(float,m.findtext('pose').split()))[:2],float(c.findtext('radius')),color='.75',lw=0))
 for group,name,color in methods:
  method='mppi' if group=='spatial_mppi' else 'slash'
  for trial in ([0] if group=='corridor_reuse' else [0,1,2]):
   p=root/f'results/{group}/{method}_world_{world}_trial_{trial}.csv'
   if not p.exists():continue
   a=np.genfromtxt(p,delimiter=',',names=True)
   ax.plot(a['world_x'],a['world_y'],color=color,lw=1.4,alpha=1. if trial==0 else .35,label=name if trial==0 else None)
 ax.scatter([-2.25,-2.25],[3,13],c=['green','red'],marker='*',s=60,zorder=5)
 ax.add_patch(Circle((-2.25,13),1,fill=False,color='red',ls='--',alpha=.3))
 ax.set(xlabel='World x (m)',ylabel='World y (m)',title=f'BARN {world}',aspect='equal',ylim=(2,14));ax.legend(fontsize=8);ax.grid(alpha=.15)
fig.tight_layout();fig.savefig(out/'trajectories.png',dpi=200);fig.savefig(out/'trajectories.pdf')
print(json.dumps(world_rows,indent=2))
