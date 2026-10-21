#!/usr/bin/env python3
"""Diagnostic plots from paired (identical-input) perception replay."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
p=argparse.ArgumentParser();p.add_argument('report',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
r=json.loads(a.report.read_text())
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','axes.unicode_minus':False,'font.size':10})
fig,axes=plt.subplots(2,2,figsize=(10,6.2),sharex=True,layout='constrained')
for key,label,color in [('baseline','修改前','#C66B2B'),('candidate','修改后','#246298')]:
 rr=[row for row in r['tracks'][key] if 3<=row['t']<=14];t=np.array([x['t'] for x in rr])
 values=[np.array([x['p'][0] for x in rr]),np.array([x['v'][0] for x in rr]),np.array([x['v'][1] for x in rr]),np.array([np.prod(x['frs_1_5s_size']) for x in rr])]
 for ax,y in zip(axes.flat,values):ax.plot(t,y,label=label,color=color,lw=1.3)
rr=r['tracks']['candidate'];t=[x['t'] for x in rr]
for ax,key,index in [(axes[0,0],'truth',0),(axes[0,1],'truth_v',0),(axes[1,0],'truth_v',1)]:
 ax.plot(t,[x[key][index] for x in rr],color='#353535',ls='--',lw=1,label='仿真真值')
for ax,title,ylabel in zip(axes.flat,['沿通道方向的位置','沿通道方向的速度','横穿方向的速度','未来 1.5 s 位置 FRS 面积'],['$x$ / m','$v_x$ / (m/s)','$v_y$ / (m/s)','面积 / m²']):
 ax.set(title=title,ylabel=ylabel,xlim=(3,14));ax.grid(alpha=.16);ax.spines[['top','right']].set_visible(False)
for ax in axes[1]:ax.set_xlabel('时间 / s')
axes[0,0].legend(frameon=False,ncol=3,fontsize=8)
fig.savefig(a.output,dpi=240)
