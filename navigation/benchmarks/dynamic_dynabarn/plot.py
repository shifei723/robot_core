#!/usr/bin/env python3
"""Plot all attempts without mixing protocols or treating failure time as arrival."""
from pathlib import Path
import argparse,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
R=Path(__file__).resolve().parent
LABEL={'mppi':'MPPI','frs_soft':'基础 FRS','frs_joint':'联合滚动 FRS','frs_hard':'FRS 硬约束','mpc_d_cbf':'MPC-D-CBF'}
COLOR={'mppi':'#777777','frs_soft':'#0072B2','frs_joint':'#009E73','frs_hard':'#D55E00','mpc_d_cbf':'#CC79A7'}
def main():
 p=argparse.ArgumentParser();p.add_argument('--report',type=Path,default=R/'report.json');p.add_argument('--output',type=Path);args=p.parse_args()
 d=json.loads(args.report.read_text());out=args.output or args.report.parent;out.mkdir(parents=True,exist_ok=True)
 font=R/'plot_fonts/NotoSerifCJKSC.ttf'
 if not font.exists():
  from fontTools.ttLib import TTCollection
  font.parent.mkdir(exist_ok=True);TTCollection('/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',lazy=True).fonts[2].save(str(font))
 for f in ['/usr/share/fonts/truetype/croscore/Tinos-Regular.ttf',str(font)]:
  if Path(f).exists():fm.fontManager.addfont(f)
 plt.rcParams.update({'font.family':['Tinos','Noto Serif CJK SC'],'font.size':10.5,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False})
 rows=d['runs'];methods=[m for m in LABEL if m in d['summary']]
 def save(fig,name):
  fig.tight_layout();fig.savefig(out/(name+'.pdf'),bbox_inches='tight');fig.savefig(out/(name+'.png'),dpi=220,bbox_inches='tight');plt.close(fig)
 fig,ax=plt.subplots(figsize=(7.,3.6));bottom=np.zeros(len(methods));x=np.arange(len(methods))
 for state,label,c in [('success','到达','#0072B2'),('collision','碰撞','#D55E00'),('timeout','超时','#E69F00'),('other','启动或其他异常','#AAAAAA')]:
  n=[sum(r['method']==m and (r['status']==state if state!='other' else r['status'] not in ['success','collision','timeout']) for r in rows) for m in methods]
  ax.bar(x,n,bottom=bottom,color=c,label=label,width=.6)
  for i,v in enumerate(n):
   if v:ax.text(i,bottom[i]+v/2,str(v),ha='center',va='center',color='white')
  bottom+=n
 ax.set_xticks(x,[LABEL[m] for m in methods]);ax.set_ylabel('运行次数');ax.set_ylim(0,max(bottom)+1.);ax.legend(frameon=False,ncol=4,loc='upper center');save(fig,'outcomes')
 phases=sorted({r['phase_s'] for r in rows});fig,axs=plt.subplots(1,len(phases),figsize=(3.7*len(phases),3.4),squeeze=False)
 for ax,phase in zip(axs[0],phases):
  for i,m in enumerate(methods):
   rr=[r for r in rows if r['method']==m and r['phase_s']==phase];ok=[r['arrival_s'] for r in rr if r['success']]
   if ok:
    ax.scatter(np.full(len(ok),i)+np.linspace(-.06,.06,len(ok)),ok,c=COLOR[m],s=23);ax.plot([i-.15,i+.15],[np.mean(ok)]*2,c=COLOR[m],lw=1.4)
   ax.text(i,.97,f'{len(ok)}/{len(rr)}',ha='center',va='top',transform=ax.get_xaxis_transform())
  ax.set_xticks(range(len(methods)),[LABEL[m] for m in methods],rotation=15);ax.set_title(f'初相位 {phase:g} s');ax.set_ylabel('成功通行时间 / s');ax.grid(axis='y',alpha=.16);ax.margins(y=.25)
 save(fig,'success_times')
 fig,axs=plt.subplots(1,len(phases),figsize=(3.7*len(phases),3.5),squeeze=False)
 for ax,phase in zip(axs[0],phases):
  xy=[]
  for m in methods:
   rr=[r for r in rows if r['method']==m and r['phase_s']==phase]
   for j,r in enumerate(rr):
    p=Path(r['source_result']).parent/'robot.csv'
    a=np.atleast_1d(np.genfromtxt(p,delimiter=',',names=True))
    if not len(a):continue
    points=np.c_[a['world_x'],a['world_y']];xy.extend(points)
    ax.plot(*points.T,c=COLOR[m],lw=1.1,alpha=1 if j==0 else .5,label=LABEL[m] if j==0 else None)
    ax.plot(*points[-1],marker='x' if r['collision'] else 'o',ms=4,c=COLOR[m])
  raw=json.loads(Path(next(r for r in rows if r['phase_s']==phase)['source_result']).read_text());ax.scatter([raw['start_world'][0],raw['goal_world'][0]],[0,0],marker='*',s=35,c=['#009E73','#B2182B'])
  ax.set(xlabel='$x$ / m',ylabel='$y$ / m',title=f'初相位 {phase:g} s');ax.grid(alpha=.15);ax.legend(frameon=False,fontsize=8)
 save(fig,'trajectories')
 print('Figures:',out)
if __name__=='__main__':main()
