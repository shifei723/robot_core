#!/usr/bin/env python3
from pathlib import Path
import csv,json
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
font_path=Path(__file__).resolve().parent/'plot_fonts/NotoSerifCJKSC.ttf'
if not font_path.exists():
 from fontTools.ttLib import TTCollection
 font_path.parent.mkdir(exist_ok=True)
 TTCollection('/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc',lazy=True).fonts[2].save(str(font_path))
for font in ["/usr/share/fonts/truetype/croscore/Tinos-Regular.ttf",str(Path(__file__).resolve().parent/"plot_fonts/NotoSerifCJKSC.ttf")]:
 if Path(font).exists():fm.fontManager.addfont(font)
R=Path(__file__).resolve().parent;report=json.loads((R/'report.json').read_text())
plt.rcParams.update({'font.family':['Tinos','Noto Serif CJK SC'],'font.size':10,'pdf.fonttype':42,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False})
methods=['mppi','frs_soft','frs_hard'];labels=['MPPI','FRS 软风险','FRS 硬约束'];colors=['#777777','#0072B2','#D55E00'];phases=[0,7,14]
fig,ax=plt.subplots(figsize=(8,4.1));x=np.arange(3);width=.24
for j,(method,label,color) in enumerate(zip(methods,labels,colors)):
 for i,phase in enumerate(phases):
  row=next((z for z in report['runs'] if z['method']==method and z['phase_s']==phase),None)
  if row is None or row['termination_s'] is None:continue
  value=row['termination_s'];b=ax.bar(x[i]+(j-1)*width,value,width,color=color,label=label if i==0 else None)[0]
  if row['status']!='success':b.set_hatch('///' if row['status']=='collision' else 'xx')
  state={'success':'到达','collision':'碰撞','timeout':'超时'}.get(row['status'],'失败')
  ax.text(b.get_x()+width/2,value+1,f'{value:.1f}\n{state}',ha='center',fontsize=9)
ax.set_xticks(x,['初相位 0 s','初相位 7 s','初相位 14 s']);ax.set_ylabel('到达或终止时间 / s');ax.set_ylim(0,145);ax.legend(frameon=False,ncol=3,loc='upper left')
fig.text(.5,.015,'斜线表示碰撞，交叉线表示超时；失败终止时间不作为到达时间。',ha='center',fontsize=9)
fig.tight_layout(rect=(0,.05,1,1));fig.savefig(R/'comparison_time.png',dpi=220);fig.savefig(R/'comparison_time.pdf');plt.close(fig)
fig,axes=plt.subplots(2,1,figsize=(7,5.4),sharex=True)
for method,label,color in zip(methods,labels,colors):
 row=next((z for z in report['runs'] if z['method']==method and z['phase_s']==0),None)
 if row is None:continue
 p=Path(row['source_result']).parent
 a=np.genfromtxt(p/'robot.csv',delimiter=',',names=True)
 c=np.genfromtxt(R/'derived'/(p.name+'_clearance.csv'),delimiter=',',names=True)
 axes[0].plot(a['sim_elapsed_s'],12-a['world_x'],color=color,label=label,lw=1.5)
 axes[1].plot(c['elapsed_s'],c['clearance_m'],color=color,lw=1.1)
 if row['collision']:
  axes[0].plot(a['sim_elapsed_s'][-1],12-a['world_x'][-1],marker='x',color=color,ms=7)
  axes[1].plot(c['elapsed_s'][-1],c['clearance_m'][-1],marker='x',color=color,ms=7)
axes[0].axhline(21,color='.4',ls='--',lw=.8);axes[0].set_ylabel('沿路线前进距离 / m');axes[0].legend(frameon=False,ncol=3,loc='upper left');axes[0].set_ylim(-.3,25)
axes[1].axhline(0,color='#A00000',ls='--',lw=.8);axes[1].set_ylabel('采样动态圆盘净距 / m');axes[1].set_xlabel('下发目标后的时间 / s');axes[1].set_ylim(-.2,3.5)
fig.tight_layout();fig.savefig(R/'phase0_progress_clearance.png',dpi=220);fig.savefig(R/'phase0_progress_clearance.pdf');plt.close(fig)

# All FRS control-state durations; MPPI has no equivalent state labels.
fig,ax=plt.subplots(figsize=(8,3.8))
rows=[r for method in ['frs_soft','frs_hard'] for phase in phases for r in report['runs'] if r['method']==method and r['phase_s']==phase]
state_groups=[('正常跟踪',['TRACKING'],'#0072B2'),('风险减速',['DYNAMIC_SLOW'],'#009E73'),('动态让行',['DYNAMIC_WAIT'],'#E69F00'),('近场保护',['DYNAMIC_SHIELD'],'#D55E00'),('感知等待',['PERCEPTION_WAIT'],'#CC79A7')]
y=np.arange(len(rows));left=np.zeros(len(rows))
for label,states,color in state_groups:
 values=np.array([sum(r['state_duration_s'].get(s,0) for s in states) for r in rows]);ax.barh(y,values,left=left,label=label,color=color);left+=values
other=np.array([max(0,r['termination_s']-left[i]) for i,r in enumerate(rows)])
ax.barh(y,other,left=left,color='.85',label='其他状态')
ax.set_yticks(y,[f"{'软风险' if r['method']=='frs_soft' else '硬约束'} / {r['phase_s']:g} s" for r in rows]);ax.invert_yaxis();ax.set_xlabel('累计状态时长 / s');ax.legend(frameon=False,ncol=3,loc='upper center',bbox_to_anchor=(.5,1.22));fig.tight_layout();fig.savefig(R/'frs_control_states.png',dpi=220);fig.savefig(R/'frs_control_states.pdf');plt.close(fig)
