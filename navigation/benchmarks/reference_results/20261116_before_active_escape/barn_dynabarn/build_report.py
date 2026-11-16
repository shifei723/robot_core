#!/usr/bin/env python3
"""Review actual current-version runs; never replace missing arrivals with termination times."""
from pathlib import Path
import json,csv,subprocess,sys,math
import numpy as np
import matplotlib
matplotlib.use('pgf')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from matplotlib.lines import Line2D
import xml.etree.ElementTree as ET
R=Path(__file__).resolve().parent;CORE=R.parents[1];F=R/'figures';F.mkdir(exist_ok=True)
plt.rcParams.update({'pgf.texsystem':'xelatex','pgf.rcfonts':False,'pgf.preamble':r'\usepackage{fontspec}\usepackage{xeCJK}\setmainfont{TeX Gyre Termes}\setCJKmainfont{FandolSong-Regular.otf}', 'font.family':'serif','font.size':10.5,'axes.labelsize':10.5,'xtick.labelsize':10.5,'ytick.labelsize':10.5,'legend.fontsize':10.5,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False})
C={'mppi':'#777777','slash':'#0072B2','frs_soft':'#0072B2','frs_hard':'#D55E00'}
L={'mppi':'MPPI','slash':'本文平面方法','frs_soft':'FRS-Soft','frs_hard':'FRS-Hard'}
S={'success':'到达','collision':'碰撞','timeout':'超时','navigation_aborted':'导航中止','estimated_goal_only':'仅估计到达','infrastructure_late_ready':'未按时就绪'}
def arr(p):return np.atleast_1d(np.genfromtxt(p,delimiter=',',names=True))
def save(fig,name):
 fig.savefig(F/(name+'.pdf'),bbox_inches='tight');plt.close(fig)
 subprocess.run(['pdftoppm','-scale-to','1500','-singlefile','-png',str(F/(name+'.pdf')),str(F/name)],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=True)
def legend(fig,methods):fig.legend(handles=[Line2D([0],[0],color=C[m],lw=1.5,label=L[m],ls='--' if m=='mppi' else '-') for m in methods],loc='upper center',ncol=len(methods),frameon=False)
def statics(rows):
 fig,axs=plt.subplots(1,3,figsize=(7.0,5.3))
 for ax,w in zip(axs,[18,36,54]):
  scene=ET.parse(CORE/f'navigation/benchmarks/static_barn/upstream/jackal_helper/worlds/BARN/world_{w}.world').find('world')
  for m in scene.findall('model'):
   c=m.find('.//collision/geometry/cylinder')
   if c is not None:ax.add_patch(Circle(list(map(float,m.findtext('pose').split()))[:2],float(c.findtext('radius')),fc='.8',ec='none'))
  for r in rows:
   if r['kind']!='static' or r['world']!=w or not Path(r['source_result']).exists():continue
   a=arr(Path(r['source_result']).with_suffix('.csv'))
   ax.plot(a['world_x'],a['world_y'],c=C[r['method']],lw=1.2,ls='--' if r['method']=='mppi' else '-',alpha=1 if r['trial']==0 else .5)
   if len(a) and not r['success']:ax.plot(a['world_x'][-1],a['world_y'][-1],'x',c=C[r['method']])
  ax.scatter([-2.25,-2.25],[3,13],c=['#009E73','#B2182B'],marker='*',s=40,zorder=5)
  ax.add_patch(Circle((-2.25,13),.3,fill=False,ec='#B2182B',ls='--',lw=.7))
  ax.set(xlim=(-4.7,.3),ylim=(2.7,13.7),aspect='equal',xlabel='$x$ / m',title=f'BARN {w}')
 axs[0].set_ylabel('$y$ / m');legend(fig,['mppi','slash']);fig.tight_layout(rect=(0,0,1,.93));save(fig,'static_trajectories')
 fig,axs=plt.subplots(1,2,figsize=(7.,3.2))
 for ax,key,lab in zip(axs,['arrival_s','min_clearance_m'],['成功到达时间 / s','最小采样净距 / m']):
  for j,m in enumerate(['mppi','slash']):
   for i,w in enumerate([18,36,54]):
    pp=[r for r in rows if r['kind']=='static' and r['method']==m and r['world']==w];v=[r[key] for r in pp if r.get(key) is not None];x=i+(j-.5)*.24
    if v:ax.scatter(x+np.linspace(-.025,.025,len(v)),v,s=23,c=C[m],marker='s' if m=='mppi' else 'o');ax.plot([x-.055,x+.055],[np.mean(v)]*2,c=C[m],lw=1.5)
    if key=='arrival_s':ax.text(x,.02,f'{sum(r["success"] for r in pp)}/{len(pp)}',transform=ax.get_xaxis_transform(),ha='center',va='bottom',fontsize=9)
  ax.set_xticks(range(3),['18','36','54']);ax.set(xlabel='BARN 地图编号',ylabel=lab);ax.grid(axis='y',alpha=.2,lw=.5)
  if key=='arrival_s':ax.set_ylim(bottom=0)
 legend(fig,['mppi','slash']);fig.tight_layout(rect=(0,0,1,.87));save(fig,'static_metrics')
def dynamics(rows):
 pp=[r for r in rows if r['kind']=='dynamic'];fig,axs=plt.subplots(1,2,figsize=(7.,3.7))
 for j,m in enumerate(['mppi','frs_hard','frs_soft']):
  for i,phase in enumerate([0,7,14]):
   r=next((r for r in pp if r['method']==m and r['phase']==phase),None)
   if r is None:continue
   x=i+(j-1)*.24;t=r.get('termination_s')
   if t is not None:
    axs[0].scatter(x,t,c=C[m],s=40,marker='o' if r['success'] else 'x',linewidths=1.3)
    axs[0].annotate(S.get(r['status'],'其他失败'),(x,t),xytext=(0,6+(j%2)*11),textcoords='offset points',ha='center',fontsize=8.5)
   if r.get('min_clearance_m') is not None:axs[1].scatter(x,r['min_clearance_m'],c=C[m],s=35,marker='o' if r['success'] else 'x')
 for ax,lab in zip(axs,['到达或终止时间 / s','最小动态采样净距 / m']):
  ax.set_xticks(range(3),['0','7','14']);ax.set(xlabel='障碍运动初相位 / s',ylabel=lab);ax.grid(axis='y',lw=.5,alpha=.2);ax.margins(y=.25)
 axs[0].set_ylim(bottom=0);axs[1].axhline(0,c='#B2182B',ls='--',lw=.7);legend(fig,['mppi','frs_hard','frs_soft']);fig.tight_layout(rect=(0,0,1,.87));save(fig,'dynamic_outcomes')
 fig,axs=plt.subplots(3,1,figsize=(7.,5.5),sharex=True)
 import runpy
 geom=runpy.run_path(str(CORE/'navigation/benchmarks/dynamic_dynabarn/dynabarn_ros2/scripts/map_geometry.py'))
 walls=geom['static_wall_points'](CORE/'navigation/benchmarks/dynamic_dynabarn/dynabarn_ros2/worlds/official_world_2.world')
 for ax,phase in zip(axs,[0,7,14]):
  ax.scatter(walls[:,0],walls[:,1],s=1,c='.8',rasterized=True,zorder=0)
  for r in pp:
   if r['phase']!=phase or not Path(r['source_result']).exists():continue
   a=arr(Path(r['source_result']).parent/'robot.csv')
   ax.plot(a['world_x'],a['world_y'],c=C[r['method']],ls='--' if r['method']=='mppi' else '-',lw=1.2)
   if len(a) and not r['success']:ax.plot(a['world_x'][-1],a['world_y'][-1],'x',c=C[r['method']])
  ax.plot([12,-9],[0,0],'*',c='#009E73',ms=6);ax.set(ylabel='$y$ / m',title=f'初相位 {phase} s',xlim=(-11,14),ylim=(-4,4));ax.grid(alpha=.13,lw=.5)
 axs[-1].set_xlabel('$x$ / m');legend(fig,['mppi','frs_hard','frs_soft']);fig.tight_layout(rect=(0,0,1,.94));save(fig,'dynamic_trajectories')
def dynamic_progress(rows):
 import runpy
 helper=runpy.run_path(str(R/'analyze.py'));position=helper['position_at']
 fig,axs=plt.subplots(2,1,figsize=(7.,4.4),sharex=True)
 for r in rows:
  if r['kind']!='dynamic' or r['phase']!=0 or not Path(r['source_result']).exists():continue
  p=Path(r['source_result']).parent;d=json.loads((p/'result.json').read_text());a=arr(p/'robot.csv');objects={}
  for o in csv.DictReader((p/'objects.csv').open()):objects.setdefault(o['name'],[]).append([float(o[k]) for k in ['sim_s','world_x','world_y']])
  objects={k:np.array(v) for k,v in objects.items()};g=[]
  for tt,x,y in zip(a['sim_elapsed_s'],a['world_x'],a['world_y']):
   g.append(min(np.linalg.norm(np.array([x,y])-position(v,tt+d['start_sim_s'],k,0))-.834 for k,v in objects.items()))
  for ax,v in zip(axs,[a['odom_vx'],g]):
   ax.plot(a['sim_elapsed_s'],v,c=C[r['method']],lw=1.2,ls='--' if r['method']=='mppi' else '-')
   if len(a) and not r['success']:ax.plot(a['sim_elapsed_s'][-1],v[-1],'x',c=C[r['method']],ms=6)
 axs[0].set_ylabel('前向速度 / (m/s)');axs[1].set_ylabel('动态采样净距 / m');axs[1].set_xlabel('目标下发后时间 / s');axs[1].axhline(0,c='#B2182B',ls='--',lw=.7)
 for ax in axs:ax.grid(alpha=.15,lw=.5)
 legend(fig,['mppi','frs_hard','frs_soft']);fig.tight_layout(rect=(0,0,1,.91));save(fig,'phase0_speed_clearance')

def main():
 d=json.loads((R/'results.json').read_text());assert d['completed']==d['planned']==21, 'Batch incomplete';rows=d['runs'];statics(rows);dynamics(rows);dynamic_progress(rows)
 # Excel-compatible exported tables retain the complete raw run records.
 for src,dst in [('runs.csv','逐次结果.csv'),('summary.csv','汇总对照表.csv')]:
  (R/dst).write_text((R/src).read_text(),encoding='utf-8-sig')
 def num(v):return '--' if v is None else f'{v:.2f}'
 lines=[]
 for s in d['summary']:
  lines.append(('静态' if s['kind']=='static' else '动态')+' & '+L[s['method']]+f" & {s['successes']}/{s['runs']} & {s['collisions']} & {s['timeouts']} & {s['other_failures']} & "+num(s['mean_success_arrival_s'])+' & '+num(s['min_clearance_m'])+r' \\')
 text=r'''\documentclass[UTF8,a4paper,11pt]{ctexart}
\usepackage[margin=20mm]{geometry}
\setmainfont{TeX Gyre Termes}
\setCJKmainfont{FandolSong-Regular.otf}
\usepackage{graphicx,booktabs,hyperref,float}
\hypersetup{colorlinks=true,urlcolor=blue}
\title{当前算法 BARN 与 DynaBARN 基准场景复测}
\author{实验结果审阅稿}\date{2026年11月16日}
\begin{document}\maketitle
\section{实验方案}
本轮重新编译当前 robot\_core 源码，冻结版本后完成21次串行冷启动：静态 BARN 的18、36、54号地图，各方法每图两次，共12次；动态 DynaBARN 官方 world\_2，八个圆柱沿原轨迹运动，初相位分别为0、7、14秒，各方法每相位一次，共9次。三种初相位属于同一地图的不同交互条件，不作为三个独立地图。

平台为ROS2 Jazzy、Gazebo Harmonic与Jackal，使用模拟激光及轮速/IMU融合里程计；双方限速0.5 m/s、角速1 rad/s，真值到达门限0.3 m，120模拟秒超时。静态组关闭FRS，动态组使用共同静态墙背景及5 m激光。本文方法搜索预算0.10 s、启发式1.8、动态制动参数0.5 m/s$^2$，FRS预测3 s、时序权重2、空间权重0.5。硬组与软组仅切换时序风险策略。短时失观保留最多1 s运动预测，随后固定为未知区域，获得自由证据后清除。

MPPI保留倒车，本文控制器前向；两者内部地图及优化方式不同，故比较整套导航系统。真值仅进入仿真及离线评价。该协议属于公开基准场景子集上的统一对比，不是官方排行榜复现，也不验证Slash升降或真实深度相机定位精度。
\section{汇总结果}
到达时间仅统计成功运行；失败终止时间单独保留。碰撞以Gazebo接触为主指标，表内净距是轨迹采样时刻的圆包络几何量；动态插值避开原始周期回跳，不将采样净距代替连续接触检测。保留全部失败及运行异常，不以重跑成功结果替换。
\begin{table}[H]\centering\small
\begin{tabular}{llrrrrrr}\toprule
场景 & 方法 & 到达 & 碰撞 & 超时 & 其他失败 & 用时/s & 最小净距/m\\\midrule
'''+ '\n'.join(lines)+r'''
\bottomrule\end{tabular}\caption{当前算法基准场景对照结果}\end{table}
\section{静态通行}
\begin{figure}[H]\centering\includegraphics[width=.95\linewidth]{../figures/static_trajectories.pdf}\caption{三张静态地图的全部运行轨迹}\end{figure}
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../figures/static_metrics.pdf}\caption{静态通行时间与采样净距}\end{figure}
深色与浅色轨迹区分两次运行，叉号表示未成功运行的终点。时间图底部为成功数/运行数；原始点和均值横线仅描述本组样本，不作总体显著性检验。
\section{动态交互}
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../figures/dynamic_outcomes.pdf}\caption{三种初相位下的动态导航结果}\end{figure}
实心点表示到达，叉号表示失败；失败的横坐标条件与终止时刻均保留，不能将失败终止时间解释为更快到达。
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../figures/dynamic_trajectories.pdf}\caption{动态场景各方法的完整轨迹}\end{figure}
轨迹图用于对照路线与终止位置；各方法耗时不同，障碍同一时刻的相对位置亦不同。只有同一初相位下的运行属于相同运动脚本，不能跨初相位合成单条交互过程。
\begin{figure}[H]\centering\includegraphics[width=\linewidth]{../figures/phase0_speed_clearance.pdf}\caption{初相位0秒的速度与动态净距}\end{figure}
该条件展示初相位0秒下全部三种方法。结合停车与净距变化判断碰撞发生于运动还是保护停车阶段；具体目标匹配、末端速度与控制状态见failure\_diagnostics.json。保护停车本身不等价于能够避开继续接近的运动障碍，不能仅凭启用FRS宣称动态安全。
\section{数据与复核}
逐次结果及汇总表提供UTF-8 CSV；manifest.json记录源码、二进制、镜像及运行顺序，\path{source_snapshot.tar.gz}保存冻结源码，verification.json保存完整性检查。原始轨迹、感知跟踪及FRS决策位置由逐次结果的source\_result给出。与11月13日旧结果分组保存，不混入本轮均值或成功率。
\end{document}
'''
 p=R/'report';p.mkdir(exist_ok=True);(p/'benchmark_review.tex').write_text(text)
 with (p/'compile.log').open('w') as f:
  subprocess.run(['latexmk','-xelatex','-interaction=nonstopmode','-halt-on-error','benchmark_review.tex'],cwd=p,stdout=f,stderr=subprocess.STDOUT,check=True)
 print(json.dumps(d['summary'],ensure_ascii=False,indent=2))
if __name__=='__main__':main()
