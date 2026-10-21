#!/usr/bin/env python3
"""Publication views of recorded perception/navigation; no synthetic trajectories.
Uses actual Slash MJCF for an orthographic top-view sprite, plotted in metres.
"""
import argparse, json, hashlib, math, os
os.environ.setdefault("MUJOCO_GL", "egl")
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import Rectangle, Circle
from matplotlib.lines import Line2D
from PIL import Image
import mujoco
from render_pointcloud_interaction import Render

BLUE='#246298'; ORANGE='#C66B2B'; GREEN='#287953'; GRAY='#778791'
font_manager.fontManager.addfont('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':10,'axes.unicode_minus':False,'pdf.fonttype':3,'axes.linewidth':.6})

def latest(rows,t):
    return max((r for r in rows if r['t']<=t),key=lambda r:r['t'],default=None)

def sprite(r):
    r.pose({'pose':[0,0,0],'base_z':.2822})
    r.camera.lookat[:]=[0,0,0]; r.camera.azimuth=90; r.camera.elevation=-90
    r.camera.distance=2; r.camera.orthographic=1
    r.model.vis.global_.fovy=1.2; r.model.vis.global_.orthographic=1; r.model.vis.global_.ipd=0
    r.renderer.update_scene(r.data,camera=r.camera,scene_option=r.option)
    r.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
    rgb=r.renderer.render().copy()
    # Geometry-only alpha via segmentation, preserving white robot panels.
    r.renderer.enable_segmentation_rendering(); seg=r.renderer.render(); r.renderer.disable_segmentation_rendering()
    alpha=(seg[:,:,0]>=0).astype(np.uint8)*255
    return Image.fromarray(np.dstack([rgb,alpha]))

def draw_robot(ax,im,row,alpha=1):
    x,y,yaw=row['pose']; rotated=im.rotate(math.degrees(yaw),resample=Image.Resampling.BICUBIC)
    # Orthographic view spans fovy vertically; renderer aspect is 1800/1100.
    h=1.2; w=h*1800/1100
    ax.imshow(rotated,extent=[x-w/2,x+w/2,y-h/2,y+h/2],alpha=alpha,zorder=9)

def panel(r,im,t,i,out):
    row=latest(r.trace['poses'],t); t=row['t']; dec=latest(r.trace['decisions'],t)
    fig,ax=plt.subplots(figsize=(7.2,5.3)); fig.subplots_adjust(left=.095,right=.98,bottom=.21,top=.87)
    ax.set(xlim=(-.65,3.65),ylim=(-2.15,1.72),aspect='equal',xlabel='$x$ / m',ylabel='$y$ / m')
    ax.set_xticks(np.arange(0,3.6,.5)); ax.set_yticks([-2,-1.5,-1,-.5,0,.5,1,1.5])
    ax.tick_params(labelsize=8,length=3,color='#83909A'); ax.spines[['top','right']].set_visible(False)
    ax.grid(color='#EDF0F2',lw=.5,zorder=0)
    pts=r.cloud(t)
    floor=pts[pts[:,2]<.08]; walls=pts[pts[:,2]>=.08]
    for cloud,color,size in [(floor,'#DAE1E5',.6),(walls,GRAY,2.5)]:
        xy=np.unique(np.round(cloud[:,:2],2),axis=0)
        ax.scatter(*xy.T,s=size,c=color,linewidths=0,rasterized=True,zorder=1)
    history=[q for q in r.trace['poses'] if q['t']<=t]
    xy=np.array([q['pose'][:2] for q in history]); ax.plot(*xy.T,c=BLUE,lw=1.8,zorder=5)
    ghosts=[]
    for q in history:
        if not ghosts or np.linalg.norm(np.array(q['pose'][:2])-ghosts[-1]['pose'][:2])>.55: ghosts.append(q)
    for j,q in enumerate(ghosts):
        if np.linalg.norm(np.array(q['pose'][:2])-row['pose'][:2])>.42: draw_robot(ax,im,q,.16+.035*j)
    # Raw estimated position sets; no body-radius inflation implied.
    if dec and not dec.get('stale'):
        for tr in dec.get('tracks',[]):
            for k,col in [(14,'#F6D9AB'),(9,'#EAB568'),(4,'#D58B37')]:
                tau,x,y,hx,hy=tr['boxes'][k]
                ax.add_patch(Rectangle((x-hx,y-hy),2*hx,2*hy,facecolor=col,edgecolor='#BC7B28',lw=.8,alpha=.40,zorder=2))
            ax.text(2.85,1.25,'位置 FRS\n0.5 / 1.0 / 1.5 s',ha='center',va='center',fontsize=8,color='#936018',bbox={'facecolor':'white','edgecolor':'none','alpha':.88,'pad':2},zorder=12)
    truth=np.array([q['truth'][0]['p'] for q in history if q.get('truth')]); hist=[]
    for p in truth:
        if not hist or np.linalg.norm(p-hist[-1])>.26:hist.append(p)
    if len(truth):
        ax.plot(*truth.T,color=ORANGE,lw=1,alpha=.65,zorder=4)
        for j,p in enumerate(hist[:-1]):ax.add_patch(Rectangle(p-.22,.44,.44,facecolor=ORANGE,edgecolor=ORANGE,lw=.6,alpha=.08+.018*j,zorder=3))
        p=truth[-1]; ax.add_patch(Rectangle(p-.22,.44,.44,facecolor=ORANGE,edgecolor='#8E421C',lw=1,zorder=7))
        old=latest(history,max(0,t-.8))
        delta=p-np.array(old['truth'][0]['p'])
        if np.linalg.norm(delta)>.03:
            ax.annotate('',xy=p+delta/np.linalg.norm(delta)*.57,xytext=p+delta/np.linalg.norm(delta)*.24,arrowprops={'arrowstyle':'-|>','color':ORANGE,'lw':1.4},zorder=10)
    for x,c in [(0,BLUE),(3,GREEN)]:ax.add_patch(Circle((x,0),.16,fill=False,edgecolor=c,lw=1.2,zorder=6))
    ax.text(0,-.48,'起点',ha='center',color=BLUE,fontsize=9)
    ax.text(3,-.48,'目标',ha='center',color=GREEN,fontsize=9)
    draw_robot(ax,im,row)
    label=['观测分离与前向预测','障碍物折返与停车让行','恢复通行与到达停车'][i-1]
    fig.text(.095,.955,f'({chr(96+i)}) {label}',fontsize=12,va='top')
    fig.text(.095,.899,f'$t$ = {t:.2f} s     $v$ = {row["v"]:.2f} m/s',fontsize=9,color='#41515D')
    if i==2:
        ax.annotate('Slash 停车让行',xy=row['pose'][:2],xytext=(-.38,.85),fontsize=9,color=BLUE,arrowprops={'arrowstyle':'-','color':BLUE,'lw':.8},zorder=12)
        ax.text(1.8,.62,'横穿后折返',ha='center',fontsize=9,color=ORANGE)
    if i==3:
        error=np.linalg.norm(np.array(row['pose'][:2])-[3,0])
        ax.text(2.65,.80,f'已到达并停车\n位置误差 {error*100:.1f} cm',ha='center',fontsize=9,color=GREEN)
        ax.text(1.8,-1.35,'障碍物已停止',ha='center',fontsize=8,color=ORANGE)
    handles=[Line2D([],[],marker='.',linestyle='',color=GRAY,label='静态点云'),Line2D([],[],color=BLUE,lw=1.8,label='机器人已执行轨迹'),Rectangle((0,0),1,1,fc=ORANGE,label='障碍物与历史拖影'),Rectangle((0,0),1,1,fc='#EAB568',alpha=.5,label='在线位置 FRS')]
    fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.53,.055),ncol=2,frameon=False,fontsize=8,columnspacing=2)
    stem=out/f'ch5_frs_topview_{i}';fig.savefig(stem.with_suffix('.pdf'));fig.savefig(stem.with_suffix('.png'),dpi=450);plt.close(fig)
    return {'file':stem.name,'t':t,'speed':row['v'],'status':row['status'],'decision_time':dec['t'] if dec else None,'frs_slices_s':[.5,1,1.5] if dec and dec.get('tracks') else []}

def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    r=Render(a.run,Path(__file__).resolve().parents[4]/'slash_robot_control'); im=sprite(r)
    panels=[panel(r,im,t,i,a.output) for i,t in enumerate([4.,9.,r.trace['poses'][-1]['t']],1)]
    r.renderer.close()
    manifest={'source':str(a.run.resolve()),'trace_sha256':hashlib.sha256((a.run/'trace.json').read_bytes()).hexdigest(),'panels':panels,'limits':'Software closed loop, planar kinematics, ideal odometry and raycast depth. Entity poses are truth overlays; FRS is online estimation. Only 0.5/1/1.5 s position slices displayed, not entire 3 s horizon or hard safety envelope. No future map or poses in earlier panels.'}
    (a.output/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2))
    ims=[Image.open(a.output/f'ch5_frs_topview_{i}.png').convert('RGB') for i in range(1,4)]
    sheet=Image.new('RGB',(720,490*3),'white')
    for i,img in enumerate(ims):img.thumbnail((720,490));sheet.paste(img,((720-img.width)//2,490*i))
    sheet.save(a.output/'preview.png')
if __name__=='__main__':main()
