#!/usr/bin/env python3
"""Render measured closed-loop logs; world truth is drawn only for evaluation."""
import argparse,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Circle,Rectangle
from PIL import Image


def main():
    p=argparse.ArgumentParser();p.add_argument('run',type=Path);args=p.parse_args();root=args.run
    summary=json.loads((root/'summary.json').read_text());trace=json.loads((root/'trace.json').read_text())
    frames=np.load(root/'snapshots.npz',allow_pickle=True)['frames']
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':10,'axes.spines.top':False,
                         'axes.spines.right':False,'axes.unicode_minus':False,'pdf.fonttype':42})
    poses=trace['poses'];t=np.array([r['t'] for r in poses]);xy=np.array([r['pose'][:2] for r in poses]);vel=np.array([r['v'] for r in poses]);gap=np.array([r['clearance'] for r in poses])
    result='到达目标' if summary['reached'] else ('故障停车测试' if summary['case']=='dropout' else '未到达目标')
    fig,axs=plt.subplots(2,2,figsize=(12,7),dpi=90,gridspec_kw={'width_ratios':[1.2,1]})
    fig.subplots_adjust(left=.07,right=.97,bottom=.09,top=.89,hspace=.4,wspace=.24)
    def draw(f):
        for ax in axs.flat:ax.clear()
        tm=f['t'];k=int(np.argmin(abs(t-tm)));ax=axs[0,0]
        for lo,hi in summary['static_boxes']:
            ax.add_patch(Rectangle(lo[:2],hi[0]-lo[0],hi[1]-lo[1],fc='#d5d8dc',ec='#70757a',lw=.6))
        pts=f['map'];keep=pts[:,2]>.13 if len(pts) else []
        if len(pts):ax.scatter(pts[keep,0],pts[keep,1],s=1,color='#4a6c7d',alpha=.4,label='在线占据地图')
        ax.plot(xy[:k+1,0],xy[:k+1,1],color='#176b9b',lw=1.5,label='实际轨迹')
        ax.scatter([3.],[0.],marker='*',s=80,color='#269765',label='目标')
        for ob in poses[k]['truth']:
            half=ob['half'];pos=ob['p'];ax.add_patch(Rectangle(np.array(pos)-half,2*half[0],2*half[1],fc='#e9a963',ec='#a86427',label='障碍物真值（评估）'))
        dec=f['decision']
        if dec:
            for track in dec.get('tracks',[]):
                for box in track['boxes'][4::10]:
                    _,x,y,hx,hy=box
                    ax.add_patch(Rectangle((x-hx,y-hy),2*hx,2*hy,fc='#da5e58',alpha=.12,ec='#b9433e',lw=.8))
            path=np.array(dec['chosen_path']);ax.plot(path[:,0],path[:,1],'--',color='#49966b',lw=1,label='FRS 检查路径')
        x,y,yaw=f['pose'];ax.add_patch(Circle((x,y),.43,fc='#287caa',alpha=.18,ec='#176b9b'))
        ax.arrow(x,y,.35*np.cos(yaw),.35*np.sin(yaw),width=.025,color='#176b9b')
        ax.set(xlim=(-.5,3.6),ylim=(-1.8,1.8),xlabel='x / m',ylabel='y / m',title='(a) 在线建图、跟踪与 FRS 投影')
        ax.set_aspect('equal');ax.legend(loc='lower right',fontsize=7,frameon=False)
        ax=axs[0,1];depth=f['depth'];ax.imshow(depth,vmin=300,vmax=3000,cmap='viridis')
        filtered=f['filtered']
        if filtered is not None:
            overlay=np.zeros((*filtered.shape,4));overlay[filtered==65535]=[.95,.2,.15,.8];ax.imshow(overlay)
        ax.set(title='(b) 模拟深度图与动态射线掩码（红色）',xlabel='像素 u',ylabel='像素 v')
        ax=axs[1,0];ax.plot(t[:k+1],vel[:k+1],color='#176b9b',label='执行速度')
        d=[d for d in trace['decisions'] if d['t']<=tm]
        if d:ax.plot([a['t'] for a in d],[a['scale']*.3 for a in d],color='#b9433e',alpha=.6,lw=.9,label='推进倍率 × 0.3')
        ax.set(xlim=(0,summary.get('duration',t[-1])),ylim=(-.015,.33),xlabel='时间 / s',ylabel='速度 / (m/s)',title='(c) 闭环执行速度与 FRS 调节');ax.grid(alpha=.15);ax.legend(fontsize=8,frameon=False)
        ax=axs[1,1];ax.plot(t[:k+1],gap[:k+1],color='#269765');ax.axhline(0,color='#b9433e',ls='--',lw=1)
        ax.set(xlim=(0,summary.get('duration',t[-1])),ylim=(min(-.1,float(gap.min())-.1),max(1.5,float(gap.max())+.1)),xlabel='时间 / s',ylabel='包络净距 / m',title='(d) 真值评估：机器人与障碍物净距');ax.grid(alpha=.15)
        tracks=f['tracks'];health='等待感知' if tracks is None else ('感知正常' if tracks['healthy'] else '感知异常：'+tracks['status'])
        fig.suptitle(f'深度感知 → 建图 → FRS → 导航闭环验证   t = {tm:.1f} s\n{health}  |  {poses[k]["status"]}  |  本次结果：{result}',fontsize=12)

        fig.canvas.draw()
        return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:,:,:3].copy())
    # Representative dynamic detection; otherwise middle of the trial.
    candidates=[f for f in frames if f['tracks'] and any(o['state']==1 for o in f['tracks']['objects'])]
    key=candidates[len(candidates)//2] if candidates else frames[len(frames)//2]
    draw(key);fig.savefig(root/'pipeline_snapshot.png',dpi=180);fig.savefig(root/'pipeline_snapshot.pdf')
    stride=max(1,int(np.ceil(len(frames)/90)))
    selected=list(frames[::stride])
    if selected[-1] is not frames[-1]: selected.append(frames[-1])
    images=[draw(f) for f in selected]
    durations=[max(20,int((b["t"]-a["t"])*1000)) for a,b in zip(selected[:-1],selected[1:])]+[1000]
    images[0].save(root/'pipeline.gif',save_all=True,append_images=images[1:],duration=durations,loop=0,optimize=False)
    draw(frames[-1]);fig.savefig(root/'pipeline_final.png',dpi=180)
    plt.close(fig)
    print(root/'pipeline.gif')

if __name__=='__main__':main()
