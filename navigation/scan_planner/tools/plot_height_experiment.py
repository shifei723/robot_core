#!/usr/bin/env python3
"""Plot measured validator output and make a low-lintel replay (no ROS needed)."""
import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
from matplotlib.animation import FuncAnimation, PillowWriter


def scene(ax, detour=False):
    if detour:
        ax.add_patch(Rectangle((5,-6),1.2,.6,color='slategray',label='Solid table'))
        ax.set(xlim=(4,7),ylim=(-7.6,-3.9))
    else:
        ax.add_patch(Rectangle((-3,-.1),2,.2,color='slategray',label='Solid wall'))
        ax.add_patch(Rectangle((1,-.1),2,.2,color='slategray'))
        ax.add_patch(Rectangle((-1,-.3),2,.6,color='orange',alpha=.25,label='Overhead beam'))
        ax.set(xlim=(-1.5,1.5),ylim=(-2.6,2.6))
    ax.set_aspect('equal');ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.grid(alpha=.2)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('folder',type=Path,help='A run_1 folder containing CSV and JSON results')
    args=p.parse_args();folder=args.folder
    cases=('normal','low','blocked','detour','slow_low','sensor_loss')
    fig,axs=plt.subplots(2,6,figsize=(21,9),layout='constrained')
    for i,case in enumerate(cases):
        data=np.genfromtxt(folder/f'{case}.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
        result=json.loads((folder/f'{case}.json').read_text())
        ax=axs[0,i];scene(ax,case=='detour');ax.plot(data['x'],data['y'],color='tab:blue',lw=2)
        ax.scatter(*result['start'],c='green',marker='o');ax.scatter(*result['goal'],c='red',marker='x')
        ax.set_title(case+'\n'+('PASS' if result['passed'] else 'FAIL'))
        ax=axs[1,i];ax.plot(data['t'],data['leg_height'],label='Measured leg')
        ax.plot(data['t'],data['height_cmd'],'--',alpha=.6,label='Command')
        ax.axhline(.14,c='gray',ls=':');ax.axhline(.36,c='gray',ls=':')
        if case!='detour':
            cap=result['beam_clearance_m']-.328-.0322-.03
            ax.axhline(cap,c='orange',ls='--',label='3 cm beam clearance')
        under=data['beam_overlap'].astype(bool)
        ax.fill_between(data['t'],.12,.39,where=under,color='orange',alpha=.15)
        ax.set(xlabel='Time (s)',ylabel='Leg height (m)',ylim=(.12,.39));ax.grid(alpha=.2)
        if i==0:ax.legend(fontsize=8)
    fig.suptitle('Measured ROS simulation — kinematic navigation, independent geometric collision checks',fontsize=15)
    fig.savefig(folder/'overview.png',dpi=140);plt.close(fig)

    data=np.genfromtxt(folder/'low.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
    result=json.loads((folder/'low.json').read_text())
    fig,(ax,side)=plt.subplots(1,2,figsize=(10,5),layout='constrained');scene(ax)
    ax.plot(data['x'],data['y'],color='lightsteelblue');ax.set_title('Measured low-beam crossing')
    circles=[Circle((0,-2),.25,fc='tab:blue',alpha=.65) for _ in range(2)]
    for c in circles:ax.add_patch(c)
    side.add_patch(Rectangle((-.3,result['beam_clearance_m']),.6,.3,fc='orange',alpha=.4))
    side.axhline(0,c='gray');side.set(xlim=(-2.6,2.6),ylim=(-.02,.95),xlabel='y (m)',ylabel='Height (m)')
    body=Rectangle((-2.43,.28),.86,.328,fc='tab:blue',alpha=.6);side.add_patch(body)
    leg,=side.plot([],[],'k-',lw=4);info=side.text(.03,.96,'',transform=side.transAxes,va='top')
    times=np.arange(0,float(data['t'][-1]),.2)
    indices=np.minimum(np.searchsorted(data['t'],times),len(data)-1)
    def update(frame):
        row=data[indices[frame]];x,y,z,yaw=[float(row[k]) for k in ('x','y','z','yaw')]
        for c,sign in zip(circles,(-1,1)):c.center=(x+sign*.18*np.cos(yaw),y+sign*.18*np.sin(yaw))
        body.set_xy((y-.43,z));leg.set_data([y,y],[.0322,z])
        info.set_text(f"t={row['t']:.1f} s | leg={row['leg_height']:.3f} m\n{row['state']}\nKinematic replay; side view is schematic")
        return circles+[body,leg,info]
    animation=FuncAnimation(fig,update,frames=len(indices),interval=100)
    animation.save(folder/'low_replay.gif',writer=PillowWriter(fps=10),dpi=100);plt.close(fig)
    print(folder/'overview.png');print(folder/'low_replay.gif')

if __name__=='__main__':main()
