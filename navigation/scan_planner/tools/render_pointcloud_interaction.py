#!/usr/bin/env python3
"""Render recorded depth-only experiment with the actual Slash mesh (MuJoCo EGL).
No dynamics are simulated here. Clouds are accumulated measured static depth;
robot poses, FRS and obstacle truth overlays come from the recorded run.
"""
import argparse,json,math,os,sys
from pathlib import Path
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src/planner/plan_manage/scripts'))
import leg_height_control as legs

class Render:
 def __init__(self,run,assets):
  self.trace=json.loads((run/'trace.json').read_text());self.result=json.loads((run/'summary.json').read_text())
  self.frames=list(np.load(run/'snapshots.npz',allow_pickle=True)['frames'])
  self.model=mujoco.MjModel.from_xml_path(str(assets/'assets/mjcf/scene_flat.xml'))
  self.data=mujoco.MjData(self.model)
  self.model.vis.global_.offwidth=1800;self.model.vis.global_.offheight=1200
  self.model.geom_group[self.model.geom('floor').id]=5
  self.model.vis.quality.shadowsize=1024;self.model.vis.quality.offsamples=4
  # White skybox keeps exported figures on a consistent paper background.
  for k in range(self.model.ntex):
   if self.model.tex_type[k]==mujoco.mjtTexture.mjTEXTURE_SKYBOX:
    first=self.model.tex_adr[k];size=self.model.tex_height[k]*self.model.tex_width[k]*self.model.tex_nchannel[k]
    self.model.tex_data[first:first+size]=255
  self.option=mujoco.MjvOption();self.option.geomgroup[3]=0;self.option.geomgroup[5]=0
  self.model.vis.headlight.ambient[:]=.3;self.model.vis.headlight.diffuse[:]=.3
  self.model.light_diffuse[:]*=.5;self.model.light_ambient[:]*=.5
  self.renderer=mujoco.Renderer(self.model,height=1100,width=1800,max_geom=50000)
  self.camera=mujoco.MjvCamera();self.camera.lookat[:]=[1.5,0,.15]
  self.camera.distance=5.9;self.camera.azimuth=140;self.camera.elevation=-52
  self.font='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
 def pose(self,row):
  x,y,yaw=row['pose'];z=row.get('base_z',.2822)
  mujoco.mj_resetDataKeyframe(self.model,self.data,0)
  a=yaw-math.pi/2;self.data.qpos[:7]=[x,y,z,math.cos(a/2),0,0,math.sin(a/2)]
  for side,ik,conv in [('l',legs.left_ik,legs.firmware_to_mujoco_left),('r',legs.right_ik,legs.firmware_to_mujoco_right)]:
   a,b,_=ik(0,z-.0322,0);f,b=conv(a,b);f2,b2=legs.five_bar_knee_angles_closed(f,b)
   if side=='r':f2=-f2
   for name,value in [(side+'f_joint1',f),(side+'b_joint1',b),(side+'f_joint2',f2),(side+'b_joint2',b2)]:self.data.joint(name).qpos[0]=value
  mujoco.mj_forward(self.model,self.data)
  if np.max(np.abs(self.data.efc_pos[:6]))>1e-5:raise ValueError('leg loop closure failed')
 def geom(self,kind,size,pos,color):
  sc=self.renderer.scene;g=sc.geoms[sc.ngeom]
  mujoco.mjv_initGeom(g,kind,np.array(size,float),np.array(pos,float),np.eye(3).ravel(),np.array(color,np.float32));sc.ngeom+=1
  g.specular=0;g.shininess=0
  return g
 def line(self,a,b,color,width=.008):
  g=self.geom(mujoco.mjtGeom.mjGEOM_CAPSULE,[1,1,1],[0,0,0],color)
  mujoco.mjv_connector(g,mujoco.mjtGeom.mjGEOM_CAPSULE,width,np.array(a,float),np.array(b,float))
 def cloud(self,t):
  allpoints=[]
  for f in self.frames:
   if f['t']>t:break
   d=f.get('filtered')
   if d is None:continue
   # Filtered frames are asynchronously received. Use raw current depth only for
   # static background and omit moving-object silhouette using stored mask where available.
   d=np.asarray(f['depth'])[::3,::3].astype(float)/1000
   vv,uu=np.mgrid[:180:3,:320:3];valid=(d>.3)&(d<3.)
   filtered=np.asarray(f['filtered'])[::3,::3]
   if filtered.shape==d.shape:valid&=(filtered!=65535)
   x,y,yaw=f['pose'];z=f.get('base_z',.2822)
   fx,fy,cx,cy=self.result.get('camera_intrinsics',[220.,220.,160.,90.])
   body=np.stack((d,-(uu-cx)*d/fx,-(vv-cy)*d/fy),axis=-1)[valid]
   R=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
   pts=body@R.T+[x,y,z+.1678]
   # Keep observed floor/walls. Object truth is overlaid separately, not inserted into map.
   pts=pts[pts[:,2]<.07]
   
   allpoints.append(pts)
  latest=max((f for f in self.frames if f['t']<=t),key=lambda f:f['t'],default=self.frames[0])
  if len(latest['map']):allpoints.append(np.asarray(latest['map']))
  if not allpoints:return np.empty((0,3))
  pts=np.concatenate(allpoints);_,idx=np.unique(np.round(pts/.055).astype(int),axis=0,return_index=True)
  pts=pts[idx];return pts[(pts[:,0]>-.5)&(pts[:,0]<3.8)&(pts[:,2]<1.3)]
 def image(self,t,index,title,clean=False,show_prediction=True):
  f=dict(max((f for f in self.frames if f['t']<=t),key=lambda f:f['t'],default=self.frames[0]))
  f['decision']=max((d for d in self.trace['decisions'] if d['t']<=t),key=lambda d:d['t'],default=None)
  f['tracks']=max((d for d in self.trace['tracks'] if d['t']<=t),key=lambda d:d['t'],default=None)
  actual=min(self.trace['poses'],key=lambda r:abs(r['t']-t));t=actual['t'];f['pose']=actual['pose'];f['base_z']=actual.get('base_z',f.get('base_z',.2822));self.pose(f)
  self.renderer.update_scene(self.data,camera=self.camera,scene_option=self.option)
  self.renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False
  # Neutral ground for legibility; it is explicitly a rendering plane, not mapped free space.
  self.geom(mujoco.mjtGeom.mjGEOM_BOX,[2.3,1.85,.005],[1.5,0,-.025],[.94,.95,.96,1])
  for p in self.cloud(t):
   color=[.47,.57,.63,1] if p[2]>.08 else [.70,.76,.80,1]
   self.geom(mujoco.mjtGeom.mjGEOM_SPHERE,[.012,0,0],p,color)
  # A few spatially separated real robot poses, not evenly timed stationary duplicates.
  past=[r for r in self.frames if r['t']<t-.6];ghost=[]
  for r in past:
   if not ghost or np.linalg.norm(r['pose'][:2]-ghost[-1]['pose'][:2])>.6:ghost.append(r)
  rgba=self.model.geom_rgba.copy();mat=self.model.geom_matid.copy()
  for j,r in enumerate(ghost[-5:]):
   if np.linalg.norm(r['pose'][:2]-f['pose'][:2])<.3:continue
   self.model.geom_matid[:]=-1;self.model.geom_rgba[:]=[.15,.42,.68,.12+.05*j]
   self.pose(r);mujoco.mjv_addGeoms(self.model,self.data,self.option,mujoco.MjvPerturb(),mujoco.mjtCatBit.mjCAT_DYNAMIC,self.renderer.scene)
  self.model.geom_rgba[:]=rgba;self.model.geom_matid[:]=mat
  poses=[r for r in self.trace['poses'] if r['t']<=t][::30]
  for a,b in zip(poses,poses[1:]):self.line([*a['pose'][:2],.025],[*b['pose'][:2],.025],[.08,.35,.64,1],.012)
  truth=min(self.trace['poses'],key=lambda r:abs(r['t']-t))
  for o in truth.get('truth',[]):
   self.geom(mujoco.mjtGeom.mjGEOM_BOX,[*o['half'],.45],[*o['p'],.45],[.80,.34,.15,.95])
  hist=[]
  for r in self.trace['poses']:
   if r['t']>t:break
   if r['truth'] and (not hist or np.linalg.norm(np.array(r['truth'][0]['p'])-hist[-1])>.32):hist.append(np.array(r['truth'][0]['p']))
  for j,p in enumerate(hist[:-1]):
   self.geom(mujoco.mjtGeom.mjGEOM_BOX,[.22,.22,.45],[*p,.45],[.90,.45,.18,.08+.025*j])
  for a,b in zip(hist,hist[1:]):self.line([*a,.035],[*b,.035],[.78,.30,.12,1],.009)
  dec=f.get('decision')
  if dec and show_prediction:
   for obj in ([] if clean else (f.get('tracks') or {}).get('objects',[])):
    # Current configuration-space circle, not the full future hard envelope.
    radius=obj['radius']+.43+.10;x,y=obj['p']
    for j in range(0,64,2):
     a=j*2*math.pi/64;b=(j+1)*2*math.pi/64
     self.line([x+radius*math.cos(a),y+radius*math.sin(a),.05],
               [x+radius*math.cos(b),y+radius*math.sin(b),.05],[.58,.17,.15,.85],.007)
   for tr in dec['tracks']:
    for k,alpha in [(14,.10),(9,.16),(4,.23)]:
     _,x,y,hx,hy=tr['boxes'][k]
     self.geom(mujoco.mjtGeom.mjGEOM_BOX,[hx,hy,.004],[x,y,.018+k*.001],[.93,.55,.15,alpha])
     corners=[[x-hx,y-hy,.04],[x+hx,y-hy,.04],[x+hx,y+hy,.04],[x-hx,y+hy,.04]]
     for a,b in zip(corners,corners[1:]+corners[:1]):self.line(a,b,[.83,.43,.10,.65],.005)
  for xy,col in ([([3,0],[.18,.56,.37,1])] if clean else [([0,0],[.15,.35,.65,1]),([3,0],[.18,.56,.37,1])]):
   for j in range(40):
    a=j*2*math.pi/40;b=(j+1)*2*math.pi/40
    self.line([xy[0]+.16*math.cos(a),xy[1]+.16*math.sin(a),.025],[xy[0]+.16*math.cos(b),xy[1]+.16*math.sin(b),.025],col,.009)
  img=Image.fromarray(self.renderer.render())
  if clean:return img,t
  draw=ImageDraw.Draw(img)
  draw.rectangle([0,0,1800,110],fill='white');draw.rectangle([0,1000,1800,1100],fill='white')
  font=lambda size:ImageFont.truetype(self.font,size)
  draw.text((48,15),f'({index}) {title}',font=font(36),fill='#18344a')
  draw.text((48,65),f't = {t:.1f} s     速度 {truth["v"]:.2f} m/s     目标剩余距离 {np.linalg.norm(np.array(f["pose"][:2])-[3,0]):.2f} m',font=font(26),fill='#526777')
  cam=self.renderer.scene.camera[0];pos=np.asarray(cam.pos);forward=np.asarray(cam.forward);up=np.asarray(cam.up);right=np.cross(forward,up)
  def project(world):
   d=np.asarray(world)-pos;z=np.dot(d,forward);focal=550/math.tan(math.radians(self.model.vis.global_.fovy)/2)
   return (900+focal*np.dot(d,right)/z,550-focal*np.dot(d,up)/z)
  for point,label,color in [([0,0,.02],'起点','#275f9b'),([3,0,.02],'目标','#248451')]:
   px,py=project(point);draw.text((px+30,py+24),label,font=font(25),fill=color,stroke_width=3,stroke_fill='white')
  for x,col,txt in [(48,'#275f9b','Slash 与实际运动拖影'),(580,'#b74e23','动态障碍物与运动拖影'),(1140,'#ce8b24','位置 FRS：0.5 / 1 / 1.5 s')]:
   draw.rectangle([x,1023,x+24,1047],fill=col);draw.text((x+35,1014),txt,font=font(25),fill='#263f51')
  draw.text((48,1060),('灰点：预建底图＋实时观测' if self.result.get('prebuilt_static_map') else '灰点：观测地图')+'；绿圈：目标；红虚线：当前安全半径。实体为仿真真值，FRS 为在线估计。',font=font(22),fill='#637681')
  return img,t

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True)
 p.add_argument('--slash-root',type=Path,default=Path('/home/robotx-2/ws_tianji_sharpa/temp/slash_robot_control'))
 p.add_argument('--times',type=float,nargs=3,default=None);args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
 r=Render(args.run,args.slash_root)
 active=[d for d in r.trace['decisions'] if d.get('tracks') and not d.get('stale')]
 middle=min(active,key=lambda d:abs(d['t']-10))['t'] if active else 10.
 times=args.times or [4.,middle,r.trace['poses'][-1]['t']]
 titles=['感知建图与动态目标识别','FRS 引导下的交互避障','恢复通行并到达目标' if r.result['reached'] else '当前闭环终态（未到达）']
 manifest=[]
 for i,(t,title) in enumerate(zip(times,titles),1):
  img,actual=r.image(t,i,title);stem=args.output/f'process_{i}'
  img.save(stem.with_suffix('.png'),dpi=(300,300));img.save(stem.with_suffix('.pdf'),resolution=300)
  manifest.append({'file':stem.name,'t':actual,'title':title})
 (args.output/'manifest.json').write_text(json.dumps({'source':str(args.run.resolve()),'reached':r.result['reached'],'panels':manifest},ensure_ascii=False,indent=2))
 r.renderer.close()
if __name__=='__main__':main()
