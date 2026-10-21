#!/usr/bin/env python3
"""Four trace-backed indoor navigation stages, with shared camera and paper labels."""
import argparse,hashlib,json,math,os
os.environ.setdefault("MUJOCO_GL","egl")
from pathlib import Path
import numpy as np
import mujoco
from PIL import Image,ImageDraw,ImageFont
from reportlab.pdfgen import canvas as pdfcanvas
from render_pointcloud_interaction import Render

class PaperRender(Render):
    def cloud(self,t):
        pts=super().cloud(t)
        # Display-only ceiling cutaway, to expose the robot under the lintel.
        for lo,hi in self.result['static_boxes']:
            if lo[2]>0:
                inside=np.all((pts[:,:2]>=np.array(lo[:2])-.06)&(pts[:,:2]<=np.array(hi[:2])+.06),axis=1)&(pts[:,2]>=lo[2]-.06)
                pts=pts[~inside]
        _,idx=np.unique(np.round(pts/.08).astype(int),axis=0,return_index=True)
        return pts[idx]
    def geom(self,kind,size,pos,color):
        if list(color)==[.35,.45,.53,.35]:
            size=np.asarray(size);pos=np.asarray(pos)
            g=super().geom(kind,size,pos,[.35,.45,.53,.06])
            corners=[pos+size*np.array([x,y,z]) for z in [-1,1] for x,y in [(-1,-1),(1,-1),(1,1),(-1,1)]]
            for a,b in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),(0,4),(1,5),(2,6),(3,7)]:
                self.line(corners[a],corners[b],[.34,.43,.48,.7],.01)
            return g
        return super().geom(kind,size,pos,color)
    def project(self,point):
        cam=self.renderer.scene.camera[0]
        d=np.array(point)-cam.pos;forward=np.array(cam.forward);up=np.array(cam.up)
        focal=550/math.tan(math.radians(self.model.vis.global_.fovy)/2)
        depth=np.dot(d,forward)
        return np.array([900+focal*np.dot(d,np.cross(forward,up))/depth,550-focal*np.dot(d,up)/depth])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    r=PaperRender(a.run,Path(__file__).resolve().parents[4]/'slash_robot_control')
    r.camera.distance=11.3;r.camera.elevation=-58;r.camera.lookat[:]=[2.,1.8,.1]
    font=lambda n:ImageFont.truetype(r.font,n)
    stages=[(12.,'出发与首次交互'),(48.,'转弯后避障'),(80.,'降高过梁与让行'),(r.trace['poses'][-1]['t'],'恢复高度并到达')]
    crop=(220,30,1580,1080);panels=[];records=[]
    blue='#275f9b';orange='#b64f24';grey='#465b68'
    try:
        for i,(time,title) in enumerate(stages):
            img,t=r.image(time,i+1,title,clean=True,show_prediction=i<3)
            row=min(r.trace['poses'],key=lambda x:abs(x['t']-t))
            draw=ImageDraw.Draw(img)
            def label(point,text,offset,color=grey):
                xy=r.project(point);end=xy+offset
                box=draw.textbbox(tuple(end),text,font=font(40),anchor='mm')
                draw.line([tuple(xy),tuple(end)],fill=color,width=2)
                draw.rounded_rectangle((box[0]-10,box[1]-5,box[2]+10,box[3]+7),radius=6,fill='white')
                draw.text(tuple(end),text,font=font(40),fill=color,anchor='mm')
                draw.ellipse((xy[0]-5,xy[1]-5,xy[0]+5,xy[1]+5),fill=color)
            # Fixed task points, repeated compactly for comparison across time.
            label([0,0,.03],'A',[-55,-12],blue)
            label([4,0,.03],'P',[65,20],blue)
            label([4,4.7,.03],'B',[65,-12],'#287554')
            if i==0:label([1.8,-.15,.08],'FRS',[95,90],orange)
            if i==1:label([4.,1.5,.08],'FRS',[-105,-65],orange)
            if i==2:label([5.35,2.5,.76],'净空 0.62 m',[105,-65])
            # A tight panel crop is identical across stages; camera never changes.
            img=img.crop(crop)
            panel=Image.new('RGB',(1360,1210),'white');panel.paste(img,(0,0));d=ImageDraw.Draw(panel)
            d.text((680,1080),f'({chr(97+i)}) {title}',font=font(48),fill='#243746',anchor='mm')
            d.text((680,1145),f't = {t:.1f} s    腿高 {row["base_z"]-.0322:.2f} m',font=font(40),fill=grey,anchor='mm')
            name=f'stage_{i+1}';panel.save(a.output/(name+'.png'),dpi=(300,300));panels.append(panel)
            records.append({'file':name+'.png','time':t,'pose':row['pose'],'leg_height_m':row['base_z']-.0322,'state':row['status'],'speed':row['v'],'title':title,'frs_shown':i<3})
        def assemble(columns,name):
            rows=math.ceil(4/columns);w=1360*columns;h=1210*rows+155
            im=Image.new('RGB',(w,h),'white');d=ImageDraw.Draw(im)
            for i,panel in enumerate(panels):im.paste(panel,((i%columns)*1360,(i//columns)*1210))
            key='A：起点    P：转角路由点    B：终点'
            d.text((w/2,h-112),key,font=font(40),fill=grey,anchor='mm')
            d.text((w/2,h-52),'蓝色：机器人轨迹 / 拖影    橙色：障碍物 / 拖影及位置 FRS    灰色：点云地图与横梁',font=font(36),fill=grey,anchor='mm')
            path=a.output/(name+'.png');im.save(path,dpi=(300,300))
            pdf=a.output/(name+'.pdf');c=pdfcanvas.Canvas(str(pdf),pagesize=(w*72/300,h*72/300));c.setTitle('室内转弯、主动降高与 FRS 避障过程');c.drawImage(str(path),0,0,width=w*72/300,height=h*72/300);c.showPage();c.save()
            preview=im.copy();preview.thumbnail((1800,1600));preview.save(a.output/(name+'_preview.png'))
        assemble(2,'indoor_process_2x2');assemble(4,'indoor_process_horizontal')
        (a.output/'manifest.json').write_text(json.dumps({'source':str(a.run.resolve()),'trace_sha256':hashlib.sha256((a.run/'trace.json').read_bytes()).hexdigest(),'renderer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'panels':records,'display_note':'Single recorded software run. Ground-truth entities replayed; online estimated position FRS at 0.5/1/1.5 s, not hard forbidden areas. Ceiling cloud is visually cut away and beam drawn translucent to reveal posture; no geometry or measurements changed. Configured routing waypoint P. Final frame intentionally omits predictions.'},ensure_ascii=False,indent=2)+'\n')
        (a.output/'caption.txt').write_text('图：室内转弯、主动高度调整与动态障碍物避障过程。机器人由 A 点出发，经转角路由点 P 前往 B 点；(a) 首次动态障碍交互；(b) 转弯后对横穿障碍让行；(c) 保持低姿态通过低横梁，同时依据 FRS 软代价进行动态避障；(d) 通过后恢复腿高并到达目标。蓝色表示机器人轨迹与历史姿态，橙色表示障碍物、运动拖影及在线估计的位置可达集（预测时刻为 0.5、1.0、1.5 s），灰色表示点云地图及横梁。为显示机器人姿态，横梁采用透明轮廓表示。四幅图来自同一次软件闭环仿真；实体为真值回放，FRS 来自在线感知，非实机实验。\n')
    finally:r.renderer.close()
    print(a.output)

if __name__=='__main__':main()
