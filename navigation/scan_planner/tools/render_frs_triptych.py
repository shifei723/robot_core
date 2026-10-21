#!/usr/bin/env python3
"""Minimal horizontal start-interact-pass triptych from a single recorded run."""
import argparse,json,hashlib
from pathlib import Path
from PIL import Image,ImageDraw,ImageFont
from render_pointcloud_interaction import Render

def main():
 p=argparse.ArgumentParser();p.add_argument('run',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=True)
 r=Render(a.run,Path(__file__).resolve().parents[4]/'slash_robot_control')
 r.camera.distance=7.15
 canvas=Image.new('RGB',(4050,1050),'white');d=ImageDraw.Draw(canvas)
 titles=['(a) 导航开始','(b) 动态障碍交互','(c) 通过障碍'];times=[3.3,6.5,30.5];panels=[]
 for i,(t,title) in enumerate(zip(times,titles)):
  img,actual=r.image(t,i+1,title,clean=True,show_prediction=(i==1))
  canvas.paste(img.crop((250,105,1600,1000)),(1350*i,0))
  d.text((1350*i+675,915),title,font=ImageFont.truetype(r.font,44),fill='#243746',anchor='mm')
  row=min(r.trace['poses'],key=lambda x:abs(x['t']-actual))
  panels.append({'stage':title,'time':actual,'pose':row['pose'],'speed':row['v'],'status':row['status'],'FRS_displayed':i==1})
 # A single shared key; entity colours and shapes carry the rest of the explanation.
 for x,col,text in [(850,'#275f9b','机器人轨迹 / 拖影'),(1660,'#b74e23','障碍物 / 拖影'),(2400,'#ce8b24','位置 FRS'),(2980,'#248451','目标')]:
  if text=='目标':d.ellipse((x,986,x+28,1014),outline=col,width=4)
  else:d.rectangle((x,986,x+28,1014),fill=col)
  d.text((x+43,1000),text,font=ImageFont.truetype(r.font,36),fill='#425463',anchor='lm')
 canvas.save(a.output/'navigation_interaction_pass.png',dpi=(450,450))
 canvas.save(a.output/'navigation_interaction_pass.pdf',resolution=450)
 preview=canvas.copy();preview.thumbnail((1800,600));preview.save(a.output/'preview.png')
 (a.output/'manifest.json').write_text(json.dumps({'source':str(a.run.resolve()),'trace_sha256':hashlib.sha256((a.run/'trace.json').read_bytes()).hexdigest(),'panels':panels,'note':'Single recorded software closed loop. Raw position FRS slices 0.5,1,1.5 s shown only in interaction panel. Start prediction intentionally omitted for clarity. Entity truth replay; online estimated FRS. Final panel shows passing obstacle, not arrival.'},ensure_ascii=False,indent=2))
 r.renderer.close()
if __name__=='__main__':main()
