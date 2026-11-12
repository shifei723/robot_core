"""Static-only background geometry. Never parses a moving model or its plugins."""
import math
import numpy as np
import xml.etree.ElementTree as E

def static_wall_points(world,step=.08):
 out=[]
 for model in E.parse(world).find('world').findall('model'):
  if model.get('name') in ('ground_plane','goal') or model.find("plugin[@name='dynabarn::Motion']") is not None:continue
  box=model.find('.//collision/geometry/box/size')
  if box is None:continue
  pose=list(map(float,model.findtext('pose','0 0 0 0 0 0').split()));size=np.fromstring(box.text,sep=' ')
  R=np.array([[math.cos(pose[5]),-math.sin(pose[5])],[math.sin(pose[5]),math.cos(pose[5])]])
  u=np.linspace(-size[0]/2,size[0]/2,max(2,int(np.ceil(size[0]/step))+1))
  v=np.linspace(-size[1]/2,size[1]/2,max(2,int(np.ceil(size[1]/step))+1))
  a,b=np.meshgrid(u,v);out.extend(np.c_[a.ravel(),b.ravel()]@R.T+pose[:2])
 return np.asarray(out,dtype=float).reshape(-1,2)

def fit_circle(points):
 """Estimate circular surface support; reject short/ill-conditioned/merged arcs."""
 p=np.asarray(points)[:,:2]
 if len(p)<7:return None
 origin=p.mean(axis=0);q=p-origin;A=np.c_[2*q,np.ones(len(q))]
 if np.linalg.cond(A)>100:return None
 fit=np.linalg.lstsq(A,(q*q).sum(axis=1),rcond=None)[0]
 radius2=fit[2]+fit[:2]@fit[:2]
 if radius2<=0:return None
 radius=float(np.sqrt(radius2));center=origin+fit[:2]
 error=float(np.sqrt(np.mean((np.linalg.norm(p-center,axis=1)-radius)**2)))
 if not .12<radius<1.2 or error>.03:return None
 return center,radius,error
