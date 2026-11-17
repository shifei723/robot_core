import ast,importlib.util
from pathlib import Path
import numpy as np
import pytest
p=Path(__file__).with_name('core.py');spec=importlib.util.spec_from_file_location('mpc_core_test',p);c=importlib.util.module_from_spec(spec);spec.loader.exec_module(c)

def reference():return np.c_[np.arange(0,5.01,.1),np.zeros(51),np.zeros(51)]
def test_upstream_numerical_ast_preserved_except_declared_parameters():
 original=ast.parse((p.parent/'upstream/local_planner.py').read_text());cls=next(z for z in original.body if isinstance(z,ast.ClassDef));f=next(z for z in cls.body if isinstance(z,ast.FunctionDef) and z.name=='MPC_ellip')
 import copy
 transformed=c._Parameters().visit(copy.deepcopy(f))
 # Reverse the documented cap/budget/logging adaptations and require exact AST equality.
 class Reverse(ast.NodeTransformer):
  def visit_Assign(self,n):
   if isinstance(n.targets[0],ast.Attribute) and n.targets[0].attr=='_opti_ref':return None
   if isinstance(n.targets[0],ast.Name):
    if n.targets[0].id=='v_max':n.value=ast.Constant(1.2)
    if n.targets[0].id=='omega_max':n.value=ast.Constant(1.2)
    if n.targets[0].id=='opts_setting':n.value.keys.pop();n.value.values.pop()
   return n
 assert ast.dump(Reverse().visit(transformed),include_attributes=False)==ast.dump(f,include_attributes=False)
def test_free_motion_matched_control_bounds():
 r=c.UpstreamMPC().solve([0,0,0],reference(),np.zeros((0,25,5)))
 assert r['success'] and .1<=r['controls'][0,0]<=.5001
 assert np.max(abs(r['controls'][:,1]))<=1.0001
 assert r['states'].shape==(26,3)
def test_active_ellipse_obeys_original_dcbf_inequality():
 obstacle=np.tile([1.5,.8,.4,.4,0.],(1,25,1));r=c.UpstreamMPC().solve([0,0,0],reference(),obstacle)
 assert r['success'],r['errors']
 assert r['active_mask']==[True] and r['solver_constraints']==152
 x=r['states'];h=np.linalg.norm(x[:,:2]-[1.5,.8],axis=1)-.4-.5
 assert np.min(h[1:25]-.7*h[:24])>=-.005
 assert np.max(abs(x[:,1]))>.01 or np.min(r['controls'][:,0])<.49
def test_unknown_does_not_continue_motion_prediction():
 base=dict(p=[1.,2.],v=[2.,-1.],radius=.5,covariance=(np.eye(4)*.01).ravel(),unknown=True)
 e=c.predict_ellipses([base],.2)[0];assert np.allclose(e[:,:2],[1,2]);assert np.ptp(e[:,2])==0
 base['unknown']=False;e=c.predict_ellipses([base],.2)[0];assert not np.allclose(e[0,:2],e[-1,:2]);assert e[-1,2]>e[0,2]
def test_joined_wall_background_does_not_enclose_free_room():
 grid=np.zeros((210,210));grid[:5,:200]=100;grid[195:200,:200]=100;grid[:200,:5]=100
 e=c.static_map_ellipses(grid,.1,np.zeros(2));assert len(e)==3
 free=np.array([10.,10.])
 for x,y,a,b,theta in e:
  rot=np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]]);q=rot@(free-[x,y]);assert (q[0]/a)**2+(q[1]/b)**2>1
 yy,xx=np.where(grid>=50)
 for point in np.c_[xx+.5,yy+.5]*.1:
  covered=False
  for x,y,a,b,theta in e:
   rot=np.array([[np.cos(theta),np.sin(theta)],[-np.sin(theta),np.cos(theta)]]);q=rot@(point-[x,y]);covered|=(q[0]/a)**2+(q[1]/b)**2<=1+1e-8
  assert covered
def test_invalid_geometry_fails_closed_without_leaking_locks():
 m=c.UpstreamMPC()
 with pytest.raises(ValueError):m.solve([0,0,0],reference(),np.tile([1,0,0,.5,0],(1,25,1)))
 assert not m.planner.curr_pose_lock.locked()
 assert m.solve([0,0,0],reference(),np.zeros((0,25,5)))['success']
