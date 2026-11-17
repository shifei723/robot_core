"""ROS-independent adapter of the supplied upstream MPC_ellip numerical method.
The upstream source is retained verbatim; an AST patch changes only matched caps
and an explicit solver CPU budget. No CBF, gate, dynamics or objective is rewritten.
"""
import ast,hashlib,threading,types
from pathlib import Path
import numpy as np
import casadi as ca

class _Parameters(ast.NodeTransformer):
    def visit_Assign(self,node):
        self.generic_visit(node)
        if len(node.targets)==1 and isinstance(node.targets[0],ast.Name):
            key={'v_max':'speed_cap','omega_max':'yaw_cap'}.get(node.targets[0].id)
            if key:node.value=ast.Attribute(value=ast.Name(id='self',ctx=ast.Load()),attr=key,ctx=ast.Load())
            if node.targets[0].id=='opts_setting':
                node.value.keys.append(ast.Constant('ipopt.max_cpu_time'))
                node.value.values.append(ast.Attribute(value=ast.Name(id='self',ctx=ast.Load()),attr='cpu_budget',ctx=ast.Load()))
            if node.targets[0].id=='opti':
                record=ast.Assign(targets=[ast.Attribute(value=ast.Name(id='self',ctx=ast.Load()),attr='_opti_ref',ctx=ast.Store())],value=ast.Name(id='opti',ctx=ast.Load()))
                return [node,record]
        return node

def load_upstream(source,logger):
    tree=ast.parse(Path(source).read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='Local_Planner')
    functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='distance_global']
    methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ('choose_goal_state','MPC_ellip')]
    assert len(methods)==2
    obj=ast.ClassDef(name='NumericalPlanner',bases=[],keywords=[],body=methods,decorator_list=[])
    module=_Parameters().visit(ast.Module(body=functions+[obj],type_ignores=[]));ast.fix_missing_locations(module)
    ns={'np':np,'ca':ca,'rospy':types.SimpleNamespace(logerr=logger)}
    exec(compile(module,str(source),'exec'),ns)
    return ns['NumericalPlanner']

class UpstreamMPC:
    def __init__(self,source=None,speed_cap=.5,yaw_cap=1.,cpu_budget=.25):
        if source is None:
            root=Path(__file__).resolve().parent
            source=root/'mpc_d_cbf_upstream.py'
            if not source.exists():source=root/'upstream/local_planner.py'
        self.source=Path(source);self.source_sha256=hashlib.sha256(self.source.read_bytes()).hexdigest()
        self.errors=[];self.planner=load_upstream(self.source,self.errors.append)()
        p=self.planner;p.N=25;p.goal_state=np.zeros((25,3));p.curr_state=None;p.global_path=None
        p.speed_cap=float(speed_cap);p.yaw_cap=float(yaw_cap);p.cpu_budget=float(cpu_budget)
        if not(0.1<=p.speed_cap<=1.2 and 0<p.yaw_cap<=1.2 and p.cpu_budget>0):raise ValueError('Invalid MPC caps/budget')
        p.curr_pose_lock=threading.Lock();p.global_path_lock=threading.Lock();p.obstacle_lock=threading.Lock()
        p.last_input=np.zeros((25,2));p.last_state=np.zeros((26,3));p.mpc_success=False;p.ob=[]
        tree=ast.parse(self.source.read_text());gate=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='exceed_ob')
        scope={'np':np,'self':p};module=ast.Module(body=[gate],type_ignores=[]);ast.fix_missing_locations(module)
        exec(compile(module,str(self.source),'exec'),scope);self.gate=scope['exceed_ob']
    def solve(self,state,path,ellipses):
        p=self.planner;state=np.asarray(state,float);path=np.asarray(path,float);ellipses=np.asarray(ellipses,float).reshape(-1,25,5)
        if state.shape!=(3,) or path.ndim!=2 or path.shape[1]!=3 or len(path)<2 or not np.isfinite(state).all() or not np.isfinite(path).all() or not np.isfinite(ellipses).all() or np.any(ellipses[:,:,2:4]<=0):
            raise ValueError('Invalid state/path/predicted ellipses')
        p.curr_state=state.copy();p.global_path=path.copy();p.ob=ellipses.reshape(-1,5).copy();self.errors.clear()
        p.choose_goal_state()
        # Verbatim upstream reference heading rule; native benchmark goal is +x.
        for i in range(p.N-1):
            dx=p.goal_state[i+1,0]-p.goal_state[i,0];dy=p.goal_state[i+1,1]-p.goal_state[i,1]
            p.goal_state[i,2]=np.arctan2(dy,dx) if dx!=0 and dy!=0 else (p.goal_state[i-1,2] if i else 0.)
        p.goal_state[-1,2]=p.goal_state[-2,2]
        active_mask=[not bool(self.gate(p.ob[j*25])) for j in range(len(ellipses))]
        try:states,controls=p.MPC_ellip()
        except Exception as e:
            # Upstream's uninitialised first-failure fallback must not crash ROS.
            self.errors.append(type(e).__name__+': '+str(e));return dict(success=False,controls=np.zeros((25,2)),states=np.tile(state,(26,1)),errors=list(self.errors),solver_status='adapter_exception')
        finally:
            for lock in [p.curr_pose_lock,p.global_path_lock,p.obstacle_lock]:
                if lock.locked():lock.release()
        try:stats=p._opti_ref.stats();solver_status=str(stats.get('return_status','unknown'))
        except Exception:solver_status='no_solver_stats'
        good=bool(p.mpc_success and not self.errors and np.isfinite(states).all() and np.isfinite(controls).all())
        return dict(success=good,controls=np.asarray(controls) if good else np.zeros((25,2)),states=np.asarray(states),errors=list(self.errors),solver_status=solver_status,active_mask=active_mask,solver_constraints=int(p._opti_ref.g.numel()))

def predict_ellipses(samples,age):
    """Common KF snapshot -> upstream's 25 future ellipses at .1,...,2.5 s.
    Circular lidar fits are retained as circles, with 0.95-sigma position expansion.
    Long-lost UNKNOWN regions remain fixed instead of inventing moving tracks.
    """
    ellipses=[]
    for o in samples:
        xy=np.asarray(o['p'],float);vel=np.asarray(o['v'],float);P=np.asarray(o['covariance'],float).reshape(4,4);r=float(o['radius'])
        if not np.isfinite(xy).all() or not np.isfinite(vel).all() or not np.isfinite(P).all() or r<=0:raise ValueError('Invalid track')
        out=[]
        for k in range(25):
            t=max(0.,age)+(k+1)*.1
            if o['unknown']:
                centre=xy;variance=max(0.,float(np.max(np.linalg.eigvalsh(P[:2,:2]))))
            else:
                A=np.array([[1,0,t,0],[0,1,0,t]])
                centre=xy+t*vel;cov=A@P@A.T;variance=max(0.,float(np.max(np.linalg.eigvalsh(cov))))
            axis=max(.05,r+.95*np.sqrt(variance));out.append([*centre,axis,axis,0.])
        ellipses.append(out)
    return np.asarray(ellipses,float).reshape(-1,25,5)

def static_map_ellipses(grid,resolution,origin):
    from scipy.ndimage import label,find_objects,binary_opening
    mask=np.asarray(grid)>=50;covered=np.zeros_like(mask);rectangles=[]
    # Split joined corridor walls by orientation. Enclosing a connected U-shaped
    # boundary with one ellipse would incorrectly mark the whole free room occupied.
    length=max(3,int(np.ceil(2./resolution)))
    for structure in [np.ones((1,length),bool),np.ones((length,1),bool)]:
        part=binary_opening(mask,structure=structure);covered|=part;labs,n=label(part)
        for i,sl in enumerate(find_objects(labs),1):
            if sl is None:continue
            yy,xx=np.where(labs[sl]==i);rectangles.append((xx+sl[1].start,yy+sl[0].start))
    yy,xx=np.where(mask&~covered);tile=max(1,int(np.ceil(.5/resolution)))
    for key in set(zip(xx//tile,yy//tile)):
        selected=(xx//tile==key[0])&(yy//tile==key[1]);rectangles.append((xx[selected],yy[selected]))
    out=[]
    for xx,yy in rectangles:
        lo=np.array([xx.min(),yy.min()])*resolution+origin
        hi=(np.array([xx.max(),yy.max()])+1)*resolution+origin
        center=(lo+hi)/2;half=np.maximum((hi-lo)/2,.05)*np.sqrt(2.)
        if half[0]>=half[1]:a,b,angle=half[0],half[1],0.
        else:a,b,angle=half[1],half[0],np.pi/2
        out.append([*center,a,b,angle])
    return np.asarray(out,float).reshape(-1,5)
