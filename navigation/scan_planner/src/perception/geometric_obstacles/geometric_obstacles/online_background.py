"""Confirmed online static surfaces, separate from the immutable prebuilt map.
All coordinates are world metres; only visibility-qualified rays remove evidence.
"""
from dataclasses import dataclass
import copy
import numpy as np
from scipy.spatial import cKDTree


@dataclass
class BackgroundConfig:
    voxel: float = .05
    match_distance: float = .07
    confirm_seconds: float = 2.
    stable_distance: float = .06
    stable_speed: float = .06
    free_margin: float = .15
    free_frames: int = 3
    audit_seconds: float = 1.5
    max_points: int = 20000

    def __post_init__(self):
        if not all(np.isfinite(v) and v > 0 for v in vars(self).values()):
            raise ValueError('online background parameters must be finite and positive')
        if int(self.free_frames)!=self.free_frames or int(self.max_points)!=self.max_points:
            raise ValueError('online background counts must be integers')


def protected_points(points, tracks, stamp, margin=.12):
    """Retain support around established/pending moving tracks before subtraction.

    Prediction uses exposure time, with a bounded extra 3-sigma position margin.
    A miss cannot shield the whole map indefinitely; the existing watchdog and
    retire policy handle tracks older than one second.
    """
    mask=np.zeros(len(points),bool)
    for track in tracks:
        age=max(0.,stamp-track.last_seen)
        if track.state=='static' or age>1.:continue
        center=track.x[:2]+age*track.x[2:]
        uncertainty=min(.25,3*np.sqrt(max(track.P[0,0],track.P[1,1],0.)))
        radius=track.radius+margin+uncertainty
        mask |= ((np.linalg.norm(points[:,:2]-center,axis=1)<=radius) &
                 (np.abs(points[:,2]-track.z)<=max(.6,track.radius+.25)))
    return mask


class OnlineBackground:
    def __init__(self,cfg=None):
        self.cfg=cfg or BackgroundConfig()
        self.points=np.empty((0,3));self.owners=np.empty(0,int)
        self.free_count=np.empty(0,int);self.audit_until={};self.candidates={}
        self.tree=None;self.last_time=None;self.dormant={};self.reactivated_total=0
        self.promoted_total=0;self.cleared_total=0

    def _index(self):
        self.tree=cKDTree(self.points) if len(self.points) else None

    def known(self,points,stamp):
        if self.tree is None:return np.zeros(len(points),bool)
        d,idx=self.tree.query(points,distance_upper_bound=self.cfg.match_distance)
        known=np.isfinite(d)
        valid=np.flatnonzero(known)
        for owner,until in self.audit_until.items():
            if stamp<=until:known[valid[self.owners[idx[valid]]==owner]]=False
        return known

    def wake_for_changes(self,points,stamp,active_ids=()):
        """Reinspect whole objects, never track just newly exposed strips.

        Restore their prior support/position estimate so ego-view changes do not
        initialize a shifted centroid with a new identity and spurious velocity.
        """
        active=set(active_ids)
        for owner in active:
            if np.any(self.owners==owner):self.audit_until[owner]=stamp+self.cfg.audit_seconds
        novel=points[~self.known(points,stamp)]
        restored=[]
        for owner,track in list(self.dormant.items()):
            if owner in active:continue
            changed=self.audit_until.get(owner,-1)>=stamp
            if len(novel):
                changed |= bool(np.any((np.linalg.norm(novel[:,:2]-track.x[:2],axis=1)<track.radius+.15)&
                                       (np.abs(novel[:,2]-track.z)<max(.4,track.radius+.1))))
            if changed:
                tr=self.dormant.pop(owner)
                tr.state='tentative';tr.observed=False;tr.moving=tr.stopped=0
                tr.observation_history=[]
                # Reinspection is less certain than the last static observation.
                tr.P=tr.P+np.diag([.02**2,.02**2,.10**2,.10**2])
                restored.append(tr);self.audit_until[owner]=stamp+self.cfg.audit_seconds
                self.reactivated_total+=1
        self.dormant={o:t for o,t in self.dormant.items()
                      if np.any(self.owners==o) or self.audit_until.get(o,-1)>=stamp}
        return restored

    def observe_free(self,depth,K,T,stamp,min_depth=.3,max_depth=3.):
        """Clear after consecutive *observations* of farther valid surfaces.

        A 3x3 depth patch must agree. Occlusion, image borders, invalid depth,
        out-of-range returns and time gaps reset the consecutive evidence.
        Any removal also reopens the owner's remaining surface for reinspection.
        """
        if self.last_time is not None and (stamp<=self.last_time or stamp-self.last_time>.5):
            self.free_count[:]=0
        if self.last_time is not None and stamp<=self.last_time:return 0
        self.last_time=stamp
        if not len(self.points):return 0
        optical=(self.points-T[:3,3])@T[:3,:3]
        z=optical[:,2];valid=(z>min_depth)&(z<=max_depth)
        uv=np.zeros((len(z),2),int)
        uv[valid]=np.rint(optical[valid,:2]/z[valid,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]).astype(int)
        u,v=uv.T;h,w=depth.shape
        valid&=(u>=1)&(u<w-1)&(v>=1)&(v<h-1)
        idx=np.flatnonzero(valid);free=np.zeros(len(z),bool)
        if len(idx):
            samples=np.stack([depth[v[idx]+dy,u[idx]+dx] for dy in (-1,0,1) for dx in (-1,0,1)],axis=1)
            free[idx]=np.all(np.isfinite(samples)&(samples>z[idx,None]+self.cfg.free_margin)&(samples<=max_depth),axis=1)
        self.free_count=np.where(free,self.free_count+1,0)
        remove=self.free_count>=self.cfg.free_frames
        for owner in np.unique(self.owners[remove]):self.audit_until[int(owner)]=stamp+self.cfg.audit_seconds
        count=int(remove.sum())
        if count:
            self.points=self.points[~remove];self.owners=self.owners[~remove];self.free_count=self.free_count[~remove]
            self.cleared_total+=count;self._index()
        self.audit_until={o:t for o,t in self.audit_until.items() if t>=stamp}
        return count

    def update(self,tracks,points,pixels,stamp):
        """Promote observed, continuously stable tracks; return IDs to hand off.

        Geometry comes only from their currently observed pixels. Capacity is
        bounded; when full, keep the track rather than erase occupied evidence.
        """
        promoted=[];alive=set()
        keys=pixels[:,0].astype(np.int64)*65536+pixels[:,1]
        for tr in tracks:
            alive.add(tr.id)
            if not tr.observed or tr.state!='static' or np.linalg.norm(tr.x[2:])>self.cfg.stable_speed:
                self.candidates.pop(tr.id,None);continue
            previous=self.candidates.get(tr.id)
            if (previous is None or stamp-previous[2]>.3 or
                np.linalg.norm(tr.x[:2]-previous[1])>self.cfg.stable_distance):
                self.candidates[tr.id]=(stamp,tr.x[:2].copy(),stamp);continue
            start,anchor,_=previous;self.candidates[tr.id]=(start,anchor,stamp)
            if stamp-start<self.cfg.confirm_seconds:continue
            trackkeys=tr.pixels[:,0].astype(np.int64)*65536+tr.pixels[:,1]
            observed=points[np.isin(keys,trackkeys)]
            if not len(observed):continue
            combined=np.vstack((self.points,observed))
            _,unique=np.unique(np.floor(combined/self.cfg.voxel).astype(np.int64),axis=0,return_index=True)
            if len(unique)>self.cfg.max_points:continue
            owners=np.r_[self.owners,np.full(len(observed),tr.id,int)]
            counts=np.r_[self.free_count,np.zeros(len(observed),int)]
            self.points=combined[unique];self.owners=owners[unique];self.free_count=counts[unique]
            self._index();self.promoted_total+=1;promoted.append(tr.id)
            self.dormant[tr.id]=copy.deepcopy(tr)
            self.audit_until.pop(tr.id,None)
            self.candidates.pop(tr.id,None)
        self.candidates={k:v for k,v in self.candidates.items() if k in alive}
        return promoted
