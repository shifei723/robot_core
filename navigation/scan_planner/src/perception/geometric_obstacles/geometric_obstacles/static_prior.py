"""Immutable prebuilt obstacle map, indexed in its own TF frame."""
import numpy as np
from scipy.spatial import cKDTree

class StaticPrior:
    def __init__(self, points, voxel=.05, match_distance=.10):
        p=np.asarray(points,dtype=float).reshape(-1,3)
        if voxel<=0 or match_distance<=0 or not np.isfinite([voxel,match_distance]).all():
            raise ValueError('invalid static prior resolution/tolerance')
        p=p[np.isfinite(p).all(axis=1)]
        if not len(p):raise ValueError('static prior contains no finite points')
        _,ids=np.unique(np.floor(p/voxel).astype(np.int64),axis=0,return_index=True)
        self.points=p[ids];self.tree=cKDTree(self.points);self.distance=match_distance;self.voxel=voxel

    def known(self, points, protected=None):
        p=np.asarray(points).reshape(-1,3)
        if not len(p):return np.zeros(0,bool)
        distance,_=self.tree.query(p,distance_upper_bound=self.distance)
        known=distance<=self.distance
        if protected is not None:
            # Track protection reopens uncertain near-map matches, not points
            # geometrically coincident with the wall itself. Otherwise a broad
            # track ROI creates fresh moving wall clusters at every hand-off.
            confident=distance<=min(self.distance*.5,self.voxel*.75)
            guard=np.asarray(protected,bool)
            ambiguous=np.flatnonzero(known & guard & ~confident)
            # Sparse/rotated voxel samples on the SAME wall can be farther than
            # the point-distance confidence radius. Require a local planar
            # neighbourhood and a small normal residual, not a larger ball that
            # would swallow foreground objects standing near the wall.
            if len(ambiguous) and len(self.points)>=6:
                d,idx=self.tree.query(p[ambiguous],k=min(8,len(self.points)))
                neighbours=self.points[idx]
                center=neighbours.mean(axis=1)
                delta=neighbours-center[:,None,:]
                covariance=np.einsum('nki,nkj->nij',delta,delta)/len(idx[0])
                eigenvalues,eigenvectors=np.linalg.eigh(covariance)
                planar=(eigenvalues[:,0]<=.03*np.maximum(eigenvalues.sum(axis=1),1e-12)) & (eigenvalues[:,1]>self.voxel**2*.02)
                local=d[:,-1]<=max(3*self.voxel,self.distance)
                normal=eigenvectors[:,:,0]
                residual=np.abs(np.einsum('ni,ni->n',p[ambiguous]-center,normal))
                confident[ambiguous] |= planar & local & (residual<=min(.02,self.distance*.25))
            known &= ~guard | confident
        return known

    def local(self, center, radius):
        return self.points[self.tree.query_ball_point(center,radius)]
