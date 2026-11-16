"""Deterministic depth geometry and multi-object tracking. SI units throughout.
Tentative and moving clusters are conservatively tracked outside the static map.
Confirmed stationary surfaces are integrated into static occupancy.
"""
from dataclasses import dataclass, field
import numpy as np
from scipy.spatial import cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.optimize import linear_sum_assignment


@dataclass
class Config:
    stride: int = 4
    voxel: float = .08
    cluster_distance: float = .20
    surface_bridge_distance: float = .50
    min_points: int = 6
    min_depth: float = .3
    max_depth: float = 3.0
    ground_z: float = 0.0
    ground_band: float = .12
    max_height: float = 2.5
    max_cluster_extent: float = 2.0
    motion_on: float = .18
    motion_off: float = .08
    motion_frames: int = 3
    static_frames: int = 8
    novelty_distance: float = .06
    novelty_fraction: float = .15
    assoc_distance: float = .65
    # Trusted motion-coasting duration, then freeze as an unknown local region.
    lost_timeout: float = 1.0
    retire_distance: float = 6.0
    unknown_cell_size: float = .25
    measurement_sigma: float = .06
    accel_sigma: float = .8
    max_points: int = 15000
    geometry_mean_distance: float = .025
    geometry_relative_variance: float = .8
    visibility_extent_tolerance: float = .08


    def __post_init__(self):
        if not all(np.isfinite(v) for v in vars(self).values()):
            raise ValueError('all geometric parameters must be finite')
        if self.stride < 1 or self.min_points < 1 or self.max_points < self.min_points:
            raise ValueError('invalid sampling/cluster budget')
        if not (0 < self.min_depth < self.max_depth and 0 < self.voxel < self.cluster_distance):
            raise ValueError('invalid depth range or voxel/cluster spacing')
        if not (0 <= self.motion_off < self.motion_on and 0 <= self.novelty_fraction <= 1):
            raise ValueError('invalid motion hysteresis')
        if min(self.measurement_sigma,self.accel_sigma,self.assoc_distance,self.lost_timeout,
               self.motion_frames,self.static_frames,self.max_cluster_extent,self.novelty_distance,
               self.geometry_mean_distance,self.geometry_relative_variance,
               self.visibility_extent_tolerance,self.unknown_cell_size) <= 0:
            raise ValueError('tracking scales/counts must be positive')


@dataclass
class Cluster:
    center: np.ndarray
    radius: float
    pixels: np.ndarray
    novelty: float
    bounds: np.ndarray | None = None
    geometry_mean: float | None = None
    geometry_variance: float | None = None


@dataclass
class Track:
    id: int
    x: np.ndarray
    P: np.ndarray
    z: float
    radius: float
    last_seen: float
    state: str = 'tentative'
    moving: int = 0
    stopped: int = 0
    observed: bool = True
    unknown: bool = False
    pixels: np.ndarray = field(default_factory=lambda: np.empty((0, 2), int))
    support_extent: np.ndarray | None = None
    visibility_corrected: bool = False
    observation_history: list = field(default_factory=list)
    grounded_support: bool = False
    unknown_cells: dict | None = None



def rotation(q):
    q = np.asarray(q, float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-6:
        raise ValueError('invalid rotation')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def project(depth, K, T, cfg):
    """Depth in meters, optical frame x right/y down/z forward."""
    v, u = np.mgrid[0:depth.shape[0]:cfg.stride, 0:depth.shape[1]:cfg.stride]
    z = depth[v, u]
    valid = np.isfinite(z) & (z >= cfg.min_depth) & (z <= cfg.max_depth)
    u, v, z = u[valid], v[valid], z[valid]
    optical = np.column_stack(((u-K[0,2])*z/K[0,0], (v-K[1,2])*z/K[1,1], z))
    world = optical @ T[:3,:3].T + T[:3,3]
    return world, np.column_stack((v,u)), float(valid.mean())


def extend_contact_mask(mask, pixels, bottom):
    """Omit the observed image envelope down to projected ground, not free it.

    An oblique object's contact contour has a different last sampled row in
    each column. Starting only at the global last row leaves low surface strips
    between those contours in the static map. The envelope also closes stride
    holes; background inside it remains unobserved, never declared free.
    """
    if not len(pixels):return
    top=max(0,int(pixels[:,0].min()))
    bottom=min(mask.shape[0],int(bottom))
    left=max(0,int(pixels[:,1].min()));right=min(mask.shape[1],int(pixels[:,1].max())+1)
    mask[top:bottom,left:right]=1


class Tracker:
    def __init__(self, cfg=None):
        self.cfg = cfg or Config()
        self.tracks = {}
        self.previous = None
        self.reference_time = None
        self.time = None
        self.next_id = 0

    def clusters(self, points, pixels):
        c = self.cfg
        # Known floor height, NOT a blindly fitted plane through moving surfaces.
        keep = (points[:,2] > c.ground_z+c.ground_band) & (points[:,2] < c.ground_z+c.max_height)
        p, uv = points[keep], pixels[keep]
        if len(p) > c.max_points:
            raise ValueError('point budget exceeded')
        if not len(p):
            return []
        _, unique, inverse = np.unique(np.floor(p/c.voxel).astype(np.int32), axis=0, return_index=True, return_inverse=True)
        v = p[unique]
        tree = cKDTree(v)
        pairs = tree.query_pairs(c.cluster_distance, output_type='ndarray')
        graph = coo_matrix((np.ones(len(pairs)), (pairs[:,0],pairs[:,1])), shape=(len(v),len(v)))
        count, labels = connected_components(graph, directed=False)
        distances = None if self.previous is None or not len(self.previous) else cKDTree(self.previous).query(v)[0]
        novelty = np.ones(len(v), bool) if distances is None else distances > c.novelty_distance
        # Oblique box surfaces can split into sparse disconnected image strips.
        # Merge overlapping projected envelopes before assigning object identities.
        # This uses measured geometry only, not semantic labels or scene truth.
        bounds = {}
        for label in range(count):
            pts = v[labels == label]
            if len(pts) < c.min_points: continue
            lo, hi = pts.min(axis=0), pts.max(axis=0)
            if np.max(hi-lo) <= c.max_cluster_extent:
                bounds[label] = ((lo+hi)/2, np.linalg.norm((hi-lo)[:2])/2+.08, lo[2], hi[2])
        parent = list(range(count))
        def root(i):
            while parent[i] != i:
                parent[i] = parent[parent[i]]; i = parent[i]
            return i
        keys = list(bounds)
        for i, a in enumerate(keys):
            ca, ra, za, ha = bounds[a]
            for b in keys[i+1:]:
                cb, rb, zb, hb = bounds[b]
                if min(ra,rb)<=.12 and min(ha,hb)-max(za,zb) > .08 and np.linalg.norm(ca[:2]-cb[:2]) < ra+rb+c.voxel*1.5:
                    parent[root(b)] = root(a)
        # Sparse grazing-angle strips are adjacent in the depth image even when
        # metric neighbours are far apart. Bridge only thin components and only
        # adjacent sampled pixels; do not merge arbitrary nearby full objects.
        grid = np.full((uv[:,0].max()//c.stride+2, uv[:,1].max()//c.stride+2), -1, int)
        grid[uv[:,0]//c.stride,uv[:,1]//c.stride] = np.arange(len(uv))
        pixel_labels = labels[inverse]
        for dr,dc in ((0,1),(1,0),(1,1)):
            neighbour = grid[uv[:,0]//c.stride+dr,uv[:,1]//c.stride+dc]
            valid = neighbour>=0
            aidx = np.flatnonzero(valid); bidx = neighbour[valid]
            close = np.linalg.norm(p[aidx]-p[bidx],axis=1)<c.surface_bridge_distance
            pairs = np.unique(np.column_stack((pixel_labels[aidx[close]],pixel_labels[bidx[close]])),axis=0)
            for a,b in pairs:
                if a==b or a not in bounds or b not in bounds: continue
                if min(bounds[a][1],bounds[b][1]) <= .12:
                    parent[root(int(b))] = root(int(a))
        labels = np.array([root(int(label)) for label in labels])
        output = []
        for label in np.unique(labels):
            mask = labels == label
            if mask.sum() < c.min_points:
                continue
            pts = v[mask]
            lo, hi = np.min(pts,axis=0), np.max(pts,axis=0)
            # Walls and floor-connected structures stay in the static map.
            if np.max(hi-lo) > c.max_cluster_extent:
                continue
            center = (lo+hi)/2
            radius = max(.10, float(np.linalg.norm((hi-lo)[:2])/2 + .08))
            # FAPP-inspired geometric change: mean historical NN distance and
            # normalized variance. Newly exposed heterogeneous strips alone are
            # weaker motion evidence than a coherently displaced surface.
            mean = None if distances is None else float(distances[mask].mean())
            variance = None if distances is None else float(distances[mask].var()/max(mean*mean,1e-8))
            output.append(Cluster(center, radius, uv[mask[inverse]], float(novelty[mask].mean()),
                                  np.array([lo,hi]), mean, variance))
        return output

    def observation(self, track, cluster, stamp):
        """Anchor a partially visible cluster to its persistent bounding faces.

        No object category, known dimensions or simulator truth is used. History
        bounds visible support, not a claim to recover an unseen physical centre.
        When support shrinks, use the face consistent with the predicted support
        centre instead of differentiating the shrinking AABB midpoint. Retain
        observed support until this track is retired: a face disappearing for a
        long time is not evidence that the object physically shrank. This is a
        rigid-support approximation; rotation/deformation remains ambiguous.
        On one-sided growth, return a reference-centre shift separately from
        physical displacement so revealing a surface cannot manufacture speed.
        """
        z = cluster.center[:2].copy()
        R = np.eye(2)*self.cfg.measurement_sigma**2
        shift = np.zeros(2)
        if cluster.bounds is None or track.support_extent is None:
            return z, R, False, shift
        extent = track.support_extent
        lo, hi = cluster.bounds[:,:2]
        current = hi-lo
        corrected = False
        for axis in range(2):
            growth = current[axis]-extent[axis]
            # A newly revealed face changes the support-centre coordinate itself.
            # Translate the state/history together; only the retained face's
            # residual may update velocity. Association must use this same frame.
            if growth > .01:
                edge_delta = np.array([lo[axis]-(track.x[axis]-extent[axis]/2),
                                       hi[axis]-(track.x[axis]+extent[axis]/2)])
                residual = np.abs(edge_delta)
                if (min(residual) < self.cfg.visibility_extent_tolerance and
                        abs(residual[0]-residual[1]) > growth*.5):
                    shift[axis] = growth*.5 if residual[0] < residual[1] else -growth*.5
                    R[axis,axis] += (growth*.25)**2
                    corrected = True
            loss = extent[axis]-current[axis]
            if loss > self.cfg.visibility_extent_tolerance:
                candidates = np.array([lo[axis]+extent[axis]/2, hi[axis]-extent[axis]/2])
                residual = np.abs(candidates-track.x[axis])
                z[axis] = candidates[np.argmin(residual)]
                # Partial visibility is less informative, never more confident.
                R[axis,axis] += (min(loss,.30)*.25)**2
                if abs(residual[0]-residual[1]) < self.cfg.measurement_sigma:
                    R[axis,axis] += (loss*.5)**2
                corrected = True
        return z, R, corrected, shift

    def remember_extent(self, track, cluster, stamp):
        if cluster.bounds is not None:
            extent = np.diff(cluster.bounds[:,:2],axis=0)[0]
            track.support_extent = extent if track.support_extent is None else np.maximum(track.support_extent,extent)

    def step(self, stamp, clusters):
        c = self.cfg
        if self.time is not None and (stamp <= self.time or stamp-self.time > .5):
            raise ValueError('out-of-order frame or perception gap')
        dt = 0 if self.time is None else stamp-self.time
        self.time = stamp
        # A tentative velocity prior is needed for association/filter updates,
        # but is not evidence of motion. Retain its last measured posterior so
        # an unmatched frame cannot turn that prior into a large moving ghost.
        tentative_support = {identity: (t.x.copy(), t.P.copy())
                             for identity,t in self.tracks.items()
                             if t.state == 'tentative' and not t.unknown}
        for t in self.tracks.values():
            if not t.unknown:
                # Propagate only up to the trusted-loss boundary, even if this
                # frame arrives just after it. Thereafter freeze the local region.
                h = min(dt,max(0.,t.last_seen+c.lost_timeout-(stamp-dt)))
                A=np.eye(4); A[0,2]=A[1,3]=h
                B=np.vstack((np.eye(2)*h*h/2,np.eye(2)*h))
                t.x = A @ t.x
                t.P = A @ t.P @ A.T + B @ B.T * c.accel_sigma**2
                if stamp-t.last_seen > c.lost_timeout+1e-9:
                    t.unknown=True; t.x[2:]=0.
                    t.P[2:,:]=0.; t.P[:,2:]=0.
                    t.observation_history=[]
            t.observed = False; t.pixels = np.empty((0,2),int)
        ids = list(self.tracks)
        cost = np.full((len(ids),len(clusters)),1e6)
        R = np.eye(2)*c.measurement_sigma**2
        for i, identity in enumerate(ids):
            t = self.tracks[identity]
            for j, cl in enumerate(clusters):
                z, observation_R, _, shift = self.observation(t, cl, stamp)
                delta = z-(t.x[:2]+shift)
                mahal = float(delta @ np.linalg.solve(t.P[:2,:2]+observation_R,delta))
                if np.linalg.norm(delta) <= (max(c.assoc_distance,3*np.sqrt(max(t.P[0,0],t.P[1,1]))) if t.unknown else c.assoc_distance) and mahal < 16 and abs(cl.center[2]-t.z) < .6:
                    cost[i,j] = mahal + abs(cl.radius-t.radius)
        assigned = set()
        for i,j in zip(*linear_sum_assignment(cost)):
            if cost[i,j] >= 1e6: continue
            assigned.add(j); cl = clusters[j]; t = self.tracks[ids[i]]
            if t.unknown:
                # Reactivation is a new velocity-estimation window, not a
                # finite difference over the entire missing interval.
                t.unknown=False; t.unknown_cells=None; t.x[2:]=0.; t.P[2:,2:]=np.eye(2)*.4**2
                t.observation_history=[];t.moving=t.stopped=0;t.state='tentative'
            z, R, t.visibility_corrected, shift = self.observation(t, cl, stamp)
            t.x[:2] += shift
            t.observation_history = [(tt, pp+shift) for tt,pp in t.observation_history]
            gain = np.linalg.solve(t.P[:2,:2]+R,t.P[:2,:]).T
            t.x += gain @ (z-t.x[:2])
            A = np.eye(4); A[:,:2] -= gain
            t.P = A@t.P@A.T + gain@R@gain.T
            t.z = cl.center[2]; t.radius = cl.radius
            if cl.bounds is not None:
                # The shifted support centre must still enclose all observed points.
                furthest = np.maximum(np.abs(cl.bounds[0,:2]-t.x[:2]),np.abs(cl.bounds[1,:2]-t.x[:2]))
                t.radius = max(t.radius,float(np.linalg.norm(furthest)+.08))
            self.remember_extent(t, cl, stamp)
            if cl.bounds is not None and cl.bounds[0,2]<=c.ground_z+c.ground_band+.12:
                t.grounded_support=True
            t.observed = True; t.last_seen = stamp; t.pixels = cl.pixels
            speed = np.linalg.norm(t.x[2:])
            coherent = (cl.geometry_mean is None or
                        (cl.geometry_mean > c.geometry_mean_distance and
                         cl.geometry_variance < c.geometry_relative_variance))
            t.observation_history.append((stamp,z.copy()))
            t.observation_history = [(tt,pp) for tt,pp in t.observation_history if stamp-tt <= .6]
            span = stamp-t.observation_history[0][0]
            # Dense depth surfaces overlap even under real tangential motion.
            # Therefore FAPP NN statistics are supporting evidence, not a veto.
            # Corroborate motion using visibility-corrected positions over time.
            displacement_velocity = (z-t.observation_history[0][1])/max(span,1e-6)
            sustained = (span>=.3 and np.linalg.norm(displacement_velocity)>c.motion_on and
                         np.linalg.norm(displacement_velocity-t.x[2:])<.12)
            evidence = (cl.novelty >= c.novelty_fraction and coherent) or sustained
            # Static confirmation is no longer valid once the velocity estimate
            # leaves the stationary band. Do not keep writing a departing object
            # into static occupancy while waiting for full dynamic confirmation.
            if t.state == 'static' and speed > c.motion_off:
                t.state = 'tentative'
            # Established motion survives an ambiguous partial face, while new
            # motion needs geometric evidence in addition to a velocity estimate.
            t.moving = t.moving+1 if speed > c.motion_on and (evidence or t.state == 'dynamic') else 0
            t.stopped = t.stopped+1 if speed < c.motion_off else 0
            if t.moving >= c.motion_frames: t.state = 'dynamic'
            elif t.stopped >= c.static_frames: t.state = 'static'
        for identity,(position,covariance) in tentative_support.items():
            t = self.tracks[identity]
            if not t.observed:
                # Unconfirmed residuals become fixed unknown support immediately.
                # Keep measured positional uncertainty, never coast the .4 m/s
                # initialization prior. Free-space evidence still controls removal.
                t.x = position; t.P = covariance
                t.x[2:] = 0.; t.P[2:,:] = 0.; t.P[:,2:] = 0.
                t.unknown = True; t.observation_history = []
        for j,cl in enumerate(clusters):
            if j in assigned: continue
            identity = self.next_id; self.next_id += 1
            self.tracks[identity] = Track(identity,np.r_[cl.center[:2],0.,0.],np.diag([c.measurement_sigma**2]*2+[.4**2]*2),cl.center[2],cl.radius,stamp,pixels=cl.pixels)
            self.remember_extent(self.tracks[identity], cl, stamp)
            if cl.bounds is not None and cl.bounds[0,2]<=c.ground_z+c.ground_band+.12:
                self.tracks[identity].grounded_support=True
        return list(self.tracks.values())

    def process(self, points, pixels, stamp):
        clusters = self.clusters(points,pixels)
        tracks = self.step(stamp,clusters)
        if self.reference_time is None or stamp-self.reference_time >= .5:
            self.previous = points.copy()
            self.reference_time = stamp
        return tracks

    def carve_unknown(self, track, depth, K, T):
        """Evidence-only carving of possible centres of ground-supported objects.

        Contact with the observed ground band is required. A connected grounded
        object must intersect that band; a centre cell is eliminated only when
        its whole object-radius-expanded contact patch has valid rays beyond it.
        Cells not seen, occluded, or covered by invalid/no-return depth persist.
        The parent posterior remains frozen for association. Each published cell
        has a fixed ID/geometry and vanishes only after this depth certificate.
        """
        c=self.cfg; size=c.unknown_cell_size; half=size/2
        if track.unknown_cells is None:
            sigma=3*np.sqrt(max(track.P[0,0],track.P[1,1]))
            lo=np.floor((track.x[:2]-sigma-half)/size).astype(int)
            hi=np.floor((track.x[:2]+sigma+half)/size).astype(int)
            if np.prod(hi-lo+1)>256:
                track.grounded_support=False  # Fall back to the unchanged whole-region guard.
                return
            track.unknown_cells={}
            for ix in range(lo[0],hi[0]+1):
                for iy in range(lo[1],hi[1]+1):
                    centre=(np.array([ix,iy])+.5)*size
                    if np.linalg.norm(np.maximum(np.abs(centre-track.x[:2])-half,0))<=sigma+1e-9:
                        track.unknown_cells[(ix,iy)]=centre
        # Include the object's measured support radius when proving a possible
        # centre empty: seeing a gap under an overhang must not remove its body.
        for key,centre in list(track.unknown_cells.items()):
            if ground_patch_free(centre,half+track.radius,
                    c.ground_z+c.ground_band+.02,depth,K,T,c.min_depth,c.max_depth):
                del track.unknown_cells[key]

    def retire(self, stamp, depth, K, T):
        """Delete missing tracks only with observed free-space or outside local scope.
        Short occlusions predict; long losses freeze as unknown local regions.
        Unknown regions need actual free-space evidence or leave local scope.
        """
        c = self.cfg; healthy = True
        for identity,t in list(self.tracks.items()):
            if t.observed: continue
            # A confirmed static object is already managed by the occupancy map.
            # Losing a static surface at the image border must not invent a moving ghost.
            if t.state == 'static':
                del self.tracks[identity]; continue
            if np.linalg.norm(t.x[:2]-T[:2,3]) > c.retire_distance:
                del self.tracks[identity]; continue
            if t.unknown and t.grounded_support:
                self.carve_unknown(t,depth,K,T)
                if t.unknown_cells == {}: del self.tracks[identity]
                continue
            sigma = 3*np.sqrt(max(t.P[0,0],t.P[1,1]))
            r = t.radius+sigma
            world = np.array([[t.x[0]+dx,t.x[1]+dy,t.z+dz] for dx in [-r,0,r] for dy in [-r,0,r] for dz in [-.1,.1]])
            optical = (world-T[:3,3]) @ T[:3,:3]
            free = False
            if np.all(optical[:,2] > c.min_depth):
                uv = (optical[:,:2]/optical[:,2,None]) * [K[0,0],K[1,1]] + [K[0,2],K[1,2]]
                u,v = np.rint(uv).astype(int).T
                if np.all((u>=0)&(v>=0)&(u<depth.shape[1])&(v<depth.shape[0])):
                    if t.unknown:
                        # Check every pixel in the enclosing projected region;
                        # sparse support samples can miss a thin occluding object.
                        measured=depth[v.min():v.max()+1,u.min():u.max()+1]
                        free=np.all(np.isfinite(measured)&(measured>optical[:,2].max()+.15)&(measured<=c.max_depth))
                    else:
                        measured = depth[v,u]
                        free = np.all(np.isfinite(measured)&(measured>optical[:,2]+.15)&(measured<=c.max_depth))
            if free: del self.tracks[identity]
            elif not np.all(np.isfinite(t.x)) or not np.all(np.isfinite(t.P)):
                healthy = False
        return healthy


def laser_region_free(center, radius, origin, angles, ranges, max_range=5., margin=.15):
    """Clear a disc only when its entire angular support has valid free rays.
    No-return is free only to max_range; NaN/zero/occlusion are not evidence.
    Angles are monotone unwrapped world-frame beam angles (at most one turn).
    """
    c=np.asarray(center)-np.asarray(origin);d=float(np.linalg.norm(c))
    angles=np.asarray(angles);ranges=np.asarray(ranges)
    if len(angles)<2 or not np.isfinite(radius) or radius<=0 or d<=radius:return False
    bearing=np.arctan2(c[1],c[0]);bearing+=2*np.pi*round((np.mean(angles)-bearing)/(2*np.pi))
    half=np.arcsin(min(1.,radius/d))
    if bearing-half<angles[0] or bearing+half>angles[-1]:return False
    selected=(angles>=bearing-half)&(angles<=bearing+half)
    if selected.sum()<2:return False
    a=angles[selected];raw=ranges[selected];along=c[0]*np.cos(a)+c[1]*np.sin(a)
    exit_distance=along+np.sqrt(np.maximum(0.,radius**2-d*d+along**2))
    valid=(np.isfinite(raw)&(raw>0))|np.isposinf(raw)
    return bool(np.all(valid & (np.minimum(raw,max_range)>exit_distance+margin)))


def ground_patch_free(center, half, height, depth, K, T, min_depth=.3, max_depth=3., margin=.15):
    """Conservative per-pixel contact-plane certificate; invalid is never free.

    Check the entire enclosing image rectangle, comparing each depth against its
    own ray/plane intersection rather than a global farthest-corner depth.
    Full footprint visibility and finite ranges are mandatory.
    """
    if T[2,3]<=height or half<=0:return False
    corners=np.array([[center[0]+dx,center[1]+dy,height]
                      for dx in [-half,half] for dy in [-half,half]])
    optical=(corners-T[:3,3])@T[:3,:3]
    if np.any(optical[:,2]<min_depth) or np.any(optical[:,2]+margin>max_depth):return False
    uv=optical[:,:2]/optical[:,2,None]*[K[0,0],K[1,1]]+[K[0,2],K[1,2]]
    lo=np.floor(uv.min(axis=0)).astype(int);hi=np.ceil(uv.max(axis=0)).astype(int)
    if np.any(lo<0) or hi[0]>=depth.shape[1] or hi[1]>=depth.shape[0]:return False
    vv,uu=np.mgrid[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
    rays=np.stack(((uu-K[0,2])/K[0,0],(vv-K[1,2])/K[1,1],np.ones_like(uu)),axis=-1)@T[:3,:3].T
    if np.any(rays[:,:,2]>=-1e-8):return False
    expected=(height-T[2,3])/rays[:,:,2]
    measured=depth[lo[1]:hi[1]+1,lo[0]:hi[0]+1]
    return bool(np.all(np.isfinite(measured)&(measured>expected+margin)&(measured<=max_depth)))
