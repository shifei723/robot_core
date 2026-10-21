import numpy as np
import pytest
from geometric_obstacles.core import Config, Cluster, Tracker, Track, project, rotation


def cluster(x, novelty=1.):
    return Cluster(np.array([x,0.,.5]), .2, np.array([[1,1]]), novelty)


def test_stationary_cluster_does_not_become_dynamic():
    tracker = Tracker()
    for i in range(40): tracker.step(i*.05,[cluster(1.,0.)])
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].state == 'static'
    assert np.linalg.norm(tracker.tracks[0].x[2:]) < 1e-9


def test_moving_track_stable_id_and_hysteresis():
    tracker = Tracker()
    for i in range(30): tracker.step(i*.05,[cluster(1.+i*.025)])
    assert len(tracker.tracks) == 1
    assert tracker.tracks[0].state == 'dynamic'
    assert tracker.tracks[0].x[2] == pytest.approx(.5,abs=.04)
    for i in range(30,70): tracker.step(i*.05,[cluster(1.+29*.025,0.)])
    assert tracker.tracks[0].state == 'static'


def test_crossing_independent_tracks():
    t = Tracker()
    for i in range(25):
        a,b = cluster(i*.02),cluster(2.-i*.02)
        b.center[1] = .6
        t.step(i*.05,[a,b] if i%2 else [b,a])
    assert len(t.tracks) == 2
    assert sorted(np.sign(tr.x[2]) for tr in t.tracks.values()) == [-1,1]


def test_backprojection_compensates_camera_translation():
    cfg = Config(stride=1, ground_z=-10)
    K=np.array([[100.,0,2],[0,100.,2],[0,0,1.]])
    T=np.eye(4); depth=np.ones((5,5))*2
    a,_,_=project(depth,K,T,cfg)
    T[2,3]=.2
    b,_,_=project(depth-.2,K,T,cfg)
    # Center pixel sees the same fixed world point while camera approaches it.
    np.testing.assert_allclose(a[12],b[12])
    assert rotation([0,0,0,1])[0,0] == 1.


def test_cluster_pipeline_world_stationary_and_moving():
    cfg=Config(voxel=.04,cluster_distance=.12,min_points=4,novelty_distance=.02)
    local=np.array([[x,y,z] for x in np.arange(0,.24,.04) for y in [0,.04] for z in np.arange(.2,.6,.04)])
    uv=np.zeros((len(local),2),int)
    stationary=Tracker(cfg); moving=Tracker(cfg)
    for i in range(40):
        stationary.process(local,uv,i*.05)
        moving.process(local+np.array([i*.025,0,0]),uv,i*.05)
    assert stationary.tracks[0].state == 'static'
    assert moving.tracks[0].state == 'dynamic'


def test_time_discontinuity_rejected():
    for later in [1.,.9,1.6]:
        t=Tracker();t.step(1.,[])
        with pytest.raises(ValueError): t.step(later,[])


def test_invalid_depth_is_not_a_measurement():
    d=np.array([[0.,np.nan,np.inf,65535.,2.]])
    p,uv,f=project(d,np.eye(3),np.eye(4),Config(stride=1))
    assert len(p)==1 and uv[0,1]==4 and f==.2


def test_occlusion_does_not_delete_but_visible_free_space_does():
    t=Tracker();t.tracks[0]=Track(0,np.array([0.,0.,0.,0.]),np.eye(4)*1e-4,1.5,.1,0.,observed=False)
    K=np.array([[30.,0,50],[0,30.,50],[0,0,1.]])
    assert not t.retire(1.1,np.ones((100,100)),K,np.eye(4))
    assert 0 in t.tracks
    assert t.retire(1.2,np.full((100,100),2.5),K,np.eye(4))
    assert not t.tracks


def test_sparse_oblique_face_fragments_share_one_target():
    t=Tracker(Config(voxel=.04,min_points=4))
    front=np.array([[1.58,y,z] for y in np.arange(-.6,-.18,.04) for z in np.arange(.2,.7,.04)])
    edge=np.array([[1.92,-.2,z] for z in np.arange(.2,.7,.04)])
    points=np.vstack((front,edge));clusters=t.clusters(points,np.zeros((len(points),2),int))
    assert len(clusters)==1
    assert clusters[0].radius>.3


def test_missing_confirmed_static_surface_stays_map_managed():
    t=Tracker();t.tracks[0]=Track(0,np.zeros(4),np.eye(4),1.,.2,0.,state='static',observed=False)
    assert t.retire(2.,np.zeros((5,5)),np.eye(3),np.eye(4))
    assert not t.tracks


def test_two_full_surfaces_do_not_merge_just_because_envelopes_overlap():
    cfg=Config(voxel=.04,min_points=4)
    a=np.array([[x,y,z] for x in np.arange(1.6,2.,.04) for y in [.9] for z in np.arange(.2,.8,.04)])
    b=np.array([[x,y,z] for x in np.arange(1.8,2.6,.04) for y in [1.55] for z in np.arange(.2,1.2,.04)])
    points=np.vstack((a,b))
    assert len(Tracker(cfg).clusters(points,np.zeros((len(points),2),int)))==2


def test_image_adjacent_grazing_strip_bridge_is_distance_bounded():
    ys=np.arange(-.6,-.18,.04);zs=np.arange(.2,.7,.04)
    front=np.array([[1.58,y,z] for y in ys for z in zs])
    front_uv=np.array([[(j+1)*4,(i+1)*4] for i in range(len(ys)) for j in range(len(zs))])
    edge=np.array([[2.04,-.2,z] for z in zs])
    edge_uv=np.array([[(j+1)*4,(len(ys)+1)*4] for j in range(len(zs))])
    points=np.vstack((front,edge));uv=np.vstack((front_uv,edge_uv))
    assert len(Tracker(Config(voxel=.04,min_points=4)).clusters(points,uv))==1
    assert len(Tracker(Config(voxel=.04,min_points=4,surface_bridge_distance=.3)).clusters(points,uv))==2


def test_prebuilt_map_filters_known_wall_but_keeps_new_obstacle():
    from geometric_obstacles.static_prior import StaticPrior
    y,z=np.meshgrid(np.arange(-1,1,.04),np.arange(.1,1.5,.04))
    wall=np.column_stack((np.full(y.size,2.),y.ravel(),z.ravel()))
    prior=StaticPrior(wall)
    assert prior.known(wall+[.02,0,0]).all()
    assert not prior.known(wall+[-.3,0,0]).any()
    assert len(prior.local([2,0,.5],.3))<len(wall)
    assert len(prior.known(np.empty((0,3))))==0


def test_prebuilt_map_matching_uses_map_coordinates():
    from geometric_obstacles.static_prior import StaticPrior
    from geometric_obstacles.core import rotation
    R=rotation([0,0,np.sin(.3),np.cos(.3)])
    world=np.array([[2.,1.,.5],[3.,-1.,1.]])
    offset=np.array([1.2,-.4,0.])
    prior=StaticPrior(world@R.T+offset)
    assert prior.known(world@R.T+offset).all()
    assert not prior.known(world).any()
