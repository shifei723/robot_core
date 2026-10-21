import numpy as np
import pytest
from geometric_obstacles.core import Track
from geometric_obstacles.online_background import BackgroundConfig,OnlineBackground,protected_points
from geometric_obstacles.static_prior import StaticPrior


def fixture():
    x,y=np.meshgrid(np.arange(-.15,.16,.05),np.arange(-.15,.16,.05))
    points=np.column_stack((x.ravel(),y.ravel(),np.full(x.size,1.5)))
    K=np.array([[100.,0,40],[0,100.,40],[0,0,1.]])
    pixels=np.rint(points[:,:2]/1.5*100+40).astype(int)[:,::-1]
    track=Track(7,np.zeros(4),np.eye(4)*.001,1.5,.3,0,state='static',pixels=pixels)
    return points,pixels,track,K


def promote(bg):
    points,pixels,track,K=fixture()
    for stamp in np.arange(0,2.31,.1):
        track.last_seen=stamp
        ids=bg.update([track],points,pixels,stamp)
        if ids:return points,pixels,track,K
    raise AssertionError('no promotion')


def test_only_continuously_confirmed_static_is_promoted():
    points,pixels,tr,K=fixture();bg=OnlineBackground()
    for t in np.arange(0,1.9,.1):assert not bg.update([tr],points,pixels,t)
    assert not len(bg.points)
    tr.state='dynamic'
    bg.update([tr],points,pixels,1.95)
    tr.state='static'
    for t in np.arange(2,3.9,.1):assert not bg.update([tr],points,pixels,t)
    assert not len(bg.points)
    assert bg.update([tr],points,pixels,4.1)==[7]
    assert bg.known(points,4.2).all()


def test_missing_observation_resets_static_confirmation():
    points,pixels,tr,K=fixture();bg=OnlineBackground()
    for t in np.arange(0,1.9,.1):bg.update([tr],points,pixels,t)
    tr.observed=False;bg.update([tr],points,pixels,1.9);tr.observed=True
    assert not bg.update([tr],points,pixels,2.1)
    assert not len(bg.points)


@pytest.mark.parametrize('value',[0.,np.nan,np.inf,65535.,1.,4.])
def test_invalid_occluding_or_out_of_range_depth_never_erases(value):
    bg=OnlineBackground();points,_,_,K=promote(bg);n=len(bg.points)
    for t in [3.,3.1,3.2,3.3]:assert bg.observe_free(np.full((80,80),value),K,np.eye(4),t)==0
    assert len(bg.points)==n


def test_free_rays_clear_only_after_consecutive_observations():
    bg=OnlineBackground();points,_,_,K=promote(bg)
    depth=np.full((80,80),2.5)
    assert bg.observe_free(depth,K,np.eye(4),3.)==0
    assert bg.observe_free(depth,K,np.eye(4),3.1)==0
    assert bg.observe_free(depth,K,np.eye(4),3.2)>0
    assert not len(bg.points)


def test_occlusion_resets_free_evidence_and_fixed_map_is_immutable():
    bg=OnlineBackground();points,_,_,K=promote(bg);fixed=StaticPrior(points)
    depth=np.full((80,80),2.5)
    bg.observe_free(depth,K,np.eye(4),3.)
    bg.observe_free(np.ones((80,80)),K,np.eye(4),3.1)
    assert bg.observe_free(depth,K,np.eye(4),3.2)==0
    assert bg.observe_free(depth,K,np.eye(4),3.3)==0
    assert bg.observe_free(depth,K,np.eye(4),3.4)>0
    assert fixed.known(points).all()


def test_partial_vacancy_reopens_remaining_surface_for_motion_detection():
    bg=OnlineBackground();points,_,_,K=promote(bg)
    depth=np.full((80,80),1.5);depth[:,:40]=2.5
    for t in [3.,3.1,3.2]:bg.observe_free(depth,K,np.eye(4),t)
    assert 0<len(bg.points)<len(points)
    assert not bg.known(bg.points,3.3).any()
    assert bg.known(bg.points,5.).all()


def test_dynamic_track_near_wall_survives_prior_subtraction():
    points,_,tr,_=fixture();fixed=StaticPrior(points)
    tr.state='dynamic';tr.last_seen=1.;tr.x[:2]=[0,0]
    assert fixed.known(points).all()
    foreground=points.copy();foreground[:,2]-=.06
    assert fixed.known(foreground).all()  # broad 10 cm tolerance would swallow it
    protected=protected_points(foreground,[tr],1.05)
    assert protected.all()
    assert not fixed.known(foreground,protected).any()
    assert fixed.known(points,protected).all(), 'actual wall must not become a moving cluster'
    tr.state='static'
    assert not protected_points(points,[tr],1.05).any()


def test_camera_motion_and_fov_are_used_for_free_space_evidence():
    bg=OnlineBackground();_,_,_,K=promote(bg)
    T=np.eye(4);T[0,3]=10
    for t in [3.,3.1,3.2]:assert bg.observe_free(np.full((80,80),2.5),K,T,t)==0
    assert len(bg.points)>0
    # Same world points viewed from 0.2 m closer: surface depth 1.3 m.
    T=np.eye(4);T[2,3]=.2
    for t in [3.3,3.4,3.5]:assert bg.observe_free(np.full((80,80),1.3),K,T,t)==0


def test_capacity_does_not_silently_discard_old_obstacles():
    bg=OnlineBackground(BackgroundConfig(max_points=1));points,pixels,tr,K=fixture()
    for t in np.arange(0,3,.1):assert not bg.update([tr],points,pixels,t)
    assert not len(bg.points)


def test_repeated_timestamp_does_not_accumulate_clear_evidence():
    bg=OnlineBackground();_,_,_,K=promote(bg)
    for _ in range(5):assert bg.observe_free(np.full((80,80),2.5),K,np.eye(4),3.)==0
    assert len(bg.points)>0


def test_newly_visible_surface_restores_existing_support_not_a_new_centroid():
    bg=OnlineBackground();points,_,original,K=promote(bg)
    original.support_extent=np.array([.4,.4])
    bg.dormant[7].support_extent=original.support_extent.copy()
    newly_visible=points+np.array([.09,0,0])
    restored=bg.wake_for_changes(newly_visible,3.)
    assert len(restored)==1 and restored[0].id==7
    np.testing.assert_allclose(restored[0].x,original.x)
    np.testing.assert_allclose(restored[0].support_extent,[.4,.4])
    assert restored[0].state=='tentative'
    assert not bg.known(points,3.1).any()
    assert not bg.wake_for_changes(newly_visible,3.1,[7])
    # Active reinspection must not expire before the 2 s stationary confirmation.
    bg.wake_for_changes(points,5.,[7])
    assert not bg.known(points,5.1).any()


def test_background_matching_margin_absorbs_sampling_noise_not_active_motion():
    bg=OnlineBackground();points,_,tr,K=promote(bg)
    # Online matching is a small surface tolerance, not robot-radius inflation.
    noisy_surface=points+np.array([0,0,.04])
    assert bg.known(noisy_surface,3.).all()
    assert not bg.known(points+np.array([0,0,.12]),3.).any()
    tr.state='dynamic';tr.last_seen=3.
    protected=protected_points(noisy_surface,[tr],3.)
    assert not (bg.known(noisy_surface,3.)&~protected).any()
