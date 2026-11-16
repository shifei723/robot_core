import numpy as np
import pytest
from indoor_scenario import actor_state,indoor_actors


@pytest.mark.parametrize("variant",["follow_exit","dual_crossing"])
def test_actor_paths_are_continuous_and_have_bounded_speed_and_acceleration(variant):
    for speed in [.18,.22,.26]:
        for spec in indoor_actors(speed,variant):
            dt=.002;t=np.arange(0,40,dt)
            p,v=map(np.array,zip(*(actor_state(spec,x) for x in t)))
            assert np.max(np.linalg.norm(v,axis=1))<=speed+1e-10
            assert np.max(np.linalg.norm(np.diff(v,axis=0)/dt,axis=1))<=.400001
            np.testing.assert_allclose(np.diff(p,axis=0)/dt,(v[:-1]+v[1:])/2,atol=.001)
            np.testing.assert_allclose(p[-1],spec['waypoints'][-1])


@pytest.mark.parametrize("variant",["follow_exit","dual_crossing"])
def test_tall_actors_never_cross_the_lintel_or_each_other(variant):
    actors=indoor_actors(.22,variant)
    paths=[np.array([actor_state(s,x)[0] for x in np.arange(0,40,.02)]) for s in actors]
    # AABB against beam x=[2.45,5.55], y=[2.2,2.8], half-width=.22.
    for p in paths:
        overlap=(p[:,0]+.22>2.45)&(p[:,0]-.22<5.55)&(p[:,1]+.22>2.2)&(p[:,1]-.22<2.8)
        assert not overlap.any()
    # Independent activation times cannot cause collisions: entire path supports
    # occupy disjoint y bands with more than one object width between them.
    for i in range(3):
        for j in range(i):
            assert np.min(paths[i][:,1])-np.max(paths[j][:,1])>.44
