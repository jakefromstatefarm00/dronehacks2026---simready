

def test_carrot_is_not_shrunk_to_a_crawl_on_a_near_miss():
    """Drone inside the keep-out (blocked cell), route leading out and on: full-length carrot."""
    import numpy as np
    from pathplan.planner_process import safe_carrot_path
    from pathplan.world import GridSpec
    spec = GridSpec(half_size=1.5, resolution=.05)
    cost = np.ones((spec.n, spec.n))
    cost[:, :28] = np.inf                                     # keep-out: x < -0.1
    xy = np.array([-.12, 0.0])                                # drone just inside it
    path = np.array([xy, [-.05, 0.05], [-.05, 1.0]])          # out to free space, then on
    out = safe_carrot_path(cost, spec, xy, path, .5)
    assert np.linalg.norm(out[1] - xy) > .4, out              # not the 0.1 m crawl
