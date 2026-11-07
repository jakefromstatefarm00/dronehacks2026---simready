"""pathplan.planner_process (shared by the Sky Beta node)."""
import numpy as np


def test_carrot_is_pulled_in_at_corners():
    """PX4 flies straight at the setpoint: round a corner the 0.5 m carrot's chord must not
    cross the keep-out."""
    from pathplan.planner_process import safe_carrot_path
    from pathplan.world import GridSpec
    spec = GridSpec(half_size=1.5, resolution=.05)
    cost = np.ones((spec.n, spec.n))
    cost[30:, 30:] = np.inf                                    # keep-out: x > 0 and y > 0
    xy = np.array([.2, -.1])
    path = np.array([xy, [-.1, -.1], [-.1, .4]])               # left, then up round the corner at (0, 0)
    chord_target = np.array([-.1, .1])                         # 0.5 m along: its chord crosses x, y > 0
    out = safe_carrot_path(cost, spec, xy, path, .5)
    assert len(out) == 2 and np.allclose(out[0], xy)
    assert not np.allclose(out[1], chord_target) and out[1][1] < 0   # pulled back before the corner
    straight = np.array([xy, [.2, -.8]])                       # nothing in the way: the full 0.5 m
    assert np.allclose(safe_carrot_path(cost, spec, xy, straight, .5)[1], [.2, -.6])
