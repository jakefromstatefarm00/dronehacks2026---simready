"""numpy-only distance transform fallback == scipy's, within its cap."""
import numpy as np
import pytest

scipy_ndimage = pytest.importorskip("scipy.ndimage")

from pathplan.world import EDT_CAP_CELLS, Envelope, _edt_capped


@pytest.mark.parametrize("density", [.002, .01, .05, .2, .4])
def test_fallback_matches_scipy_within_cap(density):
    rng = np.random.default_rng(int(density * 1000))
    occ = rng.random((122, 122)) < density
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    exact = scipy_ndimage.distance_transform_edt(~occ)
    got = _edt_capped(~occ)
    near = exact <= EDT_CAP_CELLS
    np.testing.assert_allclose(got[near], exact[near])
    assert np.all(got[~near] > EDT_CAP_CELLS)


def test_cap_covers_everything_the_cost_uses():
    env = Envelope()
    assert (env.hard + env.soft_margin) / .05 < EDT_CAP_CELLS
