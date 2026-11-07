from math import inf, isclose

import numpy as np
import pytest

from pathplan.dstar_lite import DStarLite
from pathplan.grid_search import astar, line_cost, path_cost, polyline_cost, theta_star


def random_grid(rng, shape=(30, 40), density=.25):
    cost = 1.0 + rng.random(shape) * 2.0
    cost[rng.random(shape) < density] = inf
    return cost


def free_cell(rng, cost):
    free = np.argwhere(np.isfinite(cost))
    return tuple(free[rng.integers(len(free))])


@pytest.mark.parametrize("seed", range(12))
def test_dstar_matches_astar_on_fresh_grid(seed):
    rng = np.random.default_rng(seed)
    cost = random_grid(rng)
    start, goal = free_cell(rng, cost), free_cell(rng, cost)
    _, a_cost, _ = astar(cost, start, goal)
    d = DStarLite(cost, start, goal)
    d_cost = d.compute()
    if a_cost == inf:
        assert d_cost == inf and d.path() == []
    else:
        assert isclose(d_cost, a_cost, rel_tol=1e-9)
        p = d.path()
        assert p[0] == start and p[-1] == goal
        assert isclose(path_cost(cost, p), a_cost, rel_tol=1e-9)


@pytest.mark.parametrize("seed", range(12))
def test_dstar_replan_matches_astar_after_moves_and_changes(seed):
    rng = np.random.default_rng(100 + seed)
    cost = random_grid(rng, density=.15)
    start, goal = free_cell(rng, cost), free_cell(rng, cost)
    d = DStarLite(cost, start, goal)
    d.compute()
    for _ in range(6):
        path = d.path()
        if len(path) > 3:
            d.move_start(path[3])
        # block or unblock a few random cells, never the robot or goal
        cells, vals = [], []
        for _ in range(15):
            cell = tuple(rng.integers(0, s) for s in cost.shape)
            if cell in (d.start, goal):
                continue
            cells.append(cell)
            vals.append(inf if rng.random() < .6 else 1.0 + rng.random())
        d.update_costs(cells, vals)
        got = d.compute()
        _, want, _ = astar(d.cost, d.start, goal)
        if want == inf:
            assert got == inf
        else:
            assert isclose(got, want, rel_tol=1e-9)
            assert isclose(path_cost(d.cost, d.path()), want, rel_tol=1e-9)


@pytest.mark.parametrize("seed", range(40))
def test_dstar_stress_update_then_move_exact(seed):
    """Mission order (sense -> update_costs -> move_start -> compute) with fractional
    soft costs: D* Lite must equal A* exactly and always return a readable path.
    Regression for the float-key tie that stopped the search early (clutter-03)."""
    rng = np.random.default_rng(1000 + seed)
    shape = (45, 45)
    cost = 1.0 + 4.0 * rng.random(shape) ** 2
    cost[rng.random(shape) < .12] = inf
    start, goal = free_cell(rng, cost), free_cell(rng, cost)
    d = DStarLite(cost, start, goal)
    d.compute()
    for _ in range(25):
        cells, vals = [], []
        centre = np.array(free_cell(rng, d.cost))
        for _ in range(12):            # a clustered change, like a newly seen obstacle
            cell = tuple(np.clip(centre + rng.integers(-2, 3, 2), 0, 44))
            if cell in (d.start, goal):
                continue
            cells.append(cell)
            vals.append(inf if rng.random() < .7 else 1.0 + 4.0 * rng.random() ** 2)
        d.update_costs(cells, vals)
        path = d.path()
        if len(path) > 2:
            d.move_start(path[2])
        got = d.compute()
        _, want, _ = astar(d.cost, d.start, goal)
        assert got == want
        if want != inf:
            p = d.path()
            assert p and p[0] == d.start and p[-1] == goal
            assert path_cost(d.cost, p) == want
        else:
            break


def test_no_corner_cutting():
    cost = np.ones((3, 3))
    cost[0, 1] = inf          # wall end to the right of (0,0)
    path, c, _ = astar(cost, (0, 0), (1, 1))
    assert path == [(0, 0), (1, 0), (1, 1)]
    assert isclose(c, 2.0)


@pytest.mark.parametrize("seed", range(20))
def test_theta_star_no_worse_than_astar_and_lines_are_free(seed):
    """Theta* may take straight lines A* cannot, so its cost must not exceed A*'s
    (up to line-sampling error), and every segment it returns must be unblocked."""
    rng = np.random.default_rng(500 + seed)
    cost = 1.0 + 3.0 * rng.random((40, 40)) ** 2
    cost[rng.random((40, 40)) < .18] = inf
    start, goal = free_cell(rng, cost), free_cell(rng, cost)
    a_path, a_cost, _ = astar(cost, start, goal)
    t_path, t_cost, _ = theta_star(cost, start, goal)
    if a_cost == inf:
        assert t_cost == inf and t_path == []
        return
    assert t_path[0] == start and t_path[-1] == goal
    assert len(t_path) <= len(a_path)
    assert t_cost <= a_cost * 1.02
    assert isclose(polyline_cost(cost, t_path), t_cost, rel_tol=1e-9)
    assert np.isfinite(polyline_cost(cost, t_path))


def test_theta_star_open_room_is_one_straight_line():
    cost = np.ones((30, 30))
    path, c, _ = theta_star(cost, (2, 3), (25, 20))
    assert path == [(2, 3), (25, 20)]
    assert isclose(c, np.hypot(23, 17), rel_tol=1e-9)


def test_line_cost_blocked_by_any_sample():
    cost = np.ones((10, 10))
    cost[5, 5] = inf
    assert line_cost(cost, (5, 0), [(5, 9)])[0] == inf
    assert isclose(line_cost(cost, (4, 0), [(4, 9)])[0], 9.0)


def test_rejects_costs_below_one():
    with pytest.raises(ValueError):
        astar(np.full((2, 2), .5), (0, 0), (1, 1))
