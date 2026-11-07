"""Route search on a 2-D cost grid: Theta* (default) and A* (grid baseline).

Cost model shared by every planner, so their costs compare directly:

* ``cost[r, c]`` >= 1 is the per-cell-length cost of being in a cell; ``inf`` = blocked.
* A grid edge between neighbours costs ``step * (cost[u] + cost[v]) / 2``, step 1 or
  sqrt(2) cells; a diagonal may not cut the corner of a blocked cell.
* A straight line costs the mean cell cost sampled along it times its length
  (``line_cost``); it is blocked if any sample is.

Theta* (Daniel, Nash, Koenig & Felner, JAIR 2010) is A* in which a cell may take
its parent's parent as its own parent when the straight line between them is
cheaper.  Paths come out any-angle (no 45-degree zig-zag), so no separate
shortcut/smoothing step is needed, and because the line cost includes the soft
wall penalty the path still keeps the clearance the search pays for.

A* keeps exact integer edge costs (1/SCALE units) so it agrees bit-for-bit with
D* Lite in ``dstar_lite.py``; see that module.  Public costs are in cell units.
"""
from __future__ import annotations

import heapq
from math import hypot, inf, sqrt

import numpy as np

SQRT2 = sqrt(2.0)
SCALE = 1000
_H_STRAIGHT, _H_DIAG_EXTRA = SCALE, 414            # 414 <= 1000 * (sqrt(2) - 1)
_NEIGHBOURS = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
               (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2)]
_SAMPLES_PER_CELL = 4                              # line sampling; finer never clips a cell corner


def octile(a, b):
    """Integer octile lower bound on the scaled grid cost between two cells.

    Consistent with the scaled edges: a straight step costs >= 1000 and changes h by
    at most 1000; a diagonal step costs >= round(1414.2) = 1414 = 1000 + 414.
    """
    dr, dc = abs(a[0] - b[0]), abs(a[1] - b[1])
    return _H_DIAG_EXTRA * min(dr, dc) + _H_STRAIGHT * max(dr, dc)


class _Graph:
    """8-connected edge model over a cost grid."""

    def __init__(self, cost: np.ndarray):
        if cost.ndim != 2:
            raise ValueError("cost must be a 2-D array")
        finite = cost[np.isfinite(cost)]
        if finite.size and finite.min() < 1.0:
            raise ValueError("finite costs must be >= 1 so the heuristics stay admissible")
        self.cost = cost.astype(float, copy=True)
        self.rows, self.cols = cost.shape

    def inside(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols

    def neighbours(self, u):
        """Yield (v, scaled edge cost) for every neighbour of u, including blocked ones (inf)."""
        r, c = u
        cost = self.cost
        cu = cost[r, c]
        for dr, dc, step in _NEIGHBOURS:
            rr, cc = r + dr, c + dc
            if not self.inside(rr, cc):
                continue
            cv = cost[rr, cc]
            if cu == inf or cv == inf:
                yield (rr, cc), inf
                continue
            if dr and dc and (cost[r + dr, c] == inf or cost[r, c + dc] == inf):
                yield (rr, cc), inf
                continue
            yield (rr, cc), float(round(step * 0.5 * (cu + cv) * SCALE))


# ------------------------------------------------------------------------- A*
def astar(cost: np.ndarray, start, goal):
    """A* over grid edges. Returns (path cells, cost in cell units, expansions)."""
    graph = _Graph(cost)
    start, goal = tuple(start), tuple(goal)
    if graph.cost[start] == inf or graph.cost[goal] == inf:
        return [], inf, 0
    g = {start: 0.0}
    parent = {start: None}
    heap = [(octile(start, goal), 0.0, start)]
    closed = set()
    expansions = 0
    while heap:
        _, gu, u = heapq.heappop(heap)
        if u in closed:
            continue
        closed.add(u)
        expansions += 1
        if u == goal:
            return _unwind(parent, u), gu / SCALE, expansions
        for v, w in graph.neighbours(u):
            if w == inf or v in closed:
                continue
            gv = gu + w
            if gv < g.get(v, inf):
                g[v] = gv
                parent[v] = u
                heapq.heappush(heap, (gv + octile(v, goal), gv, v))
    return [], inf, expansions


def path_cost(cost: np.ndarray, path) -> float:
    """Cost of a cell path that moves between grid neighbours (cell units)."""
    graph = _Graph(cost)
    total = 0.0
    for a, b in zip(path, path[1:]):
        total += next((w for v, w in graph.neighbours(a) if v == b), inf)
    return total / SCALE


# ------------------------------------------------------------------ lines
def line_cost(cost: np.ndarray, a, targets) -> np.ndarray:
    """Cost (cell units) of straight lines from ``a`` to each cell in ``targets`` (K, 2).

    ``a`` is one cell, or (K, 2) origins paired with the targets.

    Each line is sampled at ``_SAMPLES_PER_CELL`` per cell of its own length (so a
    segment costs the same whether computed alone or in a batch); a line is
    blocked (inf) if any sample is.
    """
    a = np.asarray(a, float)
    targets = np.atleast_2d(np.asarray(targets, float))
    d = targets - a
    length = np.hypot(d[:, 0], d[:, 1])
    if not len(length):
        return length
    n = np.ceil(length * _SAMPLES_PER_CELL).astype(int) + 1             # samples per line, >= 1
    n = np.maximum(n, 2)
    k = np.arange(n.max())[:, None]                                    # samples, 1
    valid = k < n[None, :]                                             # samples, K
    t = np.where(valid, k / (n[None, :] - 1), 1.0)
    rc = np.rint(a + t[..., None] * d[None]).astype(int)               # samples, K, 2 (a broadcasts)
    c = np.where(valid, cost[rc[..., 0], rc[..., 1]], 0.0)
    return c.sum(axis=0) / n * length


def polyline_cost(cost: np.ndarray, vertices) -> float:
    """Cost of a polyline through cell vertices, segment by segment (cell units)."""
    total = 0.0
    for a, b in zip(vertices, vertices[1:]):
        total += float(line_cost(cost, a, [b])[0])
    return total


# ----------------------------------------------------------------- Theta*
def theta_star(cost: np.ndarray, start, goal):
    """Theta* with weighted line costs. Returns (vertex cells, cost in cell units, expansions).

    The path is the list of turning points (any-angle); consecutive vertices are
    joined by straight lines.  Heuristic: Euclidean distance, admissible because
    every finite cost is >= 1.
    """
    graph = _Graph(cost)
    cost = graph.cost
    start, goal = tuple(start), tuple(goal)
    if cost[start] == inf or cost[goal] == inf:
        return [], inf, 0

    def h(u):
        return hypot(u[0] - goal[0], u[1] - goal[1])

    g = {start: 0.0}
    parent = {start: start}
    heap = [(h(start), 0.0, start)]
    closed = set()
    expansions = 0
    while heap:
        _, gu, u = heapq.heappop(heap)
        if u in closed or gu > g[u]:
            continue
        closed.add(u)
        expansions += 1
        if u == goal:
            return _unwind(parent, u), gu, expansions
        # grid neighbours that are legal moves (the edge rule also forbids corner cutting)
        cand = [v for v, w in graph.neighbours(u) if w != inf and v not in closed]
        if not cand:
            continue
        # every step is costed as a straight line, so g always equals polyline_cost(path).
        # path 1: u -> v;  path 2: straight from u's parent p -> v.  One batched call.
        p = parent[u]
        k = len(cand)
        lines = line_cost(cost, np.array([u] * k + [p] * k), cand + cand)
        grid_g, line_g = gu + lines[:k], g[p] + lines[k:]
        for v, g1, g2 in zip(cand, grid_g, line_g):
            if g2 <= g1:
                gv, pv = float(g2), p
            else:
                gv, pv = float(g1), u
            if gv < g.get(v, inf):
                g[v] = gv
                parent[v] = pv
                heapq.heappush(heap, (gv + h(v), gv, v))
    return [], inf, expansions


def _unwind(parent, u):
    """Follow parents back to the root (whose parent is None or itself)."""
    path = [u]
    while parent[u] is not None and parent[u] != u:
        u = parent[u]
        path.append(u)
    return path[::-1]
