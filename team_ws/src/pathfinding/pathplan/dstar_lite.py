"""D* Lite: incremental replanning (Koenig & Likhachev, AAAI 2002, optimised version).

Experimental and off by default. On the 122 x 122 course grid it expanded ~36 %
fewer nodes than A* from scratch but its worst single call was ~9x slower in
Python (pathplan-v1 benchmark), so ``theta`` replanning is the default.

It searches from the goal back to the robot, so when the robot moves and a few
cells change cost only the affected part of the search is repaired.  It uses the
exact integer edge costs of ``grid_search`` (see that module's docstring for why).
"""
from __future__ import annotations

import heapq
from math import inf

import numpy as np

from .grid_search import SCALE, SQRT2, _Graph, octile


class DStarLite(_Graph):
    """Incremental shortest path from a moving start to a fixed goal."""

    def __init__(self, cost: np.ndarray, start, goal):
        super().__init__(cost)
        self.start = tuple(start)
        self.goal = tuple(goal)
        self.last = self.start
        self.km = 0.0
        self.g = np.full(cost.shape, inf)
        self.rhs = np.full(cost.shape, inf)
        self.rhs[self.goal] = 0.0
        self.open = []                       # heap of (k1, k2, r, c)
        self.key_of = {}                     # live keys: cell -> (k1, k2)
        self._push(self.goal, self._key(self.goal))
        self.expansions = 0

    # ---------------------------------------------------------------- queue
    def _key(self, u):
        m = min(self.g[u], self.rhs[u])
        return (m + octile(self.start, u) + self.km, m)

    def _push(self, u, k):
        self.key_of[u] = k
        heapq.heappush(self.open, (k[0], k[1], u[0], u[1]))

    def _top(self):
        while self.open:
            k1, k2, r, c = self.open[0]
            if self.key_of.get((r, c)) == (k1, k2):
                return (k1, k2), (r, c)
            heapq.heappop(self.open)         # stale entry
        return (inf, inf), None

    def _update_vertex(self, u):
        if self.g[u] != self.rhs[u]:
            self._push(u, self._key(u))
        else:
            self.key_of.pop(u, None)

    # ---------------------------------------------------------------- search
    def compute(self, max_expansions: int | None = None):
        """Repair the search until the start is consistent. Returns the start's cost-to-goal."""
        while True:
            k_old, u = self._top()
            start_key = self._key(self.start)
            if u is None or (k_old >= start_key and self.rhs[self.start] == self.g[self.start]):
                break
            if max_expansions is not None and self.expansions >= max_expansions:
                raise RuntimeError("D* Lite expansion budget exceeded")
            self.expansions += 1
            k_new = self._key(u)
            if k_old < k_new:
                self._push(u, k_new)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                self.key_of.pop(u, None)
                heapq.heappop(self.open)
                for s, _ in self.neighbours(u):
                    if s != self.goal:
                        self.rhs[s] = min(self.rhs[s], self._edge(s, u) + self.g[u])
                    self._update_vertex(s)
            else:
                g_old = self.g[u]
                self.g[u] = inf
                for s in [*(v for v, _ in self.neighbours(u)), u]:
                    if self.rhs[s] == self._edge(s, u) + g_old and s != self.goal:
                        self.rhs[s] = self._best_rhs(s)
                    self._update_vertex(s)
        return self.g[self.start] / SCALE

    def _edge(self, a, b):
        """Cost of edge a-b, computed directly (same rule as ``neighbours``)."""
        dr, dc = b[0] - a[0], b[1] - a[1]
        if (dr == 0 and dc == 0) or abs(dr) > 1 or abs(dc) > 1:
            return inf
        cost = self.cost
        cu, cv = cost[a], cost[b]
        if cu == inf or cv == inf:
            return inf
        if dr and dc:
            if cost[a[0] + dr, a[1]] == inf or cost[a[0], a[1] + dc] == inf:
                return inf
            return float(round(SQRT2 * 0.5 * (cu + cv) * SCALE))
        return float(round(0.5 * (cu + cv) * SCALE))

    def _best_rhs(self, s):
        best = inf
        for v, w in self.neighbours(s):
            cand = w + self.g[v]
            if cand < best:
                best = cand
        return best

    # ---------------------------------------------------------------- changes
    def move_start(self, new_start):
        new_start = tuple(new_start)
        self.km += octile(self.last, new_start)
        self.last = new_start
        self.start = new_start

    def update_costs(self, cells, values):
        """Set ``cost[cell] = value`` for each pair and repair rhs of every affected vertex.

        Changing one cell's cost changes all of its edges, so the cell and its
        neighbours (and, for corner cutting, diagonal pairs around it) are updated.
        """
        cells = [tuple(map(int, c)) for c in cells]
        values = list(values)
        touched = set()
        for (r, c), val in zip(cells, values):
            if self.cost[r, c] == val:
                continue
            self.cost[r, c] = val
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    rr, cc = r + dr, c + dc
                    if self.inside(rr, cc):
                        touched.add((rr, cc))
        for s in touched:
            if s != self.goal:
                self.rhs[s] = self._best_rhs(s)
            self._update_vertex(s)
        return len(touched)

    # ---------------------------------------------------------------- output
    def path(self, limit: int | None = None):
        """Greedy descent of g from the start (the optimal path once ``compute`` returned)."""
        if self.g[self.start] == inf:
            return []
        u = self.start
        out = [u]
        limit = limit or self.rows * self.cols
        while u != self.goal and len(out) <= limit:
            best, nxt = inf, None
            for v, w in self.neighbours(u):
                cand = w + self.g[v]
                if cand < best:
                    best, nxt = cand, v
            if nxt is None or best == inf:
                return []
            u = nxt
            out.append(u)
        return out if u == self.goal else []
