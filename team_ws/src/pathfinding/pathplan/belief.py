"""The drone's map: the published layout corrected by what the camera has seen.

This is the piece the ROS 2 node will reuse unchanged; only where ``free``,
``hit`` and ``behind`` cells come from differs (``sensor.DepthCamera`` here,
projected OAK-D depth on the drone).
"""
from __future__ import annotations

import numpy as np

from .world import Envelope, GridSpec, clearance, cost_grid


class BeliefMap:
    """Occupancy belief + planning cost.

    occ        believed occupied cells (starts as the prior)
    seen_free  cells the camera has seen to be empty
    suspect    cells just behind an unexpected obstacle's visible face; not blocked
               (that could seal a corridor behind a shifted wall) but they add
               ``suspect_penalty`` per cell-length within the keep-out distance
    confirmed  cells the camera has actually seen occupied
    cost       planning cost grid for the search (>= 1, inf = blocked)

    ``clear_after``: an occupied cell is cleared after this many free observations (a hit
    always marks it occupied at once).  Default 1.  3 was tried against edge-cell flicker
    but in Gazebo it made phantom obstacles from pose/depth misregistration during turns
    stick (run-07: 171 s instead of 80 s; run-08: no path), so it stays opt-in until depth
    frames are matched to the pose at their capture time.
    """

    def __init__(self, prior: np.ndarray, spec: GridSpec, env: Envelope, suspect_penalty: float = 8.0,
                 unconfirmed_penalty: float = 20.0, clear_after: int = 1):
        self.prior = prior
        self.spec, self.env = spec, env
        self.suspect_penalty = suspect_penalty
        self.unconfirmed_penalty = unconfirmed_penalty
        self.clear_after = max(1, int(clear_after))
        self.occ = prior.copy()
        self.seen_free = np.zeros_like(prior)
        self.suspect = np.zeros_like(prior)
        self.confirmed = np.zeros_like(prior)
        self.free_votes = np.zeros(prior.shape, np.int16)
        self.cost = self._make_cost()

    def optimistic_cost(self, trusted=None):
        """Cost if published-map obstacles the camera has not confirmed might not be there.

        Used only when no path exists on ``cost``: a wall that moved leaves its old
        mapped position blocking the way, sometimes hidden behind its new position.
        Unconfirmed published cells (except ``trusted`` ones, e.g. the structure that
        defines the active checkpoint) become passable at ``unconfirmed_penalty`` extra
        per cell-length, so the drone goes to look; anything real is seen on the way
        (the camera looks ahead along the plan) and blocks again.
        """
        doubtful = self.prior & ~self.confirmed
        if trusted is not None:
            doubtful &= ~trusted
        c = self._make_cost(self.occ & ~doubtful)
        doubtful = np.isfinite(c) & ~np.isfinite(self.cost)
        c[doubtful] += self.unconfirmed_penalty
        return c

    def _make_cost(self, occ=None):
        c = cost_grid(self.occ if occ is None else occ, self.spec, self.env)
        if self.suspect_penalty and self.suspect.any():
            near = (clearance(self.suspect, self.spec) < self.env.hard + self.env.soft_margin) & np.isfinite(c)
            c[near] += self.suspect_penalty
        return c

    def integrate(self, free, hit, behind):
        """Apply one observation. Returns the (K, 2) cells whose cost changed, or None."""
        before_occ, before_sus = self.occ.copy(), self.suspect.copy()
        if len(free):
            r, c = free[:, 0], free[:, 1]
            self.suspect[r, c] = False
            was_occ = self.occ[r, c]
            self.free_votes[r[was_occ], c[was_occ]] += 1
            clear = ~was_occ | (self.free_votes[r, c] >= self.clear_after)
            rc, cc = r[clear], c[clear]
            self.occ[rc, cc] = False
            self.seen_free[rc, cc] = True
            self.confirmed[rc, cc] = False
            self.free_votes[rc, cc] = 0
        if len(hit):
            r, c = hit[:, 0], hit[:, 1]
            self.occ[r, c] = True
            self.suspect[r, c] = False
            self.confirmed[r, c] = True
            self.free_votes[r, c] = 0
        if len(behind):
            r, c = behind[:, 0], behind[:, 1]
            ok = ~self.seen_free[r, c] & ~self.occ[r, c]
            self.suspect[r[ok], c[ok]] = True
        if (self.occ == before_occ).all() and (self.suspect == before_sus).all():
            return None
        new = self._make_cost()
        changed = np.argwhere(new != self.cost)
        self.cost = new
        return changed if len(changed) else None
